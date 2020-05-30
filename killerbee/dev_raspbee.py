import serial
import struct
#import sys
import time
#from zigdiggity.radios.radio import Radio
#from scapy.layers.dot15d4 import Dot15d4FCS, conf

from datetime import datetime
from .kbutils import KBCapabilities

CMD_OFF = 0x30
CMD_RX = 0x31
CMD_TX = 0x32
CMD_RX_AACK = 0x33
CMD_TX_ARET = 0x34
CMD_SET_CHANNEL = 0x35
CMD_LOAD_FRAME = 0x36
CMD_FIRE_FRAME = 0x37
CMD_FIRE_ARET = 0x38
CMD_RX_WMETADATA = 0x39
DEFAULT_BAUD_RATE = 38400

STATE_OFF = 0x00
STATE_RX = 0x01
STATE_TX = 0x02
STATE_RX_AACK = 0x03
STATE_TX_ARET = 0x04
STATE_ON = 0x05
STATE_RX_WMETADATA = 0x06


RESPONSE_MAP = {RZ_RESP_LOCAL_TIMEOUT: "Local Timeout Error",
                RZ_RESP_SUCCESS : "Success",
                RZ_RESP_SYNTACTICAL_ERROR : "Syntactical Error",
                RZ_RESP_SEMANTICAL_ERROR : "Semantical Error",
                RZ_RESP_HW_TIMEOUT : "Hardware Timeout",
                RZ_RESP_SIGN_ON : "Sign On",
                RZ_RESP_GET_PARAMETER : "Get Parameter",
                RZ_RESP_TRX_READ_REGISTER : "Transceiver Read Register",
                RZ_RESP_TRX_READ_FRAME : "Transceiver Read Frame",
                RZ_RESP_TRX_READ_SRAM : "Transceiver Read SRAM",
                RZ_RESP_TRX_GET_PIN : "Transceiver Get PIN",
                RZ_RESP_TRX_BUSY : "Transceiver Busy",
                RZ_RESP_PRITMITIVE_FAILED : "Primitive Failed",
                RZ_RESP_PRITMITIVE_UNKNOWN : "Unknown Primitive",
                RZ_RESP_COMMAND_UNKNOWN : "Unknown Command",
                RZ_RESP_BUSY_SCANING : "Busy Scanning",
                RZ_RESP_BUSY_CAPTURING : "Busy Capturing",
                RZ_RESP_OUT_OF_MEMORY : "Out of Memory",
                RZ_RESP_BUSY_JAMMING : "Busy Jamming",
                RZ_RESP_NOT_INITIALIZED : "Not Initialized",
                RZ_RESP_NOT_IMPLEMENTED : "Not Implemented by USB firmware",
                RZ_RESP_PRIMITIVE_FAILED : "Primitive Failed",
                RZ_RESP_VRT_KERNEL_ERROR : "Could not execute due to vrt_kernel_error",
                RZ_RESP_BOOT_PARAM : "Boot Param Error"} #: Dictionary of RZUSB error to strings

RZ_USB_VEND_ID                = 0x03EB #: RZUSB USB VID
RZ_USB_PROD_ID                = 0x210A #: RZUSB USB PID
RZ_USB_COMMAND_EP             = 0x02 #: RZUSB USB Command Endpoint Identifier
RZ_USB_RESPONSE_EP            = 0x84 #: RZUSB USB Response Endpoint Identifier
RZ_USB_PACKET_EP              = 0x81 #: RZUSB USB Packet Endpoint Identifier

class Raspbee:
    def __init__(self, dev, bus):
        #TODO deprecate bus param, and dev becomes a usb.core.Device object, not a string in pyUSB 1.x use
        '''
        Instantiates the KillerBee class for the Raspbee hardware.

        @type dev:   TODO
        @param dev:  USB device identifier
        @type bus:   TODO
        @param bus:  Identifies the USB bus the device is on
        @return: None
        @rtype: None
        '''
        self._channel = None
        self._page = 0
        self.handle = None
        self.dev = dev
        self.__bus = bus

        self.state = STATE_OFF
        self.serial = serial.Serial(self.dev, DEFAULT_BAUD_RATE, timeout=1, inter_byte_timeout=0.5)
        self.serial.write(bytearray(struct.pack("B", CMD_OFF)))

        # Tracking the command operating mode (None or AirCapture Mode)
        self.__cmdmode = RZ_CMD_MODE_NONE

        # Tracking if the RZ_CMD_OPEN_STREAM parameter is set for packet reception
        self.__stream_open = False

        # Capabilities list
        self.capabilities = KBCapabilities()
        self.__set_capabilities()

    def close(self):
        '''
        Closes the device handle.  To be re-used, class should be re-instantiated.
        @return: None
        @rtype: None
        '''
        self.serial.write(bytearray(struct.pack("B", CMD_OFF)))
        self.serial.close()

    # KillerBee implements these, maybe it shouldn't and instead leave it to the driver as needed.
    def check_capability(self, capab):
        return self.capabilities.check(capab)
    def get_capabilities(self):
        return self.capabilities.getlist()

    def __set_capabilities(self):
        '''
        Sets the capability information for Raspbee devices.
        @rtype: None
        @return: None
        '''
        self.capabilities.setcapab(KBCapabilities.FREQ_2400, True)
        self.capabilities.setcapab(KBCapabilities.SNIFF, True)
        self.capabilities.setcapab(KBCapabilities.SETCHAN, True)
        self.capabilities.setcapab(KBCapabilities.INJECT, True)
        self.capabilities.setcapab(KBCapabilities.PHYJAM, True)

    def get_dev_info(self):
        '''
        Returns device information in a list identifying the device identifier,
        product string and serial number in a list of strings.
        @rtype: List
        @return: List of 3 strings identifying device.
        '''
        if USBVER == 0:
            return [''.join([self.__bus.dirname, ":", self.dev.filename]), self.dev.open().getString(self.dev.iProduct, 50), self.dev.open().getString(self.dev.iSerialNumber, 12)]
        elif USBVER == 1:
            return ["{0}:{1}".format(self.dev.bus, self.dev.address),         \
                    usb.util.get_string(self.dev, self.dev.iProduct),     \
                    usb.util.get_string(self.dev, self.dev.iSerialNumber) ]

    def __usb_read(self):
        '''
        Read data from the USB device opened as self.handle.

        @rtype: String
        @param data: The data received from the USB endpoint
        '''
        response = None
        print("_process_receive")
        try:
            length = self.serial.read()
            print("receiving {0}".format(len(length)))
            if len(length) > 0:
                # intLength = int.from_bytes(length, "big")
                #length_bytes = bytes(str(length).encode('utf-8'))
                if (len(length) == 1):
                    intLength = ord(length)
                else:
                    intLength = struct.unpack('>i', length)[0]

                if intLength > 127:
                    if intLength == 0xff:
                        next_byte = self.serial.read()
                        if len(next_byte) > 0:
                            if (len(next_byte) == 1):
                                # next_length = int.from_bytes(next_byte, "big")
                                next_length = ord(next_byte)
                            else:
                                # next_length = int.from_bytes(next_byte, "big")
                                next_length = struct.unpack('>i', next_byte)[0]
                            print("BBB Reading: {0} bytes".format(next_length))
                            message = self.serial.read(next_length)
                            print("BBB Got: {0} bytes".format(len(message)))
                            print("DEBUG: " + str(message))
                            return None
                    elif intLength == 0xf0:
                        rssi = self.serial.read()
                        next_length = self.serial.read()
                        # next_length_int = int.from_bytes(next_length, "big")
                        if (len(next_length) == 1):
                            # next_length = int.from_bytes(next_byte, "big")
                            next_length_int = ord(next_length)
                        else:
                            # next_length = int.from_bytes(next_byte, "big")
                            next_length_int = struct.unpack('>i', next_length)[0]
                        if next_length_int < 3:
                            print("too short")
                            return None
                        print("CCC Reading: {0} bytes".format(next_length_int))
                        packet = self.serial.read(next_length_int)
                        print("CCC Got: {0} bytes".format(len(packet)))
                        if len(packet) != next_length_int:
                            # Receive timeout occurred - discard.
                            print("incomplete packet 111")
                            return None

                        if STATE_RX_WMETADATA:
                            print("RSSI = {0}".format(rssi))
                            result = dict()
                            result["rssi"]=rssi
                            result["frame"]=Dot15d4FCS(packet)
                            return result
                        else:
                            print("unexpected state 111")
                            return None
                    print("unexpected length: {0}".format(intLength))
                    return None

                recv_start = time.time()
                print("AAA Reading: {0} bytes".format(intLength))
                packet = self.serial.read(intLength)
                print("AAA Got: {0} bytes".format(len(packet)))
                recv_end = time.time()
                self.add_recv_time(recv_end - recv_start)

                if len(packet) != intLength or intLength < 5:
                    # Receive timeout occurred, or bad data - discard.
                    print("incomplete packet 222")
                    return None

                if self.state==STATE_RX_WMETADATA:
                    print("unexpected state 222")
                    return None

                try:
                    print("Creating Dot154d4 Packet")
                    pkt = Dot15d4FCS(packet)
                    print("...done.")
                    return pkt
                except Exception as e:
                    print("Failed to decode packet: {0}".format(e), file=sys.stderr)
                    return None
            else:
                return None
        except serial.serialutil.SerialException as se:
            traceback.print_exec()
            pass

    def __usb_write(self, endpoint, data, expected_response=RZ_RESP_SUCCESS):
        '''
        Write data to the USB device opened as self.handle.

        @type endpoint: Integer
        @param endpoint: The USB endpoint to write to
        @param expected_response: The desired response - defaults to RZ_RESP_SUCCESS
        @type data: Mixed
        @param data: The data to send to the USB endpoint
        '''
        if USBVER == 0:
            try:
                self.handle.bulkWrite(endpoint, data)
                # Returns a tuple, first value is an int as the RZ_RESP_* code
#                response = self.handle.bulkRead(RZ_USB_RESPONSE_EP, 1)[0]
            except usb.USBError as e:
                if e.args != ('No error',): # http://bugs.debian.org/476796
                    raise e
#            time.sleep(0.0005)
#            if response != RZ_RESP_SUCCESS:
#                if response in RESPONSE_MAP:
#                    raise Exception("Error: %s" % RESPONSE_MAP[response])
#                else:
#                    raise Exception("Unknown USB write error: 0x%02x" % response)
        else: #pyUSB 1.x
            try:
                res = self.dev.write(endpoint, data)#, 0, 100)
                if len(data) != res:
                    raise Exception("Issue writing USB data {0} to endpoint {1}, got a return of {2}.".format(data, endpoint, res))
#                response = self.dev.read(RZ_USB_RESPONSE_EP, self.dev.bMaxPacketSize0, timeout=500)
#                response = response.pop()
            except usb.core.USBError as e:
                if e.errno != 110: #Not Operation timed out
                    print("Error args:", e.args)
                    raise e
                elif e.errno == 110:
                    print("DEBUG: Received operation timed out error ...attempting to continue.")
        #time.sleep(0.0005)
        response = self.__usb_read()
        #print('response 0: %x' % response[0])
        if response[0] != expected_response:
            if response[0] in RESPONSE_MAP:
                raise Exception("Error: %s" % RESPONSE_MAP[response[0]])
            else:
                raise Exception("Unknown USB write error: 0x%02x" % response[0])
        return response[1:]

    def _set_mode(self, mode=RZ_CMD_MODE_AC):
        '''
        Change the operating mode of the USB device to one of the RZ_CMD_MODE_*
        values.  Currently, RZ_CMD_MODE_AC (Air Capture) is the only mode that is
        used other than RZ_CMD_MODE_NONE.
        @type mode: Integer
        @param mode: Operating mode for the USB stick
        @rtype: None
        '''
        self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_SET_MODE, mode])
        self.__cmdmode = mode

    def _open_stream(self):
        '''
        Opens a data stream for receiving packets.
        '''
        self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_OPEN_STREAM])
        self.__stream_open = True

    def _close_stream(self):
        '''
        Closes a data stream.
        '''
        self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_CLOSE_STREAM])
        self.__stream_open = False

    # KillerBee expects the driver to implement this function
    def sniffer_on(self, channel=None, page=0):
        '''
        Turns the sniffer on such that pnext() will start returning observed
        data.  Will set the command mode to Air Capture if it is not already
        set.
        @type channel: Integer
        @param channel: Sets the channel, optional
        @type page: Integer
        @param page: Sets the subghz page, not supported on this device
        @rtype: None
        '''
        self.capabilities.require(KBCapabilities.SNIFF)

        if self.__cmdmode != RZ_CMD_MODE_AC:
            self._set_mode(RZ_CMD_MODE_AC)

        if channel != None:
            self.set_channel(channel, page)

        self._open_stream()

    # KillerBee expects the driver to implement this function
    def sniffer_off(self):
        '''
        Turns the sniffer off, freeing the hardware for other functions.  It is
        not necessary to call this function before closing the interface with
        close().
        @rtype: None
        '''
        self._close_stream()

    def jammer_on(self, channel=None, page=0):
        '''
        @type channel: Integer
        @param channel: Sets the channel, optional
        @type page: Integer
        @param page: Sets the subghz page, not supported on this device
        @rtype: None
        '''
        self.capabilities.require(KBCapabilities.PHYJAM)

        if self.__cmdmode != RZ_CMD_MODE_AC:
            self._set_mode(RZ_CMD_MODE_AC)

        if channel != None:
            self.set_channel(channel, page)

        self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_JAMMER_ON])

    def enter_bootloader(self):
        '''
        @rtype: None
        '''
        self.capabilities.require(KBCapabilities.BOOT)

        self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_ENTER_BOOT])

    def get_bootloader_version(self):
        '''
        @rtype: List
        @return: List: Version number as [Major, Minor]
        '''
        self.capabilities.require(KBCapabilities.BOOT)

        return self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_BOOT_GET_VERSION], RZ_RESP_BOOT_PARAM)

    def get_bootloader_signature(self):
        '''
        @rtype: List
        @return: List: Chip signature as [Low, Mid, High]
        '''
        self.capabilities.require(KBCapabilities.BOOT)

        return self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_BOOT_READ_SIGNATURE], RZ_RESP_BOOT_PARAM)

    def bootloader_sign_on(self):
        '''
        @rtype: String
        @return: Bootloader sign_on message (array of length + length bytes)
        '''
        self.capabilities.require(KBCapabilities.BOOT)

        return self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_SIGN_ON], RZ_RESP_SIGN_ON)

    def bootloader_start_application(self):
        '''
        Instruct bootloader to run main app
        '''
        self.capabilities.require(KBCapabilities.BOOT)

        return self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_BOOT_START_APPLICATION])

    def jammer_off(self, channel=None, page=0):
        '''
        @type channel: Integer
        @param channel: Sets the channel, optional
        @type page: Integer
        @param page: Sets the subghz page, not supported on this device
        @rtype: None
        '''
        self.capabilities.require(KBCapabilities.PHYJAM)

        if self.__cmdmode != RZ_CMD_MODE_AC:
            self._set_mode(RZ_CMD_MODE_AC)

        self.__usb_write(RZ_USB_COMMAND_EP, [RZ_CMD_JAMMER_OFF])

    # KillerBee expects the driver to implement this function
    def set_channel(self, channel, page):
        '''
        Sets the radio interface to the specifid channel.  Currently, support is
        limited to 2.4 GHz channels 11 - 26.
        @type channel: Integer
        @param channel: Sets the channel, optional
        @type page: Integer
        @param page: Sets the subghz channel, not supported on this device
        @rtype: None
        '''
        self.capabilities.require(KBCapabilities.SETCHAN)

        if (11 <= channel <= 26):
            set_channel_packet = bytearray(struct.pack("BB",CMD_SET_CHANNEL,channel))
            self.serial.write(set_channel_packet)
            self.channel = channel
        else:
            raise Exception('Invalid channel')
        if page:
            raise Exception('SubGHz not supported')

    # KillerBee expects the driver to implement this function
    def inject(self, packet, channel=None, count=1, delay=0, page=0):
        '''
        Injects the specified packet contents.
        @type packet: Bytes
        @param packet: Packet contents to transmit, without FCS.
        @type channel: Integer
        @param channel: Sets the channel, optional
        @type page: Integer
        @param page: Sets the subghz channel, not supported on this device
        @type count: Integer
        @param count: Transmits a specified number of frames, def=1
        @type delay: Float
        @param delay: Delay between each frame, def=1
        @rtype: None
        '''
        self.capabilities.require(KBCapabilities.INJECT)

        if len(packet) < 1:
            raise Exception('Empty packet')
        if len(packet) > 125:                   # 127 - 2 to accommodate FCS
            raise Exception('Packet too long')

        if channel != None:
            self.set_channel(channel, page)

        send_start = time.time()
        self.serial.write(struct.pack("BB", CMD_TX, len(packet)) + bytes(packet))
        send_end = time.time()
        self.add_send_time(send_end - send_start)

        self.state = STATE_TX

    # KillerBee expects the driver to implement this function
    def pnext(self, timeout=100):
        '''
        Returns packet data as a string, else None.
        @type timeout: Integer
        @param timeout: Timeout to wait for packet reception in usec
        @rtype: List
        @return: Returns None is timeout expires and no packet received.  When a packet is received, a list is returned, in the form [ String: packet contents | Bool: Valid CRC | Int: Unscaled RSSI ]
        '''
        if self.state != STATE_RX:
            self.serial.write(bytearray(struct.pack("B",CMD_RX)))
            self.state = STATE_RX

        ret = None
        framedata = []
        explen = 0 # expected remaining packet length
        while True:
            # The RZ_USB_PACKET_EP doesn't return error codes like the standard
            # RZ_USB_RESPONSE_EP does, so we don't use __usb_read() here.
            pdata = None
            try:
                pdata = self._process_receive()
            except serial.Error as e:
                if e.errno != 110: #Operation timed out
                    print("Error args: {}".format(e.args))
                    raise e
                    #TODO error handling enhancements for Serial
                else:
                    return None

            # PyUSB returns an empty tuple occasionally, handle as "no data"
            # TODO added len(pdata) check as some arrays were failing
            if pdata == None or pdata == () or len(pdata) == 0 or len(pdata) <= 10:
                return None

            if pdata[0] == RZ_EVENT_STREAM_AC_DATA and ret is None:
                explen = pdata[1]
                rssi = int(pdata[6])
                validcrc = True if (pdata[7] == 1) else False
                frame = pdata[9:]
                # Accumulate the tuple response data in a list
                for byteval in frame:
                    framedata.append(struct.pack("B", byteval))
                #Return in a nicer dictionary format, so we don't have to reference by number indicies.
                #Note that 0,1,2 indicies inserted twice for backwards compatibility.
                ret = {1:validcrc, 2:rssi,
                        'validcrc':validcrc, 'rssi':rssi,
                        'dbm':rssi,'datetime':datetime.utcnow()}
                # TODO: calculate dbm based on RSSI conversion formula for the chip
            else:
                if ret is not None:
                    # This is a continuation of a long AC packet, so append frame data
                    for byteval in pdata:
                        framedata.append(struct.pack("B", byteval))
                else:
                    # raise Exception("Unrecognized AirCapture Data Response: 0x%02x" % pdata[0])
                    return None

            # We will only get to this point if we received valid data.
            # If we've now received the whole frame, return it.
            if explen == len(pdata):
                # The last byte of frame data is the link quality indicator
                ret['lqi'] = framedata[-1]
                # Convert the framedata to a string for the return value
                ret[0] = b''.join(framedata[:-1])
                ret['bytes'] = ret[0]
                return ret
            else:
                # ...otherwise we're expecting a continuation in the next USB read
                explen = explen - len(pdata)

    def ping(self, da, panid, sa, channel=None, page=0):
        '''
        Not yet implemented.
        @return: None
        @rtype: None
        '''
        raise Exception('Not yet implemented')
