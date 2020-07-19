'''
WIP

Support for the Raspbee hardware plugged into a Raspberry Pi host where KillerBee runs.
'''

import serial
import struct
import sys
from datetime import datetime
from .kbutils import KBCapabilities, makeFCS
from scapy.layers.dot15d4 import Dot15d4FCS, conf

conf.dot15d4_protocol = 'zigbee'

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


class Raspbee:
    def __init__(self, dev):
        '''
        Instantiates the KillerBee class for our Raspbee interface.
        @type dev:   String
        @param dev:  Serial device identifier (ex /dev/ttyUSB0)
        @return: None
        @rtype: None
        '''
        self._channel = None
        self._page = 0
        self.handle = None
        self.dev = dev

        self.state = STATE_OFF
        # NOTE: handle now called serial for now here:
        self.serial = serial.Serial(self.dev, DEFAULT_BAUD_RATE, timeout=1, inter_byte_timeout=0.5)
        self.serial.write(bytearray(struct.pack("B", CMD_OFF)))
        self.serial.flush()

        self.__stream_open = False
        self.capabilities = KBCapabilities()
        self.__set_capabilities()

    def close(self):
        # TODO
        self.serial.close()
        self.handle = None

    def check_capability(self, capab):
        return self.capabilities.check(capab)
    def get_capabilities(self):
        return self.capabilities.getlist()
    def __set_capabilities(self):
        '''
        Sets the capability information appropriate for GoodFETCCSPI client and firmware.
        @rtype: None
        @return: None
        '''
        self.capabilities.setcapab(KBCapabilities.FREQ_2400, True)
        self.capabilities.setcapab(KBCapabilities.SNIFF, True)
        self.capabilities.setcapab(KBCapabilities.SETCHAN, True)
        self.capabilities.setcapab(KBCapabilities.INJECT, True)
        return

    # KillerBee expects the driver to implement this function
    def get_dev_info(self):
        '''
        Returns device information in a list identifying the device.
        @rtype: List
        @return: List of 3 strings identifying device.
        '''
        return [self.dev, "Raspbee", "TODO"]  # TODO

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

        #if self.state != STATE_RX:
        self.serial.write(bytearray(struct.pack("B", CMD_RX)))
        self.serial.flush()
        self.state = STATE_RX

        if channel != None:
            self.set_channel(channel, page)
        
        #print "Sniffer started (listening as %010x on %i MHz)" % (self.handle.RF_getsmac(), self.handle.RF_getfreq()/10**6);
        self.__stream_open = True

    # KillerBee expects the driver to implement this function
    def sniffer_off(self):
        '''
        Turns the sniffer off, freeing the hardware for other functions.  It is
        not necessary to call this function before closing the interface with
        close().
        @rtype: None
        '''
        self.serial.write(bytearray(struct.pack("B", CMD_OFF)))
        self.serial.flush()
        self.__stream_open = False

    # KillerBee expects the driver to implement this function
    def set_channel(self, channel, page=0):
        '''
        Sets the radio interface to the specifid channel (limited to 2.4 GHz channels 11-26)
        @type channel: Integer
        @param channel: Sets the channel, optional
        @type page: Integer
        @param page: Sets the subghz page, not supported on this device
        @rtype: None
        '''
        self.capabilities.require(KBCapabilities.SETCHAN)
        # Discard any "old" data.
        self.serial.reset_input_buffer()

        if channel >= 11 and channel <= 26:
            self.serial.write(bytearray(struct.pack("BB", CMD_SET_CHANNEL, channel)))
            self.serial.flush()
            self._channel = channel
        else:
            raise Exception('Invalid channel')
        if page:
            raise Exception('SubGHz not supported')

        # This seems to be necesary, but I don't know why?
        self.serial.write(bytearray(struct.pack("B", CMD_RX)))
        self.serial.flush()
        self.state = STATE_RX

    # KillerBee expects the driver to implement this function
    def inject(self, packet, channel=None, count=1, delay=0, page=0):
        '''
        Injects the specified packet contents.
        @type packet: String
        @param packet: Packet contents to transmit, without FCS.
        @type channel: Integer
        @param channel: Sets the channel, optional
        @type page: Integer
        @param page: Sets the subghz page, not supported on this device
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

        #self.handle.RF_autocrc(1)               #let radio add the CRC
        for pnum in range(0, count):
            #if not Dot15d4FCS in packet:
            self.serial.write(struct.pack("BB", CMD_TX, len(packet) +2) + bytes(packet, "latin-1") + makeFCS(bytes(packet, "latin-1")))
            self.serial.flush()
            self.state = STATE_TX

    # KillerBee expects the driver to implement this function
    def pnext(self, timeout=100):
        '''
        Returns a dictionary containing packet data, else None.
        @type timeout: Integer
        @param timeout: Timeout to wait for packet reception in usec
        @rtype: List
        @return: Returns None is timeout expires and no packet received.  When a packet is received, a dictionary is returned with the keys bytes (string of packet bytes), validcrc (boolean if a vaid CRC), rssi (unscaled RSSI), and location (may be set to None). For backwards compatibility, keys for 0,1,2 are provided such that it can be treated as if a list is returned, in the form [ String: packet contents | Bool: Valid CRC | Int: Unscaled RSSI ]
        '''
        # if self.__stream_open == False:
        #     self.sniffer_on()  # start sniffing
        #if self.state != STATE_RX:
        self.serial.write(bytearray(struct.pack("B", CMD_RX)))
        self.serial.flush()
        self.state = STATE_RX

        ret = dict()
        start = datetime.utcnow()

        #while (packet is None and (start + timedelta(microseconds=timeout) > datetime.utcnow())):
        rssi = None

        try:
            length = self.serial.read()
            if len(length) > 0:
                if (len(length) == 1):
                    intLength = ord(length)
                else:
                    intLength = struct.unpack('>i', length)[0]

                if intLength > 127:
                    if intLength == 0xff:
                        next_byte = self.serial.read()
                        if len(next_byte) > 0:
                            if len(next_byte) == 1:
                                next_length = ord(next_byte)
                            else:
                                next_length = struct.unpack('>i', next_byte)[0]
                            message = self.serial.read(next_length)
                            return None
                    elif intLength == 0xf0:
                        rssi = self.serial.read()  # TODO calibrate
                        next_length = self.serial.read()
                        if len(next_length) == 1:
                            next_length_int = ord(next_length)
                        else:
                            next_length_int = struct.unpack('>i', next_length)[0]
                        packet = self.serial.read(next_length_int)
                        if len(packet) != next_length_int:
                            return None

                        if STATE_RX_WMETADATA:
                            result = dict()
                            result["rssi"] = rssi
                            result[0] = packet[:-1].decode('latin-1')
                            result[1] = True   # TODO: Calculate
                            result["frame"] = Dot15d4FCS(packet)
                            return result
                        else:
                            return None
                    return None

                packet = self.serial.read(intLength)
                if len(packet) != intLength or intLength < 3:
                    return None

                if self.state == STATE_RX_WMETADATA:
                    return None
                
                try:
                    pkt = Dot15d4FCS(packet)
                    ret[0] = packet[:-1].decode('latin-1')
                    ret[1] = True  # TODO: Calculate
                    ret['bytes'] = ret[0]
                    return ret
                except Exception as e:
                    return None
            else:
                return None

            if packet is None:
                return None

            frame = packet[1:]
            if frame[-2:] == makeFCS(frame[:-2]): validcrc = True
            else: validcrc = False
            #Return in a nicer dictionary format, so we don't have to reference by number indicies.
            #Note that 0,1,2 indicies inserted twice for backwards compatibility.
            result = {0:frame, 1:validcrc, 2:rssi, 'bytes':frame, 'validcrc':validcrc, 'rssi':rssi, 'location':None}
            if rssi is not None:
                result['dbm'] = rssi - 45 #TODO tune specifically to the platform
            result['datetime'] = datetime.utcnow()
            return result
        except serial.serialutil.SerialException as se:
            # We seem to get occasional serial errors, so just swallow them
            #traceback.print_exc(file=sys.stdout)
            return None
 
    def ping(self, da, panid, sa, channel=None, page=0):
        '''
        Not yet implemented.
        @return: None
        @rtype: None
        '''
        raise Exception('Not yet implemented')

    def jammer_on(self, channel=None, page=0):
        '''
        Not yet implemented.
        @type channel: Integer
        @param channel: Sets the channel, optional
        @type page: Integer
        @param page: Sets the subghz page, not support on this device
        @rtype: None
        '''
        raise Exception('Not yet implemented')

    def jammer_off(self, channel=None, page=0):
        '''
        Not yet implemented.
        @return: None
        @rtype: None
        '''
        raise Exception('Not yet implemented')

