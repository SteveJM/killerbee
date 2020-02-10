import re

class RangeParser:
    def __init__(self):
        self.regex = re.compile(r"^(\d+(\-\d+)?,( )?)*(\d+(\-\d+)?)$")

    def parse(self, raw):
        if not self.regex.match(raw):
            raise ValueError("Invalid String")
        nums = []
        for sec in raw.split(','):
            if sec.count('-') == 1:
                spl = sec.split('-')
                start = int(spl[0])
                end = int(spl[1])
                if start >= end:
                    raise ValueError("Range end is lesser than start")
                nums += list(range(start, end+1))
            else:
                nums.append(int(sec))
        return nums