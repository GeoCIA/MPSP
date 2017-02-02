
# adapted from https://github.com/WanderingStar/rpi/blob/master/tlc59711.py


from pyb import SPI
from hashlib import sha256

HIGH = 2**16-1
LOW = 0
BITS = range(0, 224, 8)






class Tlc59711:
    """Communicates with the TLC59711"""
    def __init__(self, bus=1):
        self.outtmg = 1
        self.extgck = 0
        self.tmgrst = 1
        self.dsprpt = 1
        self.blank = 0
        self.brightness = tuple([0b1111111] * 3)  # (R, G, B) brightness
        self.pixels = [(0, 0, 0)] * 4             # (R, G, B) 0-3
        if bus:
            self._spi = SPI(bus, SPI.MASTER, phase=1)

    def clear(self, flush=False):
        for i in range(4):
            for j in range(3):
                self.set_led(i, j, 0, flush=False)
        if flush:
            self.flush()

    def set_led(self, pixel, color, value, flush=True):

        pixel = 3 - pixel
        p = self.pixels[pixel]
        p = list(p)
        p[2-color] = value
        self.pixels[pixel] = tuple(p)
        if flush:
            self.flush()

    def command(self):
        """The bytes of the command that should update the data"""
        #command = BitArray('0b100101') # magic WRITE code
        command = '100101'
        # make sure the values are single bits
        command += ''.join([str(bit) for bit in (
                self.outtmg, self.extgck, self.tmgrst,
                self.dsprpt, self.blank)])
        for b in self.brightness:
            command += '{:07b}'.format(b)
        for rgb in self.pixels:
            for color in rgb:
                command += '{:016b}'.format(color)
        assert len(command) == 224

        h = sha256(command)
        cmd = bytearray(int(command[i:i+8], 2) for i in BITS)
        return cmd, h.digest()

    def flush(self):
        spi = self._spi
        cmd = self.command()
        #print('sending {}'.format(cmd))
        spi.write(cmd)

if __name__ == '__main__':
    tlc = Tlc59711()

    tlc.set_led(0, 0, 2 ** 16 - 109)
    tlc.command()

    #pixels = [min(65535, max(0, int(p))) for p in sys.argv[1:]]
    # if len(pixels) == 1:
    #     tlc.pixels = [tuple(pixels) * 3] * 4
    # elif len(pixels) == 3:
    #     tlc.pixels = [tuple(pixels)] * 4
    # elif len(sys.argv) == 12:
    #     for i in range(4):
    #         tlc.pixels[i] = tuple(pixels[i*3 : i*3+3])
    # else:
    #     print "please specify 1, 3, or 12 pixel values 0-65535"
    #   exit(1)
    #tlc.blank = 1
    #print " ".join(["%02x" % ba for ba in tlc.command()])
    #tlc.sendCommand()
