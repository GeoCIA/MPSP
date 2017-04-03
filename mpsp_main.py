from mpsp import FLIGHT, GROUNDTEST
from mpsp.mpsp import MPSP
import pyb

switch = pyb.Switch()
pyb.LED(3).on()
pyb.LED(1).on()
pyb.delay(4000)
pyb.LED(3).off()
pyb.LED(1).off()

if switch():
    mode = FLIGHT
else:
    mode = GROUNDTEST

pyb.delay(1000)
m = MPSP(mode)
m.init()
m.run()
