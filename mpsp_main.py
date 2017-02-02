from mpsp import FLIGHT, GROUNDTEST
from mpsp.mpsp import MPSP

mode = FLIGHT
# mode = GROUNDTEST
m = MPSP(mode)
m.init()
m.run()
