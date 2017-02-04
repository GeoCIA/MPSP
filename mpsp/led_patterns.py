HIGH = 2**16-1
LOW = 0

LEFT0=1,0
LEFT1=1,1
LEFT2=1,2

RIGHT0=0,0
RIGHT1=0,1
RIGHT2=0,2

SPOT1=3,0

from mpsp.drivers.pwm_led import Tlc59711

t = Tlc59711(bus=0)
t.set_led(*SPOT1, value=HIGH, flush=False)
t.set_led(*LEFT1, value=HIGH, flush=False)
t.set_led(*RIGHT1, value=HIGH, flush=False)

t.set_led(*LEFT0, value=HIGH, flush=False)
t.set_led(*LEFT2, value=HIGH, flush=False)
t.set_led(*RIGHT0, value=HIGH, flush=False)
t.set_led(*RIGHT2, value=HIGH, flush=False)

on = t.command()

t.set_led(*LEFT0, value=LOW, flush=False)
t.set_led(*LEFT2, value=LOW, flush=False)
t.set_led(*RIGHT0, value=LOW, flush=False)
t.set_led(*RIGHT2, value=LOW, flush=False)
off = t.command()

def generate_tail_ground_pattern():

    return (on, off, on, off, off, off)


def generate_tail_flight_pattern():

    return (on, on, on, on, off, off)


def generate_tail_landing_pattern():
    return (on,off)


def clear():
    t.clear()
    return t.command()


# Patterns =====================================================
TAIL_FLIGHT_PATTERN = generate_tail_flight_pattern()
TAIL_GROUND_PATTERN = generate_tail_ground_pattern()
TAIL_LANDING_PATTERN = generate_tail_landing_pattern()
TAIL_CLEAR = clear()


DOME_GROUND_PATTERN = (1, 0, 1, 0, 0, 0)
DOME_FLIGHT_PATTERN = (0,0,0,0,0,1,1)
STATUS_PATTERN = (0,0,0,0,0,1,1,1,1,1)
