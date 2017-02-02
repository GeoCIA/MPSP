HIGH = 2**16-1
LOW = 0

LEFT0=1,0
LEFT1=1,1
LEFT2=1,2

RIGHT0=0,0
RIGHT1=0,1
RIGHT2=0,2

from mpsp.drivers.pwm_led import Tlc59711

def generate_tail_ground_pattern():
    t = Tlc59711(bus=0)

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

    return (on, off, on, off, off, off)


def generate_tail_flight_pattern():
    t = Tlc59711(bus=0)

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

    return (on, on, on, on, off, off)


def clear():
    t = Tlc59711(bus=0)
    t.clear()
    return t.command()
