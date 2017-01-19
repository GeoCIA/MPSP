import pyb
from pyb import Pin
from pyb import ExtInt

# We need to use global properties here as any allocation of a memory (aka declaration of a variable)
# during the read cycle causes non-acceptable delay and we are loosing data than
data = None
timer = None
micros = None
dhttype = 0  # 0=DHT11 1=DHT21/DHT22

FALL_EDGES = 42  # we have 42 falling edges during data receive

times = list(range(FALL_EDGES))
index = 0


# The interrupt handler
def edge(line):
    global index
    global times
    global micros
    times[index] = micros.counter()
    if index < (FALL_EDGES - 1):  # Avoid overflow of the buffer in case of any noise on the line
        index += 1


def init(timer_id=2, data_pin='Y2', the_dhttype='DHT22'):
    global data
    global micros
    global timer
    global dhttype

    if (the_dhttype == 'DHT11'):
        dhttype = 0
    else:
        dhttype = 1

    # Configure the pid for data communication
    data = Pin(data_pin)
    # Save the ID of the timer we are going to use
    timer = timer_id
    # setup the 1uS timer
    micros = pyb.Timer(timer, prescaler=83, period=0x3fffffff)  # 1MHz ~ 1uS
    # Prepare interrupt handler
    ExtInt(data, ExtInt.IRQ_FALLING, Pin.PULL_UP, None)
    ExtInt(data, ExtInt.IRQ_FALLING, Pin.PULL_UP, edge)


# Start signal
def do_measurement():
    global data
    global micros
    global index
    # Send the START signal
    data.init(Pin.OUT_PP)
    data.low()
    micros.counter(0)
    while micros.counter() < 25000:
        pass
    data.high()
    micros.counter(0)
    while micros.counter() < 20:
        pass
    # Activate reading on the data pin
    index = 0
    data.init(Pin.IN, Pin.PULL_UP)
    # Till 5mS the measurement must be over
    pyb.delay(5)


# Parse the data read from the sensor
def process_data():
    global dhttype
    global times


    i = 2  # We ignore the first two falling edges as it is a respomse on the start signal
    result_i = 0
    result = list([0, 0, 0, 0, 0])
    while i < FALL_EDGES:
        result[result_i] <<= 1
        if times[i] - times[i - 1] > 100:
            result[result_i] += 1
        if (i % 8) == 1:
            result_i += 1
        i += 1
    [int_rh, dec_rh, int_t, dec_t, csum] = result

    if dhttype == 0:  # dht11
        humidity = int_rh  # dht11 20% ~ 90%
        temperature = int_t  # dht11 0..50°C
    else:  # dht21,dht22
        humidity = ((int_rh * 256) + dec_rh) / 10
        temperature = (((int_t & 0x7F) * 256) + dec_t) / 10
        if (int_t & 0x80) > 0:
            temperature *= -1

    comp_sum = int_rh + dec_rh + int_t + dec_t
    if (comp_sum & 0xFF) != csum:
        raise ValueError('Checksum does not match')
    return (humidity, temperature)


def measure():
    do_measurement()
    if index != (FALL_EDGES - 1):
        raise ValueError('Data transfer failed: %s falling edges only' % str(index))
    return process_data()


class DHT22:
    def __init__(self, **kw):
        init(**kw)

    def get_measurement(self):
        return measure()