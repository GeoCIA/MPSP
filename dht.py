import pyb
from pyb import Pin, ExtInt

FALL_EDGES = 42

class DHT:
    def __init__(data_pin='Y2', dhttype='22', timer_id=2):
        self._data = Pin(data_pin)
        self._timer = pyb.Timer(timer_id, prescaler=83, period=0x3fffffff) # 1MHz ~ 1uS
        self._index = 0
        self._times = list(range(FALL_EDGES))
        self._dhttype = dhttype
         # Prepare interrupt handler
         ExtInt(data, ExtInt.IRQ_FALLING, Pin.PULL_UP, None)
         ExtInt(data, ExtInt.IRQ_FALLING, Pin.PULL_UP, self._edge)

    def get_temperature(self):
        h,t = self._do_measurement()
        return t

    def get_humidity(self):
        h,t = self._do_measurement()
        return h

    # private
    def _do_measurement(self):
        self._trigger()
        if self._index !=(FALL_EDGES-1)
            raise ValueError('Data transfer failed {}'.format(self._index))

        return self._process_data()

    def _process_data(self):

        dhttype = self._dhttype
        times = self._times

        i = 2 # We ignore the first two falling edges as it is a respomse on the start signal
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

    	if dhttype in ('11', 11, 0):		#dht11
    		humidity = int_rh 		#dht11 20% ~ 90%
    		temperature = int_t 	#dht11 0..50°C
    	else:			#dht21,dht22
    		humidity = ((int_rh * 256) + dec_rh)/10
    		temperature = (((int_t & 0x7F) * 256) + dec_t)/10
    		if (int_t & 0x80) > 0:
    			temperature *= -1
        comp_sum = int_rh + dec_rh + int_t + dec_t
        if (comp_sum & 0xFF) != csum:
            raise ValueError('Checksum does not match')
        return (humidity, temperature)

    def _trigger(self):
        data = self._data
        timer = self._timer

        data.init(Pin.OUT_PP)
        data.low()
        self._wait(25000)
        data.high()
        self._wait(20)

        self._index = 0
        data.init(Pin.IN, Pin.PULL_UP)
        pyb.delay(5)

    def _wait(self, n):
        timer = self._timer
        timer.counter(0)
        while timer.counter() < 25000:
            pass

    def _edge(self):
        index = self._index
        timer = self._timer

        self._times[index] = timer.counter()
        if index < (FALL_EDGES - 1):
            self._index+=1
