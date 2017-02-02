# ===============================================================================
# Copyright 2017 ross
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ===============================================================================

import json
import pyb
import os
import io

from mavlink import GLOBAL_POSITION_INT, HEARTBEAT, ATTITUDE
from mavlink.mavlink import MAVLink
from mpsp import FLIGHT
from mpsp.drivers.tail_leds import generate_tail_flight_pattern, generate_tail_ground_pattern

DATA_ROOT = '/sd/mpsp_data'

# RESERVED TIMERS 2,3,5,6
STATUS_TIMER = const( 1)
HEARTBEAT_TIMER = const( 7)
LED_TIMER = const(8)
STATUS_LED = const( 2)

WARNING_LED = const( 1)
TFUNC_LED = const( 3)

OPEN_FILES = []

DOME_GROUND_PATTERN = (1, 0, 1, 0, 0, 0)
DOME_FLIGHT_PATTERN = (0,0,0,0,0,1,1)
DOME_CNT = 0

TAIL_CNT = 0
TAIL_FLIGHT_PATTERN = generate_tail_flight_pattern()
TAIL_GROUND_PATTERN = generate_tail_ground_pattern()
CURRENT_HASH = None

def ads1115_event(dev, eid, period, display):
    return csv_datalogger_wrapper(dev, 'ads115', 'A', 'A0,A1,A2,A3', eid,period,  verbose=display)


def ds18x20_event(dev, eid, period, display):
    return csv_datalogger_wrapper(dev, 'ds18x20', 'ds18x20', 'TempC', eid,period, verbose=display)


def dht_event(dev, eid, period, display):
    return csv_datalogger_wrapper(dev, 'dht', 'dht', 'Humidity%,TempC', eid, period, verbose=display)


def csv_datalogger_wrapper(dev, rootname, name, header, msg_idx, period, verbose=False):
    root = '{}/{}'.format(DATA_ROOT, rootname)
    try:
        os.mkdir(root)
    except OSError:
        pass

    cnt = 1
    for f in os.listdir(root):
        if not f.endswith('.csv'):
            continue

        h, _ = f.split('.')
        cnt = max(cnt, int(h) + 1)

    p = '{}/{:06n}.csv'.format(root, cnt)
    wfile = io.open(p, 'w')
    header = 'GPS_BOOT_TIME, LAT, LON, ALT, REL_ALT,{}\n'.format(header)
    wfile.write(header)

    OPEN_FILES.append(wfile)
    st = pyb.millis()
    def tfunc(ctx):
        m = dev.get_measurement()
        print((pyb.millis()-st)/1000, dev, m)
        if verbose:
            try:
                from display import DISPLAY
                #print('{}={}'.format(dev, m))
                if isinstance(m, (list, tuple)):
                    for i,mi in enumerate(m):
                        DISPLAY.message('{}{}:{}'.format(name,i, mi), msg_idx+i)
                else:
                    DISPLAY.message('{}:{}'.format(name, m), msg_idx)
            except OSError:
                pass

        if m is not None:
            try:
                # with open(p, 'a') as afile:
                gps = ctx.get('gps', (None,None,None,None,None))
                gps = ','.join(map(str, gps))
                if isinstance(m, (list, tuple)):
                    m = ','.join(map(str, m))
                wfile.write('{},{}\n'.format(gps, m))
                if not ctx['iteration']%5:
                    print('flushing file')
                    wfile.flush()
            except KeyboardInterrupt:
                # never allow Ctrl+C when writing to disk
                pass
    return event_wrapper(tfunc, None, period)


def event_wrapper(tfunc, ffunc, period, count_threshold=0, iteration_threshold=100):
    ctx = {'last_call': pyb.millis(), 'cnt': 0, 'iteration': 0, 'flopbit':0, 'pflopbit': 0}
    def func(mctx):
        if pyb.millis() - ctx['last_call'] > period:
            ctx['pflopbit'] = ctx['flopbit']
            ctx['flopbit'] = 1
            if tfunc is not None:
                mctx['iteration']= ctx['iteration']
                try:
                    tfunc(mctx)
                except KeyboardInterrupt as e:
                    raise e
                except BaseException as e:
                    pyb.LED(TFUNC_LED).on()
                    print('tfunc exception={}'.format(e))
            ctx['cnt'] += 1
            ctx['iteration'] +=1
            if ctx['iteration'] >= iteration_threshold:
                ctx['iteration'] =0

            if ctx['cnt'] > count_threshold:
                ctx['last_call'] = pyb.millis()
                ctx['cnt'] = 0
        else:
            ctx['pflopbit'] = ctx['flopbit']
            ctx['flopbit'] = 0
            if ffunc is not None:
                try:
                    ffunc(ctx)
                except KeyboardInterrupt as e:
                    raise e
                except BaseException as e:
                    print('ffunc exception={}'.format(e))

    return func


class MPSP:
    _devices = None
    _events = None
    _period = 1
    _last_hb = 0
    _oled_enabled = True
    _mavlink = None
    _dome_led_pin = None

    def __init__(self, mode):
        self._mode = mode

    def init(self):
        print('FlightM Mode = {}'.format(self._mode==FLIGHT))

        evts = []
        if self._mode == FLIGHT:
            self._mavlink = MAVLink()

        try:
            os.mkdir('/sd/mpsp_data')
        except OSError:
            pass

        names = []
        with open('mpsp/config.json', 'r') as rfile:
            obj = json.loads(rfile.read())
            print(obj)
            self._period = obj['loop_period']
            self._oled_enabled = obj['oled_enabled']
            self._dome_led_pin = obj.get('dome_led_pin','X2')
            eid = 2
            for di in obj.get('devices'):
                if di.get('enabled'):

                    evt = self._create_device_event(di, eid)

                    if evt is not None:
                        eid += 1
                        evts.append(evt)
                        names.append(di)

        if self._oled_enabled:
            from display import DISPLAY
            mode = 'F' if self._mode ==FLIGHT else 'G'
            DISPLAY.header('MPSP v0.1  {}'.format(mode), '  ')

        self._events = evts
        pyb.delay(100)

    def _dome_cb(self, timer):
        global DOME_CNT
        try:
            v = self._dome_pattern[DOME_CNT]
        except IndexError:
            DOME_CNT = 0
            v = self._dome_pattern[0]

        DOME_CNT+=1
        if v:
            self._dome_led.high()
        else:
            self._dome_led.low()

        global TAIL_CNT
        global CURRENT_HASH
        try:
            v = self._tail_pattern[TAIL_CNT]
        except IndexError:
            TAIL_CNT = 0
            v = self._tail_pattern[0]

        TAIL_CNT += 1
        if v[1]!=CURRENT_HASH:
            CURRENT_HASH = v[1]
            self._spi1.write(v[0])

    def run(self):
        print('run')
        heartbeat_timeout = 5000

        status_tim = pyb.Timer(STATUS_TIMER, freq=1)

        self._tail_pattern = TAIL_GROUND_PATTERN
        self._spi1 = pyb.SPI(1, pyb.SPI.MASTER, phase=1)

        self._dome_pattern = DOME_GROUND_PATTERN
        self._led_timer = pyb.Timer(LED_TIMER, freq=5)
        self._dome_led = pyb.Pin(self._dome_led_pin, pyb.Pin.OUT_PP)
        self._led_timer.callback(self._dome_cb)
        
        status_led = pyb.LED(STATUS_LED)
        def status_cb(t):
            status_led.toggle()
            
        status_tim.callback(status_cb)

        if self._mode == FLIGHT:
            if not self._mavlink.wait_heartbeat():
                 self._mavlink_warning()
                 self._cancel(status_tim)
                 return

        switch = pyb.Switch()
        hbwtim = None
        ctx={}
        cnt = 0
        evts = self._events


        st = pyb.millis()
        while 1:
            try:
                now = pyb.millis()
                if self._mode == FLIGHT:
                    # check for heartbeat timeout
                    if now-self._last_hb > heartbeat_timeout:
                        if hbwtim is None:
                            hbwtim = pyb.Timer(HEARTBEAT_TIMER, freq=10)
                            hbwtim.callback(lambda t: pyb.LED(WARNING_LED).toggle())
                            status_tim.callback(None)
                    elif hbwtim:
                        status_tim.callback(status_cb)
                        pyb.LED(WARNING_LED).off()
                        hbwtim.callback(None)
                        hbwtim = None

                    msgs = self._mavlink.get_messages()
                    if msgs:
                        for msg in msgs:
                        # msg = self._mavlink.get_message()
                        # if msg:
                            #print('mavlink msg={}'.format(msg))
                            mid = msg[0]
                            if mid == HEARTBEAT:
                                self._last_hb = pyb.millis()
                            elif mid == GLOBAL_POSITION_INT:
                                #print('GPS', msg[1])
                                ctx['gps'] = msg[1]
                                if abs(msg[1][4]-msg[1][3])>1000: # 1 meter
                                    self._dome_pattern = DOME_FLIGHT_PATTERN
                                    self._tail_pattern = TAIL_FLIGHT_PATTERN
                                else:
                                    self._dome_pattern = DOME_GROUND_PATTERN
                                    self._tail_pattern = TAIL_GROUND_PATTERN

                            elif mid == ATTITUDE:
                                ctx['attitude'] = msg[1]

                # for evt in evts:
                #     evt(ctx)
                if self._events:
                    try:
                        evt = evts[cnt]
                    except IndexError:
                        evt = evts[0]
                        cnt = 0

                    cnt+=1
                    evt(ctx)

            except KeyboardInterrupt:
                self._cancel(status_tim)
                break

            #except BaseException as e:
            #    print('run exception', e)

            if switch():
                self._cancel(status_tim)
                break

        if OPEN_FILES:
            for f in OPEN_FILES:
                f.close()

    def _mavlink_warning(self):
        led1 = pyb.LED(1)
        led2 = pyb.LED(2)
        led3 = pyb.LED(3)
        led4 = pyb.LED(4)

        led1.on()
        led2.on()
        led3.on()
        led4.on()

        for i in range(11):
            led1.toggle()
            led2.toggle()
            led3.toggle()
            led4.toggle()
            pyb.delay(250)

    def _cancel(self, tim):
        tim.callback(None)
        pyb.LED(WARNING_LED).off()
        self._led_timer.callback(None)

    def _create_device_event(self, dev, eid):
        klass = dev['klass']
        # name = dev.get('name', klass)
        factory = None
        if klass == 'DHT22':
            def factory():
                from mpsp.drivers.dht import DHT22
                d = DHT22(data_pin=dev.get('data_pin', 'Y2'))
                return dht_event(d, eid, dev.get('period',1000), self._oled_enabled)
        elif klass == 'DS18X20':
            def factory():
                from mpsp.drivers.ds18x20 import DS18X20
                d = DS18X20(dev.get('data_pin', 'Y3'))
                return ds18x20_event(d, eid, dev.get('period', 1000), self._oled_enabled)
        elif klass == 'ADS1115':
            def factory():
                from mpsp.drivers.ads1x15 import ADS1115
                i2c = pyb.I2C(dev.get('bus', 1), pyb.I2C.MASTER)
                d = ADS1115(i2c)
                return ads1115_event(d, eid, dev.get('period', 250), self._oled_enabled)

        if factory:
            dd = factory()
            return dd

# ============= EOF =============================================
