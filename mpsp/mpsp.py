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

DATA_ROOT = '/sd/mpsp_data'

# RESERVED TIMERS 2,3,5,6
STATUS_TIMER = 1
HEARTBEAT_TIMER = 7
DOME_LED_TIMER = 8
DOME_LED_PIN = 'X2'
STATUS_LED = 2

WARNING_LED = 1
TFUNC_LED = 3

OPEN_FILES = []

DOME_GROUND_PATTERN = (0, 1, 0, 1, 0, 1)
DOME_FLIGHT_PATTERN = (0,0,0,0,0,1,1)
DOME_CNT = 0


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
    def tfunc(ctx):
        m = dev.get_measurement()

        if verbose:
            from display import DISPLAY
            #print('{}={}'.format(dev, m))
            if isinstance(m, (list, tuple)):
                for i,mi in enumerate(m):
                    DISPLAY.message('{}{}:{}'.format(name,i, mi), msg_idx+i)
            else:
                DISPLAY.message('{}:{}'.format(name, m), msg_idx)

        if m is not None:
            try:
                # with open(p, 'a') as afile:
                gps = ctx.get('gps', (None,None,None,None,None))
                gps = ','.join(map(str, gps))
                if isinstance(m, (list, tuple)):
                    m = ','.join(map(str, m))
                wfile.write('{},{}\n'.format(gps, m))
            except KeyboardInterrupt:
                # never allow Ctrl+C when writing to disk
                pass

    return event_wrapper(tfunc, None, period)


def event_wrapper(tfunc, ffunc, period, count_threshold=0):
    ctx = {'last_call': pyb.millis(), 'cnt': 0}
    def func(mctx):
        if pyb.millis() - ctx['last_call'] > period:
            if tfunc is not None:
                try:
                    tfunc(mctx)
                except KeyboardInterrupt as e:
                    raise e
                except BaseException as e:
                    pyb.LED(TFUNC_LED).on()
                    print('tfunc exception={}'.format(e))
            ctx['cnt'] += 1
            if ctx['cnt'] > count_threshold:
                ctx['last_call'] = pyb.millis()
                ctx['cnt'] = 0
        else:
            if ffunc is not None:
                try:
                    ffunc()
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

    def __init__(self, mode):
        self._mode = mode

    def init(self):
        print('FlightM Mode = {}'.format(self._mode==FLIGHT))
        if self._mode == FLIGHT:
            self._mavlink = MAVLink()

        try:
            os.mkdir('/sd/mpsp_data')
        except OSError:
            pass

        evts = []
        names = []
        with open('mpsp/config.json', 'r') as rfile:
            obj = json.loads(rfile.read())
            print(obj)
            self._period = obj['loop_period']
            self._oled_enabled = obj['oled_enabled']

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
            v = self._pattern[DOME_CNT]
        except IndexError:
            DOME_CNT = 0
            v = self._pattern[0]

        DOME_CNT+=1

        if v:
            self._dome_led.high()
        else:
            self._dome_led.low()

    def run(self):

        heartbeat_timeout = 5000

        status_tim = pyb.Timer(STATUS_TIMER, freq=1)
        self._dome_timer = pyb.Timer(DOME_LED_TIMER, freq=10)
        self._dome_led = pyb.Pin(DOME_LED_PIN, pyb.Pin.OUT_PP)
        self._dome_timer.callback(self._dome_cb)
        self._pattern = DOME_GROUND_PATTERN

        if self._mode == FLIGHT:
            if not self._mavlink.wait_heartbeat():
                 self._mavlink_warning()
                 self._cancel(status_tim)
                 return
        
        status_led = pyb.LED(STATUS_LED)
        def status_cb(t):
            status_led.toggle()
            
        status_tim.callback(status_cb)

        switch = pyb.Switch()
        hbwtim = None
        ctx={}
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
                                    self._pattern = DOME_FLIGHT_PATTERN
                                else:
                                    self._pattern = DOME_GROUND_PATTERN
                            elif mid == ATTITUDE:
                                ctx['attitude'] = msg[1]

                for i, evt in enumerate(self._events):
                     evt(ctx)
                pyb.delay(5)
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
        self._dome_timer.callback(None)

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
