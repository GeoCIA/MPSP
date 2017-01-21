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

from mavlink import GLOBAL_POSITION_INT, HEARTBEAT
from mavlink.mavlink import MAVLink
from mpsp import FLIGHT

DATA_ROOT = '/sd/mpsp_data'

# RESERVED TIMERS 2,3,5,6
STATUS_TIMER = 1
HEARTBEAT_TIMER = 7
GPS_TIMER = 8


def ds18x20_event(dev, eid, display):
    return csv_datalogger_wrapper(dev, 'ds18x20', 'ds18', 'Time,TempC', eid, verbose=display)


def dht_event(dev, eid, display):
    return csv_datalogger_wrapper(dev, 'dht', 'dht', 'Time,Humidity%,TempC', eid, verbose=display)


def csv_datalogger_wrapper(dev, rootname, name, header, msg_idx, verbose=False):
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
    with open(p, 'w') as wfile:
        wfile.write(header)

    def tfunc():
        m = dev.get_measurement()
        if verbose:
            from display import DISPLAY
            # print('{}={}'.format(dev, m))
            DISPLAY.message('{}:{}'.format(name, m), msg_idx)
        if m is not None:
            with open(p, 'a') as afile:
                afile.write('{},{}\n'.format(pyb.millis(), m))

    return event_wrapper(tfunc, None, 1000)


def event_wrapper(tfunc, ffunc, period, count_threshold=0):
    ctx = {'last_call': pyb.millis(), 'cnt': 0}

    def func():
        if pyb.millis() - ctx['last_call'] > period:
            if tfunc is not None:
                tfunc()
            ctx['cnt'] += 1
            if ctx['cnt'] > count_threshold:
                ctx['last_call'] = pyb.millis()
                ctx['cnt'] = 0
        else:
            if ffunc is not None:
                ffunc()

    return func


class MPSP:
    _devices = None
    _events = None
    _period = 1
    _last_hb = 0
    _oled_enabled = True

    def __init__(self, mode):
        self._mode = mode

    def init(self):

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
            DISPLAY.header('MPSP v0.1', '  ')

        self._events = evts

    def run(self):

        period = self._period
        tim = pyb.Timer(STATUS_TIMER, freq=1)

        if self._mode == FLIGHT:
            if not self._mavlink.wait_heartbeat():
                 self._mavlink_warning()
                 self._cancel(tim)
                 return

            if not self._setup_rtc():
                self._cancel(tim)
                return

        tim.callback(lambda t: pyb.LED(2).toggle())

        switch = pyb.Switch()
        while 1:
            try:
                if self._mode == FLIGHT:    
                    print('get_message={}'.format(self._mavlink.get_message()))

                for i, evt in enumerate(self._events):
                    try:
                        evt()
                    except BaseException as e:
                        pass

            except BaseException as e:
                print(e)

            if switch():
                self._cancel(tim)
                break


    # private
    # def _mav_event(self):
    #     msg = self._mavlink.recv_match(message_type=None, blocking=False)
    #     if msg:
    #         print('got message: {} {}'.format(msg.message_id, msg.payload))
    #         self._mavlink.clear()
    def _mavlink_warning(self):
        led1 = pyb.LED(1)
        led2 = pyb.LED(2)
        led3 = pyb.LED(3)
        led4 = pyb.LED(4)

        led1.on()
        led2.on()
        led3.on()
        led4.on()

        for i in range(10):
            led1.toggle()
            led2.toggle()
            led3.toggle()
            led4.toggle()
            pyb.delay(250)

    def _cancel(self, tim):
        tim.callback(None)
        pyb.LED(2).off()

    def _setup_rtc(self):
        ts = self._mavlink.wait_timestamp()
        if not ts:
            self._mavlink_warning()

    def _create_device_event(self, dev, eid):
        klass = dev['klass']
        # name = dev.get('name', klass)
        factory = None
        if klass == 'DHT22':
            def factory():
                from mpsp.drivers.dht import DHT22
                d = DHT22(data_pin=dev.get('data_pin', 'Y2'))
                return dht_event(d, eid, self._oled_enabled)

        elif klass == 'DS18X20':
            def factory():
                from mpsp.drivers.ds18x20 import DS18X20
                d = DS18X20(dev.get('data_pin', 'Y3'))
                return ds18x20_event(d, eid, self._oled_enabled)

        if factory:
            dd = factory()
            return dd

# ============= EOF =============================================
