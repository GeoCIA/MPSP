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

from display import DISPLAY
from mavlink.mavlink import MAVLink
from mpsp import FLIGHT


def status_event(period):
    led = pyb.LED(2)

    def tfunc():
        return led.on()

    def ffunc():
        return led.off()

    return event_wrapper(tfunc, ffunc, period, 50)


DATA_ROOT = '/sd/mpsp_data'


def ds18x20_event(dev, eid):
    return csv_datalogger_wrapper(dev, 'ds18x20', 'ds18', 'Time,TempC', eid, verbose=True)


def dht_event(dev, eid):
    return csv_datalogger_wrapper(dev, 'dht', 'dht', 'Time,Humidity%,TempC', eid, verbose=True)


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

    def __init__(self, mode):
        self._mode = mode

    def init(self):

        if self._mode == FLIGHT:
            self._mavlink = MAVLink()

        try:
            os.mkdir('/sd/mpsp_data')
        except OSError:
            pass

        evts = [status_event(500)]

        names = []
        with open('mpsp/config.json', 'r') as rfile:
            obj = json.loads(rfile.read())
            self._period = obj['loop_period']
            eid = 2
            for di in obj.get('devices'):
                if di.get('enabled'):

                    evt = self._create_device_event(di, eid)
                    if evt is not None:
                        eid += 1
                        evts.append(evt)
                        names.append(di)

        DISPLAY.header('MPSP v0.1', '  ')
        self._events = evts

    def run(self):
        cnt = 0
        period = self._period
        mode = self._mode
        hbfunc = self._mavlink.get_heartbeat

        while 1:
            if mode == FLIGHT:
                hb = hbfunc()
                print('hb={}'.format(hb.payload))

            for evt in self._events:
                evt()

            pyb.delay(period)
            cnt += 1

    # private
    def _create_device_event(self, dev, eid):
        klass = dev['klass']
        # name = dev.get('name', klass)
        factory = None
        if klass == 'DHT22':
            def factory():
                from mpsp.drivers.dht import DHT22
                d = DHT22(data_pin=dev.get('data_pin', 'Y2'))
                return dht_event(d, eid)

        elif klass == 'DS18X20':
            def factory():
                from mpsp.drivers.ds18x20 import DS18X20
                d = DS18X20(dev.get('data_pin', 'Y3'))
                return ds18x20_event(d, eid)

        if factory:
            dd = factory()
            return dd

# ============= EOF =============================================
