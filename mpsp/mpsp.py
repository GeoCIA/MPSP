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


def status_event(period):
    led = pyb.LED(2)

    def tfunc():
        return led.on()

    def ffunc():
        return led.off()

    return event_wrapper(tfunc, ffunc, period, 50)


DATA_ROOT = '/sd/mpsp_data'


def dht_event(dev):
    root = '{}/dht'.format(DATA_ROOT)
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

    def tfunc():
        m = dev.get_measurement()
        print(m)
        with open(p, 'a') as afile:
            afile.write('{},{},{}\n'.format(pyb.millis(), m[0], m[1]))

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

    def init(self):
        try:
            os.mkdir('/sd/mpsp_data')
        except OSError:
            pass

        devs = {}
        with open('mpsp/config.json', 'r') as rfile:
            obj = json.loads(rfile.read())
            self._period = obj['loop_period']

            for di in obj.get('devices'):
                dev = self._create_device(di)
                if dev is not None:
                    devs[dev.name] = dev

        self._devices = devs

        self._events = [status_event(500), dht_event(devs['DHT22'])]

    def run(self):
        cnt = 0
        period = self._period
        while 1:
            for evt in self._events:
                evt()

            pyb.delay(period)
            cnt += 1

    # private
    def _create_device(self, dev):
        klass = dev['klass']
        name = dev.get('name', klass)

        if klass == 'DHT22':
            def factory():
                from mpsp.drivers.dht import DHT22
                d = DHT22(data_pin=dev.get('data_pin','Y2'))
                return d

        dd = factory()
        dd.name = name
        return dd


# ============= EOF =============================================
