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
import os

import pyb

from mavlink import GLOBAL_POSITION_INT, HEARTBEAT, ATTITUDE
from mavlink.mavlink import MAVLink
from mpsp import FLIGHT
from mpsp.events import ads1115_event, ds18x20_event, dht_event, OPEN_FILES
from mpsp.led_patterns import TAIL_FLIGHT_PATTERN,TAIL_LANDING_PATTERN,TAIL_GROUND_PATTERN, TAIL_CLEAR, \
    DOME_FLIGHT_PATTERN, DOME_GROUND_PATTERN, STATUS_PATTERN


# RESERVED TIMERS 2,3,5,6
STATUS_TIMER = const( 1)
HEARTBEAT_TIMER = const( 7)
LED_TIMER = const(8)
STATUS_LED = const( 2)

WARNING_LED = const( 1)


class MPSP:
    _devices = None
    _events = None
    _period = 1
    _last_hb = 0
    _oled_enabled = True
    _mavlink = None
    _dome_led_pin = None
    _status_cnt =  0
    _dome_cnt = 0
    _tail_cnt = 0
    _current_hash = None

    def __init__(self, mode):
        self._mode = mode
        self._status_pattern = STATUS_PATTERN

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
            DISPLAY.header('MPSP v0.2  {}'.format(mode), '  ')

        self._events = evts
        pyb.delay(100)

    def run(self):
        print('run')
        heartbeat_timeout = 5000

        #status_tim = pyb.Timer(STATUS_TIMER, freq=1)
        self._status_led = pyb.LED(STATUS_LED)
        self._dome_led = pyb.Pin(self._dome_led_pin, pyb.Pin.OUT_PP)

        self._tail_pattern = TAIL_GROUND_PATTERN
        self._spi1 = pyb.SPI(1, pyb.SPI.MASTER, phase=1)

        self._dome_pattern = DOME_GROUND_PATTERN
        self._led_timer = pyb.Timer(LED_TIMER, freq=8)
        self._led_timer.callback(self._led_cb)

        if self._mode == FLIGHT:
            if not self._mavlink.wait_heartbeat():
                 self._mavlink_warning()
                 self._cancel()
                 return

        switch = pyb.Switch()
        hbwtim = None
        ctx={}
        cnt = 0
        evts = self._events

        while 1:
            try:
                now = pyb.millis()
                if self._mode == FLIGHT:
                    # check for heartbeat timeout
                    if now-self._last_hb > heartbeat_timeout:
                        if hbwtim is None:
                            hbwtim = pyb.Timer(HEARTBEAT_TIMER, freq=10)
                            hbwtim.callback(lambda t: pyb.LED(WARNING_LED).toggle())
                    elif hbwtim:
                        pyb.LED(WARNING_LED).off()
                        hbwtim.callback(None)
                        hbwtim = None

                    msgs = self._mavlink.get_messages()
                    if msgs:
                        for msg in msgs:
                            mid = msg[0]
                            if mid == HEARTBEAT:
                                self._last_hb = pyb.millis()
                            elif mid == GLOBAL_POSITION_INT:
                                ctx['gps'] = msg[1]
                                relalt = abs(msg[1][4]-msg[1][3])
                                relalt = 501
                                if relalt >1000: # 1 meter
                                    self._dome_pattern = DOME_FLIGHT_PATTERN
                                    self._tail_pattern = TAIL_FLIGHT_PATTERN
                                elif relalt>500:
                                    self._tail_pattern = TAIL_LANDING_PATTERN
                                else:
                                    self._dome_pattern = DOME_GROUND_PATTERN
                                    self._tail_pattern = TAIL_GROUND_PATTERN

                            elif mid == ATTITUDE:
                                ctx['attitude'] = msg[1]

                if evts:
                    try:
                        evt = evts[cnt]
                    except IndexError:
                        evt = evts[0]
                        cnt = 0

                    cnt+=1
                    evt(ctx)

            except KeyboardInterrupt:
                self._cancel()
                break

            if switch():
                self._cancel()
                break

        if OPEN_FILES:
            for f in OPEN_FILES:
                f.close()

        self._cleanup()

    def _led_cb(self, timer):
        # status
        status_cnt = self._status_cnt
        try:
             v = self._status_pattern[status_cnt]
        except IndexError:
             status_cnt = 0
             v = self._status_pattern[0]
        status_cnt +=1
        self._status_cnt = status_cnt
        if v:
            self._status_led.on()
        else:
            self._status_led.off()

        # dome
        dome_cnt = self._dome_cnt
        try:
            v = self._dome_pattern[dome_cnt]
        except IndexError:
            dome_cnt = 0
            v = self._dome_pattern[0]

        dome_cnt+=1
        self._dome_cnt = dome_cnt
        if v:
            self._dome_led.high()
        else:
            self._dome_led.low()

        # tail
        tail_cnt = self._tail_cnt
        try:
            v = self._tail_pattern[tail_cnt]
        except IndexError:
            tail_cnt = 0
            v = self._tail_pattern[0]

        tail_cnt += 1
        self._tail_cnt = tail_cnt
        if v[1]!=self._current_hash:
            self._current_hash = v[1]
            self._spi1.write(v[0])

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

    def _cancel(self):
        self._cleanup()

    def _cleanup(self):
        pyb.LED(WARNING_LED).off()
        self._led_timer.callback(None)
        self._spi1.write(TAIL_CLEAR[0])
        self._dome_led.low()

    def _create_device_event(self, dev, eid):
        klass = dev['klass']
        # name = dev.get('name', klass)
        factory = None
        if klass == 'DHT22':
            def factory():
                from mpsp.drivers.dht import DHT22
                d = DHT22(data_pin=dev.get('data_pin', 'Y2'))
                return dht_event(d, eid, dev.get('period', 1000), self._oled_enabled)
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
