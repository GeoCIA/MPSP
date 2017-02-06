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

# ============= enthought library imports =======================
# ============= standard library imports ========================
# ============= local library imports  ==========================

# ============= EOF =============================================
import io
import os
from pyb import millis

DATA_ROOT = '/sd/mpsp_data'

OPEN_FILES = []

TFUNC_LED = const(3)


def ads1115_event(dev, eid, period, display):
    return csv_datalogger_wrapper(dev, 'ads115', 'A', 'A0,A1,A2,A3', eid, period, verbose=display)


def ds18x20_event(dev, eid, period, display):
    return csv_datalogger_wrapper(dev, 'ds18x20', 'ds18x20', 'TempC', eid, period, verbose=display)


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
    st = millis()

    def tfunc(ctx):
        m = dev.get_measurement()
        print((millis() - st) / 1000, dev, m)
        if verbose and ctx.get('display_enabled', False):
            try:
                from display import DISPLAY
                # print('{}={}'.format(dev, m))
                if isinstance(m, (list, tuple)):
                    for i, mi in enumerate(m):
                        DISPLAY.message('{}{}:{}'.format(name, i, mi), msg_idx + i)
                else:
                    DISPLAY.message('{}:{}'.format(name, m), msg_idx)
            except OSError as e:
                print('display error {}'.format(e))

        if m is not None:
            try:
                # with open(p, 'a') as afile:
                gps = ctx.get('gps', (None, None, None, None, None))
                gps = ','.join(map(str, gps))
                if isinstance(m, (list, tuple)):
                    m = ','.join(map(str, m))
                try:
                    wfile.write('{},{}\n'.format(gps, m))
                except OSError as e:
                    print('write error {}'.format(e))

                if not ctx['iteration'] % 5:
                    print('flushing file')
                    wfile.flush()
            except KeyboardInterrupt:
                # never allow Ctrl+C when writing to disk
                pass

    return event_wrapper(tfunc, None, period)


def event_wrapper(tfunc, ffunc, period, count_threshold=0, iteration_threshold=100):
    ctx = {'last_call': millis(), 'cnt': 0, 'iteration': 0, 'flopbit': 0, 'pflopbit': 0, 'display_enabled': True}

    def func(mctx):

        # permanently disable display
        if ctx['iteration'] == 50:
            ctx['display_enabled'] = False

        if millis() - ctx['last_call'] > period:
            ctx['pflopbit'] = ctx['flopbit']
            ctx['flopbit'] = 1
            if tfunc is not None:
                mctx['iteration'] = ctx['iteration']
                try:
                    tfunc(mctx)
                except KeyboardInterrupt as e:
                    raise e
                except BaseException as e:
                    pyb.LED(TFUNC_LED).on()
                    print('tfunc exception={}'.format(e))
            ctx['cnt'] += 1
            ctx['iteration'] += 1
            if ctx['iteration'] >= iteration_threshold:
                ctx['iteration'] = 0

            if ctx['cnt'] > count_threshold:
                ctx['last_call'] = millis()
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
