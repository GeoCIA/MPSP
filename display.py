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

from oled import SSD1306_I2C
from pyb import I2C
class Line:
    def __init__(self, msg):
        self.msg = msg


class DOC:
    def __init__(self):
        self._lines = []

    def add_message(self, msg, idx):
        if len(self._lines)-1 >= idx:
            line = self._lines[idx]
            line.msg = msg
        else:
            self._lines.append(Line(msg))

        return self._lines

    def header(self, msg, msg2=''):
        self._lines.insert(0, Line(msg2))
        self._lines.insert(0, Line(msg))


class OLED:
    def __init__(self, bus=1, width=128, height=64):
        i2c = I2C(bus, I2C.MASTER)
        o = SSD1306_I2C(width, height, i2c)
        self._scr = o
        self._width = width
        self._height = height
        self._doc = DOC()

    def message(self, msg, idx):
        lines = self._doc.add_message(msg, idx)

        self._scr.fill(0)
        for i, l in enumerate(lines):
            self._scr.text(str(l.msg), 0, i * 10)
        self._scr.show()

    def header(self, msg1, msg2):
        self._doc.header(msg1, msg2)


DISPLAY = OLED()
# ============= EOF =============================================
