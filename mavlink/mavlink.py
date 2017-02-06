import struct
from pyb import UART, millis, delay

from mavlink import HEARTBEAT, GLOBAL_POSITION_INT, STATUSTEXT, ATTITUDE, AHRS2, AHRS3


class Message:
    payload_len = None
    seq_num = None
    sys_id = None
    comp_id = None
    message_id = None

    def __init__(self):
        self._buffer = None

    def clear(self):
        self._buffer = None
        self.payload_len = None
        self.seq_num = None
        self.sys_id = None
        self.comp_id = None
        self.message_id = None

    def update(self, buf):
        if not buf:
            return

        if self._buffer:
            self._buffer += buf
        else:
            if buf[0] == 0xFE:
                self._buffer = buf

        return self.complete()

    def complete(self):
        b = self._buffer
        try:
            self.payload_len = b[1]
            self.seq_num = b[2]
            self.sys_id = b[3]
            self.comp_id = b[4]
            self.message_id = b[5]
        except (IndexError, TypeError):
            return
        nb = len(b) - 8
        return nb >= self.payload_len

    def payload(self):
        mid = self.message_id
        payload = None
        buf = self._buffer[6:-2]
        if mid == HEARTBEAT:
            payload = struct.unpack('IBBBBB', buf)
        elif mid == GLOBAL_POSITION_INT:
            payload = struct.unpack('Iiiii', buf)
        elif mid == STATUSTEXT:
            severity = struct.unpack('B', buf)[0]
            payload = (severity, buf[1:])
        elif mid == ATTITUDE:
            payload = struct.unpack('Iffffff', buf)
        elif mid == AHRS2:
            payload = struct.unpack('IIffff', buf)
        elif mid == AHRS3:
            payload = struct.unpack('IIffffffff', buf)

        return self.message_id, payload

    def extra(self):

        e = self._buffer[self.payload_len + 8:]
        if e:
            msg = Message()
            msg.update(e)
            return msg


class MAVLink:
    def __init__(self, uartID=6, baudrate=115200):
        self._uart = UART(uartID, baudrate)
        self.message = Message()

    def wait_heartbeat(self, timeout=5):
        return self.wait_for(HEARTBEAT, timeout)

    def wait_timestamp(self, timeout=5):
        return self.wait_for(None, timeout)

    def wait_for(self, mtype, timeout):
        timeout *= 1000

        st = millis()
        while 1:
            now = millis()
            if now - st > timeout:
                print('wait for timed out')
                return

            msgs = self.get_messages()
            if msgs:
                for msg in msgs:
                    if msg:
                        print('wait for={}, msg={}'.format(mtype, msg))
                        if msg[0] == mtype:
                            return True

    def get_messages(self, timeout=750):
        st = millis()

        msg = None
        uart = self._uart
        payloads = []
        while 1:
            now = millis()
            if now - st > timeout:
                return

            n = uart.any()
            if n:
                if not msg:
                    msg = Message()

                if msg.update(uart.read(n)):
                    payloads.append(msg.payload())
                    msg = msg.extra()

            else:
                if msg:
                    if msg.complete():
                        payloads.append(msg.payload())
                return payloads

# ============= EOF =============================================
