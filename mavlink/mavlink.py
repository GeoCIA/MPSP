import struct
from pyb import UART, millis, delay

from mavlink import HEARTBEAT, GLOBAL_POSITION_INT, STATUSTEXT

class Message:
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
            self._buffer+=buf
        else:
            if buf[0]==0xFE:
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

        nb = len(b)-8
        return nb>=self.payload_len
        # if nb > self.payload_len:
        #     self._buffer = None
        #     raise ValueError('too long- mid={} pl={} nb={}'.format(self.message_id, self.payload_len, nb))
        # else:
        #     return nb == self.payload_len

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

        return self.message_id, payload

class MAVLink:
    def __init__(self, uartID=3, baudrate=115200):
        self._uart = UART(uartID, baudrate)
        self.message = Message()

    def wait_heartbeat(self, timeout=5):
        return self.wait_for(HEARTBEAT, timeout)

    def wait_timestamp(self, timeout=5):
        return self.wait_for(None, timeout)

    def wait_for(self, mtype, timeout):
        timeout = timeout * 1000

        st = millis()
        while 1:
            now = millis()
            if now - st > timeout:
                print('timed out')
                return

            msg = self.get_message()
            if msg:
                if msg[0] == mtype:
                    return True

    def get_message(self, timeout=500):
        msg =self.message
        msg.clear()

        st = millis()

        uart = self._uart
        while 1:
            now = millis()
            if now - st > timeout:
                print('timed out')
                return

            n = uart.any()
            if n:
                r=  uart.read(n)
                if msg.update(r):
                    return msg.payload()

#
# class Message:
#     def __init__(self, s):
#         self._buffer = s
#         self.payload_len = 0
#         self.payload = None
#         self._parse()
#
#     def update(self, s):
#         self._buffer += s
#         self._parse()
#
#     def to_text(self):
#         if self.payload_len and len(self._buffer) - 8 >= self.payload_len:
#             return self._buffer
#
#     def _parse(self):
#         b = self._buffer
#         try:
#             self.payload_len = b[1]
#             self.seq_num = b[2]
#             self.sys_id = b[3]
#
#             self.comp_id = b[4]
#             self.message_id = b[5]
#         except IndexError:
#             return
#
#         #print('mid {} {} {}'.format(self.message_id, self.payload_len, len(b)))
#         if len(b) - 8 >= self.payload_len:
#             mid = self.message_id
#             # print('message {}'.format(self.message_id))
#             buf = self._buffer[6:-2]
#             if mid == HEARTBEAT:
#                 self.payload = struct.unpack('IBBBBB', buf)
#             elif mid == GLOBAL_POSITION_INT:
#                 self.payload = struct.unpack('Iiiii', buf)
#             elif mid == STATUSTEXT:
#                 severity = struct.unpack('B', buf)[0]
#                 self.payload = (severity, buf[1:])
#
#     def __str__(self):
#         return str(self._buffer)
#
#     def __repr__(self):
#         return str(self)
#
#
# class MAVLinkSerial:
#     def __init__(self, dev=3, baudrate=57600):
#         self._uart = UART(dev, baudrate)
#
#     def write(self, msg):
#         self._uart.send(msg)
#
#     def recv(self, n=1):
#         waiting = self._uart.any()
#         if waiting < n:
#             n = waiting
#         return self._uart.read(n)
#
#
# class MAVLink:
#     def __init__(self):
#         self._link = MAVLinkSerial()
#         self.message = None
#
#     def wait_heartbeat(self):
#         m = self.recv_match(message_type=HEARTBEAT, blocking=True)
#         self.clear()
#
#     def clear(self):
#         self._clear=True
#
#     def recv_match(self, message_type=HEARTBEAT, blocking=True, timeout=None):
#         if message_type is not None and not isinstance(message_type, list):
#             message_type = [message_type]
#         self._clear = False
#         st = millis()
#         clear = False
#         while 1:
#             #delay(5)
#             if timeout is not None:
#                 now = millis()
#                 if now < st:
#                     st = now
#                 if now - st > timeout:
#                     return
#
#             m = self.recv_msg(clear)
#
#             if m is None:
#                 if blocking:
#                     if timeout is None:
#                         self.select(0.05)
#                     else:
#                         self.select(timeout / 2)
#                     continue
#
#             if message_type is not None and m.message_id not in message_type:
#                 clear=True
#                 continue
#             self.message = None
#             return m
#
#     def select(self, timeout):
#         delay(int(timeout * 1000))
#
#     def recv_msg(self, clear):
#         self.pre_message()
#         if clear:
#             self.message = None
#
#         while True:
#             s = self._link.recv()
#             if s is None:
#                 return
#
#             numnew = len(s)
#             if numnew == 0:
#                 return None
#
#             # if numnew != 0:
#             #     if self.logfile_raw:
#             #         self.logfile_raw.write(str(s))
#             #     if self.first_byte:
#             #         self.auto_mavlink_version(s)
#
#             # We always call parse_char even if the new string is empty, because the existing message buf might already have some valid packet
#             # we can extract
#             msg = self._parse_char(s)
#             if msg:
#                 # if self.logfile and  msg.get_type() != 'BAD_DATA' :
#                 #     usec = int(time.time() * 1.0e6) & ~3
#                 #     self.logfile.write(str(struct.pack('>Q', usec) + msg.get_msgbuf()))
#                 self.post_message(msg)
#                 return msg
#                 # if we failed to parse any messages _and_ no new bytes arrived, return immediately so the client has the option to
#                 # timeout
#
#     def pre_message(self):
#         pass
#
#     def post_message(self, msg):
#         # print('msg. {}'.format(msg.payload_len))
#         pass
#
#     def _bytes_needed(self):
#         return
#
#     def _parse_char(self, s):
#         if self.message is None or self._clear:
#             if s and s[0] == 0xFE:
#                 self.message = Message(s)
#             else:
#                 self.message = Message(b'')
#         else:
#             self.message.update(s)
#
#         if self.message:
#             if self.message.to_text():
#                 return self.message
#                 # if self._message_buffer.endswith('\r'):
#                 #     return self._message_buffer

# ============= EOF =============================================
