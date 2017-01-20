import struct
from pyb import UART, millis, delay

from mavlink import HEARTBEAT, GLOBAL_POSITION_INT, STATUSTEXT


class Message:
    def __init__(self, s):
        self._buffer = s
        self.payload_len = 0
        self.payload = None

    def update(self, s):
        self._buffer += s

    def to_text(self):
        self._parse()
        if self.payload_len and len(self._buffer) - 8 >= self.payload_len:
            return self._buffer

    def _parse(self):
        b = self._buffer
        try:
            self.payload_len = b[1]
            self.seq_num = b[2]
            self.sys_id = b[3]

            self.comp_id = b[4]
            self.message_id = b[5]
        except IndexError:
            pass

        if len(b) - 8 >= self.payload_len:
            mid = self.message_id
            # print('message {}'.format(self.message_id))
            buf = self._buffer[6:-2]
            if mid == HEARTBEAT:
                self.payload = struct.unpack('IBBBBB', buf)
            elif mid == GLOBAL_POSITION_INT:
                self.payload = struct.unpack('Iiiii', buf)
            elif mid == STATUSTEXT:
                severity = struct.unpack('B', buf)[0]
                self.payload = (severity, buf[1:])

    def __str__(self):
        return str(self._buffer)

    def __repr__(self):
        return str(self)


class MAVLinkSerial:
    def __init__(self, dev=3, baudrate=57600):
        self._uart = UART(dev, baudrate)

    def write(self, msg):
        self._uart.send(msg)

    def recv(self, n=1):
        waiting = self._uart.any()
        if waiting < n:
            n = waiting
        return self._uart.read(n)


class MAVLink:
    def __init__(self):
        self._link = MAVLinkSerial()
        self.message = None

    def wait_heartbeat(self):
        return self.recv_match(message_type=HEARTBEAT, blocking=True)

    def recv_match(self, message_type=HEARTBEAT, blocking=True, timeout=None):
        if message_type is not None and not isinstance(message_type, list):
            message_type = [message_type]

        st = millis()
        while 1:
            if timeout is not None:
                now = millis()
                if now < st:
                    st = now
                if now - st > timeout:
                    return

            m = self.recv_msg()

            if m is None:
                if blocking:
                    if timeout is None:
                        self.select(0.05)
                    else:
                        self.select(timeout / 2)
                    continue

            if message_type is not None and m.message_id not in message_type:
                continue

            return m

    def select(self, timeout):
        delay(int(timeout * 1000))

    def recv_msg(self):
        self.pre_message()

        while True:
            s = self._link.recv()
            if s is None:
                return

            numnew = len(s)
            if numnew == 0:
                return None

            # if numnew != 0:
            #     if self.logfile_raw:
            #         self.logfile_raw.write(str(s))
            #     if self.first_byte:
            #         self.auto_mavlink_version(s)

            # We always call parse_char even if the new string is empty, because the existing message buf might already have some valid packet
            # we can extract
            msg = self._parse_char(s)
            if msg:
                # if self.logfile and  msg.get_type() != 'BAD_DATA' :
                #     usec = int(time.time() * 1.0e6) & ~3
                #     self.logfile.write(str(struct.pack('>Q', usec) + msg.get_msgbuf()))
                self.post_message(msg)
                self.message = None
                return msg
                # if we failed to parse any messages _and_ no new bytes arrived, return immediately so the client has the option to
                # timeout

    def pre_message(self):
        pass

    def post_message(self, msg):
        # print('msg. {}'.format(msg.payload_len))
        pass

    def _bytes_needed(self):
        return

    def _parse_char(self, s):
        if self.message is None:
            if s and s[0] == 0xFE:
                self.message = Message(s)
        else:
            self.message.update(s)

        if self.message:
            if self.message.to_text():
                return self.message
                # if self._message_buffer.endswith('\r'):
                #     return self._message_buffer

# ============= EOF =============================================
