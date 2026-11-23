"""Feetech STS系サーボのバスから現在位置を読む(SCSプロトコルの同期読み出しだけを実装)。"""
import os
import select
import termios
import time

PRESENT_POSITION = 56
SYNC_READ = 0x82
BROADCAST_ID = 0xFE


class FeetechBus:
    def __init__(self, port, baud=termios.B1000000):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = baud
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def close(self):
        os.close(self.fd)

    def read_positions(self, ids, timeout=0.02):
        """idsの順に生の位置を返す。応答が揃わなければNone。"""
        params = [PRESENT_POSITION, 2, *ids]
        body = [BROADCAST_ID, len(params) + 2, SYNC_READ, *params]
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, bytes([0xFF, 0xFF, *body, ~sum(body) & 0xFF]))
        want = 8 * len(ids)
        buf = b''
        deadline = time.monotonic() + timeout
        while len(buf) < want:
            left = deadline - time.monotonic()
            if left <= 0 or not select.select([self.fd], [], [], left)[0]:
                return None
            buf += os.read(self.fd, want - len(buf))
        found = {}
        i = 0
        while i + 8 <= len(buf):
            if buf[i] != 0xFF or buf[i + 1] != 0xFF or buf[i + 3] != 4:
                i += 1
                continue
            sid, err, lo, hi, chk = buf[i + 2], buf[i + 4], buf[i + 5], buf[i + 6], buf[i + 7]
            if (~(sid + 4 + err + lo + hi) & 0xFF) == chk:
                v = lo | (hi << 8)
                # 位置は15ビット目が符号の符号付き絶対値で返る。
                found[sid] = -(v & 0x7FFF) if v & 0x8000 else v
            i += 8
        if any(sid not in found for sid in ids):
            return None
        return [found[sid] for sid in ids]
