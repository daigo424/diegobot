#!/usr/bin/env python3
"""LiDAR本体のシリアル番号から、右前・左後それぞれが挿さっているポート(by-path)を探して出力する。

使い方: find_lidar_ports.py <右前のシリアル番号> <左後のシリアル番号>
出力: 右前と左後のポートを空白区切りで1行(見つからなければ "-")。

付属のUSB変換基板(CP2102)はどれもシリアル番号が同じで見分けられないため、LiDAR本体に問い合わせる。
動いているLiDARに問い合わせると止めてしまうため、結果は起動ごとに覚えておき、ポートの組み合わせが
変わったときだけ問い合わせ直す。
"""
import glob
import json
import os
import subprocess
import sys
import termios
import time

CACHE = '/tmp/diegobot-lidar-ports.json'
CP2102 = '10c4:ea60'
CONTAINERS = ['diegobot_lidar_front_right', 'diegobot_lidar_rear_left']
BAUD = termios.B230400
CMD_STOP = b'\xa5\x65'
CMD_DEVICE_INFO = b'\xa5\x90'
ANS_DEVICE_INFO = 0x04


def usb_id(tty):
    d = os.path.realpath(f'/sys/class/tty/{tty}/device')
    while d != '/':
        if os.path.exists(f'{d}/idVendor'):
            with open(f'{d}/idVendor') as v, open(f'{d}/idProduct') as p:
                return f'{v.read().strip()}:{p.read().strip()}'
        d = os.path.dirname(d)
    return None


def lidar_ports():
    # by-idは同じシリアル番号の基板が1つにまとめられるため、USBのIDで探す。
    return [p for p in sorted(glob.glob('/dev/serial/by-path/*-port0'))
            if usb_id(os.path.basename(os.path.realpath(p))) == CP2102]


def read_serial(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = BAUD
        termios.tcsetattr(fd, termios.TCSANOW, a)
        for _ in range(3):
            os.write(fd, CMD_STOP)
            time.sleep(0.1)
            termios.tcflush(fd, termios.TCIOFLUSH)
            os.write(fd, CMD_DEVICE_INFO)
            buf = b''
            deadline = time.time() + 1.0
            while time.time() < deadline:
                try:
                    buf += os.read(fd, 256)
                except BlockingIOError:
                    time.sleep(0.01)
                i = buf.find(b'\xa5\x5a')
                # 見出し7バイト(最後が種類)の後に、機種1・ファーム2・ハード1・シリアル番号16バイトが続く。
                if i >= 0 and len(buf) >= i + 27 and buf[i + 6] == ANS_DEVICE_INFO:
                    return ''.join(f'{b:X}' for b in buf[i + 11:i + 27])
        return None
    finally:
        os.close(fd)


def serials_in_use():
    """動いているLiDARのコンテナが持つポートと、そのドライバがログに出したシリアル番号。
    起動時はmakeより先にDockerがコンテナを再開しているため、それらのポートには問い合わせない。"""
    found = {}
    for name in CONTAINERS:
        r = subprocess.run(['docker', 'inspect', '-f', '{{.State.Running}} {{range .HostConfig.Devices}}{{.PathOnHost}}{{end}}', name],
                           capture_output=True, text=True)
        running, _, path = r.stdout.strip().partition(' ')
        if running != 'true' or not path:
            continue
        logs = subprocess.run(['docker', 'logs', name], capture_output=True, text=True)
        serials = [line.split('Serial:')[1].strip() for line in (logs.stdout + logs.stderr).splitlines() if 'Serial:' in line]
        found[path] = serials[-1] if serials else None
    return found


def boot_id():
    with open('/proc/sys/kernel/random/boot_id') as f:
        return f.read().strip()


def main():
    wanted = sys.argv[1:3]
    ports = lidar_ports()
    try:
        with open(CACHE) as f:
            cache = json.load(f)
        if cache['boot_id'] != boot_id() or cache['ports'] != ports:
            raise ValueError
        serials = cache['serials']
    except (OSError, ValueError, KeyError):
        in_use = serials_in_use() if ports else {}
        serials = {p: in_use[p] if p in in_use else read_serial(p) for p in ports}
        if all(serials.values()):
            with open(CACHE, 'w') as f:
                json.dump({'boot_id': boot_id(), 'ports': ports, 'serials': serials}, f)
    by_serial = {s: p for p, s in serials.items() if s}
    print(' '.join(by_serial.get(s, '-') if s else '-' for s in wanted))


if __name__ == '__main__':
    main()
