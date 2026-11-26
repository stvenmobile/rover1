#!/usr/bin/env python3
"""fake_llc.py - pretend to be rover1's ESP32 llc firmware (v0.1.0) on a pseudo-terminal.

Lets you run the ROS 2 stack without the robot, through the real rover1_hardware serial code:

  python3 fake_llc.py /tmp/rover1_fake        # creates the link /tmp/rover1_fake -> /dev/pts/N
  ros2 launch rover1_bringup rover1.launch.py device:=/tmp/rover1_fake

Wheels follow V commands with the firmware's acceleration limit; the 300 ms watchdog stops them.
`kill -USR1 <pid>` simulates an ESP32 reboot (counts back to 0, settings to defaults, I line).
"""
import math
import os
import pty
import select
import signal
import sys
import termios
import time
import tty

ACCEL = 20000.0  # counts/s^2, as in the firmware
MAX_TARGET = 7000.0


def cs(body):
    x = 0
    for c in body.encode():
        x ^= c
    return x


def send(fd, body):
    os.write(fd, f'{body}*{cs(body):02X}\n'.encode())


def main():
    link = sys.argv[1] if len(sys.argv) > 1 else '/tmp/rover1_fake'
    master, slave = pty.openpty()
    tty.setraw(slave)
    attrs = termios.tcgetattr(slave)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(slave, termios.TCSANOW, attrs)
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(os.ttyname(slave), link)
    print(f'fake llc on {link} -> {os.ttyname(slave)}', flush=True)

    target = [0.0, 0.0]
    speed = [0.0, 0.0]
    count = [0.0, 0.0]
    mode = 'I'
    watchdog_ms, tel_ms = 300, 20
    last_cmd = time.monotonic()
    last_tel = last_step = time.monotonic()
    buf = b''
    reboot = []
    signal.signal(signal.SIGUSR1, lambda *_: reboot.append(1))
    send(master, 'I,rover1 llc,0.1.0-fake,sim,cpr=3960')

    while True:
        r, _, _ = select.select([master], [], [], 0.002)
        if r:
            buf += os.read(master, 1024)
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = raw.decode(errors='replace').strip()
                if '*' in line:
                    body, hh = line.rsplit('*', 1)
                    if len(hh) != 2 or int(hh, 16) != cs(body):
                        send(master, 'E,checksum')
                        continue
                    line = body
                f = line.replace(',', ' ').split()
                if not f:
                    continue
                c = f[0].upper()
                if c == 'V' and len(f) == 3:
                    target = [max(-MAX_TARGET, min(MAX_TARGET, float(v))) for v in f[1:]]
                    mode, last_cmd = 'V', time.monotonic()
                elif c == 'S':
                    target, speed, mode = [0.0, 0.0], [0.0, 0.0], 'I'
                    send(master, 'A,stopped')
                elif c == 'Z':
                    count = [0.0, 0.0]
                    send(master, 'A,counts zeroed')
                elif c == 'W' and len(f) == 2:
                    watchdog_ms = int(f[1])
                    send(master, f'A,wd={watchdog_ms}')
                elif c == 'T' and len(f) == 2:
                    tel_ms = int(f[1])
                    send(master, f'A,tel={tel_ms}')
                elif c == 'I':
                    send(master, 'I,rover1 llc,0.1.0-fake,sim,cpr=3960')
                else:
                    send(master, 'E,unknown command')
        if reboot:
            reboot.clear()
            target, speed, count, mode = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], 'I'
            watchdog_ms, tel_ms = 300, 20
            send(master, 'I,rover1 llc,0.1.0-fake,sim,cpr=3960')
        now = time.monotonic()
        if mode == 'V' and (now - last_cmd) * 1000 > watchdog_ms:
            target, speed, mode = [0.0, 0.0], [0.0, 0.0], 'W'
        dt, last_step = now - last_step, now
        for i in range(2):
            step = ACCEL * dt
            speed[i] += max(-step, min(step, target[i] - speed[i]))
            count[i] += speed[i] * dt
        if tel_ms and (now - last_tel) * 1000 >= tel_ms:
            last_tel = now
            pct = [s / 68.0 for s in speed]
            send(master, f'T,{int(now * 1000) & 0xffffffff},{math.floor(count[0])},{math.floor(count[1])},'
                         f'{round(speed[0])},{round(speed[1])},{round(pct[0])},{round(pct[1])},{mode}')


if __name__ == '__main__':
    main()
