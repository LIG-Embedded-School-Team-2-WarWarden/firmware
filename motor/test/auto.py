#!/usr/bin/env python3
import socket
import time

BOARD = ("10.0.0.3", 5001)
SPEED = 200
MOVE_INTERVAL = 3.0  # 실제 120°↔240° 이동 시간에 맞춰 조정

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sequence = 0
tilt_ticks = round(30 * 4096 / 360)


def move(pan_deg):
    global sequence
    sequence += 1

    pan_ticks = max(1366, min(2730, round(pan_deg * 4096 / 360)))
    message = (
        f"MOVE {sequence} {pan_ticks} {tilt_ticks} "
        f"{SPEED} {SPEED}"
    )
    sock.sendto(message.encode("ascii"), BOARD)
    print(f"PAN={pan_deg}°, TILT=30°")


try:
    sock.sendto(b"HELLO", BOARD)
    time.sleep(0.5)

    while True:
        move(120)
        time.sleep(MOVE_INTERVAL)

        move(240)
        time.sleep(MOVE_INTERVAL)

except KeyboardInterrupt:
    print("\n중단")

finally:
    sock.sendto(b"Stop", BOARD)
    sock.close()
