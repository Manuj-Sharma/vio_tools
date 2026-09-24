#!/usr/bin/env python3
"""Live compass/EKF-yaw monitor -- run directly in a terminal (Konsole), Ctrl+C to stop.

Prints EKF fused yaw (ATTITUDE.yaw) plus a compass-direction label, live, at ~5Hz.
Requires /dev/ttyACM0 to be free (nothing else -- fc node, other pymavlink scripts --
connected at the same time).

Usage:
    python3 ~/vio_tools/watch_compass.py
"""
import math
import sys
import time

from pymavlink import mavutil


def heading_label(deg):
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    idx = int(((deg % 360) + 22.5) // 45) % 8
    return dirs[idx]


def main():
    print("connecting to /dev/ttyACM0 ...")
    m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    m.wait_heartbeat(timeout=10)
    print("connected. streaming ATTITUDE at 10Hz. Ctrl+C to stop.\n")

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000, 0, 0, 0, 0, 0)  # 10Hz

    try:
        while True:
            msg = m.recv_match(type='ATTITUDE', blocking=True, timeout=2)
            if msg is None:
                print("... no ATTITUDE message received (link stalled?)")
                continue
            deg = math.degrees(msg.yaw)
            deg_0_360 = deg % 360
            label = heading_label(deg_0_360)
            sys.stdout.write(
                f"\ryaw = {deg:7.2f} deg  (0-360: {deg_0_360:6.2f})  ~{label:2s}   "
            )
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
