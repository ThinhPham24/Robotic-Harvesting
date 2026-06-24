#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import socket
import time


def main() -> None:
    parser = argparse.ArgumentParser(description="Fake read-only RC7M gateway.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=15000, type=int)
    parser.add_argument("--rate", default=20.0, type=float)
    args = parser.parse_args()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    started = time.monotonic()
    period = 1.0 / max(1.0, args.rate)
    print(f"Sending fake RC7M telemetry to {args.host}:{args.port}")
    try:
        while True:
            t = time.monotonic() - started
            joints = [
                20.0 * math.sin(t * 0.25),
                -25.0 + 8.0 * math.sin(t * 0.31),
                45.0 + 12.0 * math.sin(t * 0.21),
                10.0 * math.sin(t * 0.42),
                30.0 + 6.0 * math.sin(t * 0.38),
                15.0 * math.sin(t * 0.55),
            ]
            packet = {
                "protocol": "denso_rc7m.telemetry.v1",
                "sequence": sequence,
                "source_time_unix_ns": time.time_ns(),
                "controller": "RC7M-SIM",
                "robot": "VS-6556E",
                "joint_names": [
                    "joint_1",
                    "joint_2",
                    "joint_3",
                    "joint_4",
                    "joint_5",
                    "joint_6",
                ],
                "joint_position_deg": joints,
                "joint_velocity_deg_s": [0.0] * 6,
                "mode": "MANUAL",
                "servo_on": False,
                "emergency_stop": False,
                "protective_stop": False,
                "alarm_code": 0,
                "alarm_text": "",
                "native_task": "SIMULATED_IDLE",
                "cycle_count": sequence // max(1, int(args.rate * 10)),
            }
            udp.sendto(json.dumps(packet, separators=(",", ":")).encode(), (args.host, args.port))
            sequence += 1
            time.sleep(period)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
