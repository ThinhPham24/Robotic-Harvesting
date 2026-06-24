#!/usr/bin/env python3
"""Display telemetry packets from a future Windows RC7M gateway."""

from __future__ import annotations

import argparse
import json
import socket
import time


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=15000)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind((args.bind, args.port))
    receiver.settimeout(args.timeout)
    print(f"Listening for telemetry on {args.bind}:{args.port}")

    last_sequence = None
    while True:
        try:
            payload, address = receiver.recvfrom(8193)
        except socket.timeout:
            print(f"No packet received for {args.timeout:.1f} seconds.")
            continue
        received = time.time_ns()
        if len(payload) > 8192:
            print(f"Rejected oversized packet from {address}")
            continue
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            print(f"Invalid JSON from {address}: {exc}")
            continue
        sequence = message.get("sequence")
        source_ns = int(message.get("source_time_unix_ns", 0))
        age_ms = (received - source_ns) / 1_000_000.0 if source_ns else -1.0
        gap = (
            sequence - last_sequence - 1
            if isinstance(sequence, int) and isinstance(last_sequence, int)
            else 0
        )
        last_sequence = sequence
        print(
            f"{address[0]} seq={sequence} gap={max(0, gap)} "
            f"age_ms={age_ms:.1f} mode={message.get('mode')} "
            f"joints={message.get('joint_position_deg')}"
        )


if __name__ == "__main__":
    main()
