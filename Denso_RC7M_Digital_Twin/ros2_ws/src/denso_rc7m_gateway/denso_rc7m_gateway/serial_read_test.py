from __future__ import annotations

import argparse
import json

from .serial_protocol import DensoRc7mSerialClient, DensoSerialError


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only RC7M serial protocol test."
    )
    parser.add_argument("--port", required=True, help="Example: /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=19200)
    parser.add_argument("--timeout", type=float, default=0.5)
    args = parser.parse_args()

    try:
        with DensoRc7mSerialClient(
            port=args.port,
            baud_rate=args.baud,
            message_timeout_sec=args.timeout,
        ) as client:
            joints = client.get_joint_degrees()
            position = client.get_position()
    except (DensoSerialError, ValueError) as exc:
        print(json.dumps({"success": False, "error": str(exc)}, indent=2))
        return 2

    print(
        json.dumps(
            {
                "success": True,
                "port": args.port,
                "baud": args.baud,
                "joint_position_deg": joints,
                "cartesian_position": list(position.pose),
                "figure": position.figure,
                "motion_commands_sent": False,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
