#!/usr/bin/env python3
"""Non-mutating RC7M network diagnostics.

This tool performs ICMP ping and TCP connect tests only. It does not send a
b-CAP command, write controller variables, start tasks, or enable the servo.
"""

from __future__ import annotations

import argparse
import json
import platform
import socket
import subprocess
import time
from dataclasses import asdict, dataclass
from typing import List


@dataclass
class PortResult:
    host: str
    port: int
    reachable: bool
    latency_ms: float | None
    error: str


def ping(host: str, timeout_sec: float) -> tuple[bool, str]:
    timeout_arg = str(max(1, int(round(timeout_sec))))
    command = ["ping", "-c", "1", "-W", timeout_arg, host]
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout_sec + 1.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    output = (result.stdout + result.stderr).strip()
    return result.returncode == 0, output


def tcp_probe(host: str, port: int, timeout_sec: float) -> PortResult:
    started = time.perf_counter()
    try:
        with socket.create_connection((host, port), timeout=timeout_sec):
            latency = (time.perf_counter() - started) * 1000.0
            return PortResult(host, port, True, latency, "")
    except OSError as exc:
        return PortResult(host, port, False, None, str(exc))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only network probe for DENSO RC7M."
    )
    parser.add_argument("host", help="RC7M or Windows ORiN gateway IP address")
    parser.add_argument(
        "--ports",
        default="5007",
        help="Comma-separated TCP ports; default is the common b-CAP port 5007",
    )
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--json", action="store_true", dest="json_output")
    args = parser.parse_args()

    try:
        ports: List[int] = [int(value) for value in args.ports.split(",")]
    except ValueError:
        parser.error("--ports must contain integers separated by commas")
    if any(port < 1 or port > 65535 for port in ports):
        parser.error("port must be in range 1..65535")

    ping_ok, ping_output = ping(args.host, args.timeout)
    results = [tcp_probe(args.host, port, args.timeout) for port in ports]
    report = {
        "tool": "denso_rc7m_network_probe",
        "platform": platform.platform(),
        "host": args.host,
        "ping_reachable": ping_ok,
        "ping_output": ping_output,
        "tcp_ports": [asdict(result) for result in results],
        "interpretation": (
            "Network reachability only. This does not prove RC7M ORiN/b-CAP "
            "compatibility and does not authorize robot commands."
        ),
    }

    if args.json_output:
        print(json.dumps(report, indent=2))
    else:
        print(f"Host: {args.host}")
        print(f"Ping: {'PASS' if ping_ok else 'FAIL'}")
        for result in results:
            if result.reachable:
                print(
                    f"TCP {result.port}: OPEN "
                    f"({result.latency_ms:.2f} ms connect time)"
                )
            else:
                print(f"TCP {result.port}: CLOSED/UNREACHABLE ({result.error})")
        print()
        print(report["interpretation"])

    return 0 if ping_ok or any(item.reachable for item in results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
