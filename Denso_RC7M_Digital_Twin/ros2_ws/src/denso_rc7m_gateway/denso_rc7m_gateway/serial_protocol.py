from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

import serial


CR = b"\r"


class DensoSerialError(RuntimeError):
    pass


class DensoSerialTimeout(DensoSerialError):
    pass


class DensoProtocolError(DensoSerialError):
    pass


@dataclass(frozen=True)
class CartesianPosition:
    pose: tuple[float, float, float, float, float, float]
    figure: int


class DensoRc7mSerialClient:
    """Single-owner implementation of the user's RC7M PACScript protocol.

    The controller-side program defines numeric commands:
      0 = joint move
      1 = PTP Cartesian move
      2 = read six joints
      3 = read Cartesian position plus figure
      5 = linear Cartesian move

    Every message is CR terminated. Motion returns R then F.
    """

    def __init__(
        self,
        port: str,
        baud_rate: int = 19200,
        message_timeout_sec: float = 0.5,
        position_timeout_sec: float = 1500.0,
    ) -> None:
        self.port = port
        self.baud_rate = int(baud_rate)
        self.message_timeout_sec = float(message_timeout_sec)
        self.position_timeout_sec = float(position_timeout_sec)
        self._serial: serial.Serial | None = None
        self._lock = threading.RLock()

    @property
    def connected(self) -> bool:
        return bool(self._serial is not None and self._serial.is_open)

    def connect(self) -> None:
        with self._lock:
            if self.connected:
                return
            try:
                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baud_rate,
                    timeout=self.message_timeout_sec,
                    write_timeout=self.message_timeout_sec,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
                time.sleep(0.5)
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except serial.SerialException as exc:
                self._serial = None
                raise DensoSerialError(
                    f"Cannot open {self.port} at {self.baud_rate} baud: {exc}"
                ) from exc

    def close(self) -> None:
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                finally:
                    self._serial = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()

    def _require_serial(self) -> serial.Serial:
        if not self.connected or self._serial is None:
            raise DensoSerialError("Serial client is not connected.")
        return self._serial

    @staticmethod
    def _finite_values(values: Iterable[float], count: int, label: str) -> list[float]:
        result = [float(value) for value in values]
        if len(result) != count:
            raise ValueError(f"{label} requires {count} values, got {len(result)}.")
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{label} contains a non-finite value.")
        return result

    @staticmethod
    def _format(value: float | int) -> str:
        if isinstance(value, int):
            return str(value)
        return f"{float(value):.8g}"

    def _write(self, payload: bytes) -> None:
        device = self._require_serial()
        try:
            device.write(payload)
            device.flush()
        except serial.SerialTimeoutException as exc:
            raise DensoSerialTimeout("Serial write timed out.") from exc
        except serial.SerialException as exc:
            raise DensoSerialError(f"Serial write failed: {exc}") from exc

    def _read_cr_message(self, timeout_sec: float) -> str:
        device = self._require_serial()
        old_timeout = device.timeout
        device.timeout = max(0.01, float(timeout_sec))
        try:
            raw = device.read_until(CR)
        except serial.SerialException as exc:
            raise DensoSerialError(f"Serial read failed: {exc}") from exc
        finally:
            device.timeout = old_timeout
        if not raw.endswith(CR):
            raise DensoSerialTimeout("CR-terminated response was not received.")
        try:
            return raw[:-1].decode("ascii").strip()
        except UnicodeDecodeError as exc:
            raise DensoProtocolError("Controller response is not ASCII.") from exc

    def _wait_for_token(self, expected: str, timeout_sec: float) -> None:
        device = self._require_serial()
        deadline = time.monotonic() + float(timeout_sec)
        ignored = {b"\r", b"\n", b" ", b"\t"}
        old_timeout = device.timeout
        try:
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                device.timeout = min(
                    max(remaining, 0.01), self.message_timeout_sec
                )
                try:
                    token = device.read(1)
                except serial.SerialException as exc:
                    raise DensoSerialError(
                        f"Serial token read failed: {exc}"
                    ) from exc
                if not token or token in ignored:
                    continue
                try:
                    received = token.decode("ascii")
                except UnicodeDecodeError as exc:
                    raise DensoProtocolError(
                        "Received non-ASCII confirmation."
                    ) from exc
                if received != expected:
                    raise DensoProtocolError(
                        f"Expected confirmation {expected!r}, "
                        f"received {received!r}."
                    )
                return
        finally:
            device.timeout = old_timeout
        raise DensoSerialTimeout(
            f"Timed out waiting for controller confirmation {expected!r}."
        )

    def _prepare_transaction(self) -> None:
        device = self._require_serial()
        # This client must be the only serial owner. Clearing stale bytes avoids
        # treating a previous R/F as acknowledgement for a new command.
        device.reset_input_buffer()

    def get_joint_degrees(self) -> list[float]:
        with self._lock:
            self._prepare_transaction()
            self._write(b"2\r")
            response = self._read_cr_message(self.message_timeout_sec)
            try:
                values = [float(token) for token in response.replace(",", " ").split()]
            except ValueError as exc:
                raise DensoProtocolError(
                    f"Joint response contains non-numeric data: {response!r}"
                ) from exc
            return self._finite_values(values, 6, "Joint response")

    def get_position(self) -> CartesianPosition:
        with self._lock:
            self._prepare_transaction()
            self._write(b"3\r")
            response = self._read_cr_message(self.message_timeout_sec)
            try:
                values = [float(token) for token in response.replace(",", " ").split()]
            except ValueError as exc:
                raise DensoProtocolError(
                    f"Position response contains non-numeric data: {response!r}"
                ) from exc
            values = self._finite_values(values, 7, "Position response")
            figure_value = values[6]
            if not float(figure_value).is_integer():
                raise DensoProtocolError(
                    f"Figure must be an integer, got {figure_value}."
                )
            return CartesianPosition(tuple(values[:6]), int(figure_value))

    def _execute_motion(self, command_code: int, fields: Sequence[float | int]) -> None:
        with self._lock:
            self._prepare_transaction()
            payload = (
                f"{command_code}\r"
                + ",".join(self._format(value) for value in fields)
                + "\r"
            ).encode("ascii")
            self._write(payload)
            self._wait_for_token("R", self.message_timeout_sec)
            self._wait_for_token("F", self.position_timeout_sec)

    def move_joint_degrees(
        self,
        joints: Sequence[float],
        speed: int = 15,
        gripper_closed: bool = False,
        figure: int = 5,
    ) -> None:
        values = self._finite_values(joints, 6, "Joint command")
        if not 1 <= int(speed) <= 100:
            raise ValueError("speed must be in range 1..100.")
        self._execute_motion(
            0,
            [*values, int(speed), int(bool(gripper_closed)), int(figure)],
        )

    def move_ptp(
        self,
        pose: Sequence[float],
        speed: int = 15,
        tool: int = 0,
        figure: int = 5,
        fp: int = 95,
        gripper_closed: bool = False,
    ) -> None:
        values = self._finite_values(pose, 6, "PTP pose")
        self._execute_motion(
            1,
            [
                *values,
                int(figure),
                int(speed),
                int(tool),
                int(fp),
                int(bool(gripper_closed)),
            ],
        )

    def move_line(
        self,
        pose: Sequence[float],
        speed: int = 15,
        tool: int = 0,
        figure: int = 5,
        fp: int = 95,
        gripper_closed: bool = False,
    ) -> None:
        values = self._finite_values(pose, 6, "Linear pose")
        self._execute_motion(
            5,
            [
                *values,
                int(figure),
                int(speed),
                int(tool),
                int(fp),
                int(bool(gripper_closed)),
            ],
        )
