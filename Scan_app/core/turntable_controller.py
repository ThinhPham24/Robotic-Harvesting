from __future__ import annotations

import importlib.util
import time
from pathlib import Path
from typing import Callable, Optional

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

try:
    from IOControl import IOControl  # type: ignore
except Exception:  # pragma: no cover
    IOControl = None

from .project_io import get_nested


def _load_project_iocontrol():
    """Load the legacy IOControl.py stored beside the Scan_app folder."""
    global IOControl
    if IOControl is not None:
        return IOControl
    module_path = Path(__file__).resolve().parents[2] / "IOControl.py"
    if not module_path.exists():
        return None
    try:
        spec = importlib.util.spec_from_file_location("cucumber_system_iocontrol", module_path)
        if spec is None or spec.loader is None:
            return None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        IOControl = getattr(module, "IOControl", None)
    except Exception:
        IOControl = None
    return IOControl


class TurntableController:
    """Turntable wrapper.

    Preferred path: use your existing IOControl.py because your old scanner code
    already used `turntableRotate(angle, speed, response=True, hold=True)`.

    Fallback path: send a plain serial command: `ROTATE <angle> <speed>\n`.
    Edit `_rotate_with_serial` if your Arduino command is different.
    """

    def __init__(self, cfg: dict, log_fn: Callable[[str], None] = print):
        self.cfg = cfg
        self.log = log_fn
        self.io = None
        self.ser = None

    def connect(self) -> None:
        port = str(get_nested(self.cfg, "turntable.serial_port", "COM3"))
        baudrate = int(get_nested(self.cfg, "turntable.baudrate", 19200))
        io_class = _load_project_iocontrol()
        if io_class is not None:
            self.log(f"[TURNTABLE] Opening {port} at {baudrate} baud with IOControl.")
            self.io = io_class(port=port, baudrate=baudrate)
            self.log(f"[OK] Turntable connected with IOControl on {port}.")
            return
        if serial is None:
            raise RuntimeError("Turntable serial support requires pyserial or IOControl.py.")
        self.log(
            f"[TURNTABLE WARNING] IOControl.py was not found; using the native "
            f"CucumberSystem serial protocol on {port}."
        )
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.3, write_timeout=0.3)
        time.sleep(2.0)
        self.log(f"[OK] Turntable connected with native serial protocol on {port}.")

    def close(self) -> None:
        try:
            if self.io is not None:
                try:
                    self.io.turntableHoldOff(response=False)
                except Exception:
                    pass
                try:
                    if self.io.serial.is_open:
                        self.io.serial.close()
                except Exception:
                    pass
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        finally:
            self.io = None
            self.ser = None

    def rotate_relative(
        self,
        angle_deg: float,
        speed: Optional[int] = None,
        acceleration: Optional[int] = None,
        wait_after: bool = True,
    ) -> None:
        speed = int(speed if speed is not None else get_nested(self.cfg, "turntable.rotate_speed", 20))
        acceleration = int(
            acceleration
            if acceleration is not None
            else get_nested(self.cfg, "turntable.rotate_acceleration", 75)
        )
        acceleration = max(1, min(acceleration, 100))
        angle_deg = float(angle_deg)
        if abs(angle_deg) < 1e-9:
            return
        backend = "IOControl" if self.io is not None else "native serial"
        self.log(
            f"[TURNTABLE COMMAND] Rotate {angle_deg:.3f} deg at speed {speed}, "
            f"acceleration {acceleration} "
            f"using {backend}."
        )
        started = time.monotonic()
        if self.io is not None:
            self._rotate_with_iocontrol(angle_deg, speed, acceleration)
        elif self.ser is not None:
            self._rotate_with_serial(angle_deg, speed, acceleration)
        else:
            raise RuntimeError("Turntable is not connected.")
        self.log(
            f"[TURNTABLE DONE] Rotation {angle_deg:.3f} deg completed in "
            f"{time.monotonic() - started:.2f} s."
        )
        if wait_after:
            # The IOControl command should already block until rotation finishes.
            # This short sleep gives the controller time to settle before the next command.
            time.sleep(0.05)

    def _rotate_with_iocontrol(self, angle_deg: float, speed: int, acceleration: int) -> None:
        try:
            try:
                self.io.turntableHoldOn(response=False)
            except Exception:
                pass
            status = self.io.turntableRotate(
                angle=float(angle_deg),
                speed=int(speed),
                response=True,
                hold=True,
                accel=int(acceleration),
            )
            if status != 1:
                raise RuntimeError(f"turntableRotate returned status={status}")
        except Exception as exc:
            raise RuntimeError(f"Turntable rotation failed: {exc}") from exc

    def _rotate_with_serial(self, angle_deg: float, speed: int, acceleration: int) -> None:
        direction = 0 if angle_deg < 0 else 1
        angle_steps = abs(float(angle_deg)) * 25.0
        speed = max(1, min(int(speed), 100))
        motor_delay = int((speed - 1) * ((400 - 500) / (100 - 1)) + 500)
        cmd = (
            f"p,{direction},{angle_steps},{motor_delay},1,{int(acceleration)},1\n"
        ).encode("ascii")
        self.ser.write(cmd)
        self.ser.flush()
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            response = self.ser.read_until(b"\n").decode("utf-8", errors="ignore").strip()
            if response == "F":
                return
            if response:
                self.log(f"[TURNTABLE RESPONSE] {response}")
        raise TimeoutError("Turntable did not return completion response 'F' within 60 seconds.")

    def release_backlash_or_shaking(self) -> None:
        extra = float(get_nested(self.cfg, "turntable.final_extra_rotation_deg", 5.0))
        if abs(extra) > 1e-9:
            self.log(f"[TURNTABLE] Final compensation rotation: {extra:.3f} deg. Not used for stitching.")
            self.rotate_relative(extra, wait_after=True)
