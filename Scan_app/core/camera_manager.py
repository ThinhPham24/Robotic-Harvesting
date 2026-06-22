from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Tuple

import cv2
import numpy as np

try:
    from pypylon import pylon
    HAS_PYLON = True
except Exception:  # pragma: no cover
    pylon = None
    HAS_PYLON = False

from .project_io import get_nested, timestamp


@dataclass
class StereoPair:
    left: np.ndarray
    right: np.ndarray
    timestamp: str


class StereoCameraManager:
    def __init__(self, cfg: dict, log_fn: Callable[[str], None] = print):
        self.cfg = cfg
        self.log = log_fn
        self.cameras = None
        self.left_idx: Optional[int] = None
        self.right_idx: Optional[int] = None
        self.converter = None

    def connect(self) -> None:
        if self.cameras is not None:
            self.close()
        if not HAS_PYLON:
            raise RuntimeError("pypylon/Basler SDK is not available. Install pypylon or use offline images.")
        if bool(get_nested(self.cfg, "camera.use_camera_emulation", False)):
            import os
            os.environ["PYLON_CAMEMU"] = "9"
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        tl_factory = pylon.TlFactory.GetInstance()
        devices = list(tl_factory.EnumerateDevices())
        if len(devices) < 2:
            raise RuntimeError("Less than two Basler cameras detected.")

        left_serial = str(get_nested(self.cfg, "camera.left_serial", "")).strip()
        right_serial = str(get_nested(self.cfg, "camera.right_serial", "")).strip()
        if left_serial and right_serial and left_serial == right_serial:
            raise ValueError(f"Left and right camera serial numbers are identical: {left_serial}")
        serial_to_device = {d.GetSerialNumber(): d for d in devices}
        detected_serials = list(serial_to_device)
        self.log(f"[CAMERA] Detected Basler serials: {', '.join(detected_serials)}")
        selected = []
        selected_serials = set()
        for serial_no in [left_serial, right_serial]:
            if serial_no in serial_to_device:
                selected.append(serial_to_device[serial_no])
                selected_serials.add(serial_no)
        for d in devices:
            if len(selected) >= 2:
                break
            if d.GetSerialNumber() not in selected_serials:
                selected.append(d)
                selected_serials.add(d.GetSerialNumber())
        if len(selected) < 2:
            raise RuntimeError("Cannot select a stereo camera pair.")

        self.cameras = pylon.InstantCameraArray(len(selected))
        for i, cam in enumerate(self.cameras):
            serial_no = selected[i].GetSerialNumber()
            try:
                cam.Attach(tl_factory.CreateDevice(selected[i]))
            except Exception as exc:
                self.close()
                raise RuntimeError(
                    f"Cannot open Basler camera {serial_no}. It may already be in use by "
                    f"the stereo preview, another app, or another process. Original error: {exc}"
                ) from exc
        self.cameras.Open()

        for idx, cam in enumerate(self.cameras):
            serial_no = cam.DeviceInfo.GetSerialNumber()
            cam.SetCameraContext(idx)
            self.log(f"[CAMERA] index={idx}, serial={serial_no}")
            if serial_no == left_serial:
                self.left_idx = idx
            if serial_no == right_serial:
                self.right_idx = idx
            self._configure_camera(cam)

        if self.left_idx is None:
            self.left_idx = 0
        if self.right_idx is None:
            self.right_idx = 1 if self.left_idx == 0 else 0
        if self.left_idx == self.right_idx:
            raise RuntimeError("Left and right camera indices are identical.")
        self.cameras.StartGrabbing(pylon.GrabStrategy_OneByOne)
        self.log("[OK] Stereo cameras connected with software trigger.")

    def close(self) -> None:
        try:
            if self.cameras is not None:
                if self.cameras.IsGrabbing():
                    self.cameras.StopGrabbing()
                self.cameras.Close()
        except Exception:
            pass
        finally:
            self.cameras = None
            self.left_idx = None
            self.right_idx = None

    def _is_writable(self, node) -> bool:
        try:
            return bool(pylon.IsWritable(node))
        except Exception:
            try:
                return bool(node.IsWritable())
            except Exception:
                return True

    def _set_node(self, cam, node_name: str, value, warn: bool = False):
        try:
            node = getattr(cam, node_name)
            if not self._is_writable(node):
                return node.GetValue()
            node.SetValue(value)
            return node.GetValue()
        except Exception as exc:
            if warn:
                self.log(f"[CAMERA WARNING] Cannot set {node_name}={value}: {exc}")
            return None

    def _configure_camera(self, cam) -> None:
        self._set_node(cam, "ExposureAuto", "Off", warn=True)
        self._set_node(cam, "GainAuto", "Off", warn=True)
        self._set_node(cam, "ExposureTime", float(get_nested(self.cfg, "camera.exposure_time_us", 5000.0)), warn=True)
        self._set_node(cam, "BalanceWhiteAuto", str(get_nested(self.cfg, "camera.balance_white_auto", "Once")), warn=False)
        cam.TriggerSelector.SetValue("FrameStart")
        cam.TriggerMode.SetValue("On")
        cam.TriggerSource.SetValue("Software")

    def _crop(self, img: np.ndarray) -> np.ndarray:
        if not bool(get_nested(self.cfg, "camera.enable_center_crop", False)):
            return img
        h, w = img.shape[:2]
        cw = int(get_nested(self.cfg, "camera.crop_width", 0) or w)
        ch = int(get_nested(self.cfg, "camera.crop_height", 0) or h)
        cx = int(get_nested(self.cfg, "camera.crop_center_x", -1))
        cy = int(get_nested(self.cfg, "camera.crop_center_y", -1))
        if cx < 0:
            cx = w // 2
        if cy < 0:
            cy = h // 2
        cw, ch = min(cw, w), min(ch, h)
        x1 = max(0, min(int(cx - cw / 2), w - cw))
        y1 = max(0, min(int(cy - ch / 2), h - ch))
        return img[y1:y1 + ch, x1:x1 + cw].copy()

    def grab_synchronized_pair(self) -> StereoPair:
        if self.cameras is None or self.left_idx is None or self.right_idx is None:
            raise RuntimeError("Stereo cameras are not connected.")
        cam_l = self.cameras[self.left_idx]
        cam_r = self.cameras[self.right_idx]
        timeout_ms = int(get_nested(self.cfg, "camera.timeout_ms", 5000))
        try:
            cam_l.WaitForFrameTriggerReady(timeout_ms, pylon.TimeoutHandling_ThrowException)
            cam_r.WaitForFrameTriggerReady(timeout_ms, pylon.TimeoutHandling_ThrowException)
        except Exception:
            pass
        ts = timestamp()
        try:
            cam_l.ExecuteSoftwareTrigger()
        except Exception:
            cam_l.TriggerSoftware.Execute()
        try:
            cam_r.ExecuteSoftwareTrigger()
        except Exception:
            cam_r.TriggerSoftware.Execute()

        res_l = res_r = None
        try:
            res_l = cam_l.RetrieveResult(timeout_ms, pylon.TimeoutHandling_ThrowException)
            res_r = cam_r.RetrieveResult(timeout_ms, pylon.TimeoutHandling_ThrowException)
            if not res_l.GrabSucceeded() or not res_r.GrabSucceeded():
                raise RuntimeError("Stereo grab failed.")
            left = self.converter.Convert(res_l).GetArray()
            right = self.converter.Convert(res_r).GetArray()
            return StereoPair(self._crop(left), self._crop(right), ts)
        finally:
            if res_l is not None:
                res_l.Release()
            if res_r is not None:
                res_r.Release()

    @staticmethod
    def load_pair_from_files(left_path: str | Path, right_path: str | Path) -> StereoPair:
        left = cv2.imread(str(left_path), cv2.IMREAD_COLOR)
        right = cv2.imread(str(right_path), cv2.IMREAD_COLOR)
        if left is None or right is None:
            raise FileNotFoundError(f"Cannot load stereo pair: {left_path}, {right_path}")
        return StereoPair(left, right, timestamp())
