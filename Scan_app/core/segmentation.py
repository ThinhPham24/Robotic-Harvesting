from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from .project_io import get_nested


class ObjectSegmenter:
    """Optional Ultralytics YOLO instance-segmentation backend."""

    def __init__(self, cfg: dict, log_fn: Callable[[str], None] = print):
        self.cfg = cfg
        self.log = log_fn
        self.enabled = bool(get_nested(cfg, "segmentation.enabled", False))
        self.model = None
        self.device = "cpu"
        self.use_half = False
        self.model_path = str(get_nested(cfg, "segmentation.model_path", "") or "").strip()
        if not self.enabled:
            return
        if not self.model_path:
            raise ValueError("Segmentation is enabled but segmentation.model_path is empty.")
        if Path(self.model_path).suffix.lower() != ".pt":
            raise ValueError("Ultralytics segmentation requires a .pt model file.")
        if not Path(self.model_path).is_file():
            raise FileNotFoundError(f"YOLO segmentation model not found: {self.model_path}")

        try:
            import torch
        except ImportError as exc:
            raise RuntimeError(
                "PyTorch is not installed in the Python environment running Scan_app.\n"
                f"Python: {sys.executable}\n\n"
                "For this Windows NVIDIA system, install CUDA-enabled PyTorch into "
                "that exact interpreter, then install Ultralytics:\n"
                f'"{sys.executable}" -m pip install torch torchvision '
                "--index-url https://download.pytorch.org/whl/cu126\n"
                f'"{sys.executable}" -m pip install ultralytics'
            ) from exc
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "Ultralytics is not installed in the Python environment running "
                f"Scan_app.\nPython: {sys.executable}\n\nInstall with:\n"
                f'"{sys.executable}" -m pip install ultralytics'
            ) from exc

        configured_device = str(get_nested(
            cfg,
            "segmentation.device",
            "auto",
        ) or "auto").strip().lower()
        if configured_device == "auto":
            self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        else:
            self.device = configured_device
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                f"Segmentation device is '{self.device}', but PyTorch reports that "
                "CUDA is unavailable. Use device=cpu or install CUDA-enabled PyTorch."
            )

        self.use_half = bool(get_nested(
            cfg,
            "segmentation.half_precision",
            True,
        )) and self.device.startswith("cuda")
        self.model = YOLO(self.model_path, task="segment")
        self.log(
            f"[SEGMENTATION] Loaded Ultralytics model: {self.model_path} | "
            f"device={self.device} | FP16={self.use_half}"
        )

    def _full_image_fallback(
        self,
        image_bgr: np.ndarray,
        reason: str,
    ) -> np.ndarray:
        """Keep all valid stereo points when YOLO misses the object."""
        self.log(
            f"[SEGMENTATION WARNING] {reason} "
            "Using the full image for stereo reconstruction. The point cloud "
            "will still be cleaned by depth, statistical-outlier, and "
            "turntable-radius filtering."
        )
        return np.ones(image_bgr.shape[:2], dtype=bool)

    @staticmethod
    def cuda_diagnostics() -> dict:
        info = {
            "python": sys.executable,
            "nvidia_smi": False,
            "gpu": "",
            "torch_installed": False,
            "torch_version": "",
            "torch_cuda_build": "",
            "cuda_available": False,
            "cuda_device_count": 0,
            "cuda_device_name": "",
            "ultralytics_installed": False,
            "ultralytics_version": "",
        }
        try:
            output = subprocess.check_output(
                [
                    "nvidia-smi",
                    "--query-gpu=name,driver_version",
                    "--format=csv,noheader",
                ],
                text=True,
                timeout=5,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            ).strip()
            info["nvidia_smi"] = bool(output)
            info["gpu"] = output
        except Exception:
            pass
        try:
            import torch
            info["torch_installed"] = True
            info["torch_version"] = str(torch.__version__)
            info["torch_cuda_build"] = str(torch.version.cuda or "CPU-only")
            info["cuda_available"] = bool(torch.cuda.is_available())
            info["cuda_device_count"] = int(torch.cuda.device_count())
            if torch.cuda.is_available():
                info["cuda_device_name"] = str(torch.cuda.get_device_name(0))
        except Exception:
            pass
        try:
            import ultralytics
            info["ultralytics_installed"] = True
            info["ultralytics_version"] = str(ultralytics.__version__)
        except Exception:
            pass
        return info

    def predict_mask(self, image_bgr: np.ndarray) -> np.ndarray:
        if not self.enabled or self.model is None:
            return np.ones(image_bgr.shape[:2], dtype=bool)

        image_size = int(get_nested(self.cfg, "segmentation.image_size", 640))
        confidence = float(get_nested(self.cfg, "segmentation.confidence_threshold", 0.25))
        iou = float(get_nested(self.cfg, "segmentation.iou_threshold", 0.7))
        max_detections = int(get_nested(self.cfg, "segmentation.max_detections", 20))
        class_id = int(get_nested(self.cfg, "segmentation.object_class_id", -1))
        classes = None if class_id < 0 else [class_id]

        results = self.model.predict(
            source=image_bgr,
            imgsz=image_size,
            conf=confidence,
            iou=iou,
            classes=classes,
            device=self.device,
            half=self.use_half,
            max_det=max_detections,
            retina_masks=True,
            verbose=False,
        )
        if not results:
            return self._full_image_fallback(
                image_bgr,
                "YOLO returned no result object.",
            )
        result = results[0]
        if result.masks is None or result.masks.data is None:
            return self._full_image_fallback(
                image_bgr,
                "YOLO found no segmentation masks. Check object visibility, "
                "class ID, confidence threshold, and that the .pt model is a "
                "segmentation model rather than detection-only.",
            )

        masks = result.masks.data.detach().float().cpu().numpy()
        if masks.ndim == 2:
            masks = masks[None, ...]
        if masks.size == 0 or masks.shape[0] == 0:
            return self._full_image_fallback(
                image_bgr,
                "YOLO returned an empty segmentation-mask array.",
            )
        if masks.shape[1:] != image_bgr.shape[:2]:
            masks = np.stack([
                cv2.resize(
                    mask,
                    (image_bgr.shape[1], image_bgr.shape[0]),
                    interpolation=cv2.INTER_LINEAR,
                )
                for mask in masks
            ])

        mask_threshold = float(get_nested(
            self.cfg,
            "segmentation.mask_threshold",
            0.5,
        ))
        binary_masks = masks >= mask_threshold
        if bool(get_nested(self.cfg, "segmentation.keep_largest_instance_only", False)):
            areas = binary_masks.reshape(len(binary_masks), -1).sum(axis=1)
            object_mask = binary_masks[int(np.argmax(areas))]
        else:
            object_mask = np.any(binary_masks, axis=0)

        close_kernel = max(0, int(get_nested(
            self.cfg,
            "segmentation.morphology_close_kernel",
            5,
        )))
        if close_kernel > 1:
            if close_kernel % 2 == 0:
                close_kernel += 1
            kernel = np.ones((close_kernel, close_kernel), dtype=np.uint8)
            object_mask = cv2.morphologyEx(
                object_mask.astype(np.uint8),
                cv2.MORPH_CLOSE,
                kernel,
            ).astype(bool)

        minimum_fraction = float(get_nested(
            self.cfg,
            "segmentation.minimum_mask_fraction",
            0.001,
        ))
        fraction = float(object_mask.mean())
        if fraction < minimum_fraction:
            return self._full_image_fallback(
                image_bgr,
                f"YOLO mask is too small ({fraction * 100.0:.3f}% of image). "
                "Check class ID, confidence, mask threshold, and model training.",
            )
        self.log(
            f"[SEGMENTATION] YOLO kept {len(binary_masks)} instance mask(s), "
            f"{int(object_mask.sum()):,}/{object_mask.size:,} pixels "
            f"({fraction * 100.0:.2f}%)."
        )
        return object_mask
