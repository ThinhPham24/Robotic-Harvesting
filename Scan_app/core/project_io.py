from __future__ import annotations

import csv
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None


def app_root() -> Path:
    return Path(__file__).resolve().parents[1]


def timestamp() -> str:
    return datetime.now().strftime("%Y_%m_%d_%H%M%S_%f")


def timestamp_for_scan_folder() -> str:
    """Human-readable scan folder timestamp: year_month_day_time."""
    return datetime.now().strftime("%Y_%m_%d_%H%M%S")


def ensure_dir(path: os.PathLike | str) -> Path:
    p = Path(path)
    p.mkdir(parents=True, exist_ok=True)
    return p


def resolve_path(path_value: str, base: Optional[Path] = None) -> Path:
    base = base or app_root()
    p = Path(path_value)
    if not p.is_absolute():
        p = base / p
    return p.resolve()


def load_yaml(path: os.PathLike | str) -> Dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML is required. Install with: pip install PyYAML")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data


def save_yaml(path: os.PathLike | str, data: Dict[str, Any]) -> None:
    if yaml is None:
        raise RuntimeError("PyYAML is required. Install with: pip install PyYAML")
    ensure_dir(Path(path).parent)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def load_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    root = app_root()
    config_file = resolve_path(config_path or "configs/default_config.yaml", root)
    cfg = load_yaml(config_file)
    cfg["_app_root"] = str(root)
    cfg["_config_file"] = str(config_file)

    # Normalize key paths to absolute paths so every module saves to the same place.
    paths = cfg.setdefault("paths", {})
    for key in [
        "output_root",
        "calibration_file",
        "turntable_calibration_file",
        "calibration_capture_dir",
        "turntable_placement_guide_image",
    ]:
        if key in paths:
            paths[key] = str(resolve_path(str(paths[key]), root))
    ensure_dir(paths.get("output_root", str(root / "data" / "scans")))
    ensure_dir(Path(paths.get("calibration_file", str(root / "configs" / "stereoMap.yml"))).parent)
    ensure_dir(Path(paths.get("turntable_calibration_file", str(root / "configs" / "turntable_axis_calibration.json"))).parent)
    ensure_dir(paths.get("calibration_capture_dir", str(root / "data" / "stereo_calibration_images")))
    return cfg


def get_nested(cfg: Dict[str, Any], path: str, default: Any = None) -> Any:
    node: Any = cfg
    for part in path.split("."):
        if isinstance(node, dict) and part in node:
            node = node[part]
        else:
            return default
    return node


def set_nested(cfg: Dict[str, Any], path: str, value: Any) -> None:
    node = cfg
    parts = path.split(".")
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value


def sanitize_folder_name(text: str) -> str:
    text = str(text or "").strip()
    if not text:
        return "Object"
    safe = []
    for ch in text:
        if ch.isalnum() or ch in ("-", "_", "."):
            safe.append(ch)
        elif ch.isspace():
            safe.append("_")
    out = "".join(safe).strip("._-")
    return out[:80] if out else "Object"


def get_next_object_id(output_root: os.PathLike | str) -> int:
    root = ensure_dir(output_root)
    ids: List[int] = []
    for child in root.iterdir():
        if not child.is_dir():
            continue
        # Backward compatibility with older folders: Object_0001_name
        if child.name.startswith("Object_"):
            try:
                ids.append(int(child.name.split("_", 2)[1]))
            except Exception:
                pass
        # New folders store the ID in object_info.json, not in the folder name.
        info_path = child / "object_info.json"
        if info_path.exists():
            try:
                info = read_json(info_path)
                ids.append(int(info.get("object_index", 0)))
            except Exception:
                pass
    return 1 if not ids else max(ids) + 1


def _safe_weight_for_folder(weight: Any, unit: str = "") -> str:
    text = str(weight if weight is not None else "").strip()
    if not text:
        text = "unknown"
    unit_text = str(unit or "").strip()
    if unit_text:
        text = f"{text}_{unit_text}"
    return sanitize_folder_name(text)


def create_scan_folder(
    cfg: Dict[str, Any],
    object_name: str = "",
    weight: Any = "",
    weight_unit: str = "",
    created_at_folder: str | None = None,
) -> Path:
    """Create one scan folder for one object.

    The object name is fixed to the timestamp generated at scan start:
        YYYY_MM_DD_HHMMSS

    Folder format:
        YYYY_MM_DD_HHMMSS_weight_<weight>_<unit>

    Example:
        2026_06_22_145901_weight_125_g
    """
    output_root = ensure_dir(get_nested(cfg, "paths.output_root"))
    safe_weight = _safe_weight_for_folder(weight, weight_unit)
    stamp = created_at_folder or timestamp_for_scan_folder()
    base_name = f"{stamp}_weight_{safe_weight}"
    scan_dir = Path(output_root) / base_name
    suffix = 1
    while scan_dir.exists():
        scan_dir = Path(output_root) / f"{base_name}_v{suffix}"
        suffix += 1
    for sub in ["raw_stereo", "rectified", "disparity", "pointclouds", "registration", "debug"]:
        ensure_dir(scan_dir / sub)
    return scan_dir


def write_json(path: os.PathLike | str, data: Any) -> None:
    ensure_dir(Path(path).parent)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def read_json(path: os.PathLike | str) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def append_csv_row(csv_path: os.PathLike | str, row: Dict[str, Any], fieldnames: Iterable[str]) -> None:
    csv_path = Path(csv_path)
    ensure_dir(csv_path.parent)
    exists = csv_path.exists()
    fieldnames = list(fieldnames)
    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not exists:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in fieldnames})
