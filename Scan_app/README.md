# Stereo Turntable Scanner App

This is a modular Python/PyQt5 stereo turntable scanner application. It follows the requested structure and now includes three operator workflows:

1. **Stereo Vision Calibration** — capture chessboard stereo pairs and generate `stereoMap.yml`.
2. **Turntable Calibration** — use a 13 x 8 ChArUco board to estimate turntable center, rotation axis, and camera-to-turntable distance.
3. **Scan** — stable step-and-capture scanning, stereo reconstruction, calibrated-axis stitching, post-processing, and PLY export.

## Project structure

```text
scanner_app/
  main.py
  gui/
    main_window.py
    widgets.py
  core/
    camera_manager.py
    turntable_controller.py
    calibration.py
    stereo_reconstruction.py
    stitching.py
    postprocessing.py
    mesh_generation.py
    project_io.py
  configs/
    default_config.yaml
  data/
    scans/
  requirements.txt
  README.md
```

## Install

```bash
pip install -r requirements.txt
```

ChArUco requires `opencv-contrib-python`, not only `opencv-python`.

## Run

```bash
python main.py
```

## Recommended operation order

### 1. Stereo Vision Calibration tab

Use this before any turntable calibration or scan.

1. Press **Connect**. The button changes to **Disconnect** and live preview starts automatically.
2. Put the stereo chessboard in different positions, angles, and distances.
3. Press **Save Pair [Space]** for each good pose. The Space key also saves one pair.
4. Capture enough valid left/right pairs.
5. Press **Calibration**.
6. The app saves `scanner_app/configs/stereoMap.yml` by default.
7. Press **SGBM 3D** to load the saved calibration and generate one test point cloud.

The saved calibration images use the same naming format as your original code:

```text
calib_SL_*.png
calib_SR_*.png
```

### 2. Turntable Calibration tab

Use this after `stereoMap.yml` exists.

By default, live scanning requires a successful Turntable Calibration once
after every application launch. An older JSON is not accepted as proof that
the current camera, turntable, and scene geometry are unchanged. Pressing
**Start Scan** before session calibration shows a notice and switches directly
to this tab.

1. Mount the 13 x 8 ChArUco board vertically on a stiff backing plate, offset from the rotation axis.
2. Clamp it at two rigid points. Flexible wire supports allow post-stop motion and should not be used.
3. Before starting, place the board near one edge of camera visibility. The default one-sided-board sweep is 90 degrees.
4. Keep the board center about 80–150 mm from the rotation axis.
5. Make sure the board is visible to both stereo cameras.
6. Press **Run Turntable Calibration**.
7. The app waits for the board to stop moving, captures the configured viewpoints, robustly fits the turntable circle, and saves:

Measure one complete square edge on the printed board with calipers. The current
configuration uses 10 mm squares and 8 mm markers. Pose translation scales
directly with these physical dimensions.

```text
scanner_app/configs/turntable_axis_calibration.json
```

The JSON contains:

```json
{
  "turntable_center_m": [0, 0, 0],
  "turntable_axis_unit": [0, 1, 0],
  "turntable_radius_m": 0.0,
  "camera_to_turntable_distance_m": 0.0,
  "rms_circle_fit_error_m": 0.0
}
```

A placement guide image is displayed inside the Turntable Calibration tab.

### 3. Scan tab

The default scan sequence is stable capture after rotation:

```text
rotate to next viewpoint
wait until rotation finishes
wait extra stabilization time
capture synchronized left/right image pair
save metadata
```

After the last scan viewpoint, the turntable rotates by `final_extra_rotation_deg`, default 5 degrees. This motion is only for shaking/backlash compensation and is not used as a scan viewpoint.

If the user presses **Start Scan** before `turntable_axis_calibration.json` exists, the app automatically:

1. Stops the scan.
2. Shows a warning window.
3. Switches directly to the **Turntable Calibration** tab.
4. Displays the board-placement guide.

If `stereoMap.yml` is missing, the app switches to the **Stereo Vision Calibration** tab first.

## Output

Each scan is saved in a timestamp-and-weight folder:

```text
scanner_app/data/scans/YYYY_MM_DD_HHMMSS_weight_<weight>_<unit>/
```

Main outputs:

```text
raw_stereo/                         left/right source images
rectified/                          rectified image pairs
disparity/                          disparity previews
pointclouds/view_000.ply            one cloud per viewpoint
registration/registered_views/      transformed clouds
registration/<scan_folder_name>.ply
registration/registration_report.json
viewpoint_transforms.json
turntable_axis_calibration_used.json
```

### Optional YOLO object segmentation

The Scan tab can load an Ultralytics YOLO segmentation `.pt` model. Enable the
model flag, select the model path, and configure device, class ID, confidence,
IoU, mask threshold, and inference image size. `device: auto` uses `cuda:0`
when CUDA-enabled PyTorch is available; FP16 can be enabled for faster GPU
inference.

The instance masks are combined and applied to each rectified left image before
3D point generation and registration. Set class ID to `-1` to keep every class,
or select one trained class. Enable **Largest only** when exactly one object
should be retained. When segmentation is disabled, the model is not loaded or
applied.

On Windows, use **Check CUDA** in the Scan tab to verify the exact Python
environment used by the app. The NVIDIA driver seeing the GPU is not enough;
that Python environment must also contain a CUDA-enabled PyTorch build and
Ultralytics. The app reports the interpreter path, GPU, PyTorch CUDA build, and
Ultralytics version.

### Completion notices

Successful stereo calibration, turntable calibration, deferred point-cloud
processing, and scanning all display a DONE window. Close it with Enter,
Escape, the OK/Continue button, or the window X.

### Capture now, process later

Disable **Generate after scan** in the Scan tab to finish immediately after
homing, rotation, and stereo image capture. The scan folder still contains
`scan_log.csv`, `raw_stereo/`, the configuration snapshot, and the turntable
calibration used for capture.

Open the **Point Cloud Processing** tab later, select that scan folder, and
press **Generate Point Cloud**. Deferred processing uses the current stereo
reconstruction, YOLO segmentation, turntable-radius crop, ICP, and output
settings. The final PLY is saved under `registration/` with the same name as
the selected scan folder.

Turntable **Speed** and **Acceleration** are configurable in both the Scan and
Turntable Calibration tabs. Acceleration is passed directly to the existing
`IOControl.turntableRotate(..., accel=...)` protocol.

## Turntable control

The app first tries to use your existing `IOControl.py` with:

```python
turntableRotate(angle=float(angle_deg), speed=int(speed), response=True, hold=True)
```

If `IOControl.py` is not found, it falls back to `pyserial` and sends:

```text
ROTATE <angle_deg> <speed>\n
```

Edit `core/turntable_controller.py` if your Arduino command is different.

## Notes

- The old ArUco home/alignment logic can still be added later as a fallback, but the preferred stitching path is calibrated turntable-axis alignment plus optional ICP refinement.
- For accurate stitching, keep continuous/during-rotation capture disabled. The default is stable capture after rotation.


## Parameter editing behavior

Each parameter row in the **Scan**, **Stereo Vision Calibration**, and **Turntable Calibration** tabs follows the original calibration-tab style:

```text
Edit -> change value -> OK
```

Path rows also provide a `...` browse button while editing. This avoids accidental parameter changes during camera preview or scanning.

## Per-object weight dialog and folder naming

When the operator presses **Start Scan**, the app asks only for the current object's weight. The object name is fixed automatically using the scan timestamp.

The object name rule is:

```text
YYYY_MM_DD_HHMMSS
```

The scan folder is named with this rule:

```text
YYYY_MM_DD_HHMMSS_weight_<weight>_<unit>/
```

Example:

```text
2026_06_22_145901_weight_125_g/
```

All stereo pairs for the rotation viewpoints are saved inside that folder under:

```text
raw_stereo/
```

The app also writes:

```text
object_info.json
scan_log.csv
```

These files include the automatically generated timestamp object name, weight, weight unit, output folder, image paths, viewpoint angle, and point-cloud path.

Update note - stereo calibration 2D preview
-------------------------------------------
The Stereo Vision Calibration tab keeps the original behavior from the single-file code:
- Connect starts live left/right stereo preview.
- Save Pair [Space] saves calib_SL_*.png and calib_SR_*.png.
- Calibration loads saved pairs from the image folder.
- During Calibration, each saved pair is displayed in the 2D Images tab with chessboard-corner detection status.
- After Calibration, the 2D Images tab shows the rectification-performance preview.
