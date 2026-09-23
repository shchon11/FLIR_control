# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 Humble colcon workspace for FLIR/Teledyne Spinnaker (Blackfly S GigE) cameras. It covers camera streaming, intrinsic/extrinsic calibration, undistortion, ROS 2 bag recording, and image-only nuScenes export. The rig is 8 GigE cameras on a dedicated 10 GbE network. Runtime docs (README.md and the per-package READMEs) are written in Korean; keep that convention when editing them.

## Build and run

```bash
# Always source this first — it sets up ROS 2 Humble, SPINNAKER_ROOT (if /opt/spinnaker exists),
# install/setup, and FLIR_ROS_WS. Source it, do not execute it.
source scripts/setup_flir_env.bash

# Build everything (or a single package)
colcon build --symlink-install
colcon build --symlink-install --packages-select flir_spinnaker_camera
source install/setup.bash   # re-source after every build

# Run
ros2 launch flir_spinnaker_camera flir_camera.launch.py            # single camera
ros2 launch flir_spinnaker_camera multicam.launch.py               # 8-camera rig
ros2 launch flir_spinnaker_camera all_sensors.launch.py            # 8 cameras + Ouster lidar
ros2 launch flir_camera_calibration calibration.launch.py          # intrinsic calibration UI
ros2 launch flir_camera_undistort_viewer undistort_viewer.launch.py
```

There is no test suite, no linter config, and no CI. `colcon build` compiling cleanly is the only build gate. The C++ builds with `-Wall -Wextra -Wpedantic`.

Requires the Spinnaker SDK 4.x installed separately (not via rosdep/pip). The optional GPU JPEG path needs CUDA *libraries* only (no nvcc/sudo): NVIDIA's pip wheels unpacked into the gitignored `third_party/cuda12` (`pip3 install --no-deps --target third_party/cuda12 nvidia-cuda-runtime-cu12 nvidia-nvjpeg-cu12 nvidia-npp-cu12 nvidia-cuda-nvcc-cu12`); without them the node builds CPU-only. CMake prints `GPU JPEG encoder: ON/OFF`. If it is not at `/opt/spinnaker`, `export SPINNAKER_ROOT=/path/to/spinnaker` before building.

## Two kinds of executables

**ROS 2 C++ nodes** (built by colcon, run via `ros2 launch`/`ros2 run`) — live in `src/`.

**Standalone Python scripts** in `scripts/` — run directly with `python3 scripts/foo.py`, NOT via `ros2 run`. They still need `source install/setup.bash` because they import `rclpy`/`sensor_msgs` for message deserialization, but they are not ROS nodes. `bag_to_nuscenes.py` in particular reads bag `.db3` files with raw `sqlite3` + `deserialize_message`, so it does not spin up a ROS graph.

## Architecture

Three packages, data flows left to right:

```
flir_spinnaker_camera ──> flir_camera_undistort_viewer
   │  (image_rgb/compressed, camera_info, image_raw/metadata, image_raw)
   └──> flir_camera_calibration (produces the YAMLs the camera node reads back)
```

### `src/flir_spinnaker_camera` — the core package, three binaries
- **`flir_spinnaker_camera_node`** (`flir_spinnaker_camera_node.cpp`, one big `FlirSpinnakerCameraNode` class, ~3500 lines): opens the camera, acquires frames, and publishes `image_raw`, `camera_info`, `image_raw/metadata`, `image_rgb/compressed`. Also owns ForceIP assignment, GPIO hardware trigger setup, and PTP scheduled-action trigger setup — all done against the Spinnaker node map before/around `Init()`.
- **`flir_camera_extrinsics_tf_node`**: reads `calibration/flir_camera_extrinsics.yaml` and publishes the rig transforms (`flir_rig_frame -> <camera>_optical_frame`) to `/tf_static`. Launched by `multicam.launch.py` by default.
- **`flir_multicam_inventory_tool`**: probes connected Spinnaker cameras and upserts them into `multicam_cameras.yaml` (used by `auto_update_cameras_file:=true`).

### `src/flir_camera_calibration` — chessboard calibration (OpenCV windows, key-driven)
- **`flir_camera_calibration_node`**: intrinsics → `calibration/flir_camera_info.yaml`.
- **`flir_camera_extrinsic_calibration_node`**: pairwise multi-camera extrinsics composed into a rig graph → `calibration/flir_camera_extrinsics.yaml`.
- UI keys (both): `space` capture sample, `c` calibrate + save, `r` reset, `q`/`Esc` quit.

### `src/flir_camera_undistort_viewer`
- **`flir_camera_undistort_viewer_node`**: subscribes `image_rgb/compressed` + `camera_info`, builds/caches an undistort map from `K/D/R/P`, republishes `image_rgb/undistorted/compressed` (re-encodes JPEG→JPEG, PNG→PNG).

## Key facts that cross files

- **`src/flir_spinnaker_camera/config/multicam_cameras.yaml` is the single source of truth for the rig.** It maps each camera by **serial** to a position-based **namespace** (`camera_center`, `camera_front_right`, …), `frame_id`, `force_ip_address`, `mac_address`, and per-camera sync roles. The multicam launch files in *all three* packages read this same file, so adding/replacing a camera means editing this YAML (serials change with hardware). Do not hardcode the 8-camera list elsewhere — the README/docs tables are copies for humans; this YAML wins.

- **Thermal cameras (FLIR A70) have their own inventory and params:** `config/multicam_thermal_cameras.yaml` + `config/thermal_camera.yaml`, namespaces `thermalN`, IPs from `192.168.1.11`. `multicam.launch.py` launches them alongside the Blackflies (`enable_thermal_cameras`/`enable_visible_cameras`). The two inventory updates exclude each other by model (`thermal_model_patterns`) and serial, so an A70 never lands in `multicam_cameras.yaml` with a PTP action role. A70 has no FrameStart trigger, no Action command, and its PTP (`PtpEnable`/`PtpStatus`, no `GevIEEE1588*`) did not reach Slave against ptp4l, so it free-runs with host-arrival stamps. Output is `image_raw` mono16 where value × 0.01 = Kelvin. It sends frames in bunches (host arrival gaps 2–140 ms at a steady 33 ms capture), so its `buffer_handling_mode` is `OldestFirst`: `NewestOnly` silently dropped 4–22% of A70 frames.

- **Startup `camera.*` overrides are read back before streaming** (`VerifyControlOverridesTookEffect`, called after PTP / trigger / action / chunk setup, before `BeginAcquisition`).
  - Why: overrides of one type are written in name order, and paired GenICam registers recompute each other. On 2026-09-22, `DeviceLinkThroughputLimit` 75 MB/s went in before `GevSCPD` 0, and 13 of 14 cameras ended at 125 MB/s. Simultaneous PTP-action bursts then overflowed the switch.
  - What it does: re-applies drifted values once, with the link limit last. Then it logs ERROR `… not in effect after startup …`, which DM_clipGUI turns into an orange row.
  - Tolerance: camera rounding (increment, or 0.5 %) counts as a match.
  - Skipped: nodes the startup logic sets itself (`Trigger*`, `Line*`, `GevIEEE1588*`, `Ptp*`, `Action*`, `Chunk*`, …).
  - Don't set `GevSCPD` together with `DeviceLinkThroughputLimit`.
- **Two mutually-exclusive frame-sync mechanisms**, both selected per-camera in `multicam_cameras.yaml`:
  - GPIO hardware trigger: `hardware_trigger_role: master|slave|none` (needs the physical trigger cable).
  - PTP scheduled action: `ptp_action_role: sender|receiver|none`. Exactly one `sender`. The PTP grandmaster is the **OS `ptp4l` (linuxptp)**, not the camera node — `multicam.launch.py` can start it via `ptp_master_interface:=<nic>`. Common per-run tunables live under `ptp.*`/`ptp_action.*` in `flir_camera.yaml`.

- **Calibration is a round trip.** `flir_camera_calibration` writes `calibration/*.yaml`; the camera node reads intrinsics back via `camera_info.yaml_path` and the TF node reads extrinsics. The `calibration/` YAMLs are committed; `calibration/captures/`, `bags/`, and `nuscenes_export/` are gitignored.

- **Debayering runs on the host and is the tightest CPU budget** (8 cams × 30 Hz × 1920×1200). `color_processing: ipp` is a deliberate choice — `hq_linear` overran the per-frame budget and silently dropped frames under `NewestOnly`. See the long comment in `flir_camera.yaml`; do not casually change `color_processing`, `buffer_handling_mode`, or `pixel_format`. Since 2026-09-19 `rgb_encoder: gpu` (default in `flir_camera.yaml`) moves demosaic + JPEG to the GPU (`src/gpu_jpeg_encoder.cpp`, NPP + nvJPEG; ~5x less CPU per camera node), so `color_processing` only matters on the CPU fallback path (`gpu_demosaic: false`, or a build without the CUDA libraries).

- **Timestamps:** `header.stamp` is host *arrival* time by default (`use_camera_timestamp_in_header: false`); the true device/PTP timestamp survives only in `image_raw/metadata.camera_timestamp_ns`.

- **The Ouster lidar is not built here.** `ouster_ros` lives in the ROS underlay (`/opt/ros/humble`) and stays visible after `source scripts/setup_flir_env.bash`, so no colcon change is needed to use it. `flir_spinnaker_camera/launch/all_sensors.launch.py` is a pure composition layer over `multicam.launch.py` + the vendor `driver.launch.py`; it must not fork camera defaults. Lidar params are `flir_spinnaker_camera/config/lidar_driver_params.yaml` (copied in from `~/lidar_ws` so nothing depends on a path outside the workspace). The lidar sits on `eno1`/169.254.x, the cameras on `enp3s0f1`/192.168.1.x — separate NICs, no bandwidth contention — and `ptp4l` runs grandmaster-only (no `-s`, no `phc2sys`) so it never disciplines the system clock that both sensors' stamps are read from.

- **A dead lidar kills the cameras.** ouster's `driver.launch.py` emits `launch.events.Shutdown` when it cannot reach the sensor, and that event is global to the launch service — it cannot be scoped to one `IncludeLaunchDescription`. `all_sensors.launch.py` therefore TCP-probes the sensor *before* returning any actions. Keep that probe if you touch the file.

- **`ouster_ros` writes `<sensor-ip>-metadata.json` into the CWD** when its `metadata` param is empty, so launching from the repo root drops one there; `*-metadata.json` is gitignored. The same data rides on `/ouster/metadata`, which is why `scripts/all_sensors_bagging.sh` records that topic — without it the recorded `lidar_packets` cannot be reconstructed into points on replay.

- **Topic naming in nuScenes export:** the `camera_` namespace prefix is dropped and uppercased (`camera_front_right` → `CAM_FRONT_RIGHT`). `bag_to_nuscenes.py` prefers an `.../undistorted/compressed` topic if present, otherwise undistorts on the fly using `CameraInfo`.

## Spinnaker/CMake linking gotcha (already solved — don't undo it)

`flir_spinnaker_camera/CMakeLists.txt` deliberately (1) runs a `NO_DEFAULT_PATH` find pass so `SPINNAKER_ROOT` wins over any ROS-vendored `libSpinnaker` on `CMAKE_PREFIX_PATH`, and (2) sets `BUILD_RPATH`/`INSTALL_RPATH` with `-Wl,--disable-new-dtags` (emits `DT_RPATH`, searched ahead of `LD_LIBRARY_PATH`). If headers and the runtime `.so` come from different SDK versions, Spinnaker fails at runtime with `System instance has not been initialized [-1002]`. Preserve both mechanisms when touching that file.

## Network / NIC setup

The camera network is isolated from the internet (see `docs/network_layout.md` for the measured layout — camera NIC, host IP `192.168.1.100/24`, cameras `.1`–`.8`). The camera NIC runs jumbo frames (MTU 9000, set in the NetworkManager profile `FLIR-LAN`, not by the setup script) and `camera.GevSCPSPacketSize` is 9000 to match — revert both together. Host RX tuning and NIC/PTP capabilities are applied by `scripts/setup_camera_nic.bash` and persisted across reboots by `scripts/flir-camera-nic.service`. The camera NIC host IP must not be a subnet network/broadcast address (e.g. use `.10`/`.100`, not `.0`).
