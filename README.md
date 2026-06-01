# lab_robot_tracking

ROS 2 Jazzy workspace for ground-truth pose tracking of lab robots using overhead USB cameras and AprilTag fiducial markers.

## Current Status

- A working single-camera prototype exists.
- A multi-camera tracker scaffold now exists for the future 6-camera ceiling system.
- The single-camera prototype is still useful for calibration and bringup, but it
  uses AprilTag TF internally and is deprecated as the architecture boundary for
  the final multi-camera system.
- Intrinsics, extrinsics, and robot tag transforms are still placeholders until
  physically calibrated.

## Packages

| Package | Purpose |
|---|---|
| `lab_tracker_fusion` | Nodes that compute robot poses from tag detections |
| `lab_tracker_bringup` | Launch files that bring up the full pipeline |
| `lab_tracker_eval` | Evaluation and benchmarking tools |

## Prerequisites

```bash
# ROS 2 Jazzy (assumed installed)
sudo apt install ros-jazzy-usb-cam ros-jazzy-apriltag-ros ros-jazzy-image-proc \
                 ros-jazzy-tf-transformations ros-jazzy-tf2-ros
```

Or let rosdep install everything automatically (see Build section).

---

## Milestone 1 – Single-camera AprilTag ground-truth

One USB camera → AprilTag detector → ground-truth pose for **roxi_1**.

Warning:
This is a legacy prototype path. It uses `apriltag_ros` tag TF and should not be
the basis for the final 6-camera architecture.

### Pipeline

```
/dev/video0
  └─ v4l2_camera (namespace cam_1)
       /cam_1/image_raw, /cam_1/camera_info
       └─ image_proc/rectify_node
            /cam_1/image_rect
            └─ apriltag_ros
                 /cam_1/detections
                 TF: cam_1_optical_frame → tag10
                 └─ single_camera_tag_to_robot_pose_node
                      /lab_gt/roxi_1/pose  (PoseWithCovarianceStamped)
                      /lab_gt/roxi_1/odom  (Odometry)
                      TF: lab_map → roxi_1/gt_base_link
```

### Build

```bash
cd ~/path/to/lab_robot_tracking
source /opt/ros/jazzy/setup.bash

# Install ROS dependencies declared in package.xml files
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

### Quick configuration check

Before launching, verify the three config files match your physical setup:

| File | What to fill in |
|---|---|
| `configs/cameras/camera_extrinsics.yaml` | Measured pose of each `cam_X_optical_frame` in `lab_map` |
| `configs/tag_boards/robots.yaml` | Measured pose of tag 10 in `roxi_1` base_link frame |
| `configs/cameras/cam_X_calibration.yaml` | Camera intrinsics from `ros2 run camera_calibration cameracalibrator` |
| `configs/cameras/cam_X_v4l2.yaml` | Per-camera `v4l2_camera` parameters |

All files ship with **placeholder values**; the pipeline will run but pose output will be unreliable until real values are substituted.

### Launch

```bash
source install/setup.bash

# Default: /dev/video0, tag 36h11 ID 10, 1280×720 YUYV
ros2 launch lab_tracker_bringup single_camera_tracking.launch.py

# Override camera device
ros2 launch lab_tracker_bringup single_camera_tracking.launch.py device:=/dev/video1
```

### Multi-camera scaffold launch

```bash
source install/setup.bash

# Launch the 6-camera scaffold
ros2 launch lab_tracker_bringup multi_camera_tracking.launch.py

# Launch tracker + detectors only, without starting cameras
ros2 launch lab_tracker_bringup multi_camera_tracking.launch.py use_cameras:=false
```

### Verify the pipeline is running

```bash
# Check active topics
ros2 topic list | grep -E "cam_1|lab_gt"

# Watch detections (should show tag ID 10 when tag is in view)
ros2 topic echo /cam_1/detections

# Watch ground-truth pose
ros2 topic echo /lab_gt/roxi_1/pose

# Inspect TF tree
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo lab_map roxi_1/gt_base_link
```

### Dry-run (no camera hardware)

To confirm the node loads and reads configs without real hardware:

```bash
source install/setup.bash

# Start just the fusion node with a fake detections publisher
ros2 run lab_tracker_fusion single_camera_tag_to_robot_pose_node \
  --ros-args \
  -p detections_topic:=/cam_1/detections \
  -p tag_id:=10

# In a second terminal, publish a fake detection to trigger the callback
ros2 topic pub --once /cam_1/detections apriltag_msgs/msg/AprilTagDetectionArray \
  "{header: {frame_id: 'cam_1_optical_frame'}, detections: [{id: 10, family: '36h11'}]}"
```

The node will log a TF lookup failure (expected – no TF is being published), confirming the callback fires correctly.

### Camera calibration

Use the dedicated camera-only launch file so the calibration tool gets a clean stream without apriltag or fusion nodes running:

**Terminal 1 — start the camera:**
```bash
source install/setup.bash
ros2 launch lab_tracker_bringup camera_only.launch.py camera_id:=cam_1 device:=/dev/video0
```

Confirm the stream is live:
```bash
ros2 topic hz /cam_1/image_raw      # expect ~30 Hz
ros2 topic echo /cam_1/camera_info  # expect 1280×720
```

**Terminal 2 — run the calibrator** (use a 8×6 checkerboard with 25 mm squares, or adjust `--size` and `--square` to match yours):
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 --no-service-check \
  --ros-args --remap image:=/cam_1/image_raw --remap camera:=/cam_1
```

Move the checkerboard to cover the full frame until the `X`, `Y`, `Size`, and `Skew` bars turn green, then click **Calibrate** → **Save** → **Commit**.

The calibration is written to `~/.ros/camera_info/<camera-name>.yaml`. Copy it into this repo:

```bash
cp ~/.ros/camera_info/cam_1.yaml configs/cameras/cam_1_calibration.yaml
```

The launch file already points `camera_info_url` at `configs/cameras/<camera_id>_calibration.yaml`, so no further config changes are needed. Re-enable rectification when ready with:

```bash
ros2 launch lab_tracker_bringup single_camera_tracking.launch.py use_rectify:=true
```

### Camera intrinsics checklist

1. Start the camera-only launch for the target camera:
   `ros2 launch lab_tracker_bringup camera_only.launch.py camera_id:=cam_X device:=/dev/videoN`
2. Apply fixed camera controls:
   `./scripts/set_camera_controls.sh /dev/videoN`
3. Verify `/<camera_id>/image_raw` and `/<camera_id>/camera_info`.
4. Run camera calibration:
   `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --no-service-check --ros-args --remap image:=/cam_X/image_raw --remap camera:=/cam_X`
5. Copy the generated calibration output into `configs/cameras/cam_X_calibration.yaml`.
6. Run `python3 scripts/check_camera_intrinsics.py configs/cameras/cam_X_calibration.yaml`.
7. Run `python3 scripts/compute_fov_from_camera_info.py configs/cameras/cam_X_calibration.yaml --height-to-tag-plane 1.885`.
8. Only then enable AprilTag pose estimation for that camera.

---

## Configuration reference

| File | Purpose |
|---|---|
| `configs/cameras/camera_extrinsics.yaml` | T_lab_cam for each camera (world→camera) |
| `configs/cameras/cam_X_v4l2.yaml` | `v4l2_camera` parameters for each camera |
| `configs/cameras/cam_X_calibration.yaml` | Camera intrinsics (ROS camera_info format) |
| `configs/cameras/camera_layout_6.yaml` | Planned 6-camera ceiling layout and overlap assumptions |
| `configs/lab_layout.yaml` | Camera list and lab-frame layout for the central tracker |
| `configs/tracker.yaml` | Tracker stale-timeout and publish settings |
| `configs/cameras/cam_X.yaml` | App-level per-camera tracker config including extrinsics |
| `configs/robots/robots.yaml` | App-level robot/tag mapping used by the central tracker |
| `configs/tag_boards/robots.yaml` | T_base_tag for each robot's AprilTag |
| `configs/apriltag/tags.yaml` | apriltag_ros detector parameters |
| `configs/apriltag/tags_multi_camera.yaml` | apriltag_ros parameters for the multi-camera scaffold |
| `configs/fusion.yaml` | Fusion node parameters (topics, IDs, covariance) |
| `docs/camera_intrinsics_requirements.md` | Intrinsics requirements, calibration workflow, and validation tooling |
| `docs/multi_camera_tracking_architecture.md` | Multi-camera architecture boundary and tracker design notes |
