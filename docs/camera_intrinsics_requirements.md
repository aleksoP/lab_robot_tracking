# Camera Intrinsics Requirements

AprilTag 3-D pose estimation depends on real camera intrinsics. Without a calibrated `CameraInfo`,
the detector may still report tags, but the recovered position and orientation are not trustworthy
enough for absolute lab localization.

## Real Calibration vs Placeholder

The repository currently ships placeholder CameraInfo files for `cam_1` through `cam_6`.
These placeholders are only meant to let the ROS graph start. They are not valid intrinsics for
AprilTag pose estimation.

Real calibrated CameraInfo must provide:

- `image_width` / `width`
- `image_height` / `height`
- `camera_name`
- `camera_matrix` (`K`)
- `distortion_model` such as `plumb_bob`
- `distortion_coefficients` (`D`)
- `rectification_matrix` (`R`)
- `projection_matrix` (`P`)

## One-Camera Calibration Workflow

Start the camera for the desired logical camera ID:

```bash
ros2 launch lab_tracker_bringup camera_only.launch.py camera_id:=cam_1 device:=/dev/video0
./scripts/set_camera_controls.sh /dev/video0
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 --no-service-check \
  --ros-args --remap image:=/cam_1/image_raw --remap camera:=/cam_1
```

`--size 8x6` means the checkerboard has `8 x 6` internal corners, not squares.

After saving calibration, replace `configs/cameras/cam_1_calibration.yaml` with the generated
`ost.yaml` contents for that physical camera. If your saved file has a different name, copy the
equivalent YAML output into the repository calibration file.

The `camera_name` field may need to match the UVC name expected by `camera_info_manager`.
On this system the observed value is currently `usb_camera3:_usb_camera3`.

## Per-Camera Requirement

Calibration must be repeated for every physical camera `cam_1` through `cam_6`, and it must be
done at the same resolution used during AprilTag tracking. A `1280 x 720` calibration is not valid
for `1920 x 1080` tracking, and vice versa.

## Verification Tooling

Check whether a calibration still looks like a placeholder:

```bash
python3 scripts/check_camera_intrinsics.py configs/cameras/cam_1_calibration.yaml
```

Compute FOV and floor footprint at the tag plane:

```bash
python3 scripts/compute_fov_from_camera_info.py \
  configs/cameras/cam_1_calibration.yaml \
  --height-to-tag-plane 1.885
```
