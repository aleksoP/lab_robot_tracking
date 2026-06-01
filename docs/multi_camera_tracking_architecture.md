# Multi-Camera Tracking Architecture

## Why Global Tag TF Is Unsafe

The single-camera prototype relies on `apriltag_ros` publishing a TF frame such as `tag10`.
That is acceptable with one camera, but unsafe with multiple cameras because two detectors may
publish the same child frame from different camera parents at the same time.

For the 6-camera system, the tracker must ignore AprilTag TF and consume
`AprilTagDetectionArray` messages directly while preserving `camera_id` explicitly.

## Correct System Boundary

```text
/<camera_id>/image_raw
  -> /<camera_id>/detections   (apriltag_ros)
  -> multi_camera_tracker_node
  -> /lab_gt/<robot_id>/pose
  -> /lab_gt/<robot_id>/odom
  -> optional TF: lab_map -> <robot_id>/gt_base_link
```

Ground-truth TF remains separate from robot localization TF. This project must not publish:

- `map -> odom`
- `odom -> base_link`
- `map -> base_link`

## Transform Math

For each known robot tag observation:

```text
lab_T_base = lab_T_camera * camera_T_tag * inverse(base_T_tag)
```

Where:

- `lab_T_camera` comes from per-camera extrinsics
- `camera_T_tag` is estimated centrally from detected tag corners and per-camera intrinsics
- `base_T_tag` comes from robot configuration

## Current Multi-Camera Scaffold

The current scaffold uses:

- `configs/lab_layout.yaml`
- `configs/tracker.yaml`
- `configs/cameras/cam_X.yaml`
- `configs/robots/robots.yaml`
- `configs/apriltag/tags_multi_camera.yaml`

The central node is:

- `lab_tracker_fusion/multi_camera_tracker_node`

## Current Best-Observation Selection

This first implementation does not do weighted fusion yet.

Per robot:

- collect the latest valid observation from each camera
- reject stale observations older than `tracker.stale_timeout_sec`
- choose the best remaining observation
- currently prefer higher `decision_margin`, then newer timestamps

Future work should incorporate:

- tag pixel area
- reprojection error
- camera center distance
- explicit covariance modeling
- cross-camera consistency checks
