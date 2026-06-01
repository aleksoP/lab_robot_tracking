#!/usr/bin/env python3
"""Compute FOV and ground footprint from a ROS CameraInfo YAML file."""

import argparse
import math
from pathlib import Path

import yaml


def _load_camera_info(path: Path) -> dict:
    with path.open() as f:
        return yaml.safe_load(f)


def _get_width(data: dict) -> float:
    return float(data.get('image_width', data.get('width', 0.0)))


def _get_height(data: dict) -> float:
    return float(data.get('image_height', data.get('height', 0.0)))


def _get_camera_matrix(data: dict) -> list[float]:
    matrix = data.get('camera_matrix') or {}
    values = matrix.get('data')
    if not values or len(values) < 9:
        raise ValueError('camera_matrix.data must contain 9 values')
    return [float(v) for v in values]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('calibration_yaml', help='Path to CameraInfo YAML')
    parser.add_argument(
        '--height-to-tag-plane',
        type=float,
        required=True,
        help='Vertical distance from camera optical center to the AprilTag plane in meters',
    )
    args = parser.parse_args()

    data = _load_camera_info(Path(args.calibration_yaml))
    width = _get_width(data)
    height = _get_height(data)
    if width <= 0.0 or height <= 0.0:
        raise ValueError('image_width/image_height must be present and non-zero')

    k = _get_camera_matrix(data)
    fx = k[0]
    fy = k[4]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('fx and fy must be non-zero')

    hfov_rad = 2.0 * math.atan(width / (2.0 * fx))
    vfov_rad = 2.0 * math.atan(height / (2.0 * fy))

    footprint_width = 2.0 * args.height_to_tag_plane * math.tan(hfov_rad / 2.0)
    footprint_height = 2.0 * args.height_to_tag_plane * math.tan(vfov_rad / 2.0)

    print(f'calibration: {args.calibration_yaml}')
    print(f'image size: {int(width)} x {int(height)}')
    print(f'horizontal FOV: {math.degrees(hfov_rad):.2f} deg')
    print(f'vertical FOV: {math.degrees(vfov_rad):.2f} deg')
    print(f'footprint width at {args.height_to_tag_plane:.3f} m: {footprint_width:.3f} m')
    print(f'footprint height at {args.height_to_tag_plane:.3f} m: {footprint_height:.3f} m')


if __name__ == '__main__':
    main()
