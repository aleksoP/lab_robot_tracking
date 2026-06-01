#!/usr/bin/env python3
"""Check a camera calibration YAML for placeholder or suspicious intrinsics."""

import argparse
from pathlib import Path

import yaml


OBSERVED_UVC_CAMERA_NAME = 'usb_camera3:_usb_camera3'


def _load_camera_info(path: Path) -> dict:
    with path.open() as f:
        return yaml.safe_load(f)


def _get_width(data: dict) -> float:
    return float(data.get('image_width', data.get('width', 0.0)))


def _get_height(data: dict) -> float:
    return float(data.get('image_height', data.get('height', 0.0)))


def _get_matrix_value(data: dict, row: int, col: int) -> float:
    matrix = data.get('camera_matrix') or {}
    values = matrix.get('data') or []
    if len(values) < 9:
        return 0.0
    return float(values[row * 3 + col])


def _get_distortion(data: dict) -> list[float]:
    distortion = data.get('distortion_coefficients') or {}
    values = distortion.get('data') or []
    return [float(v) for v in values]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('calibration_yaml', help='Path to CameraInfo YAML')
    args = parser.parse_args()

    path = Path(args.calibration_yaml)
    data = _load_camera_info(path)

    width = _get_width(data)
    height = _get_height(data)
    fx = _get_matrix_value(data, 0, 0)
    fy = _get_matrix_value(data, 1, 1)
    cx = _get_matrix_value(data, 0, 2)
    cy = _get_matrix_value(data, 1, 2)
    distortion = _get_distortion(data)
    camera_name = str(data.get('camera_name', '')).strip()

    warnings: list[str] = []

    if fx == 0.0 or fy == 0.0:
        warnings.append('fx or fy is zero')
    if distortion and all(value == 0.0 for value in distortion):
        warnings.append('distortion coefficients are all zero')
    if width > 0.0 and fx == width / 2.0 and fy == width / 2.0:
        warnings.append('fx == width/2 and fy == width/2 exactly; looks like placeholder intrinsics')
    if width > 0.0 and height > 0.0 and cx == width / 2.0 and cy == height / 2.0:
        warnings.append('cx == width/2 and cy == height/2 exactly; looks like placeholder principal point')
    if not camera_name:
        warnings.append('camera_name is missing')
    elif camera_name.startswith('cam_'):
        warnings.append(
            f"camera_name '{camera_name}' looks like a logical camera ID; "
            f'camera_info_manager may instead expect the UVC name {OBSERVED_UVC_CAMERA_NAME!r}'
        )
    elif 'usb' not in camera_name.lower() and ':' not in camera_name:
        warnings.append(
            f"camera_name '{camera_name}' does not look like the observed UVC name "
            f'{OBSERVED_UVC_CAMERA_NAME!r}; verify camera_info_manager compatibility'
        )

    status = 'PASS' if not warnings else 'WARN'
    print(f'{status}: {args.calibration_yaml}')
    print(f'  image size: {int(width) if width else 0} x {int(height) if height else 0}')
    print(f'  camera_name: {camera_name or "<missing>"}')
    print(f'  fx={fx:.6f} fy={fy:.6f} cx={cx:.6f} cy={cy:.6f}')
    if warnings:
        for warning in warnings:
            print(f'  - {warning}')


if __name__ == '__main__':
    main()
