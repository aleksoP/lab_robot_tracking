#!/usr/bin/env python3
"""Validate the multi-camera tracker scaffold configuration."""

from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent.parent
CONFIGS = ROOT / 'configs'


def load_yaml(path: Path) -> dict:
    with path.open() as f:
        return yaml.safe_load(f) or {}


def main() -> None:
    warnings: list[str] = []
    errors: list[str] = []

    lab_layout_path = CONFIGS / 'lab_layout.yaml'
    tracker_path = CONFIGS / 'tracker.yaml'
    robots_path = CONFIGS / 'robots' / 'robots.yaml'

    for required in [lab_layout_path, tracker_path, robots_path]:
        if not required.exists():
            errors.append(f'missing required config: {required}')

    if errors:
        print('FAIL')
        for error in errors:
            print(f'  - {error}')
        raise SystemExit(1)

    lab_layout = load_yaml(lab_layout_path)
    robots = load_yaml(robots_path).get('robots', {})
    camera_ids = lab_layout.get('cameras', [])

    for camera_id in camera_ids:
        path = CONFIGS / 'cameras' / f'{camera_id}.yaml'
        if not path.exists():
            errors.append(f'missing camera config: {path}')
            continue
        data = load_yaml(path)
        if 'detections_topic' not in data:
            errors.append(f'{path}: missing detections_topic')
        if 'frame_id' not in data:
            errors.append(f'{path}: missing frame_id')
        if 'extrinsic' not in data:
            errors.append(f'{path}: missing extrinsic')
        else:
            extrinsic = data['extrinsic']
            for key in ['parent_frame', 'child_frame', 'translation_xyz_m', 'rotation_rpy_rad']:
                if key not in extrinsic:
                    errors.append(f'{path}: extrinsic missing {key}')
            if extrinsic.get('placeholder', False):
                warnings.append(f'{path}: extrinsic.placeholder is true')

    for robot_id, robot_data in robots.items():
        tags = robot_data.get('tags', {})
        if not tags:
            warnings.append(f'robot {robot_id}: no tags configured')
        for tag_id, tag_data in tags.items():
            base_t_tag = tag_data.get('base_T_tag')
            if base_t_tag is None:
                errors.append(f'robot {robot_id} tag {tag_id}: missing base_T_tag')
                continue
            for key in ['translation_xyz_m', 'rotation_rpy_rad']:
                if key not in base_t_tag:
                    errors.append(f'robot {robot_id} tag {tag_id}: base_T_tag missing {key}')
            if base_t_tag.get('placeholder', False):
                warnings.append(f'robot {robot_id} tag {tag_id}: base_T_tag.placeholder is true')

    if errors:
        print('FAIL')
        for error in errors:
            print(f'  - {error}')
        raise SystemExit(1)

    status = 'PASS' if not warnings else 'WARN'
    print(status)
    print(f'  cameras: {len(camera_ids)}')
    print(f'  robots: {len(robots)}')
    if warnings:
        for warning in warnings:
            print(f'  - {warning}')


if __name__ == '__main__':
    main()
