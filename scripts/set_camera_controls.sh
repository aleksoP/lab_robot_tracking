#!/usr/bin/env bash
set -euo pipefail

DEVICE="${1:-/dev/video0}"

echo "[INFO] Setting manual camera controls for ${DEVICE}"

# Put camera into manual modes first.
v4l2-ctl -d "${DEVICE}" --set-ctrl=auto_exposure=1 || true
v4l2-ctl -d "${DEVICE}" --set-ctrl=focus_automatic_continuous=0 || true
v4l2-ctl -d "${DEVICE}" --set-ctrl=white_balance_automatic=0 || true
v4l2-ctl -d "${DEVICE}" --set-ctrl=exposure_dynamic_framerate=0 || true

# For AprilTag tracking under 30 FPS lab lighting, start with a shorter manual
# exposure to reduce motion blur and avoid over-bright frames / mains flicker.
# UVC exposure_time_absolute uses 100 us units, so 167 ~= 16.7 ms.
v4l2-ctl -d "${DEVICE}" --set-ctrl=exposure_time_absolute=167 || true
v4l2-ctl -d "${DEVICE}" --set-ctrl=focus_absolute=68 || true
v4l2-ctl -d "${DEVICE}" --set-ctrl=white_balance_temperature=4600 || true
# This camera advertises power_line_frequency but rejects 2 (60 Hz) when set,
# so leave the current device value unchanged.

echo "[INFO] Current relevant controls:"
v4l2-ctl -d "${DEVICE}" --all | grep -E \
  "auto_exposure|exposure_time_absolute|exposure_dynamic_framerate|focus_absolute|focus_automatic_continuous|white_balance_automatic|white_balance_temperature|power_line_frequency" || true
