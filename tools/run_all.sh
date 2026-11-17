#!/usr/bin/env bash
set -euo pipefail

flight_profile=${1:?flight profile is required}
drone_model=${2:?drone model is required}

case "$flight_profile:$drone_model" in
  internal:racer)
    physics_config=../config/internal.yaml
    mission_config=../config/bullet_profile.yaml
    ;;
  betaflight:racer)
    physics_config=../config/betaflight.yaml
    mission_config=../config/betaflight_los.yaml
    ;;
  internal:heavy_quad)
    physics_config=../config/internal_heavy_quad.yaml
    mission_config=../config/bullet_profile_heavy_quad.yaml
    ;;
  betaflight:heavy_quad)
    physics_config=../config/betaflight_heavy_quad.yaml
    mission_config=../config/betaflight_los_heavy_quad.yaml
    ;;
  *)
    echo "Unknown backend/drone combination: $flight_profile:$drone_model" >&2
    exit 2
    ;;
esac

cd "$(dirname "$0")/.."
python3 tools/cleanup_flight.py
cmake --preset debug
cmake --build --preset debug

uv run --locked --package bullet-view bullet-view &
bullet_view_pid=$!
uv run --locked --package bullet-visual bullet-visual \
  --config ../bullet_visual/config/zmq.yaml &
bullet_visual_pid=$!

cleanup() {
  kill "$bullet_visual_pid" "$bullet_view_pid" 2>/dev/null || true
}
trap cleanup EXIT

exec uv run --locked --all-packages --all-extras --all-groups \
  python tools/run_flight.py \
  --physics-config "$physics_config" \
  --mission-config "$mission_config" \
  --mcap-output-dir ~/workspaces/bullet/recordings
