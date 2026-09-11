#!/usr/bin/env bash
set -euo pipefail

viewer="${1:-true}"
native_cuda="${2:-true}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
roudi_config="$project_root/scripts/roudi_insta360.toml"
roudi_log="$(mktemp)"

cleanup()
{
  kill "$roudi_pid" 2>/dev/null || true
  rm -f "$roudi_log"
}

iox-roudi -c "$roudi_config" --log-level warning >"$roudi_log" 2>&1 &
roudi_pid=$!
trap cleanup EXIT INT TERM

for _ in {1..100}; do
  grep -q "RouDi is ready for clients" "$roudi_log" && break
  if ! kill -0 "$roudi_pid" 2>/dev/null; then
    break
  fi
  sleep 0.1
done

if ! grep -q "RouDi is ready for clients" "$roudi_log"; then
  cat "$roudi_log" >&2
  exit 1
fi

set +u
source "$project_root/install/setup.bash"
set -u
ros2 launch insta360_ros2_cuda_driver bringup.launch.xml \
  equirectangular:=true \
  perspective:=false \
  viewer:="$viewer" \
  native_cuda_stitcher:="$native_cuda"
