#!/usr/bin/env bash
#
# Record one bag containing both the 8 FLIR cameras and the Ouster lidar.
#
# Usage:
#   scripts/all_sensors_bagging.sh                 # <repo>/bags/all_<timestamp>
#   scripts/all_sensors_bagging.sh /path/my_bag    # explicit output directory
#
# Bring the rig up first (see all_sensors.launch.py), then start this. Stop with
# Ctrl-C. This is the camera+lidar counterpart to camera_bagging.sh; that script
# stays the right one when only the cameras are running.

set -euo pipefail

# Resolve against the repo, not the caller's cwd, so the bag always lands in
# <repo>/bags (a symlink to the data disk) no matter where this is invoked from.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bags_dir="${repo_root}/bags"

output="${1:-${bags_dir}/all_$(date +%Y%m%d_%H%M%S)}"

# Both sensors must be recorded from the same middleware as the one that launched
# them, or the recorder simply never discovers the topics and writes an empty bag.
# Nothing in this repo sets RMW_IMPLEMENTATION, so both the launch and this script
# land on the rmw_fastrtps_cpp default -- pinning it here makes that explicit and
# survives a shell that inherited cyclonedds from ~/flir_ouster_ws, which is a
# separate workspace built against a different camera driver.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# ros2 bag record takes topic names, not globs, so the camera set is selected with
# --regex (see the long note in camera_bagging.sh). Lidar topics are named
# explicitly. --regex applies to the whole invocation, so the lidar names are
# written as exact-match patterns rather than plain topic arguments.
camera_regex='/camera_[a-z0-9_]+/(image_rgb/compressed|image_raw/metadata|camera_info)'
# Thermal cameras (FLIR A70) have no image_rgb; image_raw is mono16 temperature
# (value * 0.01 = Kelvin), ~18 MB/s per camera. Optional: no warning when absent.
thermal_regex='/thermal[0-9]+/(image_raw|image_raw/metadata|camera_info)'

# Packets, not points. /ouster/points is the debayer-equivalent for the lidar: it
# is reconstructed from the raw packets on replay, so recording lidar_packets +
# metadata keeps the bag several times smaller with no information lost.
# /ouster/metadata is what makes that replay possible -- without it the packets
# cannot be interpreted and the bag is dead weight.
lidar_topics=(
  '/ouster/lidar_packets'
  '/ouster/imu_packets'
  '/ouster/metadata'
  '/ouster/imu'
)

common_topics=(
  '/tf'
  '/tf_static'
)

# Anchor each literal name so --regex cannot match a longer topic that merely
# contains it.
patterns=("^${camera_regex}$" "^${thermal_regex}$")
for t in "${lidar_topics[@]}" "${common_topics[@]}"; do
  patterns+=("^${t}$")
done
regex="$(IFS='|'; echo "${patterns[*]}")"

# A missing topic is not an error to ros2 bag -- it waits forever and records
# nothing, so the failure only shows up as an empty bag hours later. Check first.
# Mirrors the pre-flight in ~/lidar_ws/bagging_lidar_gps.sh.
live="$(ros2 topic list)"
missing=()
for t in "${lidar_topics[@]}" "${common_topics[@]}"; do
  grep -qxF "$t" <<<"$live" || missing+=("$t")
done
if ! grep -qE "${camera_regex}" <<<"$live"; then
  missing+=("(카메라 토픽 없음: /camera_*/image_rgb/compressed 등)")
fi

if (( ${#missing[@]} )); then
  echo "경고: 아래 토픽이 발행되고 있지 않습니다. 빈 채로 기록됩니다."
  printf '  %s\n' "${missing[@]}"
  echo
  read -rp "그래도 계속할까요? [y/N] " ans
  [[ "$ans" == [yY] ]] || exit 1
fi

echo "[all_sensors_bagging] recording to: ${output}"
echo "[all_sensors_bagging] stop with Ctrl-C"

# mcap over the default sqlite3: the lidar packet stream pushes the combined rate
# well past what the sqlite3 writer keeps up with, and dropped messages there are
# silent. camera_bagging.sh can stay on the default because cameras alone fit.
exec ros2 bag record --output "${output}" --storage mcap --regex "${regex}"
