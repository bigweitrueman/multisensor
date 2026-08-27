#!/usr/bin/env bash
set -eo pipefail

# Record only synchronization metadata and WLR-722 data. Raw camera images are
# deliberately excluded; they must be stored by a camera-side recorder.

DURATION_SEC=""
OUTPUT_ROOT=""
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-37}"
SPLIT_SIZE="${ATLAS_BAG_SPLIT_SIZE:-4294967296}"
SPLIT_DURATION="${ATLAS_BAG_SPLIT_DURATION:-1800}"
MAX_CACHE="${ATLAS_BAG_CACHE_SIZE:-33554432}"
QOS_FILE="/userdata/atlas/config/atlas_bag_qos.yaml"

usage() {
  cat <<'EOF'
Usage: record_atlas_session.sh [--duration SEC] [--output DIR]

Records WLR-722 point clouds, LiDAR IMU, camera Header topics, and /tf_static
to MCAP. Raw /imx586/*/image* topics are never selected by this script. Start
the camera node with image_record_dir=<output>/camera to store matching H.264
(default) or raw frames alongside the bag.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      DURATION_SEC="$2"
      shift 2
      ;;
    --output)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

source /opt/ros/jazzy/setup.bash
source /userdata/atlas/install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ -z "${OUTPUT_ROOT}" ]]; then
  OUTPUT_ROOT="/userdata/atlas/sessions/atlas_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${OUTPUT_ROOT}"
mkdir -p "${OUTPUT_ROOT}/camera"

if [[ "$(git -C /userdata/atlas branch --show-current 2>/dev/null || true)" != \
  "feature/atlas-camera-lidar-sync-20260825" ]]; then
  echo "warning: /userdata/atlas is not on feature/atlas-camera-lidar-sync-20260825" >&2
fi

topic_present() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

echo "waiting for WLR-722 and IMU topics..."
deadline=$((SECONDS + 30))
while ((SECONDS < deadline)); do
  if topic_present /vanjee_points722 && topic_present /vanjee_lidar_imu_packets; then
    break
  fi
  sleep 1
done

required_topics=(/vanjee_points722 /vanjee_lidar_imu_packets)
for topic in "${required_topics[@]}"; do
  topic_present "${topic}" || {
    echo "required topic is not available: ${topic}" >&2
    exit 1
  }
done

topics=("${required_topics[@]}")
if topic_present /tf_static; then
  topics+=(/tf_static)
fi

# Header topics are tiny and can be recorded without copying image payloads.
while IFS= read -r topic; do
  [[ "${topic}" =~ ^/imx586/[^/]+/header$ ]] || continue
  topics+=("${topic}")
done < <(ros2 topic list 2>/dev/null | sort -u)

bag_prefix="${OUTPUT_ROOT}/radar_imu_headers"
printf '%s\n' "${topics[@]}" > "${OUTPUT_ROOT}/topics.txt"
{
  echo "created=$(date --iso-8601=seconds)"
  echo "git_branch=$(git -C /userdata/atlas branch --show-current 2>/dev/null || true)"
  echo "git_commit=$(git -C /userdata/atlas rev-parse HEAD 2>/dev/null || true)"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "storage=mcap"
  echo "storage_preset=fastwrite"
  echo "max_bag_size=${SPLIT_SIZE}"
  echo "max_bag_duration=${SPLIT_DURATION}"
  echo "raw_images=excluded"
  echo "camera_image_dir=${OUTPUT_ROOT}/camera"
  echo "topics_file=${OUTPUT_ROOT}/topics.txt"
} > "${OUTPUT_ROOT}/manifest.txt"

record_pid=""
stop_recorder() {
  [[ -n "${record_pid}" ]] || return 0
  if ! kill -0 "${record_pid}" 2>/dev/null; then
    wait "${record_pid}" 2>/dev/null || true
    record_pid=""
    return 0
  fi
  # rosbag2 treats SIGINT as a pause in this build. SIGTERM performs the
  # normal flush-and-close path; fall back to SIGINT only if needed.
  kill -TERM "${record_pid}" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 "${record_pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${record_pid}" 2>/dev/null; then
    kill -INT "${record_pid}" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "${record_pid}" 2>/dev/null || break
      sleep 0.1
    done
  fi
  wait "${record_pid}" 2>/dev/null || true
  record_pid=""
}
cleanup() {
  set +e
  stop_recorder
}
trap cleanup EXIT INT TERM

echo "recording to ${bag_prefix}"
echo "topics:"
sed 's/^/  /' "${OUTPUT_ROOT}/topics.txt"

ros2 bag record -s mcap -o "${bag_prefix}" \
  --topics "${topics[@]}" \
  --qos-profile-overrides-path "${QOS_FILE}" \
  --storage-preset-profile fastwrite \
  --max-bag-size "${SPLIT_SIZE}" \
  --max-bag-duration "${SPLIT_DURATION}" \
  --max-cache-size "${MAX_CACHE}" \
  --disable-keyboard-controls \
  --custom-data "raw_images=excluded" \
  --custom-data "camera_headers=recorded" \
  > "${OUTPUT_ROOT}/record.log" 2>&1 &
record_pid="$!"

if [[ -n "${DURATION_SEC}" ]]; then
  [[ "${DURATION_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
    echo "--duration must be a positive number" >&2
    exit 2
  }
  sleep "${DURATION_SEC}"
  stop_recorder
fi

if [[ -n "${record_pid}" ]]; then
  wait "${record_pid}" 2>/dev/null || true
  record_pid=""
fi
ros2 bag info "${bag_prefix}" > "${OUTPUT_ROOT}/bag_info.txt" 2>&1 || true
echo "recording finished: ${OUTPUT_ROOT}"
