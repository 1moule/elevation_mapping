#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 BAG OUTPUT_DIR FEATURE_ENABLED ROS_DOMAIN_ID" >&2
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
POSE_LEAD_SECONDS=1.0
POSE_TOPICS=(/fastlio2/lio_odom)
CLOUD_TOPICS=(/tf /fastlio2/body_cloud)
MAPPER_READY_LOG="Successfully launched node."
MAPPER_CALLBACK_THREADS=1

resolve_workspace_setup() {
  local candidate
  local candidates=(
    "${ELEVATION_MAPPING_WORKSPACE_SETUP:-}"
    "$SCRIPT_DIR/../../../install/setup.bash"
    "$SCRIPT_DIR/../../../setup.bash"
  )
  for candidate in "${candidates[@]}"; do
    if [[ -n "$candidate" && -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

source_setup() {
  set +u
  source "$1"
  set -u
}

mapper_stop_status_is_expected() {
  case "$1" in
    0|130|245)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

if [[ ${1:-} == "--check-mapper-stop-status" ]]; then
  if [[ $# -ne 2 ]]; then
    usage
    exit 2
  fi
  mapper_stop_status_is_expected "$2"
  exit
fi

if [[ ${1:-} == "--print-workspace-setup" ]]; then
  if [[ $# -ne 1 ]]; then
    usage
    exit 2
  fi
  if ! resolve_workspace_setup; then
    echo "Unable to infer elevation_mapping workspace setup; set ELEVATION_MAPPING_WORKSPACE_SETUP" >&2
    exit 2
  fi
  exit 0
fi

if [[ ${1:-} == "--print-replay-plan" ]]; then
  if [[ $# -ne 1 ]]; then
    usage
    exit 2
  fi
  printf 'mapper_ready_log=%s\n' "$MAPPER_READY_LOG"
  printf 'mapper_callback_threads=%s\n' "$MAPPER_CALLBACK_THREADS"
  printf 'pose_topics=%s\n' "${POSE_TOPICS[0]}"
  printf 'pose_lead_seconds=%s\n' "$POSE_LEAD_SECONDS"
  printf 'cloud_topics=%s,%s\n' "${CLOUD_TOPICS[0]}" "${CLOUD_TOPICS[1]}"
  exit 0
fi

if [[ $# -ne 4 ]]; then
  usage
  exit 2
fi

BAG_INPUT=$1
OUTPUT_DIR_INPUT=$2
FEATURE_ENABLED=$3
DOMAIN_ID=$4

if [[ ! -d "$BAG_INPUT" || ! -f "$BAG_INPUT/metadata.yaml" ]]; then
  echo "Input bag must be a rosbag2 directory with metadata.yaml: $BAG_INPUT" >&2
  exit 2
fi
if [[ -e "$OUTPUT_DIR_INPUT" || -L "$OUTPUT_DIR_INPUT" ]]; then
  echo "Output directory already exists: $OUTPUT_DIR_INPUT" >&2
  exit 2
fi
case "$FEATURE_ENABLED" in
  true|false)
    ;;
  *)
    echo "FEATURE_ENABLED must be true or false: $FEATURE_ENABLED" >&2
    exit 2
    ;;
esac
if [[ ! "$DOMAIN_ID" =~ ^[0-9]+$ ]] || (( DOMAIN_ID > 232 )); then
  echo "ROS_DOMAIN_ID must be an integer from 0 through 232: $DOMAIN_ID" >&2
  exit 2
fi

BAG=$(realpath "$BAG_INPUT")

# Source the ROS underlay, optional developer underlay, and inferred overlay.
ROS_SETUP=${ELEVATION_MAPPING_ROS_SETUP:-/opt/ros/humble/setup.bash}
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup does not exist: $ROS_SETUP" >&2
  exit 2
fi
source_setup "$ROS_SETUP"

if [[ -n ${ELEVATION_MAPPING_DEV_UNDERLAY_SETUP:-} ]]; then
  if [[ ! -f "$ELEVATION_MAPPING_DEV_UNDERLAY_SETUP" ]]; then
    echo "Developer underlay setup does not exist: $ELEVATION_MAPPING_DEV_UNDERLAY_SETUP" >&2
    exit 2
  fi
  source_setup "$ELEVATION_MAPPING_DEV_UNDERLAY_SETUP"
elif [[ -f /home/guanlin/catkin_ws/install/setup.bash ]]; then
  source_setup /home/guanlin/catkin_ws/install/setup.bash
fi

if ! workspace_setup=$(resolve_workspace_setup); then
  echo "Unable to infer elevation_mapping workspace setup; set ELEVATION_MAPPING_WORKSPACE_SETUP" >&2
  exit 2
fi
source_setup "$workspace_setup"

mkdir -p "$OUTPUT_DIR_INPUT"
OUTPUT_DIR=$(realpath "$OUTPUT_DIR_INPUT")
mkdir -p "$OUTPUT_DIR/ros_logs"

export ROS_DOMAIN_ID=$DOMAIN_ID
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
export ROS_LOG_DIR="$OUTPUT_DIR/ros_logs"

PACKAGE_PREFIX=$(ros2 pkg prefix elevation_mapping)
ROBOT_PARAMS="$PACKAGE_PREFIX/share/elevation_mapping/config/robots/fastlio_mid360.yaml"
POSTPROCESSOR_PARAMS="$PACKAGE_PREFIX/share/elevation_mapping/config/postprocessing/postprocessor_pipeline.yaml"
ANALYZER="$PACKAGE_PREFIX/lib/elevation_mapping/analyze_elevation_bag.py"
for required_file in "$ROBOT_PARAMS" "$POSTPROCESSOR_PARAMS" "$ANALYZER"; do
  if [[ ! -f "$required_file" ]]; then
    echo "Required installed elevation_mapping file is absent: $required_file" >&2
    exit 2
  fi
done

MAPPER_PID=
RECORDER_PID=
PLAYER_PID=
POSE_PLAYER_PID=
STOP_STATUS=0

child_running() {
  local pid=$1
  local state
  state=$(ps -o stat= -p "$pid" 2>/dev/null) || return 1
  state=${state#"${state%%[![:space:]]*}"}
  [[ -n "$state" && "${state:0:1}" != "Z" ]]
}

signal_group() {
  local pid=$1
  local signal=$2
  kill -s "$signal" -- "-$pid" 2>/dev/null ||
    kill -s "$signal" "$pid" 2>/dev/null ||
    true
}

stop_child() {
  local pid=$1
  local signal=${2:-INT}
  local attempt

  STOP_STATUS=0
  if [[ -z "$pid" ]]; then
    return
  fi
  if child_running "$pid"; then
    signal_group "$pid" "$signal"
  fi
  for attempt in {1..100}; do
    if ! child_running "$pid"; then
      break
    fi
    sleep 0.1
  done
  if child_running "$pid"; then
    signal_group "$pid" TERM
    for attempt in {1..50}; do
      if ! child_running "$pid"; then
        break
      fi
      sleep 0.1
    done
  fi
  if child_running "$pid"; then
    signal_group "$pid" KILL
  fi

  set +e
  wait "$pid"
  STOP_STATUS=$?
  set -e
}

cleanup() {
  local status=$?
  trap - EXIT
  stop_child "$PLAYER_PID" INT
  stop_child "$POSE_PLAYER_PID" INT
  stop_child "$RECORDER_PID" INT
  stop_child "$MAPPER_PID" INT
  exit "$status"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

MAPPER_TIME="$OUTPUT_DIR/mapper_time.txt"
setsid /usr/bin/time -q -f "%U %S" -o "$MAPPER_TIME" \
  ros2 run elevation_mapping elevation_mapping --ros-args \
  --params-file "$ROBOT_PARAMS" \
  --params-file "$POSTPROCESSOR_PARAMS" \
  -p "num_callback_threads:=$MAPPER_CALLBACK_THREADS" \
  -p "enable_multimodal_cells:=$FEATURE_ENABLED" \
  >"$OUTPUT_DIR/mapper.log" 2>&1 &
MAPPER_PID=$!

MAPS_BAG="$OUTPUT_DIR/maps"
setsid ros2 bag record \
  -o "$MAPS_BAG" \
  /elevation_map_raw_post \
  /elevation_map \
  >"$OUTPUT_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!

subscription_count() {
  local topic=$1
  local output
  local count
  output=$(timeout 1 ros2 topic info "$topic" --verbose 2>/dev/null) ||
    return 1
  count=$(awk '/^Subscription count:/ {print $3; exit}' <<<"$output")
  if [[ "$count" =~ ^[0-9]+$ ]]; then
    echo "$count"
    return 0
  fi
  return 1
}

deadline=$((SECONDS + 15))
mapper_ready=false
recorder_ready=false
mapper_launched=false
while (( SECONDS < deadline )); do
  if ! child_running "$MAPPER_PID"; then
    echo "Mapper exited before discovery completed; see mapper.log" >&2
    exit 1
  fi
  if ! child_running "$RECORDER_PID"; then
    echo "Recorder exited before discovery completed; see recorder.log" >&2
    exit 1
  fi

  mapper_subscriptions=$(subscription_count /fastlio2/body_cloud || echo 0)
  recorder_subscriptions=$(subscription_count /elevation_map_raw_post || echo 0)
  if (( mapper_subscriptions >= 1 )); then
    mapper_ready=true
  fi
  if (( recorder_subscriptions >= 1 )); then
    recorder_ready=true
  fi
  if grep -Fq "$MAPPER_READY_LOG" "$OUTPUT_DIR/mapper.log"; then
    mapper_launched=true
  fi
  if [[ "$mapper_ready" == true && "$recorder_ready" == true &&
        "$mapper_launched" == true ]]; then
    break
  fi
  sleep 0.2
done

if [[ "$mapper_ready" != true ]]; then
  echo "Timed out waiting for mapper /fastlio2/body_cloud subscription" >&2
  exit 1
fi
if [[ "$recorder_ready" != true ]]; then
  echo "Timed out waiting for recorder /elevation_map_raw_post subscription" >&2
  exit 1
fi
if [[ "$mapper_launched" != true ]]; then
  echo "Timed out waiting for mapper initialization; see mapper.log" >&2
  exit 1
fi

setsid ros2 bag play "$BAG" --rate 1.0 \
  --topics "${POSE_TOPICS[@]}" \
  >"$OUTPUT_DIR/pose_player.log" 2>&1 &
POSE_PLAYER_PID=$!

setsid ros2 bag play "$BAG" --rate 1.0 --delay "$POSE_LEAD_SECONDS" \
  --topics "${CLOUD_TOPICS[@]}" \
  >"$OUTPUT_DIR/player.log" 2>&1 &
PLAYER_PID=$!
set +e
wait "$PLAYER_PID"
player_status=$?
set -e
PLAYER_PID=
if (( player_status != 0 )); then
  echo "Bag player failed with exit status $player_status; see player.log" >&2
  exit 1
fi

set +e
wait "$POSE_PLAYER_PID"
pose_player_status=$?
set -e
POSE_PLAYER_PID=
if (( pose_player_status != 0 )); then
  echo "Pose bag player failed with exit status $pose_player_status; see pose_player.log" >&2
  exit 1
fi

stop_child "$RECORDER_PID" INT
recorder_status=$STOP_STATUS
RECORDER_PID=
if (( recorder_status != 0 && recorder_status != 130 )); then
  echo "Recorder failed with exit status $recorder_status; see recorder.log" >&2
  exit 1
fi

stop_child "$MAPPER_PID" INT
mapper_status=$STOP_STATUS
MAPPER_PID=
if ! mapper_stop_status_is_expected "$mapper_status"; then
  echo "Mapper failed with exit status $mapper_status; see mapper.log" >&2
  exit 1
fi

python3 "$ANALYZER" "$MAPS_BAG" \
  --cpu-seconds-file "$MAPPER_TIME" \
  --output "$OUTPUT_DIR/metrics.json"

trap - EXIT
exit 0
