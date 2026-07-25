#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPOSITORY_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
RUNNER="$REPOSITORY_ROOT/scripts/run_elevation_bag_regression.sh"

if ! bash "$RUNNER" --check-mapper-stop-status 245; then
  echo "runner rejected observed post-SIGINT mapper status 245" >&2
  exit 1
fi
if bash "$RUNNER" --check-mapper-stop-status 1; then
  echo "runner accepted unrelated mapper failure status 1" >&2
  exit 1
fi

expected_replay_plan=$'mapper_ready_log=Successfully launched node.\nmapper_callback_threads=1\npose_topics=/fastlio2/lio_odom\npose_lead_seconds=1.0\ncloud_topics=/tf,/fastlio2/body_cloud'
replay_plan=$(bash "$RUNNER" --print-replay-plan)
if [[ "$replay_plan" != "$expected_replay_plan" ]]; then
  printf 'unexpected replay plan:\n%s\n' "$replay_plan" >&2
  exit 1
fi

TEMPORARY_ROOT=$(mktemp -d)
trap 'rm -rf "$TEMPORARY_ROOT"' EXIT

BAG="$TEMPORARY_ROOT/input"
OUTPUT="$TEMPORARY_ROOT/output"
SETUP="$TEMPORARY_ROOT/setup.bash"
PREFIX="$TEMPORARY_ROOT/prefix"
mkdir -p "$BAG"
touch "$BAG/metadata.yaml"

cat >"$SETUP" <<'EOF'
if [[ -n "$RUNNER_TEST_UNSET_SENTINEL" ]]; then
  :
fi

ros2() {
  if [[ "$1" == "pkg" && "$2" == "prefix" ]]; then
    printf '%s\n' "$RUNNER_TEST_PREFIX"
    return 0
  fi
  return 2
}
EOF

set +e
runner_output=$(
  RUNNER_TEST_PREFIX="$PREFIX" \
    ELEVATION_MAPPING_ROS_SETUP="$SETUP" \
    ELEVATION_MAPPING_DEV_UNDERLAY_SETUP="$SETUP" \
    ELEVATION_MAPPING_WORKSPACE_SETUP="$SETUP" \
    bash "$RUNNER" "$BAG" "$OUTPUT" false 77 2>&1
)
runner_status=$?
set -e

expected_error="Required installed elevation_mapping file is absent: $PREFIX/share/elevation_mapping/config/robots/fastlio_mid360.yaml"
if [[ $runner_status -ne 2 || "$runner_output" != *"$expected_error"* ]]; then
  printf 'runner status: %d\nrunner output:\n%s\n' \
    "$runner_status" "$runner_output" >&2
  exit 1
fi

echo "run_elevation_bag_regression setup sourcing test: PASS"
