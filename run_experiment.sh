#!/usr/bin/env bash
set -euo pipefail

# Configuration
WORKSPACE="/home/sophie/visionaid-1"
ROS_WS="$WORKSPACE/isaac_ros_ws"
OUT_DIR="$WORKSPACE/experiment_data"
MODEL_PATH="$WORKSPACE/models/yolov8/balanced.onnx"
SEGMENT_SECONDS=${SEGMENT_SECONDS:-10}
MAX_SEGMENTS=${MAX_SEGMENTS:-6}
RECORD_VIZ=${RECORD_VIZ:-false}
TEGRastats=${TEGRastats:-true}

# Source ROS 2 under set +u to avoid AMENT unbound variable issues
set +u
[ -f "/opt/ros/humble/setup.bash" ] && source /opt/ros/humble/setup.bash
set -u

# Build workspace if needed (or rebuild to register new launch/scripts)
if [ ! -f "$ROS_WS/install/setup.bash" ]; then
  echo "[INFO] Building workspace..."
  cd "$ROS_WS"
  colcon build --symlink-install
else
  echo "[INFO] Rebuilding to ensure package index is up-to-date..."
  cd "$ROS_WS"
  colcon build --symlink-install --packages-select traffic_crossing_assistant || true
fi

# Source overlay under set +u
set +u
source "$ROS_WS/install/setup.bash"
set -u

# Verify package is discoverable
if ! ros2 pkg prefix traffic_crossing_assistant >/dev/null 2>&1; then
  echo "[ERROR] Package 'traffic_crossing_assistant' not found after build."
  exit 1
fi

mkdir -p "$OUT_DIR"

# Run launch with recorder
set +e
ros2 launch traffic_crossing_assistant experiment_record_and_system.launch.py \
  model_path:=$MODEL_PATH \
  output_dir:=$OUT_DIR \
  segment_seconds:=$SEGMENT_SECONDS \
  max_segments:=$MAX_SEGMENTS \
  record_viz:=$RECORD_VIZ \
  enable_tegrastats:=$TEGRastats | cat
LAUNCH_RC=$?
set -e

# Analyze latest session
python3 "$ROS_WS/src/traffic_crossing_assistant/scripts/analyze_experiments.py" \
  --root "$OUT_DIR" --session latest | cat

exit $LAUNCH_RC 