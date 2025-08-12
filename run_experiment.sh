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

# Build (optional if already built)
# source ROS2 env
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
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