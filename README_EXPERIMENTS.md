### Experiment Recording and Analysis

- Ensure your RealSense is connected. This will launch the full Taiwan system, record 10s clips, and analyze results.

```bash
# Optional: set how long to run (10s x 6 segments = 1 minute)
export SEGMENT_SECONDS=10
export MAX_SEGMENTS=6
# Run end-to-end
/home/sophie/visionaid-1/run_experiment.sh
```

- Direct launch via ROS2 with custom args:

```bash
ros2 launch traffic_crossing_assistant experiment_record_and_system.launch.py \
  model_path:=/home/sophie/visionaid-1/models/yolov8/balanced.onnx \
  output_dir:=/home/sophie/visionaid-1/experiment_data \
  segment_seconds:=10 \
  max_segments:=6 \
  record_viz:=false \
  enable_tegrastats:=true
```

- Analyze latest session afterwards (generates `summary.md` and CSVs):

```bash
python3 /home/sophie/visionaid-1/isaac_ros_ws/src/traffic_crossing_assistant/scripts/analyze_experiments.py \
  --root /home/sophie/visionaid-1/experiment_data --session latest
```

Artifacts:
- Clips: `experiment_data/session_*/clips/*.mp4`
- Logs: `experiment_data/session_*/logs/messages.jsonl`, `latency_summary.json`, `tegrastats.log`
- Reports: `experiment_data/session_*/summary.md`, `latency_stats.csv`, `decision_counts.csv`, `tegrastats_summary.csv` 