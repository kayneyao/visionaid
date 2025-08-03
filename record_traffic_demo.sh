#!/bin/bash
# Traffic System Demo Recording Script
# Records the full traffic system in action

set -e

echo "🚦 Traffic System Demo Recording Script"
echo "======================================"

# Configuration
DEMO_NAME="traffic_system_demo"
RECORDING_DURATION=60  # seconds
OUTPUT_DIR="./demo_recordings"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Check if recording tools are available
check_recording_tools() {
    echo "🔍 Checking recording tools..."
    
    if command -v ffmpeg &> /dev/null; then
        RECORDER="ffmpeg"
        echo "✅ FFmpeg found - will use for high-quality recording"
    elif command -v recordmydesktop &> /dev/null; then
        RECORDER="recordmydesktop"
        echo "✅ recordmydesktop found - will use for simple recording"
    else
        echo "❌ No recording tools found. Installing recordmydesktop..."
        sudo apt update && sudo apt install -y recordmydesktop
        RECORDER="recordmydesktop"
    fi
}

# Start screen recording
start_recording() {
    echo "🎬 Starting screen recording..."
    
    case $RECORDER in
        "ffmpeg")
            # High quality FFmpeg recording
            ffmpeg -f x11grab -r 30 -s 1920x1080 -i :0.0 \
                   -vcodec libx264 -preset ultrafast -crf 18 \
                   -y "$OUTPUT_DIR/${DEMO_NAME}_$(date +%Y%m%d_%H%M%S).mp4" &
            RECORDING_PID=$!
            ;;
        "recordmydesktop")
            # Simple recordmydesktop recording
            recordmydesktop --output "$OUTPUT_DIR/${DEMO_NAME}_$(date +%Y%m%d_%H%M%S).ogv" &
            RECORDING_PID=$!
            ;;
    esac
    
    echo "📹 Recording started with PID: $RECORDING_PID"
    sleep 3  # Give recording time to start
}

# Launch the traffic system
launch_traffic_system() {
    echo "🚀 Launching Taiwan Complete Traffic System..."
    
    # Source the workspace
    source /home/sophie/visionaid-1/isaac_ros_ws/install/setup.bash
    
    # Launch the system
    ros2 launch traffic_crossing_assistant taiwan_complete_system.launch.py &
    TRAFFIC_PID=$!
    
    echo "🔄 Traffic system launched with PID: $TRAFFIC_PID"
    echo "⏱️  System will run for $RECORDING_DURATION seconds..."
}

# Launch RViz for visualization
launch_rviz() {
    echo "📊 Launching RViz visualization..."
    
    # Wait a moment for the system to start
    sleep 5
    
    # Launch RViz with the traffic system configuration
    ros2 run rviz2 rviz2 -d /home/sophie/visionaid-1/yolov8_detection.rviz &
    RVIZ_PID=$!
    
    echo "📈 RViz launched with PID: $RVIZ_PID"
}

# Monitor and cleanup
monitor_and_cleanup() {
    echo "⏰ Recording for $RECORDING_DURATION seconds..."
    
    # Wait for recording duration
    sleep $RECORDING_DURATION
    
    echo "🛑 Stopping recording and traffic system..."
    
    # Stop recording
    if [ ! -z "$RECORDING_PID" ]; then
        kill $RECORDING_PID 2>/dev/null || true
        echo "📹 Recording stopped"
    fi
    
    # Stop traffic system
    if [ ! -z "$TRAFFIC_PID" ]; then
        kill $TRAFFIC_PID 2>/dev/null || true
        echo "🚦 Traffic system stopped"
    fi
    
    # Stop RViz
    if [ ! -z "$RVIZ_PID" ]; then
        kill $RVIZ_PID 2>/dev/null || true
        echo "📊 RViz stopped"
    fi
    
    # Kill any remaining ROS2 processes
    pkill -f "ros2" 2>/dev/null || true
    
    echo "✅ Demo recording completed!"
    echo "📁 Recording saved in: $OUTPUT_DIR/"
    ls -la "$OUTPUT_DIR/"*.mp4 "$OUTPUT_DIR/"*.ogv 2>/dev/null || echo "No recordings found"
}

# Main execution
main() {
    check_recording_tools
    start_recording
    launch_traffic_system
    launch_rviz
    monitor_and_cleanup
}

# Handle script interruption
cleanup_on_exit() {
    echo "🛑 Script interrupted - cleaning up..."
    if [ ! -z "$RECORDING_PID" ]; then kill $RECORDING_PID 2>/dev/null || true; fi
    if [ ! -z "$TRAFFIC_PID" ]; then kill $TRAFFIC_PID 2>/dev/null || true; fi
    if [ ! -z "$RVIZ_PID" ]; then kill $RVIZ_PID 2>/dev/null || true; fi
    pkill -f "ros2" 2>/dev/null || true
    exit 1
}

# Set up signal handlers
trap cleanup_on_exit SIGINT SIGTERM

# Run main function
main "$@" 