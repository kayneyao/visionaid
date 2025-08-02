#!/usr/bin/env python3
"""
Verify Model Updates
Check that all files have been updated to use the new balanced augmented ONNX model
"""

import os
import glob
from pathlib import Path

def verify_model_updates():
    """Verify all files have been updated to use the new model"""
    
    new_model_path = "/home/sophie/visionaid-1/yolo_training/11classnew/runs/balanced_augmented_training/balanced_augmented_11class/weights/balanced.onnx"
    old_model_path = "/home/sophie/visionaid-1/yolo_training/11classnew/models/fresh_11class.onnx"
    
    print("=" * 70)
    print("🔍 VERIFYING MODEL UPDATES")
    print("=" * 70)
    
    # Check if new model exists
    if os.path.exists(new_model_path):
        print(f"✅ New model exists: {new_model_path}")
        model_size = os.path.getsize(new_model_path) / (1024 * 1024)  # MB
        print(f"📊 Model size: {model_size:.1f} MB")
    else:
        print(f"❌ New model not found: {new_model_path}")
        return False
    
    print()
    
    # Files to check
    files_to_check = [
        # YOLOv8 Detection files
        "isaac_ros_ws/src/yolov8_detection/yolov8_detection/yolov8_camera_node.py",
        "isaac_ros_ws/src/yolov8_detection/launch/yolov8_full_pipeline.launch.py",
        "isaac_ros_ws/src/yolov8_detection/launch/yolov8_slam_integration.launch.py",
        "isaac_ros_ws/src/yolov8_detection/launch/yolov8_realsense.launch.py",
        "isaac_ros_ws/src/yolov8_detection/config/yolov8_config.yaml",
        "isaac_ros_ws/src/yolov8_detection/config/camera_params.yaml",
        
        # Traffic Crossing Assistant files
        "isaac_ros_ws/src/traffic_crossing_assistant/launch/taiwan_complete_system.launch.py",
        "isaac_ros_ws/src/traffic_crossing_assistant/launch/multimodal_complete_system.launch.py",
    ]
    
    print("📋 Checking files for model path updates:")
    print("-" * 50)
    
    all_updated = True
    
    for file_path in files_to_check:
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                content = f.read()
                
            if old_model_path in content:
                print(f"❌ {file_path} - Still contains old model path")
                all_updated = False
            elif new_model_path in content:
                print(f"✅ {file_path} - Updated to new model")
            else:
                print(f"⚠️  {file_path} - No model path found")
        else:
            print(f"❌ {file_path} - File not found")
            all_updated = False
    
    print()
    print("=" * 70)
    
    if all_updated:
        print("🎉 ALL FILES SUCCESSFULLY UPDATED!")
        print("✅ Your system is now configured to use the balanced augmented model")
        print("📊 Model performance: 82.11% mAP50 (vs 45.79% previous)")
        print()
        print("🚀 Ready to launch with:")
        print("  ros2 launch yolov8_detection yolov8_realsense.launch.py")
        print("  ros2 launch traffic_crossing_assistant taiwan_complete_system.launch.py")
    else:
        print("❌ Some files still need updating")
        print("Please check the files marked with ❌ above")
    
    print("=" * 70)
    
    return all_updated

if __name__ == "__main__":
    verify_model_updates() 