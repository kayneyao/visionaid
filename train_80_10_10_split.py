#!/usr/bin/env python3
"""
YOLOv8 Training Script for 80/10/10 Dataset Split
Trains the balanced 11-class model with the new split
"""

import os
import subprocess
import argparse
from pathlib import Path

def train_yolov8_80_10_10():
    """Train YOLOv8 with the new 80/10/10 split"""
    
    # Dataset path for the new split
    dataset_path = "/home/sophie/visionaid-1/yolo_training/11classnew/balanced_augmented_dataset_80_10_10"
    data_yaml = os.path.join(dataset_path, "data.yaml")
    
    # Check if dataset exists
    if not Path(data_yaml).exists():
        print(f"❌ Error: Dataset not found at {data_yaml}")
        print("Please run reorganize_dataset_80_10_10.py first")
        return
    
    # Training parameters
    model_size = "n"  # YOLOv8-n for speed
    epochs = 150
    batch_size = 64
    img_size = 640
    
    # Output directory
    output_dir = f"runs/balanced_80_10_10_training"
    
    # YOLO training command
    cmd = [
        "yolo", "train",
        "model=yolov8n.pt",
        f"data={data_yaml}",
        f"epochs={epochs}",
        f"batch={batch_size}",
        f"imgsz={img_size}",
        f"project={output_dir}",
        "name=balanced_80_10_10_11class",
        "patience=30",
        "save=True",
        "save_period=10",
        "cache=True",
        "device=0"  # Use GPU 0
    ]
    
    print("🚀 Starting YOLOv8 training with 80/10/10 split...")
    print(f"📁 Dataset: {data_yaml}")
    print(f"📊 Split: 80% train, 10% val, 10% test")
    print(f"Model: YOLOv8-{model_size}")
    print(f"Epochs: {epochs}")
    print(f"📦 Batch size: {batch_size}")
    print(f"🖼️  Image size: {img_size}")
    print(f"📁 Output: {output_dir}")
    
    # Run training
    try:
        subprocess.run(cmd, check=True)
        print("✅ Training completed successfully!")
    except subprocess.CalledProcessError as e:
        print(f"❌ Training failed with error: {e}")
    except KeyboardInterrupt:
        print("\n⏹️  Training interrupted by user")

if __name__ == "__main__":
    train_yolov8_80_10_10() 