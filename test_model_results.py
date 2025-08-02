#!/usr/bin/env python3
"""
Simple YOLOv8 Model Test Script
Loads the model and shows current training results
"""

import os
import sys
from pathlib import Path
import torch
from ultralytics import YOLO
import yaml
import json
import numpy as np
import pandas as pd
from datetime import datetime

def load_model_and_show_results():
    """Load YOLOv8 model and display training results"""
    
    print("=" * 60)
    print("YOLOv8 MODEL TEST - TRAINING RESULTS")
    print("=" * 60)
    
    # Check CUDA availability
    print(f"CUDA Available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"GPU Device: {torch.cuda.get_device_name(0)}")
        print(f"GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
    print()
    
    # Specific model path provided by user
    model_path = "/home/sophie/visionaid-1/yolo_training/11classnew/runs/merged_dataset_training/merged_dataset_11class/weights/best.pt"
    
    # Check if model exists
    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        return False
    
    file_size = os.path.getsize(model_path) / (1024 * 1024)  # MB
    print(f"✅ Model found: {model_path} ({file_size:.1f} MB)")
    print()
    
    print(f"Loading model: {model_path}")
    print("-" * 40)
    
    try:
        # Load the model
        model = YOLO(model_path)
        
        # Get model info
        print("📊 MODEL INFORMATION:")
        print(f"Model Type: {type(model).__name__}")
        print(f"Model Path: {model_path}")
        
        # Get model parameters
        if hasattr(model, 'model'):
            total_params = sum(p.numel() for p in model.model.parameters())
            trainable_params = sum(p.numel() for p in model.model.parameters() if p.requires_grad)
            print(f"Total Parameters: {total_params:,}")
            print(f"Trainable Parameters: {trainable_params:,}")
        
        # Get class names
        if hasattr(model, 'names'):
            class_names = model.names
            print(f"Number of Classes: {len(class_names)}")
            print("Class Names:")
            for i, name in class_names.items():
                print(f"  {i}: {name}")
        
        print()
        
        # Check for training results in the training directory
        print("📈 TRAINING RESULTS:")
        training_dir = Path(model_path).parent.parent  # Go up to the training run directory
        
        # Check for results.csv file
        results_csv = training_dir / "results.csv"
        if results_csv.exists():
            print(f"Found detailed results: {results_csv}")
            try:
                df = pd.read_csv(results_csv)
                print(f"Training completed for {len(df)} epochs")
                
                # Show final metrics
                if len(df) > 0:
                    final_row = df.iloc[-1]
                    print("\n🏆 FINAL TRAINING METRICS:")
                    print(f"  Epoch: {final_row.get('epoch', 'N/A')}")
                    print(f"  Train Loss: {final_row.get('train/box_loss', 'N/A'):.4f} (box) + {final_row.get('train/cls_loss', 'N/A'):.4f} (cls) + {final_row.get('train/dfl_loss', 'N/A'):.4f} (dfl)")
                    print(f"  Val Loss: {final_row.get('val/box_loss', 'N/A'):.4f} (box) + {final_row.get('val/cls_loss', 'N/A'):.4f} (cls) + {final_row.get('val/dfl_loss', 'N/A'):.4f} (dfl)")
                    print(f"  mAP50: {final_row.get('metrics/mAP50(B)', 'N/A'):.4f}")
                    print(f"  mAP50-95: {final_row.get('metrics/mAP50-95(B)', 'N/A'):.4f}")
                    print(f"  Precision: {final_row.get('metrics/precision(B)', 'N/A'):.4f}")
                    print(f"  Recall: {final_row.get('metrics/recall(B)', 'N/A'):.4f}")
                    
                    # Show best metrics
                    if 'metrics/mAP50(B)' in df.columns:
                        best_map50_idx = df['metrics/mAP50(B)'].idxmax()
                        best_map50_row = df.iloc[best_map50_idx]
                        print(f"\n🥇 BEST mAP50 (Epoch {best_map50_row.get('epoch', 'N/A')}): {best_map50_row.get('metrics/mAP50(B)', 'N/A'):.4f}")
                    
                    if 'metrics/mAP50-95(B)' in df.columns:
                        best_map50_95_idx = df['metrics/mAP50-95(B)'].idxmax()
                        best_map50_95_row = df.iloc[best_map50_95_idx]
                        print(f"🥇 BEST mAP50-95 (Epoch {best_map50_95_row.get('epoch', 'N/A')}): {best_map50_95_row.get('metrics/mAP50-95(B)', 'N/A'):.4f}")
                
            except Exception as e:
                print(f"Error reading results.csv: {e}")
        
        # Look for other results files
        results_files = list(training_dir.glob("*.yaml")) + list(training_dir.glob("*.json"))
        results_files.extend(list(training_dir.glob("results*.yaml")) + list(training_dir.glob("results*.json")))
        
        if results_files:
            for results_file in results_files:
                if results_file.name != "args.yaml":  # Skip args.yaml as we already processed it
                    print(f"Found results file: {results_file}")
                    try:
                        if results_file.suffix == '.yaml':
                            with open(results_file, 'r') as f:
                                results = yaml.safe_load(f)
                        else:
                            with open(results_file, 'r') as f:
                                results = json.load(f)
                        
                        # Display key metrics
                        if isinstance(results, dict):
                            print(f"Results from {results_file.name}:")
                            for key, value in results.items():
                                if isinstance(value, (int, float)):
                                    print(f"  {key}: {value}")
                                elif isinstance(value, dict):
                                    print(f"  {key}:")
                                    for sub_key, sub_value in value.items():
                                        if isinstance(sub_value, (int, float)):
                                            print(f"    {sub_key}: {sub_value}")
                    except Exception as e:
                        print(f"Error reading {results_file}: {e}")
        
        print()
        
        # Check for other model files in the weights directory
        print("📦 OTHER MODEL FILES:")
        weights_dir = Path(model_path).parent
        weights_files = list(weights_dir.glob("*.pt"))
        for weight_file in weights_files:
            if weight_file.name != "best.pt":  # Skip the one we're testing
                file_size = weight_file.stat().st_size / (1024 * 1024)  # MB
                mod_time = datetime.fromtimestamp(weight_file.stat().st_mtime)
                print(f"  {weight_file.name} ({file_size:.1f} MB, {mod_time.strftime('%Y-%m-%d %H:%M:%S')})")
        
        print()
        
        # Model validation info
        print("🔍 MODEL VALIDATION:")
        try:
            # Try to get model metrics
            if hasattr(model, 'metrics'):
                metrics = model.metrics
                if metrics:
                    print("Available metrics:")
                    for key, value in metrics.items():
                        print(f"  {key}: {value}")
                else:
                    print("No metrics available")
            else:
                print("Model metrics not accessible")
        except Exception as e:
            print(f"Error accessing model metrics: {e}")
        
        print()
        
        # Test inference capability
        print("🧪 INFERENCE TEST:")
        try:
            # Create a dummy image for testing
            dummy_image = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
            
            # Test inference
            start_time = torch.cuda.Event(enable_timing=True) if torch.cuda.is_available() else None
            end_time = torch.cuda.Event(enable_timing=True) if torch.cuda.is_available() else None
            
            if start_time and end_time:
                start_time.record()
            
            results = model(dummy_image, verbose=False)
            
            if end_time:
                end_time.record()
                torch.cuda.synchronize()
                inference_time = start_time.elapsed_time(end_time)
                print(f"Inference time: {inference_time:.2f} ms")
            
            print("✅ Model inference successful")
            
            # Show detection info
            if results and len(results) > 0:
                result = results[0]
                if hasattr(result, 'boxes') and result.boxes is not None:
                    num_detections = len(result.boxes)
                    print(f"Test detections: {num_detections}")
                    
                    if num_detections > 0:
                        confidences = result.boxes.conf.cpu().numpy()
                        classes = result.boxes.cls.cpu().numpy()
                        print(f"Confidence range: {confidences.min():.3f} - {confidences.max():.3f}")
                        print(f"Classes detected: {np.unique(classes)}")
            
        except Exception as e:
            print(f"❌ Inference test failed: {e}")
        
        print()
        print("=" * 60)
        print("Test completed successfully!")
        
    except Exception as e:
        print(f"❌ Error loading model: {e}")
        return False
    
    return True

if __name__ == "__main__":
    success = load_model_and_show_results()
    sys.exit(0 if success else 1) 