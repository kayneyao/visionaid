#!/usr/bin/env python3
"""
Monitor Current Training Progress
Shows real-time progress of the balanced augmented training
"""

import os
import time
import pandas as pd
from pathlib import Path
from datetime import datetime

def monitor_training_progress():
    """Monitor the current training progress"""
    
    # Training directory
    training_dir = Path("/home/sophie/visionaid-1/yolo_training/11classnew/runs/balanced_augmented_training/balanced_augmented_11class")
    results_file = training_dir / "results.csv"
    
    print("=" * 70)
    print("BALANCED AUGMENTED TRAINING MONITOR")
    print("=" * 70)
    print(f"Training Directory: {training_dir}")
    print(f"📊 Results File: {results_file}")
    print()
    
    if not results_file.exists():
        print("Results file not found. Training may not have started yet.")
        return
    
    try:
        # Read current results
        df = pd.read_csv(results_file)
        
        if len(df) == 0:
            print("No training data yet. Training may be starting...")
            return
        
        # Get latest metrics
        latest = df.iloc[-1]
        current_epoch = latest['epoch']
        
        print(f"CURRENT EPOCH: {current_epoch}")
        print(f"  Total Training Time: {latest['time']:.1f} seconds ({latest['time']/3600:.2f} hours)")
        print()
        
        print("📈 CURRENT METRICS:")
        print(f"  🎯 mAP50: {latest['metrics/mAP50(B)']:.4f} ({latest['metrics/mAP50(B)']*100:.2f}%)")
        print(f"  🎯 mAP50-95: {latest['metrics/mAP50-95(B)']:.4f} ({latest['metrics/mAP50-95(B)']*100:.2f}%)")
        print(f"  📊 Precision: {latest['metrics/precision(B)']:.4f} ({latest['metrics/precision(B)']*100:.2f}%)")
        print(f"  📊 Recall: {latest['metrics/recall(B)']:.4f} ({latest['metrics/recall(B)']*100:.2f}%)")
        print()
        
        print("📉 LOSS METRICS:")
        print(f"  Train Loss: {latest['train/box_loss']:.4f} (box) + {latest['train/cls_loss']:.4f} (cls) + {latest['train/dfl_loss']:.4f} (dfl)")
        print(f"  Val Loss: {latest['val/box_loss']:.4f} (box) + {latest['val/cls_loss']:.4f} (cls) + {latest['val/dfl_loss']:.4f} (dfl)")
        print()
        
        # Show progress towards target epochs
        target_epochs = 150
        progress = (current_epoch / target_epochs) * 100
        print(f"PROGRESS: {current_epoch}/{target_epochs} epochs ({progress:.1f}%)")
        
        # Show best metrics so far
        if len(df) > 1:
            best_map50_idx = df['metrics/mAP50(B)'].idxmax()
            best_map50 = df.iloc[best_map50_idx]
            best_map50_95_idx = df['metrics/mAP50-95(B)'].idxmax()
            best_map50_95 = df.iloc[best_map50_95_idx]
            
            print()
            print("🏆 BEST METRICS SO FAR:")
            print(f"  🥇 Best mAP50: {best_map50['metrics/mAP50(B)']:.4f} at epoch {best_map50['epoch']}")
            print(f"  🥇 Best mAP50-95: {best_map50_95['metrics/mAP50-95(B)']:.4f} at epoch {best_map50_95['epoch']}")
            
            # Check if current epoch is best
            if current_epoch == best_map50['epoch']:
                print("  🎉 Current epoch has the best mAP50!")
            if current_epoch == best_map50_95['epoch']:
                print("  🎉 Current epoch has the best mAP50-95!")
        
        print()
        
        # Check for model files
        weights_dir = training_dir / "weights"
        if weights_dir.exists():
            model_files = list(weights_dir.glob("*.pt"))
            print(f"💾 MODEL FILES ({len(model_files)}):")
            for model_file in model_files:
                file_size = model_file.stat().st_size / (1024 * 1024)  # MB
                mod_time = datetime.fromtimestamp(model_file.stat().st_mtime)
                print(f"  {model_file.name} ({file_size:.1f} MB, {mod_time.strftime('%H:%M:%S')})")
        
        print()
        print("=" * 70)
        
        # Show recent trend
        if len(df) >= 3:
            print("📈 RECENT TREND (last 3 epochs):")
            recent = df.tail(3)
            for _, row in recent.iterrows():
                print(f"  Epoch {row['epoch']:.0f}: mAP50={row['metrics/mAP50(B)']:.4f}, mAP50-95={row['metrics/mAP50-95(B)']:.4f}")
        
    except Exception as e:
        print(f"Error reading training progress: {e}")

def continuous_monitor():
    """Continuously monitor training progress"""
    print("Starting continuous monitoring (Ctrl+C to stop)...")
    print()
    
    try:
        while True:
            os.system('clear')  # Clear screen
            monitor_training_progress()
            print(f"\nLast updated: {datetime.now().strftime('%H:%M:%S')}")
            print("Refreshing in 30 seconds...")
            time.sleep(30)
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--continuous":
        continuous_monitor()
    else:
        monitor_training_progress() 