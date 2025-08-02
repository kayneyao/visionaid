#!/usr/bin/env python3
"""
Training Time Estimation
Calculate expected completion time based on current progress
"""

import pandas as pd
from pathlib import Path
from datetime import datetime, timedelta
import numpy as np

def estimate_training_time():
    """Estimate total training time"""
    
    # Training directory
    training_dir = Path("/home/sophie/visionaid-1/yolo_training/11classnew/runs/balanced_augmented_training/balanced_augmented_11class")
    results_file = training_dir / "results.csv"
    
    print("=" * 70)
    print("⏱️  TRAINING TIME ESTIMATION")
    print("=" * 70)
    
    if not results_file.exists():
        print("❌ Results file not found")
        return
    
    try:
        df = pd.read_csv(results_file)
        
        if len(df) < 2:
            print("📋 Need at least 2 epochs for estimation")
            return
        
        # Calculate time per epoch
        time_per_epoch = df['time'].diff().mean()
        current_epoch = df.iloc[-1]['epoch']
        target_epochs = 150
        
        # Calculate remaining epochs
        remaining_epochs = target_epochs - current_epoch
        
        # Estimate remaining time
        remaining_time_seconds = remaining_epochs * time_per_epoch
        remaining_time_hours = remaining_time_seconds / 3600
        
        # Calculate completion time
        current_time = datetime.now()
        completion_time = current_time + timedelta(seconds=remaining_time_seconds)
        
        print(f"📊 CURRENT PROGRESS:")
        print(f"  Epoch: {current_epoch}/{target_epochs} ({current_epoch/target_epochs*100:.1f}%)")
        print(f"  Remaining: {remaining_epochs} epochs")
        print()
        
        print(f"⏱️  TIME ESTIMATES:")
        print(f"  Average time per epoch: {time_per_epoch:.1f} seconds ({time_per_epoch/60:.1f} minutes)")
        print(f"  Remaining time: {remaining_time_hours:.1f} hours ({remaining_time_hours/24:.1f} days)")
        print(f"  Estimated completion: {completion_time.strftime('%Y-%m-%d %H:%M:%S')}")
        print()
        
        # Early stopping analysis
        print("🎯 EARLY STOPPING ANALYSIS:")
        patience = 50  # From training config
        
        # Check if we might hit early stopping
        if len(df) >= 10:
            recent_map50 = df['metrics/mAP50(B)'].tail(10)
            recent_map50_95 = df['metrics/mAP50-95(B)'].tail(10)
            
            # Check for plateau
            map50_improvement = recent_map50.iloc[-1] - recent_map50.iloc[0]
            map50_95_improvement = recent_map50_95.iloc[-1] - recent_map50_95.iloc[0]
            
            print(f"  Recent mAP50 improvement: {map50_improvement:.4f}")
            print(f"  Recent mAP50-95 improvement: {map50_95_improvement:.4f}")
            
            if abs(map50_improvement) < 0.001 and abs(map50_95_improvement) < 0.001:
                print("  ⚠️  Possible plateau detected - early stopping likely")
                early_stop_epochs = current_epoch + patience
                early_stop_time = current_time + timedelta(seconds=patience * time_per_epoch)
                print(f"  Early stop estimate: {early_stop_time.strftime('%Y-%m-%d %H:%M:%S')}")
            else:
                print("  ✅ Good improvement - likely to continue training")
        
        print()
        
        # Performance trends
        print("📈 PERFORMANCE TRENDS:")
        if len(df) >= 3:
            recent_metrics = df.tail(3)
            print("  Recent epochs:")
            for _, row in recent_metrics.iterrows():
                print(f"    Epoch {row['epoch']:.0f}: mAP50={row['metrics/mAP50(B)']:.4f}, mAP50-95={row['metrics/mAP50-95(B)']:.4f}")
        
        # Compare with previous training
        print()
        print("🔄 COMPARISON WITH PREVIOUS TRAINING:")
        prev_training_dir = Path("/home/sophie/visionaid-1/yolo_training/11classnew/runs/merged_dataset_training/merged_dataset_11class")
        prev_results_file = prev_training_dir / "results.csv"
        
        if prev_results_file.exists():
            prev_df = pd.read_csv(prev_results_file)
            if len(prev_df) > 0:
                prev_final_map50 = prev_df['metrics/mAP50(B)'].iloc[-1]
                current_map50 = df['metrics/mAP50(B)'].iloc[-1]
                
                print(f"  Previous training final mAP50: {prev_final_map50:.4f}")
                print(f"  Current training mAP50 (epoch {current_epoch}): {current_map50:.4f}")
                
                if current_map50 > prev_final_map50:
                    print("  🎉 Current training is performing better!")
                else:
                    print("  📊 Current training needs more epochs to match previous")
        
        print()
        print("=" * 70)
        
    except Exception as e:
        print(f"❌ Error in estimation: {e}")

if __name__ == "__main__":
    estimate_training_time() 