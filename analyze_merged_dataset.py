#!/usr/bin/env python3
"""
Analyze class distribution in merged dataset
"""

import json
from pathlib import Path
from collections import Counter

def analyze_merged_dataset(dataset_path):
    """Analyze class distribution in merged dataset"""
    dataset_path = Path(dataset_path)
    
    print(f"🔍 Analyzing merged dataset at: {dataset_path}")
    
    class_names = {
        0: "bicycle", 1: "bus", 2: "car", 3: "crosswalk", 4: "greenlight",
        5: "motorcycle", 6: "pedestrian", 7: "redlight", 8: "sidewalk",
        9: "truck", 10: "yellowlight"
    }
    
    # Counters for each split
    split_stats = {}
    
    for split in ['train', 'val', 'test']:
        print(f"\n📁 Analyzing {split} split...")
        
        lbl_dir = dataset_path / split / 'labels'
        if not lbl_dir.exists():
            print(f"   ❌ Labels directory not found: {lbl_dir}")
            continue
        
        # Count classes
        class_counts = Counter()
        total_images = 0
        total_detections = 0
        
        label_files = list(lbl_dir.glob('*.txt'))
        print(f"   📄 Found {len(label_files)} label files")
        
        for lbl_file in label_files:
            total_images += 1
            try:
                with open(lbl_file, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line:
                            parts = line.split()
                            if len(parts) >= 5:
                                class_id = int(parts[0])
                                class_counts[class_id] += 1
                                total_detections += 1
            except Exception as e:
                print(f"   ⚠️  Error reading {lbl_file}: {e}")
        
        split_stats[split] = {
            'images': total_images,
            'detections': total_detections,
            'class_counts': class_counts
        }
        
        print(f"   📊 {split} Statistics:")
        print(f"      Images: {total_images}")
        print(f"      Total detections: {total_detections}")
        
        if total_detections > 0:
            print(f"      Class distribution:")
            for class_id in sorted(class_counts.keys()):
                count = class_counts[class_id]
                percentage = (count / total_detections) * 100
                class_name = class_names.get(class_id, f"unknown_{class_id}")
                print(f"        Class {class_id} ({class_name}): {count} ({percentage:.1f}%)")
    
    # Overall statistics
    print(f"\n🎯 OVERALL DATASET STATISTICS:")
    total_images = sum(stats['images'] for stats in split_stats.values())
    total_detections = sum(stats['detections'] for stats in split_stats.values())
    
    print(f"   Total images: {total_images}")
    print(f"   Total detections: {total_detections}")
    
    # Combined class distribution
    combined_counts = Counter()
    for stats in split_stats.values():
        combined_counts.update(stats['class_counts'])
    
    if total_detections > 0:
        print(f"   Overall class distribution:")
        for class_id in sorted(combined_counts.keys()):
            count = combined_counts[class_id]
            percentage = (count / total_detections) * 100
            class_name = class_names.get(class_id, f"unknown_{class_id}")
            print(f"     Class {class_id} ({class_name}): {count} ({percentage:.1f}%)")
    
    # Compare with target distribution
    target_distribution = {
        0: 0.07,   # bicycle: 7%
        1: 0.07,   # bus: 7%
        2: 0.20,   # car: 20%
        3: 0.03,   # crosswalk: 3%
        4: 0.13,   # greenlight: 13%
        5: 0.20,   # motorcycle: 20%
        6: 0.07,   # pedestrian: 7%
        7: 0.10,   # redlight: 10%
        8: 0.03,   # sidewalk: 3%
        9: 0.07,   # truck: 7%
        10: 0.03   # yellowlight: 3%
    }
    
    print(f"\n📈 COMPARISON WITH TARGET DISTRIBUTION:")
    for class_id in sorted(combined_counts.keys()):
        actual_count = combined_counts[class_id]
        actual_pct = (actual_count / total_detections) * 100 if total_detections > 0 else 0
        target_pct = target_distribution.get(class_id, 0) * 100
        class_name = class_names.get(class_id, f"unknown_{class_id}")
        
        diff = actual_pct - target_pct
        status = "✅" if abs(diff) < 5 else "⚠️" if abs(diff) < 10 else "❌"
        
        print(f"   {status} Class {class_id} ({class_name}):")
        print(f"      Target: {target_pct:.1f}% | Actual: {actual_pct:.1f}% | Diff: {diff:+.1f}%")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python3 analyze_merged_dataset.py <dataset_path>")
        print("Example: python3 analyze_merged_dataset.py ./merged_dataset_large")
        sys.exit(1)
    
    analyze_merged_dataset(sys.argv[1]) 