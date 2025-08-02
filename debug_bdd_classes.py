#!/usr/bin/env python3
"""
Debug script to analyze BDD dataset classes
"""

import json
from pathlib import Path
from collections import Counter

def analyze_bdd_classes(bdd_path, max_files=100):
    """Analyze what classes are available in BDD dataset"""
    bdd_path = Path(bdd_path)
    lbl_dir = bdd_path / "bdd100k_labels" / "100k" / "train"
    
    print(f"🔍 Analyzing BDD classes in: {lbl_dir}")
    
    # BDD to 11-class mapping
    bdd_to_11class = {
        "car": 2,
        "bus": 1,
        "bicycle": 0,
        "motorcycle": 5,
        "truck": 9,
        "person": 6,
        "traffic light": None,  # Will be handled separately
        "crosswalk": 3
    }
    
    traffic_light_colors = {
        "red": 7,
        "green": 4,
        "yellow": 10
    }
    
    class_names = {
        0: "bicycle", 1: "bus", 2: "car", 3: "crosswalk", 4: "greenlight",
        5: "motorcycle", 6: "pedestrian", 7: "redlight", 8: "sidewalk",
        9: "truck", 10: "yellowlight"
    }
    
    # Counters
    bdd_classes = Counter()
    mapped_classes = Counter()
    traffic_light_colors_found = Counter()
    files_with_valid_detections = 0
    total_files = 0
    
    annotation_files = list(lbl_dir.glob('*.json'))[:max_files]
    
    for ann_file in annotation_files:
        total_files += 1
        with open(ann_file, 'r') as f:
            data = json.load(f)
        
        valid_detections = False
        objects = data.get("frames", [{}])[0].get("objects", [])
        
        for obj in objects:
            category = obj.get("category", "")
            bdd_classes[category] += 1
            
            # Handle traffic lights
            if category == "traffic light":
                color = obj.get("attributes", {}).get("trafficLightColor", "").lower()
                traffic_light_colors_found[color] += 1
                if color in traffic_light_colors:
                    mapped_classes[traffic_light_colors[color]] += 1
                    valid_detections = True
            else:
                # Map other classes
                if category in bdd_to_11class:
                    class_id = bdd_to_11class[category]
                    if class_id is not None:
                        mapped_classes[class_id] += 1
                        valid_detections = True
        
        if valid_detections:
            files_with_valid_detections += 1
    
    print(f"\n📊 Analysis Results (from {total_files} files):")
    print(f"   Files with valid detections: {files_with_valid_detections}")
    print(f"   Files without valid detections: {total_files - files_with_valid_detections}")
    
    print(f"\n🏷️  BDD Classes Found:")
    for class_name, count in bdd_classes.most_common():
        print(f"   {class_name}: {count}")
    
    print(f"\n🎨 Traffic Light Colors Found:")
    for color, count in traffic_light_colors_found.most_common():
        print(f"   {color}: {count}")
    
    print(f"\n🎯 Mapped to 11-Class System:")
    for class_id, count in mapped_classes.most_common():
        class_name = class_names.get(class_id, f"unknown_{class_id}")
        print(f"   Class {class_id} ({class_name}): {count}")
    
    print(f"\n❌ Classes NOT in our mapping:")
    unmapped = set(bdd_classes.keys()) - set(bdd_to_11class.keys())
    for class_name in unmapped:
        print(f"   {class_name}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python3 debug_bdd_classes.py <bdd_path>")
        sys.exit(1)
    
    analyze_bdd_classes(sys.argv[1]) 