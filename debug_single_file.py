#!/usr/bin/env python3
"""
Debug script to test processing a single BDD annotation file
"""

import json
from pathlib import Path
from PIL import Image

def debug_single_file(bdd_path, annotation_file):
    """Debug processing of a single BDD annotation file"""
    bdd_path = Path(bdd_path)
    
    print(f"🔍 Debugging file: {annotation_file}")
    
    # BDD to 11-class mapping
    bdd_to_11class = {
        "car": 2,
        "bus": 1,
        "bicycle": 0,
        "bike": 0,
        "motorcycle": 5,
        "motor": 5,
        "rider": 5,
        "truck": 9,
        "person": 6,
        "traffic light": None,
        "crosswalk": 3,
        "lane/crosswalk": 3
    }
    
    traffic_light_colors = {
        "red": 7,
        "green": 4,
        "yellow": 10
    }
    
    # Load annotation
    with open(annotation_file, 'r') as f:
        data = json.load(f)
    
    print(f"📄 JSON loaded successfully")
    print(f"   Image name: {data['name']}")
    
    # Check image path
    img_name = data["name"]
    img_path = bdd_path / "100k" / "test" / f"{img_name}.jpg"
    
    print(f"🔍 Checking image path: {img_path}")
    print(f"   Image exists: {img_path.exists()}")
    
    if not img_path.exists():
        print(f"❌ Image not found!")
        return
    
    # Get image dimensions
    try:
        with Image.open(img_path) as img:
            img_w, img_h = img.size
        print(f"✅ Image loaded: {img_w}x{img_h}")
    except Exception as e:
        print(f"❌ Error loading image: {e}")
        return
    
    # Process objects
    label_lines = []
    objects = data.get("frames", [{}])[0].get("objects", [])
    
    print(f"📦 Processing {len(objects)} objects...")
    
    for i, obj in enumerate(objects):
        category = obj.get("category", "")
        bbox = obj.get("box2d")
        
        print(f"   Object {i}: {category}")
        
        if not bbox:
            print(f"     ❌ No bbox, skipping")
            continue
        
        # Handle traffic lights specially
        if category == "traffic light":
            color = obj.get("attributes", {}).get("trafficLightColor", "").lower()
            print(f"     🚦 Traffic light color: {color}")
            if color in traffic_light_colors:
                class_id = traffic_light_colors[color]
                print(f"     ✅ Mapped to class {class_id}")
                label_lines.append(f"{class_id} 0.5 0.5 0.1 0.1")  # Placeholder bbox
            else:
                print(f"     ❌ Unknown color, skipping")
        else:
            # Map other classes
            class_id = bdd_to_11class.get(category)
            if class_id is not None:
                print(f"     ✅ Mapped to class {class_id}")
                label_lines.append(f"{class_id} 0.5 0.5 0.1 0.1")  # Placeholder bbox
            else:
                print(f"     ❌ Not in mapping, skipping")
    
    print(f"\n📊 Results:")
    print(f"   Valid detections: {len(label_lines)}")
    print(f"   Label lines: {label_lines}")
    
    if label_lines:
        print(f"✅ This file SHOULD be included!")
    else:
        print(f"❌ This file will be skipped (no valid detections)")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("Usage: python3 debug_single_file.py <bdd_path> <annotation_file>")
        sys.exit(1)
    
    debug_single_file(sys.argv[1], sys.argv[2]) 