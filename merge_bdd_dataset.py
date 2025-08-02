#!/usr/bin/env python3
"""
BDD Dataset Merger for 11-Class Taiwan Traffic Safety System
Merges BDD100K dataset with existing dataset and remaps classes to 11-class system

Expected BDD100K Dataset Structure:
    bdd_path/
    ├── 100k/
    │   ├── train/ (images: *.jpg)
    │   ├── val/ (images: *.jpg)
    │   └── test/ (images: *.jpg)
    └── bdd100k_labels/
        └── 100k/
            ├── train/ (annotations: *.json)
            ├── val/ (annotations: *.json)
            └── test/ (annotations: *.json)

Usage:
    python merge_bdd_dataset.py --bdd_path /path/to/bdd100k_images_100k --output_path ./merged_dataset
"""

import os
import json
import shutil
from PIL import Image
import argparse
from pathlib import Path
import random
from tqdm import tqdm

class BDDMerger:
    def __init__(self, bdd_path, output_path, existing_dataset_path=None):
        self.bdd_path = Path(bdd_path)
        self.output_path = Path(output_path)
        self.existing_dataset_path = Path(existing_dataset_path) if existing_dataset_path else None
        
        # Your 11-class system mapping
        self.class_names = {
            0: 'bicycle', 1: 'bus', 2: 'car', 3: 'crosswalk',
            4: 'greenlight', 5: 'motorcycle', 6: 'pedestrian',
            7: 'redlight', 8: 'sidewalk', 9: 'truck', 10: 'yellowlight'
        }
        
        # BDD to your 11-class mapping
        self.bdd_to_11class = {
            # Direct matches
            "car": 2,           # BDD car → your car (class 2)
            "bus": 1,           # BDD bus → your bus (class 1)
            "bicycle": 0,       # BDD bicycle → your bicycle (class 0)
            "bike": 0,          # BDD bike → your bicycle (class 0)
            "motorcycle": 5,    # BDD motorcycle → your motorcycle (class 5)
            "motor": 5,         # BDD motor → your motorcycle (class 5)
            "rider": 5,         # BDD rider → your motorcycle (class 5)
            "truck": 9,         # BDD truck → your truck (class 9)
            "person": 6,        # BDD person → your pedestrian (class 6)
            
            # Traffic lights (BDD has color attributes)
            "traffic light": None,  # Will be handled specially based on color
            
            # Infrastructure - ONLY crosswalk (ignore others)
            "crosswalk": 3,      # BDD crosswalk → your crosswalk (class 3)
            "lane/crosswalk": 3, # BDD lane/crosswalk → your crosswalk (class 3)
            
            # IGNORED CLASSES (will be skipped):
            # "traffic sign": None,  # Ignored - not needed for Taiwan system
            # "pole": None,         # Ignored - not needed for Taiwan system  
            # "tree": None,         # Ignored - not needed for Taiwan system
            # "trash can": None,    # Ignored - not needed for Taiwan system
        }
        
        # Traffic light color mapping
        self.traffic_light_colors = {
            "red": 7,      # red light → your redlight (class 7)
            "green": 4,    # green light → your greenlight (class 4)
            "yellow": 10,  # yellow light → your yellowlight (class 10)
        }
        
        # Create output directories
        self.setup_directories()
        
    def setup_directories(self):
        """Create output directory structure"""
        for split in ['train', 'val', 'test']:
            (self.output_path / split / 'images').mkdir(parents=True, exist_ok=True)
            (self.output_path / split / 'labels').mkdir(parents=True, exist_ok=True)
        
        print(f"✅ Created output directories at: {self.output_path}")
    
    def convert_bbox(self, bbox, img_w, img_h):
        """Convert BDD bbox format to YOLO format"""
        x1, y1, x2, y2 = bbox
        x_center = (x1 + x2) / 2 / img_w
        y_center = (y1 + y2) / 2 / img_h
        w = (x2 - x1) / img_w
        h = (y2 - y1) / img_h
        return x_center, y_center, w, h
    
    def process_traffic_light(self, obj):
        """Handle BDD traffic light with color attributes"""
        color = obj.get("attributes", {}).get("trafficLightColor", "").lower()
        if color in self.traffic_light_colors:
            return self.traffic_light_colors[color]
        return None
    
    def process_bdd_annotation(self, annotation_file, split):
        """Process a single BDD annotation file"""
        with open(annotation_file, 'r') as f:
            data = json.load(f)
        
        # Get image info
        img_name = data["name"]
        # Updated path for your dataset structure - add .jpg extension
        img_path = self.bdd_path / "100k" / split / f"{img_name}.jpg"
        
        if not img_path.exists():
            return None
        
        # Get image dimensions
        with Image.open(img_path) as img:
            img_w, img_h = img.size
        
        # Process objects
        label_lines = []
        objects = data.get("frames", [{}])[0].get("objects", [])
        
        for obj in objects:
            category = obj.get("category", "")
            bbox = obj.get("box2d")
            
            if not bbox:
                continue
            
            # Handle traffic lights specially
            if category == "traffic light":
                class_id = self.process_traffic_light(obj)
                if class_id is None:
                    continue
            else:
                # Map other classes
                class_id = self.bdd_to_11class.get(category)
                if class_id is None:
                    continue
            
            # Convert bbox to YOLO format
            bbox_yolo = self.convert_bbox(
                [bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]], 
                img_w, img_h
            )
            
            # Create label line
            label_line = f"{class_id} {' '.join(f'{v:.6f}' for v in bbox_yolo)}"
            label_lines.append(label_line)
        
        return img_path, label_lines
    
    def merge_existing_dataset(self):
        """Merge existing dataset if provided"""
        if not self.existing_dataset_path or not self.existing_dataset_path.exists():
            return
        
        print(f"🔄 Merging existing dataset from: {self.existing_dataset_path}")
        
        for split in ['train', 'val', 'test']:
            existing_img_dir = self.existing_dataset_path / split / 'images'
            existing_lbl_dir = self.existing_dataset_path / split / 'labels'
            
            if not existing_img_dir.exists():
                continue
            
            # Copy existing images and labels
            for img_file in existing_img_dir.glob('*.jpg'):
                lbl_file = existing_lbl_dir / f"{img_file.stem}.txt"
                
                if lbl_file.exists():
                    # Copy to output
                    shutil.copy2(img_file, self.output_path / split / 'images' / img_file.name)
                    shutil.copy2(lbl_file, self.output_path / split / 'labels' / lbl_file.name)
        
        print("✅ Existing dataset merged")
    
    def process_bdd_dataset(self, max_images_per_split=None, balance_classes=True):
        """Process BDD dataset and convert to 11-class format"""
        print("🔄 Processing BDD dataset...")
        
        for split in ['train', 'val']:
            print(f"\n📁 Processing {split} split...")
            
            # BDD paths - updated for your dataset structure
            bdd_img_dir = self.bdd_path / "100k" / split
            bdd_lbl_dir = self.bdd_path / "bdd100k_labels" / "100k" / split
            
            if not bdd_img_dir.exists() or not bdd_lbl_dir.exists():
                print(f"⚠️  Skipping {split} - directories not found")
                print(f"   Images: {bdd_img_dir}")
                print(f"   Labels: {bdd_lbl_dir}")
                continue
            
            # Get annotation files
            annotation_files = list(bdd_lbl_dir.glob('*.json'))
            
            if max_images_per_split:
                annotation_files = random.sample(annotation_files, min(max_images_per_split, len(annotation_files)))
            
            # Class balancing for Taiwan traffic distribution
            if balance_classes:
                annotation_files = self.balance_class_distribution(annotation_files, split)
            
            processed_count = 0
            skipped_count = 0
            
            for ann_file in tqdm(annotation_files, desc=f"Processing {split}"):
                result = self.process_bdd_annotation(ann_file, split)
                
                if result is None:
                    skipped_count += 1
                    continue
                
                img_path, label_lines = result
                
                # Only include images with valid detections
                if label_lines:
                    # Copy image
                    shutil.copy2(img_path, self.output_path / split / 'images' / img_path.name)
                    
                    # Write labels
                    label_file = self.output_path / split / 'labels' / f"{img_path.stem}.txt"
                    with open(label_file, 'w') as f:
                        f.write('\n'.join(label_lines))
                    
                    processed_count += 1
                else:
                    skipped_count += 1
            
            print(f"✅ {split}: {processed_count} images processed, {skipped_count} skipped")
    
    def analyze_existing_dataset(self, split):
        """Analyze the existing dataset to get current class distribution"""
        existing_lbl_dir = self.output_path / split / 'labels'
        if not existing_lbl_dir.exists():
            return {i: 0 for i in range(11)}
        
        class_counts = {i: 0 for i in range(11)}
        total_detections = 0
        
        for lbl_file in existing_lbl_dir.glob('*.txt'):
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
                continue
        
        return class_counts, total_detections
    
    def balance_class_distribution(self, annotation_files, split):
        """Balance class distribution to achieve target distribution in final dataset"""
        print(f"⚖️  Balancing class distribution for {split}...")
        
        # Target distribution for the FINAL merged dataset
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
        
        # Step 1: Analyze existing dataset
        print("📊 Analyzing existing dataset...")
        existing_counts, existing_total = self.analyze_existing_dataset(split)
        
        print(f"📈 Existing dataset distribution:")
        for class_id, count in existing_counts.items():
            if count > 0:
                percentage = (existing_total > 0) and (count / existing_total) * 100 or 0
                print(f"  Class {class_id} ({self.class_names[class_id]}): {count} ({percentage:.1f}%)")
        
        # Step 2: Analyze BDD dataset
        print("📊 Analyzing BDD class distribution...")
        bdd_class_counts = {i: 0 for i in range(11)}
        bdd_class_files = {i: [] for i in range(11)}
        
        for ann_file in tqdm(annotation_files, desc="Analyzing BDD"):
            result = self.process_bdd_annotation(ann_file, split)
            if result is None:
                continue
                
            img_path, label_lines = result
            if not label_lines:
                continue
            
            # Count classes in this image
            for line in label_lines:
                class_id = int(line.split()[0])
                bdd_class_counts[class_id] += 1
                bdd_class_files[class_id].append(ann_file)
        
        print(f"📈 BDD distribution:")
        bdd_total = sum(bdd_class_counts.values())
        for class_id, count in bdd_class_counts.items():
            if count > 0:
                percentage = (bdd_total > 0) and (count / bdd_total) * 100 or 0
                print(f"  Class {class_id} ({self.class_names[class_id]}): {count} ({percentage:.1f}%)")
        
        # Step 3: Calculate how much BDD data we need to add
        print("🧮 Calculating required BDD additions...")
        
        # Estimate total final dataset size (existing + some BDD)
        estimated_final_size = existing_total + min(bdd_total, 50000)  # Reasonable estimate
        
        balanced_files = []
        
        for class_id, target_pct in target_distribution.items():
            target_count = int(estimated_final_size * target_pct)
            current_count = existing_counts.get(class_id, 0)
            needed_count = max(0, target_count - current_count)
            
            available_files = bdd_class_files.get(class_id, [])
            
            if needed_count > 0 and available_files:
                # Select files to reach target
                if len(available_files) > needed_count:
                    selected_files = random.sample(available_files, needed_count)
                else:
                    selected_files = available_files
                
                balanced_files.extend(selected_files)
                print(f"  Class {class_id} ({self.class_names[class_id]}): {len(selected_files)} files selected (need {needed_count}, have {len(available_files)})")
            else:
                print(f"  Class {class_id} ({self.class_names[class_id]}): 0 files selected (need {needed_count}, have {len(available_files)})")
        
        # Remove duplicates and shuffle
        balanced_files = list(set(balanced_files))
        random.shuffle(balanced_files)
        
        print(f"⚖️  Balanced dataset: {len(balanced_files)} files selected")
        return balanced_files
    
    def create_dataset_info(self):
        """Create dataset information file"""
        info = {
            "dataset_name": "Taiwan_11Class_BDD_Merged",
            "classes": self.class_names,
            "class_count": len(self.class_names),
            "description": "Merged dataset combining BDD100K with Taiwan traffic safety system",
            "bdd_mapping": self.bdd_to_11class,
            "traffic_light_colors": self.traffic_light_colors
        }
        
        with open(self.output_path / "dataset_info.json", 'w') as f:
            json.dump(info, f, indent=2)
        
        print("✅ Created dataset_info.json")
    
    def print_statistics(self):
        """Print dataset statistics"""
        print("\n📊 Dataset Statistics:")
        
        for split in ['train', 'val', 'test']:
            img_dir = self.output_path / split / 'images'
            lbl_dir = self.output_path / split / 'labels'
            
            if img_dir.exists():
                img_count = len(list(img_dir.glob('*.jpg')))
                lbl_count = len(list(lbl_dir.glob('*.txt')))
                print(f"  {split}: {img_count} images, {lbl_count} labels")
        
        print(f"\n🎯 11-Class System:")
        for class_id, class_name in self.class_names.items():
            print(f"  {class_id}: {class_name}")

def main():
    parser = argparse.ArgumentParser(description="Merge BDD dataset with 11-class Taiwan system")
    parser.add_argument("--bdd_path", required=True, 
                       help="Path to BDD100K dataset (e.g., /path/to/bdd100k_images_100k)")
    parser.add_argument("--output_path", required=True, help="Output path for merged dataset")
    parser.add_argument("--existing_dataset", help="Path to existing dataset to merge")
    parser.add_argument("--max_images", type=int, help="Maximum images per split to process")
    parser.add_argument("--no_balance", action="store_true", help="Disable class balancing (use original BDD distribution)")
    
    args = parser.parse_args()
    
    # Validate BDD path structure
    bdd_path = Path(args.bdd_path)
    expected_img_dir = bdd_path / "100k"
    expected_lbl_dir = bdd_path / "bdd100k_labels" / "100k"
    
    if not expected_img_dir.exists():
        print(f"❌ Error: Images directory not found at {expected_img_dir}")
        print("Expected structure:")
        print("  bdd_path/")
        print("  ├── 100k/train/ (images)")
        print("  ├── 100k/val/ (images)")
        print("  └── bdd100k_labels/100k/train/ (labels)")
        return
    
    if not expected_lbl_dir.exists():
        print(f"❌ Error: Labels directory not found at {expected_lbl_dir}")
        print("Expected structure:")
        print("  bdd_path/")
        print("  ├── 100k/train/ (images)")
        print("  ├── 100k/val/ (images)")
        print("  └── bdd100k_labels/100k/train/ (labels)")
        return
    
    print(f"✅ BDD dataset structure validated:")
    print(f"   Images: {expected_img_dir}")
    print(f"   Labels: {expected_lbl_dir}")
    
    # Create merger
    merger = BDDMerger(args.bdd_path, args.output_path, args.existing_dataset)
    
    # Merge existing dataset first
    merger.merge_existing_dataset()
    
    # Process BDD dataset
    balance_classes = not args.no_balance
    merger.process_bdd_dataset(args.max_images, balance_classes)
    
    # Create dataset info
    merger.create_dataset_info()
    
    # Print statistics
    merger.print_statistics()
    
    print(f"\n🎉 Dataset merging complete!")
    print(f"📁 Output location: {args.output_path}")
    print(f"📋 Check dataset_info.json for mapping details")

if __name__ == "__main__":
    main() 