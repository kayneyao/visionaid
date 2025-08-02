#!/usr/bin/env python3
"""
Test script to verify BDD100K dataset structure
"""

import json
from pathlib import Path

def test_bdd_structure(bdd_path):
    """Test if the BDD dataset structure is correct"""
    bdd_path = Path(bdd_path)
    
    print(f"🔍 Testing BDD dataset structure at: {bdd_path}")
    
    # Check main directories
    img_dir = bdd_path / "100k"
    lbl_dir = bdd_path / "bdd100k_labels" / "100k"
    
    print(f"\n📁 Checking directories:")
    print(f"   Images: {img_dir} - {'✅' if img_dir.exists() else '❌'}")
    print(f"   Labels: {lbl_dir} - {'✅' if lbl_dir.exists() else '❌'}")
    
    if not img_dir.exists() or not lbl_dir.exists():
        print("\n❌ Dataset structure is incorrect!")
        return False
    
    # Check splits
    splits = ['train', 'val', 'test']
    for split in splits:
        split_img_dir = img_dir / split
        split_lbl_dir = lbl_dir / split
        
        print(f"\n📂 Split: {split}")
        print(f"   Images: {split_img_dir} - {'✅' if split_img_dir.exists() else '❌'}")
        print(f"   Labels: {split_lbl_dir} - {'✅' if split_lbl_dir.exists() else '❌'}")
        
        if split_img_dir.exists():
            img_count = len(list(split_img_dir.glob('*.jpg')))
            print(f"   Image count: {img_count}")
        
        if split_lbl_dir.exists():
            lbl_count = len(list(split_lbl_dir.glob('*.json')))
            print(f"   Label count: {lbl_count}")
    
    # Test a sample annotation file
    print(f"\n🔍 Testing sample annotation file...")
    sample_lbl_dir = lbl_dir / "train"
    if sample_lbl_dir.exists():
        sample_files = list(sample_lbl_dir.glob('*.json'))
        if sample_files:
            sample_file = sample_files[0]
            print(f"   Sample file: {sample_file.name}")
            
            try:
                with open(sample_file, 'r') as f:
                    data = json.load(f)
                
                print(f"   ✅ JSON loaded successfully")
                print(f"   Image name: {data.get('name', 'N/A')}")
                print(f"   Frame count: {len(data.get('frames', []))}")
                
                if data.get('frames'):
                    frame = data['frames'][0]
                    objects = frame.get('objects', [])
                    print(f"   Object count: {len(objects)}")
                    
                    # Show object categories
                    categories = set()
                    for obj in objects:
                        category = obj.get('category', 'unknown')
                        categories.add(category)
                    
                    print(f"   Categories found: {sorted(list(categories))}")
                
            except Exception as e:
                print(f"   ❌ Error reading sample file: {e}")
        else:
            print("   ❌ No annotation files found")
    else:
        print("   ❌ Train labels directory not found")
    
    print(f"\n✅ BDD dataset structure test completed!")
    return True

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) != 2:
        print("Usage: python test_bdd_structure.py <bdd_path>")
        print("Example: python test_bdd_structure.py /path/to/bdd100k_images_100k")
        sys.exit(1)
    
    bdd_path = sys.argv[1]
    test_bdd_structure(bdd_path) 