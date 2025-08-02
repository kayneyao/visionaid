#!/usr/bin/env python3
"""
Usage Examples for BDD Dataset Merger
Shows different ways to merge BDD with your 11-class Taiwan system
"""

import subprocess
import sys
from pathlib import Path

def run_merge_command(cmd_args):
    """Run the merge command and show output"""
    print(f"🚀 Running: python merge_bdd_dataset.py {' '.join(cmd_args)}")
    print("=" * 60)
    
    try:
        result = subprocess.run(
            [sys.executable, "merge_bdd_dataset.py"] + cmd_args,
            capture_output=True,
            text=True,
            check=True
        )
        print(result.stdout)
        if result.stderr:
            print("Warnings/Errors:", result.stderr)
    except subprocess.CalledProcessError as e:
        print(f"❌ Error: {e}")
        print(f"Error output: {e.stderr}")
        return False
    
    print("=" * 60)
    return True

def main():
    print("🎯 BDD Dataset Merger - Usage Examples")
    print("This script shows different ways to merge BDD with your 11-class system\n")
    
    # Example 1: Basic BDD merge (no existing dataset)
    print("📋 Example 1: Basic BDD Merge (No Existing Dataset)")
    print("This will process BDD and create a new 11-class dataset")
    
    example1_cmd = [
        "--bdd_path", "/path/to/bdd100k",
        "--output_path", "./merged_dataset_basic",
        "--max_images", "1000"  # Limit to 1000 images per split for testing
    ]
    
    print("Command:", " ".join(example1_cmd))
    print("Press Enter to run this example, or Ctrl+C to skip...")
    try:
        input()
        run_merge_command(example1_cmd)
    except KeyboardInterrupt:
        print("Skipped Example 1\n")
    
    # Example 2: Merge BDD with existing dataset
    print("📋 Example 2: Merge BDD with Existing Dataset")
    print("This will combine your existing Taiwan dataset with BDD data")
    
    example2_cmd = [
        "--bdd_path", "/path/to/bdd100k",
        "--output_path", "./merged_dataset_complete",
        "--existing_dataset", "/path/to/your/existing/dataset",
        "--max_images", "2000"  # More images for better training
    ]
    
    print("Command:", " ".join(example2_cmd))
    print("Press Enter to run this example, or Ctrl+C to skip...")
    try:
        input()
        run_merge_command(example2_cmd)
    except KeyboardInterrupt:
        print("Skipped Example 2\n")
    
    # Example 3: Full BDD processing (no limits)
    print("📋 Example 3: Full BDD Processing (No Limits)")
    print("This will process the entire BDD dataset (may take a while)")
    
    example3_cmd = [
        "--bdd_path", "/path/to/bdd100k",
        "--output_path", "./merged_dataset_full",
        "--existing_dataset", "/path/to/your/existing/dataset"
        # No max_images limit - processes everything
    ]
    
    print("Command:", " ".join(example3_cmd))
    print("Press Enter to run this example, or Ctrl+C to skip...")
    try:
        input()
        run_merge_command(example3_cmd)
    except KeyboardInterrupt:
        print("Skipped Example 3\n")
    
    print("✅ All examples completed!")
    print("\n📝 Manual Usage:")
    print("python merge_bdd_dataset.py --bdd_path /path/to/bdd100k --output_path ./output")
    print("\n📝 With existing dataset:")
    print("python merge_bdd_dataset.py --bdd_path /path/to/bdd100k --output_path ./output --existing_dataset /path/to/existing")
    print("\n📝 With image limits:")
    print("python merge_bdd_dataset.py --bdd_path /path/to/bdd100k --output_path ./output --max_images 1000")

if __name__ == "__main__":
    main() 