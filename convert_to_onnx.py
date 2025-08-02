#!/usr/bin/env python3
"""
Convert YOLOv8 PT model to ONNX format
"""

import os
import sys
from pathlib import Path
from ultralytics import YOLO
import torch

def convert_pt_to_onnx():
    """Convert the best.pt model to ONNX format"""
    
    # Model paths
    pt_model_path = "/home/sophie/visionaid-1/yolo_training/11classnew/runs/balanced_augmented_training/balanced_augmented_11class/weights/best.pt"
    output_dir = Path(pt_model_path).parent
    
    print("=" * 60)
    print("🔄 YOLOv8 PT to ONNX Conversion")
    print("=" * 60)
    
    # Check if model exists
    if not os.path.exists(pt_model_path):
        print(f"❌ Model not found: {pt_model_path}")
        return False
    
    print(f"📁 Input model: {pt_model_path}")
    print(f"📁 Output directory: {output_dir}")
    
    # Get model file size
    pt_size = os.path.getsize(pt_model_path) / (1024 * 1024)  # MB
    print(f"📊 Model size: {pt_size:.1f} MB")
    print()
    
    try:
        # Load the model
        print("🔄 Loading YOLOv8 model...")
        model = YOLO(pt_model_path)
        
        # Get model info
        print("📊 Model Information:")
        if hasattr(model, 'names'):
            class_names = model.names
            print(f"  Classes: {len(class_names)}")
            print("  Class names:", list(class_names.values()))
        
        print()
        
        # Convert to ONNX
        print("🔄 Converting to ONNX...")
        onnx_path = output_dir / "best.onnx"
        
        # Export to ONNX
        success = model.export(format="onnx", 
                              imgsz=640, 
                              half=False, 
                              simplify=True,
                              opset=11,
                              dynamic=True,
                              batch=1)
        
        if success:
            # Check if ONNX file was created
            if os.path.exists(onnx_path):
                onnx_size = os.path.getsize(onnx_path) / (1024 * 1024)  # MB
                print(f"✅ ONNX conversion successful!")
                print(f"📁 ONNX file: {onnx_path}")
                print(f"📊 ONNX size: {onnx_size:.1f} MB")
                print(f"📊 Size ratio: {onnx_size/pt_size:.2f}x")
            else:
                # Check for other possible ONNX file names
                onnx_files = list(output_dir.glob("*.onnx"))
                if onnx_files:
                    onnx_path = onnx_files[0]
                    onnx_size = os.path.getsize(onnx_path) / (1024 * 1024)  # MB
                    print(f"✅ ONNX conversion successful!")
                    print(f"📁 ONNX file: {onnx_path}")
                    print(f"📊 ONNX size: {onnx_size:.1f} MB")
                else:
                    print("❌ ONNX file not found after conversion")
                    return False
        else:
            print("❌ ONNX conversion failed")
            return False
        
        print()
        
        # Test ONNX model
        print("🧪 Testing ONNX model...")
        try:
            import onnxruntime as ort
            
            # Create ONNX session
            session = ort.InferenceSession(str(onnx_path))
            
            # Get input/output info
            input_name = session.get_inputs()[0].name
            output_names = [output.name for output in session.get_outputs()]
            
            print(f"✅ ONNX model loaded successfully!")
            print(f"  Input name: {input_name}")
            print(f"  Output names: {output_names}")
            
            # Test inference with dummy data
            import numpy as np
            dummy_input = np.random.randn(1, 3, 640, 640).astype(np.float32)
            
            outputs = session.run(output_names, {input_name: dummy_input})
            print(f"✅ ONNX inference test successful!")
            print(f"  Output shapes: {[output.shape for output in outputs]}")
            
        except ImportError:
            print("⚠️  onnxruntime not installed, skipping ONNX test")
        except Exception as e:
            print(f"⚠️  ONNX test failed: {e}")
        
        print()
        print("=" * 60)
        print("🎉 Conversion completed successfully!")
        print(f"📁 ONNX model: {onnx_path}")
        print("=" * 60)
        
        return True
        
    except Exception as e:
        print(f"❌ Error during conversion: {e}")
        return False

if __name__ == "__main__":
    success = convert_pt_to_onnx()
    sys.exit(0 if success else 1) 