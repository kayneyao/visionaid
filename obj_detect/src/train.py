from ultralytics import YOLO
import torch
import os

def main():
    # Check for GPU
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"✅ Using device: {device}")

    # Path to dataset
    data_yaml = r"C:\Users\Sophie\Desktop\VISIONPROJECT (LOCAL)\visionaid\ros_objdetect\data\yolo_dataset\data.yaml"

    # Sanity check for data.yaml path
    if not os.path.exists(data_yaml):
        raise FileNotFoundError(f"❌ data.yaml not found at {data_yaml}")

    # Load a pretrained model
    model = YOLO("yolov8n.pt")  # Options: yolov8n.pt, yolov8s.pt, etc.

    # Train the model
    results = model.train(
        data=data_yaml,
        epochs=50,
        imgsz=640,
        batch=16,
        project="yolo_training",
        name="bdd100k_yolov8",
        exist_ok=True,
        device=device  # explicitly set the device
    )

if __name__ == "__main__":
    main()
