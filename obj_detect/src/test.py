from ultralytics import YOLO

# Load trained model
model = YOLO(r"C:\Users\Sophie\Desktop\VISIONPROJECT (LOCAL)\visionaid\yolo_training\bdd100k_yolov8\weights\best.pt")

# Export to ONNX
model.export(format="onnx")
