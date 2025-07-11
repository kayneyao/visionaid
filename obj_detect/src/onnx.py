import onnxruntime as ort
import numpy as np
import cv2

session = ort.InferenceSession(
    r"C:\Users\Sophie\Desktop\VISIONPROJECT (LOCAL)\visionaid\yolo_training\bdd100k_yolov8\weights\best.onnx",
    providers=["CPUExecutionProvider"]
)

input_name = session.get_inputs()[0].name
imgsz = 640

CLASS_NAMES = [
    "car", "bus", "bicycle", "pedestrian", "pole", "tree",
    "trash_bin", "crosswalk", "road_sign", "red_light", "green_light", "motorcycle"
]

CONF_THRESH = 0.25

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    input_img = cv2.resize(frame, (imgsz, imgsz))
    input_img = input_img.transpose(2, 0, 1).astype(np.float32) / 255.0
    input_tensor = np.expand_dims(input_img, axis=0)

    outputs = session.run(None, {input_name: input_tensor})
    preds = outputs[0][0]

    for pred in preds:
        if len(pred) < 6:
            continue
        x1, y1, x2, y2, conf, cls_id = pred[:6]
        if conf < CONF_THRESH:
            continue
        cls_id = int(cls_id)
        if cls_id >= len(CLASS_NAMES):
            continue
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        label = f"{CLASS_NAMES[cls_id]} {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("ONNX Webcam Inference", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
