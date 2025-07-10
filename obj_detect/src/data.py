#convert BDD images data to YOLO format

import os
import json
import shutil
from PIL import Image

# === CONFIG ===

BASE_DIR = r"C:\Users\Sophie\Desktop\VISIONPROJECT (LOCAL)\visionaid\ros_objdetect\data"
IMAGE_DIR = os.path.join(BASE_DIR, "bdd100k_images_100k", "100k")
LABEL_DIR = os.path.join(BASE_DIR, "bdd100k_labels (1)", "100k")
OUTPUT_DIR = os.path.join(BASE_DIR, "yolo_dataset")

# Final YOLO class list with additional mappings
CLASS_MAP = {
    "car": 0,
    "bus": 1,
    "bicycle": 2,
    "pedestrian": 3,
    "pole": 4,
    "tree": 5,
    "trash_bin": 6,
    "crosswalk": 7,
    "road_sign": 8,
    "red_light": 9,
    "green_light": 10,
    "motorcycle": 11
}

# Renaming map from BDD100K raw labels to unified names
RENAME_MAP = {
    "bike": "bicycle",
    "motor": "motorcycle",
    "person": "pedestrian",
    "traffic sign": "road_sign",
    "trash can": "trash_bin"
}

def convert_bbox(bbox, img_w, img_h):
    x, y, w, h = bbox
    x_center = (x + w / 2) / img_w
    y_center = (y + h / 2) / img_h
    w /= img_w
    h /= img_h
    return x_center, y_center, w, h

for split in ["train", "val"]:
    label_path = os.path.join(LABEL_DIR, split)
    image_path = os.path.join(IMAGE_DIR, split)
    output_img = os.path.join(OUTPUT_DIR, split, "images")
    output_lbl = os.path.join(OUTPUT_DIR, split, "labels")

    os.makedirs(output_img, exist_ok=True)
    os.makedirs(output_lbl, exist_ok=True)

    for filename in os.listdir(label_path):
        if not filename.endswith(".json"):
            continue

        json_path = os.path.join(label_path, filename)
        print(f"🟢 Processing: {filename}")

        try:
            with open(json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"⚠️ Skipping {json_path}: {e}")
            continue

        img_file = data.get("name")
        if not img_file:
            continue
        if not img_file.endswith(".jpg"):
            img_file += ".jpg"

        img_path = os.path.join(image_path, img_file)
        if not os.path.exists(img_path):
            print(f"⚠️ Image file not found: {img_path}")
            continue

        try:
            with Image.open(img_path) as img:
                img_w, img_h = img.size
        except Exception as e:
            print(f"⚠️ Could not open image {img_file}: {e}")
            continue

        if "frames" not in data or not data["frames"]:
            continue

        objects = data["frames"][0].get("objects", [])
        label_lines = []

        for obj in objects:
            label = obj["category"]
            bbox = obj.get("box2d")
            if bbox is None:
                continue

            label = RENAME_MAP.get(label, label)

            if label == "traffic light":
                color = obj["attributes"].get("trafficLightColor")
                if color == "red":
                    label = "red_light"
                elif color == "green":
                    label = "green_light"
                else:
                    continue

            if label not in CLASS_MAP:
                continue

            class_id = CLASS_MAP[label]
            x1 = bbox["x1"]
            y1 = bbox["y1"]
            x2 = bbox["x2"]
            y2 = bbox["y2"]
            bbox_yolo = convert_bbox([x1, y1, x2 - x1, y2 - y1], img_w, img_h)
            label_line = f"{class_id} {' '.join(f'{v:.6f}' for v in bbox_yolo)}"
            label_lines.append(label_line)

        if label_lines:
            label_filename = img_file.replace(".jpg", ".txt")
            with open(os.path.join(output_lbl, label_filename), "w") as out_f:
                out_f.write("\n".join(label_lines))

            shutil.copy(img_path, os.path.join(output_img, img_file))

print("✅ Conversion complete. YOLO-formatted dataset saved to:", OUTPUT_DIR)
