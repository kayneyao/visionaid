# BDD Dataset Merger for 11-Class Taiwan Traffic Safety System

This script merges the Berkeley DeepDrive (BDD100K) dataset with your existing Taiwan traffic safety dataset and remaps all classes to fit your 11-class system.

## 🎯 What This Script Does

### **Class Mapping**
Maps BDD classes to your 11-class Taiwan system:

| BDD Class | Your 11-Class System | Class ID |
|-----------|---------------------|----------|
| `car` | `car` | 2 |
| `bus` | `bus` | 1 |
| `bicycle` | `bicycle` | 0 |
| `motorcycle` | `motorcycle` | 5 |
| `truck` | `truck` | 9 |
| `person` | `pedestrian` | 6 |
| `traffic light (red)` | `redlight` | 7 |
| `traffic light (green)` | `greenlight` | 4 |
| `traffic light (yellow)` | `yellowlight` | 10 |
| `crosswalk` | `crosswalk` | 3 |
| `traffic sign` | **IGNORED** | - |
| `pole` | **IGNORED** | - |
| `tree` | **IGNORED** | - |
| `trash can` | **IGNORED** | - |

### **Key Features**
- ✅ **Traffic Light Color Detection**: Automatically detects red/green/yellow traffic lights
- ✅ **Class Balancing**: Balances dataset to match Taiwan traffic patterns (reduces car overrepresentation)
- ✅ **Existing Dataset Merge**: Combines with your current Taiwan dataset
- ✅ **YOLO Format Output**: Creates ready-to-train YOLO format
- ✅ **Progress Tracking**: Shows processing progress with tqdm
- ✅ **Statistics**: Provides detailed dataset statistics
- ✅ **Flexible Limits**: Can limit images per split for testing

## 📋 Prerequisites

```bash
pip install pillow tqdm
```

## 🚀 Usage

### **Test Your Dataset Structure First**
```bash
python test_bdd_structure.py /path/to/bdd100k_images_100k
```

### **Basic Usage (BDD Only)**
```bash
python merge_bdd_dataset.py \
    --bdd_path /path/to/bdd100k_images_100k \
    --output_path ./merged_dataset \
    --max_images 1000
```

### **Merge with Existing Dataset**
```bash
python merge_bdd_dataset.py \
    --bdd_path /path/to/bdd100k_images_100k \
    --output_path ./merged_dataset \
    --existing_dataset /path/to/your/existing/dataset \
    --max_images 2000
```

### **Full Processing (No Limits)**
```bash
python merge_bdd_dataset.py \
    --bdd_path /path/to/bdd100k_images_100k \
    --output_path ./merged_dataset \
    --existing_dataset /path/to/your/existing/dataset
```

## 📁 Expected Directory Structure

### **BDD Dataset Structure**
```
bdd100k_images_100k/
├── 100k/
│   ├── train/ (images: *.jpg)
│   ├── val/ (images: *.jpg)
│   └── test/ (images: *.jpg)
└── bdd100k_labels/
    └── 100k/
        ├── train/ (annotations: *.json)
        ├── val/ (annotations: *.json)
        └── test/ (annotations: *.json)
```

### **Existing Dataset Structure (Optional)**
```
your_dataset/
├── train/
│   ├── images/
│   └── labels/
├── val/
│   ├── images/
│   └── labels/
└── test/
    ├── images/
    └── labels/
```

### **Output Structure**
```
merged_dataset/
├── train/
│   ├── images/
│   └── labels/
├── val/
│   ├── images/
│   └── labels/
├── test/
│   ├── images/
│   └── labels/
└── dataset_info.json
```

## 🔧 Command Line Arguments

| Argument | Required | Description |
|----------|----------|-------------|
| `--bdd_path` | Yes | Path to BDD100K dataset |
| `--output_path` | Yes | Output directory for merged dataset |
| `--existing_dataset` | No | Path to existing dataset to merge |
| `--max_images` | No | Maximum images per split to process |
| `--no_balance` | No | Disable class balancing (use original BDD distribution) |

## 📊 Output Files

### **dataset_info.json**
Contains mapping information and dataset statistics:
```json
{
  "dataset_name": "Taiwan_11Class_BDD_Merged",
  "classes": {
    "0": "bicycle",
    "1": "bus",
    "2": "car",
    "3": "crosswalk",
    "4": "greenlight",
    "5": "motorcycle",
    "6": "pedestrian",
    "7": "redlight",
    "8": "sidewalk",
    "9": "truck",
    "10": "yellowlight"
  },
  "class_count": 11,
  "description": "Merged dataset combining BDD100K with Taiwan traffic safety system",
  "bdd_mapping": {...},
  "traffic_light_colors": {...}
}
```

## 🎯 Why This Helps Your System

### **1. Traffic Light Quality**
- BDD has excellent traffic light annotations with color attributes
- Perfect for your 3-color traffic light system (red/green/yellow)

### **2. Class Balancing for Taiwan Traffic**
- **Problem**: BDD has ~70% cars, but Taiwan has ~20% motorcycles
- **Solution**: Automatic class balancing to match Taiwan traffic patterns
- **Result**: Better training for Taiwan-specific scenarios

### **3. Real-World Diversity**
- Different weather conditions (day/night/rain/snow)
- Various intersection types and angles
- Different lighting conditions

### **4. Easy Integration**
- Direct class mapping to your 11-class system
- YOLO format ready for training
- No manual annotation needed

## ⚖️ Class Balancing Strategy

### **Target Taiwan Distribution:**
- **Motorcycle**: 20% (increased from BDD's ~5%)
- **Car**: 20% (reduced from BDD's ~70%)
- **Bus**: 7% (Taiwan public transport)
- **Truck**: 7% (commercial vehicles)
- **Bicycle**: 7% (active transportation)
- **Pedestrian**: 7% (active transportation)
- **Traffic Lights**: 26% (red: 10%, green: 13%, yellow: 3%)
- **Crosswalk**: 3% (infrastructure)
- **Sidewalk**: 3% (infrastructure)

### **Benefits:**
- **Better Motorcycle Detection**: Critical for Taiwan safety
- **Reduced Car Bias**: Prevents model from being car-centric
- **Balanced Training**: All classes get adequate representation
- **Taiwan-Specific**: Matches real Taiwan traffic patterns

## 🔍 Example Output

```
✅ Created output directories at: ./merged_dataset
🔄 Processing BDD dataset...

📁 Processing train split...
Processing train: 100%|██████████| 1000/1000 [02:30<00:00, 6.67it/s]
✅ train: 850 images processed, 150 skipped

📁 Processing val split...
Processing val: 100%|██████████| 200/200 [00:30<00:00, 6.67it/s]
✅ val: 180 images processed, 20 skipped

✅ Created dataset_info.json

📊 Dataset Statistics:
  train: 850 images, 850 labels
  val: 180 images, 180 labels
  test: 0 images, 0 labels

🎯 11-Class System:
  0: bicycle
  1: bus
  2: car
  3: crosswalk
  4: greenlight
  5: motorcycle
  6: pedestrian
  7: redlight
  8: sidewalk
  9: truck
  10: yellowlight

🎉 Dataset merging complete!
📁 Output location: ./merged_dataset
📋 Check dataset_info.json for mapping details
```

## 🚨 Important Notes

1. **Traffic Light Mapping**: BDD traffic lights are automatically mapped based on color attributes
2. **Ignored Classes**: The following BDD classes are **ignored** and not included in training:
   - `traffic sign` - Not needed for Taiwan traffic safety system
   - `pole` - Not relevant for crossing assistance
   - `tree` - Not relevant for crossing assistance  
   - `trash can` - Not relevant for crossing assistance
3. **Image Limits**: Use `--max_images` for testing, remove for full processing
4. **Memory Usage**: Full BDD processing requires significant disk space (~50GB+)
5. **Processing Time**: Full dataset may take several hours to process

## 🔧 Troubleshooting

### **Common Issues**

1. **BDD Path Not Found**
   ```
   Error: BDD directories not found
   ```
   - Ensure BDD dataset is downloaded and extracted correctly
   - Check path structure matches expected format

2. **Memory Issues**
   ```
   Error: Out of memory
   ```
   - Use `--max_images` to limit processing
   - Process in smaller batches

3. **Permission Errors**
   ```
   Error: Permission denied
   ```
   - Check write permissions for output directory
   - Ensure sufficient disk space

## 📈 Next Steps

After merging:

1. **Train Your Model**:
   ```bash
   # Use the merged dataset for training
   python train.py --data merged_dataset/dataset_info.json
   ```

2. **Validate Results**:
   - Check class distribution in merged dataset
   - Verify traffic light detection quality
   - Test on your Taiwan scenarios

3. **Fine-tune**:
   - Adjust class mappings if needed
   - Add more Taiwan-specific data
   - Optimize for your specific use case

## 🤝 Contributing

Feel free to modify the class mappings or add additional features:
- Add new BDD class mappings
- Implement data augmentation
- Add validation checks
- Optimize processing speed 