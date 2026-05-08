# Scripts Folder

This folder contains Jupyter notebooks and pre-trained model weights for the perception module of the SKRATCH simulation project. The scripts focus on YOLO (You Only Look Once)-based object detection and data preprocessing for Lego labelling tasks in robotic simulations. 

The required lego dataset is available in sciebo.

## Overview

The scripts provide a complete pipeline for:
1. Data preprocessing and annotation conversion
2. Dataset splitting for training/validation
3. YOLO model training
4. Object detection predictions

All scripts use the Ultralytics YOLO library and are designed to work with image datasets for object detection in the SKRATCH Gazebo simulation environment.

## Notebooks

### Data Preparation
- **`data_preprocessing.ipynb`**: Processes raw images using a trained YOLO model to generate JSON label annotations. Loads a pre-trained model and applies detection to create training labels from input images.
  
- **`pre_processing.ipynb`**: Converts existing JSON label files to YOLO format (.txt files with normalized coordinates). Includes functions to transform bounding box annotations for YOLO training.

- **`split_train_val.ipynb`**: Splits the dataset into training and validation sets. Randomly distributes images and corresponding label files into separate folders for model training and evaluation.

### Training
- **`go25_yolo_train.ipynb`**: Trains a YOLO model on the prepared dataset. Configures the model, loads data, and performs training iterations. (Appears to be customized for GO25 or similar dataset variant.)

### Prediction
- **`yolo_pred.ipynb`**: Performs object detection on new images using a trained YOLO model. Loads the model and runs inference to detect and classify objects.

- **`yolo_pred_empty.ipynb`**: Specialized prediction script for detecting objects in empty or sparse scenes. Optimized for scenarios with minimal objects present.

- **`empty_pred.ipynb`**: Another prediction script focused on empty space detection, using a model trained specifically for identifying unoccupied areas.

## Model Weights

Pre-trained YOLO model weights stored as PyTorch (.pt) files:

- **`yolo11n-obb.pt`**: YOLOv11 nano model optimized for oriented bounding box (OBB) detection. Suitable for detecting objects at various angles.

- **`yolov8n-obb.pt`**: YOLOv8 nano model for oriented bounding box detection. Lightweight version for efficient inference.

- **`yolov8n.pt`**: Standard YOLOv8 nano model for general object detection tasks.

## Downloading Model Weights

If the weight files are not present, they can be downloaded using the Ultralytics YOLO library or manually via wget. The library will automatically download them when first used.

### Using Python (Recommended)
```python
from ultralytics import YOLO

# Download and load YOLOv8n
model = YOLO('yolov8n.pt')

# Download and load YOLOv8n-obb
model = YOLO('yolov8n-obb.pt')

# Download and load YOLOv11n-obb
model = YOLO('yolo11n-obb.pt')
```

### Using Command Line
```bash
# Install ultralytics if not already installed
pip install ultralytics

# Download weights (this will cache them locally)
yolo predict model=yolov8n.pt source='path/to/image.jpg'
yolo predict model=yolov8n-obb.pt source='path/to/image.jpg'
yolo predict model=yolo11n-obb.pt source='path/to/image.jpg'
```

### Manual Download (Alternative)
If you prefer to download manually:
```bash
# YOLOv8n
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt

# YOLOv8n-obb
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n-obb.pt

# YOLOv11n-obb (check latest release for exact URL)
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolo11n-obb.pt
```

Note: Check the [Ultralytics releases](https://github.com/ultralytics/assets/releases) for the latest version URLs.

## Dependencies

- PyTorch
- Ultralytics YOLO
- PIL (Pillow)
- NumPy
- torchvision

## Usage Workflow

1. **Prepare Data**: Run `data_preprocessing.ipynb` to generate initial labels, then `pre_processing.ipynb` to convert formats.
2. **Split Dataset**: Use `split_train_val.ipynb` to create train/validation splits.
3. **Train Model**: Execute `go25_yolo_train.ipynb` to train a custom YOLO model.
4. **Run Predictions**: Use prediction notebooks (`yolo_pred.ipynb`, `yolo_pred_empty.ipynb`, `empty_pred.ipynb`) for inference on new data.

## Notes

- Ensure CUDA-compatible GPU is available for training and inference acceleration.
- Model weights are large binary files; consider using Git LFS for version control or store externally.
- Paths in scripts may need adjustment based on your local directory structure.
- All scripts assume YOLO format labels (class x_center y_center width height, normalized 0-1).</content>
<parameter name="filePath">/home/deekshaa/Lego_labelling/skratch_simulation/skratch_perception/scripts/README.md