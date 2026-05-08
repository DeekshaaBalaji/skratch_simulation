import cv2
import numpy as np
import os
import json

# Absolute Paths
input_folder = '/home/deekshaa/Lego_labelling/Lego_Images'
output_folder = '/home/deekshaa/Lego_labelling/Lego_Labels_Output'

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Processing the first 10 for your check
all_files = sorted([f for f in os.listdir(input_folder) if f.lower().endswith(('.jpg', '.png', '.jpeg'))])

colors = {
    "red":    [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [180, 255, 255])],
    "blue":   [([90, 80, 50], [130, 255, 255])],
    "green":  [([35, 60, 40], [85, 255, 255])],
    "yellow": [([20, 100, 100], [35, 255, 255])]
}

for filename in all_files:
    img = cv2.imread(os.path.join(input_folder, filename))
    if img is None: continue
    
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    display_img = img.copy()
    shapes = []

    for color_name, ranges in colors.items():
        full_mask = np.zeros(hsv.shape[:2], dtype="uint8")
        for (low, high) in ranges:
            mask = cv2.inRange(hsv, np.array(low), np.array(high))
            full_mask = cv2.bitwise_or(full_mask, mask)

        # Keep the existing logic that you confirmed is correct
        kernel = np.ones((3,3), np.uint8)
        full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > 600:
                rect = cv2.minAreaRect(cnt)
                (rcx, rcy), (rw, rh), angle = rect
                
                # Determine Label
                aspect_ratio = max(rw, rh) / min(rw, rh)
                size_type = "4x2" if aspect_ratio > 1.5 else "2x2"
                label_name = f"{size_type}_{color_name}_block"

                # Box Points
                box_points = cv2.boxPoints(((rcx, rcy), (rw, rh), angle))
                box_int = np.int0(box_points)

                # 1. DRAW BOX
                cv2.drawContours(display_img, [box_int], 0, (0, 255, 0), 2)
                
                # 2. DRAW LABEL (Moved slightly inside/above the box to ensure visibility)
                # We use the top-most point of the box to place the text
                text_y = box_int[1][1] - 10 if box_int[1][1] > 20 else box_int[1][1] + 20
                cv2.putText(display_img, label_name, (box_int[1][0], text_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                shapes.append({
                    "label": label_name,
                    "points": box_points.tolist(),
                    "shape_type": "rotation",
                    "direction": float(angle),
                    "flags": {},
                    "attributes": {}
                })

    # Save outputs
    cv2.imwrite(os.path.join(output_folder, f"label_check_{filename}"), display_img)
    
    json_output = {
        "version": "2.3.6",
        "flags": {},
        "shapes": shapes,
        "imagePath": filename,
        "imageData": None,
        "imageHeight": h,
        "imageWidth": w,
        "text": ""
    }
    
    with open(os.path.join(output_folder, os.path.splitext(filename)[0] + ".json"), 'w') as f:
        json.dump(json_output, f, indent=2)

print(f"Check complete. Verify the green labels in the JPGs in: {output_folder}")