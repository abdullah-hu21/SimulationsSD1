import os
import torch
from pathlib import Path
from PIL import Image
import cv2 
# Paths
images_folder = r"images"
output_folder = r"output_images"
model_path = r"best.pt"


# Create output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Load YOLOv5 custom model
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, source='local')

# Process images
for image_file in Path(images_folder).glob("*.*"):
    # Load image
    img = Image.open(image_file)
    
    # Run YOLOv5 inference
    results = model(img)
    
    # Render annotated image
    annotated_img = results.render()[0]
    
    # Save annotated image
    output_path = os.path.join(output_folder, os.path.basename(image_file))
    cv2.imwrite(output_path, cv2.cvtColor(annotated_img, cv2.COLOR_RGB2BGR))

print(f"Annotated images saved in: {output_folder}")
