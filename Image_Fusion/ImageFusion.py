import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pandas as pd
import time
from ultralytics import YOLO

# Paths to model and data
ir_folder = Path(r"test_IR")
vi_folder = Path(r"test_VIS")
output_dir = (r"Results")
os.makedirs(output_dir, exist_ok=True)

# Initialize YOLO model
model = YOLO('yolo11x.pt')

# Directories for images
ir_images = sorted(list(ir_folder.glob("*.jpg")))
vi_images = sorted(list(vi_folder.glob("*.jpg")))

# Confidence threshold
CONFIDENCE_THRESHOLD = 0.5

# Function to fuse images with weights
def fuse_images(ir_img, vi_img):
    return cv2.addWeighted(vi_img, 0.7, ir_img, 0.3, 0)

# Categories for evaluation
categories = ["IR", "VI", "Fused"]
category_dirs = {cat: os.path.join(output_dir, cat) for cat in categories}
for cat_dir in category_dirs.values():
    os.makedirs(cat_dir, exist_ok=True)

# Results log
results_log = []

# Store metrics for all categories
category_metrics = {}

# Main loop for each category
for category, folder_images in zip(categories, [ir_images, vi_images, list(zip(ir_images, vi_images))]):
    total_time = 0
    total_images = 0
    total_confidence_sum = 0
    total_objects_detected = 0

    for i, img_path in enumerate(folder_images):
        if category == "Fused":
            # Start timing for fusion
            fusion_start_time = time.time()
            
            # Prepare fused image
            ir_img = cv2.imread(str(img_path[0]))
            vi_img = cv2.imread(str(img_path[1]))
            img = fuse_images(ir_img, vi_img)
            
            # Include fusion time
            fusion_end_time = time.time()
            fusion_time = fusion_end_time - fusion_start_time
            
            img_name = f"fused_{i}.jpg"
        else:
            # Prepare single-modality images
            img = cv2.imread(str(img_path))
            img_name = os.path.basename(str(img_path))
            fusion_time = 0  # No fusion time for single-modality images

        # Resize image to model input size
        img_resized = cv2.resize(img, (640, 640))

        # Perform inference
        start_time = time.time()
        results = model(img_resized, classes=[0])  # Restrict detection to "person"
        end_time = time.time()

        # Total time includes fusion time and inference time
        total_image_time = fusion_time + (end_time - start_time)

        # Extract detections and confidence
        res = results[0]
        boxes = res.boxes.xyxy.cpu().numpy()  # Bounding box coordinates
        scores = res.boxes.conf.cpu().numpy()  # Confidence scores
        classes = res.boxes.cls.cpu().numpy()  # Detected classes

        # Filter detections by confidence threshold
        confident_indices = scores >= CONFIDENCE_THRESHOLD
        confident_scores = scores[confident_indices]

        # Annotate image
        annotated_image = res.plot()

        # Save annotated image
        save_path = os.path.join(category_dirs[category], img_name)
        cv2.imwrite(save_path, annotated_image)

        # Log results
        results_log.append({
            "category": category,
            "image": img_name,
            "detections": len(confident_scores),
            "avg_confidence": confident_scores.mean() if len(confident_scores) > 0 else 0,
            "inference_time": total_image_time,
            "output_image": save_path
        })

        total_time += total_image_time
        total_images += 1
        total_objects_detected += len(confident_scores)
        total_confidence_sum += confident_scores.sum()

    # Calculate metrics for the current category
    avg_time = total_time / total_images if total_images > 0 else 0
    avg_detection_accuracy = np.mean([
        log["avg_confidence"] for log in results_log if log["category"] == category
    ]) * 100

    # Store metrics in a dictionary
    category_metrics[category] = {
        "avg_time": avg_time,
        "detection_accuracy": avg_detection_accuracy
    }

# Print metrics for all categories at the end
print("\n--- Detection Results ---")
for category, metrics in category_metrics.items():
    print(f"{category}: Avg Inference Time: {metrics['avg_time']:.4f}s, Detection Accuracy: {metrics['detection_accuracy']:.2f}%")

# Plot detection accuracy
avg_accuracies = [metrics["detection_accuracy"] for metrics in category_metrics.values()]
plt.figure()
plt.bar(categories, avg_accuracies, label="Detection Accuracy (%)")
plt.title("Detection Accuracy by Category")
plt.ylabel("Accuracy (%)")
plt.xlabel("Category")
plt.legend()
plt.show()

# Plot average inference times
avg_inference_times = [metrics["avg_time"] for metrics in category_metrics.values()]
plt.figure()
plt.bar(categories, avg_inference_times, label="Average Inference Time (s)", color="orange")
plt.title("Average Inference Time by Category")
plt.ylabel("Time (s)")
plt.xlabel("Category")
plt.legend()
plt.show()

# Save log to CSV
df = pd.DataFrame(results_log)
df.to_csv(os.path.join(output_dir, "detection_results_log.csv"), index=False)
print(f"\nResults saved to {os.path.join(output_dir, 'detection_results_log.csv')}")
