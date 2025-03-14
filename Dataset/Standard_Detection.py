import os
import json
import cv2
from ultralytics import YOLO

# Load YOLOv8 model (change to 'yolov8s.pt' or other variants if needed)
model = YOLO("yolov8n.pt")

# Directory containing images
image_dir = "images"  # Change this to your images folder
output_file = "detections.txt"

# Ensure output directory exists
os.makedirs("output", exist_ok=True)

# List to store results
results_list = []

# Process each image in the directory
for image_name in os.listdir(image_dir):
    if image_name.endswith(('.jpg', '.jpeg', '.png')):  # Process only image files
        image_path = os.path.join(image_dir, image_name)

        # Run YOLOv8 object detection
        results = model(image_path)

        # Store detection data
        detections = []
        for result in results:
            for i, box in enumerate(result.boxes.xywh.cpu().numpy()):  # Move tensor to CPU
                x, y, w, h = box
                label = model.names[int(result.boxes.cls[i].cpu().item())]  # Get class label
                confidence = float(result.boxes.conf[i].cpu().item())  # Confidence score

                detections.append({
                    "label": label,
                    "confidence": confidence,
                    "x": float(x),
                    "y": float(y),
                    "width": float(w),
                    "height": float(h)
                })

        # Store result for this image
        results_list.append({
            "image": image_name,
            "detections": detections
        })

        # Draw bounding boxes on image
        img = cv2.imread(image_path)
        for det in detections:
            x, y, w, h = int(det["x"]), int(det["y"]), int(det["width"]), int(det["height"])
            label = det["label"]
            cv2.rectangle(img, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 255, 0), 2)
            cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Save processed image with bounding boxes
        output_image_path = os.path.join("output", image_name)
        cv2.imwrite(output_image_path, img)

# Save results to a .txt file in JSON format
with open(output_file, "w") as file:
    json.dump(results_list, file, indent=4)

print(f"✅ Object detection completed! Results saved in '{output_file}' and images in 'output/'")
