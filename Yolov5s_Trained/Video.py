import cv2
import torch
from tkinter import Tk, filedialog

# Load YOLOv5 model
model_path = r"best.pt"
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

# Open file dialog to choose a video
root = Tk()
root.withdraw()  # Hide the root window
video_path = filedialog.askopenfilename(title="Select a Video File",
                                        filetypes=[("Video Files", "*.mp4 *.avi *.mkv")])
if not video_path:
    print("No video selected. Exiting.")
    exit()

# Open video
cap = cv2.VideoCapture(video_path)

# Process video frame by frame
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLOv5 inference
    results = model(frame)

    # Get annotated frame
    annotated_frame = results.render()[0]

    # Display frame
    cv2.imshow('YOLOv5 Video Analysis', annotated_frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
