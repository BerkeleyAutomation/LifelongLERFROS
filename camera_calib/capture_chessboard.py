import cv2
import os
from datetime import datetime

# Create the directory for calibration images if it doesn't exist
calibration_directory = "./calibration"
if not os.path.exists(calibration_directory):
    os.makedirs(calibration_directory)

# Define the video capture object
cap = cv2.VideoCapture(0)  # 0 is typically the default camera

if not cap.isOpened():
    raise IOError("Cannot open webcam")

# Capture a single frame
ret, frame = cap.read()

if ret:
    # Get the current date and time for unique filename
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    # Save the captured image to the calibration directory with the timestamp in the filename
    filename = f"calibration_image_{timestamp}.jpg"
    filepath = os.path.join(calibration_directory, filename)
    cv2.imwrite(filepath, frame)
    print(f"Image saved to {filepath}")
else:
    print("Failed to capture image")

# Release the video capture object
cap.release()
cv2.destroyAllWindows()
