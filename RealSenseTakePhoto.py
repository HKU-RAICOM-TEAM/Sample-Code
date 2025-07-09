import pyrealsense2 as rs
import numpy as np
import cv2
import os
import re

# Folder to save images
save_folder = "captured_images"
os.makedirs(save_folder, exist_ok=True)

# Function to get the next image number
def get_next_image_number(folder):
    existing_files = os.listdir(folder)
    numbers = []
    for file in existing_files:
        match = re.match(r"image(\d{3})\.png", file)
        if match:
            numbers.append(int(match.group(1)))
    return max(numbers, default=0) + 1

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable color stream
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

print("RealSense camera started. Press SPACE to capture, ESC to quit.")

try:
    next_image_number = get_next_image_number(save_folder)

    while True:
        # Get frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert to image
        color_image = np.asanyarray(color_frame.get_data())

        # Display image
        cv2.imshow("RealSense Color Camera", color_image)

        key = cv2.waitKey(1)

        # Press SPACE to save image
        if key == 32:  # SPACE
            filename = f"image{next_image_number:03}.png"
            filepath = os.path.join(save_folder, filename)
            cv2.imwrite(filepath, color_image)
            print(f"Saved: {filepath}")
            next_image_number += 1

        # Press ESC to exit
        elif key == 27:  # ESC
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
