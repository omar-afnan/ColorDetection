import cv2
import numpy as np
import math
import time

# Define HSV color ranges
lower_red_1 = np.array([0, 100, 100])
upper_red_1 = np.array([10, 255, 255])

lower_red_2 = np.array([170, 100, 100])
upper_red_2 = np.array([180, 255, 255])

lower_blue = np.array([100, 150, 100])
upper_blue = np.array([140, 255, 255])

# Reference point (e.g., center of the frame)
reference_point = (640, 360)  # Assuming 1280x720 resolution

# Initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Function to find the closest contour and draw borders
def find_closest_contour_with_border(frame, mask, color_name, border_color, text_color):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_distance = float('inf')
    closest_contour = None
    closest_coords = None

    for contour in contours:
        if cv2.contourArea(contour) > 1000:  # Minimum contour area
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                distance = math.sqrt((cx - reference_point[0]) ** 2 + (cy - reference_point[1]) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    closest_contour = contour
                    closest_coords = (cx, cy)

    if closest_contour is not None:
        x, y, w, h = cv2.boundingRect(closest_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), border_color, 3)
        cx, cy = closest_coords
        cv2.putText(frame, f"{color_name} ({cx}, {cy})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
        return closest_coords
    return None

# Function to analyze a single frame
def analyze_frame():
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        return
    
    # Convert to HSV
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for red and blue
    mask_red_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
    mask_red_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
    mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
    mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

    # Morphological operations to reduce noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    # Find closest red object
    closest_red_coords = find_closest_contour_with_border(frame, mask_red, "Red", (0, 0, 0), (0, 0, 255))
    # Find closest blue object
    closest_blue_coords = find_closest_contour_with_border(frame, mask_blue, "Blue", (255, 255, 255), (255, 0, 0))

    # Print coordinates of the closest objects
    if closest_red_coords:
        print(f"Closest Red Object at: {closest_red_coords}")
    if closest_blue_coords:
        print(f"Closest Blue Object at: {closest_blue_coords}")

    # Display the frame (non-blocking)
    cv2.imshow('Closest Object Detection with Borders and Coordinates', frame)
    
    # Use non-blocking wait and close window after 2 seconds
    cv2.waitKey(100)  # Wait for 100ms so that window stays responsive
    time.sleep(2)  # Display for 2 seconds
    cv2.destroyAllWindows()

# Run analysis every 2 minutes
try:
    while True:
        print("Capturing and analyzing frame...")
        analyze_frame()
        time.sleep(120)  # Wait for 2 minutes before capturing the next frame

except KeyboardInterrupt:
    print("Stopped by user")

# Release resources properly
cap.release()
cv2.destroyAllWindows()
