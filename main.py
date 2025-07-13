import cv2
import numpy as np
import time

# === HSV Ranges for Red and Blue ===
lower_red_1 = np.array([0, 150, 150])
upper_red_1 = np.array([10, 255, 255])
lower_red_2 = np.array([170, 150, 150])
upper_red_2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 100])
upper_blue = np.array([140, 255, 255])

# === Constants for Distance Calculation ===
KNOWN_WIDTH = 3.9  # cm
FOCAL_LENGTH = 625.33  # adjust for your camera calibration

# === Video Capture Setup ===
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# === Reference Point (Center of Frame) ===
reference_point = (640, 360)

# === Contour Detection Helper ===
def get_largest_contour_center_and_box(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 800:
        return None, None
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    x, y, w, h = cv2.boundingRect(largest)
    return (cx, cy), (x, y, w, h)

# === Distance Estimation ===
def estimate_distance(w):
    return (KNOWN_WIDTH * FOCAL_LENGTH) / w if w > 0 else None

# === Main Loop ===
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Could not read frame")
            break

        frame = cv2.flip(frame, 1)  # Mirror horizontally if needed

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create red and blue masks
        mask_red1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_red2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        # Process each color
        for color_name, mask, color_bgr in [
            ("Red", mask_red, (0, 0, 255)),
            ("Blue", mask_blue, (255, 0, 0))
        ]:
            center, box = get_largest_contour_center_and_box(mask)
            if center and box:
                cx, cy = center
                x, y, w, h = box
                dx, dy = cx - reference_point[0], cy - reference_point[1]
                distance = estimate_distance(w)

                # === Draw Visuals ===
                cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.circle(frame, (cx, cy), 8, color_bgr, -1)
                cv2.putText(frame, f"{color_name}: ({cx},{cy})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

                if distance:
                    cv2.putText(frame, f"{distance:.1f} cm", (x, y + h + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

        # === Reference Point Indicator ===
        cv2.circle(frame, reference_point, 8, (0, 255, 0), -1)
        cv2.putText(frame, "Center", (reference_point[0] - 50, reference_point[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Color Object Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("🛑 Interrupted")

cap.release()
cv2.destroyAllWindows()
