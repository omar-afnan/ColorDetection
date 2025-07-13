import cv2
import serial
import time
import numpy as np

# === SERIAL SETUP ===
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port if needed
time.sleep(2)

def send_command(cmd):
    print(f"[RPi] Sending: {cmd}")
    ser.write((cmd + '\n').encode())

# === CAMERA SETUP ===
cap = cv2.VideoCapture(0)
cap.set(3, 320)  # Width
cap.set(4, 240)  # Height

# === HSV RANGES (Improved for red cube) ===
red_lower1 = np.array([0, 100, 100])
red_upper1 = np.array([10, 255, 255])
red_lower2 = np.array([160, 100, 100])
red_upper2 = np.array([180, 255, 255])

blue_lower = np.array([100, 150, 50])
blue_upper = np.array([140, 255, 255])

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Flip frame vertically and horizontally (180° rotation)
    frame = cv2.flip(frame, -1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create color masks
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    # Use morphological operations to clean up masks
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

    # Count colored pixels
    red_pixels = cv2.countNonZero(red_mask)
    blue_pixels = cv2.countNonZero(blue_mask)

    # Send commands based on detections
    if red_pixels > 500:
        send_command("PUSH_RED")
    elif blue_pixels > 500:
        send_command("PUSH_BLUE")
    else:
        send_command("SCAN")

    # Show processed frame (for debug)
    cv2.imshow("View", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ==== CLEANUP ====
cap.release()
cv2.destroyAllWindows()
ser.close()
