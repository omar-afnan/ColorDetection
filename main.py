import cv2
import numpy as np

# === CAMERA SETUP ===
cap = cv2.VideoCapture(0)
cap.set(3, 320)  # Width
cap.set(4, 240)  # Height

# === HSV RANGES (tuned for red cube) ===
red_lower1 = np.array([0, 100, 100])
red_upper1 = np.array([10, 255, 255])
red_lower2 = np.array([160, 100, 100])
red_upper2 = np.array([180, 255, 255])

blue_lower = np.array([100, 150, 50])
blue_upper = np.array([140, 255, 255])

# === HELPER: get largest blob ===
def get_largest_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 400:
        return None, None
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    x, y, w, h = cv2.boundingRect(largest)
    return (cx, cy), (x, y, w, h)

# === MAIN LOOP ===
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Flip 180° (both vertically and horizontally)
    frame = cv2.flip(frame, -1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # RED mask (both hue ranges)
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # BLUE mask
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    # Clean masks
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

    # RED detection
    red_center, red_box = get_largest_contour(red_mask)
    if red_center and red_box:
        cx, cy = red_center
        x, y, w, h = red_box
        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, f"Red ({cx},{cy})", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # BLUE detection
    blue_center, blue_box = get_largest_contour(blue_mask)
    if blue_center and blue_box:
        cx, cy = blue_center
        x, y, w, h = blue_box
        cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(frame, f"Blue ({cx},{cy})", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # Show result
    cv2.imshow("Red/Blue Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# === CLEANUP ===
cap.release()
cv2.destroyAllWindows()
