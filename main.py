import cv2
import numpy as np
import serial
import time

# === SERIAL SETUP ===
esp_serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

# === HSV COLOR RANGES ===
lower_red_1 = np.array([0, 120, 70])
upper_red_1 = np.array([10, 255, 255])
lower_red_2 = np.array([160, 120, 70])
upper_red_2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 100])
upper_blue = np.array([140, 255, 255])

# === CAMERA SETUP ===
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

reference_point = (640, 360)
KNOWN_WIDTH = 3.9
FOCAL_LENGTH = 625.33

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

def get_black_and_white_tiles(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, white_thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    _, black_thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    contours_white, _ = cv2.findContours(white_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_black, _ = cv2.findContours(black_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    white_tiles = [cv2.boundingRect(c) for c in contours_white if 1000 < cv2.contourArea(c) < 50000]
    black_tiles = [cv2.boundingRect(c) for c in contours_black if 1000 < cv2.contourArea(c) < 50000]
    return black_tiles, white_tiles

# === MAIN LOOP ===
frame_count = 0
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, -1)  # Flip if cam is mounted upside down
        frame_count += 1

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_red2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        black_tiles, white_tiles = get_black_and_white_tiles(frame)

        for color_name, mask, color_bgr, tile_zone, push_cmd in [
            ("Red", mask_red, (0, 0, 255), white_tiles, "PUSH_RED"),
            ("Blue", mask_blue, (255, 0, 0), black_tiles, "PUSH_BLUE")
        ]:
            center, box = get_largest_contour_center_and_box(mask)
            if center and box:
                cx, cy = center
                x, y, w, h = box

                # Draw
                cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.circle(frame, (cx, cy), 8, color_bgr, -1)
                cv2.putText(frame, f"{color_name}: ({cx},{cy})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

                for tx, ty, tw, th in tile_zone:
                    if tx < cx < tx + tw and ty < cy < ty + th:
                        cv2.putText(frame, f"Drop {color_name.upper()}!", (cx + 20, cy + 40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        if frame_count % 20 == 0:  # Throttle serial spam
                            print(f"📤 Sending: {push_cmd}")
                            esp_serial.write(f"{push_cmd}\n".encode())
                        break

        cv2.circle(frame, reference_point, 8, (0, 255, 0), -1)
        cv2.putText(frame, "Center", (reference_point[0] - 50, reference_point[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("🛑 Interrupted by user.")

cap.release()
cv2.destroyAllWindows()
esp_serial.close()
