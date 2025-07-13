import cv2
import numpy as np
import serial
import time

# ==== SERIAL SETUP ====
esp_serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

# ==== HSV COLOR RANGES ====
lower_red_1 = np.array([0, 150, 150])
upper_red_1 = np.array([10, 255, 255])
lower_red_2 = np.array([170, 150, 150])
upper_red_2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 100])
upper_blue = np.array([140, 255, 255])

# ==== CAMERA SETUP ====
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
reference_point = (640, 360)

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
    _, thresh_white = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    _, thresh_black = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    contours_white, _ = cv2.findContours(thresh_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_black, _ = cv2.findContours(thresh_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    white_tiles = [cv2.boundingRect(cnt) for cnt in contours_white if 1000 < cv2.contourArea(cnt) < 50000]
    black_tiles = [cv2.boundingRect(cnt) for cnt in contours_black if 1000 < cv2.contourArea(cnt) < 50000]

    return black_tiles, white_tiles

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1
    frame = cv2.flip(frame, -1)  # Flip vertically (camera is upside down)

    black_tiles, white_tiles = get_black_and_white_tiles(frame)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask_red2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    red_center, red_box = get_largest_contour_center_and_box(mask_red)
    if red_center and red_box:
        cx, cy = red_center
        x, y, w, h = red_box
        for wx, wy, ww, wh in white_tiles:
            if wx < cx < wx + ww and wy < cy < wy + wh:
                cv2.putText(frame, "Drop RED!", (cx + 20, cy + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                print("📤 Sending: PUSH_RED")
                esp_serial.write(b"PUSH_RED\n")
                time.sleep(2)
                break

    blue_center, blue_box = get_largest_contour_center_and_box(mask_blue)
    if blue_center and blue_box:
        cx, cy = blue_center
        x, y, w, h = blue_box
        for bx, by, bw, bh in black_tiles:
            if bx < cx < bx + bw and by < cy < by + bh:
                cv2.putText(frame, "Drop BLUE!", (cx + 20, cy + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                print("📤 Sending: PUSH_BLUE")
                esp_serial.write(b"PUSH_BLUE\n")
                time.sleep(2)
                break

    # Reference point
    cv2.circle(frame, reference_point, 8, (0, 255, 0), -1)
    cv2.putText(frame, "Center", (reference_point[0] - 50, reference_point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Color + Field Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
esp_serial.close()
