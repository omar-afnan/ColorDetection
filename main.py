import cv2
import numpy as np
# import serial  # Uncomment if using serial
import time

# ==== SERIAL SETUP ====
# Uncomment if needed
# esp_serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
# time.sleep(2)

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

# ==== TILE DETECTION ====
def detect_field_squares(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    _, thresh_white = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
    _, thresh_black = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)

    contours_white, _ = cv2.findContours(thresh_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_black, _ = cv2.findContours(thresh_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    black_tiles = []
    white_tiles = []

    for cnt in contours_black:
        area = cv2.contourArea(cnt)
        if 1000 < area < 50000:
            x, y, w, h = cv2.boundingRect(cnt)
            black_tiles.append((x, y, w, h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (50, 50, 50), 2)
            cv2.putText(frame, "Black", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 50, 50), 1)

    for cnt in contours_white:
        area = cv2.contourArea(cnt)
        if 1000 < area < 50000:
            x, y, w, h = cv2.boundingRect(cnt)
            white_tiles.append((x, y, w, h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (200, 200, 200), 2)
            cv2.putText(frame, "White", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    return black_tiles, white_tiles

# ==== CONTOUR DETECTION ====
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

# ==== MAIN LOOP ====
frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.flip(frame, 1)
    frame_count += 1

    # Detect tiles
    black_tiles, white_tiles = detect_field_squares(frame)

    # Create HSV masks
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask_red2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    # RED detection
    red_center, red_box = get_largest_contour_center_and_box(mask_red)
    if red_center and red_box:
        cx, cy = red_center
        x, y, w, h = red_box
        dx, dy = cx - reference_point[0], cy - reference_point[1]

        cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, f"Red: ({cx},{cy})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        for wx, wy, ww, wh in white_tiles:
            if wx < cx < wx + ww and wy < cy < wy + wh:
                cv2.putText(frame, "Drop RED!", (cx + 20, cy + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                print(" Command: DROP_RED")
                # esp_serial.write(b"DROP_RED\n")  # Uncomment to send
                break

        if frame_count % 10 == 0:
            print(f"Red Center: ({cx}, {cy}) | Offset: dx={dx}, dy={dy}")

    # BLUE detection
    blue_center, blue_box = get_largest_contour_center_and_box(mask_blue)
    if blue_center and blue_box:
        cx, cy = blue_center
        x, y, w, h = blue_box
        dx, dy = cx - reference_point[0], cy - reference_point[1]

        cv2.circle(frame, (cx, cy), 10, (255, 0, 0), -1)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(frame, f"Blue: ({cx},{cy})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        for bx, by, bw, bh in black_tiles:
            if bx < cx < bx + bw and by < cy < by + bh:
                cv2.putText(frame, "Drop BLUE!", (cx + 20, cy + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                print(" Command: DROP_BLUE")
                # esp_serial.write(b"DROP_BLUE\n")  # Uncomment to send
                break

        if frame_count % 10 == 0:
            print(f"Blue Center: ({cx}, {cy}) | Offset: dx={dx}, dy={dy}")

    # Show output
    cv2.circle(frame, reference_point, 8, (0, 255, 0), -1)
    cv2.putText(frame, "Center", (reference_point[0] - 50, reference_point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Color + Field Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ==== CLEANUP ====
cap.release()
cv2.destroyAllWindows()
# esp_serial.close()  # Uncomment if using serial
