import cv2
import numpy as np
import time

# HSV color ranges (improved for better red detection)
lower_red_1 = np.array([0, 120, 70])    # First red range (beginning of hue spectrum)
upper_red_1 = np.array([10, 255, 255])
lower_red_2 = np.array([160, 120, 70])  # Second red range (end of hue spectrum)
upper_red_2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 100])
upper_blue = np.array([140, 255, 255])

reference_point = (640, 360)  # Assuming 1280x720 resolution

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def process_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for red and blue colors
    mask_red1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask_red2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Debug visualization - show the masks
    cv2.imshow("Red Mask", mask_red)
    cv2.imshow("Blue Mask", mask_blue)
    
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    def detect_color(mask, color_name, circle_color, text_color):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print(f"❌ No {color_name} objects detected.")
            return

        for contour in contours:
            if cv2.contourArea(contour) < 100:  # Reduced from 300 to detect smaller objects
                continue

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                x, y, w, h = cv2.boundingRect(contour)
                dx, dy = cx - reference_point[0], cy - reference_point[1]

                # Draw visuals
                cv2.circle(frame, (cx, cy), 10, circle_color, -1)
                cv2.rectangle(frame, (x, y), (x + w, y + h), circle_color, 2)
                cv2.putText(frame, f"{color_name}: ({cx},{cy})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)

                # Terminal Output
                print(f"{color_name} Center: ({cx}, {cy}) | Offset: dx={dx}, dy={dy}")

    # Detect multiple objects for Red and Blue
    detect_color(mask_red, "Red", (0, 0, 255), (0, 0, 255))
    detect_color(mask_blue, "Blue", (255, 0, 0), (255, 0, 0))

    # Draw reference point
    cv2.circle(frame, reference_point, 10, (0, 255, 0), -1)
    cv2.putText(frame, "Center", (reference_point[0] - 50, reference_point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Display the frame for a short duration (snapshot style)
    cv2.imshow("Snapshot Detection", frame)
    cv2.waitKey(2000)  # Show for 2 seconds
    cv2.destroyAllWindows()

try:
    while True:
        print("\n📸 Capturing snapshot...")
        ret, frame = cap.read()
        if ret:
            process_frame(frame)
        else:
            print("❌ Failed to capture frame.")

        time.sleep(10)  # Change to 60 when done testing

except KeyboardInterrupt:
    print("🛑 Process interrupted by user.")

cap.release()
cv2.destroyAllWindows()
