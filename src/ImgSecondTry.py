import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
motor_left_forward = 17
motor_left_backward = 18
motor_right_forward = 22
motor_right_backward = 23

motor_pins = [motor_left_forward, motor_left_backward, motor_right_forward, motor_right_backward]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

def move_forward():
    GPIO.output(motor_left_forward, GPIO.HIGH)
    GPIO.output(motor_right_forward, GPIO.HIGH)
    GPIO.output(motor_left_backward, GPIO.LOW)
    GPIO.output(motor_right_backward, GPIO.LOW)

def turn_left():
    GPIO.output(motor_left_forward, GPIO.LOW)
    GPIO.output(motor_right_forward, GPIO.HIGH)
    GPIO.output(motor_left_backward, GPIO.LOW)
    GPIO.output(motor_right_backward, GPIO.LOW)

def turn_right():
    GPIO.output(motor_left_forward, GPIO.HIGH)
    GPIO.output(motor_right_forward, GPIO.LOW)
    GPIO.output(motor_left_backward, GPIO.LOW)
    GPIO.output(motor_right_backward, GPIO.LOW)

def stop():
    for pin in motor_pins:
        GPIO.output(pin, GPIO.LOW)

# --- HSV Color Ranges ---
lower_red_1 = np.array([0, 120, 70])
upper_red_1 = np.array([10, 255, 255])
lower_red_2 = np.array([160, 120, 70])
upper_red_2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 100])
upper_blue = np.array([140, 255, 255])

# --- Reference Setup ---
reference_point = (320, 240)  # 640x480 center
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

def detect_largest_object(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 150:
        return None
    return largest

def process_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red_1, upper_red_1),
        cv2.inRange(hsv, lower_red_2, upper_red_2)
    )
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    largest_red = detect_largest_object(mask_red)
    largest_blue = detect_largest_object(mask_blue)

    target_contour = None
    color_name = ""
    color_draw = (0, 0, 0)

    if largest_red is not None:
        target_contour = largest_red
        color_name = "Red"
        color_draw = (0, 0, 255)
    elif largest_blue is not None:
        target_contour = largest_blue
        color_name = "Blue"
        color_draw = (255, 0, 0)

    if target_contour is not None:
        M = cv2.moments(target_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            x, y, w, h = cv2.boundingRect(target_contour)
            dx = cx - reference_point[0]

            # Draw rectangle + dot
            cv2.rectangle(frame, (x, y), (x + w, y + h), color_draw, 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"{color_name} ({cx}, {cy})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_draw, 2)

            # Movement Logic
            if abs(dx) < 50:
                move_forward()
                print(f" {color_name} in center — Moving Forward")
            elif dx < -50:
                turn_left()
                print(f"↪ {color_name} left — Turning Left")
            else:
                turn_right()
                print(f"↩ {color_name} right — Turning Right")
    else:
        stop()
        print(" No object detected — Stopping")

    # Draw center
    cv2.circle(frame, reference_point, 5, (0, 255, 255), -1)
    return frame

# --- Main Loop ---
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera error")
            break

        frame = cv2.flip(frame, 1)
        output = process_frame(frame)
        cv2.imshow("Object Tracker", output)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC key to break
            break

except KeyboardInterrupt:
    print(" Stopped by user")

finally:
    stop()
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    print(" Clean exit")
