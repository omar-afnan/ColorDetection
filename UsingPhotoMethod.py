import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)

motor_pins = [17, 18, 22, 23]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

motor_pwms = {
    'left_fwd': GPIO.PWM(17, 100),
    'left_rev': GPIO.PWM(18, 100),
    'right_fwd': GPIO.PWM(22, 100),
    'right_rev': GPIO.PWM(23, 100)
}

for pwm in motor_pwms.values():
    pwm.start(0)

# Servo setup
SERVO_PIN = 12
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# Center the servo at startup (90° = 7.5% duty)
def center_servo():
    servo.ChangeDutyCycle(7.5)
    time.sleep(0.4)
    servo.ChangeDutyCycle(0)

center_servo()

# === Movement Functions ===
def stop():
    for pwm in motor_pwms.values():
        pwm.ChangeDutyCycle(0)

def move_forward(speed=30):
    stop()
    motor_pwms['left_fwd'].ChangeDutyCycle(speed + 5)  # left motor boost to fix drift
    motor_pwms['right_fwd'].ChangeDutyCycle(speed)

def move_backward(speed=30):
    stop()
    motor_pwms['left_rev'].ChangeDutyCycle(speed + 5)
    motor_pwms['right_rev'].ChangeDutyCycle(speed)

def turn_left(speed=30):
    stop()
    motor_pwms['right_fwd'].ChangeDutyCycle(speed)

def turn_right(speed=30):
    stop()
    motor_pwms['left_fwd'].ChangeDutyCycle(speed)

# === Color Detection HSV Ranges ===
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 120, 70])
upper_red2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 100])
upper_blue = np.array([140, 255, 255])

# === Camera Setup ===
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # For Pi Cam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
time.sleep(2)

if not cap.isOpened():
    print("ERROR: Camera not detected.")
    GPIO.cleanup()
    exit()

reference_x = 160  # Center of frame

# === Contour Filter ===
def find_valid_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 800:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = w / float(h)
        if 0.75 < aspect_ratio < 1.25:
            valid.append((cnt, area))
    if valid:
        return max(valid, key=lambda tup: tup[1])[0]
    return None

# === Smart Scan with Servo ===
def scan_for_object():
    print("Scanning with servo...")
    angles = list(range(60, 121, 10)) + list(range(110, 59, -10))  # Left to right, then back
    for angle in angles:
        duty = angle / 18 + 2
        servo.ChangeDutyCycle(duty)
        time.sleep(0.3)
        servo.ChangeDutyCycle(0)

        cap.grab()
        ret, frame = cap.retrieve()
        if not ret:
            continue

        frame = cv2.flip(frame, -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2)
        )
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        red_contour = find_valid_contour(red_mask)
        blue_contour = find_valid_contour(blue_mask)

        if red_contour is not None:
            print(f"Red object seen at servo angle {angle}")
            center_servo()
            return 'red', angle
        elif blue_contour is not None:
            print(f"Blue object seen at servo angle {angle}")
            center_servo()
            return 'blue', angle

    print("Scan complete. No object found.")
    center_servo()
    return None, None

# === Main Loop ===
try:
    print("Starting main loop")
    while True:
        cap.grab()
        ret, frame = cap.retrieve()
        if not ret:
            print("Camera error")
            continue

        frame = cv2.flip(frame, -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2)
        )
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

        color_detected = None
        cx = None

        red_contour = find_valid_contour(red_mask)
        if red_contour is not None:
            x, y, w, h = cv2.boundingRect(red_contour)
            cx = x + w // 2
            color_detected = 'red'

        if color_detected is None:
            blue_contour = find_valid_contour(blue_mask)
            if blue_contour is not None:
                x, y, w, h = cv2.boundingRect(blue_contour)
                cx = x + w // 2
                color_detected = 'blue'

        if color_detected and cx:
            if cx < reference_x - 40:
                print(f"{color_detected} cube on LEFT")
                turn_left(25)
                time.sleep(0.25)
                stop()
            elif cx > reference_x + 40:
                print(f"{color_detected} cube on RIGHT")
                turn_right(25)
                time.sleep(0.25)
                stop()
            else:
                print(f"{color_detected} cube CENTERED — pushing forward")
                move_forward(40)
                time.sleep(1.2)
                stop()

                print("Backing up slightly")
                move_backward(35)
                time.sleep(0.3)
                stop()

                print("Rescanning...")
                time.sleep(0.5)
        else:
            print("No object detected — servo scan time")
            stop()
            color, angle = scan_for_object()

            if color:
                if angle < 80:
                    print("Turning slightly LEFT to face object")
                    turn_left(25)
                    time.sleep(0.25)
                    stop()
                elif angle > 100:
                    print("Turning slightly RIGHT to face object")
                    turn_right(25)
                    time.sleep(0.25)
                    stop()
            else:
                print("Nothing found. Pausing.")
                time.sleep(1)

finally:
    print("Shutting down...")
    stop()
    for pwm in motor_pwms.values():
        pwm.stop()
    servo.stop()
    GPIO.cleanup()
    cap.release()
