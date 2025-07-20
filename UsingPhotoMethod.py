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

# === Movement Functions ===
def stop():
    for pwm in motor_pwms.values():
        pwm.ChangeDutyCycle(0)

def move_forward(speed=30):
    stop()
    motor_pwms['left_fwd'].ChangeDutyCycle(speed)
    motor_pwms['right_fwd'].ChangeDutyCycle(speed)

def move_backward(speed=30):
    stop()
    motor_pwms['left_rev'].ChangeDutyCycle(speed)
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
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # smoother for USB webcams
time.sleep(2)

if not cap.isOpened():
    print("ERROR: Camera not detected.")
    GPIO.cleanup()
    exit()

reference_x = 160  # Center of 320px frame

# === Scanning logic ===
def scan_for_object():
    print("Scanning for object...")
    found = False
    for angle in range(60, 121, 10):
        duty = angle / 18 + 2
        servo.ChangeDutyCycle(duty)
        time.sleep(0.2)

        ret, frame = cap.read()
        if not ret or frame is None:
            print("Failed to read frame during scan.")
            continue

        frame = cv2.flip(frame, -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2)
        )
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        if cv2.countNonZero(red_mask) > 800 or cv2.countNonZero(blue_mask) > 800:
            print("Object detected.")
            found = True
            break

    servo.ChangeDutyCycle(0)
    return found

# === Main Loop ===
try:
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Failed to read camera frame.")
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
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_DILATE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_DILATE, kernel)

        color_detected = None
        cx = None

        # Check for red
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 800:
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                color_detected = 'red'

        # Check for blue
        if not color_detected:
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 800:
                    x, y, w, h = cv2.boundingRect(largest)
                    cx = x + w // 2
                    color_detected = 'blue'

        # === Movement Logic ===
        if color_detected and cx:
            if cx < reference_x - 40:
                print(f"{color_detected} object on LEFT")
                turn_left()
                time.sleep(0.4)
                stop()
            elif cx > reference_x + 40:
                print(f"{color_detected} object on RIGHT")
                turn_right()
                time.sleep(0.4)
                stop()
            else:
                print(f"{color_detected} object CENTERED — moving forward")
                move_forward()
                time.sleep(1.0)
                move_backward()
                time.sleep(0.8)
                stop()
        else:
            print("No object detected — scanning...")
            stop()
            if not scan_for_object():
                print("Still nothing found after scanning.")
                time.sleep(1)

finally:
    print("Shutting down...")
    stop()
    for pwm in motor_pwms.values():
        pwm.stop()
    servo.stop()
    GPIO.cleanup()
    cap.release()
