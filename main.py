import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import threading

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)

# --- Motor Pins ---
motor_pins = {
    'left_fwd': 17,
    'left_rev': 18,
    'right_fwd': 22,
    'right_rev': 23
}
for pin in motor_pins.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# --- Servo Setup ---
SERVO_PIN = 12
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz
servo.start(0)

# === Servo sweeping thread ===
def sweep_servo():
    while True:
        for angle in range(60, 120, 2):
            duty = angle / 18 + 2
            servo.ChangeDutyCycle(duty)
            time.sleep(0.05)
        time.sleep(0.2)
        for angle in range(120, 60, -2):
            duty = angle / 18 + 2
            servo.ChangeDutyCycle(duty)
            time.sleep(0.05)
        time.sleep(0.2)

# Start sweeping
sweeper = threading.Thread(target=sweep_servo, daemon=True)
sweeper.start()

# === Movement Functions ===
def move_forward():
    GPIO.output(motor_pins['left_fwd'], GPIO.HIGH)
    GPIO.output(motor_pins['right_fwd'], GPIO.HIGH)
    GPIO.output(motor_pins['left_rev'], GPIO.LOW)
    GPIO.output(motor_pins['right_rev'], GPIO.LOW)

def move_backward():
    GPIO.output(motor_pins['left_fwd'], GPIO.LOW)
    GPIO.output(motor_pins['right_fwd'], GPIO.LOW)
    GPIO.output(motor_pins['left_rev'], GPIO.HIGH)
    GPIO.output(motor_pins['right_rev'], GPIO.HIGH)

def turn_left():
    GPIO.output(motor_pins['left_fwd'], GPIO.LOW)
    GPIO.output(motor_pins['right_fwd'], GPIO.HIGH)
    GPIO.output(motor_pins['left_rev'], GPIO.LOW)
    GPIO.output(motor_pins['right_rev'], GPIO.LOW)

def turn_right():
    GPIO.output(motor_pins['left_fwd'], GPIO.HIGH)
    GPIO.output(motor_pins['right_fwd'], GPIO.LOW)
    GPIO.output(motor_pins['left_rev'], GPIO.LOW)
    GPIO.output(motor_pins['right_rev'], GPIO.LOW)

def stop():
    for pin in motor_pins.values():
        GPIO.output(pin, GPIO.LOW)

# === Color Ranges (HSV) ===
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 120, 70])
upper_red2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 100])
upper_blue = np.array([140, 255, 255])

# === Camera Setup ===
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
time.sleep(2)

reference_x = 320  # screen center

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red detection
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Blue detection
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Morph filters
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_DILATE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        color_detected = None
        cx = None

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 800:
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                cy = y + h // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                color_detected = 'red'

        if not color_detected:
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 800:
                    x, y, w, h = cv2.boundingRect(largest)
                    cx = x + w // 2
                    cy = y + h // 2
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    color_detected = 'blue'

        # Movement Logic
        if color_detected and cx:
            if cx < reference_x - 60:
                print(f"{color_detected.capitalize()} left — turning left")
                turn_left()
            elif cx > reference_x + 60:
                print(f"{color_detected.capitalize()} right — turning right")
                turn_right()
            else:
                print(f"{color_detected.capitalize()} center — pushing block")
                move_forward()
                time.sleep(1.5)  # Push forward
                move_backward()
                time.sleep(1.0)  # Pull back
                stop()
        else:
            print("No object — stop")
            stop()

        cv2.imshow("Camera", frame)
        cv2.imshow("Red Mask", red_mask)
        cv2.imshow("Blue Mask", blue_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("Cleaning up...")
    stop()
    servo.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
