import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)

# Motor PWM Setup
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

# Servo Setup
SERVO_PIN = 12
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# === Movement Functions ===
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

def stop():
    for pwm in motor_pwms.values():
        pwm.ChangeDutyCycle(0)

# === Color Ranges ===
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
time.sleep(2)
reference_x = 160  # center of 320px frame

# === Scanning Logic ===
def scan_for_object():
    print("Scanning for object...")
    found = False
    for angle in range(60, 121, 10):
        duty = angle / 18 + 2
        servo.ChangeDutyCycle(duty)
        time.sleep(0.2)

        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.flip(frame, -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        if cv2.countNonZero(red_mask) > 800 or cv2.countNonZero(blue_mask) > 800:
            print("Object detected during scan.")
            found = True
            break

    servo.ChangeDutyCycle(0)
    return found

# === Main Loop ===
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed.")
            continue

        frame = cv2.flip(frame, -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_DILATE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_DILATE, kernel)

        color_detected = None
        cx = None

        # Red detection
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 800:
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                color_detected = 'red'

        # Blue detection if no red
        if not color_detected:
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 800:
                    x, y, w, h = cv2.boundingRect(largest)
                    cx = x + w // 2
                    color_detected = 'blue'

        # === Movement Decision ===
        if color_detected and cx:
            if cx < reference_x - 40:
                print(f"{color_detected.capitalize()} object is on the left. Turning left.")
                turn_left()
                time.sleep(0.4)
                stop()
            elif cx > reference_x + 40:
                print(f"{color_detected.capitalize()} object is on the right. Turning right.")
                turn_right()
                time.sleep(0.4)
                stop()
            else:
                print(f"{color_detected.capitalize()} object centered. Moving forward.")
                move_forward()
                time.sleep(1.0)
                stop()
                move_backward()
                time.sleep(0.8)
                stop()
        else:
            print("No object detected. Initiating scan...")
            stop()
            if not scan_for_object():
                print("Still nothing found after scanning.")
                time.sleep(1)

finally:
    print("Cleaning up GPIO and camera...")
    stop()
    for pwm in motor_pwms.values():
        pwm.stop()
    servo.stop()
    GPIO.cleanup()
    cap.release()
 
