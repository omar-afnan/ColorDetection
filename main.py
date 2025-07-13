import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)

# Motor control using PWM (GPIO 17, 18, 22, 23)
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
    pwm.start(0)  # Start with 0% duty

# Servo Setup
SERVO_PIN = 12
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# === Movement Functions ===
def move_forward(speed=30):
    motor_pwms['left_fwd'].ChangeDutyCycle(speed)
    motor_pwms['right_fwd'].ChangeDutyCycle(speed)
    motor_pwms['left_rev'].ChangeDutyCycle(0)
    motor_pwms['right_rev'].ChangeDutyCycle(0)

def move_backward(speed=30):
    motor_pwms['left_fwd'].ChangeDutyCycle(0)
    motor_pwms['right_fwd'].ChangeDutyCycle(0)
    motor_pwms['left_rev'].ChangeDutyCycle(speed)
    motor_pwms['right_rev'].ChangeDutyCycle(speed)

def turn_left(speed=30):
    motor_pwms['left_fwd'].ChangeDutyCycle(0)
    motor_pwms['right_fwd'].ChangeDutyCycle(speed)
    motor_pwms['left_rev'].ChangeDutyCycle(0)
    motor_pwms['right_rev'].ChangeDutyCycle(0)

def turn_right(speed=30):
    motor_pwms['left_fwd'].ChangeDutyCycle(speed)
    motor_pwms['right_fwd'].ChangeDutyCycle(0)
    motor_pwms['left_rev'].ChangeDutyCycle(0)
    motor_pwms['right_rev'].ChangeDutyCycle(0)

def stop():
    for pwm in motor_pwms.values():
        pwm.ChangeDutyCycle(0)

# === Color Detection Ranges ===
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

reference_x = 320  # center of frame

# === Servo scanning logic ===
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

        red_found = cv2.countNonZero(red_mask) > 800
        blue_found = cv2.countNonZero(blue_mask) > 800
        if red_found or blue_found:
            print("Object detected during scan.")
            found = True
            break
    servo.ChangeDutyCycle(0)
    return found

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, -1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create masks
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Noise filtering
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

        # Movement logic
        if color_detected and cx:
            if cx < reference_x - 60:
                print(f"{color_detected.capitalize()} left — turning left")
                turn_left(speed=30)
            elif cx > reference_x + 60:
                print(f"{color_detected.capitalize()} right — turning right")
                turn_right(speed=30)
            else:
                print(f"{color_detected.capitalize()} center — pushing forward")
                move_forward(speed=30)
                time.sleep(1.5)
                move_backward(speed=30)
                time.sleep(1.0)
                stop()
        else:
            print("No object detected — scanning...")
            stop()
            found = scan_for_object()
            if not found:
                print("Still no object after scanning.")
                time.sleep(1)

        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("Cleaning up...")
    stop()
    for pwm in motor_pwms.values():
        pwm.stop()
    servo.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
