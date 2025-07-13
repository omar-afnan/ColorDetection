import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
motor_pins = {
    'left_fwd': 17,
    'left_rev': 18,
    'right_fwd': 22,
    'right_rev': 23
}
for pin in motor_pins.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

def move_forward():
    GPIO.output(motor_pins['left_fwd'], GPIO.HIGH)
    GPIO.output(motor_pins['left_rev'], GPIO.LOW)
    GPIO.output(motor_pins['right_fwd'], GPIO.HIGH)
    GPIO.output(motor_pins['right_rev'], GPIO.LOW)

def stop():
    for pin in motor_pins.values():
        GPIO.output(pin, GPIO.LOW)

# === Color detection ===
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 120, 70])
upper_red2 = np.array([180, 255, 255])

cap = cv2.VideoCapture(0)
time.sleep(2)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 + mask2

        if cv2.countNonZero(red_mask) > 1000:
            print("Red detected — moving forward")
            move_forward()
        else:
            stop()

        cv2.imshow("View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
