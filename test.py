import serial
import time

esp = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

print("📤 Sending test command to ESP32...")
esp.write(b"PUSH_RED\n")

esp.close()
