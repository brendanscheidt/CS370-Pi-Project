#!/usr/bin/env python3
import time
import lgpio as GPIO
import socket
import math

# ─── Configuration ─────────────────────────────────────────────────────────────
TRIG = 23          # BCM pin for trigger
ECHO = 24          # BCM pin for echo

MIN_DISTANCE = 15   # cm
MAX_DISTANCE = 70   # cm
MIN_FREQ     = 220.0  # Hz
MAX_FREQ     = 1000.0  # Hz

REFERENCE_PITCH = 440.0 # Hz

SENSOR_SETTLING_DELAY = 0.1     # s
TRIGGER_PULSE_LENGTH  = 0.00001 # s

SERVER_HOST = '192.168.155.53'  # ← change to your server’s LAN IP
SERVER_PORT = 8080

# ─── Setup LGPIO ───────────────────────────────────────────────────────────────
h = GPIO.gpiochip_open(0)
GPIO.gpio_claim_output(h, TRIG)
GPIO.gpio_claim_input(h, ECHO)

def get_distance():
    """Trigger HC‑SR04 and return distance in cm."""
    GPIO.gpio_write(h, TRIG, 0)
    time.sleep(SENSOR_SETTLING_DELAY)
    GPIO.gpio_write(h, TRIG, 1)
    time.sleep(TRIGGER_PULSE_LENGTH)
    GPIO.gpio_write(h, TRIG, 0)

    while GPIO.gpio_read(h, ECHO) == 0:
        start = time.time()
    while GPIO.gpio_read(h, ECHO) == 1:
        end = time.time()

    duration = end - start
    return (duration * 34300) / 2

def map_distance_to_frequency(d):
    """Clamp [MIN_DISTANCE…MAX_DISTANCE] → [MIN_FREQ…MAX_FREQ]."""
    """
    d = max(MIN_DISTANCE, min(MAX_DISTANCE, d))
    return ((d - MIN_DISTANCE) * (MAX_FREQ - MIN_FREQ)
            / (MAX_DISTANCE - MIN_DISTANCE)) + MIN_FREQ
    """
    n = min(max(math.floor(d), MIN_DISTANCE), MAX_DISTANCE)
    return pow(2, n / 12) * REFERENCE_PITCH

def main():
    # Connect once
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {SERVER_HOST}:{SERVER_PORT}…")
    sock.connect((SERVER_HOST, SERVER_PORT))
    print("Connected ✓")

    try:
        while True:
            dist = get_distance()
            freq = map_distance_to_frequency(dist)
            line = f"{freq:.2f}\n".encode('ascii')
            sock.sendall(line)
            print(f"Sent → {freq:.2f} Hz")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nSender stopped by user")
    finally:
        sock.close()
        GPIO.gpiochip_close(h)

if __name__ == '__main__':
    main()
