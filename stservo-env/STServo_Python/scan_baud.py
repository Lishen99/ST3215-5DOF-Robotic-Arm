#!/usr/bin/env python
"""
Scan all baud rates to find servos
"""

import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()

sys.path.append("..")
from STservo_sdk import *

# Baud rates to try (most common for ST servos)
BAUD_RATES = [1000000, 500000, 250000, 115200, 57600, 38400, 19200, 9600]
SERVO_IDS = [1, 2, 3, 4, 5, 6]
DEVICENAME = 'COM3'

print(f"Scanning for servos on {DEVICENAME}...")
print("=" * 50)

portHandler = PortHandler(DEVICENAME)

if not portHandler.openPort():
    print("Failed to open port!")
    quit()

print("Port opened successfully")

found_baud = None
found_servos = []

for baud in BAUD_RATES:
    print(f"\nTrying baud rate: {baud}...")
    
    if not portHandler.setBaudRate(baud):
        print(f"  Failed to set baud rate")
        continue
    
    packetHandler = sts(portHandler)
    
    for servo_id in SERVO_IDS:
        model_number, comm_result, error = packetHandler.ping(servo_id)
        if comm_result == COMM_SUCCESS:
            print(f"  *** FOUND servo ID {servo_id} at {baud} baud! Model: {model_number} ***")
            found_baud = baud
            found_servos.append(servo_id)

    if found_servos:
        break

portHandler.closePort()

print("\n" + "=" * 50)
if found_baud:
    print(f"SUCCESS! Found {len(found_servos)} servo(s) at {found_baud} baud")
    print(f"Servo IDs: {found_servos}")
    print(f"\nUpdate your firmware to use baud rate: {found_baud}")
else:
    print("No servos found at any baud rate!")
    print("Check:")
    print("  1. Servo power is ON")
    print("  2. Correct COM port")
    print("  3. Wiring connections")

print("\nPress any key to exit...")
getch()
