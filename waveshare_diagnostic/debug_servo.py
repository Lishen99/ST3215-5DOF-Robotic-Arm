#!/usr/bin/env python
"""
Debug servo communication - shows raw bytes
"""

import sys
import os
import time
import serial

DEVICENAME = 'COM3'
BAUD_RATES = [1000000, 500000, 115200]

def ping_servo_raw(ser, servo_id):
    """Send raw ping packet and show response"""
    # STS Ping packet: FF FF ID LEN INST CHECKSUM
    # ID=servo_id, LEN=2, INST=0x01 (ping)
    length = 2
    instruction = 0x01
    checksum = (~(servo_id + length + instruction)) & 0xFF
    
    packet = bytes([0xFF, 0xFF, servo_id, length, instruction, checksum])
    
    print(f"  TX: {' '.join(f'{b:02X}' for b in packet)}")
    
    # Clear any pending data
    ser.reset_input_buffer()
    
    # Send packet
    ser.write(packet)
    ser.flush()
    
    # Wait for response
    time.sleep(0.1)  # 100ms wait - longer timeout
    
    # Read response
    response = ser.read(ser.in_waiting or 100)
    
    if response:
        print(f"  RX: {' '.join(f'{b:02X}' for b in response)}")
        return True
    else:
        print(f"  RX: (no response)")
        return False

print(f"Debug servo communication on {DEVICENAME}")
print("=" * 50)

for baud in BAUD_RATES:
    print(f"\n--- Baud rate: {baud} ---")
    
    try:
        ser = serial.Serial(DEVICENAME, baud, timeout=0.2)
        time.sleep(0.1)  # Let port settle
        
        # Try broadcast ID first (0xFE = 254)
        print(f"\nPing BROADCAST (ID 254):")
        ping_servo_raw(ser, 254)
        
        # Then try individual IDs
        for servo_id in [1, 2, 3, 4, 5, 6]:
            print(f"\nPing servo ID {servo_id}:")
            if ping_servo_raw(ser, servo_id):
                print(f"  *** GOT RESPONSE! ***")
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

print("\n" + "=" * 50)
print("Done. If no RX responses, check:")
print("  1. Servo data cable connected to UART board")
print("  2. UART board mode switch (USB vs UART)")
print("  3. Servo chain connections")
input("Press Enter to exit...")
