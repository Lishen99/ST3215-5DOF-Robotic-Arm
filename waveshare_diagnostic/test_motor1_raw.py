#!/usr/bin/env python
"""
RAW TX-ONLY TEST - No response waiting
======================================
This sends raw packets without any response waiting.
Pure TX test to see if data gets to the servo.
"""

import serial
import time

PORT = "COM3"
BAUD = 1000000

def build_write_packet(servo_id, start_addr, data):
    """Build a WRITE packet (instruction 0x03)"""
    length = len(data) + 3  # data + start_addr + instruction + checksum
    packet = [0xFF, 0xFF, servo_id, length, 0x03, start_addr] + data
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    return bytes(packet)

def lobyte(w):
    return w & 0xFF

def hibyte(w):
    return (w >> 8) & 0xFF

def toscs(a):
    """Convert signed speed to STS format"""
    if a < 0:
        return (-a) | 0x8000
    return a

def main():
    print("=" * 60)
    print("RAW TX-ONLY TEST - Direct serial writes")
    print("=" * 60)
    
    ser = serial.Serial(PORT, BAUD, timeout=0.01)
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(f"[OK] Opened {PORT} at {BAUD}")
    
    SERVO_ID = 1
    
    # Register addresses
    STS_LOCK = 55
    STS_MODE = 33
    STS_TORQUE_ENABLE = 40
    STS_ACC = 41
    
    print()
    print("=" * 60)
    print("Step 1: Unlock EPROM (register 55 = 0)")
    print("=" * 60)
    packet = build_write_packet(SERVO_ID, STS_LOCK, [0])
    ser.write(packet)
    print(f"  Sent: {packet.hex(' ')}")
    time.sleep(0.1)
    
    print()
    print("=" * 60)
    print("Step 2: Set Wheel Mode (register 33 = 1)")
    print("=" * 60)
    packet = build_write_packet(SERVO_ID, STS_MODE, [1])
    ser.write(packet)
    print(f"  Sent: {packet.hex(' ')}")
    time.sleep(0.1)
    
    print()
    print("=" * 60)
    print("Step 3: Lock EPROM (register 55 = 1)")
    print("=" * 60)
    packet = build_write_packet(SERVO_ID, STS_LOCK, [1])
    ser.write(packet)
    print(f"  Sent: {packet.hex(' ')}")
    time.sleep(0.1)
    
    print()
    print("=" * 60)
    print("Step 4: Enable Torque (register 40 = 1)")
    print("=" * 60)
    packet = build_write_packet(SERVO_ID, STS_TORQUE_ENABLE, [1])
    ser.write(packet)
    print(f"  Sent: {packet.hex(' ')}")
    time.sleep(0.1)
    
    print()
    print("=" * 60)
    print("Step 5: SPIN CLOCKWISE - Speed 500")
    print("=" * 60)
    print(">>> WATCH MOTOR 1 NOW! <<<")
    
    speed = toscs(500)
    # WriteSpec format: ACC, POS_L, POS_H, TIME_L, TIME_H, SPEED_L, SPEED_H
    # Starting at register 41 (ACC)
    data = [50, 0, 0, 0, 0, lobyte(speed), hibyte(speed)]
    packet = build_write_packet(SERVO_ID, STS_ACC, data)
    ser.write(packet)
    print(f"  Sent: {packet.hex(' ')}")
    
    print("\n  Waiting 3 seconds...")
    time.sleep(3)
    
    print()
    print("=" * 60)
    print("Step 6: SPIN COUNTER-CLOCKWISE - Speed -500")
    print("=" * 60)
    
    speed = toscs(-500)
    data = [50, 0, 0, 0, 0, lobyte(speed), hibyte(speed)]
    packet = build_write_packet(SERVO_ID, STS_ACC, data)
    ser.write(packet)
    print(f"  Sent: {packet.hex(' ')}")
    
    print("\n  Waiting 3 seconds...")
    time.sleep(3)
    
    print()
    print("=" * 60)
    print("Step 7: STOP - Speed 0")
    print("=" * 60)
    
    speed = 0
    data = [50, 0, 0, 0, 0, lobyte(speed), hibyte(speed)]
    packet = build_write_packet(SERVO_ID, STS_ACC, data)
    ser.write(packet)
    print(f"  Sent: {packet.hex(' ')}")
    time.sleep(0.5)
    
    print()
    print("=" * 60)
    print("Step 8: Disable torque & restore position mode")
    print("=" * 60)
    
    # Disable torque
    packet = build_write_packet(SERVO_ID, STS_TORQUE_ENABLE, [0])
    ser.write(packet)
    time.sleep(0.05)
    
    # Unlock, set mode 0, lock
    packet = build_write_packet(SERVO_ID, STS_LOCK, [0])
    ser.write(packet)
    time.sleep(0.05)
    
    packet = build_write_packet(SERVO_ID, STS_MODE, [0])
    ser.write(packet)
    time.sleep(0.05)
    
    packet = build_write_packet(SERVO_ID, STS_LOCK, [1])
    ser.write(packet)
    
    ser.close()
    print("\n[DONE] Port closed")
    
    print()
    print("=" * 60)
    print("Did motor 1 spin at any point?")
    print("=" * 60)
    print()
    print("  YES -> Board TX works, servos work, only RX broken")
    print("  NO  -> Board TX broken OR servo damaged")
    print()

if __name__ == "__main__":
    main()
