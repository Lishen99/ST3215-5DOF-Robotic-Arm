#!/usr/bin/env python
"""
ST3215 SERVO RECOVERY TOOL
==========================
This scans ALL baud rates and ALL IDs to find "lost" servos.
If a servo's baud rate or ID was accidentally changed, this will find it.

Also attempts factory reset if a servo is found.
"""

import serial
import time

PORT = "COM3"

# All possible baud rates for ST3215
BAUD_RATES = [
    1000000,  # Default
    500000,
    250000,
    128000,
    115200,
    76800,
    57600,
    38400,
]

def build_ping_packet(servo_id):
    """Build a PING packet (instruction 0x01)"""
    packet = [0xFF, 0xFF, servo_id, 2, 0x01]
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    return bytes(packet)

def build_write_packet(servo_id, start_addr, data):
    """Build a WRITE packet (instruction 0x03)"""
    length = len(data) + 3
    packet = [0xFF, 0xFF, servo_id, length, 0x03, start_addr] + data
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    return bytes(packet)

def try_ping(ser, servo_id, timeout=0.02):
    """Try to ping a servo and check for response"""
    ser.reset_input_buffer()
    packet = build_ping_packet(servo_id)
    ser.write(packet)
    time.sleep(timeout)
    response = ser.read(100)
    return len(response) > 0, response

def enable_torque(ser, servo_id):
    """Enable torque on a servo"""
    packet = build_write_packet(servo_id, 40, [1])  # Register 40 = Torque Enable
    ser.write(packet)
    time.sleep(0.01)

def reset_to_defaults(ser, servo_id):
    """Try to reset servo to factory defaults"""
    # Unlock EPROM
    packet = build_write_packet(servo_id, 55, [0])
    ser.write(packet)
    time.sleep(0.05)
    
    # Set baud rate to 0 (1Mbps)
    packet = build_write_packet(servo_id, 6, [0])
    ser.write(packet)
    time.sleep(0.05)
    
    # Set mode to 0 (position mode)
    packet = build_write_packet(servo_id, 33, [0])
    ser.write(packet)
    time.sleep(0.05)
    
    # Lock EPROM
    packet = build_write_packet(servo_id, 55, [1])
    ser.write(packet)
    time.sleep(0.05)
    
    # Enable torque
    enable_torque(ser, servo_id)

def main():
    print("=" * 60)
    print("ST3215 SERVO RECOVERY TOOL")
    print("=" * 60)
    print()
    print("This will scan ALL baud rates and IDs 0-20 to find servos")
    print("that may have had their settings changed.")
    print()
    
    found_servos = []
    
    for baud in BAUD_RATES:
        print(f"\n{'='*60}")
        print(f"Scanning at {baud} baud...")
        print('='*60)
        
        try:
            ser = serial.Serial(PORT, baud, timeout=0.02)
            time.sleep(0.1)
            ser.reset_input_buffer()
        except Exception as e:
            print(f"  Could not open port at {baud}: {e}")
            continue
        
        # Scan IDs 0-20 (covers original IDs 1-6 plus common alternatives)
        for servo_id in range(0, 21):
            found, response = try_ping(ser, servo_id)
            if found:
                print(f"  *** FOUND SERVO at ID {servo_id}! ***")
                print(f"      Response: {response.hex(' ')}")
                found_servos.append((baud, servo_id))
                
                # Try to enable torque immediately
                print(f"      Enabling torque...")
                enable_torque(ser, servo_id)
        
        # Also try broadcast ping (ID 254) - some servos respond to this
        found, response = try_ping(ser, 254, timeout=0.1)
        if found:
            print(f"  *** GOT BROADCAST RESPONSE! ***")
            print(f"      Response: {response.hex(' ')}")
            # Try to parse which IDs responded
        
        ser.close()
    
    print("\n" + "=" * 60)
    print("SCAN COMPLETE")
    print("=" * 60)
    
    if found_servos:
        print(f"\nFound {len(found_servos)} servo(s):")
        for baud, sid in found_servos:
            print(f"  - ID {sid} at {baud} baud")
        
        print("\nWould you like to reset them to defaults? (y/n)")
        choice = input().strip().lower()
        
        if choice == 'y':
            for baud, sid in found_servos:
                print(f"\nResetting ID {sid} at {baud} baud...")
                ser = serial.Serial(PORT, baud, timeout=0.05)
                time.sleep(0.1)
                reset_to_defaults(ser, sid)
                ser.close()
                print(f"  Done!")
            
            print("\nReset complete! Power cycle the servos and try again at 1Mbps.")
    else:
        print("\nNo servos found at any baud rate or ID.")
        print()
        print("Possible causes:")
        print("  1. Servos lost power (check power supply)")
        print("  2. Waveshare board TX circuit is dead")
        print("  3. Servo firmware is corrupted (rare)")
        print()
        print("Try:")
        print("  - Check if servo LEDs are on when powered")
        print("  - Try a different power supply")
        print("  - Test with Arduino/Teensy directly (bypass Waveshare)")

if __name__ == "__main__":
    main()
