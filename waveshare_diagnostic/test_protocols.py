#!/usr/bin/env python
"""
Try different servo protocols - STS, SCS, and raw
"""
import serial
import time

DEVICENAME = 'COM3'
BAUD_RATES = [1000000, 500000, 115200, 38400]

def send_and_receive(ser, packet, name=""):
    """Send packet and show response"""
    print(f"  {name} TX: {' '.join(f'{b:02X}' for b in packet)}")
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    time.sleep(0.1)
    response = ser.read(ser.in_waiting or 100)
    if response:
        print(f"  {name} RX: {' '.join(f'{b:02X}' for b in response)}")
        return True
    else:
        print(f"  {name} RX: (no response)")
        return False

print(f"Trying multiple protocols on {DEVICENAME}")
print("=" * 60)

for baud in BAUD_RATES:
    print(f"\n{'='*60}")
    print(f"BAUD RATE: {baud}")
    print('='*60)
    
    try:
        ser = serial.Serial(DEVICENAME, baud, timeout=0.2)
        time.sleep(0.05)
        
        for servo_id in [1, 254]:  # Try ID 1 and broadcast
            print(f"\n--- Servo ID {servo_id} ---")
            
            # STS/SCS Ping: FF FF ID 02 01 CHECKSUM
            checksum = (~(servo_id + 2 + 0x01)) & 0xFF
            sts_ping = bytes([0xFF, 0xFF, servo_id, 0x02, 0x01, checksum])
            send_and_receive(ser, sts_ping, "STS Ping")
            
            # Try reading position: FF FF ID 04 02 38 02 CHECKSUM
            # Register 0x38 (56) = present position
            checksum = (~(servo_id + 4 + 0x02 + 0x38 + 0x02)) & 0xFF
            read_pos = bytes([0xFF, 0xFF, servo_id, 0x04, 0x02, 0x38, 0x02, checksum])
            send_and_receive(ser, read_pos, "Read Pos")
            
            # Try Dynamixel v1 protocol (some servos use this)
            # Same format but different checksum calc in some cases
            
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

print("\n" + "=" * 60)
print("If still no response, the issue is hardware:")
print("  - Waveshare half-duplex circuit may be damaged")
print("  - Try connecting servo directly to a USB-TTL adapter")
print("    (with TX-RX tied together through a resistor)")
input("\nPress Enter to exit...")
