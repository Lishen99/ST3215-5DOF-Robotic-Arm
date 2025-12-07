#!/usr/bin/env python
"""
Test if the Waveshare board USB-serial is working at all
This doesn't need servos connected
"""
import serial
import time

DEVICENAME = 'COM3'

print("Testing Waveshare board USB-Serial...")
print("=" * 50)

try:
    ser = serial.Serial(DEVICENAME, 115200, timeout=0.5)
    print(f"✓ Port {DEVICENAME} opened successfully")
    
    # Check if we can write without errors
    print("\nSending test bytes...")
    test_data = bytes([0x55, 0xAA, 0x55, 0xAA])
    bytes_written = ser.write(test_data)
    ser.flush()
    print(f"✓ Wrote {bytes_written} bytes without error")
    
    # The board won't echo back (half-duplex), but if it doesn't error, USB is working
    time.sleep(0.1)
    
    # Try reading - might get something back depending on bus state
    response = ser.read(100)
    if response:
        print(f"  Received: {' '.join(f'{b:02X}' for b in response)}")
    else:
        print("  No echo (expected for half-duplex)")
    
    print("\n✓ USB-Serial chip appears to be working!")
    print("\nThe problem is likely:")
    print("  1. Data cable between Waveshare and servo")
    print("  2. Servo itself not responding")
    print("  3. Servo baud rate mismatch (unlikely)")
    
    ser.close()
    
except serial.SerialException as e:
    print(f"✗ Serial error: {e}")
    print("\nThe USB-serial chip might be damaged!")
    
except Exception as e:
    print(f"✗ Error: {e}")

input("\nPress Enter to exit...")
