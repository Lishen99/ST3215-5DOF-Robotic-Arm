#!/usr/bin/env python
"""
FULL SCAN - Try ALL servo IDs (0-253) at multiple baud rates
This will take a while but will find any servo regardless of what ID it got set to
"""
import serial
import time

DEVICENAME = 'COM3'
BAUD_RATES = [1000000, 500000, 115200]

def ping_servo(ser, servo_id):
    """Send ping and check for response"""
    checksum = (~(servo_id + 2 + 0x01)) & 0xFF
    packet = bytes([0xFF, 0xFF, servo_id, 0x02, 0x01, checksum])
    
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    time.sleep(0.02)  # 20ms
    
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        return True, response
    return False, None

print(f"FULL SERVO SCAN on {DEVICENAME}")
print("Scanning ALL IDs (0-253) - this will take a minute...")
print("=" * 60)

found_any = False

for baud in BAUD_RATES:
    print(f"\n--- Baud rate: {baud} ---")
    
    try:
        ser = serial.Serial(DEVICENAME, baud, timeout=0.05)
        time.sleep(0.1)
        
        for servo_id in range(254):  # 0 to 253
            found, response = ping_servo(ser, servo_id)
            if found:
                print(f"\n*** FOUND SERVO ID {servo_id}! ***")
                print(f"    Response: {' '.join(f'{b:02X}' for b in response)}")
                found_any = True
            
            # Progress indicator every 50 IDs
            if servo_id % 50 == 0:
                print(f"  Scanned IDs 0-{servo_id}...", end='\r')
        
        print(f"  Scanned IDs 0-253 - Done      ")
        ser.close()
        
        if found_any:
            break  # Stop if we found something
        
    except Exception as e:
        print(f"Error: {e}")

print("\n" + "=" * 60)
if found_any:
    print("SUCCESS! Found servo(s) - see above for details")
else:
    print("No servos found at any ID or baud rate.")
    print("\nThis confirms a HARDWARE issue:")
    print("  - Waveshare board half-duplex circuit likely damaged")
    print("  - Or all servos somehow got corrupted (very unlikely)")

input("\nPress Enter to exit...")
