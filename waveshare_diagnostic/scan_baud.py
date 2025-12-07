#!/usr/bin/env python3
"""
Scan for servo baud rate - tries multiple baud rates to find servos
"""

import sys
import os

# Add the STservo_sdk to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'stservo-env', 'STservo_sdk'))

from port_handler import PortHandler
from sts import sts

# Common baud rates to try
BAUD_RATES = [1000000, 500000, 250000, 115200, 57600, 38400, 19200, 9600]
SERVO_IDS = [1, 2, 3, 4, 5, 6]

def scan_baud_rate(port='COM3'):
    print(f"Scanning for servos on {port}...")
    print("=" * 50)
    
    for baud in BAUD_RATES:
        print(f"\nTrying baud rate: {baud}")
        
        try:
            port_handler = PortHandler(port)
            if not port_handler.openPort():
                print(f"  Failed to open port")
                continue
            
            if not port_handler.setBaudRate(baud):
                print(f"  Failed to set baud rate")
                port_handler.closePort()
                continue
            
            servo = sts(port_handler)
            
            found = []
            for servo_id in SERVO_IDS:
                # Try to ping
                result, error = servo.ping(servo_id)
                if result:
                    found.append(servo_id)
                    print(f"  Found servo ID {servo_id}!")
            
            port_handler.closePort()
            
            if found:
                print(f"\n*** SUCCESS! Found {len(found)} servos at {baud} baud ***")
                print(f"    Servo IDs: {found}")
                return baud, found
                
        except Exception as e:
            print(f"  Error: {e}")
            try:
                port_handler.closePort()
            except:
                pass
    
    print("\nNo servos found at any baud rate!")
    return None, []

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM3'
    print(f"Usage: python scan_baud.py [COM_PORT]")
    print(f"Using port: {port}\n")
    
    baud, servos = scan_baud_rate(port)
    
    if baud:
        print(f"\nTo use the servos, set baud rate to: {baud}")
