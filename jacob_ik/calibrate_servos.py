import sys
import os
import time
import msvcrt
import json
import serial.tools.list_ports

# This script must be run from the root of the STServo_Python project
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from STservo_sdk import *

# --- Constants ---
BAUDRATE = 1000000
MAX_MOTOR_ID = 10

def get_port():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    if not ports:
        print("Error: No COM ports found.")
        return None
    
    if len(ports) == 1:
        print(f"Using the only available port: {ports[0]}")
        return ports[0]

    print("Available COM ports:")
    for i, port in enumerate(ports):
        print(f"  {i}: {port}")
    
    while True:
        try:
            choice = int(input("Select a port number: "))
            if 0 <= choice < len(ports):
                return ports[choice]
            else:
                print("Invalid choice.")
        except ValueError:
            print("Invalid input.")

def find_motors(packetHandler):
    print(f"Scanning for motors up to ID {MAX_MOTOR_ID}...")
    found_motors = []
    for motor_id in range(MAX_MOTOR_ID + 1):
        print(f"Pinging ID: {motor_id}", end='\r')
        if packetHandler.ping(motor_id)[1] == COMM_SUCCESS:
            found_motors.append(motor_id)
            print(f"\nFound motor with ID: {motor_id}")
    print("\nScan complete.")
    return sorted(found_motors)

def calibrate_paired_motors(packetHandler, motor_id1, motor_id2):
    print("-" * 60)
    print(f"Calibrating PAIRED SERVOS {motor_id1} and {motor_id2}")
    print("Move the shared joint by hand to its MINIMUM and MAXIMUM positions.")
    print("The script will automatically record the limits.")
    print("Press ENTER when you are finished.")
    print("-" * 60)

    min_pos1, max_pos1 = 4095, 0
    min_pos2, max_pos2 = 4095, 0
    try:
        while True:
            if msvcrt.kbhit() and msvcrt.getch() == b'\r':
                break

            pos1, _, _ = packetHandler.read2ByteTxRx(motor_id1, STS_PRESENT_POSITION_L)
            pos2, _, _ = packetHandler.read2ByteTxRx(motor_id2, STS_PRESENT_POSITION_L)

            if pos1 is not None and pos2 is not None:
                min_pos1, max_pos1 = min(pos1, min_pos1), max(pos1, max_pos1)
                min_pos2, max_pos2 = min(pos2, min_pos2), max(pos2, max_pos2)
                print(f"--> M{motor_id1}: {pos1:04d} | M{motor_id2}: {pos2:04d} | Limits M{motor_id1}: {min_pos1}-{max_pos1} M{motor_id2}: {min_pos2}-{max_pos2}", end='\r')
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")

    print("\n" + "="*40)
    print(f"SERVO {motor_id1} -> Min: {min_pos1}, Max: {max_pos1}")
    print(f"SERVO {motor_id2} -> Min: {min_pos2}, Max: {max_pos2}")
    print("="*40 + "\n")
    
    return {str(motor_id1): {'min': min_pos1, 'max': max_pos1}, str(motor_id2): {'min': min_pos2, 'max': max_pos2}}

def calibrate_single_motor(packetHandler, motor_id):
    print("-" * 60)
    print(f"Calibrating SERVO {motor_id}")
    print("Move the motor by hand to its MINIMUM and MAXIMUM positions.")
    print("Press ENTER when you are finished.")
    print("-" * 60)

    min_pos, max_pos = 4095, 0
    try:
        while True:
            if msvcrt.kbhit() and msvcrt.getch() == b'\r':
                break
            pos, _, _ = packetHandler.read2ByteTxRx(motor_id, STS_PRESENT_POSITION_L)
            if pos is not None:
                min_pos, max_pos = min(pos, min_pos), max(pos, max_pos)
                print(f"--> Current Position: {pos:04d} | Min Found: {min_pos:04d} | Max Found: {max_pos:04d}", end='\r')
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
    
    print("\n" + "="*20)
    print(f"SERVO {motor_id} -> Min: {min_pos}, Max: {max_pos}")
    print("="*20 + "\n")
    return {str(motor_id): {'min': min_pos, 'max': max_pos}}

def main():
    selected_port = get_port()
    if not selected_port: return

    portHandler = PortHandler(selected_port)
    packetHandler = sts(portHandler)

    try:
        if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
            raise IOError("Failed to open port or set baudrate")
        
        print(f"Successfully opened port {selected_port}.\n")
        motor_ids = find_motors(packetHandler)
        if not motor_ids:
            print("No motors found. Exiting."); return

        print("\nSwitching all motors to Velocity Control Mode for consistent position reading...")
        for motor_id in motor_ids:
            packetHandler.WheelMode(motor_id)
        
        print("Disabling torque on all motors. You can now move the arm by hand.\n")
        for motor_id in motor_ids:
            packetHandler.write1ByteTxRx(motor_id, STS_TORQUE_ENABLE, 0)

        calibration_results = {}
        remaining_ids = list(motor_ids)

        if 2 in remaining_ids and 3 in remaining_ids:
            results = calibrate_paired_motors(packetHandler, 2, 3)
            calibration_results.update(results)
            remaining_ids.remove(2); remaining_ids.remove(3)

        for motor_id in remaining_ids:
            results = calibrate_single_motor(packetHandler, motor_id)
            calibration_results.update(results)

        print("\n--- FINAL CALIBRATION RESULTS ---")
        print(json.dumps(calibration_results, indent=4))
        
        if input("Save this data to 'jacob_ik/servo_limits.json'? (y/n): ").lower() == 'y':
            try:
                with open('jacob_ik/servo_limits.json', 'w') as f:
                    json.dump(calibration_results, f, indent=4)
                print("Successfully saved.")
            except Exception as e:
                print(f"Error saving file: {e}")
        else:
            print("Save cancelled.")

    except IOError as e:
        print(f"Error: {e}")
    finally:
        if 'portHandler' in locals() and portHandler.is_open:
            portHandler.closePort()
            print("\nPort closed.")

if __name__ == "__main__":
    main()
