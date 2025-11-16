

import sys
import os
import time
import msvcrt
import json
import serial.tools.list_ports

# Add the SDK to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), "stservo-env"))

from STservo_sdk import *

# --- Constants ---
BAUDRATE = 1000000
MAX_MOTOR_ID = 10 # Scan up to ID 10 for efficiency
PROTOCOL_VERSION = 2.0

# --- Main Application ---
def get_available_ports():
    """Gets the list of available serial ports."""
    return [port.device for port in serial.tools.list_ports.comports()]

def find_motors(packetHandler, portHandler):
    """Scans for connected servo motors."""
    print(f"Scanning for motors up to ID {MAX_MOTOR_ID}...")
    found_motors = []
    for motor_id in range(MAX_MOTOR_ID + 1):
        print(f"Pinging ID: {motor_id}", end='\r')
        # The function to use depends on the SDK's implementation, ping() is common
        scs_model_number, scs_comm_result, scs_error = packetHandler.ping(motor_id)
        if scs_comm_result == COMM_SUCCESS:
            found_motors.append(motor_id)
            print(f"\nFound motor with ID: {motor_id}")
    print("\nScan complete.")
    return sorted(found_motors)

def disable_torque_all(packetHandler, motor_ids):
    """Disables torque for a list of motors."""
    print("Disabling torque on all found motors...")
    for motor_id in motor_ids:
        packetHandler.write1ByteTxRx(motor_id, STS_TORQUE_ENABLE, 0)
        print(f"  - Torque disabled for motor ID {motor_id}")
    print("All motors are now free to be moved manually.\n")

def calibrate_single_motor(packetHandler, motor_id):
    """Guides the user through calibrating a single motor."""
    print("---------------------------------------------------------")
    print(f"Calibrating SERVO {motor_id}")
    print("Move the motor to its MINIMUM and MAXIMUM positions.")
    print("Press ENTER when you are finished.")
    print("---------------------------------------------------------")

    min_pos = 4095
    max_pos = 0
    try:
        while True:
            # Check for key press
            if msvcrt.kbhit() and msvcrt.getch() == b'\r':
                break

            pos, _, _ = packetHandler.read2ByteTxRx(motor_id, STS_PRESENT_POSITION_L)
            if pos is not None:
                if pos < min_pos: min_pos = pos
                if pos > max_pos: max_pos = pos
                print(f"--> Current Position: {pos:04d} | Min: {min_pos:04d} | Max: {max_pos:04d}", end='\r')
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
    
    print("\n" + "="*20)
    print(f"SERVO {motor_id} -> Min Position: {min_pos}, Max Position: {max_pos}")
    print("="*20 + "\n")
    return {motor_id: {'min': min_pos, 'max': max_pos}}

def calibrate_paired_motors(packetHandler, motor_id1, motor_id2):
    """Guides the user through calibrating two paired motors."""
    print("---------------------------------------------------------")
    print(f"Calibrating PAIRED SERVOS {motor_id1} and {motor_id2}")
    print("Move the shared joint to its MINIMUM and MAXIMUM positions.")
    print("Press ENTER when you are finished.")
    print("---------------------------------------------------------")

    min_pos1, max_pos1 = 4095, 0
    min_pos2, max_pos2 = 4095, 0
    try:
        while True:
            if msvcrt.kbhit() and msvcrt.getch() == b'\r':
                break

            pos1, _, _ = packetHandler.read2ByteTxRx(motor_id1, STS_PRESENT_POSITION_L)
            pos2, _, _ = packetHandler.read2ByteTxRx(motor_id2, STS_PRESENT_POSITION_L)

            if pos1 is not None and pos2 is not None:
                if pos1 < min_pos1: min_pos1 = pos1
                if pos1 > max_pos1: max_pos1 = pos1
                if pos2 < min_pos2: min_pos2 = pos2
                if pos2 > max_pos2: max_pos2 = pos2
                print(f"--> M{motor_id1}: {pos1:04d} | M{motor_id2}: {pos2:04d} | M{motor_id1} Min/Max: {min_pos1:04d}/{max_pos1:04d} | M{motor_id2} Min/Max: {min_pos2:04d}/{max_pos2:04d}", end='\r')
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")

    print("\n" + "="*40)
    print(f"SERVO {motor_id1} -> Min Position: {min_pos1}, Max Position: {max_pos1}")
    print(f"SERVO {motor_id2} -> Min Position: {min_pos2}, Max Position: {max_pos2}")
    # Based on motor_gui, pos2 should be the inverse of pos1. Let's check.
    # A perfect alignment would have pos1 + pos2 = 4095
    print(f"Alignment Check (pos1 + pos2): Min Sum = {min_pos1 + max_pos2}, Max Sum = {max_pos1 + min_pos2}")
    print("="*40 + "\n")
    
    results = {}
    results[motor_id1] = {'min': min_pos1, 'max': max_pos1}
    results[motor_id2] = {'min': min_pos2, 'max': max_pos2}
    return results

def main():
    """Main execution function."""
    ports = get_available_ports()
    if not ports:
        print("Error: No COM ports found. Please check your connection.")
        return

    print("Available COM ports:", ports)
    # For simplicity, we'll use the first available port.
    # You could expand this to let the user choose.
    selected_port = ports[1]
    print(f"Using port: {selected_port}\n")

    portHandler = PortHandler(selected_port)
    packetHandler = sts(portHandler)

    try:
        if not portHandler.openPort():
            raise IOError("Failed to open port")
        if not portHandler.setBaudRate(BAUDRATE):
            raise IOError("Failed to set baudrate")
        
        print(f"Successfully opened port {selected_port} at {BAUDRATE} baud.\n")

        motor_ids = find_motors(packetHandler, portHandler)

        if not motor_ids:
            print("No motors found. Exiting.")
            return

        disable_torque_all(packetHandler, motor_ids)
        
        calibration_results = {}
        
        # Use a copy of the list to modify it while iterating
        remaining_ids = list(motor_ids)

        # Calibrate paired motors first
        if 2 in remaining_ids and 3 in remaining_ids:
            results = calibrate_paired_motors(packetHandler, 2, 3)
            calibration_results.update(results)
            remaining_ids.remove(2)
            remaining_ids.remove(3)

        # Calibrate remaining single motors
        for motor_id in remaining_ids:
            results = calibrate_single_motor(packetHandler, motor_id)
            calibration_results.update(results)

        print("\n--- FINAL CALIBRATION RESULTS ---")
        for motor_id, limits in sorted(calibration_results.items()):
            print(f"Motor ID {motor_id}: Min={limits['min']}, Max={limits['max']}")
        print("---------------------------------")

        # Save results to a JSON file
        try:
            with open('servo_limits.json', 'w') as f:
                json.dump(calibration_results, f, indent=4)
            print("\nSuccessfully saved calibration data to servo_limits.json")
        except Exception as e:
            print(f"\nError saving calibration data: {e}")

        print("\nCalibration complete. You can now use these values in your main script.")

    except IOError as e:
        print(f"Error: {e}")
    finally:
        if 'portHandler' in locals() and portHandler.is_open:
            # Re-enable torque on closing if desired, or leave disabled
            # for motor_id in motor_ids:
            #     packetHandler.write1ByteTxRx(motor_id, STS_TORQUE_ENABLE, 1)
            portHandler.closePort()
            print("\nPort closed.")

if __name__ == "__main__":
    main()

