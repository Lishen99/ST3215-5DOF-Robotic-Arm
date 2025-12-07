"""
BLIND SERVO MOVEMENT TEST
=========================
This script sends movement commands to servos WITHOUT waiting for responses.
If the servos physically move, we know:
  - The servos are working
  - TX (sending commands) works
  - Only RX (receiving responses) is broken

Watch the servos carefully when running this!
"""

import serial
import time

# Configuration
PORT = "COM3"
BAUD = 1000000

def build_packet(servo_id, instruction, params=[]):
    """Build an STS protocol packet"""
    length = len(params) + 2  # params + instruction + checksum
    packet = [0xFF, 0xFF, servo_id, length, instruction] + params
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    return bytes(packet)

def write_position(ser, servo_id, position, speed=500):
    """
    Write position command (instruction 0x03 = WRITE)
    Register 42 (0x2A) = Goal Position (2 bytes)
    Register 44 (0x2C) = Moving Speed (2 bytes)
    """
    # STS servos use register 42 for goal position
    # We'll write 4 bytes: position(2) + time(2)
    pos_l = position & 0xFF
    pos_h = (position >> 8) & 0xFF
    time_l = speed & 0xFF
    time_h = (speed >> 8) & 0xFF
    
    # Instruction 0x03 = WRITE, starting at register 42
    params = [42, pos_l, pos_h, time_l, time_h]
    packet = build_packet(servo_id, 0x03, params)
    
    ser.write(packet)
    print(f"  Sent to ID {servo_id}: position={position}, time={speed}ms")
    print(f"  Packet: {packet.hex(' ')}")

def sync_write_position(ser, positions, speed=1000):
    """
    Sync write to multiple servos at once
    Instruction 0x83 = SYNC_WRITE
    """
    # Start address (42) + data length per servo (4: pos_l, pos_h, time_l, time_h)
    start_addr = 42
    data_len = 4
    
    params = [start_addr, data_len]
    
    for servo_id, position in positions.items():
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF
        time_l = speed & 0xFF
        time_h = (speed >> 8) & 0xFF
        params.extend([servo_id, pos_l, pos_h, time_l, time_h])
    
    # Sync write uses broadcast ID (0xFE)
    packet = build_packet(0xFE, 0x83, params)
    
    ser.write(packet)
    print(f"  Sync write to {len(positions)} servos")
    print(f"  Packet: {packet.hex(' ')}")

def unlock_servo(ser, servo_id):
    """Unlock servo (set torque enable to 0, then 1)"""
    # Register 40 = Torque Enable
    # First disable
    packet = build_packet(servo_id, 0x03, [40, 0])
    ser.write(packet)
    time.sleep(0.01)
    # Then enable
    packet = build_packet(servo_id, 0x03, [40, 1])
    ser.write(packet)
    time.sleep(0.01)

def main():
    print("=" * 50)
    print("BLIND SERVO MOVEMENT TEST")
    print("=" * 50)
    print(f"\nOpening {PORT} at {BAUD} baud...")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.01)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("Port opened successfully!\n")
    except Exception as e:
        print(f"Error opening port: {e}")
        return
    
    print("This test will try to MOVE the servos.")
    print("Watch the physical servos carefully!\n")
    print("If they move, TX works and servos are OK.")
    print("If they don't move, either TX is broken or servos are damaged.\n")
    
    input("Press ENTER to start the test (make sure arm is safe to move)...")
    print()
    
    # Test each servo ID individually
    print("=" * 50)
    print("TEST 1: Individual servo commands")
    print("=" * 50)
    
    for servo_id in range(1, 7):
        print(f"\nTrying servo ID {servo_id}...")
        
        # First unlock/enable the servo
        print(f"  Enabling torque...")
        unlock_servo(ser, servo_id)
        
        # Move to position 2048 (center)
        print(f"  Moving to center (2048)...")
        write_position(ser, servo_id, 2048, 1000)
        time.sleep(0.1)
    
    print("\n" + "=" * 50)
    print("Waiting 3 seconds for movement...")
    print("=" * 50)
    time.sleep(3)
    
    # Now try moving to a different position
    print("\n" + "=" * 50)
    print("TEST 2: Move to offset position")
    print("=" * 50)
    
    for servo_id in range(1, 7):
        print(f"\nMoving servo ID {servo_id} to 1500...")
        write_position(ser, servo_id, 1500, 1000)
        time.sleep(0.1)
    
    print("\n" + "=" * 50)
    print("Waiting 3 seconds for movement...")
    print("=" * 50)
    time.sleep(3)
    
    # Try sync write (all at once)
    print("\n" + "=" * 50)
    print("TEST 3: Sync write (all servos at once)")
    print("=" * 50)
    
    positions = {1: 2048, 2: 2048, 3: 2048, 4: 2048, 5: 2048, 6: 2048}
    print("\nMoving all servos back to center (2048)...")
    sync_write_position(ser, positions, 1500)
    
    print("\n" + "=" * 50)
    print("Waiting 3 seconds for movement...")
    print("=" * 50)
    time.sleep(3)
    
    # Final test - try broadcast
    print("\n" + "=" * 50)
    print("TEST 4: Broadcast command (ID 254)")
    print("=" * 50)
    
    print("\nSending position 1800 to broadcast ID (all servos)...")
    write_position(ser, 254, 1800, 1000)
    
    print("\nWaiting 3 seconds...")
    time.sleep(3)
    
    ser.close()
    
    print("\n" + "=" * 50)
    print("TEST COMPLETE")
    print("=" * 50)
    print("\nResults interpretation:")
    print("  - If servos MOVED: TX works, servos are OK, only RX is broken")
    print("  - If servos DID NOT move: Either TX is broken OR servos are damaged")
    print("\nDid any servos move? (y/n)")

if __name__ == "__main__":
    main()
