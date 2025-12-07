#!/usr/bin/env python
"""
MOTOR 1 FOCUSED TEST - Using SDK
=================================
This test focuses on just motor 1:
1. Enable wheel mode (mode = 1)
2. Enable torque
3. Spin in velocity mode

Watch motor 1 closely!
"""

import sys
import time
sys.path.append("stservo-env")

from STservo_sdk import *

# Configuration
STS_ID = 1
BAUDRATE = 1000000
DEVICENAME = 'COM3'

# Speed settings
SPEED_CW = 500      # Clockwise speed
SPEED_CCW = -500    # Counter-clockwise speed
ACC = 50            # Acceleration

def main():
    print("=" * 60)
    print("MOTOR 1 FOCUSED TEST - Wheel/Velocity Mode")
    print("=" * 60)
    
    # Initialize port
    portHandler = PortHandler(DEVICENAME)
    packetHandler = sts(portHandler)
    
    # Open port
    if portHandler.openPort():
        print(f"[OK] Port {DEVICENAME} opened")
    else:
        print(f"[FAIL] Could not open port {DEVICENAME}")
        return
    
    # Set baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print(f"[OK] Baudrate set to {BAUDRATE}")
    else:
        print("[FAIL] Could not set baudrate")
        portHandler.closePort()
        return
    
    print()
    print("=" * 60)
    print("STEP 1: Unlock EPROM (to allow mode change)")
    print("=" * 60)
    
    # Unlock EPROM to allow writing mode
    result, error = packetHandler.unLockEprom(STS_ID)
    print(f"  unLockEprom() -> result: {result}, error: {error}")
    if result != COMM_SUCCESS:
        print(f"  TX Result: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.1)
    
    print()
    print("=" * 60)
    print("STEP 2: Set Wheel Mode (Mode = 1)")
    print("=" * 60)
    
    # Set wheel mode (velocity control mode)
    result, error = packetHandler.WheelMode(STS_ID)
    print(f"  WheelMode() -> result: {result}, error: {error}")
    if result != COMM_SUCCESS:
        print(f"  TX Result: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.1)
    
    print()
    print("=" * 60)
    print("STEP 3: Lock EPROM")
    print("=" * 60)
    
    result, error = packetHandler.LockEprom(STS_ID)
    print(f"  LockEprom() -> result: {result}, error: {error}")
    if result != COMM_SUCCESS:
        print(f"  TX Result: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.1)
    
    print()
    print("=" * 60)
    print("STEP 4: Enable Torque")
    print("=" * 60)
    
    # Enable torque (register 40 = 1)
    result, error = packetHandler.write1ByteTxRx(STS_ID, STS_TORQUE_ENABLE, 1)
    print(f"  Torque Enable -> result: {result}, error: {error}")
    if result != COMM_SUCCESS:
        print(f"  TX Result: {packetHandler.getTxRxResult(result)}")
    time.sleep(0.1)
    
    print()
    print("=" * 60)
    print("STEP 5: SPIN CLOCKWISE")
    print("=" * 60)
    print(f"  Speed: {SPEED_CW}, Acceleration: {ACC}")
    print("  >>> WATCH MOTOR 1! <<<")
    
    result, error = packetHandler.WriteSpec(STS_ID, SPEED_CW, ACC)
    print(f"  WriteSpec() -> result: {result}, error: {error}")
    if result != COMM_SUCCESS:
        print(f"  TX Result: {packetHandler.getTxRxResult(result)}")
    
    print("\n  Waiting 3 seconds...")
    time.sleep(3)
    
    print()
    print("=" * 60)
    print("STEP 6: SPIN COUNTER-CLOCKWISE")
    print("=" * 60)
    print(f"  Speed: {SPEED_CCW}, Acceleration: {ACC}")
    
    result, error = packetHandler.WriteSpec(STS_ID, SPEED_CCW, ACC)
    print(f"  WriteSpec() -> result: {result}, error: {error}")
    if result != COMM_SUCCESS:
        print(f"  TX Result: {packetHandler.getTxRxResult(result)}")
    
    print("\n  Waiting 3 seconds...")
    time.sleep(3)
    
    print()
    print("=" * 60)
    print("STEP 7: STOP")
    print("=" * 60)
    
    result, error = packetHandler.WriteSpec(STS_ID, 0, ACC)
    print(f"  WriteSpec(0) -> result: {result}, error: {error}")
    if result != COMM_SUCCESS:
        print(f"  TX Result: {packetHandler.getTxRxResult(result)}")
    
    time.sleep(0.5)
    
    # Disable torque
    result, error = packetHandler.write1ByteTxRx(STS_ID, STS_TORQUE_ENABLE, 0)
    print(f"  Torque Disable -> result: {result}, error: {error}")
    
    # Set back to position mode (mode = 0)
    print()
    print("=" * 60)
    print("STEP 8: Restore Position Mode (Mode = 0)")
    print("=" * 60)
    
    packetHandler.unLockEprom(STS_ID)
    time.sleep(0.05)
    result, error = packetHandler.write1ByteTxRx(STS_ID, STS_MODE, 0)
    print(f"  Set Mode=0 -> result: {result}, error: {error}")
    packetHandler.LockEprom(STS_ID)
    
    portHandler.closePort()
    print("\n[DONE] Port closed")
    
    print()
    print("=" * 60)
    print("RESULTS")
    print("=" * 60)
    print()
    print("Did motor 1 spin?")
    print()
    print("  YES -> TX works! Servos work! Only RX is broken.")
    print("         You can use arm in 'blind mode' or replace board.")
    print()
    print("  NO  -> TX path is also broken or motor 1 is damaged.")
    print("         Try testing with a different servo if you have one.")
    print()

if __name__ == "__main__":
    main()
