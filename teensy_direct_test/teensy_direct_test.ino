/*
 * ST3215 DIRECT TEST - Teensy 4.1
 * ================================
 * Bypass Waveshare board completely - connect Teensy directly to servo.
 * 
 * WIRING (no resistor needed for TX-only test):
 * - Servo V (red)   -> External 6-7.4V power (NOT from Teensy!)
 * - Servo G (black) -> Teensy GND (MUST share ground!)
 * - Servo D (data)  -> Teensy Pin 1 (TX1)
 * 
 * The servo connector pinout (looking at the connector holes):
 *   [G] [V] [D]   or   [D] [V] [G]
 *   Check your servo's marking!
 * 
 * Use jumper wires or carefully insert into the connector.
 * 
 * This sketch will:
 * 1. Try all servo IDs (1-6)
 * 2. Enable torque
 * 3. Move servos
 * 4. Try wheel mode
 */

void sendPacket(HardwareSerial& ser, uint8_t id, uint8_t instruction, uint8_t* params, uint8_t paramLen) {
    uint8_t length = paramLen + 2;
    uint8_t checksum = id + length + instruction;
    
    ser.write(0xFF);
    ser.write(0xFF);
    ser.write(id);
    ser.write(length);
    ser.write(instruction);
    
    for (int i = 0; i < paramLen; i++) {
        ser.write(params[i]);
        checksum += params[i];
    }
    
    ser.write(~checksum & 0xFF);
    ser.flush();  // Wait for TX to complete
}

void enableTorque(HardwareSerial& ser, uint8_t id) {
    uint8_t params[] = {40, 1};
    sendPacket(ser, id, 0x03, params, 2);
}

void setPosition(HardwareSerial& ser, uint8_t id, uint16_t position, uint16_t time_ms) {
    uint8_t params[] = {
        42,
        position & 0xFF,
        (position >> 8) & 0xFF,
        time_ms & 0xFF,
        (time_ms >> 8) & 0xFF
    };
    sendPacket(ser, id, 0x03, params, 5);
}

void setWheelMode(HardwareSerial& ser, uint8_t id) {
    uint8_t unlock[] = {55, 0};
    sendPacket(ser, id, 0x03, unlock, 2);
    delay(50);
    
    uint8_t mode[] = {33, 1};
    sendPacket(ser, id, 0x03, mode, 2);
    delay(50);
    
    uint8_t lock[] = {55, 1};
    sendPacket(ser, id, 0x03, lock, 2);
    delay(50);
}

void setPositionMode(HardwareSerial& ser, uint8_t id) {
    uint8_t unlock[] = {55, 0};
    sendPacket(ser, id, 0x03, unlock, 2);
    delay(50);
    
    uint8_t mode[] = {33, 0};
    sendPacket(ser, id, 0x03, mode, 2);
    delay(50);
    
    uint8_t lock[] = {55, 1};
    sendPacket(ser, id, 0x03, lock, 2);
    delay(50);
}

void setSpeed(HardwareSerial& ser, uint8_t id, int16_t speed, uint8_t acc) {
    uint16_t stsSpeed = speed;
    if (speed < 0) {
        stsSpeed = (-speed) | 0x8000;
    }
    
    uint8_t params[] = {
        41, acc,
        0, 0,
        0, 0,
        stsSpeed & 0xFF,
        (stsSpeed >> 8) & 0xFF
    };
    sendPacket(ser, id, 0x03, params, 8);
}

void setup() {
    Serial.begin(115200);   // USB for debug output
    Serial1.begin(1000000); // Servo communication at 1Mbps
    
    delay(2000);  // Wait for USB serial and servo power
    
    Serial.println("====================================");
    Serial.println("ST3215 DIRECT TEENSY TEST");
    Serial.println("====================================");
    Serial.println();
    Serial.println("Make sure:");
    Serial.println("  - Servo D (data) -> Teensy Pin 1 (TX1)");
    Serial.println("  - Servo G -> Teensy GND");
    Serial.println("  - Servo V -> 6-7.4V external power");
    Serial.println();
    Serial.println("Starting test in 3 seconds...");
    Serial.println("WATCH THE SERVO!");
    delay(3000);
}

void loop() {
    // Try ALL servo IDs 1-6
    for (uint8_t id = 1; id <= 6; id++) {
        Serial.print("\n\n========== TESTING SERVO ID ");
        Serial.print(id);
        Serial.println(" ==========");
        
        // Enable torque
        Serial.println("Enabling torque...");
        enableTorque(Serial1, id);
        delay(200);
        
        // Move to center
        Serial.println("Moving to center (2048)...");
        setPosition(Serial1, id, 2048, 1000);
        delay(1500);
        
        // Move to 1500
        Serial.println("Moving to 1500...");
        setPosition(Serial1, id, 1500, 500);
        delay(1000);
        
        // Move to 2500
        Serial.println("Moving to 2500...");
        setPosition(Serial1, id, 2500, 500);
        delay(1000);
        
        // Back to center
        Serial.println("Back to center...");
        setPosition(Serial1, id, 2048, 500);
        delay(1000);
    }
    
    // Also try broadcast ID
    Serial.println("\n\n========== BROADCAST TEST (all servos) ==========");
    
    Serial.println("Enabling torque on ALL servos...");
    enableTorque(Serial1, 254);  // Broadcast ID
    delay(200);
    
    Serial.println("Moving ALL to 2048...");
    setPosition(Serial1, 254, 2048, 1000);
    delay(2000);
    
    Serial.println("Moving ALL to 1500...");
    setPosition(Serial1, 254, 1500, 1000);
    delay(2000);
    
    Serial.println("Moving ALL to 2500...");
    setPosition(Serial1, 254, 2500, 1000);
    delay(2000);
    
    Serial.println("\n========== WHEEL MODE TEST ==========");
    
    // Try wheel mode on ID 1 only
    Serial.println("Setting wheel mode on ID 1...");
    setWheelMode(Serial1, 1);
    enableTorque(Serial1, 1);
    delay(100);
    
    Serial.println("Spinning clockwise...");
    setSpeed(Serial1, 1, 400, 50);
    delay(2000);
    
    Serial.println("Spinning counter-clockwise...");
    setSpeed(Serial1, 1, -400, 50);
    delay(2000);
    
    Serial.println("Stopping...");
    setSpeed(Serial1, 1, 0, 50);
    delay(500);
    
    Serial.println("Restoring position mode...");
    setPositionMode(Serial1, 1);
    enableTorque(Serial1, 1);
    setPosition(Serial1, 1, 2048, 1000);
    delay(2000);
    
    Serial.println("\n\n====================================");
    Serial.println("TEST CYCLE COMPLETE");
    Serial.println("Did any servo move?");
    Serial.println("  YES = Servos are OK, Waveshare board is dead");
    Serial.println("  NO  = Check wiring or servo may be damaged");
    Serial.println("====================================");
    Serial.println("\nRestarting in 10 seconds...\n");
    delay(10000);
}
