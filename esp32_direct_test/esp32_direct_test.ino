/*
 * ESP32 DIRECT SERVO TEST
 * ========================
 * Test servo directly with ESP32 - no Waveshare board needed.
 * 
 * WIRING (simple, no resistor):
 *   ESP32 GPIO17 (TX2) -> Servo D (data)
 *   ESP32 GPIO16 (RX2) -> Servo D (data)  [same wire, tied together]
 *   ESP32 GND          -> Servo G (ground)
 *   ESP32 5V or 3.3V   -> Servo V (power) - for testing only!
 *   
 * NOTE: 3.3V/5V is too low for motor to move, but servo MCU might respond!
 * For actual movement, use external 6-7.4V power.
 * 
 * Open Serial Monitor at 115200 baud.
 */

#define RXD2 16
#define TXD2 17

void sendPacket(uint8_t id, uint8_t instruction, uint8_t* params, uint8_t paramLen) {
    uint8_t length = paramLen + 2;
    uint8_t checksum = id + length + instruction;
    
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(id);
    Serial2.write(length);
    Serial2.write(instruction);
    
    for (int i = 0; i < paramLen; i++) {
        Serial2.write(params[i]);
        checksum += params[i];
    }
    
    Serial2.write(~checksum & 0xFF);
    Serial2.flush();
}

bool pingServo(uint8_t id) {
    // Clear any old data
    while (Serial2.available()) Serial2.read();
    
    // Send ping (instruction 0x01)
    sendPacket(id, 0x01, NULL, 0);
    
    // Wait for response
    delay(50);
    
    int available = Serial2.available();
    if (available > 0) {
        Serial.print("    Response (");
        Serial.print(available);
        Serial.print(" bytes): ");
        while (Serial2.available()) {
            uint8_t b = Serial2.read();
            if (b < 0x10) Serial.print("0");
            Serial.print(b, HEX);
            Serial.print(" ");
        }
        Serial.println();
        return true;
    }
    return false;
}

void enableTorque(uint8_t id) {
    uint8_t params[] = {40, 1};
    sendPacket(id, 0x03, params, 2);
}

void setPosition(uint8_t id, uint16_t position, uint16_t time_ms) {
    uint8_t params[] = {
        42,
        position & 0xFF,
        (position >> 8) & 0xFF,
        time_ms & 0xFF,
        (time_ms >> 8) & 0xFF
    };
    sendPacket(id, 0x03, params, 5);
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);
    
    delay(2000);
    
    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32 DIRECT SERVO TEST");
    Serial.println("========================================");
    Serial.println();
    Serial.println("Wiring:");
    Serial.println("  ESP32 GPIO17 -> Servo D");
    Serial.println("  ESP32 GPIO16 -> Servo D (same wire)");
    Serial.println("  ESP32 GND    -> Servo G");
    Serial.println("  ESP32 5V/3.3V -> Servo V");
    Serial.println();
    Serial.println("NOTE: At 3.3V/5V motor won't move,");
    Serial.println("but we can check if servo responds!");
    Serial.println();
}

void setWheelMode(uint8_t id) {
    // Unlock EPROM
    uint8_t unlock[] = {55, 0};
    sendPacket(id, 0x03, unlock, 2);
    delay(50);
    
    // Set mode to 1 (wheel mode)
    uint8_t mode[] = {33, 1};
    sendPacket(id, 0x03, mode, 2);
    delay(50);
    
    // Lock EPROM
    uint8_t lock[] = {55, 1};
    sendPacket(id, 0x03, lock, 2);
    delay(50);
}

void setSpeed(uint8_t id, int16_t speed, uint8_t acc) {
    uint16_t stsSpeed = speed;
    if (speed < 0) {
        stsSpeed = (-speed) | 0x8000;
    }
    
    uint8_t params[] = {
        41, acc,          // Register 41 = ACC
        0, 0,             // Position (ignored in wheel mode)
        0, 0,             // Time (ignored)
        stsSpeed & 0xFF,
        (stsSpeed >> 8) & 0xFF
    };
    sendPacket(id, 0x03, params, 8);
}

void setPositionMode(uint8_t id) {
    uint8_t unlock[] = {55, 0};
    sendPacket(id, 0x03, unlock, 2);
    delay(50);
    
    uint8_t mode[] = {33, 0};
    sendPacket(id, 0x03, mode, 2);
    delay(50);
    
    uint8_t lock[] = {55, 1};
    sendPacket(id, 0x03, lock, 2);
    delay(50);
}

void loop() {
    Serial.println("========================================");
    Serial.println("MOTOR 1 DIRECT TEST - WHEEL MODE");
    Serial.println("========================================");
    Serial.println();
    
    // Step 1: Ping
    Serial.println("Step 1: Ping servo ID 1...");
    bool found = pingServo(1);
    if (found) {
        Serial.println("  --> SERVO RESPONDED!");
    } else {
        Serial.println("  --> No response (continuing anyway)");
    }
    delay(200);
    
    // Step 2: Unlock EPROM and set wheel mode
    Serial.println("\nStep 2: Setting WHEEL MODE...");
    setWheelMode(1);
    Serial.println("  --> Done");
    delay(200);
    
    // Step 3: Enable torque
    Serial.println("\nStep 3: Enabling TORQUE...");
    enableTorque(1);
    Serial.println("  --> Done");
    delay(200);
    
    // Step 4: Spin clockwise
    Serial.println("\nStep 4: SPINNING CLOCKWISE (speed 400)...");
    Serial.println("  >>> WATCH THE MOTOR! <<<");
    setSpeed(1, 400, 50);
    delay(3000);
    
    // Step 5: Spin counter-clockwise
    Serial.println("\nStep 5: SPINNING COUNTER-CLOCKWISE (speed -400)...");
    setSpeed(1, -400, 50);
    delay(3000);
    
    // Step 6: Stop
    Serial.println("\nStep 6: STOPPING...");
    setSpeed(1, 0, 50);
    delay(500);
    
    // Step 7: Restore position mode
    Serial.println("\nStep 7: Restoring position mode...");
    setPositionMode(1);
    enableTorque(1);
    delay(200);
    
    // Step 8: Move to center
    Serial.println("\nStep 8: Moving to center position (2048)...");
    setPosition(1, 2048, 1000);
    delay(2000);
    
    // Step 9: Move left
    Serial.println("\nStep 9: Moving to 1500...");
    setPosition(1, 1500, 500);
    delay(1500);
    
    // Step 10: Move right
    Serial.println("\nStep 10: Moving to 2500...");
    setPosition(1, 2500, 500);
    delay(1500);
    
    Serial.println("\n========================================");
    Serial.println("TEST CYCLE COMPLETE");
    Serial.println("========================================");
    Serial.println("\nDid motor 1 move at all?");
    Serial.println("  YES -> Servo works! Waveshare board is dead.");
    Serial.println("  NO  -> Check wiring or servo may be damaged.");
    Serial.println("\nRestarting in 10 seconds...\n\n");
    delay(10000);
}
