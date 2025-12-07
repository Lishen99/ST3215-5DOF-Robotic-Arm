/*
 * ST3215 SERVO TEST - Arduino Uno
 * ================================
 * TX-only test to verify if servos respond to commands.
 * 
 * WIRING:
 * - Servo V (red) -> External 6-7.4V power (NOT Arduino 5V!)
 * - Servo G (black) -> Arduino GND
 * - Servo D (data) -> Arduino Pin 1 (TX)
 * 
 * NOTE: Disconnect Pin 1 when uploading! (TX is shared with USB)
 * 
 * After upload:
 * 1. Disconnect USB
 * 2. Connect servo D to Arduino TX (pin 1)
 * 3. Power Arduino with external power or reconnect USB
 * 4. Watch the servo - it should move!
 */

// STS Protocol helpers
void sendPacket(uint8_t id, uint8_t instruction, uint8_t* params, uint8_t paramLen) {
    uint8_t length = paramLen + 2;  // params + instruction + checksum
    uint8_t checksum = id + length + instruction;
    
    Serial.write(0xFF);  // Header
    Serial.write(0xFF);  // Header
    Serial.write(id);    // Servo ID
    Serial.write(length);
    Serial.write(instruction);
    
    for (int i = 0; i < paramLen; i++) {
        Serial.write(params[i]);
        checksum += params[i];
    }
    
    Serial.write(~checksum & 0xFF);  // Checksum
}

void enableTorque(uint8_t id) {
    uint8_t params[] = {40, 1};  // Register 40 = Torque Enable, value = 1
    sendPacket(id, 0x03, params, 2);  // 0x03 = WRITE
}

void disableTorque(uint8_t id) {
    uint8_t params[] = {40, 0};
    sendPacket(id, 0x03, params, 2);
}

void setPosition(uint8_t id, uint16_t position, uint16_t time_ms) {
    // Write to register 42 (Goal Position) + 44 (Goal Time)
    uint8_t params[] = {
        42,                       // Start register
        position & 0xFF,          // Position low byte
        (position >> 8) & 0xFF,   // Position high byte
        time_ms & 0xFF,           // Time low byte
        (time_ms >> 8) & 0xFF     // Time high byte
    };
    sendPacket(id, 0x03, params, 5);
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

void setSpeed(uint8_t id, int16_t speed, uint8_t acc) {
    // Convert signed speed to STS format
    uint16_t stsSpeed = speed;
    if (speed < 0) {
        stsSpeed = (-speed) | 0x8000;
    }
    
    // WriteSpec: ACC, POS_L, POS_H, TIME_L, TIME_H, SPEED_L, SPEED_H
    uint8_t params[] = {
        41,                       // Start register (ACC)
        acc,                      // Acceleration
        0, 0,                     // Position (ignored in wheel mode)
        0, 0,                     // Time (ignored in wheel mode)
        stsSpeed & 0xFF,          // Speed low byte
        (stsSpeed >> 8) & 0xFF    // Speed high byte
    };
    sendPacket(id, 0x03, params, 8);
}

void setup() {
    // ST3215 default baud rate is 1000000
    // Arduino Uno can't do 1Mbps reliably, so we'll try 115200
    // YOU MAY NEED TO CHANGE SERVO BAUD RATE FIRST with Waveshare board
    
    // Try 1000000 first (may or may not work on Uno)
    Serial.begin(1000000);
    
    delay(1000);  // Wait for servo to power up
}

void loop() {
    uint8_t servoId = 1;  // Test servo ID 1
    
    // ===== TEST 1: Enable torque =====
    enableTorque(servoId);
    delay(500);
    
    // ===== TEST 2: Move to position 2048 (center) =====
    setPosition(servoId, 2048, 1000);
    delay(2000);
    
    // ===== TEST 3: Move to position 1500 =====
    setPosition(servoId, 1500, 1000);
    delay(2000);
    
    // ===== TEST 4: Move to position 2500 =====
    setPosition(servoId, 2500, 1000);
    delay(2000);
    
    // ===== TEST 5: Try wheel mode spin =====
    setWheelMode(servoId);
    delay(100);
    enableTorque(servoId);
    delay(100);
    
    // Spin clockwise
    setSpeed(servoId, 300, 50);
    delay(2000);
    
    // Spin counter-clockwise
    setSpeed(servoId, -300, 50);
    delay(2000);
    
    // Stop
    setSpeed(servoId, 0, 50);
    delay(500);
    
    // Back to position mode
    setPositionMode(servoId);
    delay(100);
    enableTorque(servoId);
    delay(100);
    
    // Move to center
    setPosition(servoId, 2048, 1000);
    delay(3000);
    
    // Repeat forever...
}
