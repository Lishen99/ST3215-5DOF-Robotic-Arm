/*
 * ESP32 FAKE SERVO - Bus Sniffer
 * ===============================
 * ESP32 pretends to be a servo and listens to what the Waveshare board sends.
 * This tests if the board's TX circuit is working.
 * 
 * WIRING:
 *   Waveshare D (data) -> ESP32 GPIO16 (RX2)
 *   Waveshare G (ground) -> ESP32 GND
 *   
 * Leave actual servos disconnected for this test.
 * 
 * Open Serial Monitor at 115200 baud to see what's received.
 * Then run the Python test scripts on PC to send commands.
 */

#define RXD2 16  // ESP32 RX2 pin - connect to Waveshare D
#define TXD2 17  // Not used, but needed for Serial2

void setup() {
    Serial.begin(115200);  // USB for debug output
    Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);  // Servo bus at 1Mbps
    
    delay(1000);
    
    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32 FAKE SERVO - BUS SNIFFER");
    Serial.println("========================================");
    Serial.println();
    Serial.println("Wiring:");
    Serial.println("  Waveshare D -> ESP32 GPIO16");
    Serial.println("  Waveshare G -> ESP32 GND");
    Serial.println();
    Serial.println("Now run Python test script on PC...");
    Serial.println("Listening at 1000000 baud...");
    Serial.println();
    Serial.println("Any data received will appear below:");
    Serial.println("----------------------------------------");
}

unsigned long lastByteTime = 0;
int packetCount = 0;
int byteCount = 0;

void loop() {
    if (Serial2.available()) {
        // Start of new packet after gap
        if (millis() - lastByteTime > 10) {
            if (byteCount > 0) {
                Serial.println();  // End previous line
            }
            packetCount++;
            Serial.print("[PKT ");
            Serial.print(packetCount);
            Serial.print("] ");
        }
        
        uint8_t b = Serial2.read();
        byteCount++;
        
        // Print byte in hex
        if (b < 0x10) Serial.print("0");
        Serial.print(b, HEX);
        Serial.print(" ");
        
        lastByteTime = millis();
    }
    
    // Print summary every 5 seconds if we have data
    static unsigned long lastSummary = 0;
    if (millis() - lastSummary > 5000) {
        if (byteCount > 0) {
            Serial.println();
            Serial.print("--- Total: ");
            Serial.print(byteCount);
            Serial.print(" bytes, ");
            Serial.print(packetCount);
            Serial.println(" packets ---");
        } else {
            Serial.println("(no data received yet...)");
        }
        lastSummary = millis();
    }
}
