#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include <vector>
#include <TeensyThreads.h>

// STS Protocol Definitions - EXACT COPY FROM ESP32
#define STS_INST_PING 0x01
#define STS_INST_READ 0x02
#define STS_INST_WRITE 0x03
#define STS_INST_REG_WRITE 0x04
#define STS_INST_ACTION 0x05
#define STS_INST_SYNC_READ 0x82
#define STS_INST_SYNC_WRITE 0x83

#define STS_GOAL_POSITION_L 42
#define STS_GOAL_SPEED_L 46
#define STS_PRESENT_POSITION_L 56
#define STS_PRESENT_SPEED_L 58
#define STS_LOCK 55
#define STS_MODE 33
#define STS_TORQUE_ENABLE 40
#define STS_ACC 41

// Communication timeouts (ms) - Reduced for 1000Hz operation
#define STS_RX_TIMEOUT 3     // 3ms max per servo response (was 15ms)
#define STS_BYTE_TIMEOUT 2   // 2ms between bytes (was 5ms)

// Direction switching delays (microseconds) - SAME AS ESP32
#define STS_DIR_DELAY_US 50
#define STS_TX_DELAY_US 100

class ServoDriver {
public:
    // Teensy version - no direction pin needed (UART driver board handles half-duplex)
    ServoDriver(HardwareSerial& serial);
    void begin(unsigned long baud = 1000000);

    bool ping(uint8_t id);
    int readPosition(uint8_t id);
    void writePosition(uint8_t id, int position, int speed, int acc);
    void setWheelMode(uint8_t id);
    void setTorqueEnable(uint8_t id, bool enable);
    void unlockEEPROM(uint8_t id);
    
    void syncWriteVelocity(const std::vector<uint8_t>& ids, const std::vector<int>& speeds);
    bool syncReadPosition(const std::vector<uint8_t>& ids, std::vector<int>& positions);
    int readPositionSingle(uint8_t id);
    
    uint32_t getSuccessCount() { return _successCount; }
    uint32_t getFailCount() { return _failCount; }
    void resetStats() { _successCount = 0; _failCount = 0; }

private:
    HardwareSerial& _serial;
    unsigned long _baud;
    
    uint32_t _successCount;
    uint32_t _failCount;
    
    // Mutex for thread-safe access (TeensyThreads)
    Threads::Mutex _mutex;

    void sendPacket(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params);
    bool receivePacket(uint8_t& id, std::vector<uint8_t>& data, unsigned long timeout_ms = STS_RX_TIMEOUT);
    void clearRxBuffer();
    bool waitForBytes(size_t count, unsigned long timeout_ms);
};

#endif
