#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include <vector>

// STS Protocol Definitions
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

// Communication timeouts (ms)
#define STS_RX_TIMEOUT 15
#define STS_BYTE_TIMEOUT 5

// Direction switching delays (microseconds)
#define STS_DIR_DELAY_US 50
#define STS_TX_DELAY_US 100

class ServoDriver {
public:
    ServoDriver(HardwareSerial& serial, int dir_pin, int rx_pin, int tx_pin);
    void begin(unsigned long baud = 1000000);

    bool ping(uint8_t id);
    int readPosition(uint8_t id);
    void writePosition(uint8_t id, int position, int speed, int acc);
    void setWheelMode(uint8_t id);
    void setTorqueEnable(uint8_t id, bool enable);
    
    // Sync Write (Velocity Control)
    void syncWriteVelocity(const std::vector<uint8_t>& ids, const std::vector<int>& speeds);
    
    // Sync Read (Position) - Returns true if at least one servo responded
    bool syncReadPosition(const std::vector<uint8_t>& ids, std::vector<int>& positions);
    
    // Individual read - more reliable fallback
    int readPositionSingle(uint8_t id);
    
    // Communication stats
    uint32_t getSuccessCount() { return _successCount; }
    uint32_t getFailCount() { return _failCount; }
    void resetStats() { _successCount = 0; _failCount = 0; }

private:
    HardwareSerial& _serial;
    int _dir_pin;
    int _rx_pin;
    int _tx_pin;
    unsigned long _baud;
    
    // Communication stats
    uint32_t _successCount;
    uint32_t _failCount;
    
    // Mutex for thread-safe access
    SemaphoreHandle_t _mutex;

    void sendPacket(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params);
    bool receivePacket(uint8_t& id, std::vector<uint8_t>& data, unsigned long timeout_ms = STS_RX_TIMEOUT);
    void setDirection(bool tx);
    void clearRxBuffer();
    bool waitForBytes(size_t count, unsigned long timeout_ms);
};

#endif
