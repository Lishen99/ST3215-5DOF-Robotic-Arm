#include "ServoDriver.h"

ServoDriver::ServoDriver(HardwareSerial& serial) 
    : _serial(serial),
      _baud(1000000), _successCount(0), _failCount(0) {
    // Mutex is auto-initialized by TeensyThreads
}

void ServoDriver::begin(unsigned long baud) {
    _baud = baud;
    // Teensy Serial1 uses fixed pins (0=RX, 1=TX)
    // No direction pin needed - UART driver board handles half-duplex
    _serial.begin(baud, SERIAL_8N1);
    // Teensy has large built-in buffers, no need to resize
    delay(10);
    clearRxBuffer();
}

void ServoDriver::clearRxBuffer() {
    while (_serial.available()) {
        _serial.read();
    }
}

bool ServoDriver::waitForBytes(size_t count, unsigned long timeout_ms) {
    unsigned long start = millis();
    while ((size_t)_serial.available() < count) {
        if (millis() - start > timeout_ms) return false;
        delayMicroseconds(50);  // Shorter delay for faster polling
    }
    return true;
}

void ServoDriver::sendPacket(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params) {
    // Lock mutex using Threads::Scope (RAII-style)
    Threads::Scope lock(_mutex);
    
    clearRxBuffer();
    
    std::vector<uint8_t> packet;
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(id);
    packet.push_back(params.size() + 2);
    packet.push_back(instruction);
    
    uint8_t checksum = id + (params.size() + 2) + instruction;
    for (uint8_t p : params) {
        packet.push_back(p);
        checksum += p;
    }
    packet.push_back(~checksum);
    
    _serial.write(packet.data(), packet.size());
    _serial.flush();  // CRITICAL: Wait for TX to complete
    
    // Match ESP32 timing: 100μs before switch + 50μs after = 150μs total
    // Waveshare board needs time to finish TX and switch buffer IC to RX mode
    delayMicroseconds(200);
}

bool ServoDriver::receivePacket(uint8_t& id, std::vector<uint8_t>& data, unsigned long timeout_ms) {
    unsigned long start = millis();
    
    if (!waitForBytes(6, timeout_ms)) {
        return false;
    }

    int header_count = 0;
    int bytes_scanned = 0;
    const int max_scan = 50;
    
    while (header_count < 2 && bytes_scanned < max_scan) {
        if (_serial.available()) {
            uint8_t b = _serial.read();
            bytes_scanned++;
            if (b == 0xFF) {
                header_count++;
            } else {
                header_count = 0;
            }
        }
        if (millis() - start > timeout_ms) return false;
    }
    
    if (header_count < 2) return false;

    if (!waitForBytes(3, STS_BYTE_TIMEOUT)) {
        return false;
    }
    
    uint8_t rx_id = _serial.read();
    uint8_t rx_len = _serial.read();
    uint8_t rx_err = _serial.read();
    
    if (rx_len < 2 || rx_len > 50) {
        return false;
    }
    
    int param_len = rx_len - 2;
    
    data.clear();
    uint8_t checksum = rx_id + rx_len + rx_err;
    
    if (param_len > 0) {
        if (!waitForBytes(param_len, STS_BYTE_TIMEOUT)) {
            return false;
        }
        
        for (int i = 0; i < param_len; i++) {
            uint8_t val = _serial.read();
            data.push_back(val);
            checksum += val;
        }
    }
    
    if (!waitForBytes(1, STS_BYTE_TIMEOUT)) {
        return false;
    }
    
    uint8_t rx_chk = _serial.read();
    if (rx_chk != (uint8_t)(~checksum)) {
        return false; 
    }
    
    id = rx_id;
    return true;
}

int ServoDriver::readPositionSingle(uint8_t id) {
    // Lock mutex using Threads::Scope (RAII-style)
    Threads::Scope lock(_mutex);
    
    clearRxBuffer();
    
    std::vector<uint8_t> packet;
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(id);
    packet.push_back(4);
    packet.push_back(STS_INST_READ);
    packet.push_back(STS_PRESENT_POSITION_L);
    packet.push_back(2);
    
    uint8_t checksum = id + 4 + STS_INST_READ + STS_PRESENT_POSITION_L + 2;
    packet.push_back(~checksum);
    
    _serial.write(packet.data(), packet.size());
    _serial.flush();  // CRITICAL: Wait for TX to complete
    
    // Match ESP32 timing for half-duplex turnaround
    delayMicroseconds(200);
    
    uint8_t rx_id;
    std::vector<uint8_t> data;
    
    if (receivePacket(rx_id, data, STS_RX_TIMEOUT)) {
        if (rx_id == id && data.size() >= 2) {
            int pos = data[0] + (data[1] << 8);
            _successCount++;
            return pos;
        }
    }
    
    _failCount++;
    return -1;
}

bool ServoDriver::syncReadPosition(const std::vector<uint8_t>& ids, std::vector<int>& positions) {
    // Lock mutex using Threads::Scope (RAII-style)
    Threads::Scope lock(_mutex);
    
    clearRxBuffer();
    
    std::vector<uint8_t> packet;
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(0xFE);
    packet.push_back(ids.size() + 4);
    packet.push_back(STS_INST_SYNC_READ);
    packet.push_back(STS_PRESENT_POSITION_L);
    packet.push_back(2);
    
    uint8_t checksum = 0xFE + (ids.size() + 4) + STS_INST_SYNC_READ + STS_PRESENT_POSITION_L + 2;
    for (uint8_t id : ids) {
        packet.push_back(id);
        checksum += id;
    }
    packet.push_back(~checksum);
    
    _serial.write(packet.data(), packet.size());
    _serial.flush();  // CRITICAL: Wait for TX to complete
    
    // Match ESP32 timing: 100μs before switch + 50μs after = 150μs total
    // Waveshare board needs time to finish TX and switch buffer IC to RX mode
    delayMicroseconds(200);
    
    positions.assign(ids.size(), -1);
    
    delayMicroseconds(500);
    
    int received = 0;
    unsigned long start = millis();
    unsigned long timeout = STS_RX_TIMEOUT * ids.size() + 10;
    
    while (received < (int)ids.size() && (millis() - start) < timeout) {
        uint8_t rx_id;
        std::vector<uint8_t> data;
        
        if (receivePacket(rx_id, data, STS_BYTE_TIMEOUT)) {
            for (size_t j = 0; j < ids.size(); j++) {
                if (ids[j] == rx_id && positions[j] == -1) {
                    if (data.size() >= 2) {
                        int pos = data[0] + (data[1] << 8);
                        positions[j] = pos;
                        received++;
                        _successCount++;
                    }
                    break;
                }
            }
        } else {
            delayMicroseconds(200);
        }
    }
    
    for (size_t i = 0; i < ids.size(); i++) {
        if (positions[i] == -1) {
            _failCount++;
        }
    }
    
    return received > 0;
}

void ServoDriver::setWheelMode(uint8_t id) {
    std::vector<uint8_t> params;
    params.push_back(STS_MODE);
    params.push_back(1);
    sendPacket(id, STS_INST_WRITE, params);
    delay(5);
}

void ServoDriver::setTorqueEnable(uint8_t id, bool enable) {
    std::vector<uint8_t> params;
    params.push_back(STS_TORQUE_ENABLE);
    params.push_back(enable ? 1 : 0);
    sendPacket(id, STS_INST_WRITE, params);
    delay(5);
}

void ServoDriver::unlockEEPROM(uint8_t id) {
    std::vector<uint8_t> params;
    params.push_back(STS_LOCK);
    params.push_back(0);  // 0 = unlocked
    sendPacket(id, STS_INST_WRITE, params);
    delay(5);
}

void ServoDriver::syncWriteVelocity(const std::vector<uint8_t>& ids, const std::vector<int>& speeds) {
    // Lock mutex using Threads::Scope (RAII-style)
    Threads::Scope lock(_mutex);
    
    clearRxBuffer();
    
    std::vector<uint8_t> packet;
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(0xFE);
    uint8_t data_len = 2;
    packet.push_back((data_len + 1) * ids.size() + 4);
    packet.push_back(STS_INST_SYNC_WRITE);
    packet.push_back(STS_GOAL_SPEED_L);
    packet.push_back(data_len);
    
    uint8_t checksum = 0xFE + ((data_len + 1) * ids.size() + 4) + STS_INST_SYNC_WRITE + STS_GOAL_SPEED_L + data_len;
    
    for (size_t i = 0; i < ids.size(); i++) {
        packet.push_back(ids[i]);
        checksum += ids[i];
        
        int s = speeds[i];
        if (s < 0) { 
            s = -s; 
            s |= (1<<15);
        }
        
        uint8_t low = s & 0xFF;
        uint8_t high = (s >> 8) & 0xFF;
        
        packet.push_back(low);
        packet.push_back(high);
        checksum += low + high;
    }
    packet.push_back(~checksum);
    
    _serial.write(packet.data(), packet.size());
    _serial.flush();  // CRITICAL: Wait for TX to complete
    
    // Match ESP32 timing: sync write doesn't expect response, but still need settling time
    // Keep at 100μs for write-only operations
    delayMicroseconds(100);
}

bool ServoDriver::ping(uint8_t id) {
    Threads::Scope lock(_mutex);
    
    clearRxBuffer();
    
    std::vector<uint8_t> packet;
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    packet.push_back(id);
    packet.push_back(2);
    packet.push_back(STS_INST_PING);
    uint8_t checksum = id + 2 + STS_INST_PING;
    packet.push_back(~checksum);
    
    // Debug: print what we're sending
    Serial.printf("PING TX[%d]: ", id);
    for (size_t i = 0; i < packet.size(); i++) {
        Serial.printf("%02X ", packet[i]);
    }
    Serial.println();
    
    _serial.write(packet.data(), packet.size());
    _serial.flush();
    
    // Wait longer for response - try different delays
    delay(20);
    
    // Debug: print what we received
    Serial.printf("PING RX[%d]: avail=%d bytes: ", id, _serial.available());
    int count = 0;
    while (_serial.available() && count < 20) {
        Serial.printf("%02X ", _serial.read());
        count++;
    }
    Serial.println();
    
    return false;  // Temporarily return false to see debug output
}

int ServoDriver::readPosition(uint8_t id) {
    return readPositionSingle(id);
}

void ServoDriver::writePosition(uint8_t id, int position, int speed, int acc) {
    std::vector<uint8_t> params;
    params.push_back(STS_GOAL_POSITION_L);
    params.push_back(position & 0xFF);
    params.push_back((position >> 8) & 0xFF);
    params.push_back(0);  // time low
    params.push_back(0);  // time high
    params.push_back(speed & 0xFF);
    params.push_back((speed >> 8) & 0xFF);
    params.push_back(acc);
    sendPacket(id, STS_INST_WRITE, params);
}
