#include "ServoDriver.h"

ServoDriver::ServoDriver(HardwareSerial& serial, int dir_pin, int rx_pin, int tx_pin) 
    : _serial(serial), _dir_pin(dir_pin), _rx_pin(rx_pin), _tx_pin(tx_pin),
      _baud(1000000), _successCount(0), _failCount(0) {
    _mutex = xSemaphoreCreateMutex();
}

void ServoDriver::begin(unsigned long baud) {
    _baud = baud;
    _serial.begin(baud, SERIAL_8N1, _rx_pin, _tx_pin);
    _serial.setRxBufferSize(256);
    pinMode(_dir_pin, OUTPUT);
    setDirection(false);
    delay(10);
    clearRxBuffer();
}

void ServoDriver::setDirection(bool tx) {
    if (tx) {
        digitalWrite(_dir_pin, HIGH);
        delayMicroseconds(STS_DIR_DELAY_US);
    } else {
        delayMicroseconds(STS_TX_DELAY_US);
        digitalWrite(_dir_pin, LOW);
        delayMicroseconds(STS_DIR_DELAY_US);
    }
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
        delayMicroseconds(100);
    }
    return true;
}

void ServoDriver::sendPacket(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    clearRxBuffer();
    setDirection(true);
    
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
    _serial.flush();
    
    setDirection(false);
    xSemaphoreGive(_mutex);
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
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return -1;
    }
    
    clearRxBuffer();
    setDirection(true);
    
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
    _serial.flush();
    
    setDirection(false);
    
    uint8_t rx_id;
    std::vector<uint8_t> data;
    
    if (receivePacket(rx_id, data, STS_RX_TIMEOUT)) {
        if (rx_id == id && data.size() >= 2) {
            int pos = data[0] + (data[1] << 8);
            _successCount++;
            xSemaphoreGive(_mutex);
            return pos;
        }
    }
    
    _failCount++;
    xSemaphoreGive(_mutex);
    return -1;
}

bool ServoDriver::syncReadPosition(const std::vector<uint8_t>& ids, std::vector<int>& positions) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        positions.assign(ids.size(), -1);
        return false;
    }
    
    clearRxBuffer();
    setDirection(true);
    
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
    _serial.flush();
    
    setDirection(false);
    
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
    
    xSemaphoreGive(_mutex);
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
    // Write 0 to LOCK register (addr 55) to unlock servo for writes
    std::vector<uint8_t> params;
    params.push_back(STS_LOCK);
    params.push_back(0);  // 0 = unlocked
    sendPacket(id, STS_INST_WRITE, params);
    delay(5);
}

void ServoDriver::syncWriteVelocity(const std::vector<uint8_t>& ids, const std::vector<int>& speeds) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    clearRxBuffer();
    setDirection(true);
    
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
    _serial.flush();
    
    setDirection(false);
    xSemaphoreGive(_mutex);
}
