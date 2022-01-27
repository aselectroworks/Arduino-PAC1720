/**************************************************************************/
/*!
  @file     PAC1720.cpp
  Author: Atsushi Sasaki (https://github.com/aselectroworks)
  License: MIT (see LICENSE)
*/
/**************************************************************************/

#include "PAC1720.h"
#include "HardwareSerial.h"

PAC1720::PAC1720() {
#ifdef PAC1720_DEBUG
    DEBUG_PRINTER.end(); 
    delay(100); 
    DEBUG_PRINTER.begin(115200);
#endif
    DEBUG_PRINTLN("Call Contructor");
}
#if defined(ESP32) || defined(ESP8266)
PAC1720::PAC1720(int8_t sda, int8_t scl, PAC1720_ADDR_Enum ad) 
: _sda(sda), _scl(scl), _deviceAddress(ad) {
#ifdef PAC1720_DEBUG
    DEBUG_PRINTER.end();
    delay(100); 
    DEBUG_PRINTER.begin(115200);
#endif
    DEBUG_PRINTLN("Call Contructor");
}
#endif
PAC1720::~PAC1720() {

}

void PAC1720::begin() {
    // Initial wait after Power Up
    delay(100);
    // Initialize I2C
    if(_sda != -1 && _scl != -1) {
        Wire.begin(_sda, _scl); 
    }
    else {
        Wire.begin(); 
    }
    // Check PID
    uint8_t pid = readPID(); 
    if(pid == 0x58) {
        DEBUG_PRINTF("PAC1710 is detected\n"); 
    }
    else if(pid == 0x57) {
        DEBUG_PRINTF("PAC1720 is detected\n"); 
    }
}
void PAC1720::begin(float shuntResistance) {
    _shuntResistance = shuntResistance; 
    calcFSV(); 
    calcFSC(); 
    calcFSP(); 

    begin();
}
void PAC1720::end() {
    enableMeasurement(false, false, false, false); 
}

void PAC1720::setClockSpeed(PAC1720_I2C_CLOCK_SPEED speed) {
    Wire.setClock(speed);  
}

void PAC1720::setAddress(PAC1720_ADDR_Enum ad) {
    _deviceAddress = ad; 
} 

uint16_t PAC1720::readVoltageRaw(uint8_t ch) {
    if(ch == 1) {
        return (readWord(PAC1720_ADDR_CH1_VSRC_HIGH) >> (8 - _vsrc_samp_time));
    }
    if(ch == 2) {
        return (readWord(PAC1720_ADDR_CH2_VSRC_HIGH) >> (8 - _vsrc_samp_time));
    }
    return 0; 
}
float PAC1720::readVoltage(uint8_t ch) {
    return _fsv * (readVoltageRaw(ch) / (float)((256 * (1 << _vsrc_samp_time)) - 1)); 
}
float PAC1720::readVoltage_mV(uint8_t ch) {
    return readVoltage(ch) * 1000; 
}

int16_t PAC1720::readShuntVoltageRaw(uint8_t ch) {
    if(ch == 1) {
        return ((int16_t)readWord(PAC1720_ADDR_CH1_VSEN_HIGH) >> (9 - ((_cs_samp_time < 5) ? _cs_samp_time : 5)));
    }
    if(ch == 2) {
        return ((int16_t)readWord(PAC1720_ADDR_CH2_VSEN_HIGH) >> (9 - ((_cs_samp_time < 5) ? _cs_samp_time : 5)));
    }
    return 0; 
}

float PAC1720::readCurrent(uint8_t ch) {
    return _fsc * readShuntVoltageRaw(ch) / ((_cs_samp_time < 5) ? (64 * (1 << _cs_samp_time)) - 1 : 2047); 
}
float PAC1720::readCurrent_mA(uint8_t ch) {
    return readCurrent(ch) * 1000; 
}
float PAC1720::readCurrent_uA(uint8_t ch) {
    return readCurrent_mA(ch) * 1000; 
}

float PAC1720::readPower(uint8_t ch) {
    if(ch == 1) {
        return _fsp * readWord(PAC1720_ADDR_CH1_PWR_RATIO_HIGH) / 65535; 
    }
    if(ch == 2) {
        return _fsp * readWord(PAC1720_ADDR_CH2_PWR_RATIO_HIGH) / 65535; 
    }
    return 0; 
}

void PAC1720::setVsrcConfig(AVERAGING_SETTING_Enum avg, SAMPLING_TIME_SETTING_Enum st) {
    _vsrc_samp_avg = avg; 
    _vsrc_samp_time = st; 
    calcFSV(); 
    calcFSP(); 
    PAC1720_VSRC_SAMPLING_CONFIG_REG conf; 
    conf.c1ra = _vsrc_samp_avg; 
    conf.c1rs = _vsrc_samp_time; 
    conf.c2ra = _vsrc_samp_avg; 
    conf.c2rs = _vsrc_samp_time; 
    writeByte(PAC1720_ADDR_VSRC_CONFIG, conf.raw); 
}
void PAC1720::setVsenConfig(AVERAGING_SETTING_Enum avg, SAMPLING_TIME_SETTING_Enum st, CURRENT_SENSING_RANGE_SETTING_Enum rng) {
    _cs_samp_avg = avg; 
    _cs_samp_time = st; 
    _cs_rng = rng; 
    calcFSC(); 
    PAC1720_VSEN_SAMPLING_CONFIG_REG conf; 
    conf.cxsr = _cs_rng; 
    conf.cxsa = _cs_samp_avg; 
    conf.cxcss = _cs_samp_time; 
    writeByte(PAC1720_ADDR_CH1_VSEN_CONFIG, conf.raw); 
    writeByte(PAC1720_ADDR_CH2_VSEN_CONFIG, conf.raw); 
}

void PAC1720::enableMeasurement(bool c1i_en, bool c1v_en, bool c2i_en, bool c2v_en) {
    PAC1720_CONFIGURATION_REG conf; 
    conf.raw = readByte(PAC1720_ADDR_CONFIG); 
    conf.c1ids = !c1i_en; 
    conf.c1vds = !c1v_en; 
    conf.c2ids = !c2i_en; 
    conf.c2vds = !c2v_en; 
    writeByte(PAC1720_ADDR_CONFIG, conf.raw); 
}

void PAC1720::enableAlertPin(bool conv_en, bool limit_en) {
    PAC1720_CONFIGURATION_REG conf; 
    conf.raw = readByte(PAC1720_ADDR_CONFIG); 
    conf.cden = conv_en; 
    conf.mskal = limit_en; 
    writeByte(PAC1720_ADDR_CONFIG, conf.raw); 
}

void PAC1720::setChannelMask(bool c1i_en, bool c1v_en, bool c2i_en, bool c2v_en) {
    PAC1720_CHANNEL_MASK_REG conf; 
    conf.c1vs = !c1i_en;     
    conf.c1vsr = !c1v_en;     
    conf.c2vs = !c2i_en;     
    conf.c2vsr = !c2v_en;     
    writeByte(PAC1720_ADDR_CHANNEL_MASK, conf.raw); 
}

PAC1720_HIGH_LIMIT_STATUS_REG PAC1720::getHighLimitStatus() {
    PAC1720_HIGH_LIMIT_STATUS_REG status; 
    status.raw =  readByte(PAC1720_ADDR_HIGH_LIM_STATUS); 
    return status; 
}

PAC1720_LOW_LIMIT_STATUS_REG PAC1720::getLowLimitStatus() {
    PAC1720_LOW_LIMIT_STATUS_REG status; 
    status.raw =  readByte(PAC1720_ADDR_LOW_LIM_STATUS); 
    return status; 
}

void PAC1720::setVoltageHighLimit(float ch1_lim, float ch2_lim) {
    DEBUG_PRINTF("VSRC1 HIGH LIM: %d / RAW %f\n", (uint16_t)(ch1_lim / 0.15625), ch1_lim); 
    writeByte(PAC1720_ADDR_CH1_VSRC_HIGH_LIM, (uint16_t)(ch1_lim / 0.15625)); 
    DEBUG_PRINTF("VSRC2 HIGH LIM: %d / RAW %f\n", (uint16_t)(ch2_lim / 0.15625), ch2_lim); 
    writeByte(PAC1720_ADDR_CH2_VSRC_HIGH_LIM, (uint16_t)(ch2_lim / 0.15625)); 
}
void PAC1720::setVoltageLowLimit(float ch1_lim, float ch2_lim) {
    DEBUG_PRINTF("VSRC1 LOW LIM: %d / RAW %f\n", (uint16_t)(ch1_lim / 0.15625), ch1_lim); 
    writeByte(PAC1720_ADDR_CH1_VSRC_LOW_LIM, (uint16_t)(ch1_lim / 0.15625)); 
    DEBUG_PRINTF("VSRC2 LOW LIM: %d / RAW %f\n", (uint16_t)(ch2_lim / 0.15625), ch2_lim); 
    writeByte(PAC1720_ADDR_CH2_VSRC_LOW_LIM, (uint16_t)(ch2_lim / 0.15625)); 
}
void PAC1720::setCurrentHighLimit(float ch1_lim, float ch2_lim) {
    DEBUG_PRINTF("VSENSE1 HIGH LIM: %d / RAW %f\n", (int16_t)(ch1_lim / ((10 << _cs_rng) * 0.001 / _shuntResistance) * 2047) >> 4, ch1_lim); 
    DEBUG_PRINTF("VSENSE2 HIGH LIM: %d / RAW %f\n", (int16_t)(ch2_lim / ((10 << _cs_rng) * 0.001 / _shuntResistance) * 2047) >> 4, ch2_lim); 
    writeByte(PAC1720_ADDR_CH1_VSEN_HIGH_LIM, (int16_t)(ch1_lim / ((10 << _cs_rng) * 0.001 / _shuntResistance) * 2047) >> 4); 
    writeByte(PAC1720_ADDR_CH2_VSEN_HIGH_LIM, (int16_t)(ch2_lim / ((10 << _cs_rng) * 0.001 / _shuntResistance) * 2047) >> 4); 
}
void PAC1720::setCurrentLowLimit(float ch1_lim, float ch2_lim) {
    DEBUG_PRINTF("VSENSE1 LOW LIM: %d / RAW %f\n", (int16_t)(ch1_lim / ((10 << _cs_rng) * 0.001 / _shuntResistance) * 2047) >> 4, ch1_lim); 
    DEBUG_PRINTF("VSENSE2 LOW LIM: %d / RAW %f\n", (int16_t)(ch2_lim / ((10 << _cs_rng) * 0.001 / _shuntResistance) * 2047) >> 4, ch2_lim); 
    writeByte(PAC1720_ADDR_CH1_VSEN_LOW_LIM, (int16_t)(ch1_lim / ((10 << _cs_rng) * 0.001 / _shuntResistance) * 2047) >> 4); 
    writeByte(PAC1720_ADDR_CH2_VSEN_LOW_LIM, (int16_t)(ch2_lim / ((10 << _cs_rng) * 0.001 / _shuntResistance) * 2047) >> 4); 
}

uint8_t PAC1720::readPID() {
    return readByte(PAC1720_ADDR_PID); 
}
uint8_t PAC1720::readMID() {
    return readByte(PAC1720_ADDR_MID); 
}
uint8_t PAC1720::readREV() {
    return readByte(PAC1720_ADDR_MID); 
}

// Private Function
void PAC1720::calcFSC() {
    _fsc = ((10 << _cs_rng) * 0.001 / _shuntResistance); 
}
void PAC1720::calcFSV() {
    _fsv = (40 - (40 / (float)(256 * (1 << _vsrc_samp_time)))); 
}
void PAC1720::calcFSP() {
    _fsp = _fsc * _fsv; 
}

void PAC1720::readMultiByte(uint8_t addr, uint8_t size, uint8_t* data) {
    Wire.beginTransmission(_deviceAddress);
    Wire.write(addr);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_deviceAddress, size);
    for (uint8_t i = 0; i < size; ++i) {
        data[i] = Wire.read();
    }
}
uint16_t PAC1720::readWord(uint8_t addr) {
    uint8_t rxData[2];
    readMultiByte(addr, 2, rxData);
    return ((rxData[0] << 8) | rxData[1]);
}
uint8_t PAC1720::readByte(uint8_t addr) {
    uint8_t rdData = 0xFF; 
    uint8_t rdDataCount; 
    do {
        Wire.beginTransmission(_deviceAddress); 
        Wire.write(addr); 
        Wire.endTransmission(false); // Restart
        delay(10); 
        rdDataCount = Wire.requestFrom(_deviceAddress, 1); 
    } while(rdDataCount == 0); 
    while(Wire.available()) {
        rdData = Wire.read(); 
    }
    return rdData; 

}
void PAC1720::writeMultiByte(uint8_t addr, uint8_t* data, uint8_t size) {
    Wire.beginTransmission(_deviceAddress);
    DEBUG_PRINTF("writeI2C reg 0x%02x\n", addr)
    Wire.write(addr);
    for (uint8_t i = 0; i < size; i++) {
        DEBUG_PRINTF(" -> data[%d]:0x%02x\n", i, data[i])
        Wire.write(data[i]);
    }
    Wire.endTransmission();
}
void PAC1720::writeWord(uint8_t addr, uint16_t data) {
    uint8_t txData[2] = {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0x00FF)};
    writeMultiByte(addr, txData, 2);
}
void PAC1720::writeByte(uint8_t addr, uint8_t data) {
    Wire.beginTransmission(_deviceAddress); 
    Wire.write(addr); 
    Wire.write(data); 
    Wire.endTransmission(); 
}