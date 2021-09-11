/**************************************************************************/
/*!
  @file     PAC1720.h
  Author: Atsushi Sasaki(https://github.com/aselectroworks)
  License: MIT (see LICENSE)
*/
/**************************************************************************/

#ifndef PAC1720_H
#define PAC1720_H

#include <Wire.h>
#include <stdbool.h>
#include <stdint.h>

// Device Address // 7-bit Address
typedef enum {
    ADDR_RES_0      = 0b1001100, 
    ADDR_RES_100    = 0b1001101, 
    ADDR_RES_180    = 0b1001110, 
    ADDR_RES_300    = 0b1001111, 
    ADDR_RES_430    = 0b1001000, 
    ADDR_RES_560    = 0b1001001, 
    ADDR_RES_750    = 0b1001010, 
    ADDR_RES_1270   = 0b1001011, 
    ADDR_RES_1600   = 0b0101000, 
    ADDR_RES_2000   = 0b0101001, 
    ADDR_RES_2700   = 0b0101010, 
    ADDR_RES_3600   = 0b0101011, 
    ADDR_RES_5600   = 0b0101100, 
    ADDR_RES_9100   = 0b0101101, 
    ADDR_RES_20000  = 0b0101110, 
    ADDR_RES_OPEN   = 0b0011000, 
} PAC1720_ADDR_Enum; 

// Register Address
#define PAC1720_ADDR_CONFIG 0x00
typedef union {
    uint8_t raw;
    struct {
        bool c1vds : 1;
        bool c1ids : 1;
        bool tout : 1;
        bool c2vds : 1;
        bool c2ids : 1;
        bool mskal : 1;
        bool cden : 1;
        bool rsv : 1;
    };
} PAC1720_CONFIGURATION_REG;
#define PAC1720_ADDR_CONV_RATE 0x01
typedef enum {
    ONE_SAMPLE_PER_SEC = 0, 
    TWO_SAMPLE_PER_SEC, 
    FOUR_SAMPLE_PER_SEC, 
    CONTINUOUS,     
} CONVERSION_RATE_Enum; 
typedef union {
    uint8_t raw;
    struct {
        CONVERSION_RATE_Enum conv : 2;
        uint8_t rsv : 6;
    };
} PAC1720_CONVERSION_RATE_REG;
#define PAC1720_ADDR_ONE_SHOT 0x02
#define PAC1720_ADDR_CHANNEL_MASK 0x03
typedef union {
    uint8_t raw;
    struct {
        bool c1vsr : 1; 
        bool c1vs : 1; 
        bool c2vsr : 1; 
        bool c2vs : 1; 
        uint8_t rsv : 4;
    };
} PAC1720_CHANNEL_MASK_REG;
#define PAC1720_ADDR_HIGH_LIM_STATUS 0x04
typedef union {
    uint8_t raw;
    struct {
        bool c1vrh : 1; 
        bool c1vsh : 1; 
        bool c2vrh : 1; 
        bool c2vsh : 1;
        uint8_t rsv : 3;
        bool cvdn : 1; 
    };
} PAC1720_HIGH_LIMIT_STATUS_REG;
#define PAC1720_ADDR_LOW_LIM_STATUS 0x05
typedef union {
    uint8_t raw;
    struct {
        bool c1vrl : 1; 
        bool c1vsl : 1; 
        bool c2vrl : 1; 
        bool c2vsl : 1;
        uint8_t rsv : 4;
    };
} PAC1720_LOW_LIMIT_STATUS_REG;
#define PAC1720_ADDR_VSRC_CONFIG 0x0A
typedef enum {
    SAMPLE_2_5MS = 0, 
    SAMPLE_5MS, 
    SAMPLE_10MS, 
    SAMPLE_20MS, 
    SAMPLE_40MS, 
    SAMPLE_80MS, 
    SAMPLE_160MS, 
    SAMPLE_320MS, 
} SAMPLING_TIME_SETTING_Enum; 
typedef enum {
    AVG_DISABLED = 0, 
    AVG_TWO, 
    AVG_FOUR, 
    AVG_EIGHT, 
} AVERAGING_SETTING_Enum; 
typedef union {
    uint8_t raw;
    struct {
        AVERAGING_SETTING_Enum c1ra : 2; 
        SAMPLING_TIME_SETTING_Enum c1rs : 2; 
        AVERAGING_SETTING_Enum c2ra : 2; 
        SAMPLING_TIME_SETTING_Enum c2rs : 2; 
    };
} PAC1720_VSRC_SAMPLING_CONFIG_REG;
#define PAC1720_ADDR_CH1_VSEN_CONFIG 0x0B
#define PAC1720_ADDR_CH2_VSEN_CONFIG 0x0C
typedef enum {
    PLUS_MINUS_10MV = 0, 
    PLUS_MINUS_20MV, 
    PLUS_MINUS_40MV, 
    PLUS_MINUS_80MV, 
} CURRENT_SENSING_RANGE_SETTING_Enum; 
typedef union {
    uint8_t raw;
    struct {
        CURRENT_SENSING_RANGE_SETTING_Enum cxsr : 2; 
        AVERAGING_SETTING_Enum cxsa : 2; 
        SAMPLING_TIME_SETTING_Enum cxcss : 3; 
        uint8_t rsv : 1; 
    };
} PAC1720_VSEN_SAMPLING_CONFIG_REG;
#define PAC1720_ADDR_CH1_VSEN_HIGH 0x0D
#define PAC1720_ADDR_CH1_VSEN_LOW 0x0E
#define PAC1720_ADDR_CH2_VSEN_HIGH 0x0F
#define PAC1720_ADDR_CH2_VSEN_LOW 0x10
typedef union {
    uint16_t raw; 
    struct {
        uint8_t rsv : 4; 
        uint16_t cxsr : 12; 
    }; 
} PAC1720_VSEN_RESULT_REG; 
#define PAC1720_ADDR_CH1_VSRC_HIGH 0x11
#define PAC1720_ADDR_CH1_VSRC_LOW 0x12
#define PAC1720_ADDR_CH2_VSRC_HIGH 0x13
#define PAC1720_ADDR_CH2_VSRC_LOW 0x14
typedef union {
    uint16_t raw; 
    struct {
        uint8_t rsv : 5; 
        uint16_t cxsr : 11; 
    }; 
} PAC1720_VSRC_RESULT_REG; 
#define PAC1720_ADDR_CH1_PWR_RATIO_HIGH 0x15
#define PAC1720_ADDR_CH1_PWR_RATIO_LOW 0x16
#define PAC1720_ADDR_CH2_PWR_RATIO_HIGH 0x17
#define PAC1720_ADDR_CH2_PWR_RATIO_LOW 0x18

#define PAC1720_ADDR_CH1_VSEN_HIGH_LIM 0x19
#define PAC1720_ADDR_CH2_VSEN_HIGH_LIM 0x1A
#define PAC1720_ADDR_CH1_VSEN_LOW_LIM 0x1B
#define PAC1720_ADDR_CH2_VSEN_LOW_LIM 0x1C

#define PAC1720_ADDR_CH1_VSRC_HIGH_LIM 0x1D
#define PAC1720_ADDR_CH2_VSRC_HIGH_LIM 0x1E
#define PAC1720_ADDR_CH1_VSRC_LOW_LIM 0x1F
#define PAC1720_ADDR_CH2_VSRC_LOW_LIM 0x20

#define PAC1720_ADDR_PID 0xFD
#define PAC1720_ADDR_MID 0xFE
#define PAC1720_ADDR_REV 0xFF

typedef enum {
    I2C_CLOCK_100KHZ = 100000, 
    I2C_CLOCK_400KHZ = 400000, 
    I2C_CLOCK_1MHZ =  1000000, 
} PAC1720_I2C_CLOCK_SPEED; 

// Uncomment to enable debug messages
//#define PAC1720_DEBUG

// Define where debug output will be printed
#define DEBUG_PRINTER Serial

// Setup debug printing macros
#ifdef PAC1720_DEBUG
#define DEBUG_PRINT(...)                                                   \
    {                                                                      \
        DEBUG_PRINTER.printf("[PAC1720(0x%02x)]: ",                        \
                             _deviceAddress); \
        DEBUG_PRINTER.print(__VA_ARGS__);                                  \
    }
#define DEBUG_PRINTLN(...)                                                 \
    {                                                                      \
        DEBUG_PRINTER.printf("[PAC1720(0x%02x)]: ",                        \
                             _deviceAddress); \
        DEBUG_PRINTER.println(__VA_ARGS__);                                \
    }
#define DEBUG_PRINTF(...)                                                  \
    {                                                                      \
        DEBUG_PRINTER.printf("[PAC1720(0x%02x)]: ",                        \
                             _deviceAddress); \
        DEBUG_PRINTER.printf(__VA_ARGS__);                                 \
    }
#else
#define DEBUG_PRINT(...) \
    {}
#define DEBUG_PRINTLN(...) \
    {}
#define DEBUG_PRINTF(...) \
    {}
#endif

class PAC1720 {
   public:
    PAC1720(void);
#if defined(ESP32) || defined(ESP8266)
    PAC1720(int8_t sda, int8_t scl, PAC1720_ADDR_Enum ad);
#endif
    virtual ~PAC1720();

    void begin();
    void begin(float shuntResistance);

    void setClockSpeed(PAC1720_I2C_CLOCK_SPEED speed); 
    void setAddress(PAC1720_ADDR_Enum ad); 

    uint16_t readVoltageRaw(uint8_t ch);
    float readVoltage(uint8_t ch);
    float readVoltage_mV(uint8_t ch);

    int16_t readShuntVoltageRaw(uint8_t ch);
    float readCurrent(uint8_t ch);
    float readCurrent_mA(uint8_t ch);
    float readCurrent_uA(uint8_t ch);

    float readPower(uint8_t ch);

    void setVsrcConfig(AVERAGING_SETTING_Enum avg, SAMPLING_TIME_SETTING_Enum st); 
    void setVsenConfig(AVERAGING_SETTING_Enum avg, SAMPLING_TIME_SETTING_Enum st, CURRENT_SENSING_RANGE_SETTING_Enum rng); 

    void enableMeasurement(bool c1i_en, bool c1v_en, bool c2i_en, bool c2v_en); 
    
    void enableAlertPin(bool conv_en, bool limit_en); 
    void setChannelMask(bool c1i_en, bool c1v_en, bool c2i_en, bool c2v_en); 
    PAC1720_HIGH_LIMIT_STATUS_REG getHighLimitStatus(); 
    PAC1720_LOW_LIMIT_STATUS_REG getLowLimitStatus(); 
    void setVoltageHighLimit(float ch1_lim, float ch2_lim);  
    void setVoltageLowLimit(float ch1_lim, float ch2_lim); 
    void setCurrentHighLimit(float ch1_lim, float ch2_lim); 
    void setCurrentLowLimit(float ch1_lim, float ch2_lim); 

    uint8_t readPID();
    uint8_t readMID();
    uint8_t readREV();

   private:
    int8_t _sda = -1;
    int8_t _scl = -1;

    // Device Address
    PAC1720_ADDR_Enum _deviceAddress = ADDR_RES_OPEN;

    // VSOURCE Avaragin Setting 
    AVERAGING_SETTING_Enum _vsrc_samp_avg = AVG_DISABLED; 
    // VSOURCE Sampling Time
    SAMPLING_TIME_SETTING_Enum _vsrc_samp_time = SAMPLE_10MS; 
    // Full-Scale Voltage
    float _fsv; 
    
    // Current-Sensing Avaraging Setting
    AVERAGING_SETTING_Enum _cs_samp_avg = AVG_DISABLED; 
    // Current-Sensing Sampling Time
    SAMPLING_TIME_SETTING_Enum _cs_samp_time = SAMPLE_80MS; 
    // Current-Sensing Range
    CURRENT_SENSING_RANGE_SETTING_Enum _cs_rng = PLUS_MINUS_80MV; 
    // Full-Scale Current
    float _fsc; 

    // Full-Scale Power
    float _fsp; 
    
    // Shunt Resistance Parameter
    float _shuntResistance; 

    void calcFSC(); 
    void calcFSV(); 
    void calcFSP(); 
    void readMultiByte(uint8_t addr, uint8_t size, uint8_t* data);
    uint16_t readWord(uint8_t addr);
    uint8_t readByte(uint8_t addr);
    void writeMultiByte(uint8_t addr, uint8_t* data, uint8_t size);
    void writeWord(uint8_t addr, uint16_t data);
    void writeByte(uint8_t addr, uint8_t data);
};

#endif