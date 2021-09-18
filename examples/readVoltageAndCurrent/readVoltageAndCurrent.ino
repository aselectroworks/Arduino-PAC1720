#include "PAC1720.h"

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

PAC1720 PAC1720(I2C_SDA_PIN, I2C_SCL_PIN, ADDR_RES_0);

void setup() {
    Serial.begin(115200);
    delay(10);
    // Begin PAC1720
    PAC1720.begin(0.027);
    // Read Product ID
    Serial.printf("Product ID: 0x%x\n", PAC1720.readPID());
    delay(500);
    PAC1720.setVoltageHighLimit(20, 10); // CH1: 8V, CH2: 10V
    PAC1720.setCurrentHighLimit(0.04, 0.5); // CH1 0.04A, CH2: 0.5A
}

void loop() {
    delay(10);
    Serial.printf("VSRC1: %4d / %2.3f V | VSEN1: %4d / %2.3f mA | ", 
        PAC1720.readVoltageRaw(1), PAC1720.readVoltage(1), PAC1720.readShuntVoltageRaw(1), PAC1720.readCurrent_mA(1)); 
    Serial.printf("Limit Status : H-0x%02x L-0x%02x\r", PAC1720.getHighLimitStatus().raw, PAC1720.getLowLimitStatus().raw); 

}
