

#include <Wire.h>
#include <MCP3424.h>

MCP3424 adc(0x68);

void setup() {

    Serial.begin(57600);

    Wire.begin();

    adc.generalCall(GC_RESET);

    adc.creg[CH1].bits = { GAINx1, R12B, ONE_SHOT, CH1, 1 };
    adc.creg[CH2].bits = { GAINx1, R14B, ONE_SHOT, CH2, 1 };
    adc.creg[CH3].bits = { GAINx1, R16B, ONE_SHOT, CH3, 1 };
    adc.creg[CH4].bits = { GAINx1, R18B, ONE_SHOT, CH4, 1 };

}

double value;
Channel active_ch = CH1;
bool blocking = false;

static char * errmsg[] = {"", "underflow", "overflow", "i2c", "in progress", "timeout"};

void loop() {

    ConvStatus error = adc.read(active_ch, value, blocking);
    if (error == R_OK || error == R_OVERFLOW || error == R_UNDERFLOW) {
        if (error != R_OK) {
            Serial.print("error: ");
            Serial.println(error, DEC);
        } else {
            Serial.print("CH");
            Serial.print(1+active_ch, DEC);
            Serial.print(": ");
            Serial.println(value, DEC);
        }
        switch (active_ch) {
          case CH1: active_ch = CH2; break;
          case CH2: active_ch = CH3; break;
          case CH3: active_ch = CH4; break;
          case CH4: active_ch = CH1; break;
        }
        if (!blocking)
          adc.startNewConversion(active_ch);
    } else {
        Serial.print("CH");
        Serial.print(1+active_ch, DEC);
        Serial.print(": ");
        Serial.print("error: ");
        Serial.println(errmsg[error]);
    }
    asm volatile ("nop");
}
