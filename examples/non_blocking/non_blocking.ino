#include <Wire.h>
#include <MCP3424.h>

MCP3424 adc(PIN_FLOAT, PIN_FLOAT);

void setup() {

    Serial.begin(57600);

    Wire.begin();

    adc.generalCall(GC_RESET);

    adc.creg[CH1].bits = { GAINx1, R18B, CONTINUOUS, CH1, 1 };
}

double value;

static char * errmsg[] = {"", "underflow", "overflow", "i2c", "in progress", "timeout"};

void loop() {
    ConvStatus err = adc.read(CH1, value, false);
    if (err == R_OK) 
      Serial.println(value, DEC); 
    else if (err != R_IN_PROGRESS) {
      Serial.print("conversion error: ");
      Serial.println(errmsg[err]);
    }
    asm volatile ("nop");
}
