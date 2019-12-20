#include <Wire.h>
#include <MCP3424.h>

MCP3424 adc(PIN_LOW, PIN_LOW);

void setup() {

    Serial.begin(57600);

    Wire.begin();

    adc.generalCall(GC_RESET);
}

void show_v(double& value, _ConfReg r) {

    char v[24], buff[48];

    dtostrf(value,8,8,v);
    sprintf(buff, "channel: %d, gain: %d, value: %s", r.bits.ch, 1 << r.bits.pga, v);
    Serial.println(buff);
}

double value;

void loop() {

    for (int i = (int)CH1; i <= (int)CH4; i++) {
        _ConfReg& c = adc.creg[(Channel)i];
        c.bits = { GAINx1, R12B, ONE_SHOT, (Channel)i, 1 };
        ConvStatus err = adc.read(c.bits.ch, value);
        if (err == R_OK) {
            show_v(value, c);
            c.bits.res = R18B;
            c.bits.pga = adc.findGain(value);
            err = adc.read(c.bits.ch, value);
            if (err == R_OK) {
                show_v(value, c);
                Serial.println();
            } else 
              Serial.println(" error.");
        } else {
            Serial.print("conversion error: ");
            Serial.println(err);
        }
    }
    asm volatile ("nop");
}
