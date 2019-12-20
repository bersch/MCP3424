/*
 *  Copyright (C) 2014 Bernhard Schneider <bernhard@neaptide.org>
 *   
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public License
 *  version 3 as published by the Free Software Foundation.
 * 
 */

#include <Arduino.h>
#include <MCP3424.h>

MCP3424::MCP3424 (uint8_t address): addr(address) { }

MCP3424::MCP3424 (PinType adr0, PinType adr1): addr(pin_addr[adr0*3+adr1]) {}

uint8_t MCP3424::generalCall(GCall_t call) const {
    Wire.beginTransmission(0x00);
    Wire.write(call);
    return Wire.endTransmission();
}

uint8_t MCP3424::writeConfReg(Channel ch) {
    cwrite.reg = creg[ch].reg;
    Wire.beginTransmission(addr);
    Wire.write(cwrite.reg);
    return Wire.endTransmission();
}

uint8_t MCP3424::startNewConversion(Channel ch) {
    creg[ch].bits.rdy = 1;
    return writeConfReg(ch);
}

/* tries to find the highest gain */
Gain MCP3424::findGain(double value) const {

    uint8_t g;
    
    value = abs(value);
    
    for(g = GAINx1; g <= GAINx8; g++) 
      if (value * (1<<(g+1)) >= 2.048) 
        return (Gain)g;
        
    return GAINx8;
}    

ConvStatus MCP3424::read(Channel ch, double& value, bool blocking) {

    if (blocking == false)
      return nb_read(ch, value);

    ConvStatus err;
    uint32_t t0 = millis();
    do {
        if ( (millis() - t0) > (conv_time[creg[ch].bits.res]) )
          return R_TIMEOUT;
        err = nb_read(ch, value);
    } while (err == R_IN_PROGRESS);

    return err;

}

ConvStatus MCP3424::nb_read(Channel ch, double & value) {

    int32_t lval;
    uint8_t b2, b3, b4;

    if (cwrite.reg != creg[ch].reg)
      if (creg[ch].bits.cmode == CONTINUOUS)
        writeConfReg(ch);
      else
        startNewConversion(ch);

    Wire.requestFrom(addr, (uint8_t)((cwrite.bits.res == R18B)?4:3));

    if (Wire.available() < ((cwrite.bits.res == R18B)?4:3))
      return R_I2C;

    b2 = Wire.read();
    b3 = Wire.read();

    if (creg[ch].bits.res == R18B)
      b4 = Wire.read();

    cread.reg = Wire.read();

    Wire.endTransmission();

    if (cread.bits.rdy == 1)
      return R_IN_PROGRESS;

    if (cread.bits.res == R18B) {
        lval = ((long)b2 << 24 | b3 << 16 | b4 << 8) >> 8;
    } else {
        lval = (b2 << 8) | b3;
    }

    value = 0.001 * lval / (1 << (cread.bits.res << 1)) / ( 1 << cread.bits.pga);

    if (value >  2.048) return R_OVERFLOW;
    if (value < -2.048) return R_UNDERFLOW;

    return R_OK;
}

