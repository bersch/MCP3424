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

uint8_t MCP3424::generalCall(GCall_t call) {
    Wire.beginTransmission(0x00);
    Wire.write(call);
    return Wire.endTransmission();
}

uint8_t MCP3424::writeConfReg(Channel ch) {
    cwrite = creg[ch];
    Wire.beginTransmission(addr);
    Wire.write(cwrite.reg);
    return Wire.endTransmission();
}

uint8_t MCP3424::startNewConversion(Channel ch) {
    creg[ch].rdy = 1;
    return writeConfReg(ch);
}

ConvStatus MCP3424::read(Channel ch, double& value, bool blocking) {

    if (blocking == false)
      return nb_read(ch, value);

    ConvStatus err;
    uint32_t t0 = millis();
    do {
        if ( (millis() - t0) > (conv_time[creg[ch].srate]) )
          return R_TIMEOUT;
        err = nb_read(ch, value);
    } while (err == R_IN_PROGRESS);

    return err;

}

ConvStatus MCP3424::nb_read(Channel ch, double & value) {

    int32_t lval;
    uint8_t b2, b3, b4;

    if (cwrite.reg != creg[ch].reg)
      if (creg[ch].cmode == CONTINUOUS)
        writeConfReg(ch);
      else
        startNewConversion(ch);

    Wire.requestFrom(addr, (uint8_t)((cwrite.srate == SR18B)?4:3));

    uint8_t bytes = Wire.available();

    if (creg[ch].srate == SR18B && bytes < ((cwrite.srate == SR18B)?4:3))
      return R_I2C;

    b2 = Wire.read();
    b3 = Wire.read();

    if (creg[ch].srate == SR18B)
      b4 = Wire.read();

    cread.reg = Wire.read();

    Wire.endTransmission();

    if (cread.rdy == 1)
      return R_IN_PROGRESS;

    if (cread.srate == SR18B) {
        lval = (long)(int)(char)b2 << 16;
        lval |= (uint16_t)b3 << 8 | b4;
    } else {
        lval = (b2 << 8) | b3;
    }

    ConvStatus err = R_OK;
    switch (cread.srate) {
      case SR18B: if (lval ==  131071L) err = R_OVERFLOW;
                    if (lval == -131072L) err = R_UNDERFLOW;
                  break;
      case SR16B: if (lval ==  32767L) err = R_OVERFLOW;
                    if (lval == -32768L) err = R_UNDERFLOW;
                  break;
      case SR14B: if (lval ==  8191L) err = R_OVERFLOW;
                    if (lval == -8192L) err = R_UNDERFLOW;
                  break;
      case SR12B: if (lval ==  2047L) err = R_OVERFLOW;
                    if (lval == -2048L) err = R_UNDERFLOW;
                  break;
      default: break;
    };

    value = 0.001 * lval / (1 << (cread.srate << 1)) / ( 1 << cread.pga);

    return err;
}

