/*
 *  Copyright (C) 2014 Bernhard Schneider <bernhard@neaptide.org>
 *   
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public License
 *  version 3 as published by the Free Software Foundation.
 * 
 */

#ifndef __MCP3424_h__
#define __MCP3424_h__

extern "C" {
#include <stdlib.h>
#include <inttypes.h>
}

#include <Wire.h>

typedef enum { ONE_SHOT, CONTINUOUS } ConvType;
typedef enum { CH1, CH2, CH3, CH4 } Channel;
typedef enum { R12B, R14B, R16B, R18B } Resolution; 
typedef enum { GAINx1, GAINx2, GAINx4, GAINx8 } Gain;
typedef enum { PIN_LOW, PIN_HIGH, PIN_FLOAT } PinType;
typedef enum { R_OK, R_UNDERFLOW, R_OVERFLOW, R_I2C, R_IN_PROGRESS, R_TIMEOUT } ConvStatus;
typedef enum { GC_LATCH=0x04, GC_RESET=0x06, GC_CONVERSION=0x08 } GCall_t;

//const uint16_t  conv_time[]  = {1000/176, 1000/44, 1000/11, 100000/275};
const uint16_t  conv_time[]  = { 30, 40, 90, 290}; // FIXME: have to be hw indep.
const uint8_t pin_addr[] = { 
    0b1101000, 0b1101010, 0b1101001,
    0b1101100, 0b1101110, 0b1101101,
    0b1101011, 0b1101111, 0b1101000 };

typedef struct  {
    union {
        uint8_t reg;
        struct __attribute__ ((__packed__)) {
            Gain       pga   :2;
            Resolution res   :2;
            ConvType   cmode :1;
            Channel    ch    :2;
            uint8_t    rdy   :1;
        };
        struct __attribute__ ((__packed__)) {
            Gain       pga   :2;
            Resolution res   :2;
            ConvType   cmode :1;
            Channel    ch    :2;
            uint8_t    rdy   :1;
        } bits;
    };
} _ConfReg;

class MCP3424 {

  private:
            uint8_t   addr;
            _ConfReg  cread, cwrite;

            uint8_t writeConfReg(Channel);
            ConvStatus nb_read(Channel, double&);

  public:
            _ConfReg  creg[4];

            MCP3424 (uint8_t);

            MCP3424 (PinType, PinType);

            uint8_t generalCall(GCall_t) const;

            uint8_t startNewConversion(Channel);

            Gain findGain(double value) const;

            ConvStatus read(Channel ch, double& value, bool blocking=true);

};
#endif
