#ifndef CRC15_H
#define CRC15_H

#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>



uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
        uint8_t *data //Array of data that will be used to calculate  a PEC
    );

extern const uint16_t crc15Table[256] PROGMEM;

#endif
