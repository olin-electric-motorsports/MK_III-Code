#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "crc15.c"



uint32_t o_ltc6811_pollAdc(void);
void o_ltc6811_adcv(uint8_t MD, //ADC Mode
        uint8_t DCP, //Discharge Permit
        uint8_t CH //Cell channels to be measured
    );
uint8_t o_ltc6811_rdcv(uint8_t reg, // Controls which cell voltage regulator is read back,
        uint8_t total_ic,   // Number of ICs in the system
        uint8_t cell_codes[][CELL_CHANNELS] // Array of the parsed cell codes
    );
void o_ltc6811_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
            uint8_t total_ic, //Number of ICs in the array
            uint8_t *data //An array of the unparsed cell codes
    );
int8_t o_ltc6811_rdaux(uint8_t reg, //Determines with GPIO voltage register is read back
        uint8_t total_ic, //The number of ICs in the system
        uint16_t aux_codes[][AUX_CHANNELS] //A two dimensional array of the GPIO voltage codes
    );
void o_ltc6811_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
        uint8_t total_ic, //The number of ICs in the system
        uint8_t *data //Array of the unparsed auxiliary codes
    );
