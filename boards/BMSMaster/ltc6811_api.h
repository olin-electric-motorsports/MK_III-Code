#ifndef LTC6811_API_H
#define LTC6811_API_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "BMSMaster.h"
#include "m16m1_spi_api.h"
#include "LTC_defs.h"
#include "crc15.h"





void init_cfg(void);
void wakeup_idle(uint8_t total_ic);
void wakeup_sleep(uint8_t total_ic);
void o_ltc6811_rdcfg(uint8_t total_ic, // Number of ICs in the system
        uint8_t r_config[][8] // A two dimensional array that the function stores the read configuration data.
    );
void o_ltc6811_wrcfg(uint8_t total_ic, // Number of ICs in the system
        uint8_t r_config[][6] // A two dimensional array of the configuration data that will be written
    );
uint32_t o_ltc6811_pollAdc(void);
void o_ltc6811_adcv(uint8_t MD, //ADC Mode
        uint8_t DCP, //Discharge Permit
        uint8_t CH //Cell channels to be measured
    );
uint8_t o_ltc6811_rdcv(uint8_t reg, // Controls which cell voltage regulator is read back,
        uint8_t total_ic,   // Number of ICs in the system
        uint16_t cell_codes[][CELL_CHANNELS] // Array of the parsed cell codes
    );
void o_ltc6811_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
            uint8_t total_ic, //Number of ICs in the array
            uint8_t *data //An array of the unparsed cell codes
    );
void o_ltc6811_adax(
        uint8_t MD, //ADC Mode
        uint8_t CHG //GPIO Channels to be measured)
    );
int8_t o_ltc6811_rdaux(uint8_t reg, //Determines with GPIO voltage register is read back
        uint8_t total_ic, //The number of ICs in the system
        uint16_t aux_codes[][AUX_CHANNELS] //A two dimensional array of the GPIO voltage codes
    );
void o_ltc6811_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
        uint8_t total_ic, //The number of ICs in the system
        uint8_t *data //Array of the unparsed auxiliary codes
    );

#endif
