#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "crc15.c"


void init_cfg(void);
void wakeup_idle(uint8_t total_ic);
void wakeup_sleep(uint8_t total_ic);
void o_ltc6811_rdcfg(uint8_t total_ic, // Number of ICs in the system
        uint8_t r_config[][8] // A two dimensional array that the function stores the read configuration data.
    );
void o_ltc6811_wrcfg(uint8_t total_ic, // Number of ICs in the system
        uint8_t r_config[][6] // A two dimensional array of the configuration data that will be written
    );
