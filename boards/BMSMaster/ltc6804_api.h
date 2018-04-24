#include "crc15.c"
#include "BMSMaster.h"
#include "LTC_defs.h"
#include <stdlib.h>
#include "spi_api.h"


const uint16_t OV_THRESHOLD = 36000;//35900; // Over voltage threshold ADC Code. LSB = 0.0001
const uint16_t SOFT_OV_THRESHOLD = 36200;//35500; //Soft over-voltage for discharge
const uint16_t UV_THRESHOLD = 22000; //20000;// Under voltage threshold ADC Code. LSB = 0.0001

//Thermistor voltage times this number must be greater than measured bus voltage (times 1000 for AVR)
const uint16_t THERM_V_FRACTION = 3807; //Maximum fraction is 99/26 = 3.807 (1% top thermistors, therm 26K at 58 C)


const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See ltc6811_daisy.h for Options


void init_cfg(void);
void wakeup_idle(uint8_t total_ic);
void wakeup_sleep(uint8_t total_ic);
void o_ltc6811_rdcfg(uint8_t total_ic, // Number of ICs in the system
        uint8_t r_config[][8] // A two dimensional array that the function stores the read configuration data.
    );
void o_ltc6811_wrcfg(uint8_t total_ic, // Number of ICs in the system
        uint8_t r_config[][6] // A two dimensional array of the configuration data that will be written
    );
