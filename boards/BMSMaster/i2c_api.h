#include "crc15.c"
#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>




void write_i2c(uint8_t total_ic, uint8_t address, uint8_t command, uint8_t *data, uint8_t data_len);
void wrcomm(uint8_t total_ic, //The number of ICs being written to
        uint8_t comm[][6] //A two dimesional array of the comm data that will be written
    );
void stcomm(void);
int8_t rdcomm(uint8_t total_ic, //Number of ICs in the system
        uint8_t r_comm[][8] //A two dimensional array that function stores the read configuration data
    );
