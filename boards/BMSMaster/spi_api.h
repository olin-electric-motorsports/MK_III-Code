#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>



void init_spi(void);
uint8_t spi_message(uint8_t msg);
uint8_t spi_write_array(uint8_t tx_data[], uint8_t tx_len);
void spi_write_read(uint8_t tx_data[], // Array of data to be written on SPI port
        uint8_t tx_len,  // Length of the tx data array
        uint8_t *rx_data,  // Input: an array that will store the data read by the SPI port
        uint8_t rx_len  // Option: number of bytes to be read from the SPI port
    );
