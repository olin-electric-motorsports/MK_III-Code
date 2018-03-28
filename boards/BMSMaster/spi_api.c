/*
Header:
    Describe what it does.
Author:
    @author Peter Seger & Alexander Hoppe
*/

#include "spi_api.h"







/*----- SPI Functions -----*/
void init_spi(void) {
    // Setup SPI I/O pins (MOSI & SCK)
    DDRB |= _BV(PB1) | _BV(PB7);

    // Enable SPI
    SPCR |= _BV(SPE) | _BV(MSTR) | _BV(SPR0);
}

uint8_t spi_message(uint8_t msg) {
    SPDR = msg;     // Set message
    while(!(SPSR & (1 << SPIF)));   // Wait for trasmission to finish
    return SPDR;
}

uint8_t spi_write_array(uint8_t tx_data[], uint8_t tx_len) {
    for (uint8_t i = 0; i < tx_len; i++) {
        spi_message(tx_data[i]);
    }
    return 0;
}

void spi_write_read(uint8_t tx_data[], // Array of data to be written on SPI port
        uint8_t tx_len,  // Length of the tx data array
        uint8_t *rx_data,  // Input: an array that will store the data read by the SPI port
        uint8_t rx_len  // Option: number of bytes to be read from the SPI port
    )
    /* This function writes and reads a set number of bytes using the SPI port */
{
    // Write message(s)
    for (uint8_t i = 0; i < tx_len; i++) {
        spi_message(tx_data[i]);
    }
    // Read message(s)
    for (uint8_t i = 0; i < rx_len; i++) {
        rx_data[i] = (uint8_t)spi_message(0x00);
    }
}
