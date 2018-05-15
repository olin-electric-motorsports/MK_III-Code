/*
Header:
    Calculates 15-bit Packet Error Code to verify data integrity from LTC6804
    daisy-chained devices
Author:
    @author Peter Seger & Alexander Hoppe
*/


#include "crc15.h"




uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
        uint8_t *data //Array of data that will be used to calculate  a PEC
    )
{
    /* This function calculates and returns CRC15 */
    uint16_t rmdr, addr;

    rmdr = 16;//initialize the PEC
    for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
    {
        addr = ((rmdr>>7)^data[i])&0xff;//calculate PEC table address
        rmdr = (rmdr<<8)^pgm_read_word_near(crc15Table+addr);
    }
    return(rmdr*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}
