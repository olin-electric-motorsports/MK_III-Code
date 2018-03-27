/*
Header:
    Describe what it does.
Author:
    @author Peter Seger & Alexander Hoppe
*/

#include "ltc6804_api.h"





/*----- LTC6804 Communication -----*/
void init_cfg(void) {
    /* Initialize the configuration array */
    uint16_t undervoltage_val = (UV_THRESHOLD/16)-1;
    uint16_t overvoltage_val  =(OV_THRESHOLD/16);
    for (int i = 0; i < TOTAL_IC; i++) {
        tx_cfg[i][0] = 0xFC | ADC_OPT;
        tx_cfg[i][1] = (uint8_t)(undervoltage_val & 0xFF);
        tx_cfg[i][2] = (uint8_t)((overvoltage_val & 0x00F) | (undervoltage_val * 0xF00) >> 8));
        tx_cfg[i][3] = (uint8_t)((overvoltage_val & 0xFF0) >> 4);
        tx_cfg[i][4] = 0x00;
        tx_cfg[i][5] = 0x00;
    }
}

void wakeup_idle(uint8_t total_ic) {
    /* Generic wakeup command to wake isoSPI out of idle */
    for (uint8_t i = 0; i < total_ic; i++) {
        PORTB &= ~_BV(PB4);     // Set CS low
        _delay_us(2);   // Guarentees the isoSPI will be in ready mode
        PORTB |= _BV(PB4);      // Set CS high
    }
}

void wakeup_sleep(uint8_t total_ic) {
    /* Generic wakeup command to wake the ltc6813 from sleep */
    for (int i = 0; i < total_ic; i++) {
        PORTB &= ~_BV(PB4);     // Set CS low
        _delay_us(300);     // Guarentees the ltc6813 will be in standby
        PORTB |= _BV(PB4);      // Set CS high
    }
}

void o_ltc6811_rdcfg(uint8_t total_ic, // Number of ICs in the system
        uint8_t r_config[][8] // A two dimensional array that the function stores the read configuration data.
    )
{
    /* Reads configuration registers of a ltc6811 daisy chain */
    const uint8_t BYTES_IN_REG = 8;

    uint8_t cmd[4];
    uint8_t *rx_data;

    rx_data = (uint8_t *)malloc((8*total_ic)*sizeof(uint8_t));

    cmd[0] = 0x00;
    cmd[1] = 0x02;
    cmd[2] = 0x2b;
    cmd[3] = 0x0A;

    wakeup_idle(total_ic);  // This will guarentee that the ltc6811 isoSPI port is awake.

    PORTB &= ~_BV(PB4);     // Set CS low
    spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG*total_ic));   // Read configuration data
    PORTB |= _BV(PB4);      // Set CS high

    // TODO missing logic potentially???
}

void o_ltc6811_wrcfg(uint8_t total_ic, // Number of ICs in the system
        uint8_t r_config[][6] // A two dimensional array of the configuration data that will be written
    )
{
    /* This command will write the configuration registers in
        descending order so that the last devices Configuration
        is written first.
    */
    const uint8_t BYTES_IN_REG = 6;
    const CMD_LEN = 4 + (8*total_ic);
    uint8_t *cmd;
    uint16_t cfg_pec;
    uint8_t cmd_index;  //command counter

    cmd = (uint8_t *)malloc(CMD*sizeof(uint8_t));

    cmd[0] = 0x00;
    cmd[1] = 0x01;
    cmd[2] = 0x3d;
    cmd[3] = 0x6e;

    cmd_index = 4;

    /* Iterate through each IC starting at the last and writing the first config */
    for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--) {
        // Iterate through each of the 6 bytes in the CFGR register
        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) {
            cmd[cmd_index] = config[current_ic-1][current_byte];
            cmd_index = cmd_index + 1;
        }
        cfg_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]); // Calculate the PEC for each IC's configuration register data
        cmd[cmd_index] = (uint8_t)(cfg_pec >> 8);
        cmd[cmd_index + 1] = (uint8_t)(cfg_pec);
        cmd_index = cmd_index + 2;
    }

    wakeup_idle(total_ic);  // Guarentee that the ltc6811 isoSPI port is awake

    PORTB &= ~_BV(PB4);      // Set CS low
    spi_write_array(cmd, CMD_LEN);
    PORTB |= _BV(PB4);      // Set CS high
    free(cmd);
}
