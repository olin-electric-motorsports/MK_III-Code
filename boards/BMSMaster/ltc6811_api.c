/*
Header:
    LTC68XX communication protocol abstraction library:
        Communicating with LTC68XX devices on an isoSPI daisy chain requires an
        ongoing SPI transaction between the MCU and the LTC6820. This interface
        allows the MCU to set config registers on each of the the LTC68XX
        devices. The LTC68XX devices then operate on those config registers,
        either measuring voltage, measuring an ADC pin, or accessing GPIO.
Author:
    @author Peter Seger & Alexander Hoppe
*/

#include "ltc6811_api.h"
#include "BMSMaster.h"





/*----- LTC68XX Communication -----*/
void init_cfg(void) {
    /* Initialize the configuration array */
    uint16_t undervoltage_val = (UV_THRESHOLD/16)-1;
    uint16_t overvoltage_val  =(OV_THRESHOLD/16);
    for (int i = 0; i < TOTAL_IC; i++) {
        tx_cfg[i][0] = 0xFC | ADC_OPT;
        tx_cfg[i][1] = (uint8_t)(undervoltage_val & 0xFF);
        tx_cfg[i][2] = (uint8_t)((overvoltage_val & 0x00F) | ((undervoltage_val * 0xF00) >> 8));
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

    rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));

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
        uint8_t config[][6] // A two dimensional array of the configuration data that will be written
    )
{
    /* This command will write the configuration registers in
        descending order so that the last devices Configuration
        is written first.
    */
    const uint8_t BYTES_IN_REG = 6;
    const uint8_t CMD_LEN = 4 + (8*total_ic);
    uint8_t *cmd;
    uint16_t cfg_pec;
    uint8_t cmd_index;  //command counter

    cmd = (uint8_t *) malloc(CMD_LEN*sizeof(uint8_t));

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
        cfg_pec = (uint16_t)pec15_calc(BYTES_IN_REG, & config[current_ic-1][0]); // Calculate the PEC for each IC's configuration register data
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

uint32_t o_ltc6811_pollAdc(void) {
    /* This function will block operation until the ADC has finished its conversion */
    uint32_t counter = 0;
    uint8_t finished = 0;
    uint8_t current_time = 0;
    uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = 0x07;
    cmd[1] = 0x14;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    PORTB &= ~_BV(PB4);     // Set CS low
    spi_write_array(cmd, 4);

    // Delay timer
    while((counter < 200000) && (finished == 0)) {
        current_time = spi_message(0xFF);
        if (current_time > 0) {
            finished = 1;
        } else {
            counter = counter + 10;
        }
    }

    PORTB |= _BV(PB4);      // Set CS high

    return counter;
}

void o_ltc6811_adcv(uint8_t MD, //ADC Mode
        uint8_t DCP, //Discharge Permit
        uint8_t CH //Cell channels to be measured
    )
{
    /* This function starts cell voltage conversion */
    uint8_t cmd[4];
    uint16_t cmd_pec;
    uint8_t md_bits;

    md_bits = (MD & 0x02) >> 1;
    cmd[0] = md_bits + 0x02;
    md_bits = (MD & 0x01) << 7;
    cmd[1] = md_bits + 0x60 + (DCP << 4) + CH;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    PORTB &= ~_BV(PB4);     // Set CS low
    spi_write_array(cmd, 4);
    PORTB |= _BV(PB4);      // Set CS high
}

uint8_t o_ltc6811_rdcv(uint8_t reg, // Controls which cell voltage regulator is read back,
        uint8_t total_ic,   // Number of ICs in the system
        uint16_t cell_voltages[][CELL_CHANNELS] // Array of the parsed cell codes
    )
{
    /* This function reads and parses the ltc6811 cell voltage registers */
    const uint8_t NUM_RX_BYT = 8;
    const uint8_t BYTES_IN_REG = 6;
    const uint8_t CELL_IN_REG = 3;
    const uint8_t NUM_CV_REG = 4;

    uint8_t *cell_data;
    uint8_t pec_error = 0;
    uint16_t parsed_cell;
    uint16_t received_pec;
    uint16_t data_pec;
    uint8_t data_counter = 0;   // Data counter used to calc avg.
    cell_data = (uint8_t *) malloc((NUM_RX_BYT * total_ic)*sizeof(uint8_t));

    if (reg == 0) {
        // Iterate through all ltc6811 voltage registers
        for (uint8_t cell_reg = 1; cell_reg < NUM_CV_REG+1; cell_reg++) {
            data_counter = 0;

            sprintf(uart_buffer, "Reading single voltage register");
            LOG_println(uart_buffer, strlen(uart_buffer));

            o_ltc6811_rdcv_reg(cell_reg, total_ic, cell_data);  // Reads a single cell voltage register
            // Interate through all ltc6811 in the daisy chain
            for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) {
                // Parse read-back data once for each of the 3 voltage codes in register
                for (uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++) {
                    parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);
                    cell_voltages[current_ic][current_cell + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
                    data_counter = data_counter + 2;  // Increment by 2 since 2 cells codes were read for 1 cell
                }

                received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter + 1];
                data_pec = pec15_calc(BYTES_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);

                // Compare percentages for errors
                if (received_pec != data_pec) {
                    pec_error = -1;     // Set negative if any PEC errors
                }
                data_counter = data_counter + 2;    // Increment by 2 since the trasmitted PEC code is 2 bytes long
            }
        }
    } else {
        o_ltc6811_rdcv_reg(reg, total_ic, cell_data);
        // Iterate over every ltc6811 in the daisy chain
        for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) {
            // Parse read-back data once for each of the 3 voltage codes in register
            for (uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++) {
                parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);
                cell_voltages[current_ic][current_cell + ((reg -1) * CELL_IN_REG)] = 0x0000FFFF & parsed_cell;
                data_counter = data_counter + 2; // Increment by 2 since voltage codes are 2 bytes
            }

            received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter + 1];
            data_pec = pec15_calc(BYTES_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);

            // Compare percentages for errors
            if (received_pec != data_pec) {
                pec_error = -1;     // Set negative if any PEC errors
            }
            data_counter = data_counter + 2;
        }
    }
    free(cell_data);
    return(pec_error);
}


void o_ltc6811_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
            uint8_t total_ic, //Number of ICs in the array
            uint8_t *data //An array of the unparsed cell codes
    )
{
    /* Read the raw data from the ltc6811 cell voltage register */
    const uint8_t REG_LEN = 8;  // Number of bytes in each ICs register + 2 bytes for the PEC
    uint8_t cmd[4];
    uint16_t cmd_pec;

    if (reg == 1) {     //1: RDCVA
        cmd[0] = 0x00;
        cmd[1] = 0x04;
    } else if (reg == 2) {      //2: RDCVB
        cmd[0] = 0x00;
        cmd[1] = 0x06;
    } else if (reg ==3) {       //3: RDCVC
        cmd[0] = 0x00;
        cmd[1] = 0x08;
    } else if (reg == 4) {      //4: RDCVD
        cmd[0] = 0x00;
        cmd[1] = 0x0A;
    } else if (reg == 5) {      //5: RDCVE
        cmd[0] = 0x00;
        cmd[1] = 0x09;
    } else if (reg == 6) {      //6: RDCVF
        cmd[0] = 0x00;
        cmd[1] = 0x0B;
    }

    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    PORTB &= ~_BV(PB4);     // Set CS low
    spi_write_read(cmd, 4, data, (REG_LEN * total_ic));
    PORTB |= _BV(PB4);      // Set CS high
}

//Start a GPIO and Vref2 Conversion
void o_ltc6811_adax(
        uint8_t MD, //ADC Mode
        uint8_t CHG //GPIO Channels to be measured)
    )
{
    uint8_t cmd[4];
    uint16_t cmd_pec;
    uint8_t md_bits;

    md_bits = (MD & 0x02) >> 1;
    cmd[0] = md_bits + 0x04;
    md_bits = (MD & 0x01) << 7;
    cmd[1] = md_bits + 0x60 + CHG ;

    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
    PORTB &= ~_BV(PB4); //set CS low
    spi_write_array(cmd,4);
    PORTB |= _BV(PB4); //set CS low

}

int8_t o_ltc6811_rdaux(uint8_t reg, //Determines with GPIO voltage register is read back
        uint8_t total_ic, //The number of ICs in the system
        uint16_t aux_codes[][AUX_CHANNELS] //A two dimensional array of the GPIO voltage codes
    )
{
    /* This function is used to read the parsed GPIO codes of the ltc6811.
        This function will send the requested read commands, parse the data,
        and store the GPIO voltage in aux_codes variable. */
    const uint8_t NUM_RX_BYT = 8;
    const uint8_t BYTES_IN_REG = 6;
    const uint8_t GPIO_IN_REG = 3;
    const uint8_t NUM_GPIO_REG = 2;
    uint8_t *data;
    uint8_t data_counter = 0;
    int8_t pec_error = 0;
    uint16_t parsed_aux;
    uint16_t received_pec;
    uint16_t data_pec;
    data = (uint8_t *) malloc((NUM_RX_BYT * total_ic) * sizeof(uint8_t));

    if (reg == 0) {
        // Iterate through each of the ltc6811 aux voltage registers
        for (uint8_t gpio_reg = 1; gpio_reg < NUM_GPIO_REG + 1; gpio_reg++) {
            data_counter = 0;
            o_ltc6811_rdaux_reg(gpio_reg, total_ic, data);
            // Iterate through each ltc6811 in the daisy chain
            for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) {
                // Parse the read back data into GPIO voltage
                for (uint8_t current_gpio = 0; current_gpio < GPIO_IN_REG; current_gpio++) {
                    parsed_aux = data[data_counter] + (data[data_counter + 1] << 8);
                    aux_codes[current_ic][current_gpio + ((gpio_reg - 1) * GPIO_IN_REG)] = parsed_aux;
                    data_counter = data_counter + 2; // Increase counter by 2 since each data is 2 bytes long
                }

                received_pec = (data[data_counter] << 8) + data[data_counter + 1];
                data_pec = pec15_calc(BYTES_IN_REG, &data[current_ic * NUM_RX_BYT]);

                // Compare percentages for errors
                if (received_pec != data_pec) {
                    pec_error = -1;     // Set negative if any PEC error
                }
                data_counter = data_counter + 2;
            }
        }
    } else {
        o_ltc6811_rdaux_reg(reg, total_ic, data);
        // Iterate through every ltc6811 in the daisy chain
        for (int current_ic = 0; current_ic < total_ic; current_ic++) {
            // Parse the read back data for each aux voltage in the register
            for (int current_gpio = 0; current_gpio < GPIO_IN_REG; current_gpio++) {
                parsed_aux = (data[data_counter] + (data[data_counter + 1] << 8));
                aux_codes[current_ic][current_gpio + ((reg - 1) * GPIO_IN_REG)] = parsed_aux;
                data_counter = data_counter + 2;
            }

            received_pec = (data[data_counter] << 8) + data[data_counter + 1];
            data_pec = pec15_calc(BYTES_IN_REG, &data[current_ic * NUM_RX_BYT]);

            // Compare percentages for errors
            if (received_pec != data_pec) {
                pec_error = -1;     // Set negative if any PEC errors
            }
            data_counter = data_counter + 2;
        }
    }
    free(data);
    return(pec_error);
}

void o_ltc6811_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
        uint8_t total_ic, //The number of ICs in the system
        uint8_t *data //Array of the unparsed auxiliary codes
    )
{
    /* This function reads a single GPIO voltage register and stores
        the data in the *data point as a byte array. */
    const uint8_t REG_LEN = 8;  // Number of bytes in the register + 2 bytes for the PEC
    uint8_t cmd[4];
    uint16_t cmd_pec;

    if (reg == 1) {     // Read back auxiliary group A
        cmd[0] = 0x00;
        cmd[1] = 0x0C;
    } else if (reg == 2) {       // Read back auxiliary group B
        cmd[0] = 0x00;
        cmd[1] = 0x0e;
    } else if (reg == 3) {      // Read back auxiliary group B
        cmd[0] = 0x00;
        cmd[1] = 0x0D;
    } else if (reg == 4) {      // Read back auxiliary group B
        cmd[0] = 0x00;
        cmd[1] = 0x0F;
    } else {             // Read back auxiliary group A
        cmd[0] = 0x00;
        cmd[1] = 0x0C;
    }
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    PORTB &= ~_BV(PB4);     // Set CS low
    spi_write_read(cmd, 4, data, (REG_LEN * total_ic));
    PORTB |= _BV(PB4);      // Set CS high
}
