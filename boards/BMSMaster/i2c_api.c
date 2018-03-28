/*
Header:
    Describe what it does.
Author:
    @author Peter Seger & Alexander Hoppe
*/


#include "i2c_api.h"





/*----- Write to I2C -----*/
void write_i2c(uint8_t total_ic, uint8_t address, uint8_t command, uint8_t *data, uint8_t data_len) {
    uint8_t START = 0x60;
    uint8_t NACK = 0x08;
    uint8_t BLANK = 0x00;
    uint8_t NO_TRANSMIT = 0x70;
    uint8_t NACK_STOP = 0x09;

    uint8_t loop_count;
    uint8_t remainder = 0;
    uint8_t transmitted_bytes = 0;
    uint8_t data_counter = 0;
    uint8_t comm[total_ic][6];

    if (((data_len) % 3) == 0) {
        loop_count = ((data_len)/3);
    } else {
        loop_count = ((data_len)/3);
        remainder = data_len - (loop_count * 3);
        loop_count++;
    }

    address = address << 1;     // Conver 7 bit address into 8 bits

    for (uint8_t i = 0; i < total_ic; i++) {
        comm[i][0] = START;
        comm[i][1] = NACK_STOP;
        comm[i][2] = START | (address >> 4);
        comm[i][3] = (address << 4) | NACK;
        comm[i][4] = BLANK | (command >> 4);
        comm[i][5] = (command << 4) | NACK;
    }

    // If there is no data, free up bus
    if (loop_count == 0) {
        for (uint8_t i = 0; i < total_ic; i++) {
            comm[i][5] = (command << 4) | NACK_STOP;
        }
    }

    wrcomm(total_ic, comm);
    stcomm();

    transmitted_bytes = 0;
    for (uint8_t i = 0; i < loop_count; i++) {
        if ((i == (loop_count - 1)) && (remainder != 0)) {
            for (uint8_t k = 0; k < remainder; k++) {
                comm[0][transmitted_bytes] = BLANK + (data[data_counter] >> 4);
                if (k != (remainder - 1)) {
                    comm[0][transmitted_bytes + 1] = (data[data_counter] << 4) | NACK;
                } else {
                    comm[0][transmitted_bytes + 1] = (data[data_counter] << 4) | NACK_STOP;
                }
                data_counter++;
                transmitted_bytes = transmitted_bytes + 2;
            }
            for (uint8_t k = 0; k < (3 - remainder); k++) {
                comm[0][transmitted_bytes] = NO_TRANSMIT;
                comm[0][transmitted_bytes + 1] = BLANK;
                transmitted_bytes = transmitted_bytes + 2;
            }
            for (uint8_t i = 1; i < total_ic; i++) {
                for (uint8_t j = 0; j < 6; j++) {
                    comm[i][j] = comm[0][j];
                }
            }
            wrcomm(1, comm);
            stcomm();
        } else {
            for (uint8_t k = 0; k < 3; k++) {
                comm[0][k * 2] = BLANK + (data[data_counter] >> 4);
                if (k != 2) {
                    comm[0][(k * 2) + 1] = (data[data_counter] << 4) | NACK;
                } else if (remainder != 0) {
                    comm[0][(k * 2) + 1] = (data[data_counter] << 4) | NACK;
                } else {
                    comm[0][(k * 2) + 1] = (data[data_counter] << 4) | NACK_STOP;
                }
                data_counter++;
            }
            for (uint8_t i = 1; i < total_ic; i++) {
                for (uint8_t j = 0; j < 6; j++) {
                    comm[i][j] = comm[0][j];
                }
            }
            wrcomm(1, comm);
            stcomm();
        }
    }
}

void wrcomm(uint8_t total_ic, //The number of ICs being written to
        uint8_t comm[][6] //A two dimesional array of the comm data that will be written
    )
{
    /* This function writes the COMM registers of a ltc6811 daisy chain */
    const uint8_t BYTES_IN_REG = 6;
    const uint8_t CMD_LEN = 4 + (8 * total_ic);
    uint8_t *cmd;
    uint16_t comm_pec;
    uint16_t cmd_pec;
    uint8_t cmd_index;  // Command counter

    cmd = (uint8_t *)malloc(CMD_LEN * sizeof(uint8_t));

    cmd[0] = 0x07;
    cmd[1] = 0x21;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    cmd_index = 4;
    // Iterate over each ltc6811 in daisy chain. Starts on last IC on the stack writing the first configuration
    for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--) {
        // Executes for each of the 6 bytes in the CFGR register
        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) {
            cmd[cmd_index] = comm[current_ic - 1][current_byte];
            cmd_index = cmd_index + 1;
        }
        comm_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &comm[current_ic - 1][0]);
        cmd[cmd_index] = (uint8_t)(comm_pec >> 8);
        cmd[cmd_index + 1] = (uint8_t)comm_pec;
        cmd_index = cmd_index + 2;
    }

    PORTB &= ~_BV(PB4);     // Set CS low
    spi_write_array(cmd, CMD_LEN);
    PORTB |= _BV(PB4);      // Set CS high
    free(cmd);
}

void stcomm(void) {
    /* This function shifts data in COMM register out over ltc6811 SPI/I2C port */
    uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = 0x07;
    cmd[1] = 0x23;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    PORTB &= ~_BV(PB4);
    spi_write_array(cmd, 4);
    for (int i = 0; i < 9; i++) {
        spi_message(0xFF);
    }
    PORTB |= _BV(PB4);
}

int8_t rdcomm(uint8_t total_ic, //Number of ICs in the system
        uint8_t r_comm[][8] //A two dimensional array that function stores the read configuration data
    )
{
    /* This function reads COMM registers of a ltc6811 daisy chain */
    const uint8_t BYTES_IN_REG = 8;

    uint8_t cmd[4];
    uint8_t *rx_data;
    int8_t pec_error = 0;
    uint16_t cmd_pec;
    uint16_t data_pec;
    uint16_t received_pec;

    rx_data = (unit8_t *)malloc((8 * total_ic) * sizeof(uint8_t));

    cmd[0] = 0x07;
    cmd[1] = 0x22;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    PORTB &= ~_BV(PB4);     // Set CS low
    spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG * total_ic));
    PORTB |= _BV(PB4);      // Set CS high

    // Iterate through each ltc6811 in the daisy chain and back its data into r_comm
    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) {
        // Iterate through all the bytes in the register
        for (unit8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) {
            r_comm[current_ic][current_byte] = rx_data[current_byte + (current_ic * BYTES_IN_REG)];
        }
        received_pec = (r_comm[current_ic][6] << 8) + r_comm[current_ic][7];
        data_pec = pec15_calc(6, &r_comm[current_ic][0]);

        // Compare percentages for errors
        if (received_pec != data_pec) {
            pec_error = -1;     // Set negative if any PEC errors
        }
    }
    free(rx_data);
    return(pec_error);
}
