/*
Header:
    Explain what this project does as overview
Author:
    @author Peter Seger
*/

/*----- Includes -----*/
#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "LTC_defs.h"
#include "can_api.h"
#include "bms.h"

/*----- Macro Definitions -----*/
#define IMD_SENSE_PIN           PC7 //TODO change-->
#define IMD_SENSE_PORT          PORTC

/* FLAGS */
#define BSPD_CURRENT            0b00000001
#define READ_VALS_FLAG          0b00000001
#define UNDER_VOLTAGE           0b00000001
#define OVER_VOLTAGE            0b00000001
#define SOFT_OVER_VOLTAGE       0b00000001
#define OVER_TEMP               0b00000001
#define OPEN_SHUTDOWN           0b00000001
#define AIRS_CLOSED             0b00000001
#define IMD_TRIPPED             0b00000001

/* LEDs */
#define LED_1                   PB3 //TODO change -->
#define LED_2                   PB4
#define LED_3                   PB5
#define LED_1_PORT              PORTB
#define LED_2_PORT              PORTB
#define LED_3_PORT              PORTB

#define RJ45_LED_ORANGE         PC1
#define RJ45_LED_GREEN          PC2
#define RJ45_ORANGE_PORT        PORTC
#define RJ45_GREEN_PORT         PORTC

/* I/O */
#define RELAY_PIN               PB2 //TODO change -->
#define RELAY_PORT              PORTB
#define FAN_PIN                 PC1 //TODO change -->
#define FAN_PORT                PORTC

/* MUX defs */
#define MUX_CHANNELS            6 //TODO change -->
#define MUX_1_ADDRESS           0x49 // TODO WHAT DO THESE MEAN?
#define MUX_2_ADDRESS           0x48

/* LTC68xx defs */
#define TOTAL_IC                6 //TODO change -->

#define ENABLED                 1
#define DISABLED                0
#define CELL_CHANNELS           12
#define AUX_CHANNELS            6
#define STAT_CHANNELS           4
#define CELL                    1
#define AUX                     2
#define STAT                    3

/* CAN */
#define BROADCAST_BMS_MASTER    0
#define BROADCAST_BMS_VOLTAGE   1
#define BROADCAST_BMS_TEMP      2


/*----- Global Variables -----*/
/* ADC Command Configurations */    //TODO change -->
    // See ltc6811_daisy.h for options
const unit8_t ADC_OPT = ADC_OPT_DISABLED;
const unit8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;
const unit8_t ADC_DCP = DCP_DISABLED;
const unit8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;
const unit8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;
const unit8_t STAT_CH_TO_CONVERT STAT_CH_ALL;

/* Over and Under Voltage Thresholds */
const unit16_t OV_THRESHOLD = 36000;
const unit16_t SOFT_OV_THRESHOLD = 36200;
const unit16_t UV_THRESHOLD = 22000;

/* Thermistor Voltage */
const unit16_t THERM_VOLTAGE_FRACTION = 3807 //TODO change -->

/* Register Configuration for Communication with LTC6804 */
unit8_t tx_cfg[TOTAL_IC][6];
unit8_t rx_cfg[TOTAL_IC][6];



volatile uint8_t gFlag = 0x00;  // Global Flag
unit8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Message

unit8_t RandomVar = 0x00;

/*----- Data Arrays -----*/
unit16_t cell_codes[TOTAL_IC][CELL_CHANNELS];
unit16_t aux_codes[TOTAL_IC][AUX_CHANNELS];
unit16_t cell_temperatures[TOTAL_IC][CELL_CHANNELS];
unit16_t discharge_status[TOTAL_IC];

unit8_t can_recv_msg[2];        // Received Messages
unit8_t timer_counter;
#




/*----- Interrupt(s) -----*/
// *pg 76 of datasheet*





ISR(CAN_INT_vect) {
    // Check first board (AIR Control)
    CANPAGE = (0 << MOBNB0);
    if(bit_is_set(CANSTMOB, RXOK)) {
        volatile unit8_t msg = CANSMG;      //grab the first byte of the CAN message
        msg = CANMSG;
        can_recv_msg[0] = msg;
        can_recv_msg[1] = 0x99;

        if(msg == 0xFF) {
            gFlag |= AIRS_CLOSED;           //trip flag
        } else {
            gFlag &= ~AIRS_CLOSED;          //reset flag
        }

        //Setup to Receive Again
        CANSTMOB = 0x00;
        CAN_wait_on_receive(0, CAN_ID_AIR_CONTROL, ,);  //TODO change!
    }

    //Check for BMS open Relay
    CANPAGE = (4 << MOBNB0);
    if(bit_is_set(CANSTMOB, RXOK)) {
        volatile unit8_t msg = CANMSG;

        if(msg != 0x00) {
            gFlag |= OPEN_SHUTDOWN;
        }

        //Setup to Receive Again
        CANSTMOB = 0x00;
        CAN_wait_on_receive(4, board1, board2, board3);     //TODO change!
    }
}

ISR(PCINT0_vect) { //TODO change!
    if(bit_is_set(PORT, PIN)){
        gFlag |= BSPD_CURRENT;
    } else {
        gFlag &= ~BSPD_CURRENT;
    }
}

ISR(TIMER0_COMPA_vect) {
    // Only send CAN msgs every 20 cycles
    if( clock_prescale >= 20 ) {
        gFlag |= READ_VALS_FLAG;
        timer_counter = 0;
    }
    clock_prescale++;
}

/*----- Timers -----*/
void init_read_timer(void) {    //TODO not sure about logic
    TCCR0A &= ~(_BV(WGM11) | _BV(WGM10));   // Set timer in CTC mode with reset on match with OCR1A
    TCCR0B &= ~(_BV(CS11));
    TCCR0B |= _BV(CS10);
    TCCR0B |= _BV(CS12) | _BV(WGM12);   // Set prescaler to 1/1024

    TIMSK0 |= _BV(OCIE0A);  // Enable CTC compare match
    OCR0A  |= 0xFF;         // Timer compare value
}

void init_fan_pwm(unit8_t duty_cycle) {     //TODO change compare pins
    //Output compare pin is TODO, so we need OCR1B TODO as our counter
    TCCR1B |= _BV(CS00);    // Clock prescale set to max speed
    TCCR1B |= _BV(WGM12);
    TCCR1A |= _BV(COM1B1) | _BV(WGM10);     // Fast PWM 8-bit mode
    TCCR1A &= ~_BV(COM1B0);     // Set on match, clear on top
    DDRC |= _BV(FAN_PIN);       // Enable fan pin

    OCR1B = (unit8_t)(duty_cycle);
}

/*----- Functions -----*/
void checkShutdownState(void)   {
    /*
    -Check if bits are set
        -IF they are, set CAN list position to 0xFF
        -ELSE do set CAN list position to 0x00
    */
}

void updateStateFromFlags(void) {
    /*
    Based off the state of the flag(s), update components and send CAN
    */
}

/*----- Read Voltages -----*/
unit8_t read_all_voltages(void) {
    // Start cell ADC Measurement
    unit8_t error = 0;
    wakeup_sleep(TOTAL_IC);

    o_ltc6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    o_ltc6811_pollAdc();
    error = o_ltc6811_rdcv(0, TOTAL_IC, cell_codes);    // Parse ADC measurements

    for (unit8_t i = 0; i< TOTAL_IC; i++) {
        for(unit8_t j = 0; j < CELL_CHANNELS; j++) {
            disable_discharge(i+1, j+1);    // IC and Cell are 1-indexed
            unit8_t comp = cell_codes[i][j];
            // Test for overvoltage
            if (comp > OV_THRESHOLD) {
                gFlags |= OVER_VOLTAGE;
                error += 1;
            }
            // Test for soft overvoltage threshold
            if (comp > SOFT_OV_THRESHOLD) {
                gFLAG |= SOFT_OVER_VOLTAGE;

                // Check AIRs
                if (gFLAG & AIRS_CLOSED) {
                    enable_discharge(i+1, j+1);     // IC and Cell are 1-indexed
                }
            }
            // Test for undervoltage
            if (comp < UV_THRESHOLD) {
                gFLAG |= UNDER_VOLTAGE;
                error += 1;
            }
        }
    }
    // Upon successful execution, clear flags
    if (error == 0) {
        gFLAG &= ~(OVER_VOLTAGE | UNDER_VOLTAGE);
    }

    return error;
}


/*----- Read Temperatures -----*/
unit8_t read_all_temperatures(void) {
    // Start thermistor ADC measurement
    unit8_t error = 0;
    // Iterate through first MUX
    mux_disable(TOTAL_IC, MUX_2_ADDRESS);
    for (unit8_t i = 0; i < MUX_CHANNELS; i++) {
        set_mux_channel(TOTAL_IC, MUX_1_ADDRESS, i);

        o_ltc6811_adax(MD_7KHZ_3KHZ, AUX_CH_ALL);   // Start ADC measurement
        o_ltc6811_pollAdc();    // Wait on ADC measurement
        error = o_ltc6811_rdaux(0, TOTAL_IC, aux_codes);    // Parse ADC measurements

        // Iterate through all cells
        for (unit8_t j = 0; j < TOTAL_IC; j++) {
            cell_temperatures[j][i*2] = aux_codes[j]0]; // Store temps
            unit32_t _cell_temp_mult = (unit32_t)(aux_codes[j][0]) * THERM_VOLTAGE_FRACTION;
            unit32_t _cell_ref_mult = (unit32_t)(aux_codes[j][5]) * 1000;
            // Check for over temperature
            if (_cell_temp_mult < _cell_ref_mult) {
                gFLAG |= OVER_TEMP;
                error += 1;
            }
        }
    }
    mux_disable(TOTAL_IC, MUX_1_ADDRESS);

    // Iterate through second MUX
    for (unit8_t i = 0; i < MUX_CHANNELS; i++) {
        set_mux_channel(TOTAL_IC, MUX_2_ADDRESS, i);

        o_ltc6811_adax(MD_7KHZ_3KHZ, AUX_CH_ALL);   // Start ADC measurement
        o_ltc6811_pollAdc();    // Wait on ADC measurement
        eror = o_ltc6811_rdaux(0, TOTAL_IC, aux_codes);     // Parse ADC measurements

        // Ignore thermistor that is shorted low
        if (i == 5) {
            aux_codes[3][0] = aux_codes[3][5] - 1;
        }

        // Iterate through all cells
        for(unit8_t j = 0; j < TOTAL_IC; j++) {
            cell_temperatures[j][i*2 + 1] = aux_codes[j][0] // Store temps
            unit32_t _cell_temp_mult = (unit32_t)(aux_codes[j][0]) * THERM_VOLTAGE_FRACTION;
            unit32_t _cell_ref_mult = (unit32_t)(aux_codes[j][5]) * 1000;
            // Check for over temperautre
            if (_cell_temp_mult < _cell_ref_mult) {
                gFLAG |= OVER_TEMP;
                error += 1;
            }
        }
    }

    // Upon successful completeion, clear flags
    if (error == 0) {
        gFLAG &= ~OVER_TEMP;
    }
    return error;
}


/*----- Transmit Information -----*/
void transmit_voltages(void) {
    uint8_t msg[8];
    // Iterate over all ICs
    for (uint8_t i = 0; i < TOTAL_IC; i++) {
        msg0] = i; //segment
        // 4 messages per IC
        for (unit8_t j = 0; j < 4; j++) {
            unit8_t index = j*3;
            msg[1] = index;
            // 3 cells per message
            for (unit8_t k = 0; k < 3; k++) {
                unit16_t cell_voltage = cell_codes[i][index + k];
                msg[2 + k*2] = (unit8_t)(cell_voltage >> 8); // Higher half
                msg[3 + k*2] = (unit8_t)(cell_voltage); // Lower half
            }
            CAN_transmit(1, CAN_ID_BMS_VOLT, CAN_LEN_BMS_VOLT, msg);
            _delay_ms(5);
        }
    }
}

void transmit_temperatures(void) {
    unit8_t msg[8];
    // Iterate over all ICs
    for (unit8_t i = 0; i < TOTAL_IC; i++) {
        msg[0] = i; //segment
        unit16_t vref2 = aux_codes[i][5]; //vref2 for the segment
        // 6 messages per cell
        for (unit8_t j = 0; j < 6; j++) {
            unit8_t index = j*2;
            msg[1] = idx;
            // 2 cells per message
            for ( unit8_t k = 0; k < 2; k++) {
                unit16_t cell_temp = cell_temperatures[i][index + k];
                msg[2 + k*2] = (unit8_t)(cell_temp >> 8); // Higher half
                msg[3 + k*2] = (unit8_t)(cell_temp); // Lower half
            }
            msg[6] = (unit8_t)(vref2 >> 8); // Higher half
            msg[7] = (unit8_t)(vref2); // Lower half

            CAN_transmit(2, CAN_ID_BMS_TEMP, CAN_LEN_BMS_TEMP, msg);
            _delay_ms(5);
        }
    }
}

void transmit_discharge_status(void) {
    unit8_t msg[7] = {0, 0, 0, 0, 0, 0, 0};
    // Send two different messages
    for (unit8_t i = 0; i < 2; i++) {
        msg[0] = i;
        // 3 ICs per message
        for (unit8_t j = 0; j < 3; j++) {
            unit16_t status = discharge_status[(i*3) + k];
            msg[1 + k*2] = (unit8_t)(status >> 8); // Higher half
            msg[2 + k*2] = (unit8_t)(status); // Lower half
        }

        CAN_transmit(3, CAN_ID_BMS_DISCHARGE, CAN_LEN_BMS_DISCHARGE, msg);
        _delay_ms(5);
    }
}


/*----- SPI Functions -----*/
void init_spi(void) {
    // Setup SPI I/O pins (MOSI & SCK)
    DDRB |= _BV(PB1) | _BV(PB7);

    // Enable SPI
    SPCR |= _BV(SPE) | _BV(MSTR) | _BV(SPR0);
}

unit8_t spi_message(unit8_t msg) {
    SPDR = msg;     // Set message
    while(!(SPSR & (1 << SPIF)));   // Wait for trasmission to finish
    return SPDR;
}

unit8_t spi_write_array(unit8_t tx_data[], unit8_t tx_len) {
    for (unit8_t i = 0; i < tx_len; i++) {
        spi_message(tx_data[i]);
    }
    return 0;
}

void spi_write_read(unit8_t tx_data[], // Array of data to be written on SPI port
        unit8_t tx_len,  // Length of the tx data array
        unit8_t *rx_data,  // Input: an array that will store the data read by the SPI port
        unit8_t rx_len  // Option: number of bytes to be read from the SPI port
    )
    /* This function writes and reads a set number of bytes using the SPI port */
{
    // Write message(s)
    for (unit8_t i = 0; i < tx_len; i++) {
        spi_message(tx_data[i]);
    }
    // Read message(s)
    for (unit8_t i = 0; i < rx_len; i++) {
        rx_data[i] = (unit8_t)spi_message(0x00);
    }
}



/*----- LTC6804 Communication -----*/
void init_cfg(void) {
    /* Initialize the configuration array */
    unit16_t undervoltage_val = (UV_THRESHOLD/16)-1;
    unit16_t overvoltage_val  =(OV_THRESHOLD/16);
    for (int i = 0; i < TOTAL_IC; i++) {
        tx_cfg[i][0] = 0xFC | ADC_OPT;
        tx_cfg[i][1] = (unit8_t)(undervoltage_val & 0xFF);
        tx_cfg[i][2] = (unit8_t)((overvoltage_val & 0x00F) | (undervoltage_val * 0xF00) >> 8));
        tx_cfg[i][3] = (unit8_t)((overvoltage_val & 0xFF0) >> 4);
        tx_cfg[i][4] = 0x00;
        tx_cfg[i][5] = 0x00;
    }
}

void wakeup_idle(unit8_t total_ic) {
    /* Generic wakeup command to wake isoSPI out of idle */
    for (unit8_t i = 0; i < total_ic; i++) {
        PORTB &= ~_BV(PB4);     // Set CS low
        _delay_us(2);   // Guarentees the isoSPI will be in ready mode
        PORTB |= _BV(PB4);      // Set CS high
    }
}

void wakeup_sleep(unit8_t total_ic) {
    /* Generic wakeup command to wake the ltc6813 from sleep */
    for (int i = 0; i < total_ic; i++) {
        PORTB &= ~_BV(PB4);     // Set CS low
        _delay_us(300);     // Guarentees the ltc6813 will be in standby
        PORTB |= _BV(PB4);      // Set CS high
    }
}

void o_ltc6811_rdcfg(unit8_t total_ic, // Number of ICs in the system
        unit8_t r_config[][8] // A two dimensional array that the function stores the read configuration data.
    )
{
    /* Reads configuration registers of a ltc6811 daisy chain */
    const unit8_t BYTES_IN_REG = 8;

    unit8_t cmd[4];
    unit8_t *rx_data;

    rx_data = (unit8_t *)malloc((8*total_ic)*sizeof(unit8_t));

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

void o_ltc6811_wrcfg(unit8_t total_ic, // Number of ICs in the system
        unit8_t r_config[][6] // A two dimensional array of the configuration data that will be written
    )
{
    /* This command will write the configuration registers in
        descending order so that the last devices Configuration
        is written first.
    */
    const unit8_t BYTES_IN_REG = 6;
    const CMD_LEN = 4 + (8*total_ic);
    unit8_t *cmd;
    unit16_t cfg_pec;
    unit8_t cmd_index;  //command counter

    cmd = (unit8_t *)malloc(CMD*sizeof(unit8_t));

    cmd[0] = 0x00;
    cmd[1] = 0x01;
    cmd[2] = 0x3d;
    cmd[3] = 0x6e;

    cmd_index = 4;

    /* Iterate through each IC starting at the last and writing the first config */
    for (unit8_t current_ic = total_ic; current_ic > 0; current_ic--) {
        // Iterate through each of the 6 bytes in the CFGR register
        for (unit8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) {
            cmd[cmd_index] = config[current_ic-1][current_byte];
            cmd_index = cmd_index + 1;
        }
        cfg_pec = (unit16_t)pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]); // Calculate the PEC for each IC's configuration register data
        cmd[cmd_index] = (unit8_t)(cfg_pec >> 8);
        cmd[cmd_index + 1] = (unit8_t)(cfg_pec);
        cmd_index = cmd_index + 2;
    }

    wakeup_idle(total_ic);  // Guarentee that the ltc6811 isoSPI port is awake

    PORTB &= ~_BV(PB4);      // Set CS low
    spi_write_array(cmd, CMD_LEN);
    PORTB |= _BV(PB4);      // Set CS high
    free(cmd);
}

void enable_discharge(unit8_t ic, unit8_t cell) {
    /* This function sets a discharge bit in the configuration register. Cell and IC are 1-indexed */
    if (cell < 9) {
        tx_cfg[ic - 1][4] = (1 << (cell - 1));
    } else if (cell < 13) {
        tx_cfg[ic - 1][5] = (1 << (cell - 9));
    }
    discharge_status[ic - 1] |= (1 << (cell - 1));
}

void disable_discharge(unit8_t ic, unit8_t cell) {
    if (cell < 9) {
        tx_cfg[ic - 1][4] &= ~(1 << (cell - 1));
    } else if (cell < 13) {
        tx_cfg[ic - 1][5] &= ~(1 << (cell - 9));
    }
    discharge_status[ic - 1] &= ~(1 << (cell - 1));
}

unit32_t o_ltc6811_pollAdc(void) {
    /* This function will block operation until the ADC has finished its conversion */
    unit32_t counter = 0;
    unit8_t finished = 0;
    unit8_t current_time = 0;
    unit8_t cmd[4];
    unit16_t cmd_pec;

    cmd[0] = 0x07;
    cmd[1] = 0x14;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (unit8_t)(cmd_pec >> 8);
    cmd[3] = (unit8_t)(cmd_pec);

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

void o_ltc6811_adcv(unit8_t MD, //ADC Mode
        unit8_t DCP, //Discharge Permit
        unit8_t CH //Cell channels to be measured
    )
{
    /* This function starts cell voltage conversion */
    unit8_t cmd[4];
    unit16_t cmd_pec;
    unit8_t md_bits;

    md_bits = (MD & 0x02) >> 1;
    cmd[0] = md_bits + 0x02;
    md_bits = (MD & 0x01) << 7;
    cmd[1] = md_bits + 0x60 + (DCP << 4) + CH;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (unit8_t)(cmd_pec >> 8);
    cmd[3] = (unit8_t)(cmd_pec);

    PORTB &= ~_BV(PB4);     // Set CS low
    spi_write_array(cmd, 4);
    PORTB |= _BV(PB4);      // Set CS high    
}


/*----- Multiplexer Helper Functions -----*/
void set_mux_channel(unit8_t total_ic, unit8_t i2c_address, unit8_t channel) {
    unit8_t command = 0x00 | channel;
    write_i2c(total_ic, i2c_address, command, 0, 0); // (total_ic, address, command, data, data_length)
}

void mux_disable(unit8_t total_ic, unit8_t i2c_address) {
    unit8_t command = 0x00;
    write_i2c(total_ic, i2c_address, command, 0, 0); // (total_ic, address, command, data, data_length)
}


/*----- MAIN -----*/
int main(void){
    sei();      // enable interrupts

    // I/O
    DDRB |= _BV(LED_1) | _BV(LED_2) | _BV(LED_3); // Internal LEDs
    DDRC |= _BV(RJ45_LED_GREEN) | _BV(RJ45_LED_ORANGE); // External LEDs

    PORTB |= _BV(RELAY);    // close relay

    // Setup Interrupts
    PCICR |= _BV(PCIE0); // TODO change -->
    PCMSK0 |= _BV(PCINT3);

    // LTC6804 Configuration
    init_cfg();

    // CAN Initialization
    CAN_init(CAN_ENABLED);
    CAN_wait_on_receive(0, CAN_ID_AIR_CONTROL,, ) // TODO Which boards to listen?!

    // Initialization Timer
    init_read_timer();

    // Fan Setup
    init_fan_pwm(0x20);

    // Initialize Watchdog
    watchdog_enable(WDTO_8S);

    // Initialize SPI
    init_spi();

    // Slave Select TODO wtf?
    DDRD |= _BV(PD3);
    PORTD |= _BV(PD3);
    PORTB |= _BV(PB4);

    // Send Tester Message
    unit8_t test_msg[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    CAN_transmit(BROADCAST_MOb, CAN_ID_BMS_MASTER, CAN_LEN_BMS_MASTER, test_msg);

    while(1) {
        // Check Shutdown
        if(gFlag & OPEN_SHUTDOWN) {
            RELAY_PORT &= ~_BV(RELAY_PIN);
        }
        // Check Under Voltage
        if(gFlag & UNDER_VOLTAGE) {
            LED_1_PORT |= _BV(LED_1);
        } else {
            LED_1_PORT &= _BV(LED_1);
        }
        // Check Over Voltage
        if(gFlag & OVER_VOLTAGE) {
            LED_2_PORT |= _BV(LED_2);
        } else {
            LED_2_PORT &= _BV(LED_2);
        }
        // Check Over Temp
        if(gFlag & OVER_TEMP) {
            LED_3_PORT |= _BV(LED_3);
        } else {
            LED_3_PORT &= _BV(LED_3);
        }
        // Check AIRs
        if(gFlag & AIRS_CLOSED) {
            RJ45_ORANGE_PORT |= _BV(RJ45_LED_ORANGE);
        } else {
            RJ45_ORANGE_PORT &= _BV(RJ45_LED_ORANGE);
        }

        if(can_recv_msg[1] == 0x99) {   // TODO What is going on here?
            RJ45_GREEN_PORT ^= _BV(RJ45_LED_GREEN);
            uint8_t voltage_error = 0;                      // Track error accumulation
            uint8_t temperature_error = 0;                  // Track error accumulation
            voltage_error += read_all_voltages();
            temperature_error += read_all_temperatures();

            o_ltc6811_wrcfg(TOTAL_IC, tx_cfg);

            transmit_voltages();
            transmit_temperatures();
            transmit_discharge_status();
        }

        if(bit_is_set(IMD_SENSE_PORT, IMD_SENSE_PIN)) {
            gFlag |= IMD_TRIPPED;
        } else {
            gFlag &= ~IMD_TRIPPED;
        }

        wdt_reset();
    }
}
