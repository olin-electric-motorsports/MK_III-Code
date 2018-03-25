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



/*----- LTC6804 Communication -----*/
void init_cfg(void) {
    // Coms Configuration
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
