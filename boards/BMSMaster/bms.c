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
#define RELAY                   PB2 //TODO change -->

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
#define BROADCAST_MOb           0


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
const unit16_t THERM_VOLTAGE = 3807 //TODO change -->

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





/*----- Interrupt(s) -----*/
// *pg 76 of datasheet*
ISR(CAN_INT_vect) {
    /*
    CAN Interupt
    -Check bits to see if they are set
        -IF they are, set global flag bit position
        -ELSE do nothing
    IMPORTANT, do not perform any 'real' operations in a interupt,
    just set the flag and move on
    */
}

ISR(PCINT0_vect) {
    /*
    Standard Pin Change Interupt
    */
}

ISR(TIMER0_COMPA_vect) {
    // Only send CAN msgs every 20 cycles
    if( clock_prescale > 20 ) {
        gFlag |= _BV(SEND_MSG);
        clock_prescale = 0;
    }
    clock_prescale++;
}


/*----- Functions -----*/
void initTimer_8bit(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0xFF;
}

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
    initTimer_8bit();   // TODO 8 or 16-bit timer?

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
        // do something
    }
}
