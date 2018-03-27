/**********************************************************
This is the base document for getting started with writing
firmware for OEM.
TODO delete before submitting
**********************************************************/

/*
Header:
    Code for the throttle-steering-position board loacted
    in the Dashboard-Left Enclosure
Author:
    @author coreyacl
*/

/*----- Includes -----*/
#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*----- Macro Definitions -----*/
/* Shutdown */
#define GLOBAL_SHUTDOWN         0x0

/* Throttle */
#define THROTTLE_1              PC4
#define THROTTLE_2              PC5
#define THROTTLE_PORT           PORTC

/* Steering */
#define STEERING                PD5
#define STEERING_PORT           PORTD

/* Sense Lines */
#define SD_INERTIA              PB5
#define SD_ESTOP                PB6
#define SD_BOTS                 PB7
#define SD_PORT                 PORTB

/* Ready to Drive */
#define RTD_LD                  PC7
#define RTD_PORT                PORTC

/* LEDs */
#define LED1                    PC6
#define LED1_PORT               PORTC
#define LED2                    PB3
#define LED2_PORT               PORTB
#define LED3                    PB4
#define LED3_PORT               PORTB

#define EXT_LED1                PB0
#define EXT_LED2                PB1
#define EXT_LED_PORT            PORTB

/* CAN Positions */
#define CAN_THROTTLE_POS        0
#define CAN_STEERING_POS        1
#define CAN_BOTS                2
#define CAN_INTERTIA            3
#define CAN_DRIVER_E_STOP       4

//TODO
/*ATmega must:
-Sense 3 shutdown Lines
-Read and send steering pot value
-Read both throttle pots
-Send out throttle value over CAN
-Wait on RTD and trigger it 
*/

/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x00;  // Global Flag
uint8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Message

uint8_t RandomVar = 0x00;

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
    /*
    Timer/Counter0 compare match A
    */
}


/*----- Functions -----*/
void initTimer(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0xFF;
}

void initADC(void) {
    //Get the Analog to Digital Converter started (ADC)
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);

    //Enable interal reference voltage
    ADCSRB &= _BV(AREFEN);

    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);

    //Reads by default from ADC0 (pin 11)
    //This line is redundant. The timer
    ADMUX |= _BV(0x00);
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

//TODO any other functionality goes here


/*----- MAIN -----*/
int main(void){
    /*
    -Set up I/O
    -Set up CAN timer (done)
    -Initialize external libraries (if applicable)
    -Wait on CAN
    -Infinite loop checking shutdown state!
    */

    initTimer();
    initADC();
}
