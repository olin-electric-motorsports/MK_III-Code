/**********************************************************
This is the base document for getting started with writing
firmware for OEM.
TODO delete before submitting
**********************************************************/

/*
Header:
    Explain what this project does as overview
Author:
    @author
*/

/*----- Includes -----*/
#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*----- Macro Definitions -----*/
// Shutdown
#define GLOBAL_SHUTDOWN         0x0

// Brake
#define BRAKE_PIN               PC7 //TODO Analog Brake INPUT
#define BRAKE_PORT              PORTC //TODO
#define BRAKE_LIGHT_PIN         PD2 //TODO Analog Brake Light OUTPUT
#define BRAKE_LIGHT_PORT        PORTD //TODO

// BSPD Status Output
#define BSPD_STATUS_PIN         PC3 //TODO
#define BSPD_STATUS_PORT        PORTC //TODO

// Sense Lines
#define SD_MAIN_FUSE		//TODO Main Fuse
#define SD_LEFT_E_STOP		//TODO Left E-Stop
#define SD_RIGHT_E_STOP		//TODO Right E-Stop
#define SD_BSPD     		//TODO BSPD
#define SD_HVD      		//TODO HVD
#define SD_TSMS     		//TODO TSMS
#define PORT_MAIN_FUSE
#define PORT_LEFT_E_STOP
#define PORT_RIGHT_E_STOP
#define PORT_BSPD
#define PORT_HVD
#define PORT_TSMS
#define CAN_MAIN_FUSE
#define CAN_LEFT_E_STOP
#define CAN_RIGHT_E_STOP
#define CAN_BSPD
#define CAN_HVD
#define CAN_TSMS

// Sense LEDs
/* Might be irrelevant because the gStatusBar */
#define LED1                PB6 //TODO (Purpose)
#define LED2                PB6 //TODO (Purpose)
#define LED3                PB6 //TODO (Purpose)
#define LED4                PB6 //TODO (Purpose)
#define LED5                PB6 //TODO (Purpose)
#define LED6                PB6 //TODO (Purpose)
#define PORT_LED1           PB6 //TODO (Purpose)
#define PORT_LED2           PB6 //TODO (Purpose)
#define PORT_LED3           PB6 //TODO (Purpose)
#define PORT_LED4           PB6 //TODO (Purpose)
#define PORT_LED5           PB6 //TODO (Purpose)
#define PORT_LED6           PB6 //TODO (Purpose)


/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x01;  // Global Flag
unit8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Message
unit8_t gPRECHARGE_TIMER = 0x00;
unit8_t gStatusBar[6] = {0, 0, 0, 0, 0, 0};         // LED status bar

volatile unit8_t gTSMS = 0x00;
volatile unit8_t gTSMS_OLD = 0x00;  // Used for comparison

#define SEND_MSG       0       // Used for timer to send CAN msgs
#define TSMS_STATUS     1       // Used to track changes in TSMS

unit8_t clock_prescale = 0x00;  // Used for timer

/*----- Interrupt(s) -----*/
// CAN
ISR(CAN_INT_vect) {
    // TSMS
    CANPAGE = (0 << MOBNBO); //TODO correct page
    if(bit_is_set(CANSTMOB, RXOK)) {
        volatile unit8_t msg = CANSMG;          //TODO figure out order of msg
        gTSMS = msg;
        if(msg == 0x00) {
            gFlags &= ~_BV(TSMS_STATUS)         // Clear bit
        } else {
            gFlags |= _BV(TSMS_STATUS)          // Set bit
        }
    }
}

ISR(PCINT0_vect) {
    /*
    Standard Pin Change Interupt
    */
}

// 8-bit Timer
ISR(TIMER0_COMPA_vect) {
    // Only send CAN msgs every 20 cycles
    if( clock_prescale > 20 ) {
        gFlag |= _BV(SEND_MSG);
        clock_prescale = 0;
    }
    clock_prescale++;
}


/*----- Functions -----*/
void initTimer(void) {
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

//TODO any other functionality goes here
/*
    MISSING MODULE
    TODO
    Update LED status bar
*/


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
}
