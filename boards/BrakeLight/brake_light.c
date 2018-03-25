/**********************************************************
This is the base document for getting started with writing
firmware for OEM.
TODO delete before submittin
**********************************************************/

/*
Header:
    Explain what this project does as overview
Author:
    @author
*/


/* TODO */
/*
    - Add sensing display integration
    - Add brake position sending
/*

/*----- Includes -----*/
#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*----- Macro Definitions -----*/
/* Shutdown */
#define GLOBAL_SHUTDOWN         0x0

/* Brake */
#define BRAKE_PIN               PC7 //TODO Analog Brake INPUT
#define BRAKE_PORT              PORTC //TODO
#define BRAKE_LIGHT_PIN         PD2 //TODO Analog Brake Light OUTPUT
#define BRAKE_LIGHT_PORT        PORTD //TODO

/* BSPD Status Output */
#define BSPD_STATUS_PIN         PC3 //TODO
#define BSPD_STATUS_PORT        PORTC //TODO

/* Sense Lines */
#define SD_MAIN_FUSE		PB0
#define SD_LEFT_E_STOP		PB1
#define SD_RIGHT_E_STOP		PD5
#define SD_BSPD     		PD6 //TODO CHECK?
#define SD_HVD      		PD7 //TODO CHECK?
#define SD_TSMS     		PB2 //TODO CHECK?

#define PORT_MAIN_FUSE      PORTB
#define PORT_LEFT_E_STOP    PORTB
#define PORT_RIGHT_E_STOP   PORTD
#define PORT_BSPD           PORTD
#define PORT_HVD            PORTD
#define PORT_TSMS           PORTB

/* CAN Positions */
#define CAN_BRAKE           0
#define CAN_BREAK_POS       1
#define CAN_BSPD            2
#define CAN_HVD             3
#define CAN_TSMS            4
#define CAN_LEFT_E_STOP     5
#define CAN_RIGHT_E_STOP    6
#define CAN_MAIN_FUSE       7

#define BROADCAST_MOb       0
#define //TODO readboards

/* Sense LEDs */
// Might be irrelevant because the gStatusBar
#define EXT_LED1            PD0 //TODO (Debug LED on RJ45)
#define EXT_LED2            PC0 //TODO (Debug LED on RJ45)
#define LED1                PB6 //TODO (Purpose - on LED bar)
#define LED2                PB6 //TODO (Purpose - on LED bar)
#define LED3                PB6 //TODO (Purpose - on LED bar)
#define LED4                PB6 //TODO (Purpose - on LED bar)
#define LED5                PB6 //TODO (Purpose - on LED bar)
#define LED6                PB6 //TODO (Purpose - on LED bar)

#define PORT_EXT_LED1       PORTD //TODO (Debug LED on RJ45)
#define PORT__EXT_LED2      PORTC //TODO (Debug LED on RJ45)
#define PORT_LED1           PB6 //TODO (Purpose - on LED bar)
#define PORT_LED2           PB6 //TODO (Purpose - on LED bar)
#define PORT_LED3           PB6 //TODO (Purpose - on LED bar)
#define PORT_LED4           PB6 //TODO (Purpose - on LED bar)
#define PORT_LED5           PB6 //TODO (Purpose - on LED bar)
#define PORT_LED4           PB6 //TODO (Purpose - on LED bar)

#define STATUS_MAIN_FUSE    0
#define STATUS_LEFT_E_STOP  1
#define STATUS_RIGHT_E_STOP 2
#define STATUS_BSPD         3
#define STATUS_HVD          4
#define STATUS_TSMS         5



/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x01;  // Global Flag
unit8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Message
unit8_t gPRECHARGE_TIMER = 0x00;
unit8_t gStatusBar[6] = {0, 0, 0, 0, 0, 0};         // LED status bar

volatile unit8_t gTSMS = 0x00;
volatile unit8_t gTSMS_OLD = 0x00;  // Used for comparison

#define UPDATE_STATUS   0
#define SEND_MSG        0       // Used for timer to send CAN msgs
#define TSMS_STATUS     1       // Used to track changes in TSMS

unit8_t clock_prescale = 0x00;  // Used for timer

/*----- Interrupt(s) -----*/
// CAN
ISR(CAN_INT_vect) {
    // TSMS
    CANPAGE = (0 << MOBNB0); //TODO correct page
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
    covers interupts 0-2
    Interupts covered: Main Shutdown Fuse, Left E-Stop, & TSMS
    */
    // TODO do we need?
}

ISR(PCINT2_vect) {
    /*
    Standard Pin Change Interupt
    covers interupts 21-23
    Interupts covered: Rright E-Stop, BSPD, HVD
    */
    // TODO do we need?
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
void initTimer_8bit(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0xFF;
}

static inline void read_pins(void) {

    /* Build CAN Message */
    if(bit_is_clear(PORT_MAIN_FUSE, SD_MAIN_FUSE)) {
        gCAN_MSG[CAN_MAIN_FUSE] = 0xFF;     // Electrical signal is low (meaning fuse is set)
    } else {
        gCAN_MSG[CAN_MAIN_FUSE] = 0x00;
    }

    if(bit_is_clear(PORT_LEFT_E_STOP, SD_LEFT_E_STOP)) {
        gCAN_MSG[CAN_LEFT_E_STOP] = 0xFF;
    } else {
        gCAN_MSG[CAN_LEFT_E_STOP] = 0x00;
    }

    if(bit_is_clear(PORT_RIGHT_E_STOP, SD_RIGHT_E_STOP)) {
        gCAN_MSG[CAN_RIGHT_E_STOP] = 0xFF;
    } else {
        gCAN_MSG[CAN_RIGHT_E_STOP] = 0x00;
    }

    if(bit_is_clear(PORT_BSPD, SD_BSPD)) {
        gCAN_MSG[CAN_BSPD] = 0xFF;
    } else {
        gCAN_MSG[CAN_BSPD] = 0x00;
    }

    if(bit_is_clear(PORT_HVD, SD_HVD)) {
        gCAN_MSG[CAN_HVD] = 0xFF;
    } else {
        gCAN_MSG[CAN_HVD] = 0x00;
    }

    if(bit_is_clear(PORT_TSMS, SD_TSMS)) {
        gCAN_MSG[CAN_TSMS] = 0xFF;
    } else {
        gCAN_MSG[CAN_TSMS] = 0x00;
    }

    if(bit_is_clear(BRAKE_PORT, BRAKE_PIN)) {
        gCAN_MSG[CAN_BRAKE] = 0xFF;
    } else {
        gCAN_MSG[CAN_BRAKE] = 0x00;
    }
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
    sei();                              // Enable interrupts

    /* Setup interrupt registers */
    PCICR |= _BV(PCIE0) | _BV(PCI2);
    PCMSK0 |= _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2);
    PCMSK2 |= _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23);

    initTimer_8bit();                   // Begin 8-bit timer

    gFlag |= _BV(UPDATE_STATUS);        // Read ports

    while(1) {
        if(bit_is_set(gFlag, UPDATE_STATUS)) {
            PORT_EXT_LED1 ^= _BV(EXT_LED1);     // Blink LED1 for timing check

            read_pins();                // Update all pin values

            CAN_transmit(BROADCAST_MOb, CAN_ID_BRAKE_LIGHT,
                CAN_LEN_BRAKE_LIGHT, gCAN_MSG);

            gFlag &= ~_BV(UPDATE_STATUS);
        }
    }
}
