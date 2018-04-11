/*
Header:
    This board manages the brake light, 6 sense modules, and the brake
    position. It sends sense statuses over CAN as well as the brake position
    every certain amount of time.
Author:
    @author Peter Seger & Vienna Scheyer
*/

/*----- Includes -----*/
#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*----- Macro Definitions -----*/
/* Brake */
#define BRAKE_PIN           PB5
#define PORT_BRAKE          PORTB
#define ANALOG_BRAKE_PIN    PB7
#define ANALOG_BRAKE_PORT   PORTB

/* BSPD Status Output */
#define BSPD_STATUS_PIN         PC3 //TODO
#define BSPD_STATUS_PORT        PORTC //TODO

/* Sense Lines */
#define SD_MAIN_FUSE		PB0
#define SD_LEFT_E_STOP		PB1
#define SD_RIGHT_E_STOP		PD5
#define SD_BSPD     		PD6
#define SD_HVD      		PD7
#define SD_TSMS     		PB2

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



/* Sense LEDs */
// Might be irrelevant because the gStatusBar
#define EXT_LED_GREEN           PD0 //(Debug LED on RJ45)
#define EXT_LED_ORANGE          PC0 //(Debug LED on RJ45)
#define LED1                    PC5 //(Purpose - on LED bar)
#define LED2                    PC4 //(Purpose - on LED bar)

#define PORT_EXT_LED_GREEN      PORTD //(Debug LED on RJ45)
#define PORT_EXT_LED_ORANGE     PORTC //(Debug LED on RJ45)
#define PORT_LED1               PORTC //(Purpose - on LED bar)
#define PORT_LED2               PORTC //(Purpose - on LED bar)


//gFlag Positions
#define STATUS_MAIN_FUSE    1
#define STATUS_LEFT_E_STOP  2
#define STATUS_RIGHT_E_STOP 3
#define STATUS_BSPD         4
#define STATUS_HVD          5
#define STATUS_TSMS         6
#define STATUS_BRAKE        7

//gTimerFlag Positions
#define UPDATE_STATUS   0       // Determines when to send messages
#define SEND_BRAKE      1       // Determines when to send brake positions



/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x01;          // Global Flag
volatile uint8_t gTimerFlag = 0x01;     // Timer flag
unit8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Message
unit8_t gPRECHARGE_TIMER = 0x00;

volatile unit8_t gTSMS = 0x00;
volatile unit8_t gTSMS_OLD = 0x00;  // Used for comparison

// Timer counters
unit8_t clock_prescale = 0x00;  // Used for update timer
unit8_t brake_timer = 0x00;     // Used for brake timer

// Brake POS mapping Values
uint8_t brake_HIGH = 0xE7;       //TODO change with actual values
uint8_t brake_LOW = 0xD3;        //TODO change with actual values



/*----- Interrupt(s) -----*/
// 8-bit Timer
ISR(TIMER0_COMPA_vect) {
    // Only send CAN msgs every 20 cycles
    if(clock_prescale > 20) {
        gTimerFlag |= _BV(UPDATE_STATUS);
        clock_prescale = 0;
    }
    clock_prescale++;

    // Only send a brake message every 40 cycles
    if(brake_timer > 40) {
        gTimerFlag |= _BV(SEND_BRAKE);
        brake_timer = 0;
    }
    brake_timer++;
}

// CAN
ISR(CAN_INT_vect) {
    /*
    Currently not reading CAN messages
    */
}

ISR(PCINT0_vect) {
    /*
    Standard Pin Change Interupt
    covers interupts 0-3
    Interupts covered: Main Shutdown Fuse, Left E-Stop, TSMS, & Brake
    */
    if(PORT_MAIN_FUSE, SD_MAIN_FUSE) {
        gFlag |= _BV(STATUS_MAIN_FUSE);
    } else {
        gFlag &= ~_BV(STATUS_MAIN_FUSE);
    }
    if(PORT_LEFT_E_STOP, SD_LEFT_E_STOP) {
        gFlag |= _BV(STATUS_LEFT_E_STOP);
    } else {
        gFlag &= ~_BV(STATUS_LEFT_E_STOP);
    }
    if(PORT_TSMS, SD_TSMS) {
        gTSMS = PORT_TSMS & SD_TSMS;
        gFlag |= _BV(STATUS_TSMS);
    } else {
        gFlag &= ~_BV(STATUS_TSMS);
    }
    if(PORT_BRAKE, PIN_BRAKE) {
        gFlag |= _BV(STATUS_BRAKE);
    } else {
        gFlag &= ~_BV(STATUS_BRAKE);
    }
}

ISR(PCINT2_vect) {
    /*
    Standard Pin Change Interupt
    covers interupts 21-23
    Interupts covered: Right E-Stop, BSPD, HVD
    */
    if(PORT_RIGHT_E_STOP, SD_RIGHT_E_STOP) {
        gFlag |= _BV(STATUS_RIGHT_E_STOP);
    } else {
        gFlag &= ~_BV(STATUS_RIGHT_E_STOP);
    }
    if(PORT_BSPD, PORT_BSPD) {
        gFlag |= _BV(STATUS_BSPD);
    } else {
        gFlag &= -_BV(STATUS_BSPD);
    }
    if(PORT_HVD, PORT_HVD) {
        gFlag |= _BV(STATUS_HVD);
    } else {
        gFlag &= -_BV(STATUS_HVD);
    }
}


/*----- Functions -----*/
void initTimer_8bit(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0xFF;
}

static inline void updateStateFromFlags(void) {

    /* Build CAN Message */
    if(bit_is_set(gFlag, STATUS_MAIN_FUSE)) {
        gCAN_MSG[CAN_MAIN_FUSE] = 0xFF;     // Electrical signal is low (meaning fuse is set)
    } else {
        gCAN_MSG[CAN_MAIN_FUSE] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_LEFT_E_STOP)) {
        gCAN_MSG[CAN_LEFT_E_STOP] = 0xFF;
    } else {
        gCAN_MSG[CAN_LEFT_E_STOP] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_RIGHT_E_STOP)) {
        gCAN_MSG[CAN_RIGHT_E_STOP] = 0xFF;
    } else {
        gCAN_MSG[CAN_RIGHT_E_STOP] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_BSPD)) {
        // Check if fault
        if(bit_is_clear(BSPD_STATUS_PORT, BSPD_STATUS_PIN)) {       //TODO are these different?
            gCAN_MSG[0] = 0xFF;
            // Send Global Panic
            CAN_transmit(BROADCAST_MOb, CAN_ID_PANIC,
                CAN_LEN_PANIC, gCAN_MSG);
            gCAN_MSG[CAN_BSPD] = 0xFF;
        }
    } else {
        gCAN_MSG[CAN_BSPD] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_HVD)) {
        gCAN_MSG[CAN_HVD] = 0xFF;
    } else {
        gCAN_MSG[CAN_HVD] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_TSMS)) {
        if(gTSMS != gTSMS_OLD) {
            gCAN_MSG[CAN_TSMS] = gTSMS;
            gTSMS_OLD = gTSMS;
        }
    }

    if(bit_is_set(gFlag, STATUS_BRAKE)) {
        gCAN_MSG[CAN_BRAKE] = 0xFF;
    } else {
        gCAN_MSG[CAN_BRAKE] = 0x00;
    }

}

static inline void mapBrakePos() {
    /* This function polls the brake position and maps it to
        a byte for sending over CAN. In range [0x00, 0xFF] */
    uint8_t raw = ANALOG_BRAKE_PORT & ANALOG_BRAKE_PIN;
    uint8_t mapped = ((raw - brake_LOW) * 0xFF) / (brake_HIGH - brake_LOW);
    gCAN_MSG[CAN_BREAK_POS] = mapped;
}


/*----- MAIN -----*/
int main(void){
    /*
    -Set up Interrupts
    -Set up CAN timer
    -While
        -Every 20 timer cycles, send flag status
        -Every 40 timer cycles, send brake position
    */
    sei();                              // Enable interrupts

    /* Setup interrupt registers */
    PCICR |= _BV(PCIE0) | _BV(PCI2);
    PCMSK0 |= _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT5);      // Covers Pins: Main Fuse, Left E-Stop, TSMS, & Brake Light
    PCMSK2 |= _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23);   // Covers Pins: Right E-Stop, BSPD, and HVD

    initTimer_8bit();                       // Begin 8-bit timer
    gTimerFlag |= _BV(UPDATE_STATUS);        // Read ports

    while(1) {
        if(bit_is_set(gTimerFlag, UPDATE_STATUS)) {
            PORT_EXT_LED_ORANGE ^= _BV(EXT_LED_ORANGE);     // Blink Orange LED for timing check

            updateStateFromFlags();     // Build CAN message based off flags
            gTimerFlag &= ~_BV(UPDATE_STATUS);

            if(bit_is_set(gTimerFlag, SEND_BRAKE)) {
                mapBrakePos();              // Add brake position to CAN message
                gTimerFlag &= ~_BV(SEND_BRAKE);
            }

            // Send CAN message
            CAN_transmit(BROADCAST_MOb, CAN_ID_BRAKE_LIGHT,
                CAN_LEN_BRAKE_LIGHT, gCAN_MSG);
        }
    }
}
