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
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "can_api.h"

/*----- Macro Definitions -----*/
// LEDs
#define EXT_LED_ORANGE              PB5
#define EXT_LED_GREEN               PB4
#define PORT_EXT_LED_ORANGE         PORTB
#define PORT_EXT_LED_GREEN          PORTB
#define LED1_PIN                    PB6
#define PORT_LED1                   PORTB

// AMS and IMD Lights
#define AMS_PIN                     PB1
#define PORT_AMS                    PORTB
#define IMD_PIN                     PB0
#define PORT_IMD                    PORTB


// I/O
#define START_PIN                   PB2
#define PORT_START                  PORTB


#define AMS_PIN                     PB1
#define PORT_AMS                    PORTB
#define IMD_PIN                     PB0
#define PORT_IMD                    PORTB

// CAN Positions
#define CAN_START_BUTTON            0


// CAN Mailboxes
#define BRAKE_LIGHT_MBOX            0
#define BMS_MBOX                    1
#define AIR_MBOX                    2
#define DLEFT_MBOX                  3



// gFlag positions
#define STATUS_START                1
#define BRAKE_PRESSED               2
#define TSMS_CLOSED                 3
#define AMS_LIGHT                   4
#define IMD_STATUS                  5


#define UPDATE_STATUS               0


/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x00;  // Global Flag
uint8_t gCAN_MSG[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Message
uint8_t can_recv_msg[8] = {};

// Timer counters
uint8_t gClock_prescale = 0x00;  // Used for update timer


// Temp and Voltage Vars
uint8_t curr_temp;      // Temp
uint8_t curr_volt;      // Voltage
uint8_t curr_current;   // Current
uint8_t curr_SoC = 0;       // State of charge

uint8_t throttle = 0;




/*----- Interrupt(s) -----*/
// *pg 76 of datasheet*
ISR(CAN_INT_vect) {

    /*----- Brake Light Mailbox -----*/
    CANPAGE = (BRAKE_LIGHT_MBOX << MOBNB0);
    if(bit_is_set(CANSTMOB, RXOK)) {
        can_recv_msg[0] = CANMSG;   // brake
        can_recv_msg[1] = CANMSG;   // brake_pos
        can_recv_msg[2] = CANMSG;   // bspd
        can_recv_msg[3] = CANMSG;   // hvd
        can_recv_msg[4] = CANMSG;   // tsms
        can_recv_msg[5] = CANMSG;   // left_e_stop
        can_recv_msg[6] = CANMSG;   // right_e_stop
        can_recv_msg[7] = CANMSG;   // main_fuse

        if(can_recv_msg[1] == 0xFF) {
            gFlag |= _BV(BRAKE_PRESSED);           //trip flag
        } else {
            gFlag &= ~_BV(BRAKE_PRESSED);          //reset flag
        }

        if(can_recv_msg[4] == 0xFF) {
            gFlag |= _BV(TSMS_CLOSED);
        } else {
            gFlag &= ~_BV(TSMS_CLOSED);
        }

        //Setup to Receive Again
        CANSTMOB = 0x00;
        CAN_wait_on_receive(BRAKE_LIGHT_MBOX, CAN_ID_BRAKE_LIGHT, CAN_LEN_BRAKE_LIGHT, CAN_IDM_single);
    }

    /*----- BMS Master Mailbox -----*/
    CANPAGE = (BMS_MBOX << MOBNB0);
    if(bit_is_set(CANSTMOB, RXOK)) {
      can_recv_msg[0] = CANMSG;   // AMS light
      can_recv_msg[1] = CANMSG;   // IMD light
      can_recv_msg[2] = CANMSG;   // temperature
      can_recv_msg[3] = CANMSG;   // Avg. voltage
      can_recv_msg[4] = CANMSG;   // Avg. current
      can_recv_msg[5] = CANMSG;   // State of Charge

      // Grab AMS fault light
      if(can_recv_msg[0] == 0x00) {
          gFlag |= _BV(AMS_LIGHT);
      }

      // Grab temp, voltage, and current
      curr_volt = can_recv_msg[3];
      curr_temp = can_recv_msg[2];
      curr_current = can_recv_msg[4];
      curr_SoC = can_recv_msg[5];
      OCR1B = can_recv_msg[5];



      // If AMS shutdown is true, make AMS_PIN high

      // If IMD shutdown is true, make IMD_PIN high (if IMD goes low within 2 seconds of car on)
      // make sure these latch (don't turn off until board is turned off)

      //Setup to Receive Again
      CANSTMOB = 0x00;
      CAN_wait_on_receive(BMS_MBOX, CAN_ID_BMS_MASTER, CAN_LEN_BMS_MASTER, CAN_IDM_single);
    }

    /*----- AIRs Mailbox -----*/
    CANPAGE = (AIR_MBOX << MOBNB0); //repeat with mailbox 1 to listen for AMS and IMD
    if(bit_is_set(CANSTMOB, RXOK)) {
      can_recv_msg[0] = CANMSG;   // Precharge status 0xFF if complete
      can_recv_msg[1] = CANMSG;   // Main tractive system
      can_recv_msg[2] = CANMSG;   // Connector to HVD
      can_recv_msg[3] = CANMSG;   // BMS status
      can_recv_msg[4] = CANMSG;   // IMD status

      // Grab IMD status
      if(can_recv_msg[4] == 0xFF) {
          gFlag |= _BV(IMD_STATUS);
      }


      // If IMD shutdown is true, make IMD_PIN high (if IMD goes low within 2 seconds of car on)
      // make sure these latch (don't turn off until board is turned off)

      //Setup to Receive Again
      CANSTMOB = 0x00;
      CAN_wait_on_receive(AIR_MBOX, CAN_ID_AIR_CONTROL, CAN_LEN_AIR_CONTROL, CAN_IDM_single);
    }

    CANPAGE = (DLEFT_MBOX << MOBNB0);
    if(bit_is_set(CANSTMOB, RXOK)) {
        can_recv_msg[0] = CANMSG;   // throttle
        can_recv_msg[1] = CANMSG;   // steering
        can_recv_msg[2] = CANMSG;   // bots
        can_recv_msg[3] = CANMSG;   // inertia
        can_recv_msg[4] = CANMSG;   // estop

        throttle = can_recv_msg[0];
        OCR1A = can_recv_msg[0];




        //Setup to Receive Again
        CANSTMOB = 0x00;
        CAN_wait_on_receive(DLEFT_MBOX, CAN_ID_THROTTLE, CAN_LEN_THROTTLE, CAN_IDM_single);
    }


}

ISR(PCINT0_vect) {
    /*
    Standard Pin Change Interupt
    Covers Interrupts: Start Button
    */
    if(bit_is_set(PINB,START_PIN)) {

        gFlag |= _BV(STATUS_START);
    } else {
        gFlag &= ~_BV(STATUS_START);
    }
}

ISR(PCINT1_vect) {

    if(bit_is_set(PINC,PC1)){
        PORTD |= _BV(PD3);
    }else{
        PORTD &= ~_BV(PD3);
    }

}

ISR(TIMER0_COMPA_vect) {
    // Only send CAN msgs every 20 cycles
    if(gClock_prescale > 10) {
        gFlag |= _BV(UPDATE_STATUS);
        gClock_prescale = 0;
    }
    gClock_prescale++;
}


/*----- Functions -----*/
void initTimer(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0xFF;
}

void initIO(void) {
    /* Initialize all inputs and outputs and interrupts */
    sei();                  // Enable Interrupts

    DDRB |= _BV(LED1_PIN) | _BV(EXT_LED_ORANGE) | _BV(EXT_LED_GREEN) | _BV(AMS_PIN) | _BV(IMD_PIN);


    /* Setup interrupt registers */
    PCICR |= _BV(PCIE0) | _BV(PCIE1);
    PCMSK0 |= _BV(PCINT2);
    PCMSK1 |= _BV(PCINT9);

    /*----- Setup PWM output -----*/
    //Output compare pin is OC1B, so we need OCR1B as our counter
    DDRD |= _BV(PD2); //Enable output pin
    DDRD |= _BV(PD3); //Enable output pin
    DDRC |= _BV(PC1);

    // pg 119 and 110 of datasheet
    // currently running on mode 5 for 8-bit resolution timer
    // turns on when counter hits OCRnx and off when hits top (255)
    TCCR1A |= _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0) | _BV(WGM10);
    TCCR1A &= ~_BV(WGM11) & ~_BV(COM1A0) & ~_BV(COM1B0);
    TCCR1B |= _BV(CS10); //Clock prescale set to max speed
    TCCR1B |= _BV(WGM12);
    TCCR1B &= ~_BV(WGM13);


    // OCR1A = (uint8_t) 30;      // Duty Cycle
    // OCR1B = (uint8_t) 255;      // Duty Cycle

}

void checkShutdownState(void)   {
    /*
    -Check if bits are set
        -IF they are, set CAN list position to 0xFF
        -ELSE do set CAN list position to 0x00
    */
    // Check Start Button
    if(bit_is_set(gFlag, STATUS_START)) {
        gCAN_MSG[CAN_START_BUTTON] = 0xFF;
    } else {
        gCAN_MSG[CAN_START_BUTTON] = 0x00;
    }
}

void updateStateFromFlags(void) {
    /*
    Based off the state of the flag(s), update components and send CAN
    */
    // Check AMS light
    if((bit_is_set(gFlag, AMS_LIGHT))) {
        PORT_AMS |= _BV(AMS_PIN);
    }

    // Check IMD light
    if((bit_is_set(gFlag, IMD_STATUS))) {
        PORT_IMD |= _BV(IMD_PIN);
    }
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


/*----- MAIN -----*/
int main(void){
    /*
    -Set up I/O
    -Set up CAN timer (done)
    -Initialize external libraries (if applicable)
    -Wait on CAN
    -Infinite loop checking shutdown state!
    */

    initIO();                           // Initialize I/O

    // CAN Enable
    CAN_init(CAN_ENABLED);
    // CBN Enable

    CAN_wait_on_receive(BRAKE_LIGHT_MBOX, CAN_ID_BRAKE_LIGHT, CAN_LEN_BRAKE_LIGHT, CAN_IDM_single);
    CAN_wait_on_receive(BMS_MBOX, CAN_ID_BMS_MASTER, CAN_LEN_BMS_MASTER, CAN_IDM_single);
    CAN_wait_on_receive(AIR_MBOX, CAN_ID_AIR_CONTROL, CAN_LEN_AIR_CONTROL, CAN_IDM_single);
    CAN_wait_on_receive(DLEFT_MBOX, CAN_ID_THROTTLE, CAN_LEN_THROTTLE, CAN_IDM_single);

    initTimer();                        // Initialize Timer
    gFlag |= _BV(UPDATE_STATUS);        // Read ports

    // uint8_t count = 0;

    // OCR1B = curr_SoC;

    while(1) {
        if(bit_is_set(gFlag, UPDATE_STATUS)) {
            PORT_EXT_LED_ORANGE ^= _BV(EXT_LED_ORANGE);     // Blink Orange LED for timing check
            PORT_LED1 ^= _BV(LED1_PIN);

            updateStateFromFlags();
            checkShutdownState();


            if(bit_is_set(gFlag, BRAKE_PRESSED) && bit_is_set(gFlag, TSMS_CLOSED) && bit_is_set(gFlag,STATUS_START)) {
                CAN_transmit(5, CAN_ID_DASHBOARD, CAN_LEN_DASHBOARD, gCAN_MSG);
                PORT_EXT_LED_GREEN |= _BV(EXT_LED_GREEN);     // Blink Orange LED for timing check

            }


            if(bit_is_set(gFlag, TSMS_CLOSED)) {
                PORT_EXT_LED_ORANGE |= _BV(EXT_LED_ORANGE);
            }
            gFlag &= ~_BV(UPDATE_STATUS);  // Clear Flag
        }
    }
}
