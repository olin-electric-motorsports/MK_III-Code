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
// #include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// #include "can_api.h"

/*----- Macro Definitions -----*/
// Pin Definitions
#define LED1		PB0		//TODO change
#define LED2		PB1
#define PORT_LED1	PORTB
#define PORT_LED2	PORTB

// External LEDs
#define EXT_LED1	PD0
#define EXT_LED2	PC0
#define	PORT_EXT_LED1	PORTD
#define PORT_EXT_LED2	PORTC

// Shutdown
#define FINAL_SHUTDOWN_RELAY		PD7
#define PIN_FINAL_SHUTDOWN_RELAY	PIND

#define PIN_SenseBMS PD5
#define PORT_SenseBMS PORTD
#define PIN_SenseIMD PD6
#define PORT_SenseIMD PORTD
#define PIN_SenseConnToHVD PB2
#define PORT_SenseConnToHVD PORTB
#define PIN_SenseMainTSConn PD7
#define PORT_SenseMainTSConn PORTD

#define SD2		//TODO
#define SD3		//TODO
#define SD4		//TODO
#define PIN_SD1
#define PIN_SD2
#define PIN_SD3
#define PIN_SD4
#define SD1_CAN
#define SD2_CAN
#define SD3_CAN
#define SD4_CAN

// Precharge & AIR
#define PRECHARGE 	PC4//TODO
#define PORT_PRECHARGE	PORTC//TODO
#define AIR_LSD		PC5//TODO
#define PORT_AIR_LSD	PORTC//TODO
#define AIR_Weld_Detect		PC6//TODO
#define PORT_AIR_Weld_Detecct	PORTC//TODO

// CAN Message Objects
#define BROADCAST_Mob	0
#define BMS_READ_Mob	1


// /*----- Global Variables -----*/
// volatile uint8_t gFlag = 0x01;  // Global Flag
// unit8_t gCANMessage[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // CAN Message
// unit8_t gPRECHARGE_TIMER = 0x00;
//
// #define UPDATE_STATUS 	0
// #define SET_PRECHARGE 	1
// #define SET_AIR         2
//
// unit8_t clock_prescale = 0x00;
//
// /*----- Interrupt(s) -----*/
// // CAN
// ISR(CAN_INT_vect) {
// }
//
// // 8-bit Timer
// ISR(TIMER0-COMPA-vect) {
//     /* Timer used to do processes only every 10 cycles */
//     if ( clock_prescale > 10 ) {
//     	gFlag |= _BV(UPDATE_STATUS);
//     	clock_prescale = 0;
//     }
//     clock_prescale++;
//
//
//     /* Precharge Timer
// 	-Turns on precharge
// 	-Waits 200 cycles
// 	-Turns off precharge & sets AIR */
//     if(bit_is_set(gFLAG, SET_PRECHARGE)) {
// 	PORT_EXT_LED2 |= _BV(EXT_LED2);		// Turn on LED2
// 	if(gPRECHARGE_TIMER > 200) {
// 	    gPRECHARGE_TIMER = 0x00;		// Reset timer
// 	    gFlag &= ~_BV(SET_PRECHARGE);	// Clear precharge bit
// 	    gFlag |= _BV(SET_AIR);		// Set AIR bit
// 	} else {
// 	    gPRECHARGE_TIMER++;			// Increment timer
// 	}
//     } else {
// 	PORT_EXT_LED2 &= ~_BV(EXT_LED2);
//     }
// }
//
// // 16-bit Timer
// ISR(TIMER1_COMPA_vect) {
// }
//
//
// /*----- Functions -----*/
// void initTimer_8bit(void) {
//     TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
//     TCCR0B = 0x05;          // clkio/1024 prescaler
//     TIMSK0 |= _BV(OCIE0A);
//     OCR0A = 0xFF;
// }
//
// void initTimer_16bit(void) {
// }
//
// static inline void read_pins(void) {
//     // Shutdown sense something...
//     if(bit_is_clear(PIN_FINAL_SHUTDOWN_RELAY, FINAL_SHUTDOWN_RELAY)) {
// 	PORT_LED2 |= _BV(LED2);			// Turn on LED2
// 	if(bit_is_clear(gFlag, SET_AIR)) {	// AIR not set
// 	    gFlag |= _BV(SET_PRECHARGE)		// set Precharge
// 	}
// 	gCAN_MSG[4] = 0xFF;			// send msg indicating starting of precharge
//     } else {
// 	PORT_LED2 &= ~_BV(LED2);		// Turn off LED2
// 	gFlag &= ~_BV(SET_PRECHARGE);		// Reset flags -->
// 	gFlag &= ~_BV(SET_AIR);
// 	gPRECHARGE_TIMER = 0x00;		// Reset timer
// 	gCAN_MSG[4] = 0x00;			// send msg indicating end of precharge
//     }
//
//
//     /* CAN msg bit checking  */
//     if(bit_is_clear(PIN_SD1, SD1)) {
// 	gCAN_MSG[SD1_CAN] = 0xFF;
//     } else {
// 	gCAN_MSG[SD1_CAN] = 0x00;
//     }
//
//     if(bit_is_clear(PIN_SD2, SD2)) {
// 	gCAN_MSG[SD2_CAN] = 0xFF;
//     } else {
// 	gCAN_MSG[SD2_CAN] = 0x00;
//     }
//
//     if(bit_is_clear(PIN_SD3, SD3)) {
// 	gCAN_MSG[SD3_CAN] = 0xFF;
//     } else {
// 	gCAN_MSG[SD3_CAN] = 0x00;
//     }
//
//     if(bit_is_clear(PIN_SD4, SD4)) {
// 	gCAN_MSG[SD4_CAN] = 0xFF;
//     } else {
// 	gCAN_MSG[SD4_CAN] = 0xFF;
//     }
// }


/*----- MAIN -----*/
int main(void)
{
  DDRB |= _BV(LED1) | _BV(LED2); //Makes PORT_B as output
  // DDRB &= ~(_BV(PIN_SenseConnToHVD)) //SET SenseConnToHVD as input
  // DDRC |= _BV(PRECHARGE); //SET Precharge as output
  // DDRC &= ~(_BV(AIR_Weld_Detect));
  DDRD &= ~(_BV(PIN_SenseIMD)) & ~(_BV(PIN_SenseBMS)) & ~(_BV(PIN_SenseMainTSConn)); //
  while(1)
  {
    if (PIND & _BV(PIN_SenseBMS)){
      // PIN_SenseBMS =
      PORT_LED1 ^= _BV(LED1);
      _delay_ms(100);
      // PORT_LED1 &= ~(1<<LED1);
      // _delay_ms(1000);
    }
  }


}
