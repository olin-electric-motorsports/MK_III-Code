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
