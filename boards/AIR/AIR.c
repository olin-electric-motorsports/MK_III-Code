/*
Header: Basic precharge implementation for AIR Control Board. This code has two basic states sensed through the TS- AIR auxillary contacts.
The first is when the TS- AIR is open (the shutdown circuit is not complete). In this state, the output pins which low-side-drive (LSD) the
precharge and TS+ AIR are set low and the precharge counter (which handles the length of the precharge sequence) is constantly reset to 0x00.
The second is when the TS- AIR is closed (the shutdown circuit is complete). In this state, the precharge LSD is immediately pulled high,
beginning the precharge sequence. An overflow interrupt on timer0 counts out 13 seconds (set by precharge_threshold). Once this time is exceeded,
the TS+ AIR LSD is pulled high and a CAN message should be sent stating that precharge is complete.

Author: Lucky Jordan
*/

/*----- Includes -----*/

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"
#include "log_uart.h"

/*----- Global Variables -----*/
volatile uint8_t timer_ovf_count = 0x00; //This variable is incremented on each timer overflow to give a measure of time passed
uint8_t precharge_threshold = 200; //This variable is a threshold at which we consider precharge complete when timer_ovf_count exceeds it
//Math for above precharge_threshold, I will probably change this value without bothering to change the explanation math at some point
//The timer is configured with a clock prescaler of 1024 in timer0_setup. This means that the clock cycles will increment by 1 each time
//1024 cycles pass. In this case, the clock frequency is set to 4MHz; therefore, we should see 4000000/1024 increments per second.
//Because the timer is 8 bit, it can only count up to 255 cycles and therefore overflows (4000000/1024)/255 times per second. To get the
//number of overflows in 13 seconds we multiply this value by 13: 13*((4000000/1024)/255) = 199 ish. We round up to 200 to get the threshold.
volatile uint8_t gPrechargeComplete = 0x00; //Store status of precharge for CAN message
volatile uint8_t gPechargeStarted = 0x00; //Flag to control when voltage messages start being printed
volatile uint8_t gBroadcast = 0x00; //Flag to broadcast CAN messagse, uses timer overflow for simplicity
volatile uint8_t gPrintVoltage = 0x00; //Flag to control how often voltage message is printed
volatile uint16_t MC_voltage = 0x00;

/*----- Macro Definitions -----*/
// On board LEDs
#define LED1		PB0
#define LED2		PB1

// External LEDs
#define EXT_LED1	PD0
#define EXT_LED2	PC0

// Precharge & AIR
#define PRECHARGE_LSD   PC4 //Pin to low side drive precharge relay
#define AIR_LSD         PC5 //Pin to low side drive AIR +
#define AIR_auxillary   PC6 //Pin to sense state of AIR - auxillary contacts

// CAN Message Objects
#define MOB_BROADCAST 0 //Broadcasts precharge sequence complete
#define MOB_MOTORCONTROLLER 1 //Receives messages from motor controller

// CAN Message
uint8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//Setup 8 bit timer
//helpful reference http://exploreembedded.com/wiki/AVR_Timer_programming
void timer0_setup(){
  sei(); //enable interrupts
  // Timer0 Overflow Interrupt Enabled
  TIMSK0 |= _BV(TOIE0);
  // Timer with 1024 prescaler
  TCCR0B |= _BV(CS02) | _BV(CS00);
}

ISR(TIMER0_OVF_vect){
  timer_ovf_count++; //Increment overflow count each time overflow interrupt is triggered
  gBroadcast ^= 0x01; //Flip value of gBroadcast so it broadcasts every other overflow (about 8Hz)
  gPrintVoltage = 0x00; //Print voltage each overflow
}

ISR(CAN_INT_vect) {
  CANPAGE = (MOB_MOTORCONTROLLER << MOBNB0);
  if (bit_is_set(CANSTMOB,RXOK)) {
      volatile uint8_t msg[2];
      msg[0] = CANMSG;
      msg[1] = CANMSG;

      MC_voltage = msg[0] | (msg[1]<<8);

      CANSTMOB = 0x00;
      CAN_wait_on_receive(MOB_MOTORCONTROLLER,
                          CAN_ID_MC_VOLTAGE,
                          CAN_LEN_MC_VOLTAGE,
                          CAN_IDM_single);
  }
}

int main(void){
  // need to enable can i think here

  timer0_setup();
  LOG_init();
  CAN_init(CAN_ENABLED);

  CAN_wait_on_receive(MOB_MOTORCONTROLLER,
                      CAN_ID_MC_VOLTAGE,
                      CAN_LEN_MC_VOLTAGE,
                      CAN_IDM_single);

  DDRB |= _BV(LED1) | _BV(LED2); //Set on board programming LEDs as uptputs
  DDRC |= _BV(PRECHARGE_LSD) | _BV(EXT_LED2) | _BV(AIR_LSD); //Set precharge lsd, air + lsd and one external led as output
  DDRD |= _BV(EXT_LED1); //Set other external led as output

  DDRC &= ~_BV(AIR_auxillary); //Set auxillary air - contact sense as input

  PORTC &= ~_BV(PRECHARGE_LSD) & ~_BV(AIR_LSD); //Make sure precharge and AIR + relays are open to start

  //strings for uart debugging
  char in_progress[] = "PRECHRGGGGRRRING";
  char complete[] = "PRECHRGGGG COMPLEEEEET";

  while(1){
    //Should I change this to interrupts? not sure, for loop seems like it might work fine but also this board does other stuff that I haven't implemented yet
    //If AIR - is closed (pin is high), start precharge sequence (pull precharge lsd high)
    if (bit_is_set(PINC, AIR_auxillary)) {
      // do precharge
      if (timer_ovf_count < precharge_threshold) {
        PORTC |= _BV(PRECHARGE_LSD);
        PORTD |= _BV(EXT_LED1);
        PORTB |= _BV(LED1);
        gPechargeStarted = 0x01;
        LOG_println(in_progress,strlen(in_progress));
      } else {
        PORTC |= _BV(AIR_LSD);
        PORTC |= _BV(EXT_LED2);
        PORTB |= _BV(LED2);
        gPrechargeComplete = 0xFF;
        LOG_println(complete,strlen(complete));
      }
    } else { //otherwise set both relays LSDs low (open the relays) and reset overflow count
      PORTC &= ~_BV(PRECHARGE_LSD) & ~_BV(AIR_LSD) & ~_BV(EXT_LED2);
      PORTD &= ~_BV(EXT_LED1);
      PORTB &= ~_BV(LED1) & ~_BV(LED2);
      timer_ovf_count = 0x00;
      gPrechargeComplete = 0x00;
      gPechargeStarted = 0x00;
    }

    //Send CAN message saying precharge is done, car can then be started by dashboard so throttle can start sending torque requests to motor controller
    if (gBroadcast) {
      gCANMessage[0] = gPrechargeComplete;
      CAN_transmit(MOB_BROADCAST,
                   CAN_ID_AIR_CONTROL,
                   CAN_LEN_AIR_CONTROL,
                   gCANMessage);
    }

    //Print voltage measurement from MC to monitor precharge
    if (gPechargeStarted && gPrintVoltage) {
      //sprintf(disp_string,"MC Voltage %d",MC_voltage);
      //LOG_println(disp_string,strlen(disp_string));
      gPrintVoltage = 0x00;
    }
  }
}
