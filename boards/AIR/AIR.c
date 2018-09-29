/*
Header: Basic precharge implementation for AIR Control Board. This code has two basic states sensed through the TS- AIR auxillary contacts.
The first is when the TS- AIR is open (the shutdown circuit is not complete). In this state, the output pins which low-side-drive (LSD) the
precharge and TS+ AIR are set low and the precharge counter (which handles the length of the precharge sequence) is constantly reset to 0x00.
The second is when the TS- AIR is closed (the shutdown circuit is complete). In this state, the precharge LSD is immediately pulled high,
beginning the precharge sequence. An overflow interrupt on timer0 counts out 13 seconds (set by precharge_threshold). Once this time is exceeded,
the TS+ AIR LSD is pulled high and a CAN message should be sent stating that precharge is complete.

UPDATE 09/28/18:
This code has been modified to be compatible with the rework to sense the AIR - shutdown lead using the HVD connector shutdown sense module.

Author: Lucky Jordan
*/

/*----- Includes -----*/
#include <stdio.h> //for sprintf
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"
#include "log_uart.h"

/*----- Global Variables -----*/
volatile uint8_t timer_ovf_count = 0x00; //This variable is incremented on each timer overflow to give a measure of time passed
uint8_t precharge_threshold = 100; //This variable is a threshold at which we consider precharge complete when timer_ovf_count exceeds it
//Math for above precharge_threshold, I will probably change this value without bothering to change the explanation math at some point
//The timer is configured with a clock prescaler of 1024 in timer0_setup. This means that the clock cycles will increment by 1 each time
//1024 cycles pass. In this case, the clock frequency is set to 4MHz; therefore, we should see 4000000/1024 increments per second.
//Because the timer is 8 bit, it can only count up to 255 cycles and therefore overflows (4000000/1024)/255 times per second. To get the
//number of overflows in 13 seconds we multiply this value by 13: 13*((4000000/1024)/255) = 199 ish. We round up to 200 to get the threshold.
volatile uint16_t MC_voltage = 0xFFFF;
uint8_t voltage_clock_prescale = 0x05;
uint8_t can_clock_prescale = 0x02;
volatile uint8_t voltage_ovf_count = 0x00; //used for printing motor controller voltage
volatile uint8_t can_ovf_count = 0x00; //used for when to send can messages
volatile uint8_t imd_ovf_count = 0x00; //used for when to send can messages
volatile uint8_t gFlag = 0x00;
uint8_t imd_delay_threshold = 50; //wait a bit before checking imd status

/*----- Macro Definitions -----*/
//for gFlag
#define print_voltage      0
#define send_can           1
#define imd_status         2
#define imd_shutdown       3
#define imd_delay_over     4
#define drive_air_lsd      7

//interrupts for shutdown and imd imd_status
#define imd_sd_pin          PCINT22
#define main_pack_conn_pin  PCINT23
// #define conn_to_hvd_pin     PCINT2
#define imd_status_pin      PCINT17

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
#define MOB_PANIC 2 //Panic MOB for BMS to open shutdown circuit

// CAN Message
uint8_t gCANMessage[5] = {0, 0, 0, 0, 0};
#define precharge_complete    0
#define sd_main_pack_conn     1
// #define sd_conn_to_hvd        2
#define sd_bms                3
#define sd_imd                4

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
  cli();
  timer_ovf_count++; //Increment overflow count each time overflow interrupt is triggered
  voltage_ovf_count++;
  can_ovf_count++;
  imd_ovf_count++;
  if (voltage_ovf_count > voltage_clock_prescale) {
    gFlag |= _BV(print_voltage);
    voltage_ovf_count = 0x00;
  }
  if (can_ovf_count > can_clock_prescale) {
    gFlag |= _BV(send_can);
    can_ovf_count = 0x00;
  }
  if (imd_ovf_count > imd_delay_threshold) {
    gFlag |= _BV(imd_delay_over);
  }
  if (timer_ovf_count > precharge_threshold) {
    gFlag |= _BV(drive_air_lsd);
  }
  sei();
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

// ISR(PCINT0_vect) {
//     if(bit_is_set(PINB,conn_to_hvd_pin)){
//         gCANMessage[sd_conn_to_hvd] = 0xFF;
//     } else {
//         gCANMessage[sd_conn_to_hvd] = 0x00;
//     }
// }

ISR(PCINT2_vect) {
    if(bit_is_set(PIND,imd_sd_pin)){
        gFlag |= _BV(imd_shutdown);
    } else {
        gFlag &= ~_BV(imd_shutdown);
    }

    if(bit_is_set(PIND,main_pack_conn_pin)){
        gCANMessage[sd_main_pack_conn] = 0xFF;
    } else {
        gCANMessage[sd_main_pack_conn] = 0x00;
    }

    if(bit_is_set(PIND,imd_status_pin)){
        gFlag |= _BV(imd_status);
    } else {
        gFlag &= ~_BV(imd_status);
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

  //interrupts
  PCICR |= _BV(PCIE0) | _BV(PCIE2);
  // PCMSK0 |= _BV(conn_to_hvd_pin);
  PCMSK2 |= _BV(imd_sd_pin) | _BV(main_pack_conn_pin) | _BV(imd_status_pin);

  //other IO stuff
  DDRB |= _BV(LED1) | _BV(LED2); //Set on board programming LEDs as uptputs
  DDRC |= _BV(PRECHARGE_LSD) | _BV(EXT_LED2) | _BV(AIR_LSD); //Set precharge lsd, air + lsd and one external led as output
  DDRD |= _BV(EXT_LED1); //Set other external led as output

  DDRC &= ~_BV(AIR_auxillary); //Set auxillary air - contact sense as input
  //Actually it's measuring a shutdown sense module connected to the low side AIR positive lead. HOPPE

  //if AIR is close immediately on startup send panic messasge
  if (bit_is_set(PINC, AIR_auxillary)) {
    CAN_transmit(MOB_PANIC,
          CAN_ID_PANIC,
          CAN_LEN_PANIC,
          0x00);
          char air_panic[] = "AIR PANIC!";
          LOG_println(air_panic,strlen(air_panic));
  }

  PORTC &= ~_BV(PRECHARGE_LSD) & ~_BV(AIR_LSD); //Make sure precharge and AIR + relays are open to start
  timer_ovf_count = 0x00;
  while(1){
    //Should I change this to interrupts? not sure, for loop seems like it might work fine but also this board does other stuff that I haven't implemented yet
    //If AIR - is closed (pin is high), start precharge sequence (pull precharge lsd high)
    if (bit_is_set(PINC, AIR_auxillary)) {
    // if (1) {
      // do precharge
      if (bit_is_clear(gFlag,drive_air_lsd)) {
        PORTC |= _BV(PRECHARGE_LSD);
        PORTD |= _BV(EXT_LED1);
        PORTB |= _BV(LED1);
        LOG_println("precharge",strlen("precharge"));
        // There is an interesting case here when timer_ovf_count resets because it's value gets too big for uint8_t
        // It doesn't affect operation but it's going to continuously cycle between this condition and the one below
      } else {
        PORTC &= ~_BV(PRECHARGE_LSD);
        PORTC |= _BV(AIR_LSD);
        PORTC |= _BV(EXT_LED2);
        PORTB |= _BV(LED2);
        gCANMessage[precharge_complete] = 0xFF;
        LOG_println("done",strlen("done"));
      }
    } else { //otherwise set both relays LSDs low (open the relays) and reset overflow count
      PORTC &= ~_BV(PRECHARGE_LSD) & ~_BV(AIR_LSD) & ~_BV(EXT_LED2);
      PORTD &= ~_BV(EXT_LED1);
      PORTB &= ~_BV(LED1) & ~_BV(LED2);
      gCANMessage[precharge_complete] = 0x00;
      timer_ovf_count = 0x00;
      gFlag &= ~_BV(drive_air_lsd);
      LOG_println("air open",strlen("air open"));
    }

    //check if imd status isn't right
    /*if (bit_is_set(gFlag,imd_delay_over)) {
      if(~(bit_is_set(gFlag,imd_status) ^ bit_is_set(gFlag,imd_shutdown))) {
        CAN_transmit(MOB_PANIC,
                    CAN_ID_PANIC,
                    CAN_LEN_PANIC,
                    0x00);
      }
      char imd_panic[] = "IMD PANIC!";
      LOG_println(imd_panic,strlen(imd_panic));
    }*/

    //Print voltage measurement from MC to monitor precharge
    if (bit_is_set(gFlag,print_voltage)) {
      gFlag &= ~_BV(print_voltage);
      char disp_string[64];
      sprintf(disp_string,"MC Voltage %u",MC_voltage);
      LOG_println(disp_string,strlen(disp_string));
    }

    //Send CAN message
    if (bit_is_set(gFlag,send_can) && bit_is_set(gFlag,imd_delay_over)) {
      gFlag &= ~_BV(send_can);
      if (bit_is_set(gFlag,imd_status)) {
        gCANMessage[sd_imd] = 0xFF;
      } else {
        gCANMessage[sd_imd] = 0x00;
      }
      CAN_transmit(MOB_BROADCAST,
                  CAN_ID_AIR_CONTROL,
                  CAN_LEN_AIR_CONTROL,
                  gCANMessage);
      char can_sent[] = "CAN sent!";
      LOG_println(can_sent,strlen(can_sent));
    }
  }
}
