#include <stdio.h> //for sprintf
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"
#include "log_uart.h"

//Global flag to transmit CAN
volatile uint8_t gFlag = 0x00;
volatile uint8_t msg_count = 0x00;
uint8_t clock_prescale = 0x0A;
volatile uint8_t ovf_count = 0x00;
volatile uint8_t msg[8]; //adding this for troubleshooting throttle

//CAN message objects
#define MOB 0

//CAN message
uint8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void timer0_setup(){
  sei(); //enable interrupts
  // Timer0 Overflow Interrupt Enabled
  TIMSK0 |= _BV(TOIE0);
  // Timer with 1024 prescaler
  TCCR0B |= _BV(CS02) | _BV(CS00);
}

ISR(TIMER0_OVF_vect){
  ovf_count++;
  if (ovf_count > clock_prescale) {
    gFlag ^= 0x01;
    ovf_count = 0x00;
  }
}

ISR(CAN_INT_vect) {
  msg_count++;
  //adding this for troubleshooting throttle messages
  CANPAGE = (MOB << MOBNB0);
  if (bit_is_set(CANSTMOB,RXOK)) {
      msg[0] = CANMSG;
      msg[1] = CANMSG;
      msg[2] = CANMSG;
      msg[3] = CANMSG;
      msg[4] = CANMSG;
      msg[5] = CANMSG;
      msg[6] = CANMSG;
      msg[7] = CANMSG;

      CANSTMOB = 0x00;
      CAN_wait_on_receive(MOB,
                          CAN_ID_MC_COMMAND,
                          CAN_LEN_MC_COMMAND,
                          CAN_IDM_single);
  }
}

//Suspension Strain or Air Control main

int main(void){

  timer0_setup();
  LOG_init();
  CAN_init(CAN_ENABLED);

  DDRB |= _BV(PB0);

  CAN_wait_on_receive(MOB,
                      CAN_ID_MC_COMMAND,
                      CAN_LEN_MC_COMMAND,
                      CAN_IDM_single);

  while(1) {
    if(gFlag) {
      gFlag = 0x00;
      char disp_string[128];
      //sprintf(disp_string,"%u messages received!",msg_count);
      sprintf(disp_string,"CAN message to motorcontroller:\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d",
      msg[0],msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7]);
      LOG_println(disp_string,strlen(disp_string));
      PORTB ^= _BV(PB0);
    }
  }
}


//Brakelight main
/*
int main(void) {

  timer0_setup();
  LOG_init();
  CAN_init(CAN_ENABLED);

  DDRC |= _BV(PC5);
  while(1){
    if(gFlag){
      CAN_transmit(MOB,
                   CAN_ID_BRAKE_LIGHT,
                   CAN_LEN_BRAKE_LIGHT,
                   gCANMessage);
      LOG_println("CAN message sent!",strlen("CAN message sent!"));
      PORTC ^= _BV(PC5);
    }
  }
}*/
