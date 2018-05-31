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
volatile uint8_t msg_voltages[8];
volatile uint8_t msg_temperatures[8];

//CAN message objects
#define MOB_TEMPERATURES 0
#define MOB_VOLTAGES 1

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
  CANPAGE = (MOB_VOLTAGES << MOBNB0);
  if (bit_is_set(CANSTMOB,RXOK)) {
      msg_voltages[0] = CANMSG;
      msg_voltages[1] = CANMSG;
      msg_voltages[2] = CANMSG;
      msg_voltages[3] = CANMSG;
      msg_voltages[4] = CANMSG;
      msg_voltages[5] = CANMSG;
      msg_voltages[6] = CANMSG;
      msg_voltages[7] = CANMSG;

      CANSTMOB = 0x00;
      CAN_wait_on_receive(MOB_VOLTAGES,
                          0x13,
                          8,
                          CAN_IDM_single);
  }

  CANPAGE = (MOB_TEMPERATURES << MOBNB0);
  if (bit_is_set(CANSTMOB,RXOK)) {
      msg_temperatures[0] = CANMSG;
      msg_temperatures[1] = CANMSG;
      msg_temperatures[2] = CANMSG;
      msg_temperatures[3] = CANMSG;
      msg_temperatures[4] = CANMSG;
      msg_temperatures[5] = CANMSG;
      msg_temperatures[6] = CANMSG;
      msg_temperatures[7] = CANMSG;

      CANSTMOB = 0x00;
      CAN_wait_on_receive(MOB_TEMPERATURES,
                          0x14,
                          8,
                          CAN_IDM_single);
  }
}

//Suspension Strain or Air Control main

int main(void){

  timer0_setup();
  LOG_init();
  CAN_init(CAN_ENABLED);

  DDRB |= _BV(PB0);

  CAN_wait_on_receive(MOB_VOLTAGES,
                      0x13,
                      8,
                      CAN_IDM_single);

  CAN_wait_on_receive(MOB_TEMPERATURES,
                      0x14,
                      8,
                      CAN_IDM_single);

  while(1) {
    if(gFlag) {
      gFlag = 0x00;
      char disp_string_voltages[128];
      //sprintf(disp_string,"%u messages received!",msg_count);
      sprintf(disp_string_voltages,"Voltage Message:\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d",
      msg_voltages[0],msg_voltages[1],msg_voltages[2],msg_voltages[3],
      msg_voltages[4],msg_voltages[5],msg_voltages[6],msg_voltages[7]);
      LOG_println(disp_string_voltages,strlen(disp_string_voltages));
      char disp_string_temperatures[128];
      //sprintf(disp_string,"%u messages received!",msg_count);
      sprintf(disp_string_temperatures,"Temperature Message:\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d",
      msg_temperatures[0],msg_temperatures[1],msg_temperatures[2],msg_temperatures[3],
      msg_temperatures[4],msg_temperatures[5],msg_temperatures[6],msg_temperatures[7]);
      LOG_println(disp_string_temperatures,strlen(disp_string_temperatures));
      PORTB ^= _BV(PB0);
    }
  }
}
