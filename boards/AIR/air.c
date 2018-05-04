/*
Header:
    This is the firmware code for AIR Control Board.
Author:
    @Josh Deng, Sherie SHen
*/

/*----- Includes -----*/
#define F_CPU(16000000UL)
#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"

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
#define IMD_STATUS PD1
#define BMS_STATUS PC1

// Precharge & AIR
#define PRECHARGE 	PC4//TODO
#define PORT_PRECHARGE	PORTC//TODO
#define PIN_AIR_LSD		PC5
#define PORT_AIR_LSD	PORTC//
#define AIR_Weld_Detect		PC6//TODO
#define PORT_AIR_Weld_Detecct	PORTC//TODO

// CAN Message Objects
#define GLOBAL_SHUTDOWN      0x0
#define BROADCAST_Mob	0
#define BMS_READ_Mob	1

/*---- CAN Position Macros ----*/
#define BMS 0
#define IMD 1
#define MainTSConn 3
#define HVD 4
#define BMS_Status 5
#define IMD_Status 6

/*----- GLOBAL FLAGS -----*/
#define HVD_SD_FLAG 	0
#define BMS_SD_FLAG 	1
#define IMD_SD_FLAG  2
#define MainTS_SD_FLAG 3
#define IMD_STATUS_FLAG 4
#define BMS_STATUS_FLAG 5
#define AIR_minus_STATUS_FLAG 6
#define Brake_Light 7


#define UPDATE_STATUS_FLAG 0
#define AIR_plus_STATUS_FLAG 0


volatile uint8_t gFlag = 0x00;
volatile unsigned long timer0_millis = timer0_frac = timer0_overflow_num = 0;
volatile uint8_t clock_prescale = 0x00;
volatile uint8_t gFlag2 = 0x01; // Timer Flag

unsigned long time = 0;
unsigned long precharge_start_time = 0;

uint8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};


/*----- FUNCTION DEFINITIONS -----*/
void enableInterrupt(){
  sei();  //Enable Global Interrupt

  // Enable Pin Change Interrupt Enable 2
  PCICR |= _BV(PCIE2) | _BV(PCIE1) | _BV(PCIE0);
  // Enable Pin Change Interrupts on PCINT[23:21] (PD[7:5], Sense[BMS, IMD, MainTS])
  // and PCINT17(IMD Status, PD1)
  PCMSK2 = (_BV(PCINT23) | _BV(PCINT22) | _BV(PCINT21);
  // Enable Pin Change Interrupts on PCINT9 (BMS_Staus, PC1)
  PCMSK1 = _BV(PCINT9);
  // Enable Pin Change Interrupts on PCINT2 (PB2)
  PCMSK0 = _BV(PCINT2);
}


static inline uint8_t check_BMS(){
  if (PIND & _BV(PIN_SenseBMS)){
    return 1;
  }else{
    return 0;
  }
}


static inline uint8_t check_IMD(){
  if (PIND & _BV(PIN_SenseIMD)){
    return 1;
  }else{
    return 0;
  }
}


static inline uint8_t check_MainTSConn(){
  if (PIND & _BV(PIN_SenseMainTSConn)){
    return 1;
  }else{
    return 0;
  }
}


static inline uint8_t check_HVD(){
  if (PIND & _BV(PIN_SenseConnToHVD)){
    return 1;
  }else{
    return 0;
  }
}


static inline uint8_t IMD_status(){
  time = millis();
  if millis() - time = 2000{
    if (PIND & _BV(IMD_STATUS)){

      return 1;
    }else{
      return 0;
    }
  }
}


static inline uint8_t BMS_status(){
  if (PINC & _BV(BMS_STATUS)){
    return 1;
  }else{
    return 0;
  }
}

static inline uint8_t AIR_minus_status(){
  if (PINC & _BV(AIR_Weld_Detect) && (gFlag & _BV(AIR_minus_STATUS_FLAG))){
    return 1;
  }else{
    return 0;
  }
}

void timer0_setup(){
  sei();
  // Timer0 Overflow Interrupt Enabled
  TIMSK0 |= _BV(TOIE);
  // Timer 0 with 64 prescaler if F_CPU == 16,000,000
  TCCR0B |= _BV(CS01) | _BV(CS00);
}


unsigned long millis(){
  oldSREG = SREG;
  cli();
  unsigned long m = timer0_millis;
  SREG = oldSREG;
  return m;
}



inline void low_side_drive_precharge(){
  if (gFlag & _BV(Brake_Light)){
    if (!precharge_start_status){
      precharge_start_time = millis();
    }
    PORTC |= _BV(PRECHARGE);
    precharge_start_status = 1;

    // open relay after charging time
    if(milis() - precharge_start_time > charging_time){
      PORTC &= ~_BV(PRECHARGE);
      gFlag |= _BV(AIR_plus_STATUS_FLAG);
      gFlag &= ~_BV(Brake_Light);
    }
  }
}


void low_side_drive_AIR_plus(){
  if (gFlag2 & _BV(AIR_plus_STATUS_FLAG)){
    PORTC |= _BV(PIN_AIR_LSD);
  }
}


/*----- INTERRUPT DEFINITIONS -----*/
ISR(PCI2_vect){
  /*When Pin Change Interrupt 2 is triggered, check the status on
  BMS, IMD, and MainTSConn; check whether BMS are the same or
  different, if not send message*/
  // TODO: to make sure the first two seconds of IMD is not read
  if(!IMD_status()){
     gCANMessage[IMD_Status] = 0x00;
     gFlag &= ~_BV(IMD_STATUS_FLAG)
     CAN_transmit(MOB_BROADCAST,
                 CAN_ID_PANIC,
                 CAN_LEN_PANIC,
                 gCANMessage);
  }

  // Check MainTS_STATUS
  if (check_MainTSConn()){
    if (!(_BV(MainTS_SD_FLAG) & gFlag)){
      /*If the MainTS_SD_FLAG is not set in gFlag, a TRUE check_HVD()
      denotes a chage in HVD Sense status*/
      // Set MainTS_SD_FLAG bit HIGH in global flag
      gFlag |= _BV(MainTS_SD_FLAG);
      gCANMessage[MainTSConn] = 0xFF;
    }
  }else{
    if ((_BV(MainTS_SD_FLAG) & gFlag)){
      /*If the MainTS_SD_FLAG is set in gFlag, a FALSE check_HVD()
      denotes a chage in HVD Sense status*/
      // Set MainTS_SD_FLAG bit LOW in global flag
      gFlag &= ~_BV(MainTS_SD_FLAG);
      gCANMessage[MainTSConn] = 0x00;
    }
  }

  if (check_BMS()){
    if (!(_BV(BMS_SD_FLAG) & gFlag)){
      /*If the BMS_SD_FLAG is not set in gFlag, a TRUE check_BMS()
      denotes a chage in BMS Sense status*/
      // Set BMS_SD_FLAG bit HIGH in global flag
      gFlag |= _BV(BMS_SD_FLAG);
      gCANMessage[BMS] = 0xFF;

    }
  }else{
    if ((_BV(BMS_SD_FLAG) & gFlag)){
      // If the BMS_SD_FLAG is set in gFlag, a FALSE check_BMS()
      // denotes a chage in BMS Sense status

      // Set BMS_SD_FLAG bit LOW in global flag
      gFlag &= ~_BV(BMS_SD_FLAG);
      gCANMessage[BMS] = 0x00;
    }
  }

  if (check_IMD()){
    if (!(_BV(IMD_SD_FLAG) & gFlag)){
      /*If the MainTS_SD_FLAG is not set in gFlag, a TRUE check_HVD()
      denotes a chage in HVD Sense status*/
      // Set MainTS_SD_FLAG bit HIGH in global flag
      gFlag |= _BV(IMD_SD_FLAG);
      gCANMessage[IMD] = 0xFF;
    }
  }else{
    if ((_BV(IMD_SD_FLAG) & gFlag)){
      /*If the MainTS_SD_FLAG is set in gFlag, a FALSE check_HVD()
      denotes a chage in HVD Sense status*/

      // Set MainTS_SD_FLAG bit LOW in global flag
      gFlag &= ~_BV(IMD_SD_FLAG);
      gCANMessage[IMD] = 0x00;
    }
  }
}


ISR(PCI1_vect){
  if(!BMS_status()){
    gCANMessage[BMS_Status] = 0x00;
    gFlag &= ~_BV(BMS_STATUS_FLAG);
  }else{
    gCANMessage[BMS_Status] = 0xFF;
    gFlag |= _BV(BMS_STATUS_FLAG);
  }
  CAN_transmit(MOB_BROADCAST,
              CAN_ID_PANIC,
              CAN_LEN_PANIC,
              gCANMessage);
}


ISR(PCI0_vect){
  /*When Pin Change Interrupt 1 is triggered, check the status on
  ConnToHVD*/
  if (check_HVD()){
    if (!(_BV(HVD_SD_FLAG) & gFlag)){
      /*If the HVD_SD_FLAG is not set in gFlag, a TRUE check_HVD()
      denotes a chage in HVD Sense status*/

      // Set HVD_SD_FLAG bit high in global flag
      gFlag |= _BV(HVD_SD_FLAG)
      gCANMessage[HVD] = 0xFF;
    }
  }else{
    if ((_BV(HVD_SD_FLAG) & gFlag)){
      /*If the HVD_SD_FLAG is set in gFlag, a FALSE check_HVD()
      denotes a chage in HVD Sense status*/

      // Set HVD_SD_FLAG bit high in global flag
      gFlag &= ~_BV(HVD_SD_FLAG)
      gCANMessage[HVD] = 0x00;
    }
  }
}


ISR(TIMER0_OVF_vect){
  unsigned long f = timer0_frac;
  unsigned long m = timer0_millis;
  m += MILLIS_INCREMENT;
  f += FRAC_STEP;
  if(f>FRAC_MAX){
    f -= FRAC_MAX;
    m += MILLIS_INCR EMENT;
  }
  timer0_frac = f;
  timer0_millis = m;
  timer0_overflow_num++;

  if(clock_prescale>20) {
      gTimerFlag |= _BV(UPDATE_STATUS);
      clock_prescale = 0;
  }
  clock_prescale ++;
}

// CAN MESSAGE INTERRUPT
#define MOB_BRAKE_LIGHT 0
#define MOB_IMD_STATUS 1
#define MOB_BROADCAST 2

ISR(CAN_INT_vect) {
  // Check first board (Dashboard)

  /*** Check Brake Light First ***/
  // Turn to the Brake Light MOB
  CANPAGE = MOB_BRAKE_LIGHT << MOBNB0;
  if (bit_is_set(CANSTMOB, RXOK)){
    volatile uint8_t msg = CANMSG;

    if (msg == 0xFF){
      gFlag |= _BV(PRECHARGE);
    }else{
      // TODO: sample, not actual functionality
      gFlag &= ~_BV(PRECHARGE);
    }
  }

  //Reset status
  CANSTMOB = 0x00;
  CAN_wait_on_receive(MOB_BRAKELIGHT,
                      CAN_ID_BRAKE_LIGHT,
                      CAN_LEN_BRAKE_LIGHT,
                      CAN_IDM_single);
}



/*----- MAIN -----*/
int main(void){
  time = milis();
  checking_time = 0;
  precharge_start_status = 0;
  DDRB |= _BV(LED1) | _BV(LED2); //Makes PB0 and PB1 as output
  // DDRB &= ~(_BV(PIN_SenseConnToHVD)) //SET SenseConnToHVD as input
  // DDRC |= _BV(PRECHARGE); //SET Precharge as output
  // DDRC &= ~(_BV(AIR_Weld_Detect));
  DDRD &= ~(_BV(PIN_SenseIMD)) & ~(_BV(PIN_SenseBMS)) & ~(_BV(PIN_SenseMainTSConn)); //
  // uint8_t old_SenseBMS = PIND & _BV(PIN_SenseBMS);
  uint8_t current_SenseBMS;
  // uint8_t old_SenseIMD = PIND & _BV(PIN_SenseIMD);
  uint8_t current_SenseIMD;

  DDRC |= _BV(PC5) // Make PC5 as output
  while(1){
    // enable pin change interrupt for IMD after 2s delay
    if (!checking_time){
      if(milis() - time > 2000){
        PCMSK2 |= _BV(PCINT21);
        checking_time = 1;
      }
    }

    // low side drive precharge
    low_side_drive_precharge()

    //update BMS status
    current_SenseBMS = (PIND & _BV(PIN_SenseBMS));


    //update IMD status
    current_SenseIMD = (PIND & _BV(PIN_SenseIMD));

    // old_SenseBMS = current_SenseBMS;

    if (PIND & _BV(PIN_SenseBMS)){
      PORT_LED1 ^= _BV(LED1);
      _delay_ms(100);
    }
    if(bit_is_set(gFlag2, UPDATE_STATUS)){
      gFlag2 &= ~_BV(UPDATE_STATUS);
      CAN_transmit(MOB_BROADCAST,
                  CAN_ID_AIR_CONTROL,
                  CAN_LEN_AIR_CONTROL,
                  gCANMessage);
    }
  }
}
