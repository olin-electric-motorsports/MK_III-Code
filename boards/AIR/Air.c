/*
Header:
    This is the firmware code for AIR Control Board.
Author:
    @Josh Deng, Sherie SHen
*/

/*----- Includes -----*/

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"

/*----- Macro Definitions -----*/
// Pin Definitions
#define LED1		PB0
#define LED2		PB1

// External LEDs
#define EXT_LED1	PD0
#define EXT_LED2	PC0

// Shutdown
#define FINAL_SHUTDOWN_RELAY		PD7
#define PIN_FINAL_SHUTDOWN_RELAY	PIND

#define PIN_SenseBMS                PD5
#define PIN_SenseIMD                PD6
#define PIN_SenseConnToHVD          PB2
#define PIN_SenseMainTSConn         PD7
#define IMD_STATUS                  PD1
#define BMS_STATUS                  PC1

// Precharge & AIR
#define PRECHARGE 	                PC4
#define PIN_AIR_LSD		            PC5
#define AIR_Weld_Detect		        PC6

// CAN Message Objects

#define MOB_BRAKE_LIGHT 0
#define MOB_IMD_STATUS 1
#define MOB_BROADCAST 2

/*---- CAN Position Macros ----*/
#define BMS                     0
#define IMD                     1
#define MainTSConn              3
#define HVD                     4
#define BMS_Status              5
#define IMD_Status              6

/*----- GLOBAL FLAGS -----*/
#define HVD_SD_FLAG 	       0
#define BMS_SD_FLAG 	       1
#define IMD_SD_FLAG            2
#define MainTS_SD_FLAG         3
#define IMD_STATUS_FLAG        4
#define BMS_STATUS_FLAG        5
#define AIR_minus_STATUS_FLAG  6
#define Brake_Light            7


#define UPDATE_STATUS_FLAG     0
#define AIR_plus_STATUS_FLAG   0

/* MILLIS implementation definitions */
#define clockCyclesPerMicrosecond ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(X) ( ((X) * 1000L) / (F_CPU / 1000L) )
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRAC_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRAC_MAX (1000 >> 3)


volatile uint8_t gFlag = 0x00;
volatile unsigned long timer0_millis = 0;
volatile unsigned long timer0_frac = 0;
volatile unsigned long timer0_overflow_num = 0;
volatile uint8_t clock_prescale = 0x00;
volatile uint8_t gFlag2 = 0x01; // Timer Flag


unsigned long time = 0;
unsigned long precharge_start_status = 0;
unsigned long precharge_start_time = 0;

uint8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t gTimerFlag = 0x0;

/*----- FUNCTION DEFINITIONS -----*/
void enableInterrupt(){
  sei();  //Enable Global Interrupt
  CAN_init(CAN_ENABLED);       //Enable CAN interrupt

  // Enable Pin Change Interrupt Enable 2
  PCICR |= _BV(PCIE2) | _BV(PCIE1) | _BV(PCIE0);
  // Enable Pin Change Interrupts on PCINT[23:21] (PD[7:5], Sense[BMS, IMD, MainTS])
  // and PCINT17(IMD Status, PD1)
  PCMSK2 = _BV(PCINT23) | _BV(PCINT22);
  // Enable Pin Change Interrupts on PCINT9 (BMS_Staus, PC1)
  PCMSK1 |= _BV(PCINT9);

  // Enable Pin Change Interrupts on PCINT2 (PB2)
  PCMSK0 |= _BV(PCINT2);
}

unsigned long millis(){
    unsigned long m;
    uint8_t oldSREG = SREG;

    // disable interrupts while we read timer0_millis or we might get an
    // inconsistent value (e.g. in the middle of a write to timer0_millis)
    cli();
    m = timer0_millis;
    SREG = oldSREG;
    sei();
    return m;
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
  if (millis() - time <= 2000) {
      return 1; // if it hasn't been long enough yet
  }
  if (PIND & _BV(IMD_STATUS)){
    return 1;
  }else{
    return 0;
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
  TIMSK0 |= _BV(TOIE0);
  // Timer 0 with 64 prescaler if F_CPU == 16,000,000
  TCCR0B |= _BV(CS01) | _BV(CS00);
}

inline void low_side_drive_precharge(int charging_time){
  if (gFlag & _BV(Brake_Light)){
    if (!precharge_start_status){
      precharge_start_time = millis();
    }
    PORTC |= _BV(PRECHARGE);
    precharge_start_status = 1;

    // open relay after charging time
    if(millis() - precharge_start_time > charging_time){
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
ISR(PCINT2_vect){
  /*When Pin Change Interrupt 2 is triggered, check the status on
  BMS, IMD, and MainTSConn; check whether BMS are the same or
  different, if not send message*/

  if(!IMD_status()){
     gCANMessage[IMD_Status] = 0x00;
     gFlag &= ~_BV(IMD_STATUS_FLAG);
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


ISR(PCINT1_vect){
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


ISR(PCINT0_vect){
  /*When Pin Change Interrupt 1 is triggered, check the status on
  ConnToHVD*/
  if (check_HVD()){
    if (!(_BV(HVD_SD_FLAG) & gFlag)){
      /*If the HVD_SD_FLAG is not set in gFlag, a TRUE check_HVD()
      denotes a chage in HVD Sense status*/

      // Set HVD_SD_FLAG bit high in global flag
      gFlag |= _BV(HVD_SD_FLAG);
      gCANMessage[HVD] = 0xFF;
    }
  }else{
    if ((_BV(HVD_SD_FLAG) & gFlag)){
      /*If the HVD_SD_FLAG is set in gFlag, a FALSE check_HVD()
      denotes a chage in HVD Sense status*/

      // Set HVD_SD_FLAG bit high in global flag
      gFlag &= ~_BV(HVD_SD_FLAG);
      gCANMessage[HVD] = 0x00;
    }
  }
}


ISR(TIMER0_OVF_vect){
  unsigned long f = timer0_frac;
  unsigned long m = timer0_millis;
  m += MILLIS_INC;
  f += FRAC_INC;
  if(f>FRAC_MAX){
    f -= FRAC_MAX;
    m += MILLIS_INC;
  }
  timer0_frac = f;
  timer0_millis = m;
  timer0_overflow_num++;

  if(clock_prescale>20) {
      gTimerFlag |= _BV(UPDATE_STATUS_FLAG);
      clock_prescale = 0;
  }
  clock_prescale ++;
}

// CAN MESSAGE INTERRUPT

ISR(CAN_INT_vect) {
  // Check first board (Dashboard)

  /*** Check Brake Light First ***/
  // Turn to the Brake Light Mailbox
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
  CAN_wait_on_receive(MOB_BRAKE_LIGHT,
                      CAN_ID_BRAKE_LIGHT,
                      CAN_LEN_BRAKE_LIGHT,
                      CAN_IDM_single);
}



/*----- MAIN -----*/
int main(void){
  enableInterrupt();
  timer0_setup();
  time = millis();
  uint8_t checking_time = 0;


  DDRB |= _BV(LED1) | _BV(LED2); //Makes PB0 and PB1 as output
  DDRB &= ~(_BV(PIN_SenseConnToHVD));//SET SenseConnToHVD as input
  DDRC |= _BV(PRECHARGE) | _BV(EXT_LED2) | _BV(PIN_AIR_LSD); //SET Precharge as output
  DDRC &= ~(_BV(AIR_Weld_Detect)) | _BV(BMS_STATUS); //SET as input
  DDRD |= _BV(EXT_LED1);    // SET external LED as output
  DDRD &= ~(_BV(PIN_SenseIMD)) & ~(_BV(PIN_SenseBMS)) & ~(_BV(PIN_SenseMainTSConn)) & ~(_BV(PD3)); //SET IMD, BMS, SenseMainTSConn as input

  while(1){
    // enable pin change interrupt for IMD after 2s delay
    if (!checking_time){
      if(millis() - time > 2000){
        PCMSK2 |= _BV(PCINT21);
        checking_time = 1;
      }
    }

    AIR_minus_status();

    // low side drive precharge and AIR+
    low_side_drive_precharge(10000);

    low_side_drive_AIR_plus();

    if(bit_is_set(gFlag2, UPDATE_STATUS_FLAG)){
      gFlag2 &= ~_BV(UPDATE_STATUS_FLAG);
      CAN_transmit(MOB_BROADCAST,
                  CAN_ID_AIR_CONTROL,
                  CAN_LEN_AIR_CONTROL,
                  gCANMessage);
    }
  }
}
