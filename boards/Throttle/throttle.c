/*
Header:
    Code for the throttle-steering-position board located
    in the Dashboard-Left Enclosure
Author:
    @author coreyacl
*/

/*----- Includes -----*/
#include <stdio.h>
// #include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*----- Macro Definitions -----*/
/* Shutdown */
#define GLOBAL_SHUTDOWN         0x0

/* Throttle */
#define THROTTLE_1              PC4
#define THROTTLE_2              PC5
#define THROTTLE_PORT           PORTC

/* Steering */
#define STEERING                PD5
#define STEERING_PORT           PORTD

/* Sense Lines */
#define SD_INERTIA              PB5 //PCINT5
#define SD_ESTOP                PB6 //PCINT6
#define SD_BOTS                 PB7 //PCINT7
#define SD_PORT                 PORTB

/* Ready to Drive */
#define RTD_LD                  PC7
#define RTD_PORT                PORTC

/* LEDs */
#define LED1                    PC6
#define LED1_PORT               PORTC
#define LED2                    PB3
#define LED2_PORT               PORTB
#define LED3                    PB4
#define LED3_PORT               PORTB

#define EXT_LED1                PB0
#define EXT_LED2                PB1
#define EXT_LED_PORT            PORTB

/* CAN Positions */
#define CAN_THROTTLE_POS        0
#define CAN_STEERING_POS        1
#define CAN_BOTS                2
#define CAN_INTERTIA            3
#define CAN_DRIVER_E_STOP       4

/* Flags */
#define FLAG_BRAKE              0
#define FLAG_AIRS               1
#define FLAG_MOTOR_ON           2
#define FLAG_INERTIA            3
#define FLAG_ESTOP              4
#define FLAG_BOTS               5

/* MOBs */
// Mesage OBjects, or mailboxes?
#define MOB_BRAKELIGHT          0
#define MOB_AIR_CONTROL         1
#define MOB_DASHBOARD           2

#define UPDATE_STATUS   0

//TODO
/*ATmega must:
-Sense 3 shutdown Lines
-Read and send steering pot value
-Read both throttle pots
-Send out throttle value over CAN
-Wait on RTD and trigger it
*/

/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x00;  // Global Flag
volatile uint8_t gTimerFlag = 0x01; // Timer Flag

uint8_t gThrottle[2] = {0x00,0x00};
uint8_t gThrottle_smoothed = 0x00;
uint8_t gThrottleThreshold = 0x7F;
uint8_t gSteering = 0x7F;

uint8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Message

/*----- Timer Counters ----- */
uint8_t clock_prescale = 0x00;
uint8_t timer_counter = 0x00;


/*----- Interrupt(s) -----*/
// *pg 76 of datasheet*
// https://github.com/olin-electric-motorsports/MK_II-Code/tree/master/lib
// https://github.com/olin-electric-motorsports/MK_III-Code/tree/master/lib
ISR(CAN_INT_vect) {
    /*
    CAN Interupt
    -Check bits to see if they are set
        -IF they are, set global flag bit position
        -ELSE do nothing
    IMPORTANT, do not perform any 'real' operations in a interupt,
    just set the flag and move on
    */

    // Brakelight
    CANPAGE = (MOB_BRAKELIGHT << MOBNB0);
    if (bit_is_set(CANSTMOB,RXOK)) {
        volatile int8_t msg = CANMSG;

        if(msg == 0xFF){
            gFlag |= _BV(FLAG_BRAKE);
        } else {
            gFlag &= ~_BV(FLAG_BRAKE);
        }

        CANSTMOB = 0x00;
        CAN_wait_on_receive(MOB_BRAKELIGHT,
                            CAN_ID_BRAKE_LIGHT,
                            CAN_LEN_BRAKE_LIGHT,
                            CAN_IDM_single);
    }

    // Air control unnesecary??
    CANPAGE = (MOB_AIR_CONTROL << MOBNB0);
    if (bit_is_set(CANSTMOB,RXOK)) {
        volatile int8_t msg = CANMSG;

        if(msg == 0xFF){
            gFlag |= _BV(FLAG_AIRS);
        } else {
            gFlag &= ~_BV(FLAG_AIRS);
        }

        CANSTMOB = 0x00;
        CAN_wait_on_receive(MOB_AIR_CONTROL,
                            CAN_ID_AIR_CONTROL,
                            CAN_LEN_AIR_CONTROL,
                            CAN_IDM_single);
    }

    //Start button
    CANPAGE = (MOB_DASHBOARD << MOBNB0);
    if (bit_is_set(CANSTMOB,RXOK)) {
        volatile int8_t msg = CANMSG;

        if(msg == 0xFF){
            gFlag |= _BV(FLAG_MOTOR_ON);
        } else {
            gFlag &= ~_BV(FLAG_MOTOR_ON);
        }

        CANSTMOB = 0x00;
        CAN_wait_on_receive(MOB_DASHBOARD,
                            CAN_ID_DASHBOARD,
                            CAN_LEN_DASHBOARD,
                            CAN_IDM_single);
    }

}

ISR(PCINT0_vect) {
    /*
    Standard Pin Change Interupt
    */
    if(bit_is_set(SD_PORT,SD_INERTIA)){
        gFlag |= _BV(FLAG_INERTIA);
    } else {
        gFlag &= ~_BV(FLAG_INERTIA);
    }

    if(bit_is_set(SD_PORT,SD_ESTOP)){
        gFlag |= _BV(FLAG_ESTOP);
    } else {
        gFlag &= ~_BV(FLAG_ESTOP);
    }

    if(bit_is_set(SD_PORT,SD_BOTS)){
        gFlag |= _BV(FLAG_BOTS);
    } else {
        gFlag &= ~_BV(FLAG_BOTS);
    }


}

ISR(TIMER0_COMPA_vect) {
    /*
    Timer/Counter0 compare match A
    */
    if(clock_prescale>20) {
        gTimerFlag |= _BV(UPDATE_STATUS);
        clock_prescale = 0;
    }
    clock_prescale ++;

}


/*----- Functions -----*/
void initTimer(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0xFF;
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

void checkShutdownState(void)   {
    /*
    -Check if bits are set
        -IF they are, set CAN list position to 0xFF
        -ELSE do set CAN list position to 0x00
    */
    if(bit_is_set(gFlag,FLAG_AIRS)){
        GLOBAL_SHUTDOWN = 0xFF;
    }
    if(bit_is_set(gFlag,FLAG_INERTIA)) {
        GLOBAL_SHUTDOWN = 0xFF;
    }
    if(bit_is_set(gFlag,FLAG_BOTS)){
        GLOBAL_SHUTDOWN = 0xFF;
    }
}

void updateStateFromFlags(void) {
    /*
    Based off the state of the flag(s), update components and send CAN
    */
}


void checkSensors(void) {
    /* For troubleshooting sensors
       Use this to test each sensor, they should all
       have pull up resistors which means a disconnect
       would result in a 5V reading and therefore the LEDs
       should turn on if they've been initiated correctly
    */

    /*--- Read values from ADC ---*/

    ADMUX = _BV(REFS0);
    ADMUX |= 8;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint8_t throttle1 = (uint8_t) (ADC >> 2);

    ADMUX = _BV(REFS0);
    ADMUX |= 9;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint8_t throttle2 = (uint8_t) (ADC >> 2);

    ADMUX = _BV(REFS0);
    ADMUX |= 2;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint8_t steering = (uint8_t) (ADC >> 2);

    /*--- Set LED's on if > 50% ---*/
    /*--- Pull ups' on all three ---*/

    if(throttle1 > gThrottleThreshold){
        LED2_PORT |= _BV(LED2);
    } else {
        LED2_PORT &= ~_BV(LED2);
    }

    if(throttle2 > gThrottleThreshold){
        LED3_PORT |= _BV(LED3);
    } else {
        LED3_PORT &= ~(_BV(LED3));
    }

    if(steering > gSteering){
        LED1_PORT |= _BV(LED1);
    } else {
        LED1_PORT &= ~(_BV(LED1));
    }




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
    initTimer();
    initADC();
    sei();

    // Set interrupt registers
    PCIR |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT5) | _BV(PCINT6) | _BV(PCINT7);

    // Set pins to output
    DDRC |= _BV(LED1);
    DDRB |= _BV(LED2);
    DDRB |= _BV(LED3);
    DDRC |= _BV(RTD_LD);

    // set pull up res for steering
    STEERING_PORT |= _BV(STEERING);

    // turn on RTD for .4 seconds
    RTD_PORT |= _BV(RTD_LD);
    _delay_ms(400);
    RTD_PORT &= ~(_BV(RTD_LD));


    while(1){
        checkSensors();
    }

}
