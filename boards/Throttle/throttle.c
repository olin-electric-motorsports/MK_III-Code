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
#include "can_api.h"
#include "log_uart.h"

/*----- Macro Definitions -----*/
/* Shutdown */
#define GLOBAL_SHUTDOWN         0x0

/* Throttle */
#define THROTTLE_1              PC4 //Throttle left
#define THROTTLE_2              PC5 //Throttle right
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
#define FLAG_THROTTLE_BRAKE     1
#define FLAG_MOTOR_ON           2
#define FLAG_INERTIA            3
#define FLAG_ESTOP              4
#define FLAG_BOTS               5
#define FLAG_THROTTLE_10        6
#define FLAG_PANIC              7

/* MOBs */
// Mesage OBjects, or mailboxes?
#define MOB_BRAKELIGHT          0
#define MOB_AIR_CONTROL         1
#define MOB_DASHBOARD           2

#define MOB_BROADCAST           3
#define MOB_MOTORCONTROLLER     4

// for gTimerFlag
#define UPDATE_STATUS           0
#define IMPLAUSIBILITY_ERROR    1

//TODO
/*ATmega must:
-Sense 3 shutdown Lines (check)
-Read and send steering pot value (check)
-Read both throttle pots (check)
-Map out both throttle pots (check)
-Send out throttle value over CAN (check)
-Wait on RTD and trigger it (check)
    -I used a pause for that, update state from flags function
RULES IC 1.13 pg87 of 2017-2018 FSAE rulebook
*/

/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x00;  // Global Flag
volatile uint8_t gTimerFlag = 0x01; // Timer Flag

uint8_t gThrottle[2] = {0x00,0x00};
uint16_t gThrottle16[2] = {0x00,0x00};
uint8_t gThrottleSmoothed = 0x00;
uint8_t gThrottleThreshold = 0xA6;// used for troubleshooting
uint8_t gSteering = 0x00;
uint8_t gSteeringThreshold = 0x7F;// used for troubleshooting
#define gAvg                    8//options are: 4,8,16,32
//gAvg is the number of values from the ADC it uses to average the
//throttle value for. so 8 would mean 8 values it averages for one
//throttle value. Would reccomend 32 for drive days and 8 for competition
//-Corey, May 9th

// CAN Message
uint8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// CAN Message for motor controller
uint8_t gCANMotorController[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Throttle mapping values
// NEEDS TO BE SET ACCORDING TO READ VALUES AFTER CENTERING
//Values set last on May 9th by Corey
uint16_t throttle1_HIGH = 0xA0;//160
uint16_t throttle1_LOW = 0x0c;
uint16_t throttle2_HIGH = 0xA0;//160
uint16_t throttle2_LOW = 0x06;

uint8_t THROTTLE_MAX_ADJUST_AMOUNT = 20;

uint16_t throttle_10_count = 0x00;

/*----- Timer Counters ----- */
uint8_t clock_prescale = 0x00;
uint8_t timer_counter = 0x00;
uint32_t imp_error = 0x00;
int buzzerSet = 0;

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
            gFlag |= _BV(FLAG_THROTTLE_BRAKE);
        } else {
           gFlag &= ~_BV(FLAG_BRAKE);
        }

        CANSTMOB = 0x00;
        CAN_wait_on_receive(MOB_BRAKELIGHT,
                            CAN_ID_BRAKE_LIGHT,
                            CAN_LEN_BRAKE_LIGHT,
                            CAN_IDM_single);
    }

    //Start button
    CANPAGE = (MOB_DASHBOARD << MOBNB0);
    if (bit_is_set(CANSTMOB,RXOK)) {
        volatile int8_t msg = CANMSG;

        if(msg == 0xFF){
            gFlag |= _BV(FLAG_MOTOR_ON);
        } else {
            // gFlag &= ~_BV(FLAG_MOTOR_ON);
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
    PCINT0 covers pins PCINT0-PCINT7
    Writes value to PINx
    */
    if(bit_is_set(PINB,SD_INERTIA)){
        gFlag |= _BV(FLAG_INERTIA);
    } else {
        gFlag &= ~_BV(FLAG_INERTIA);
    }

    if(bit_is_set(PINB,SD_ESTOP)){
        gFlag |= _BV(FLAG_ESTOP);
    } else {
        gFlag &= ~_BV(FLAG_ESTOP);
    }

    if(bit_is_set(PINB,SD_BOTS)){
        gFlag |= _BV(FLAG_BOTS);
    } else {
        gFlag &= ~_BV(FLAG_BOTS);
    }


}

ISR(TIMER0_COMPA_vect) {
    /*
    Timer/Counter0 compare match A
    */

    clock_prescale ++;
    if(clock_prescale>0) {
        gTimerFlag |= _BV(UPDATE_STATUS);
        clock_prescale = 0;
    }

    if(bit_is_set(gFlag,FLAG_THROTTLE_10)){
        imp_error++;
        // 14Hz *.1 = 4 cycles
        if(imp_error > 5){
            gFlag |= _BV(FLAG_PANIC);
        }
    } else {
        imp_error = 0;
    }


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
    if(bit_is_set(gFlag,FLAG_ESTOP)){
        gCANMessage[CAN_DRIVER_E_STOP] = 0xFF;
    } else {
        gCANMessage[CAN_DRIVER_E_STOP] = 0x00;
    }
    if(bit_is_set(gFlag,FLAG_INERTIA)) {
        gCANMessage[CAN_INTERTIA]= 0xFF;
    } else {
        gCANMessage[CAN_INTERTIA] = 0x00;
    }
    if(bit_is_set(gFlag,FLAG_BOTS)){
        gCANMessage[CAN_BOTS] = 0xFF;
        // GLOBAL_SHUTDOWN = 0xFF;
    } else {
        gCANMessage[CAN_BOTS] = 0x00;
    }
}

void updateStateFromFlags(void) {
    /*
    Based off the state of the flag(s), update components and send CAN
    */

    //Based off of ready to drive sound rules (pg113)
    if(bit_is_set(gFlag,FLAG_MOTOR_ON) && buzzerSet == 0){
        RTD_PORT |= _BV(RTD_LD);
        _delay_ms(3000);
        RTD_PORT &= ~(_BV(RTD_LD));
        buzzerSet = 1;
    } else {
        RTD_PORT &= ~_BV(RTD_LD);
    }

    if(bit_is_set(gFlag,FLAG_PANIC)){
        gThrottle[0] = 0x00;
        gThrottle[1] = 0x00;
        LED1_PORT |= _BV(LED1);
        LED2_PORT |= _BV(LED2);
        LED3_PORT |= _BV(LED3);

        //DO MORE
    }



}

void testInputs(int test) {
    /* For troubleshooting sensors
       Use this to test each sensor, they should all
       have pull up resistors which means a disconnect
       would result in a 5V reading and therefore the LEDs
       should turn on if they've been initiated correctly
       test == 1 checks the pots on the car.
       test == 2 checks the interrpt flags for the shutdown circuit
       test == 3 checks if the throttle is 10% to one another

    */


    /*--- Set LED's on if > 50% ---*/
    /*--- Pull ups' on all three ---*/
    if(test == 1){
        // LOG_chr(gThrottle[0]);
        // LOG_chr(gThrottle[1]);
        char disp_string[64];
        // char static_msg1[] = "Throttle Left";
        // char static_msg2[] = "Throttle Right";
        // LOG_println(static_msg1,strlen(static_msg1))
        sprintf(disp_string,"Throttle left is %d, Throttle right is %d",gThrottle[0],gThrottle[1]);
        LOG_println(disp_string,strlen(disp_string));

        if(gThrottle[1] > gThrottleThreshold){
            LED2_PORT |= _BV(LED2);
        } else {
            LED2_PORT &= ~_BV(LED2);
        }

        if(gThrottle[0] > gThrottleThreshold){
            LED3_PORT |= _BV(LED3);
        } else {
            LED3_PORT &= ~(_BV(LED3));
        }

        if(gSteering > gSteeringThreshold){
            LED1_PORT |= _BV(LED1);
        } else {
            LED1_PORT &= ~(_BV(LED1));
        }
    }

    /*--- Test shutdown circuit flags ---*/
    // Uses the PCINT vectors
    if(test == 2){

        if(bit_is_set(gFlag,FLAG_ESTOP)){
            LED2_PORT |= _BV(LED2);
        } else {
            LED2_PORT &= ~_BV(LED2);
        }
        if(bit_is_set(gFlag,FLAG_INERTIA)) {
            LED3_PORT |= _BV(LED3);
        } else {
            LED3_PORT &= ~_BV(LED3);
        }
        if(bit_is_set(gFlag,FLAG_BOTS)){
            LED1_PORT |= _BV(LED1);
        } else {
            LED1_PORT &= ~_BV(LED1);
        }
    }

    /*--- Test 10% throttle pots --*/
    // Uses global flag for throttle_10
    if(test == 3){

        if(bit_is_set(gFlag,FLAG_THROTTLE_10)){
            LED1_PORT |= _BV(LED1);
            LED2_PORT |= _BV(LED2);
            LED3_PORT |= _BV(LED3);

        } else {
            LED1_PORT &= ~_BV(LED1);
            LED2_PORT &= ~_BV(LED2);
            LED3_PORT &= ~_BV(LED3);
        }
    }




}

void readPots(void) {
    /* Read values from ADC and store them
       in their appropriate variables
       Reads: throttle1,throttle2, and steering
    */

    ADMUX = _BV(REFS0);
    ADMUX |= 8; //pin is also known as ADC8
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint16_t throttle1 = ADC;

    ADMUX = _BV(REFS0);
    ADMUX |= 9;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint16_t throttle2 = ADC;


    ADMUX = _BV(REFS0);
    ADMUX |= 2;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint8_t steering = (uint8_t) (ADC >> 2);

    // char disp_string[64];
    // sprintf(disp_string,"Throttle left is %d, Throttle right is %d",throttle1,throttle2);
    // LOG_println(disp_string,strlen(disp_string));
    //
    // LED1_PORT ^= _BV(LED1);

    gThrottle16[0] = throttle1;
    gThrottle16[1] = throttle2;
    gSteering = steering;
}

void mapAndStoreThrottle(void){
    // we store a 10-bit value into a 32-bits so that the atmega can have enough room to
    // do the math necessary to accomodate for the amplification
    uint32_t throttle1 = gThrottle16[0];
    uint32_t throttle2 = gThrottle16[1];

    if(throttle1 > 900 || throttle2 > 900){
        gFlag |= _BV(FLAG_PANIC);
        return;
    }

    // Adjust for amplification
    uint8_t throttle1_centered = throttle1 >> 2;
    uint8_t throttle2_centered = ((throttle2 * 100)/122) >> 2;

    if(throttle1_centered > throttle1_HIGH && throttle1_centered < throttle1_HIGH + THROTTLE_MAX_ADJUST_AMOUNT){
        throttle1_centered = throttle1_HIGH;
    } else if(throttle1_centered  < throttle1_LOW){
        throttle1_centered = throttle1_LOW;
    }
    if(throttle2_centered > throttle2_HIGH && throttle1_centered < throttle2_HIGH + THROTTLE_MAX_ADJUST_AMOUNT){
        throttle2_centered = throttle2_HIGH;
    } else if(throttle2_centered  < throttle2_LOW){
        throttle2_centered = throttle2_LOW;
    }

    //Map between 0 and 255
    uint8_t throttle1_mapped = ((throttle1_centered - throttle1_LOW) * 0xff) / (throttle1_HIGH-throttle1_LOW);
    uint8_t throttle2_mapped = ((throttle2_centered - throttle2_LOW) * 0xff) / (throttle2_HIGH-throttle2_LOW);

    // char disp_string3[64];
    // sprintf(disp_string3,"Throttle1 centered is %d, Throttle2 centered is %d",throttle1_centered,throttle2_centered);
    // LOG_println(disp_string3,strlen(disp_string3));

    // char tmap[64];
    // sprintf(tmap,"Throttle left is %d, Throttle right is %d",throttle1_mapped,throttle2_mapped);
    // LOG_println(tmap,strlen(tmap));

    // Rolling average
    // Only one of below will be flashed during compiling according to gAvg
    // The length of the arrays dictates not only the accuracy but the speed
    // at which the throttle value changes
    #if(gAvg == 4)
        uint8_t scale = 4;
        uint8_t pow2 = 2;
        static uint8_t rolling1[4];
        static uint8_t rolling2[4];
    #elif(gAvg == 8)
        uint8_t scale = 8;
        uint8_t pow2 = 3;
        static uint8_t rolling1[8];
        static uint8_t rolling2[8];
    #elif(gAvg == 16)
        uint8_t scale = 16;
        uint8_t pow2 = 4;
        static uint8_t rolling1[16];
        static uint8_t rolling2[16];
    #else
        uint8_t scale = 32;
        uint8_t pow2 = 5;
        static uint8_t rolling1[32];
        static uint8_t rolling2[32];
    #endif


    for (int i=0; i < scale-1; i++) {
        rolling1[i] = rolling1[i+1];
        rolling2[i] = rolling2[i+1];
    }
    rolling1[scale-1] = throttle1_mapped;
    rolling2[scale-1] = throttle2_mapped;

    long long avg1 = 0;
    long long avg2 = 0;

    for (int i=0; i < scale; i++) {
        avg1 += rolling1[i];
        avg2 += rolling2[i];
    }
    // bit shift to the right since it's the same as dividing by 32 (2^5)
    throttle1_mapped = avg1 >> pow2;
    throttle2_mapped = avg2 >> pow2;

    // char disp_string[64];
    // sprintf(disp_string,"Throttle left AVERAGE is %d, Throttle right AVERAGE is %d",throttle1_mapped,throttle2_mapped);
    // LOG_println(disp_string,strlen(disp_string));

    // Check if they are within 10%
    if (throttle1_mapped > throttle2_mapped && (throttle1_mapped - throttle2_mapped) >= (0xFF/10)) {
        throttle_10_count++;
        gFlag |= _BV(FLAG_THROTTLE_10);
    }
    else if (throttle2_mapped > throttle1_mapped && (throttle2_mapped - throttle1_mapped) >= (0xFF/10)) {
        throttle_10_count++;
        gFlag |= _BV(FLAG_THROTTLE_10);
    } else {
        gFlag &= ~_BV(FLAG_THROTTLE_10);
    }

    if(throttle1_mapped == 0 && bit_is_clear(gFlag,FLAG_BRAKE)){
        gFlag &= ~_BV(FLAG_THROTTLE_BRAKE);
    }

    
    if (bit_is_clear(gFlag, FLAG_BRAKE) && bit_is_clear(gFlag,FLAG_THROTTLE_BRAKE) && bit_is_set(gFlag,FLAG_MOTOR_ON) ) {
        gThrottle[0] = throttle1_mapped;
        gThrottle[1] = throttle2_mapped;
    } else {
        gThrottle[0] = 0x00;
        gThrottle[1] = 0x00;
        // gFlag |= _BV(FLAG_THROTTLE_BRAKE);
    }
    
}

void sendCanMessages(int viewCan){

    gCANMessage[0] = gThrottle[0];
    gCANMessage[1] = gSteering;
    gCANMessage[2] = bit_is_set(gFlag,FLAG_BOTS) ? 0xFF : 0x00;
    gCANMessage[3] = bit_is_set(gFlag,FLAG_INERTIA) ? 0xFF : 0x00;
    gCANMessage[4] = bit_is_set(gFlag,FLAG_ESTOP) ? 0xFF : 0x00;

    CAN_transmit(MOB_BROADCAST,
                 CAN_ID_THROTTLE,
                 CAN_LEN_THROTTLE,
                 gCANMessage);

    // Send out Motor controller info
    // REMAP
    uint16_t thrott = (uint16_t) gThrottle[0];
    //uint16_t thrott = (uint16_t) gthrottlesmoothed;
    uint16_t mc_remap = (uint16_t)(thrott * 9);
    gCANMotorController[0] = mc_remap;
    gCANMotorController[1] = mc_remap >> 8;
    gCANMotorController[2] = 0x00;
    gCANMotorController[3] = 0x00;
    gCANMotorController[4] = 1;
    gCANMotorController[5] = bit_is_set(gFlag,FLAG_MOTOR_ON) ? 0x01 : 0x00;
    gCANMotorController[6] = 0x00;
    gCANMotorController[7] = 0x00;

    CAN_transmit(MOB_MOTORCONTROLLER,
                 CAN_ID_MC_COMMAND,
                 CAN_LEN_MC_COMMAND,
                 gCANMotorController);
    if(viewCan){
        char msg1[128];
        char msg2[128];
        sprintf(msg1,"CAN message one to all:\nThrottle:%d\nSteering:%d\nBOTS:%d\nInertia:%d\nEstop:%d",
        gCANMessage[0],gCANMessage[1],gCANMessage[2],gCANMessage[3],gCANMessage[4]);
        LOG_println(msg1,strlen(msg1));
        sprintf(msg2,"CAN message to motorcontroller:\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d",
        gCANMotorController[0],gCANMotorController[1],gCANMotorController[2],gCANMotorController[3],
        gCANMotorController[4],gCANMotorController[5],gCANMotorController[6],gCANMotorController[7]);
        LOG_println(msg2,strlen(msg2));

    }


    // EXT_LED_PORT ^= _BV(EXT_LED2);
}


/*----- MAIN -----*/
int main(void){
    /*
    -Set up I/O (done)
    -Set up CAN timer (done)
    -Initialize external libraries (if applicable)
    -Wait on CAN (hmm)
    -Infinite loop checking shutdown state! (done)
    */
    initTimer();
    initADC();
    sei();
    LOG_init();
    CAN_init(CAN_ENABLED);

    // Set interrupt registers
    PCICR |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT5) | _BV(PCINT6) | _BV(PCINT7);

    // Set pins to output
    DDRC |= _BV(LED1);
    DDRB |= _BV(LED2) | _BV(LED3);
    DDRC |= _BV(RTD_LD);
    DDRB |= _BV(EXT_LED1) | _BV(EXT_LED2);

    // set pull up resistor for steering
    STEERING_PORT |= _BV(STEERING);

    // Commenting this out because it's awful - Hoppe 4/28/18
    // // turn on RTD for .4 seconds...heheh
    // // just for wiring harness, not necessary
    // RTD_PORT |= _BV(RTD_LD);
    // _delay_ms(400);
    // RTD_PORT &= ~(_BV(RTD_LD));
    
    _delay_ms(1000);
    CAN_transmit(MOB_MOTORCONTROLLER,
                 CAN_ID_MC_COMMAND,
                 CAN_LEN_MC_COMMAND,
                 gCANMotorController);

    //In order to enable the motor controller, we must first send a byte disabling it (this is by RMS design to prevent accidental enbaling)
    _delay_ms(1000);
    CAN_transmit(MOB_MOTORCONTROLLER,
                 CAN_ID_MC_COMMAND,
                 CAN_LEN_MC_COMMAND,
                 gCANMotorController);

    CAN_wait_on_receive(MOB_DASHBOARD,
                        CAN_ID_DASHBOARD,
                        CAN_LEN_DASHBOARD,
                        CAN_IDM_single);

    CAN_wait_on_receive(MOB_BRAKELIGHT,
                        CAN_ID_BRAKE_LIGHT,
                        CAN_LEN_BRAKE_LIGHT,
                        CAN_IDM_single);

    while(1){
        if(bit_is_set(gTimerFlag,UPDATE_STATUS)){
            EXT_LED_PORT ^= _BV(EXT_LED1);
            gTimerFlag &= ~_BV(UPDATE_STATUS);

            checkShutdownState();
            readPots();
            testInputs(0);
            mapAndStoreThrottle();
            updateStateFromFlags();

            sendCanMessages(0);
        }
    }
}
