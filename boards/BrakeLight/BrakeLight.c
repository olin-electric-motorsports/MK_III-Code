/*
Header:
    This board manages the brake light, 6 sense modules, and the brake
    position. It sends sense statuses over CAN as well as the brake position
    every certain amount of time.
Author:
    @author Peter Seger & Vienna Scheyer
*/

/*----- Includes -----*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "can_api.h"
// #include "brake_light_32.c"

/*----- Macro Definitions -----*/
/* Brake */
#define BRAKE_PIN           PB5
#define PORT_BRAKE          PORTB
#define PIN_BRAKE           PINB
#define ANALOG_BRAKE_PIN    PB7
#define ANALOG_BRAKE_PORT   PORTB

/* BSPD Status Output */
#define BSPD_STATUS_PIN         PC1 //TODO
#define BSPD_STATUS_PORT        PORTC //TODO

/* Sense Lines */
#define SD_MAIN_FUSE		PB0
#define SD_LEFT_E_STOP		PB1
#define SD_RIGHT_E_STOP		PD5
#define SD_BSPD     		PD6
#define SD_HVD      		PD7
#define SD_TSMS     		PB2

#define PORT_MAIN_FUSE      PORTB
#define PORT_LEFT_E_STOP    PORTB
#define PORT_RIGHT_E_STOP   PORTD
#define PORT_BSPD           PORTD
#define PORT_HVD            PORTD
#define PORT_TSMS           PORTB

#define PIN_MAIN_FUSE      PINB
#define PIN_LEFT_E_STOP    PINB
#define PIN_RIGHT_E_STOP   PIND
#define PIN_BSPD           PIND
#define PIN_HVD            PIND
#define PIN_TSMS           PINB

/* CAN Positions */
#define CAN_BRAKE           1
#define CAN_BRAKE_POS       0
#define CAN_BSPD            2
#define CAN_HVD             3
#define CAN_TSMS            4
#define CAN_LEFT_E_STOP     5
#define CAN_RIGHT_E_STOP    6
#define CAN_MAIN_FUSE       7

#define BROADCAST_MOb       0


/* Sense LEDs */
// Might be irrelevant because the gStatusBar
#define EXT_LED_GREEN           PD0 //(Debug LED on RJ45)
#define EXT_LED_ORANGE          PC0 //(Debug LED on RJ45)
#define LED1                    PC5 //(Purpose - on LED bar)
#define LED2                    PC4 //(Purpose - on LED bar)

#define PORT_EXT_LED_GREEN      PORTD //(Debug LED on RJ45)
#define PORT_EXT_LED_ORANGE     PORTC //(Debug LED on RJ45)
#define PORT_LED1               PORTC //(Purpose - on LED bar)
#define PORT_LED2               PORTC //(Purpose - on LED bar)


//gFlag Positions
#define STATUS_MAIN_FUSE    1
#define STATUS_LEFT_E_STOP  2
#define STATUS_RIGHT_E_STOP 3
#define STATUS_BSPD         4
#define STATUS_HVD          5
#define STATUS_TSMS         6
#define STATUS_BRAKE        7

//gTimerFlag Positions
#define UPDATE_STATUS   0       // Determines when to send messages
#define SEND_BRAKE      1       // Determines when to send brake positions



/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x01;          // Global Flag
volatile uint8_t gTimerFlag = 0x01;     // Timer flag
uint8_t gCAN_MSG[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Message
uint8_t gBrake_Pressure[2] = {0, 0}; //brake pressure value

volatile uint8_t gTSMS = 0x00;
volatile uint8_t gTSMS_OLD = 0x00;  // Used for comparison

// Timer counters
uint8_t clock_prescale = 0x00;  // Used for update timer
uint8_t brake_timer = 0x00;     // Used for brake timer

// Brake POS mapping Values
uint8_t brake_HIGH = 0xE7;       //TODO change with actual values
uint8_t brake_LOW = 0xD3;        //TODO change with actual values



/*----- Interrupt(s) -----*/
// 8-bit Timer
ISR(TIMER0_COMPA_vect) {
    // Only send CAN msgs every 20 cycles
    if(clock_prescale > 40) {
        gTimerFlag |= _BV(UPDATE_STATUS);
        clock_prescale = 0;
    }
    clock_prescale++;

    // Only send a brake message every 40 cycles
    if(brake_timer > 40) {
        gTimerFlag |= _BV(SEND_BRAKE);
        brake_timer = 0;
    }
    brake_timer++;
}

// CAN
ISR(CAN_INT_vect) {
    /*
    Currently not reading CAN messages
    */
}

ISR(PCINT0_vect) {
    /*
    Standard Pin Change Interupt
    covers interupts 0-3
    Interupts covered: Main Shutdown Fuse, Left E-Stop, TSMS, & Brake
    */
    if(bit_is_set(PIN_MAIN_FUSE, SD_MAIN_FUSE)) {
        gFlag |= _BV(STATUS_MAIN_FUSE);
    } else {
        gFlag &= ~_BV(STATUS_MAIN_FUSE);
    }

    if(bit_is_set(PIN_LEFT_E_STOP, SD_LEFT_E_STOP)) {
        gFlag |= _BV(STATUS_LEFT_E_STOP);
    } else {
        gFlag &= ~_BV(STATUS_LEFT_E_STOP);
    }

    if(bit_is_set(PIN_TSMS, SD_TSMS)) {
        gTSMS = PORT_TSMS & SD_TSMS;
        gFlag |= _BV(STATUS_TSMS);
    } else {
        gFlag &= ~_BV(STATUS_TSMS);
    }

    if(bit_is_set(PIN_BRAKE, BRAKE_PIN)) {
        gFlag |= _BV(STATUS_BRAKE);
    } else {
        gFlag &= ~_BV(STATUS_BRAKE);
    }
}

ISR(PCINT2_vect) {
    /*
    Standard Pin Change Interupt
    covers interupts 21-23
    Interupts covered: Right E-Stop, BSPD, HVD
    */
    if(bit_is_set(PIN_RIGHT_E_STOP, SD_RIGHT_E_STOP)) {
        gFlag |= _BV(STATUS_RIGHT_E_STOP);
    } else {
        gFlag &= ~_BV(STATUS_RIGHT_E_STOP);
    }

    if(bit_is_set(PIN_BSPD, SD_BSPD)) {
        gFlag |= _BV(STATUS_BSPD);
    } else {
        gFlag &= -_BV(STATUS_BSPD);
    }

    if(bit_is_set(PIN_HVD, SD_HVD)) {
        gFlag |= _BV(STATUS_HVD);
    } else {
        gFlag &= -_BV(STATUS_HVD);
    }
}


/*----- Functions -----*/
void initTimer_8bit(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0xFF;
}

static inline void updateStateFromFlags(void) {

    /* Build CAN Message */
    if(bit_is_set(gFlag, STATUS_MAIN_FUSE)) {
        gCAN_MSG[CAN_MAIN_FUSE] = 0xFF;     // Electrical signal is low (meaning fuse is set)
    } else {
        gCAN_MSG[CAN_MAIN_FUSE] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_LEFT_E_STOP)) {
        gCAN_MSG[CAN_LEFT_E_STOP] = 0xFF;
    } else {
        gCAN_MSG[CAN_LEFT_E_STOP] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_RIGHT_E_STOP)) {
        gCAN_MSG[CAN_RIGHT_E_STOP] = 0xFF;
    } else {
        gCAN_MSG[CAN_RIGHT_E_STOP] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_BSPD)) {
        // Check if fault
        if(bit_is_clear(BSPD_STATUS_PORT, BSPD_STATUS_PIN)) {
            gCAN_MSG[0] = 0xFF;
            // Send Global Panic
            CAN_transmit(4, CAN_ID_PANIC,
                CAN_LEN_PANIC, gCAN_MSG);
            gCAN_MSG[CAN_BSPD] = 0xFF;
        }
    } else {
        gCAN_MSG[CAN_BSPD] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_HVD)) {
        gCAN_MSG[CAN_HVD] = 0xFF;
    } else {
        gCAN_MSG[CAN_HVD] = 0x00;
    }

    if(bit_is_set(gFlag, STATUS_TSMS)) {
        if(gTSMS != gTSMS_OLD) {
            gCAN_MSG[CAN_TSMS] = 0xFF;
            gTSMS_OLD = gTSMS;
        }
    }

    if(bit_is_set(gFlag, STATUS_BRAKE)) {
        gCAN_MSG[CAN_BRAKE] = 0xFF;
        PORT_LED2 |= _BV(LED2);
    } else {
        gCAN_MSG[CAN_BRAKE] = 0x00;
        PORT_LED2 &= ~_BV(LED2);
    }

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


static inline void mapBrakePressure() {
    /* This function polls the brake position and maps it to
        a byte for sending over CAN. In range [0x00, 0xFF] */
     ADMUX = _BV(REFS0);
     ADMUX |= 4; //pin is also known as ADC4
     ADCSRA |= _BV(ADSC);
     loop_until_bit_is_clear(ADCSRA, ADSC);
     uint16_t brakePressureRaw = ADC >> 4;
     // uint8_t bpMSB, bpLSB;

     gBrake_Pressure[0] = (uint8_t)(brakePressureRaw >> 8);
     gBrake_Pressure[1] = (uint8_t)brakePressureRaw;

     // // Check for brake analog fault
     // if(brake_pos_adc == 0) {
     //     gCAN_MSG[0] = 0xFF;
     //     // Send Global Panic
     //     CAN_transmit(BROADCAST_MOb, CAN_ID_PANIC,
     //         CAN_LEN_PANIC, gCAN_MSG);
     //}

     // if(bit_is_set(gTimerFlag, SEND_BRAKE)) {
     //     uint8_t mapped = ((brake_pos_adc - brake_LOW) * 0xFF) / (brake_HIGH - brake_LOW);
     //     gCAN_MSG[CAN_BRAKE_POS] = mapped;
     //}
}


/*----- MAIN -----*/
int main(void){
    /*
    -Set up Interrupts
    -Set up CAN timer
    -While
        -Every 20 timer cycles, send flag status (20 * 65.535 ms = 1.31 s )
        -Every 40 timer cycles, send brake position (40 * 65.535 ms = 2.62 s)
    */
    sei();                              // Enable interrupts
    CAN_init(CAN_ENABLED);
    // initADC();

    DDRC |= _BV(LED1) | _BV(LED2) | _BV(EXT_LED_ORANGE);
    DDRD |= _BV(EXT_LED_GREEN) | _BV(PD3);

    // PORTD &= ~_BV(PD3);

    /* Setup interrupt registers */
    PCICR |= _BV(PCIE0) | _BV(PCIE2);
    PCMSK0 |= _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT5);      // Covers Pins: Main Fuse, Left E-Stop, TSMS, & Brake Light
    PCMSK2 |= _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23);   // Covers Pins: Right E-Stop, BSPD, and HVD

    initTimer_8bit();                       // Begin 8-bit timer
    gTimerFlag |= _BV(UPDATE_STATUS);        // Read ports



    while(1) {
        // PORT_LED1 |= _BV(LED1);
        if(bit_is_set(gTimerFlag, UPDATE_STATUS)) {
            PORT_LED1 ^= _BV(LED1);     // Blink Orange LED for timing check
            PORT_EXT_LED_ORANGE ^= _BV(EXT_LED_ORANGE);

            updateStateFromFlags();     // Build CAN message based off flags
            gTimerFlag &= ~_BV(UPDATE_STATUS);  // Clear Flag

            mapBrakePressure();
            // Send CAN message
            CAN_transmit(5, CAN_ID_BRAKE_LIGHT,
                CAN_LEN_BRAKE_LIGHT, gCAN_MSG);

            CAN_transmit(5, CAN_ID_BRAKE_PRESSURE, CAN_LEN_BRAKE_PRESSURE,
                gBrake_Pressure);



            // send_LED_bar();

        }
    }
}
