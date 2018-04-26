/*
Header:
    This file contains a function to control the off board LED bar
Author:
    @author Peter Seger & Vienna Scheyer
*/

/*----- Includes -----*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define PORT_MAIN_FUSE      PORTB
#define PORT_LEFT_E_STOP    PORTB
#define PORT_RIGHT_E_STOP   PORTD
#define PORT_BSPD           PORTD
#define PORT_HVD            PORTD
#define PORT_TSMS           PORTB

#define STATUS_MAIN_FUSE    1
#define STATUS_LEFT_E_STOP  2
#define STATUS_RIGHT_E_STOP 3
#define STATUS_BSPD         4
#define STATUS_HVD          5
#define STATUS_TSMS         6

#define LED0_PIN            PB2
#define LED1_PIN            PC3
#define LED2_PIN            PC2
#define LED3_PIN            PD7

#define LED0_PORT           PORTB
#define LED1_PORT           PORTC
#define LED2_PORT           PORTC
#define LED3_PORT           PORTD

// call this function from the main brake_light.c file

void send_LED_bar() {

    if (bit_is_set(gFLAG, STATUS_MAIN_FUSE)) {
        LED0_PORT &= ~_BV(LED0_PIN);
        LED1_PORT &= ~_BV(LED1_PIN);
        LED2_PORT &= ~_BV(LED2_PIN);
        LED3_PORT &= ~_BV(LED3_PIN);
    }
    if (bit_is_set(gFLAG, STATUS_LEFT_E_STOP)) {
        LED0_PORT &= ~_BV(LED0_PIN);
        LED1_PORT &= ~_BV(LED1_PIN);
        LED2_PORT &= ~_BV(LED2_PIN);
        LED3_PORT &= _BV(LED3_PIN);
    }
    if (bit_is_set(gFLAG, STATUS_RIGHT_E_STOP)) {
        LED0_PORT &= ~_BV(LED0_PIN);
        LED1_PORT &= ~_BV(LED1_PIN);
        LED2_PORT &= _BV(LED2_PIN);
        LED3_PORT &= ~_BV(LED3_PIN);
    }
    if (bit_is_set(gFLAG, STATUS_BSPD)) {
        LED0_PORT &= ~_BV(LED0_PIN);
        LED1_PORT &= _BV(LED1_PIN);
        LED2_PORT &= ~_BV(LED2_PIN);
        LED3_PORT &= ~_BV(LED3_PIN);
    }
    if (bit_is_set(gFLAG, STATUS_HVD)) {
        LED0_PORT &= _BV(LED0_PIN);
        LED1_PORT &= ~_BV(LED1_PIN);
        LED2_PORT &= ~_BV(LED2_PIN);
        LED3_PORT &= ~_BV(LED3_PIN);
    }
    if (bit_is_set(gFLAG, STATUS_TSMS)) {
        LED0_PORT &= ~_BV(LED0_PIN);
        LED1_PORT &= ~_BV(LED1_PIN);
        LED2_PORT &= _BV(LED2_PIN);
        LED3_PORT &= _BV(LED3_PIN);
    }

}
