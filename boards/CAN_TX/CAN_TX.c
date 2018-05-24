/* CAN Tx */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "can_api.h"

#define PORT_LED        PORTC
#define LED             PC5
#define UPDATE_STATUS   0

volatile uint8_t gFlag = 0x01;          // Global Flag

uint8_t gClock_prescale = 0x00;  // Used for update timer



void initTimer_8bit(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0xFF;
}

ISR(TIMER0_COMPA_vect) {
    // Only send CAN msgs every 20 cycles
    if(gClock_prescale > 20) {
        gFlag |= _BV(UPDATE_STATUS);
        gClock_prescale = 0;
    }
    gClock_prescale++;
}


int main (void) {
    /* Most basic CAN transmission.
     * Sends the bytes 0x11, 0x66 and
     * 0x0a and then quits. */

     sei();

    // Initialize CAN
    CAN_init(CAN_ENABLED);
    gFlag |= _BV(UPDATE_STATUS);        // Read ports

    // Set the array msg to contain 3 bytes
    uint8_t msg[CAN_LEN_GLOBAL] = { 0xC1, 0x66, 0x0a };

    DDRC |= _BV(LED);


    initTimer_8bit();

    while(1) {
        if(bit_is_set(gFlag, UPDATE_STATUS)) {
            // Transmit message
            CAN_transmit( 0, CAN_ID_GLOBAL, CAN_LEN_GLOBAL, msg );
            PORT_LED ^= _BV(LED);

            gFlag &= ~_BV(UPDATE_STATUS);
        }
    }
}
