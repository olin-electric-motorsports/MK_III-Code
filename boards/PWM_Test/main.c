/* CAN Rx */
#include <avr/io.h>
#include <util/delay.h>
#include "can_api.h"


int main (void)
{
    TCCR1B |= _BV(CS00); //Clock prescale set to max speed
    TCCR1B |= _BV(WGM12);
    TCCR1A |= _BV(COM1B1) | _BV(WGM10); // Fast PWM 8-bit mode
    TCCR1A &= ~_BV(COM1B0); // Set on match, clear on top
    // DDRD |= _BV(PD2); //Enable output pin
    DDRD |= _BV(PD3); //Enable output pin

    OCR1A = (uint8_t) 200;      // Duty Cycle

    while(1) {

        OCR1A = 0x16;
    }
}
