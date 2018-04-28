#define F_CPU 4000000UL
#include <avr/io.h>
#include <util/delay.h>

int main (void) {
    // Set PB4 to output
    // Use pin 10 to light up an LED
    DDRB |= _BV(PC5);

    while(1) {
        // Toggle PE1 (pin 10)
        // Toggles power to pin 10 to create a "blink"
        PORTC ^= _BV(PC5);


        // Give a delay to the toggle so it doesn't infinitely toggle
        _delay_ms(500);
    }
}
