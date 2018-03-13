#include <avr/io.h>
#include <util/delay.h>

int main (void) {
    // Set PB4 to output
    // Use pin 10 to light up an LED
    DDRB |= _BV(PB5);

    while(1) {
        // Toggle PE1 (pin 10)
        // Toggles power to pin 10 to create a "blink"
        PORTB ^= _BV(PB5);


        // Give a delay to the toggle so it doesn't infinitely toggle
        _delay_ms(100);
    }
}
