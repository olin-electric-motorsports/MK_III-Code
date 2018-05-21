#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include "log_uart.h"


int main (void) {
    // Set PB4 to output
    // Use pin 10 to light up an LED
    // DDRB |= _BV(PB3);       // Wheel Speed
    DDRB |= _BV(PB6);    // Suspension
    LOG_init();

    while(1) {
        // Toggle PE1 (pin 10)
        // Toggles power to pin 10 to create a "blink"
        // PORTB ^= _BV(PB3);          // Wheel /Speed
        PORTB ^= _BV(PB6);       // Suspension
        char blink_msg[] = "*blink*";
        LOG_println(blink_msg, strlen(blink_msg));

        // Give a delay to the toggle so it doesn't infinitely toggle
        _delay_ms(100);
    }
}
