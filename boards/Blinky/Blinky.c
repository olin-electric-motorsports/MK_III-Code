#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include "log_uart.h"


int main (void) {
    DDRB |= _BV(PB3);
    LOG_init();

    while(1) {
        PORTB ^= _BV(PB3);
        char blink_msg[] = "WS: *blink*";
        LOG_println(blink_msg, strlen(blink_msg));

        // Give a delay to the toggle so it doesn't infinitely toggle
        _delay_ms(1000);
    }
}


//SS PB0, PB1, ext PB3, PB4
//BL PC4, PC5, ext PC0, PD0
//WS PB3, PD6, ext PB0, PB1
