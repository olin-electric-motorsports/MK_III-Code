#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include "log_uart.h"


int main (void) {
    DDRC |= _BV(PC5);
    LOG_init();

    while(1) {
        PORTC ^= _BV(PC5);
        char blink_msg[] = "BL: *blink*";
        LOG_println(blink_msg, strlen(blink_msg));

        // Give a delay to the toggle so it doesn't infinitely toggle
        _delay_ms(1000);
    }
}


//SS PB0, PB1, ext PB3, PB4
//BL PC4, PC5, ext PC0, PD0
//WS PB3, PD6, ext PB0, PB1
