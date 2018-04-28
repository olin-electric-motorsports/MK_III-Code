#include <avr/io.h>
#include <util/delay.h>
#include "log_uart.h"
#include <string.h> // for strlen
#include <stdio.h> // for sprintf

/*
 * This example code shows how to use the UART logging library
 */

int main (void) {
    // Initialize the UART peripheral
    LOG_init();

    uint8_t var = 0;
    char disp_string[64];
    char static_msg[] = "static example msg";


    while(1) {
        var++; // increase the dynamic variable
        LOG_println(static_msg, strlen(static_msg));
        sprintf(disp_string, "dynamic variable is %d, or %x", var, var);
        LOG_println(disp_string, strlen(disp_string));
        // Give a delay to the toggle so it doesn't infinitely toggle
        _delay_ms(500);
    }
}
