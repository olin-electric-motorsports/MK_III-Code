/* CAN Rx */
#include <avr/io.h>
#include <util/delay.h>
#include "can_api.h"


#define PORT_LED        PORTB
#define LED             PB5

volatile uint8_t gFlag = 0x00;          // Global Flag


ISR(CAN_INT_vect)
{
    // Read message into memory
    uint8_t msg[CAN_ID_GLOBAL];
    CAN_read_received(0, CAN_ID_GLOBAL, msg);

    if(CANMSG == 0xB1) {
        gFlag |= _BV(0);
    }


    // Set the chip to wait for another message.
    CAN_wait_on_receive(0, CAN_ID_GLOBAL, CAN_LEN_GLOBAL, 0xFF);
}


int main (void)
{
    DDRB |= _BV(LED);

    sei();

    // Initialize CAN
    CAN_init(CAN_ENABLED);

    // Tell the CAN system to wait for a message.
    CAN_wait_on_receive(0, CAN_ID_GLOBAL, CAN_LEN_GLOBAL, 0xFF);

    // gFlag |= _BV(0);        // Read ports

    while(1) {
        // Wait indefinitely for a message to come.
        if(bit_is_set(gFlag, 0)) {
            PORT_LED ^= _BV(LED);
            gFlag &= ~_BV(0);
        }
    }
}
