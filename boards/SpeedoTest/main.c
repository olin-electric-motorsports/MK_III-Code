/* CAN Rx */
#include <avr/io.h>
#include <util/delay.h>
#include "can_api.h"

ISR(CAN_INT_vect)
{
    // Read message into memory
    uint8_t msg[CAN_IDT_THROTTLE_L];
    CAN_read_received(0, CAN_IDT_THROTTLE_L, msg);

    // Set PWM percentage to high byte of throttle.
    OCR1B = msg[0];

    // Set the chip to wait for another message.
    CAN_wait_on_receive(0, CAN_IDT_THROTTLE, CAN_IDT_THROTTLE_L, CAN_IDM_single);
}


int main (void)
{
    // Initialize CAN
    CAN_init(CAN_ENABLED);

    // Write PB7 as input
    DDRB &= ~_BV(PB7);

    //ADC Configuration
    //Enable ADC, set prescaler to 128 (slow down ADC clock)
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    //Enable internal reference voltage
    ADCSRB &= _BV(AREFEN);
    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);
    //Read from ADC6 (PB5)
    ADMUX |= _BV(0x6);

    //Output compare pin is OC1B, so we need OCR1B as our counter
    TCCR1B |= _BV(CS00); //Clock prescale set to max speed
    TCCR1B |= _BV(WGM12);
    TCCR1A |= _BV(COM1B1) | _BV(WGM10); // Fast PWM 8-bit mode
    TCCR1A &= ~_BV(COM1B0); // Set on match, clear on top
    DDRC |= _BV(PC1); //Enable

    OCR1B = (uint8_t) 200;

    // Tell the CAN system to wait for a message.
    CAN_wait_on_receive(0, CAN_IDT_THROTTLE, CAN_IDT_THROTTLE_L, CAN_IDM_single);

    while(1) {
        // Wait indefinitely for a message to come.
        //Read from ADC
        ADCSRA |=  _BV(ADSC);
        //Wait for ADC reading
        while(bit_is_set(ADCSRA, ADSC));
        uint16_t reading = ADC;

        OCR0B = (uint8_t) (reading >> 2);
    }
}
