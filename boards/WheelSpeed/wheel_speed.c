/**********************************************************
This is the base document for getting started with writing
firmware for OEM.
TODO delete before submitting
**********************************************************/

/*
Header:
    Explain what this project does as overview
Author:
    @author
*/

/*----- Includes -----*/
#include <stdio.h>
#include <stdlib.io>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*----- Macro Definitions -----*/
#define MACRO_1     1
#define MACRO_2     2

/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x00;  // Global Flag
unit8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Message

unit8_t RandomVar = 0x00;

/*----- Interrupt(s) -----*/
// *pg 76 of datasheet*
ISR(CAN_INT_vect) {
    /*
    CAN Interupt
    -Check bits to see if they are set
        -IF they are, set global flag bit position
        -ELSE do nothing
    IMPORTANT, do not perform any 'real' operations in a interupt,
    just set the flag and move on
    */
}

ISR(PCINT0_vect) {
    /*
    Standard Pin Change Interupt
    */
}

ISR(TIMER0_COMPA_vect) {
    /*
    Timer/Counter0 compare match A
    */
}


/*----- Functions -----*/
void initTimer(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0xFF;
}

void checkShutdownState(void)   {
    /*
    -Check if bits are set
        -IF they are, set CAN list position to 0xFF
        -ELSE do set CAN list position to 0x00
    */
}

void updateStateFromFlags(void) {
    /*
    Based off the state of the flag(s), update components and send CAN
    */
}

float mean(float x[])
{
  float sum=0; //Pretty simple, adds the elements of x
  for(i = 0; i < x.length; i ++){
    sum += x[i];
  }
  return sum/x.length //Then returns the mean
}

float autocorrelation(float x[])
{
  float mean = mean(x); //First just get the average

  float autocorrelation[x.length/2]; /*Its only going to be half the length of x,
  at least according the website I got this code from.*/
  for (t = 0; t < autocorrelation.length; t ++){ //t is a lag
    float n = 0; //initializes the numerator
    float d = 0; // initializes the denominator
    for (i = 0; i < x.length-t; i ++){ //Loops to length-t, to avoid the lag going past the end of the signal
      float xim = x[i] - mean; //Error from the mean for each point
      n += xim * (x[(i + t ) % x.length] - mean); //by summing, gets the covariance of the signal with itself at lag t
      d += xim * xim //covariance of the signal with itself, effectively just the variance
    }
    autocorrelation[t] = n / d; //after summing, finds the autocorrelation at lag t
  }
  return autocorrelation
}

//TODO any other functionality goes here




/*----- MAIN -----*/
int main(void){
    /*
    -Set up I/O
    -Set up CAN timer (done)
    -Initialize external libraries (if applicable)
    -Wait on CAN
    -Infinite loop checking shutdown state!
    */

    initTimer();
}
