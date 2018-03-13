/***************************************************
  This is a library for our I2C LED Backpacks

  Designed specifically to work with the Adafruit LED 24 Bargraph Backpack
  ----> http://www.adafruit.com/products/721

  These displays use I2C to communicate, 2 pins are required to
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_24bargraph bar = Adafruit_24bargraph();

uint8_t amount = 0;

const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;         // the aver

int inputPin = A0;

void setup() {

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  
  Serial.begin(9600);
  Serial.println("HT16K33 Bi-Color Bargraph test");

  bar.begin(0x70);  // pass in the address

  for (uint8_t b=0; b<24; b++ ){
    if ((b % 3) == 0)  bar.setBar(b, LED_RED);
    if ((b % 3) == 1)  bar.setBar(b, LED_YELLOW);
    if ((b % 3) == 2)  bar.setBar(b, LED_GREEN);
  }
  bar.writeDisplay();
  delay(2000);
}


void loop() {
  calcAverage();
  uint8_t bars = average/40;
  for (uint8_t b = 0; b < bars; b++) {
    bar.setBar(b, LED_YELLOW);
  }
    for (uint8_t b = bars; b < 24; b++) {
    bar.setBar(b, LED_OFF);
  }
  bar.writeDisplay();
  delay(40);
}

void calcAverage(){
    // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(inputPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  Serial.println(average);
}

