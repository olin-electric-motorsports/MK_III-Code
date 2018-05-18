/***************************************************
  This is a library for our I2C LED Backpacks

  Designed specifically to work with the Adafruit LED 24 Bargraph Backpack
  ----> http://www.adafruit.com/products/721

  These displays e I2C to communicate, 2 pins are required to
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

Adafruit_24bargraph bar1 = Adafruit_24bargraph();
Adafruit_24bargraph bar2 = Adafruit_24bargraph();

uint8_t amount = 0;

const int numReadings = 10;

int readIndex = 0;              // the index of the current reading

int readings1[numReadings];      // the readings from the analog input
int total1 = 0;                  // the running total
int average1 = 0;         // the aver

int readings2[numReadings];      // the readings from the analog input
int total2 = 0;                  // the running total
int average2 = 0;         // the aver

int inputPin1 = A0;
int inputPin2 = A1;

void setup() {

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings1[thisReading] = 0;
    readings2[thisReading] = 0;
  }

  Serial.begin(9600);
  Serial.println("HT16K33 Bi-Color Bargraph test");

  bar1.begin(0x70);  // pass in the address
  bar2.begin(0x71);

  for (uint8_t b=0; b<24; b++ ){
//    if ((b % 3) == 0)  bar1.setBar(b, LED_RED);
//    if ((b % 3) == 1)  bar1.setBar(b, LED_YELLOW);
//    if ((b % 3) == 2)  bar1.setBar(b, LED_GREEN);
    // if ((b % 3) == 0)  bar2.setBar(b, LED_RED);
    // if ((b % 3) == 1)  bar2.setBar(b, LED_YELLOW);
    // if ((b % 3) == 2)  bar2.setBar(b, LED_GREEN);
    bar2.setBar(b,LED_GREEN);
    bar1.setBar(b,LED_RED);
  }
  bar1.writeDisplay();
  bar2.writeDisplay();
  delay(2000);
}


void loop() {
  calcAverage();
  uint8_t bars1 = average1/42;
  uint8_t bars2 = average2/42;


  for (uint8_t b = 0; b < 24; b++) {
    if (b < bars1){
        bar1.setBar((b+12)%24, LED_RED);
    } else {
        bar1.setBar((b+12)%24, LED_OFF);
    }
    if (b < bars2){
        bar2.setBar((b+12)%24, LED_GREEN);
    } else {
        bar2.setBar((b+12)%24, LED_OFF);
    }

  }
  bar1.writeDisplay();
  bar2.writeDisplay();

  delay(40);
}

void calcAverage(){
    // subtract the last reading:

  total1 = total1 - readings1[readIndex];
  total2 = total2 - readings2[readIndex];

  // read from the sensor:
  readings1[readIndex] = analogRead(inputPin1);
  readings2[readIndex] = analogRead(inputPin2);
  // add the reading to the total:
  total1 = total1 + readings1[readIndex];
  total2 = total2 + readings2[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average1 = total1 / numReadings;
  average2 = total2 / numReadings;

  Serial.println(average1);
  Serial.println(average2);

}
