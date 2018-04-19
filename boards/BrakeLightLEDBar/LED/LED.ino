#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_24bargraph bar = Adafruit_24bargraph();

int wire1 = 0;
int wire2 = 0;
int wire3 = 0;
int wire4 = 0;


void setup() {
    Serial.begin(9600);     //Depricated?

    // Setup IO Pins
    pinMode(1, INPUT);
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);

    bar.begin(0x70);    // Address

    bar.setBar(1, LED_RED);
    bar.setBar(2, LED_YELLOW);
    bar.setBar(3, LED_GREEN);

    bar.writeDisplay();

    delay(2000);
}

void loop() {
    wire1 = digitalRead(1);
    wire2 = digitalRead(2);
    wire3 = digitalRead(3);
    wire4 = digitalRead(4);

    if(wire1 == 0 && wire2 == 0 && wire3 == 0 && wire4 == 0) {
        set_main_fuse();
    } else if(wire1 == 0 && wire2 == 0 && wire3 == 0 && wire4 == 1) {
        set_L_eStop();
    } else if(wire1 == 0 && wire2 == 0 && wire3 == 1 && wire4 == 0) {
        set_R_eStop();
    } else if(wire1 == 0 && wire2 == 1 && wire3 == 0 && wire4 == 0) {
        set_BSPD();
    } else if(wire1 == 1 && wire2 == 0 && wire3 == 0 && wire4 == 0) {
        set_TSMS();
    } else if(wire1 == 0 && wire2 == 0 && wire3 == 1 && wire4 == 1) {
        set_HVD();
    }

    bar.writeDisplay();
}





/*----- Set LED Positions -----*/
void set_main_fuse() {
    bar.setBar(1, LED_RED);
    bar.setBar(2, LED_RED);
    bar.setBar(3, LED_RED);
    bar.setBar(4, LED_RED);
}

void set_L_eStop() {
    bar.setBar(5, LED_YELLOW);
    bar.setBar(6, LED_YELLOW);
    bar.setBar(7, LED_YELLOW);
    bar.setBar(8, LED_YELLOW);
}

void set_R_eStop() {
    bar.setBar(9, LED_GREEN);
    bar.setBar(10, LED_GREEN);
    bar.setBar(11, LED_GREEN);
    bar.setBar(12, LED_GREEN);
}

void set_BSPD() {
    bar.setBar(13, LED_RED);
    bar.setBar(14, LED_RED);
    bar.setBar(15, LED_RED);
    bar.setBar(16, LED_RED);
}

void set_TSMS() {
    bar.setBar(17, LED_YELLOW);
    bar.setBar(18, LED_YELLOW);
    bar.setBar(19, LED_YELLOW);
    bar.setBar(20, LED_YELLOW);
}

void set_HVD() {
    bar.setBar(21, LED_GREEN);
    bar.setBar(22, LED_GREEN);
    bar.setBar(23, LED_GREEN);
    bar.setBar(24, LED_GREEN);
}
