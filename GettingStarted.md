# Getting Started
Congrats on finishing your electrical design and welcome to the *wonderful* world of firmware! In this super brief document, I’ll outline some steps to get you started in the right directions and some useful tips too!
## Base
I’ve made a `base.c` file that lays out pretty much all the functionality most boards need or might need. It includes the following components:
* Interrupts
* Function definitions
* A `main()` function
* And macros, global variables, and includes

It’s only an outline of the code needed, and not all of it has been implemented (actually most of it hasn’t, that’s **your** job) so if you need help with implementing the actual code, take a look at last year’s code (especially dashboard, BMS master, and others). You can find it [here.](https://github.com/olin-electric-motorsports/MK_II-Code)

## The Datasheet
You’re going to be writing code for AtMega16M1 MCUs, so having the [datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8209-8-bit%20AVR%20ATmega16M1-32M1-64M1_Datasheet.pdf) on hand, it imperative. This handy dandy 429 page bible will guide you through all the magical powers hidden within our tiny black box, just waiting for you to use it! It is and will be your best friend throughout writing firmware, so use it often and religiously(it is a bible after all :stuck_out_tongue:).

## Schematics
You are writing code for custom made hardware, so you need to know the hardware interactions. You will need the schematics throughout this process so you can look at the pinout of the MCU and determine with pins you need to set, flag, and interrupt. Once the design is final, it might be worth printing out the schematic to figure out pins as you write code.
