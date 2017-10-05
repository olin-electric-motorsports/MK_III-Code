CC = avr-gcc
MCU = atmega16m1
PROGRAMMER = avrispmkII
PORT = usb
AVRDUDE = avrdude
F_CPU = 4000000


# Targets
all: blinky.c blink.o
	${CC} -std=c99 -mcu=${MCU} -g -Os -Wall -Werror -Wunused -Wl -Map=blinky.map -lm -DF_CPU=${F_CPU}

clean:
	rm -rf *.o *.so *.map
