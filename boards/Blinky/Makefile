TARGET = $(build)
CC=avr-gcc
PROGRAMMER=avrispmkII
PORT=usb
AVRDUDE=avrdude
OBJCOPY=avr-objcopy

## My project
TARGET = blinkLED
OBJECTS = blinkLED.o utility-functions.o

## Chip and project-specific global definitions
MCU=atmega16m1
F_CPU = 4000000UL
CPPFLAGS = -DF_CPU=$(F_CPU) -I.

## Compiler/linker options
CFLAGS = -Os -g -std=gnu99 -Wall
CFLAGS = += -ffunction-section -fdata-sections

TARGET_ARCH = -mmcu=$(MCU)

LDFLAGS = -Wl -Map,$(TARGET).map
LDFLAGS += -Wl,--gc-sections

## Target and rules
all: flash

.PHONY: flash
flash: $(TARGET).hex
    avrdude -c usbtiny -p $(MCU) -U flash:w:$(TARGET).hex

%.hex: %.elf
    $(OBJCOPY) -j .text -j .data -O ihex $< $@

.PHONY: fuse
fuse:
    sudo $(AVRDUDE) $(AVRFLAGS) -U lfuse:w:0x65:m

%.elf: $(OBJECTS)
    $(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

.PHONY: clean
clean:
    rm -f $(TARGET).elf $(TARGET).hex $(TARGET).map

