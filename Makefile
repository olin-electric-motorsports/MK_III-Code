CC = avr-gcc
MCU = atmega16m1
PROGRAMMER = avrispmkII
PORT = usb
AVRDUDE = avrdude
F_CPU = 4000000


# Source Vars #
SRCDIR=boards
LIBDIR=lib
BUIDIR=build
OBJDIR=obj

SRCLIBS=$(wildcard $(LIBDIR)/*.c)
INTLIBS=$(wildcard $(LIBDIR)/*.h)
OBJLIBS=$(patsubst $(LIBDIR)/%.c,$(LIBDIR)/%.o,$(SRCLIBS))

SRCS=$(wildcard $(SRCDIR)/$(TARGET)/*.c)
OBJS=$(patsubst $(SRCDIR)/$(TARGET)/%.c, $(OBJDIR)/%.o, $(SRCS))


# Build Vars #
CFLAGS+=-std=c99 -mmcu=$(MCU) -g -Os -Wall -Werror -Wunused -wl -I$(LIBDIR)/
LDFLAGS=-lm -DF_CPU=$(F_CPU)
AVRFLAGS=-p $(MCU) -v -c $(PROGRAMMER) -P $(PORT)


# Recipes #
$(LIBDIR)/%.o: $(LIBDIR)/%.c $(LIBDIR)/%.h
	$(CC) -c -o $@ $< $(CLFAGS)


AirControl: /$(OBJDIR)/AirControl.o  /$(SRCDIR)/AirControl.c /$(SRCDIR)/AirControl.h
	$(CC) $(CFLAGS)

Blinky:
	$(CC)

BMS_Master:
	$(CC)

BMS_Outline:
	$(CC)

Dashboard:
	$(CC)

Datalogging:
	$(CC)

SPI_Master:
	$(CC)

SPI_SLAVE:
	$(CC)

TRANSOM:
    $(CC)


$(BUIDIR):
	mkdir -p $(BUIDIR)

.PHONY: flash
flash: $(BUIDIR)/$(TARGET).hex
	sudo $(AVRDUDE) $(AVRFLAGS) -U flash:w:$<

.PHONY: fuse
	sudo $(AVRDUDE) $(AVRFLAGS) -U hfuse:w:0x65:m


all: blinky.c blink.o
	${CC} -std=c99 -mcu=${MCU} -g -Os -Wall -Werror -Wunused -Wl -Map=blinky.map -lm -DF_CPU=${F_CPU}

clean:
	rm -rf *.o *.so *.map
