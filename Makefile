all: midipult.elf

MCU=atmega8535
CC=avr-gcc
CFLAGS+=-Os -mmcu=$(MCU)
LDFLAGS+=-m$(MCU)

midipult.elf: midipult.o
	$(CC) $(LDFLAGS) -o $@ $<

clean:
	rm -f midipult.elf *.o *.lst
