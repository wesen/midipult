all: midipult.hex

MCU=atmega8535
CC=avr-gcc
CFLAGS+=-Os -mmcu=$(MCU) -g
LDFLAGS+=-mmcu=$(MCU)
OBJCOPY=avr-objcopy
UISP=uisp
UISPFLAGS=-dprog=stk200 -dlpt=0x3bc

midipult.elf: midipult.o
	$(CC) $(LDFLAGS) -o $@ $<

midipult.hex: midipult.elf
	$(OBJCOPY) -j .text -j .data -O ihex midipult.elf midipult.hex

upload: midipult.hex
	$(UISP) $(UISPFLAGS) --erase
	$(UISP) $(UISPFLAGS) --upload if=midipult.hex
	$(UISP) $(UISPFLAGS) --verify if=midipult.hex

reset:
	$(UISP) $(UISPFLAGS) --rd_fuses

clean:
	rm -f *.elf *.hex *.o *.lst
