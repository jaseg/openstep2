F_CPU = 16000000
DEVICE = atmega328p

CFLAGS= -mmcu=$(DEVICE) -DF_CPU=$(F_CPU) 
CFLAGS+= -Os -ffunction-sections -fdata-sections -fpack-struct -fno-move-loop-invariants -fno-tree-scev-cprop -fno-inline-small-functions  
CFLAGS+= -Wall -Wno-pointer-to-int-cast
#CFLAGS+= -Wa,-ahls=$<.lst

LDFLAGS = -Wl,--relax,--section-start=.text=0,-Map=main.map

all: main.elf main.lst

main.elf: main.c
	avr-gcc $(CFLAGS) -o $@ $^
	avr-size $@

main.lst: main.c
	avr-gcc -c -g -Wa,-a,-ad $(CFLAGS) $^ >$@

.PHONY: program
program: main.elf
	avrdude -c arduino -P /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0 -b 115200 -p atmega328p -U flash:w:$<

.PHONY:	clean
clean:
	rm -f main.elf main.lst
