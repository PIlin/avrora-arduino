
AVR_PATH :=/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin
AVR_GCC :=$(AVR_PATH)/avr-gcc
AVR_OBJDUMP :=$(AVR_PATH)/avr-objdump

MCU :=atmega32

%.elf: %.c Makefile
	$(AVR_GCC) -mmcu=$(MCU) -o $@ $<

%.od: %.elf
	$(AVR_OBJDUMP) -zhD $< > $@

all: simple.elf simple.od
	

clean: 
	-rm *.elf *.od
