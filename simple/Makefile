
AVR_PATH :=/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin
AVR_GCC :=$(AVR_PATH)/avr-gcc
AVR_OBJDUMP :=$(AVR_PATH)/avr-objdump

MCU :=atmega32u4

%.elf: %.c Makefile
	$(AVR_GCC) -mmcu=$(MCU) -g -o $@ $<

%.od: %.elf
	$(AVR_OBJDUMP) --disassemble-zeroes --section-headers --disassemble-all $< > $@

%.od.hr: %.elf
	$(AVR_OBJDUMP) --disassemble-zeroes --section-headers --disassemble-all --debugging  --source $< > $@


all: simple.elf simple.od simple.od.hr
	

clean: 
	-rm *.elf *.od
