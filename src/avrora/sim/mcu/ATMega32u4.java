/**
 * 
 */
package avrora.sim.mcu;

import java.util.HashMap;

import avrora.arch.avr.AVRProperties;
import avrora.arch.legacy.LegacyInterpreter;
import avrora.core.Program;
import avrora.sim.ActiveRegister;
import avrora.sim.AtmelInterpreter;
import avrora.sim.FiniteStateMachine;
import avrora.sim.Simulation;
import avrora.sim.clock.ClockDomain;
import avrora.sim.mcu.ATMega32;
import avrora.sim.mcu.ATMegaFamily;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.mcu.MicrocontrollerFactory;
import avrora.sim.mcu.RegisterLayout;
import avrora.sim.mcu.ReprogrammableCodeSegment;
import avrora.sim.mcu.ATMega32.Timer0;
import avrora.sim.mcu.ATMega32.Timer2;
import avrora.sim.mcu.ATMegaFamily.DirectionRegister;
import avrora.sim.mcu.ATMegaFamily.FlagRegister;
import avrora.sim.mcu.ATMegaFamily.MaskRegister;
import avrora.sim.mcu.ATMegaFamily.PinRegister;
import avrora.sim.mcu.ATMegaFamily.PortRegister;
import avrora.sim.mcu.ATMegaFamily.Timer1;
import avrora.sim.mcu.DefaultMCU.Pin;

/**
 * @author pavel
 * 
 */
public class ATMega32u4 extends ATMegaFamilyNew {

	public static final int _1kb = 1024;

	public static final int ATMEGA32U4_IOREG_SIZE = 0xFF - 0x20;
	public static final int ATMEGA32U4_SRAM_SIZE = (int) (2.5 * _1kb) + 0x100;
	public static final int ATMEGA32U4_FLASH_SIZE = 32 * _1kb;
	public static final int ATMEGA32U4_EEPROM_SIZE = _1kb;
	public static final int ATMEGA32U4_NUM_PINS = 45;
	public static final int ATMEGA32U4_NUM_INTS = 45;

	public static final int MODE_IDLE = 1;
	public static final int MODE_RESERVED1 = 2;
	public static final int MODE_ADCNRED = 3;
	public static final int MODE_RESERVED2 = 4;
	public static final int MODE_POWERDOWN = 5;
	public static final int MODE_STANDBY = 6;
	public static final int MODE_POWERSAVE = 7;
	public static final int MODE_EXTSTANDBY = 8;

	protected static final String[] idleModeNames = { "Active", "Idle",
			"RESERVED 1", "ADC Noise Reduction", "RESERVED 2", "Power Down",
			"Standby", "Power Save", "Extended Standby" };

	protected static final int[] wakeupTimes = { 0, 0, 0, 0, 0, 1000, 6, 1000,
			6 };

	// protected final ActiveRegister MCUCR_reg;

	private static final int[][] transitionTimeMatrix = FiniteStateMachine
			.buildBimodalTTM(idleModeNames.length, 0, wakeupTimes,
					new int[wakeupTimes.length]);

	// CS values 6 and 7 select external clock source and are not supported.
	// Results in an ArrayOutOfBound exception
	public static final int[] ATmega32u4Periods0 = { 0, 1, 8, 64, 256, 1024 };
	public static final int[] ATmega32u4Periods2 = { 0, 1, 8, 32, 64, 128, 256,
			1024 };

	protected FlagRegister TIFR0_reg;
	protected MaskRegister TIMSK0_reg;

	/**
	 * The <code>props</code> field stores a static reference to a properties
	 * object shared by all of the instances of this microcontroller. This
	 * object stores the IO register size, SRAM size, pin assignments, etc.
	 */
	public static final AVRProperties props;

	static {
		// statically initialize the pin assignments for this microcontroller
		HashMap<String, Integer> pinAssignments = new HashMap<String, Integer>(
				150);
		RegisterLayout rl = new RegisterLayout(ATMEGA32U4_IOREG_SIZE, 8); // TODO
		HashMap<String, Integer> interruptAssignments = new HashMap<String, Integer>(
				50);

		addPin(pinAssignments, 1, "AIN0", "INT6", "PE6");
		addPin(pinAssignments, 2, "UVCC");
		addPin(pinAssignments, 3, "D-");
		addPin(pinAssignments, 4, "D+");
		addPin(pinAssignments, 5, "UGND");
		addPin(pinAssignments, 6, "UCAP");
		addPin(pinAssignments, 7, "VBUS");
		addPin(pinAssignments, 8, "PB0", "SS", "PCINT0");
		addPin(pinAssignments, 9, "PB1", "PCINT1", "SCLK");
		addPin(pinAssignments, 10, "PB2", "PCINT2", "MOSI");
		addPin(pinAssignments, 11, "PB3", "PCINT3", "MISO");
		addPin(pinAssignments, 12, "PB7", "PCINT7", "OC0A", "OC1C", "RTS");
		addPin(pinAssignments, 13, "RESET");
		addPin(pinAssignments, 14, "VCC");
		addPin(pinAssignments, 15, "GND");
		addPin(pinAssignments, 16, "XTAL2");
		addPin(pinAssignments, 17, "XTAL1");
		addPin(pinAssignments, 18, "PD0", "OC0B", "SCL", "INT0");
		addPin(pinAssignments, 19, "PD1", "SDA", "INT1");
		addPin(pinAssignments, 20, "PD2", "RXD1", "INT2");
		addPin(pinAssignments, 21, "PD3", "TXD1", "INT3");
		addPin(pinAssignments, 22, "PD5", "XCK1", "CTS");
		addPin(pinAssignments, 23, "GND");
		addPin(pinAssignments, 24, "AVCC");
		addPin(pinAssignments, 25, "PD4", "ICP1", "ADC8");
		addPin(pinAssignments, 26, "PD6", "T1", "OC4D", "ADC9");
		addPin(pinAssignments, 27, "PD7", "T0", "OC4D", "ADC10");
		addPin(pinAssignments, 28, "PB4", "PCINT4", "ADC11");
		addPin(pinAssignments, 29, "PB5", "PCINT5", "OC1A", "OC4B", "ADC12");
		addPin(pinAssignments, 30, "PB6", "PCINT6", "OC1B", "OC4B", "ADC13");
		addPin(pinAssignments, 31, "PC6", "OC3A", "OC4A");
		addPin(pinAssignments, 32, "PC7", "ICP3", "CLK0", "OC4A");
		addPin(pinAssignments, 33, "PE2", "HWB");
		addPin(pinAssignments, 34, "VCC");
		addPin(pinAssignments, 35, "GND");
		addPin(pinAssignments, 36, "PF7", "ADC7", "TDI");
		addPin(pinAssignments, 37, "PF6", "ADC6", "TDO");
		addPin(pinAssignments, 38, "PF5", "ADC5", "TMS");
		addPin(pinAssignments, 39, "PF4", "ADC4", "TCK");
		addPin(pinAssignments, 40, "PF1", "ADC1");
		addPin(pinAssignments, 41, "PF0", "ADC0");
		addPin(pinAssignments, 42, "AREF");
		addPin(pinAssignments, 43, "GND");
		addPin(pinAssignments, 44, "AVCC");

		// extended IO registers
		// rl.addIOReg("reserved", 0xFF - 0x20);
		// rl.addIOReg("reserved", 0xFE - 0x20);
		// rl.addIOReg("reserved", 0xFD - 0x20);
		// rl.addIOReg("reserved", 0xFC - 0x20);
		// rl.addIOReg("reserved", 0xFB - 0x20);
		// rl.addIOReg("reserved", 0xFA - 0x20);
		// rl.addIOReg("reserved", 0xF9 - 0x20);
		// rl.addIOReg("reserved", 0xF8 - 0x20);
		// rl.addIOReg("reserved", 0xF7 - 0x20);
		// rl.addIOReg("reserved", 0xF6 - 0x20);
		// rl.addIOReg("reserved", 0xF5 - 0x20);
		rl.addIOReg("UEINT", 0xF4 - 0x20);
		rl.addIOReg("UEBCHX", 0xF3 - 0x20);
		rl.addIOReg("UEBCLX", 0xF2 - 0x20);
		rl.addIOReg("UEDATX", 0xF1 - 0x20);
		rl.addIOReg("UEIENX", 0xF0 - 0x20);
		rl.addIOReg("UESTA1X", 0xEF - 0x20);
		rl.addIOReg("UESTA0X", 0xEE - 0x20);
		rl.addIOReg("UECFG1X", 0xED - 0x20);
		rl.addIOReg("UECFG0X", 0xEC - 0x20);
		rl.addIOReg("UECONX", 0xEB - 0x20);
		rl.addIOReg("UERST", 0xEA - 0x20);
		rl.addIOReg("UENUM", 0xE9 - 0x20);
		rl.addIOReg("UEINTX", 0xE8 - 0x20);
		// rl.addIOReg("reserved", 0xE7 - 0x20);
		rl.addIOReg("UDMFN", 0xE6 - 0x20);
		rl.addIOReg("UDFNUMH", 0xE5 - 0x20);
		rl.addIOReg("UDFNUML", 0xE4 - 0x20);
		rl.addIOReg("UDADDR", 0xE3 - 0x20);
		rl.addIOReg("UDIEN", 0xE2 - 0x20);
		rl.addIOReg("UDINT", 0xE1 - 0x20);
		rl.addIOReg("UDCON", 0xE0 - 0x20);
		// rl.addIOReg("reserved", 0xDF - 0x20);
		// rl.addIOReg("reserved", 0xDE - 0x20);
		// rl.addIOReg("reserved", 0xDD - 0x20);
		// rl.addIOReg("reserved", 0xDC - 0x20);
		// rl.addIOReg("reserved", 0xDB - 0x20);
		rl.addIOReg("USBINT", 0xDA - 0x20);
		rl.addIOReg("USBSTA", 0xD9 - 0x20);
		rl.addIOReg("USBCON", 0xD8 - 0x20);
		rl.addIOReg("UHWCON", 0xD7 - 0x20);
		// rl.addIOReg("reserved", 0xD6 - 0x20);
		// rl.addIOReg("reserved", 0xD5 - 0x20);
		rl.addIOReg("DT4", 0xD4 - 0x20);
		// rl.addIOReg("reserved", 0xD3 - 0x20);
		rl.addIOReg("OCR4D", 0xD2 - 0x20);
		rl.addIOReg("OCR4C", 0xD1 - 0x20);
		rl.addIOReg("OCR4B", 0xD0 - 0x20);
		rl.addIOReg("OCR4A", 0xCF - 0x20);
		rl.addIOReg("UDR1", 0xCE - 0x20);
		rl.addIOReg("UBRR1H", 0xCD - 0x20);
		rl.addIOReg("UBRR1L", 0xCC - 0x20);
		// rl.addIOReg("reserved", 0xCB - 0x20);
		rl.addIOReg("UCSR1C", 0xCA - 0x20);
		rl.addIOReg("UCSR1B", 0xC9 - 0x20);
		rl.addIOReg("UCSR1A", 0xC8 - 0x20);
		rl.addIOReg("CLKSTA", 0xC7 - 0x20);
		rl.addIOReg("CLKSEL1", 0xC6 - 0x20);
		rl.addIOReg("CLKSEL0", 0xC5 - 0x20);
		rl.addIOReg("TCCR4E", 0xC4 - 0x20);
		rl.addIOReg("TCCR4D", 0xC3 - 0x20);
		rl.addIOReg("TCCR4C", 0xC2 - 0x20);
		rl.addIOReg("TCCR4B", 0xC1 - 0x20);
		rl.addIOReg("TCCR4A", 0xC0 - 0x20);
		rl.addIOReg("TC4H", 0xBF - 0x20);
		rl.addIOReg("TCNT4", 0xBE - 0x20);
		rl.addIOReg("TWAMR", 0xBD - 0x20);
		rl.addIOReg("TWCR", 0xBC - 0x20);
		rl.addIOReg("TWDR", 0xBB - 0x20);
		rl.addIOReg("TWAR", 0xBA - 0x20);
		rl.addIOReg("TWSR", 0xB9 - 0x20);
		rl.addIOReg("TWBR", 0xB8 - 0x20);
		// rl.addIOReg("reserved", 0xB7 - 0x20);
		// rl.addIOReg("reserved", 0xB6 - 0x20);
		// rl.addIOReg("reserved", 0xB5 - 0x20);
		// rl.addIOReg("reserved", 0xB4 - 0x20);
		// rl.addIOReg("reserved", 0xB3 - 0x20);
		// rl.addIOReg("reserved", 0xB2 - 0x20);
		// rl.addIOReg("reserved", 0xB1 - 0x20);
		// rl.addIOReg("reserved", 0xB0 - 0x20);
		// rl.addIOReg("reserved", 0xAF - 0x20);
		// rl.addIOReg("reserved", 0xAE - 0x20);
		// rl.addIOReg("reserved", 0xAD - 0x20);
		// rl.addIOReg("reserved", 0xAC - 0x20);
		// rl.addIOReg("reserved", 0xAB - 0x20);
		// rl.addIOReg("reserved", 0xAA - 0x20);
		// rl.addIOReg("reserved", 0xA9 - 0x20);
		// rl.addIOReg("reserved", 0xA8 - 0x20);
		// rl.addIOReg("reserved", 0xA7 - 0x20);
		// rl.addIOReg("reserved", 0xA6 - 0x20);
		// rl.addIOReg("reserved", 0xA5 - 0x20);
		// rl.addIOReg("reserved", 0xA4 - 0x20);
		// rl.addIOReg("reserved", 0xA3 - 0x20);
		// rl.addIOReg("reserved", 0xA2 - 0x20);
		// rl.addIOReg("reserved", 0xA1 - 0x20);
		// rl.addIOReg("reserved", 0xA0 - 0x20);
		// rl.addIOReg("reserved", 0x9F - 0x20);
		// rl.addIOReg("reserved", 0x9E - 0x20);
		rl.addIOReg("OCR3CH", 0x9D - 0x20);
		rl.addIOReg("OCR3CL", 0x9C - 0x20);
		rl.addIOReg("OCR3BH", 0x9B - 0x20);
		rl.addIOReg("OCR3BL", 0x9A - 0x20);
		rl.addIOReg("OCR3AH", 0x99 - 0x20);
		rl.addIOReg("OCR3AL", 0x98 - 0x20);
		rl.addIOReg("ICR3H", 0x97 - 0x20);
		rl.addIOReg("ICR3L", 0x96 - 0x20);
		rl.addIOReg("TCNT3H", 0x95 - 0x20);
		rl.addIOReg("TCNT3L", 0x94 - 0x20);
		// rl.addIOReg("reserved", 0x93 - 0x20);
		rl.addIOReg("TCCR3C", 0x92 - 0x20);
		rl.addIOReg("TCCR3B", 0x91 - 0x20);
		rl.addIOReg("TCCR3A", 0x90 - 0x20);
		// rl.addIOReg("reserved", 0x8F - 0x20);
		// rl.addIOReg("reserved", 0x8E - 0x20);
		rl.addIOReg("OCR1CH", 0x8D - 0x20);
		rl.addIOReg("OCR1CL", 0x8C - 0x20);
		rl.addIOReg("OCR1BH", 0x8B - 0x20);
		rl.addIOReg("OCR1BL", 0x8A - 0x20);
		rl.addIOReg("OCR1AH", 0x89 - 0x20);
		rl.addIOReg("OCR1AL", 0x88 - 0x20);
		rl.addIOReg("ICR1H", 0x87 - 0x20);
		rl.addIOReg("ICR1L", 0x86 - 0x20);
		rl.addIOReg("TCNT1H", 0x85 - 0x20);
		rl.addIOReg("TCNT1L", 0x84 - 0x20);
		// rl.addIOReg("reserved", 0x83 - 0x20);
		rl.addIOReg("TCCR1C", 0x82 - 0x20);
		rl.addIOReg("TCCR1B", 0x81 - 0x20);
		rl.addIOReg("TCCR1A", 0x80 - 0x20);
		rl.addIOReg("DIDR1", 0x7F - 0x20);
		rl.addIOReg("DIDR0", 0x7E - 0x20);
		rl.addIOReg("DIDR2", 0x7D - 0x20);
		rl.addIOReg("ADMUX", 0x7C - 0x20);
		rl.addIOReg("ADCSRB", 0x7B - 0x20);
		rl.addIOReg("ADCSRA", 0x7A - 0x20);
		rl.addIOReg("ADCH", 0x79 - 0x20);
		rl.addIOReg("ADCL", 0x78 - 0x20);
		// rl.addIOReg("reserved", 0x77 - 0x20);
		// rl.addIOReg("reserved", 0x76 - 0x20);
		// rl.addIOReg("reserved", 0x75 - 0x20);
		// rl.addIOReg("reserved", 0x74 - 0x20);
		// rl.addIOReg("reserved", 0x73 - 0x20);
		rl.addIOReg("TIMSK4", 0x72 - 0x20);
		rl.addIOReg("TIMSK3", 0x71 - 0x20);
		// rl.addIOReg("reserved", 0x70 - 0x20);
		rl.addIOReg("TIMSK1", 0x6F - 0x20);
		rl.addIOReg("TIMSK0", 0x6E - 0x20);
		// rl.addIOReg("reserved", 0x6D - 0x20);
		// rl.addIOReg("reserved", 0x6C - 0x20);
		rl.addIOReg("PCMSK0", 0x6B - 0x20);
		rl.addIOReg("EICRB", 0x6A - 0x20);
		rl.addIOReg("EICRA", 0x69 - 0x20);
		rl.addIOReg("PCICR", 0x68 - 0x20);
		rl.addIOReg("RCCTRL", 0x67 - 0x20);
		rl.addIOReg("OSCCAL", 0x66 - 0x20);
		rl.addIOReg("PRR1", 0x65 - 0x20);
		rl.addIOReg("PRR0", 0x64 - 0x20);
		// rl.addIOReg("reserved", 0x63 - 0x20);
		// rl.addIOReg("reserved", 0x62 - 0x20);
		rl.addIOReg("CLKPR", 0x61 - 0x20);
		rl.addIOReg("WDTCSR", 0x60 - 0x20);

		// lower 64 IO registers
		rl.addIOReg("SREG", 0x3F);
		rl.addIOReg("SPH", 0x3E);
		rl.addIOReg("SPL", 0x3D);
		// rl.addIOReg("reserved", 0x3C);
		rl.addIOReg("RAMPZ", 0x3B);
		// rl.addIOReg("reserved", 0x3A);
		// rl.addIOReg("reserved", 0x39);
		// rl.addIOReg("reserved", 0x38);
		rl.addIOReg("SPMCSR", 0x37);
		// rl.addIOReg("reserved", 0x36);
		rl.addIOReg("MCUCR", 0x35);
		rl.addIOReg("MCUSR", 0x34);
		rl.addIOReg("SMCR", 0x33);
		rl.addIOReg("PLLFRQ", 0x32);
		rl.addIOReg("OCDR", 0x31);
		rl.addIOReg("ACSR", 0x30);
		// rl.addIOReg("reserved", 0x2F);
		rl.addIOReg("SPDR", 0x2E);
		rl.addIOReg("SPSR", 0x2D);
		rl.addIOReg("SPCR", 0x2C);
		rl.addIOReg("GPIOR2", 0x2B);
		rl.addIOReg("GPIOR1", 0x2A);
		rl.addIOReg("PLLCSR", 0x29);
		rl.addIOReg("OCR0B", 0x28);
		rl.addIOReg("OCR0A", 0x27);
		rl.addIOReg("TCNT0", 0x26);
		rl.addIOReg("TCCR0B", 0x25);
		rl.addIOReg("TCCR0A", 0x24);
		rl.addIOReg("GTCCR", 0x23);
		rl.addIOReg("EEARH", 0x22);
		rl.addIOReg("EEARL", 0x21);
		rl.addIOReg("EEDR", 0x20);
		rl.addIOReg("EECR", 0x1F);
		rl.addIOReg("GPIOR0", 0x1E);
		rl.addIOReg("EIMSK", 0x1D);
		rl.addIOReg("EIFR", 0x1C);
		rl.addIOReg("PCIFR", 0x1B);
		// rl.addIOReg("reserved", 0x1A);
		rl.addIOReg("TIFR4", 0x19);
		rl.addIOReg("TIFR3", 0x18);
		// rl.addIOReg("reserved", 0x17);
		rl.addIOReg("TIFR1", 0x16);
		rl.addIOReg("TIFR0", 0x15);
		// rl.addIOReg("reserved", 0x14);
		// rl.addIOReg("reserved", 0x13);
		// rl.addIOReg("reserved", 0x12);
		rl.addIOReg("PORTF", 0x11);
		rl.addIOReg("DDRF", 0x10);
		rl.addIOReg("PINF", 0x0F);
		rl.addIOReg("PORTE", 0x0E);
		rl.addIOReg("DDRE", 0x0D);
		rl.addIOReg("PINE", 0x0C);
		rl.addIOReg("PORTD", 0x0B);
		rl.addIOReg("DDRD", 0x0A);
		rl.addIOReg("PIND", 0x09);
		rl.addIOReg("PORTC", 0x08);
		rl.addIOReg("DDRC", 0x07);
		rl.addIOReg("PINC", 0x06);
		rl.addIOReg("PORTB", 0x05);
		rl.addIOReg("DDRB", 0x04);
		rl.addIOReg("PINB", 0x03);
		// rl.addIOReg("reserved", 0x02);
		// rl.addIOReg("reserved", 0x01);
		// rl.addIOReg("reserved", 0x00);

		addInterrupt(interruptAssignments, "RESET", 1);
		addInterrupt(interruptAssignments, "INT0", 2);
		addInterrupt(interruptAssignments, "INT1", 3);
		addInterrupt(interruptAssignments, "INT2", 4);
		addInterrupt(interruptAssignments, "INT3", 5);
		// addInterrupt(interruptAssignments, "Reserved", 6);
		// addInterrupt(interruptAssignments, "Reserved", 7);
		addInterrupt(interruptAssignments, "INT6", 8);
		addInterrupt(interruptAssignments, "Reserved", 9);
		addInterrupt(interruptAssignments, "PCINT0", 10);
		addInterrupt(interruptAssignments, "USB General", 11);
		addInterrupt(interruptAssignments, "USB Endpoint", 12);
		addInterrupt(interruptAssignments, "WDT", 13);
		// addInterrupt(interruptAssignments, "Reserved", 14 );
		// addInterrupt(interruptAssignments, "Reserved", 15 );
		// addInterrupt(interruptAssignments, "Reserved", 16 );
		addInterrupt(interruptAssignments, "TIMER1 CAPT", 17);
		addInterrupt(interruptAssignments, "TIMER1 COMPA", 18);
		addInterrupt(interruptAssignments, "TIMER1 COMPB", 19);
		addInterrupt(interruptAssignments, "TIMER1 COMPC", 20);
		addInterrupt(interruptAssignments, "TIMER1 OVF", 21);
		addInterrupt(interruptAssignments, "TIMER0 COMPA", 22);
		addInterrupt(interruptAssignments, "TIMER0 COMPB", 23);
		addInterrupt(interruptAssignments, "TIMER0 OVF", 24);
		addInterrupt(interruptAssignments, "SPI, STC", 25);
		addInterrupt(interruptAssignments, "USART1, RX", 26);
		addInterrupt(interruptAssignments, "USART1, UDRE", 27);
		addInterrupt(interruptAssignments, "USART1, TX", 28);
		addInterrupt(interruptAssignments, "ANALOG COMP", 29);
		addInterrupt(interruptAssignments, "ADC", 30);
		addInterrupt(interruptAssignments, "EE READY", 31);
		addInterrupt(interruptAssignments, "TIMER3 CAPT", 32);
		addInterrupt(interruptAssignments, "TIMER3 COMPA", 33);
		addInterrupt(interruptAssignments, "TIMER3 COMPB", 34);
		addInterrupt(interruptAssignments, "TIMER3 COMPC", 35);
		addInterrupt(interruptAssignments, "TIMER3 OVF", 36);
		addInterrupt(interruptAssignments, "TWI", 37);
		addInterrupt(interruptAssignments, "SPM READY", 38);
		addInterrupt(interruptAssignments, "TIMER4 COMPA", 39);
		addInterrupt(interruptAssignments, "TIMER4 COMPB", 40);
		addInterrupt(interruptAssignments, "TIMER4 COMPD", 41);
		addInterrupt(interruptAssignments, "TIMER4 OVF", 42);
		addInterrupt(interruptAssignments, "TIMER4 FPF", 43);

		props = new AVRProperties(
				ATMEGA32U4_IOREG_SIZE, // number of io registers
				ATMEGA32U4_SRAM_SIZE, // size of sram in bytes
				ATMEGA32U4_FLASH_SIZE, // size of flash in bytes
				ATMEGA32U4_EEPROM_SIZE, // size of eeprom in bytes
				ATMEGA32U4_NUM_PINS, // number of pins
				ATMEGA32U4_NUM_INTS, // number of interrupts
				new ReprogrammableCodeSegment.Factory(ATMEGA32U4_FLASH_SIZE, 6),
				pinAssignments, // the assignment of names to physical pins
				rl, // the assignment of names to IO registers
				interruptAssignments);
	};

	public static class Factory implements MicrocontrollerFactory {

		/**
		 * The <code>newMicrocontroller()</code> method is used to instantiate a
		 * microcontroller instance for the particular program. It will
		 * construct an instance of the <code>Simulator</code> class that has
		 * all the properties of this hardware device and has been initialized
		 * with the specified program.
		 * 
		 * @param sim
		 * @param p
		 *            the program to load onto the microcontroller @return a
		 *            <code>Microcontroller</code> instance that represents the
		 *            specific hardware device with the program loaded onto it
		 */
		public Microcontroller newMicrocontroller(int id, Simulation sim,
				ClockDomain cd, Program p) {
			return new ATMega32u4(id, sim, cd, p);
		}

	}

	public ATMega32u4(int id, Simulation sim, ClockDomain cd, Program p) {
		super(cd, props, new FiniteStateMachine(cd.getMainClock(), MODE_ACTIVE,
				idleModeNames, transitionTimeMatrix));
		simulator = sim.createSimulator(id, LegacyInterpreter.FACTORY, this, p);
		interpreter = (AtmelInterpreter) simulator.getInterpreter();

		// TODO
		// MCUCR_reg = getIOReg("MCUCR");

		installPins();
		installDevices();
	}

	protected void installPins() {
		for (int cntr = 0; cntr < properties.num_pins; cntr++)
			pins[cntr] = new Pin(cntr);
	}

	protected void installDevices() {
		// set up the timer mask and flag registers and interrupt range
		int[] mapping_t0 = new int[] { 24, 22, 23, -1, -1, -1, -1, -1 };
		TIFR0_reg = buildInterrupt("TIMSK0", "TIFR0", mapping_t0);
		TIMSK0_reg = (MaskRegister) getIOReg("TIMSK0");

		addDevice(new Timer0());
		// addDevice(new Timer1(2));
		// addDevice(new Timer2());

		final boolean[] portc_mask = { false, false, false, false, false,
				false, true, true };
		final boolean[] porte_mask = { false, false, true, false, false, false,
				true, false };
		final boolean[] portf_mask = { true, true, false, false, true, true,
				true, true };

		buildPort('B');
		buildPort('C', portc_mask);
		buildPort('D');
		buildPort('E', porte_mask);
		buildPort('F', portf_mask);

		// addDevice(new EEPROM(properties.eeprom_size, this));
		// addDevice(new USART("", this));

		// addDevice(new SPI(this));
		// addDevice(new ADC(this, 8));
		
		addDevice(new USB(this));
		
		addDevice(new USART("1", this));
		addDevice(new SPI(this));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see avrora.sim.mcu.AtmelMicrocontroller#getSleepMode()
	 */
	@Override
	protected int getSleepMode() {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * <code>Timer0</code> is different from ATMega128
	 */
	protected class Timer0 extends Timer8bit_32u4 {
		protected Timer0() {
			super(ATMega32u4.this, 0, 1, 2, 0, 1, 2, 0, ATmega32u4Periods0);
		}
	}

	public static void addPin(HashMap<String, Integer> pinMap, int p,
			String n1, String n2, String n3, String n4, String n5) {
		Integer i = new Integer(p);
		pinMap.put(n1, i);
		pinMap.put(n2, i);
		pinMap.put(n3, i);
		pinMap.put(n4, i);
		pinMap.put(n5, i);
	}

	protected void buildPort(char p, boolean[] pin_exist) {
		Pin[] portPins = new Pin[8];
		for (int cntr = 0; cntr < 8; cntr++) {
			if (pin_exist[cntr])
				portPins[cntr] = (Pin) getPin("P" + p + cntr);
		}
		installIOReg("PORT" + p, new PortRegister(portPins));
		installIOReg("DDR" + p, new DirectionRegister(portPins));
		installIOReg("PIN" + p, new PinRegister(portPins));
	}

	protected FlagRegister buildInterrupt(String maskRegNum, String flagRegNum,
			int[] mapping) {
		FlagRegister fr = new FlagRegister(interpreter, mapping);
		MaskRegister mr = new MaskRegister(interpreter, mapping);
		installIOReg(maskRegNum, mr);
		installIOReg(flagRegNum, fr);
		return fr;
	}

}
