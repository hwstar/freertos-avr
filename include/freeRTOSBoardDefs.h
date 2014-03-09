/* freeRTOSBoardDefs.h
 *
 * Board (hardware) specific definitions for the AVR boards that I use regularly.
 * This includes
 * Arduino UNO with ATmega328p
 * Goldilocks with ATmega1284p
 * Arduino MEGA with ATmega2560
 *
 * And also Pololu SVP with ATmega1284p
 *
 */

#ifndef freeRTOSBoardDefs_h
#define freeRTOSBoardDefs_h

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/io.h>

/*-----------------------------------------------------------
 * MCU and application specific definitions.
 *
 * These definitions should be adjusted for your particular
 * application requirements.
 *
 *----------------------------------------------------------*/

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) 
		
#ifndef _MEGA_
	#define _MEGA_
#endif
	#define portUSE_TIMER3											// portUSE_TIMER3 to use 16 bit Timer3
    #define configTICK_RATE_HZ		( ( TickType_t ) 500 )			// Use 500Hz for TIMER3
                                                                    // Use 1000Hz to get mSec timing.

	#define configCPU_CLOCK_HZ		( ( uint32_t ) F_CPU)			// This F_CPU variable set by Eclipse environment

//	XRAM device options. Different methods of enabling and driving.    MegaRAM only implemented for two banks of 56kByte currently.
//	#define portMEGA_RAM											// Use the Rugged Circuits External (128kByte) MegaRAM device. - OR -
//	#define portQUAD_RAM											// Use the Rugged Circuits External (512kByte) QuadRAM device.

//	portQUAD_RAM device Options. NOT valid for use with portMEGA_RAM.  XRAM Memory is available as 8 banks of 56kByte, for heap. - OR -
//	#define portEXT_RAM_16_BANK										// XRAM Memory is available as 16 banks of 32kByte, for heap. - OR -
//	#define portEXT_RAMFS											// XRAM Memory is available as 16 banks of 32kByte for 16 Arduino clients (i.e. NOT used for heap).


#if defined (portQUAD_RAM) || defined (portMEGA_RAM)
	#define portEXT_RAM
#endif

#if defined (portMEGA_RAM) || (defined (portQUAD_RAM) && !defined (portEXT_RAMFS))
	// XRAM banks enabled. We have to set the linker to move the heap to XRAM. -> DON'T FORGET TO ADD THESE LINK OPTIONS
	#define configTOTAL_HEAP_SIZE	( (size_t ) ((uint8_t *)(XRAMEND - 0x8000)) ) // Should be 0xffff - 0x8000 = 32767 for (non malloc) heap in XRAM.
																	// Used for heap_1.c, heap2.c, and heap4.c only, and maximum Array size possible for Heap is 32767.
#else
	// There is no XRAM available for the heap.
	#define configTOTAL_HEAP_SIZE	( (size_t ) 0x1800 )			// 0x1800 = 6144 used for heap_1.c, heap2.c, and heap4.c only, where heap is NOT in XRAM.
																	// Used for heap_1.c, heap2.c, and heap4.c only, and maximum Array size possible for Heap is 32767.
#endif

	//	#define portW5200						// otherwise we assume W5100 Ethernet

	#define portHD44780_LCD					// define the use of the Freetronics HD44780 LCD (or other). Check include hd44780.h for (flexible) pin assignments.
	#define portSD_CARD						// define the use of the SD Card for Arduino Mega2560 and Freetronics EtherMega
	#define portRTC_DEFINED					// RTC DS1307 implemented, therefore define.

	#define	portSERIAL_BUFFER_RX	64		// Define the size of the serial receive buffer.
	#define	portSERIAL_BUFFER_TX	255		// Define the size of the serial transmit buffer, only as long as the longest line of text.
	#define portSERIAL_BUFFER		portSERIAL_BUFFER_TX

    #define portUSE_TIMER1_PWM				// Define which Timer to use as the PWM Timer (not the tick timer).


#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p

#ifndef _GOLDILOCKS_
	#define _GOLDILOCKS_
#endif

//	#define portUSE_TIMER0                                          // portUSE_TIMER0 to use 8 bit Timer0
	#define portUSE_TIMER2											// portUSE_TIMER2 to use 8 bit RTC Timer2 on 1284p device
//	#define portUSE_TIMER3											// portUSE_TIMER3 to use 16 bit Timer3 on 1284p device
    #define configTICK_RATE_HZ		( ( TickType_t ) 256 )			// Use 500Hz for TIMER3. MINIMUM of 128Hz for TIMER2.
                                                                    // Use 1000Hz to get mSec timing using TIMER3.

	#define configCPU_CLOCK_HZ		( ( uint32_t ) F_CPU )			// This F_CPU variable set by Eclipse environment
    #define configTOTAL_HEAP_SIZE	( (size_t )  12000  )			// used for heap_1.c and heap2.c, and heap_4.c only

	#define portW5200						// otherwise we assume W5100 Ethernet

//	#define portHD44780_LCD					// define the use of the Freetronics HD44780 LCD (or other). Check include hd44780.h for (flexible) pin assignments.
	#define portSD_CARD						// define the use of the SD Card for Goldilocks 1284p
//	#define portRTC_DEFINED					// RTC DS1307 implemented, therefore define.

	#define	portSERIAL_BUFFER_RX	64		// Define the size of the serial receive buffer.
	#define	portSERIAL_BUFFER_TX	255		// Define the size of the serial transmit buffer, only as long as the longest text.
	#define portSERIAL_BUFFER		portSERIAL_BUFFER_TX // just for compatibility with older programmes.

    #define portUSE_TIMER1_PWM				// Define which Timer to use as the PWM Timer (not the tick timer).
											// though it is better to use Pololu functions, as they support 8x multiplexed servos.

#elif defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega8U2__)
// Arduino Serial I/O MCU Compatible notation.
#ifndef _U2DUINO_
	#define _U2DUINO_
#endif

//  THIS IS IMPORTANT TO DEFINE THE freeRTOS TICK TIMER, and also Tick Rate
//  Define either: portUSE_TIMER0 or portUSE_TIMER1 for the 32u2
    #define portUSE_TIMER0                                          // portUSE_TIMER0 to use 8 bit Timer0
//  #define portUSE_TIMER1                                          // portUSE_TIMER1 to use 16 bit Timer1
    #define configTICK_RATE_HZ		( ( TickType_t ) 200  )			// Use 200Hz for TIMER0 and 400Hz for TIMER1
                                                                    // Use 1000Hz to get mSec timing.

	#define configCPU_CLOCK_HZ		( ( uint32_t ) F_CPU)			// This F_CPU variable set by Eclipse environment

	// Cannot emphasise how important it is to watch and massage this heap size number.
	// Greater than 100% memory usage. Subtle fail.
	// Less than 96%. Typically every byte counts for 328p.
	// Watch for the stack overflowing, if you use interrupts. Use configCHECK_FOR_STACK_OVERFLOW
    #define configTOTAL_HEAP_SIZE	( (size_t ) 830 )				// used for heap_1.c, heap_2.c, and heap_4.c only

	#define	portSERIAL_BUFFER_RX	16		// Define the size of the serial receive buffer.
	#define	portSERIAL_BUFFER_TX	128		// Define the size of the serial transmit buffer, only as long as the longest line of text.
	#define portSERIAL_BUFFER		portSERIAL_BUFFER_TX

#if  defined(portUSE_TIMER1)				// Define which Timer to use as the PWM Timer (not the tick timer)
    #define portUSE_TIMER0_PWM				// It's pointless to use the 8bit Timer0 for Servo PWM,
											// as these Timers can't set ICR1 as TOP, allowing the pulse width period to be adjusted.
#elif defined( portUSE_TIMER0 )
    #define portUSE_TIMER1_PWM
#else
	#error Missing definition: The PWM Timer is not defined.
#endif

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) // assume we're using an Arduino with 328p

#ifndef _UNO_
	#define _UNO_
#endif

//  THIS IS IMPORTANT TO DEFINE THE freeRTOS TICK TIMER, and also Tick Rate
//  Define either: portUSE_TIMER0 or portUSE_TIMER1 for the Arduino 328p
	#define portUSE_TIMER0                                          // portUSE_TIMER0 to use 8 bit Timer0
//	#define portUSE_TIMER1                                          // portUSE_TIMER1 to use 16 bit Timer1
    #define configTICK_RATE_HZ		( ( TickType_t ) 200  )			// Use 200Hz for TIMER0 and 400Hz for TIMER1
                                                                    // Use 1000Hz to get mSec timing.

	#define configCPU_CLOCK_HZ		( ( uint32_t ) F_CPU)			// This F_CPU variable set by Eclipse environment

	// Cannot emphasise how important it is to watch and massage this heap size number.
	// Greater than 100% memory usage. Subtle fail.
	// Less than 96%. Typically every byte counts for 328p.
	// Watch for the stack overflowing, if you use interrupts. Use configCHECK_FOR_STACK_OVERFLOW
    #define configTOTAL_HEAP_SIZE	( (size_t ) 1230 )				// used for heap_1.c, heap_2.c, and heap_4.c only

//	#define portEXT_RAMFS					// XRAM Memory is available from a 2560 as 16 banks of 32kByte for 16x 328p ArduSat (Uno) clients.

//	#define portW5200						// otherwise we assume W5100 Ethernet

//	#define portHD44780_LCD					// define the use of the Freetronics HD44780 LCD (or other). Check include hd44780.h for (flexible) pin assignments.
//	#define portRTC_DEFINED					// RTC DS1307 implemented, therefore define.

	#define	portSERIAL_BUFFER_RX	32		// Define the size of the serial receive buffer.
	#define	portSERIAL_BUFFER_TX	128		// Define the size of the serial transmit buffer, only as long as the longest line of text.
	#define portSERIAL_BUFFER		portSERIAL_BUFFER_TX	// Set the default serial buffer to be the Tx size.

#if  defined(portUSE_TIMER1)				// Define which Timer to use as the PWM Timer (not the tick timer)
    #define portUSE_TIMER0_PWM				// It's pointless to use the 8bit Timer0 or Timer2 for Servo PWM,
											// as these Timers can't set ICR1 as TOP, allowing the pulse width period to be adjusted.
#elif defined( portUSE_TIMER0 )
    #define portUSE_TIMER1_PWM
#else
	#error Missing definition: The PWM Timer is not defined.
#endif

#else
	#error Missing definition: The MCU type is not defined.
#endif


/*-----------------------------------------------------------
 * Board specific definitions.
 *
 * These definitions should probably not be adjusted for your
 * application requirements.
 *
 *----------------------------------------------------------*/

#if defined(_MEGA_)
// Setting this to match some random (linear) pin definitions for Arduino MEGA
// The MEGA so randomises its pins around the board, it is boring to try to rationalise this.

// I2C pins
#define I2C_PORT			PORTD
#define I2C_PORT_DIR		DDRD
#define I2C_PORT_STATUS		PIND
#define I2C_BIT_SDA			_BV(PD1)
#define I2C_BIT_SCL			_BV(PD0)

// SPI pins
#define SPI_PORT			PORTB
#define SPI_PORT_DIR		DDRB
#define SPI_PORT_PIN		PINB
#define SPI_BIT_SCK			_BV(PB1)
#define SPI_BIT_MISO		_BV(PB3)
#define SPI_BIT_MOSI		_BV(PB2)
#define SPI_BIT_SS			_BV(PB0)

#define SPI_BIT_SS_WIZNET	_BV(PB4)	// added for Wiznet 5100/5200 support with SS on PB4 (Pin 4)

#define SPI_PORT_SS_G2		PORTH
#define SPI_PORT_DIR_SS_G2	DDRH
#define SPI_PORT_PIN_SS_G2	PINH
#define SPI_BIT_SS_G2		_BV(PH5)	// added to for Gameduino2 using FTDI FT800 Graphics (Pin 8)

#define SPI_PORT_SS_SD		PORTG
#define SPI_PORT_DIR_SS_SD	DDRG
#define SPI_PORT_PIN_SS_SD	PING
#define SPI_BIT_SS_SD		_BV(PG5)	// added for SD Card support with SS on PG5 (Pin 10)

// port A pins
#define IO_A0				0
#define IO_A1				1
#define IO_A2				2
#define IO_A3				3
#define IO_A4				4
#define IO_A5				5
#define IO_A6				6
#define IO_A7				7

// port B pins
#define IO_B0				8
#define IO_B1				9
#define IO_B2				10
#define IO_B3				11
#define IO_B4				12
#define IO_B5				13
#define IO_B6				14
#define IO_B7				15

// port C pins
#define IO_C0				16
#define IO_C1				17
#define IO_C2				18
#define IO_C3				19
#define IO_C4				20
#define IO_C5				21
#define IO_C6				22
#define IO_C7				23

// port D pins
#define IO_D0				24
#define IO_D1				25
#define IO_D2				26
#define IO_D3				27
//#define IO_D4
//#define IO_D5
//#define IO_D6
#define IO_D7				31

// port E pins
#define IO_E0				32
#define IO_E1				33
//#define IO_E2
#define IO_E3				35
#define IO_E4				36
#define IO_E5				37
//#define IO_E6
//#define IO_E7

// port F pins
#define IO_F0				40
#define IO_F1				41
#define IO_F2				42
#define IO_F3				43
#define IO_F4				44
#define IO_F5				45
#define IO_F6				46
#define IO_F7				47

// port G pins
#define IO_G0				48
#define IO_G1				49
#define IO_G2				50
//#define IO_G3
//#define IO_G4
#define IO_G5				53
//#define IO_G6				// doesn't exist
//#define IO_G7				// doesn't exist

// port H pins
#define IO_H0				56
#define IO_H1				57
//#define IO_H2
#define IO_H3				59
#define IO_H4				60
#define IO_H5				61
#define IO_H6				62
//#define IO_H7

// port J pins
#define IO_J0				64
#define IO_J1				65
//#define IO_J2
//#define IO_J3
//#define IO_J4
//#define IO_J5
//#define IO_J6
//#define IO_J7

// port K pins
#define IO_K0				72
#define IO_K1				73
#define IO_K2				74
#define IO_K3				75
#define IO_K4				76
#define IO_K5				77
#define IO_K6				78
#define IO_K7				79

// port L pins
#define IO_L0				80
#define IO_L1				81
#define IO_L2				82
#define IO_L3				83
#define IO_L4				84
#define IO_L5				85
#define IO_L6				86
#define IO_L7				87

#elif defined(_GOLDILOCKS_)
// Setting this to match the pin definitions for Goldilocks

// I2C pins
#define I2C_PORT			PORTC
#define I2C_PORT_DIR		DDRC
#define I2C_PORT_STATUS		PINC
#define I2C_BIT_SDA			_BV(PC1)
#define I2C_BIT_SCL			_BV(PC0)

// SPI pins
#define SPI_PORT			PORTB
#define SPI_PORT_DIR		DDRB
#define SPI_PORT_PIN		PINB
#define SPI_BIT_SCK			_BV(PB7)
#define SPI_BIT_MISO		_BV(PB6)
#define SPI_BIT_MOSI		_BV(PB5)
#define SPI_BIT_SS			_BV(PB4)

#define SPI_BIT_SS_WIZNET	_BV(PB4)	// added for Wiznet 5100/5200 support with SS on PB4 (Pin 10)

#define SPI_PORT_SS_G2		PORTB
#define SPI_PORT_DIR_SS_G2	DDRB
#define SPI_PORT_PIN_SS_G2	PINB
#define SPI_BIT_SS_G2		 _BV(PB2)	// added to for Gameduino2 using FTDI FT800 Graphics (Pin 8)

//#define SPI_PORT_SS_SD		PORTB
//#define SPI_PORT_DIR_SS_SD	DDRB
//#define SPI_PORT_PIN_SS_SD	PINB
//#define SPI_BIT_SS_SD		_BV(PB0)	// added for support of integrated SD card on PB0 (Virtual Pin 14) ERRATA This is broken. SS is not on PB0. Bugger.
#define SPI_PORT_SS_SD		PORTD
#define SPI_PORT_DIR_SS_SD	DDRD
#define SPI_PORT_PIN_SS_SD	PIND
#define SPI_BIT_SS_SD		_BV(PD4)	// added for SD Card support with Standard Arduino SS on PD4 (Pin 4) for SD cages.

// port D pins
#define IO_D0				0
#define IO_D1				1
#define IO_D2				2
#define IO_D3				3
#define IO_D4				4
#define IO_D5				5
#define IO_D6				6
#define IO_D7				7

// port B pins
#define IO_B0				14
#define IO_B1				15
#define IO_B2				8
#define IO_B3				9
#define IO_B4				10
#define IO_B5				11
#define IO_B6				12
#define IO_B7				13

// port C pins
#define IO_C0				16
#define IO_C1				17
//#define IO_C2				// JTAG
//#define IO_C3				// JTAG
//#define IO_C4				// JTAG
//#define IO_C5				// JTAG
//#define IO_C6				// TOSC RTC
//#define IO_C7				// TOSC RTC

// port A pins
#define IO_A0				24
#define IO_A1				25
#define IO_A2				26
#define IO_A3				27
#define IO_A4				28
#define IO_A5				29
#define IO_A6				30
#define IO_A7				31

#elif defined(_U2DUINO_)
// Arduino Serial I/O MCU Compatible notation.

// I2C pins					// not available

// SPI pins
#define SPI_PORT			PORTB
#define SPI_PORT_DIR		DDRB
#define SPI_PORT_PIN		PINB
#define SPI_BIT_SCK			_BV(PB1)
#define SPI_BIT_MISO		_BV(PB3)
#define SPI_BIT_MOSI		_BV(PB2)
#define SPI_BIT_SS			_BV(PB0)

// port D pins
#define IO_D0				0
#define IO_D1				1
#define IO_D2				2
#define IO_D3				3
#define IO_D4				4
#define IO_D5				5
#define IO_D6				6
#define IO_D7				7

// port B pins
#define IO_B0				8
#define IO_B1				9
#define IO_B2				10
#define IO_B3				11
#define IO_B4				12
#define IO_B5				13
#define IO_B6				14
// #define IO_B7			// HWB

// port C pins
// #define IO_C0			// XTAL
// #define IO_C1			// RESET
//#define IO_C2
//#define IO_C3
//#define IO_C4
//#define IO_C5
//#define IO_C6
//#define IO_C7

#elif defined(_UNO_)
// Arduino UNO ATmega328p Compatible notation.

// I2C pins
#define I2C_PORT			PORTC
#define I2C_PORT_DIR		DDRC
#define I2C_PORT_STATUS		PINC
#define I2C_BIT_SDA			_BV(PC4)
#define I2C_BIT_SCL			_BV(PC5)

// SPI pins
#define SPI_PORT			PORTB
#define SPI_PORT_DIR		DDRB
#define SPI_PORT_PIN		PINB
#define SPI_BIT_SCK			_BV(PB5)
#define SPI_BIT_MISO		_BV(PB4)
#define SPI_BIT_MOSI		_BV(PB3)
#define SPI_BIT_SS			_BV(PB2)

#define SPI_BIT_SS_WIZNET	_BV(PB2)	// added for Wiznet 5100/5200 support with SS on PB2 (Pin 10)

#define SPI_PORT_SS_G2		PORTB
#define SPI_PORT_DIR_SS_G2	DDRB
#define SPI_PORT_PIN_SS_G2	PINB
#define SPI_BIT_SS_G2		_BV(PB0)	// added for Gameduino2 using FTDI FT800 Graphics (Pin 8)

#define SPI_PORT_SS_SD		PORTD
#define SPI_PORT_DIR_SS_SD	DDRD
#define SPI_PORT_PIN_SS_SD	PIND
#define SPI_BIT_SS_SD		_BV(PD4)	// added for SD Card support with SS on PD4 (Pin 4)

// port D pins
#define IO_D0				0
#define IO_D1				1
#define IO_D2				2
#define IO_D3				3
#define IO_D4				4
#define IO_D5				5
#define IO_D6				6
#define IO_D7				7

// port B pins
#define IO_B0				8
#define IO_B1				9
#define IO_B2				10
#define IO_B3				11
#define IO_B4				12
#define IO_B5				13
//#define IO_B6				// XTAL
//#define IO_B7				// XTAL

// port C pins
#define IO_C0				14
#define IO_C1				15
#define IO_C2				16
#define IO_C3				17
#define IO_C4				18
#define IO_C5				19
//#define IO_C6				// only used if RESET pin is changed to be a digital I/O

#else
	#error Missing definition: The Board is not defined.
#endif

#ifdef __cplusplus
}
#endif

#endif // freeRTOSBoardDefs_h
