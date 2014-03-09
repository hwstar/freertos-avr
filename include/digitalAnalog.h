/*
  OrangutanDigital.h - Library for using the digital I/O lines on the
	Orangutan LV, SV, SVP, X2, Baby Orangutan B, or 3pi robot.  The code
	is all inline, which lets it compile to very small, fast, efficient
	assembly code if you use constants as your inputs.  For example,
	the line:

		setOutput(3, HIGH);

	compiles to the assembly:

		sbi 0x0b, 3  ;i.e. PORTD |= 1 << 3;
		sbi 0x0a, 3  ;i.e. DDRD  |= 1 << 3;

	In short, if your inputs are constants, you can use this library in
	place of raw digital I/O register manipulation without worrying
	about any significantly increased overhead or processing time.
	Using variables as inputs can increase overhead and processing time,
	but the functions in this library allow for simpler programmatic
	approaches to working with digital I/O, since you no longer have to
	deal with a multitude of pin-specific registers.

	The digital pins on the AVR default to high-impedance inputs after
	a power-up or reset.
*/



/*
 * Derived from Ben Schmidel, August 11, 2009.
 * Copyright (c) 2009 Pololu Corporation. For more information, see
 *
 *   http://www.pololu.com
 *   http://forum.pololu.com
 *   http://www.pololu.com/docs/0J18
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the two links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, Pololu provides this work
 * without any warranty.  It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */


#ifndef digitalAnalog_h
#define digitalAnalog_h

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/io.h>

// Digital

#define INPUT 				0
#define OUTPUT				1
#define LOW					0
#define HIGH				1
#define TOGGLE				0xFF
#define HIGH_IMPEDANCE		0
#define PULL_UP_ENABLED		1


// Analogue

#define MODE_8_BIT		1
#define MODE_10_BIT		0

#define INTERNAL_REF    1
#define EXTERNAL_REF    0



/************** DIGITAL *****************/

// high-level method for setting the specified pin as an output with the specified output state.
// An outputState value of 0 will cause the pin to drive low; a value of 1 will cause the pin to
// drive high.  A value of 0xFF (255) will toggle the output state of the pin (i.e. high -> low and
// low -> high).
void setDigitalOutput(uint8_t pin, uint8_t outputState);

// high-level method for setting the specified pin as an input with the specified input state.
// An inputState value of 0 will cause the pin to be a high-impedance input; a value of 1 will enable the
// pin's internal pull-up resistor, which weakly pulls it to Vcc.  A value of 0xFF (255) will toggle the
// input state.
void setDigitalInput(uint8_t pin, uint8_t inputState);

// high-level method for disabling the digital input circuitry of specified pin.
// Do this to avoid loading during analogue sampling on the pin.
void disableDigitalInput(uint8_t pin);

// high-level method for reading the input value of the specified pin.  If the voltage on the pin is low,
// this method will return 0.  Otherwise, it will return a non-zero result that depends on the value of
// the pin.
uint8_t isDigitalInputHigh(uint8_t pin);



/************** ANALOGUE *****************/

// set the ADC to run in either 8-bit mode (MODE_8_BIT) or
// 10-bit mode (MODE_10_BIT)
void setAnalogMode(uint8_t mode);

// returns 0 if in 10-bit mode, otherwise returns non-zero.  The return
// value of this method can be directly compared against the macros
// MODE_8_BIT and MODE_10_BIT:
// For example: if (getMode() == MODE_8_BIT) ...
uint8_t getAnalogMode(void);

// returns 1 if the ADC is in the middle of an conversion, otherwise
// returns 0
uint8_t analogIsConverting(void);

// the following method can be used to initiate an ADC conversion
// that runs in the background, allowing the CPU to perform other tasks
// while the conversion is in progress.  The procedure is to start a
// conversion on a channel with startConversion(channel), and then
// poll isConverting in your main loop.  Once isConverting() returns
// a zero, the result can be obtained through a call to conversionResult().
// If use_internal_reference is set to true, the function will use the
// internal 1.1V voltage reference on the ATmega48/168/328 or the internal
// 2.56V voltage reference on the ATmega324/644/1284; otherwise, it uses
// the AVCC pin as a reference.
// *** NOTE ***: Some Orangutans and 3pis have their AREF pin connected directly to VCC.
//  On these Orangutans, you must not use the internal voltage reference as
//  doing so will short the internal reference voltage to VCC and could damage
//  the AVR.  It is safe to use the internal reference voltage on the
//  Orangutan SVP.

void startAnalogConversion( uint8_t channel, uint8_t use_internal_reference);

// returns the result of the previous ADC conversion.
uint16_t analogConversionResult(void);


#ifdef __cplusplus
}
#endif

#endif // digitalAnalog_h
