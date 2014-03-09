/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

#ifndef LIB_SERIAL_H
#define LIB_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/pgmspace.h>

#include "queue.h"
#include "portable.h"

#include "ringBuffer.h"

typedef enum
{
	USART0,
	USART1,
	USART2,
	USART3
} eCOMPort;

typedef enum
{
	VACANT,
	ENGAGED
} binary;

typedef struct
{
	eCOMPort usart;
	ringBuffer_t xRxedChars;
	ringBuffer_t xCharsForTx;
	uint8_t *serialWorkBuffer; // create a working buffer pointer, to later be malloc() on the heap.
	uint16_t serialWorkBufferSize; // size of working buffer as created on the heap.
	binary	serialWorkBufferInUse;	// flag to prevent overwriting by multiple tasks using the same USART.
} xComPortHandle, * xComPortHandlePtr;


/* Create reference to the handle for the serial port, USART0. */
/* This variable is special, as it is used in the interrupt */
extern xComPortHandle xSerialPort;

#if defined(__AVR_ATmega324P__)  || defined(__AVR_ATmega644P__)|| defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) ||defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

/* Create reference to the handle for the other serial port, USART1. */
/* This variable is special, as it is used in the interrupt */
extern xComPortHandle xSerial1Port;

#endif

/*----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( eCOMPort ePort, uint32_t ulWantedBaud, uint16_t uxTxQueueLength, uint16_t uxRxQueueLength );
// xComPortHandle xSerialPortInit( eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits, unsigned portBASE_TYPE uxBufferLength );

void vSerialClose( xComPortHandlePtr oldComPortPtr );

/*-----------------------------------------------------------*/

// xSerialPrintf_P(PSTR("\r\nMessage %u %u %u"), var1, var2, var2);

/* If you are worried about race situations on the serial port, then you should implement
 * a semaphore to limit this case.
 * Since I just use the serial port for debugging mainly, there seems to be too much
 * overhead to build this semaphore into the print functions themselves.
 *
 * This is now alleviated by using xSerialPort.serialWorkBufferInUse in xSerialPrintf(_P)();
 */

/**
 * Serial printf.
 * @param format printf format string.
 */
void xSerialPrintf( const char * format, ...);

/**
 * Serial printf from PROGMEM to the serial port.
 * @param format printf format string.
 */
void xSerialPrintf_P(PGM_P format, ...);

/**
 * Print a character string to the serial port.
 * @param str string to print.
 */
void xSerialPrint( uint8_t * str) __attribute__ ((flatten));

/**
 * Print a character string from PROGMEM to the serial port.
 * @param str string to print.
 */
void xSerialPrint_P(PGM_P str) __attribute__ ((flatten));

// These can be set to use any USART (but only two implemented for now).
void xSerialxPrintf(xComPortHandlePtr pxPort, const char * format, ...);
void xSerialxPrintf_P(xComPortHandlePtr pxPort, PGM_P format, ...);
void xSerialxPrint(xComPortHandlePtr pxPort, uint8_t * str) __attribute__ ((flatten));
void xSerialxPrint_P(xComPortHandlePtr pxPort, PGM_P str) __attribute__ ((flatten));

/**
 * Interrupt driven routines to interface to ISR serial port IO.
 */
void xSerialFlush( xComPortHandlePtr pxPort ) __attribute__ ((flatten));
uint16_t xSerialAvailableChar( xComPortHandlePtr pxPort ) __attribute__ ((flatten));

UBaseType_t xSerialGetChar( xComPortHandlePtr pxPort, UBaseType_t *pcRxedChar ) __attribute__ ((flatten));
UBaseType_t xSerialPutChar( xComPortHandlePtr pxPort, const UBaseType_t cOutChar ) __attribute__ ((flatten));
/*-----------------------------------------------------------*/

// Polling write and read routines, for use before freeRTOS vTaskStartScheduler
// Same as arguments and function as above, but don't use the interrupts.

void avrSerialPrintf(const char * format, ...);
void avrSerialPrintf_P(PGM_P format, ...);

void avrSerialPrint(uint8_t * str) __attribute__ ((flatten));
void avrSerialPrint_P(PGM_P str) __attribute__ ((flatten));

void avrSerialWrite(int8_t DataOut);
int8_t avrSerialRead(void);

int8_t avrSerialCheckRxComplete(void);
int8_t avrSerialCheckTxReady(void);

// These can be set to use any USART (but only two implemented for now).

void avrSerialxPrintf(xComPortHandlePtr pxPort, const char * format, ...);
void avrSerialxPrintf_P(xComPortHandlePtr pxPort, PGM_P format, ...);

void avrSerialxPrint(xComPortHandlePtr pxPort, uint8_t * str) __attribute__ ((flatten));
void avrSerialxPrint_P(xComPortHandlePtr pxPort, PGM_P str) __attribute__ ((flatten));

void avrSerialxWrite(xComPortHandlePtr pxPort, int8_t DataOut);
int8_t avrSerialxRead(xComPortHandlePtr pxPort);

int8_t avrSerialxCheckRxComplete(xComPortHandlePtr pxPort);
int8_t avrSerialxCheckTxReady(xComPortHandlePtr pxPort);


/*-----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif
