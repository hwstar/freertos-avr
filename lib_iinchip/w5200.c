/*
 * (c)COPYRIGHT
 * ALL RIGHT RESERVED
 *
 * FileName : w5200.c
  * -----------------------------------------------------------------
 */

#include <stdio.h>
#include <string.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h> // for wait function

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "spi.h"
#include "socket.h"

#ifdef portW5200		// Definition in freeRTOSBoardDefs.h w5200.h and w5200.c are used.

#include "w5200.h"

#ifdef __DEF_IINCHIP_DBG__
#include "lib_serial.h"
#endif

#ifdef __DEF_IINCHIP_PPP__
   #include "md5.h"
#endif

static uint8_t 	I_STATUS[MAX_SOCK_NUM];
static uint16_t SMASK[MAX_SOCK_NUM]; /**< Variable for Tx buffer MASK in each channel */
static uint16_t RMASK[MAX_SOCK_NUM]; /**< Variable for Rx buffer MASK in each channel */
static uint16_t SSIZE[MAX_SOCK_NUM]; /**< Max Tx buffer size by each channel */
static uint16_t RSIZE[MAX_SOCK_NUM]; /**< Max Rx buffer size by each channel */
static uint16_t SBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Tx buffer base address by each channel */
static uint16_t RBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Rx buffer base address by each channel */

static un_l2cval SUBN_VAR; // off-chip subnet mask address - solve Errata 2 & 3 v1.6 - March 2012

uint8_t windowfull_retry_cnt[MAX_SOCK_NUM];

// TX MEM SIZE: Permissible values: 1:(1024) 2:(2048) 4:(4096) 8:(8192) 16:(16384)
// RX MEM SIZE: Total for 8 ports shall equal not more than 16
uint8_t txsize[MAX_SOCK_NUM] = {4,2,2,2,2,2,1,1};
uint8_t rxsize[MAX_SOCK_NUM] = {4,2,2,2,2,2,1,1};

uint16_t IINCHIP_read_buf( uint16_t addr, uint8_t *buf, uint16_t len);
uint16_t IINCHIP_write_buf(uint16_t addr, uint8_t *buf, uint16_t len);


uint8_t incr_windowfull_retry_cnt(uint8_t s)
{
  return windowfull_retry_cnt[s]++;
}

void init_windowfull_retry_cnt(uint8_t s)
{
  windowfull_retry_cnt[s] = 0;
}

uint16_t pre_sent_ptr, sent_ptr;

uint8_t IINCHIP_getISR(uint8_t s)
{
	return I_STATUS[s];
}
void IINCHIP_putISR(uint8_t s, uint8_t val)
{
   I_STATUS[s] = val;
}
uint16_t getIINCHIP_RxMAX(uint8_t s)
{
   return RSIZE[s];
}
uint16_t getIINCHIP_TxMAX(uint8_t s)
{
   return SSIZE[s];
}
uint16_t getIINCHIP_RxMASK(uint8_t s)
{
   return RMASK[s];
}
uint16_t getIINCHIP_TxMASK(uint8_t s)
{
   return SMASK[s];
}
uint16_t getIINCHIP_RxBASE(uint8_t s)
{
   return RBUFBASEADDRESS[s];
}
uint16_t getIINCHIP_TxBASE(uint8_t s)
{
   return SBUFBASEADDRESS[s];
}



 /**
@brief	This function writes the data into W5200 registers.
*/

uint8_t IINCHIP_write(uint16_t addr, uint8_t data)
{
	uint8_t Byte;

	IINCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(IINCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addr & 0xFF00) >> 8);	// load the upper address to be transmitted
	Byte = (uint8_t)(addr & 0x00FF);		// pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the lower address to be transmitted
	Byte = 0x80;					// pre-load the Data Write command and Write data length 14-8
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data Write command and Write data length 14-8
	Byte = 0x01;					// pre-load the Write data length 7-0
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Write data length 7-0
	Byte = data;					// pre-load the byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);			// CS=1, SPI end, give semaphore

    IINCHIP_ISR_ENABLE();			// Interrupt Service Routine Enable

	return 1;
}


/**
@brief	This function reads the value from W5200 registers.
*/
uint8_t IINCHIP_read(uint16_t addr)
{
	uint8_t Byte;

	IINCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(IINCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addr & 0xFF00) >> 8);	// load the upper address to be transmitted
	Byte = (uint8_t)(addr & 0x00FF); 		// pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the lower address to be transmitted
	Byte = 0x00;					// pre-load the Data Read command and Read data length 14-8
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data Read command and Read data length 14-8
	Byte = 0x01;					// pre-load the Data Read data length 7-0
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load the Data Read data length 7-0
	Byte = 0x00;					// pre-load a dummy byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;					// load a dummy byte to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	Byte = SPDR;					// copy received byte in case we get swapped out, and lose SPDR.

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);			// CS=1, SPI end, give semaphore

	IINCHIP_ISR_ENABLE();

	return Byte;
}


/**
@brief	This function writes into W5200 memory(Buffer)
*/
uint16_t IINCHIP_write_buf(uint16_t addr, uint8_t * buf, uint16_t len)
{
	uint16_t idx;
	uint8_t Byte;

	IINCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(IINCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addr & 0xFF00) >> 8);		// load the upper address to be transmitted
	Byte = (uint8_t)(addr & 0x00FF);			// pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;									// load the lower address to be transmitted
	Byte = 0x80 | (uint8_t)((len & 0x7F00) >> 8);	// pre-load the Data Write command and Write data length 14-8
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the Data Write command and Write data length 14-8
	Byte = (uint8_t)(len & 0x00FF);				// pre-load the Write data length 7-0
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the Write data length 7-0

	idx = 0;
	while(idx < len)							// Write data in loop
	{
		Byte = buf[ idx++ ];					// pre-load the byte to be transmitted
		while ( !(SPSR & _BV(SPIF)) );
		SPDR = Byte;							// load the byte to be transmitted
	}
	while ( !(SPSR & _BV(SPIF)) );

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);						// CS=1, SPI end, give semaphore

    IINCHIP_ISR_ENABLE();						// Interrupt Service Routine Enable

	return len;
}


/**
@brief	This function reads from W5200 memory(Buffer)
*/
uint16_t IINCHIP_read_buf(uint16_t addr, uint8_t * buf,uint16_t len)
{
	uint16_t idx;
	uint8_t Byte;

	IINCHIP_ISR_DISABLE();

	spiSelect(Wiznet);							// CS=0, get semaphore, SPI start

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(IINCHIP_SPI_DIVIDER);	// SPI at maximum speed

	// Make sure you manually pull slave select low to indicate start of transfer.
	// That is NOT done by this function..., because...
	// Some devices need to have their SS held low across multiple transfer calls.
	// Using spiSelect (SS_pin);

	// If the SPI module has not been enabled yet, then return with nothing.
	if ( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte.  This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	// Check this once only at the start. Assume that things don't change.
	if ( !(SPCR & _BV(MSTR)) ) SPCR |= _BV(MSTR);
	if ( !(SPCR & _BV(MSTR)) ) return 0;

	portENTER_CRITICAL();

	SPDR = (uint8_t)((addr & 0xFF00) >> 8);		// load the upper address to be transmitted
	Byte = (uint8_t)(addr & 0x00FF);			// pre-load the lower address to be transmitted
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the lower address to be transmitted
	Byte = (uint8_t)((len & 0x7F00) >> 8);		// pre-load the Data Read command and Read data length 14-8
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the Data Read command and Read data length 14-8
	Byte = (uint8_t)(len & 0x00FF);				// pre-load the Data Read data length 7-0
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// load the Data Read data length 7-0
	Byte = 0x5A;								// pre-load the dummy byte.
	while ( !(SPSR & _BV(SPIF)) );

	SPDR = Byte;								// Begin dummy transmission

	idx = 0;
	while (idx < len - 1)
	{
		while ( !(SPSR & _BV(SPIF)) );
		Byte = SPDR;							// copy received byte
		SPDR = 0x5A;							// Continue dummy transmission
		buf [ idx++ ] = Byte;
	}
	while ( !(SPSR & _BV(SPIF)) );

	buf [ idx ] = SPDR;							// store the last byte that was read

	portEXIT_CRITICAL();

	spiDeselect(Wiznet);						// CS=1, SPI end, give semaphore

	IINCHIP_ISR_ENABLE();                       // Interrupt Service Routine Enable

	return len;
}


/**
@brief	This function is for resetting of the W5200. Initialises the iinchip to work in SPI mode
*/
void IINCHIP_init(void)
{

	_delay_ms(150); // AVR can't delay its boot long as the W5200 needs to (data sheet 150ms), so add 150ms wait before we fire up W5200.

	spiBegin(Wiznet);		// enable the EtherMega W5200

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(IINCHIP_SPI_DIVIDER);	// SPI at maximum speed

	IINCHIP_write(MR, MR_RST); // reset the W5200 chip.

	_delay_ms(150); // Delay after reset for the W5200 needs to be (data sheet) 150ms, so wait 150ms before we address W5200.

#ifdef __DEF_IINCHIP_DBG__
	xSerialPrintf_P(PSTR("\r\nMR       (0x%02x)\r\nVERSIONR (0x%02x)\r\nPSTATUS  (0x%02x)\r\n"), IINCHIP_read(MR), IINCHIP_read(VERSIONR), IINCHIP_read(PSTATUS));
#endif
}


/**
@brief	This function set the transmit & receive buffer size as per the channels is used

Maximum memory size for Tx, Rx in the W5200 is 16K Bytes,\n
In the range of 16KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and reception from non-allocated channel may cause some problems.\n
If the 16KBytes memory is already  assigned to certain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/
void IINCHIP_sysinit( uint8_t * tx_size, uint8_t * rx_size	)
{
	int16_t ssum,rsum;

#ifdef __DEF_IINCHIP_DBG__
	xSerialPrint_P(PSTR("IINCHIP_sysinit()\r\n"));
#endif

	ssum = 0;
	rsum = 0;

	SBUFBASEADDRESS[0] = (uint16_t)(__DEF_IINCHIP_MAP_TXBUF__);		/* Set base address of Tx memory for channel #0 */
	RBUFBASEADDRESS[0] = (uint16_t)(__DEF_IINCHIP_MAP_RXBUF__);		/* Set base address of Rx memory for channel #0 */

#ifdef __DEF_IINCHIP_DBG__
	xSerialPrint_P(PSTR("Channel : TX ADDR - SIZE | RX ADDR - SIZE\r\n"));
#endif

   for (uint8_t i = 0 ; i < MAX_SOCK_NUM; ++i)       // Set the size, masking and base address of Tx & Rx memory by each channel
	{
    IINCHIP_write((Sn_TXMEM_SIZE(i)),tx_size[i]);
    IINCHIP_write((Sn_RXMEM_SIZE(i)),rx_size[i]);

		SSIZE[i] = (int16_t)(0);
		RSIZE[i] = (int16_t)(0);

		if (ssum <= 16384)
		{
         switch( tx_size[i] )
			{
			case 1:
				SSIZE[i] = (int16_t)(1024);
				SMASK[i] = (uint16_t)(0x03FF);
				break;
			case 2:
				SSIZE[i] = (int16_t)(2048);
				SMASK[i] = (uint16_t)(0x07FF);
				break;
			case 4:
				SSIZE[i] = (int16_t)(4096);
				SMASK[i] = (uint16_t)(0x0FFF);
				break;
			case 8:
				SSIZE[i] = (int16_t)(8192);
				SMASK[i] = (uint16_t)(0x1FFF);
				break;
			case 16:
				SSIZE[i] = (int16_t)(16384);
				SMASK[i] = (uint16_t)(0x3FFF);
				break;
			}
		}

		if (rsum <= 16384)
		{
         switch( rx_size[i] )
			{
			case 1:
				RSIZE[i] = (int16_t)(1024);
				RMASK[i] = (uint16_t)(0x03FF);
				break;
			case 2:
				RSIZE[i] = (int16_t)(2048);
				RMASK[i] = (uint16_t)(0x07FF);
				break;
			case 4:
				RSIZE[i] = (int16_t)(4096);
				RMASK[i] = (uint16_t)(0x0FFF);
				break;
			case 8:
				RSIZE[i] = (int16_t)(8192);
				RMASK[i] = (uint16_t)(0x1FFF);
				break;
			case 16:
				RSIZE[i] = (int16_t)(16384);
				RMASK[i] = (uint16_t)(0x3FFF);
				break;
			}
		}
		ssum += SSIZE[i];
		rsum += RSIZE[i];

        if (i != 0)             // Sets base address of Tx and Rx memory for channel #1,#2,#3
		{
			SBUFBASEADDRESS[i] = SBUFBASEADDRESS[i-1] + SSIZE[i-1];
			RBUFBASEADDRESS[i] = RBUFBASEADDRESS[i-1] + RSIZE[i-1];
		}
#ifdef __DEF_IINCHIP_DBG__
		xSerialPrintf_P(PSTR("     %d  :    %.4x - %.4x |    %.4x - %.4x\r\n"), i, (uint16_t)SBUFBASEADDRESS[i], SSIZE[i], (uint16_t)RBUFBASEADDRESS[i], RSIZE[i]);
		_delay_ms(5);
		#endif
	}
}


void setMR(uint8_t val)
{
	/* 	DIRECT ACCESS	*/
	IINCHIP_write(MR,val);
}


/**
@brief	This function sets up gateway IP address.
*/
void setGAR(
	uint8_t * addr	/**< a pointer to a 4 -byte array responsible to set the Gateway IP address. */
	)
{
	IINCHIP_write((GAR0 + 0),addr[0]);
	IINCHIP_write((GAR0 + 1),addr[1]);
	IINCHIP_write((GAR0 + 2),addr[2]);
	IINCHIP_write((GAR0 + 3),addr[3]);
}

/**
@brief	It sets up SubnetMask address
*/
void saveSUBR(
		un_l2cval * addr	/**< a pointer to a 4 -byte array responsible to set the SubnetMask address */
	)
{
	// write to off-chip subnet mask address - solve Errata 2 & 3 v1.6
	// Basically the hardware ARP engine is broken, unless it is set to 0.0.0.0
	// so we have to keep it so, unless we're using TCP connect() or UDP sendto()
	SUBN_VAR.lVal = addr->lVal;
}


/**
@brief	It sets up SubnetMask address
*/
void setSUBR(
	void
	)
{
	// apply off-chip subnet mask address - solve Errata 2 & 3 v1.6
	IINCHIP_write((SUBR0 + 0), SUBN_VAR.cVal[0]);
	IINCHIP_write((SUBR0 + 1), SUBN_VAR.cVal[1]);
	IINCHIP_write((SUBR0 + 2), SUBN_VAR.cVal[2]);
	IINCHIP_write((SUBR0 + 3), SUBN_VAR.cVal[3]);
}


/**
@brief	It sets up SubnetMask address
*/
void clearSUBR(
	void
	)
{
	// clear on-chip subnet mask address - solve Errata 2 & 3 v1.6
	IINCHIP_write((SUBR0 + 0), 0);
	IINCHIP_write((SUBR0 + 1), 0);
	IINCHIP_write((SUBR0 + 2), 0);
	IINCHIP_write((SUBR0 + 3), 0);
}

void getSUBR(un_l2cval *addr)
{
	addr->lVal = SUBN_VAR.lVal;
}


/**
@brief	This function sets up MAC address.
*/
void setSHAR(
	uint8_t * addr	/**< a pointer to a 6 -byte array responsible to set the MAC address. */
	)
{
	IINCHIP_write((SHAR0 + 0), addr[0]);
	IINCHIP_write((SHAR0 + 1), addr[1]);
	IINCHIP_write((SHAR0 + 2), addr[2]);
	IINCHIP_write((SHAR0 + 3), addr[3]);
	IINCHIP_write((SHAR0 + 4), addr[4]);
	IINCHIP_write((SHAR0 + 5), addr[5]);
}


/**
@brief	This function sets up Source IP address.
*/
void setSIPR(
	uint8_t * addr	/**< a pointer to a 4 -byte array responsible to set the Source IP address. */
	)
{
	IINCHIP_write((SIPR0 + 0),addr[0]);
	IINCHIP_write((SIPR0 + 1),addr[1]);
	IINCHIP_write((SIPR0 + 2),addr[2]);
	IINCHIP_write((SIPR0 + 3),addr[3]);
}


/**
@brief	This function gets Interrupt register in common register.
 */
uint8_t getIR( void )
{
   return IINCHIP_read(IR);
}


/**
 Retransmission
 **/

/**
@brief	This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission
will be there as per RTR (Retry Time-value Register) setting
*/
void setRTR(uint16_t timeout)
{
	IINCHIP_write(RTR0,(uint8_t)((timeout & 0xff00) >> 8));
	IINCHIP_write(RTR1,(uint8_t)(timeout & 0x00ff));
}


/**
@brief	This function set the number of Retransmission.

If there is no response from the peer or delay in response then recorded time
as per RTR & RCR register setting then time out will occur.
*/
void setRCR(uint8_t retry)
{
	IINCHIP_write(RCR,retry);
}


/**
@brief	This function set the interrupt mask Enable/Disable appropriate Interrupt. ('1' : interrupt enable)

If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
set in IR register.
*/
void setIMR(uint8_t mask)
{
	IINCHIP_write(IMR,mask); // must be set to 0x10.
}


/**
@brief	These below functions are used to get the Gateway, SubnetMask
		and Source Hardware Address (MAC Address) and Source IP address
*/
void getGAR(uint8_t * addr)
{
	addr[0] = IINCHIP_read(GAR0+0);
	addr[1] = IINCHIP_read(GAR0+1);
	addr[2] = IINCHIP_read(GAR0+2);
	addr[3] = IINCHIP_read(GAR0+3);
}

/**
@brief	This function sets up MAC address.
*/
void getSHAR(uint8_t * addr)
{
	addr[0] = IINCHIP_read(SHAR0+0);
	addr[1] = IINCHIP_read(SHAR0+1);
	addr[2] = IINCHIP_read(SHAR0+2);
	addr[3] = IINCHIP_read(SHAR0+3);
	addr[4] = IINCHIP_read(SHAR0+4);
	addr[5] = IINCHIP_read(SHAR0+5);
}

void getSIPR(uint8_t * addr)
{
	addr[0] = IINCHIP_read(SIPR0+0);
	addr[1] = IINCHIP_read(SIPR0+1);
	addr[2] = IINCHIP_read(SIPR0+2);
	addr[3] = IINCHIP_read(SIPR0+3);
}



/**
@brief	This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(SOCKET s, uint16_t Sn_MSSR0)
{
	IINCHIP_write(Sn_MSSR0(s),(uint8_t)((Sn_MSSR0 & 0xff00) >> 8));
	IINCHIP_write(Sn_MSSR1(s),(uint8_t)(Sn_MSSR0 & 0x00ff));
}

void setSn_TTL(SOCKET s, uint8_t ttl)
{
   IINCHIP_write(Sn_TTL(s), ttl);
}


/**
@brief	These below function is used to setup the Protocol Field of IP Header when
		executing the IP Layer RAW mode.
*/
void setSn_PROTO(SOCKET s, uint8_t proto)
{
	IINCHIP_write(Sn_PROTO(s),proto);
}


/**
@brief	get socket interrupt status

These below functions are used to read the Interrupt & Socket Status register
*/
uint8_t getSn_IR(SOCKET s)
{
   return IINCHIP_read(Sn_IR(s));
}


/**
@brief	 get socket status
*/
uint8_t getSn_SR(SOCKET s)
{
   return IINCHIP_read(Sn_SR(s));
}


/**
@brief	get socket TX transmit free buffer size

This gives free buffer size of transmit buffer. This is the data size that user can transmit.
User should check this value first and control the size of transmitted data.
*/
uint16_t getSn_TX_FSR(SOCKET s)
{
	uint16_t val  = 0;
	uint16_t val1 = 0;
	do
	{
		val1 = IINCHIP_read(Sn_TX_FSR0(s));
		val1 = (val1 << 8) + IINCHIP_read(Sn_TX_FSR1(s));
      if (val1 != 0)
		{
   			val = IINCHIP_read(Sn_TX_FSR0(s));
   			val = (val << 8) + IINCHIP_read(Sn_TX_FSR1(s));
		}
	} while (val != val1);
   return val;
}


/**
@brief	 get socket RX received buffer size

This gives size of received data in receive buffer.
*/
uint16_t getSn_RX_RSR(SOCKET s)
{
	uint16_t val  = 0;
	uint16_t val1 = 0;
	do
	{
		val1 = IINCHIP_read(Sn_RX_RSR0(s));
		val1 = (val1 << 8) + IINCHIP_read(Sn_RX_RSR1(s));
      if(val1 != 0)
		{
   			val = IINCHIP_read(Sn_RX_RSR0(s));
   			val = (val << 8) + IINCHIP_read(Sn_RX_RSR1(s));
		}
	} while (val != val1);
   return val;
}


/**
@brief	 This function is being called by TCP send() and UDP & IP_RAW sendto() function.

This function reads the Tx write pointer register and after copy the data in buffer update the Tx write pointer
register. User should read upper byte first and lower byte later to get proper value.
*/
void send_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{

	uint16_t ptr;

	ptr = IINCHIP_read(Sn_TX_WR0(s));
	ptr = (ptr << 8) + IINCHIP_read(Sn_TX_WR1(s));

#ifdef __DEF_IINCHIP_DBG__
	xSerialPrintf_P(PSTR("ISR_TX: tx_ptr: %.4x tx_len: %.4x\r\n"), ptr, len);
#endif

	write_data(s, data, (uint8_t *)(ptr), len);
	ptr += len;

	IINCHIP_write(Sn_TX_WR0(s), (uint8_t)((ptr & 0xff00) >> 8));
	IINCHIP_write(Sn_TX_WR1(s), (uint8_t)(ptr & 0x00ff));
}


/**
@brief	This function is being called by TCP recv().

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.
*/
void recv_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{
	uint16_t ptr;
	ptr = IINCHIP_read(Sn_RX_RD0(s));
	ptr = ((ptr & 0x00ff) << 8) + IINCHIP_read(Sn_RX_RD1(s));

	#ifdef __DEF_IINCHIP_DBG__
	xSerialPrintf_P(PSTR("ISR_RX: rd_ptr: %.4x rd_len: %.4x\r\n"), ptr, len);
#endif

	read_data(s, (uint8_t *)ptr, data, len); // read data
	ptr += len;
	IINCHIP_write(Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));
	IINCHIP_write(Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));
}


/**
@brief	for copy the data from application buffer to Transmit buffer of the chip.

This function is being used for copy the data from application buffer to Transmit
buffer of the chip. It calculates the actual physical address where one has to write
the data in transmit buffer. Here also takes care of the condition that it exceeds
the Tx memory upper-bound of socket.
*/
void write_data(SOCKET s, volatile uint8_t * src, volatile uint8_t * dst, uint16_t len)
{
	uint16_t size;
	uint16_t dst_mask;
	uint8_t * dst_ptr;

	dst_mask = (uint16_t)dst & getIINCHIP_TxMASK(s);
	dst_ptr = (uint8_t *)(getIINCHIP_TxBASE(s) + dst_mask);

	if (dst_mask + len > getIINCHIP_TxMAX(s))
	{
		size = getIINCHIP_TxMAX(s) - dst_mask;
		IINCHIP_write_buf((uint16_t)dst_ptr, (uint8_t *)src, size);
		src += size;
		size = len - size;
		dst_ptr = (uint8_t *)(getIINCHIP_TxBASE(s));
		IINCHIP_write_buf((uint16_t)dst_ptr, (uint8_t *)src, size);
	}
	else
	{
		IINCHIP_write_buf((uint16_t)dst_ptr, (uint8_t *)src, len);
	}
}


/**
@brief	This function is being used for copy the data from Receive buffer of the chip to application buffer.

It calculate the actual physical address where one has to read
the data from Receive buffer. Here also take care of the condition that it exceeds
the Rx memory upper-bound of socket.
*/
void read_data(SOCKET s, volatile uint8_t * src, volatile uint8_t * dst, uint16_t len)
{
	uint16_t size;
	uint16_t src_mask;
	uint8_t * src_ptr;

	src_mask = (uint16_t)src & getIINCHIP_RxMASK(s);
	src_ptr = (uint8_t *)(getIINCHIP_RxBASE(s) + src_mask);

	if( (src_mask + len) > getIINCHIP_RxMAX(s) )
	{
		size = getIINCHIP_RxMAX(s) - src_mask;
		IINCHIP_read_buf((uint16_t)src_ptr, (uint8_t *)dst,size);
		dst += size;
		size = len - size;
		src_ptr = (uint8_t *)(getIINCHIP_RxBASE(s));
		IINCHIP_read_buf((uint16_t)src_ptr, (uint8_t *) dst,size);
	}
	else
	{
		IINCHIP_read_buf((uint16_t)src_ptr, (uint8_t *) dst,len);
	}
}


#ifdef __DEF_IINCHIP_PPP__
#define PPP_OPTION_BUF_LEN 64

uint8_t pppinit_in(uint8_t * id, uint8_t idlen, uint8_t * passwd, uint8_t passwdlen);


/**
@brief	make PPPoE connection
@return	1 => success to connect, 2 => Auth fail, 3 => timeout, 4 => Auth type not support

*/
uint8_t pppinit(uint8_t * id, uint8_t idlen, uint8_t * passwd, uint8_t passwdlen)
{
	uint8_t ret;
	uint8_t isr;

	// PHASE0. W5200 PPPoE(ADSL) setup
	// enable pppoe mode
	xSerialPrint_P(PSTR("-- PHASE 0. W5100 PPPoE(ADSL) setup process --\r\n\n"));

	spiSetDataMode(SPI_MODE0);					// Enable SPI function in mode 0, CPOL=0 CPHA=0
	spiSetClockDivider(IINCHIP_SPI_DIVIDER);	// SPI at maximum speed

	IINCHIP_write(MR,IINCHIP_read(MR) | MR_PPPOE);

	// open socket in pppoe mode
	isr = IINCHIP_read(Sn_IR(0));// first clear isr(0), W5200 at present time
	IINCHIP_write(Sn_IR(0),isr);

	IINCHIP_write(PTIMER,200); // 5sec timeout
	IINCHIP_write(PMAGIC,0x01); // magic number
	IINCHIP_write(Sn_MR(0),Sn_MR_PPPOE);
	IINCHIP_write(Sn_CR(0),Sn_CR_OPEN);

	/* wait to process the command... */
	while( IINCHIP_read(Sn_CR(0)) )
		;
	/* ------- */

	ret = pppinit_in(id, idlen, passwd, passwdlen);

	/* close ppp connection socket */
	close(0);
	/* ------- */

	return ret;
}


uint8_t pppinit_in(uint8_t * id, uint8_t idlen, uint8_t * passwd, uint8_t passwdlen)
{
	uint8_t loop_idx = 0;
	uint8_t isr = 0;
	uint8_t buf[PPP_OPTION_BUF_LEN];
	uint16_t len;
	uint8_t str[PPP_OPTION_BUF_LEN];
	uint8_t str_idx,dst_idx;

   // PHASE1. PPPoE Discovery
	// start to connect pppoe connection
	xSerialPrint_P(PSTR("-- PHASE 1. PPPoE Discovery process --"));
	xSerialPrint_P(PSTR(" ok\r\n\n"));
	IINCHIP_write(Sn_CR(0),Sn_CR_PCON);
	/* wait to process the command... */
	while( IINCHIP_read(Sn_CR(0)) ) ;

	_delay_ms(1000);

	loop_idx = 0;
	//check whether PPPoE discovery end or not
	while (!(IINCHIP_read(Sn_IR(0)) & Sn_IR_PNEXT))
	{
		xSerialPrintf(".");
		if (loop_idx++ == 10) // timeout
		{
			xSerialPrint_P(PSTR("timeout before LCP\r\n"));
			return 3;
		}
		_delay_ms(1000);
	}

   /* clear interrupt value*/
   IINCHIP_write(Sn_IR(0), 0xff);
   /*---*/

   // PHASE2. LCP process
	xSerialPrint_P(PSTR("-- PHASE 2. LCP process --"));

	// send LCP Request
	{
		// Magic number option
		// option format (type value + length value + data)
	   // write magic number value
		buf[0] = 0x05; // type value
		buf[1] = 0x06; // length value
		buf[2] = 0x01; buf[3] = 0x01; buf[4] = 0x01; buf[5]= 0x01; // data
		// for MRU option, 1492 0x05d4
		// buf[6] = 0x01; buf[7] = 0x04; buf[8] = 0x05; buf[9] = 0xD4;
	}
	send_data_processing(0, buf, 0x06);
	IINCHIP_write(Sn_CR(0),Sn_CR_PCR); // send request
	/* wait to process the command... */
	while( IINCHIP_read(Sn_CR(0)) ) ;

	_delay_ms(1000);

	while (!((isr = IINCHIP_read(Sn_IR(0))) & Sn_IR_PNEXT))
	{
		if (isr & Sn_IR_PRECV) // Not support option
		{
   /* clear interrupt value*/
         IINCHIP_write(Sn_IR(0), Sn_IR_PRECV);
   /*---*/
			len = getSn_RX_RSR(0);
			if ( len > 0 )
			{
				recv_data_processing(0, str, len);
				IINCHIP_write(Sn_CR(0),Sn_CR_RECV);
				/* wait to process the command... */
				while( IINCHIP_read(Sn_CR(0)) ) ;

				// for debug
				//xSerialPrintf("LCP proc len = %d\r\n", len); for (i = 0; i < len; i++) xSerialPrintf ("%02x ", str[i]); xSerialPrintf("\r\n");
				// get option length
				len = str[4]; len = ((len & 0x00ff) << 8) + str[5];
				len += 2;
				str_idx = 6; dst_idx = 0; // ppp header is 6 byte, so starts at 6.
				do
				{
					if ((str[str_idx] == 0x01) || (str[str_idx] == 0x02) || (str[str_idx] == 0x03) || (str[str_idx] == 0x05))
					{
						// skip as length of support option. str_idx+1 is option's length.
						str_idx += str[str_idx+1];
					}
					else
					{
						// not support option , REJECT
						memcpy((uint8_t *)(buf+dst_idx), (uint8_t *)(str+str_idx), str[str_idx+1]);
						dst_idx += str[str_idx+1]; str_idx += str[str_idx+1];
					}
				} while (str_idx != len);
	   			// for debug
	   			//xSerialPrintf("LCP dst proc\r\n"); for (i = 0; i < dst_idx; i++) xSerialPrintf ("%02x ", buf[i]); xSerialPrintf("\r\n");

	   			// send LCP REJECT packet
	   			send_data_processing(0, buf, dst_idx);
	   			IINCHIP_write(Sn_CR(0),Sn_CR_PCJ);
				/* wait to process the command... */
				while( IINCHIP_read(Sn_CR(0)) )	;
  			}
		}
		xSerialPrintf(".");
		if (loop_idx++ == 10) // timeout
		{
			xSerialPrint_P(PSTR("timeout after LCP\r\n"));
			return 3;
		}
		_delay_ms(1000);
	}
	xSerialPrint_P(PSTR(" ok\r\n\n"));

   /* clear interrupt value*/
   IINCHIP_write(Sn_IR(0), 0xff);

   /* clear receive buffer */
	len = getSn_RX_RSR(0);
	if ( len > 0 )
	{
		recv_data_processing(0, str, len);
		//xSerialPrintf("dummy proc len = %d\r\n", len); for (i = 0; i < len; i++) xSerialPrintf ("%02x ", str[i]); xSerialPrintf("\r\n");
		IINCHIP_write(Sn_CR(0),Sn_CR_RECV);
		while( IINCHIP_read(Sn_CR(0)) )
			;
	}
   /*---*/

	xSerialPrint_P(PSTR("-- PHASE 3. PPPoE(ADSL) Authentication mode --\r\n"));
	xSerialPrintf_P(PSTR("Authentication protocol : %.2x %.2x, "), IINCHIP_read(PATR0), IINCHIP_read(PATR0+1));

	loop_idx = 0;
	if (IINCHIP_read(PATR0) == 0xc0 && IINCHIP_read(PATR0+1) == 0x23)
	{
		xSerialPrintf_P(PSTR("PAP\r\n")); // in case of adsl normally supports PAP.
		// send authentication data
		// copy (idlen + id + passwdlen + passwd)
		buf[loop_idx] = idlen; loop_idx++;
		memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(id), idlen); loop_idx += idlen;
		buf[loop_idx] = passwdlen; loop_idx++;
		memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(passwd), passwdlen); loop_idx += passwdlen;
		send_data_processing(0, buf, loop_idx);
		IINCHIP_write(Sn_CR(0),Sn_CR_PCR);
		/* wait to process the command... */
		while( IINCHIP_read(Sn_CR(0)) ) ;

		_delay_ms(1000);
	}
	else if (IINCHIP_read(PATR0) == 0xc2 && IINCHIP_read(PATR0+1) == 0x23)
	{
		uint8_t chal_len;
		md5_ctx context;
		uint8_t  digest[16];

		len = getSn_RX_RSR(0);
		if ( len > 0 )
		{
			recv_data_processing(0, str, len);
			IINCHIP_write(Sn_CR(0),Sn_CR_RECV);
			/* wait to process the command... */
			while( IINCHIP_read(Sn_CR(0)) )	;

#ifdef __DEF_IINCHIP_DBG__
			xSerialPrint_P(PSTR("recv CHAP\r\n"));
			{
				int16_t i;

				for (i = 0; i < 32; i++)
					xSerialPrintf_P(PSTR("%02x "), str[i]);
			}
			xSerialPrintf_P(PSTR("\r\n"));
#endif
// str is C2 23 xx CHAL_ID xx xx CHAP_LEN CHAP_DATA
// index  0  1  2  3       4  5  6        7 ...

			memset(buf,0x00,64);
			buf[loop_idx] = str[3]; loop_idx++; // chal_id
			memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(passwd), passwdlen); loop_idx += passwdlen; //passwd
			chal_len = str[6]; // chal_id
			memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(str+7), chal_len); loop_idx += chal_len; //challenge
			buf[loop_idx] = 0x80;
#ifdef __DEF_IINCHIP_DBG__
			xSerialPrint_P(PSTR("CHAP proc d1\r\n"));
			{
				int16_t i;
				for (i = 0; i < 64; i++)
					xSerialPrintf_P(PSTR("%02x "), buf[i]);
			}
			xSerialPrint_P(PSTR("\r\n"));
#endif

			md5_init(&context);
			md5_update(&context, buf, loop_idx);
			md5_final(digest, &context);

#ifdef __DEF_IINCHIP_DBG__
			xSerialPrint_P(PSTR("CHAP proc d1\r\n"));
			{
				int16_t i;
				for (i = 0; i < 16; i++)
					xSerialPrintf_P(PSTR("%02x"), digest[i]);
			}
			xSerialPrint_P(PSTR("\r\n"));
#endif
			loop_idx = 0;
			buf[loop_idx] = 16; loop_idx++; // hash_len
			memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(digest), 16); loop_idx += 16; // hashed value
			memcpy((uint8_t *)(buf+loop_idx), (uint8_t *)(id), idlen); loop_idx += idlen; // id
			send_data_processing(0, buf, loop_idx);
			IINCHIP_write(Sn_CR(0),Sn_CR_PCR);
			/* wait to process the command... */
			while( IINCHIP_read(Sn_CR(0)) )	;

			_delay_ms(1000);
		}
	}
	else
	{
		xSerialPrint_P(PSTR("Not support\r\n"));
#ifdef __DEF_IINCHIP_DBG__
		xSerialPrintf_P(PSTR("Not support PPP Auth type: %.2x%.2x\r\n"),IINCHIP_read(PATR0), IINCHIP_read(PATR0+1));
#endif
		return 4;
	}
	xSerialPrintf("\r\n");

	xSerialPrint_P(PSTR("-- Waiting for PPPoE server's admission --"));
	loop_idx = 0;
	while (!((isr = IINCHIP_read(Sn_IR(0))) & Sn_IR_PNEXT))
	{
		if (isr & Sn_IR_PFAIL)
		{
   /* clear interrupt value*/
   IINCHIP_write(Sn_IR(0), 0xff);
   /*---*/
			xSerialPrint_P(PSTR("failed\r\nReinput id, password..\r\n"));
			return 2;
		}
		xSerialPrintf(".");
		if (loop_idx++ == 10) // timeout
		{
   /* clear interrupt value*/
   IINCHIP_write(Sn_IR(0), 0xff);
   /*---*/
			xSerialPrint_P(PSTR("timeout after PAP\r\n"));
			return 3;
		}
		_delay_ms(1000);
	}
   /* clear interrupt value*/
   IINCHIP_write(Sn_IR(0), 0xff);

   /* clear receive buffer */
	len = getSn_RX_RSR(0);
	if ( len > 0 )
	{
		recv_data_processing(0, str, len);
		//xSerialPrintf("dummy proc len = %d\r\n", len); for (i = 0; i < len; i++) xSerialPrintf ("%02x ", str[i]); xSerialPrintf("\r\n");
		IINCHIP_write(Sn_CR(0),Sn_CR_RECV);
		while( IINCHIP_read(Sn_CR(0)) )
			;
	}
   /*---*/

	xSerialPrint_P(PSTR("ok\r\n\n-- PHASE 4. IPCP process --"));
	// IP Address
	buf[0] = 0x03; buf[1] = 0x06; buf[2] = 0x00; buf[3] = 0x00; buf[4] = 0x00; buf[5] = 0x00;
	send_data_processing(0, buf, 6);
	IINCHIP_write(Sn_CR(0),Sn_CR_PCR);
	/* wait to process the command... */
	while( IINCHIP_read(Sn_CR(0)) )	;

	_delay_ms(1000);

	loop_idx = 0;
	while (1)
	{
		if (IINCHIP_read(Sn_IR(0)) & Sn_IR_PRECV)
		{
   /* clear interrupt value*/
   IINCHIP_write(Sn_IR(0), 0xff);
   /*---*/
			len = getSn_RX_RSR(0);
			if ( len > 0 )
			{
				recv_data_processing(0, str, len);
				IINCHIP_write(Sn_CR(0),Sn_CR_RECV);
				/* wait to process the command... */
				while( IINCHIP_read(Sn_CR(0)) )	;

	   			//for debug
	   			//xSerialPrintf("IPCP proc len = %d\r\n", len); for (i = 0; i < len; i++) xSerialPrintf ("%02x ", str[i]); xSerialPrintf("\r\n");
	   			str_idx = 6; dst_idx = 0;
	   			if (str[2] == 0x03) // in case of NAK
	   			{
	   				do
	   				{
	   					if (str[str_idx] == 0x03) // request only ip information
	   					{
	   						memcpy((uint8_t *)(buf+dst_idx), (uint8_t *)(str+str_idx), str[str_idx+1]);
	   						dst_idx += str[str_idx+1]; str_idx += str[str_idx+1];
	   					}
	   					else
	   					{
	   						// skip byte
	   						str_idx += str[str_idx+1];
	   					}
	   					// for debug
	   					//xSerialPrintf("s: %d, d: %d, l: %d", str_idx, dst_idx, len);
	   				} while (str_idx != len);
	   				send_data_processing(0, buf, dst_idx);
	   				IINCHIP_write(Sn_CR(0),Sn_CR_PCR); // send ipcp request
	   				/* wait to process the command... */
					while( IINCHIP_read(Sn_CR(0)) )	;

	   				_delay_ms(1000);
	   				break;
	   			}
			}
		}
		xSerialPrintf(".");
		if (loop_idx++ == 10) // timeout
		{
			xSerialPrint_P(PSTR("timeout after IPCP\r\n"));
			return 3;
		}
		_delay_ms(1000);
		send_data_processing(0, buf, 6);
		IINCHIP_write(Sn_CR(0),Sn_CR_PCR); //ipcp re-request
		/* wait to process the command... */
		while( IINCHIP_read(Sn_CR(0)) )	;
	}

	loop_idx = 0;
	while (!(IINCHIP_read(Sn_IR(0)) & Sn_IR_PNEXT))
	{
		xSerialPrint_P(PSTR("."));
		if (loop_idx++ == 10) // timeout
		{
			xSerialPrint_P(PSTR("timeout after IPCP NAK\r\n"));
			return 3;
		}
		_delay_ms(1000);
		IINCHIP_write(Sn_CR(0),Sn_CR_PCR); // send ipcp request
		/* wait to process the command... */
		while( IINCHIP_read(Sn_CR(0)) )	;
	}
   /* clear interrupt value*/
   IINCHIP_write(Sn_IR(0), 0xff);
   /*---*/
	xSerialPrint_P(PSTR("ok\r\n\n"));
	return 1;
	// after this function, User must save the pppoe server's mac address and pppoe session id in current connection
}


/**
@brief	terminate PPPoE connection
*/
uint8_t pppterm(uint8_t * mac, uint8_t * sessionid)
{
	uint16_t i;
	uint8_t isr;
#ifdef __DEF_IINCHIP_DBG__
	xSerialPrint_P(PSTR("pppterm()\r\n"));
#endif
	/* Set PPPoE bit in MR(Common Mode Register) : enable socket0 pppoe */
	IINCHIP_write(MR,IINCHIP_read(MR) | MR_PPPOE);

	// write pppoe server's mac address and session id
	// must be setted these value.
	for (i = 0; i < 6; i++) IINCHIP_write((Sn_DHAR0(0)+i),mac[i]);
	for (i = 0; i < 2; i++) IINCHIP_write((Sn_DPORT0(0)+i),sessionid[i]);
	isr = IINCHIP_read(Sn_IR(0));
	IINCHIP_write(Sn_IR(0),isr);

	//open socket in pppoe mode
	IINCHIP_write(Sn_MR(0),Sn_MR_PPPOE);
	IINCHIP_write(Sn_CR(0),Sn_CR_OPEN);
	/* wait to process the command... */
	while( IINCHIP_read(Sn_CR(0)) )	;

	_delay_ms(1);
	// close pppoe connection
	IINCHIP_write(Sn_CR(0),Sn_CR_PDISCON);
	/* wait to process the command... */
	while( IINCHIP_read(Sn_CR(0)) )	;

	_delay_ms(1000);
	/* close socket */
	close(0);
	/* ------- */


#ifdef __DEF_IINCHIP_DBG__
	xSerialPrint_P(PSTR("pppterm() end ..\r\n"));
#endif

	return 1;
}
#endif

#endif
