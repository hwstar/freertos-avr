/*
*
@file		socket.h
@brief	define function of socket API
*
*/

#ifndef	_SOCKET_H_
#define	_SOCKET_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Scheduler include files. */
#include "FreeRTOS.h"

#ifdef portW5200
#include "w5200.h"
#else
#include "w5100.h"
#endif

/****** __DEF_IINCHIP_xxx__ 	   define option for W5100 / W5200 driver *****************/

// #define __DEF_IINCHIP_DBG__ 	/* involve debug code in driver (in socket.c) */
// #define __DEF_IINCHIP_DBG2__ /* involve debug other code in driver (in socket.c) */
// #define __DEF_IINCHIP_INT__ 	/* involve interrupt service routine (in socket.c) */
// #define __DEF_IINCHIP_PPP__ 	/* involve pppoe routine (in socket.c) */
                            	/* If it is defined, the source files(md5.h,md5.c) must be included in your project.
                               	   Otherwise, the source files must be removed in your project. */

/* SPI at maximum speed, half of CPU clock  */
/* Maximum speed doesn't work on a Goldilocks 22MHz board with Wiznet5100. Too fast! */
/* W5100 SPI is 4MHz maximum, or Clk/4 for standard Arduino and Clk/8 for standard Goldilocks */
/* W5200 SPI is 33.3MHz minimum, so anything should work*/
#if defined(portW5200)
#define IINCHIP_SPI_DIVIDER SPI_CLOCK_DIV2 // single point to control SPI speed when using W5200
#else
#define IINCHIP_SPI_DIVIDER SPI_CLOCK_DIV4 // single point to control SPI speed when using W5100
#endif

uint8_t  socket(SOCKET s, uint8_t protocol, uint16_t port, uint8_t flag); // Opens a socket(TCP, UDP, IPRAW, or MACRAW mode)
void     close(SOCKET s); // Close socket
uint8_t  connect(SOCKET s, uint8_t * addr, uint16_t port); // Establish TCP connection (Active connection)
void     disconnect(SOCKET s); // disconnect the connection
uint8_t  listen(SOCKET s);	// Establish TCP connection (Passive connection)

uint16_t send(SOCKET s, const uint8_t * buf, uint16_t len); // Send data (TCP)
uint16_t recv(SOCKET s,       uint8_t * buf, uint16_t len);	// Receive data (TCP)
uint16_t sendto(SOCKET s, const uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t port); // Send data (UDP/IP RAW)
uint16_t recvfrom(SOCKET s,     uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t * port); // Receive data (UDP/IP RAW)

uint16_t macrawsend(SOCKET s, uint8_t * buf, uint16_t len); // Send data (MAC RAW)
uint16_t macrawrecv(SOCKET s, uint8_t * buf, uint16_t len); // Receive data (MAC RAW)

/******************************** Utility Functions ************************************/

//#define NO_USE_SOCKUTIL_FUNC 				/* Don't compile the Utility functions */

uint16_t htons(uint16_t hostshort);			/* converts a uint16_t from host to TCP/IP network byte order (which is big-endian).*/
uint32_t htonl(uint32_t hostlong);			/* converts a uint32_t from host to TCP/IP network byte order (which is big-endian). */
uint32_t ntohs(uint16_t netshort);			/* converts a uint16_t from TCP/IP network byte order to host byte order (which is little-endian on Intel processors). */
uint32_t ntohl(uint32_t netlong);			/* converts a uint32_t from TCP/IP network order to host byte order (which is little-endian on Intel processors). */

uint8_t* inet_ntoa(uint32_t addr);			/* Convert 32bit Address into Dotted Decimal Format */


#ifndef NO_USE_SOCKUTIL_FUNC

uint8_t* inet_ntoa_pad(uint32_t addr);
uint32_t inet_addr( uint8_t* addr);			/* Converts a string containing an (Ipv4) Internet Protocol decimal dotted address into a 32bit address */
uint8_t  VerifyIPAddress(uint8_t* src);		/* Verify dotted notation IP address string */
uint32_t GetDestAddr(SOCKET s);				/* Output destination IP address of appropriate channel */
uint16_t GetDestPort(SOCKET s);				/* Output destination port number of appropriate channel */

uint8_t  CheckDestInLocal(uint32_t destip);			/* Check Destination in local or remote */
SOCKET   getSocket( uint8_t status, SOCKET start); 	/* Get handle of socket which status is same to 'status' */
uint16_t checksum( uint8_t * src, uint16_t len);	/* Calculate checksum of a stream */

uint32_t GetIPAddress(void);				/* Get Source IP Address of W5100. */
uint32_t GetGWAddress(void);				/* Get Source IP Address of W5100. */
uint32_t GetSubMask(void);					/* Get Source Subnet mask of W5100. */

void	GetMacAddress( uint8_t* mac);		/* Get Mac address of W5100. */
void	GetDestMacAddr( SOCKET s, uint8_t* mac);
void	GetNetConfig(void);					/* Read established network information(G/W, IP, S/N, Mac) of W5100 and Output that through Serial.*/

uint8_t D2C(uint8_t c); 					/* Convert HEX(0-F) to a character */
uint8_t C2D(uint8_t c); 					/* Convert a character to HEX */
uint16_t ATOI(uint8_t* str,uint16_t base); 			/* Convert a string to integer number */
uint16_t ValidATOI(uint8_t* str, uint16_t base, uint16_t* ret); 		/* Verify character string and Convert it to (hexa-)decimal. */
void replacetochar(uint8_t * str, uint8_t oldchar, uint8_t newchar); /* Replace old character with new character in the string */

#endif


#ifdef __cplusplus
}
#endif

#endif
/* _SOCKET_H_ */
