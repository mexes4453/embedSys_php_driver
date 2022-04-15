// UART Initialization

#ifndef PHP_UART_H
#define PHP_UART_H


#include <stdint.h>         /* Standard integers. WG14/N843 C99 Standard */
#include "TM4C123GH6PM.h"
#include "php.h"

//--- User Manual: See pg 893-897
//--- pg. 918, 916
//--- pg.911 - flag reg
//--- pg.906 data reg
//--- initialization and configuration pg 902

#define UART_RXFE (1U<<4)         //  Flag Register -> Bit field
#define UART_TXFF (1U<<5)         //  Flag Register -> Bit field
#define UART_BAUDRATE_PC 115200   //  Transmission rate bit/sec
#define UART_BAUDRATE_WIFI 115200   //  Transmission rate bit/sec
#define UART_CTL_UARTEN 1U
#define UART_LCRH_WLEN_8  0x60

#define UART0_PORT GPIOA
#define UART1_PORT GPIOB

#define NVIC_PRI1_R            (*((volatile unsigned long *)0xE000E404))
#define NVIC_EN0_R             (*((volatile unsigned long *)0xE000E110))



/* UART is configured with flag to indicated interrupt mode
 * @param int: interrupt mode flag (0 -> disable interrupt configuration)
 *                                 (1 -> enable interrupt configuration)
 * @param int: interrupt priority (0-7)
 *
 * @return: void
 * */

void PHP_UART0_Init(int, int);  // Port A
void PHP_UART1_Init(int, int);  // Port B



char PHP_UART_Rx(UART0_Type *);
void PHP_UART_Tx(UART0_Type *, unsigned char);
void PHP_UART_TxString(UART0_Type *, char *str);


#endif // PHP_UART_H
