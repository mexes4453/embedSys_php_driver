#ifndef PHP_I2C_H
#define PHP_I2C_H

/*
 * Author: Udoh Chiemezie Albert
 * Date  : 12.09.2021
 *
 * This code runs on TM4C Tiva Launchpad from Texas instrument
 * it configures the launchpad to use its i2c to communicate with
 * external devices.
 *
 * All functions declarations are defined in the .c file
 *
 * */

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"
#include "misc.h"

// I2C - INTER INTEGRATED CIRCUIT DATA TRANSMISSION -   Pg.997
/*
	I2C Initialisation -
	Configuration macro has been defined for I2C_3
	Transmission speed - standard (100kHz)


*/

// Pg.236  -> SYSTEM CONTROL REGISTER MAP
// Pg.1015 -> Initialisation and configuration


#define I2C_SLAVE_ADDR           (0x68)
#define I2C_BUS_FREQ             (100000)          // 100KHz


/* I2C PORT */
#define I2C_0_GPIO_PORT          GPIOB
#define I2C_1_GPIO_PORT          GPIOA
#define I2C_2_GPIO_PORT          GPIOB
#define I2C_3_GPIO_PORT          GPIOD


/* I2C PORT CLOCK */
#define I2C_0_GPIO_PORT_CLK_EN   (1U<<1)
#define I2C_1_GPIO_PORT_CLK_EN   (1U<<0)
#define I2C_2_GPIO_PORT_CLK_EN   (1U<<1)
#define I2C_3_GPIO_PORT_CLK_EN   (1U<<3)


/* I2C GATING CLOCK */
#define RCGC1_I2C_0_CLK_EN_BIT   (1U<<0)
#define RCGC1_I2C_1_CLK_EN_BIT   (1U<<1)
#define RCGC1_I2C_2_CLK_EN_BIT   (1U<<2)
#define RCGC1_I2C_3_CLK_EN_BIT   (1U<<3)


/* I2C PINS */
// I2C_0_PINS
#define PIN_I2C_0_CLK            (1U<<2)   // PB2 - CLOCK LINE
#define PIN_I2C_0_SDA            (1U<<3)   // PB3 - DATA LINE
#define I2C_0_ALL_PINS           (PIN_I2C_0_CLK | PIN_I2C_0_SDA)

// I2C_1_PINS
#define PIN_I2C_1_CLK            (1U<<6)   // PA6 - CLOCK LINE
#define PIN_I2C_1_SDA            (1U<<7)   // PA7 - DATA LINE
#define I2C_1_ALL_PINS           (PIN_I2C_1_CLK | PIN_I2C_1_SDA)

// I2C_2_PINS
#define PIN_I2C_2_CLK            (1U<<4)   // PB4 - CLOCK LINE
#define PIN_I2C_2_SDA            (1U<<5)   // PB5 - DATA LINE
#define I2C_2_ALL_PINS           (PIN_I2C_2_CLK | PIN_I2C_2_SDA)

// I2C_3_PINS
#define PIN_I2C_3_CLK            (1U<<0)   // PD0 - CLOCK LINE
#define PIN_I2C_3_SDA            (1U<<1)   // PD1 - DATA LINE
#define I2C_3_ALL_PINS           (PIN_I2C_3_CLK | PIN_I2C_3_SDA)


/* I2C PORT CTRL - Pg.688, 1351 */
// I2C_0_PORT_CTRL
#define PCTL_I2C_0_CLK           (3U<<8)   // Pg.688, 1351 GPIO Port control - PB2
#define PCTL_I2C_0_SDA           (3U<<12)  // Pg.688, 1351 GPIO Port control - PB3
#define PCTL_I2C_0_CLK_SDA       (PCTL_I2C_0_CLK | PCTL_I2C_0_SDA)
#define PCTL_I2C_0_CLK_SDA_BITS  (0x0000FF00)

// I2C_1_PORT_CTRL
#define PCTL_I2C_1_CLK           (3U<<24)  // Pg.688, 1351 GPIO Port control - PA6
#define PCTL_I2C_1_SDA           (3U<<28)  // Pg.688, 1351 GPIO Port control - PA7
#define PCTL_I2C_1_CLK_SDA       (PCTL_I2C_1_CLK | PCTL_I2C_1_SDA)
#define PCTL_I2C_1_CLK_SDA_BITS  (0xFF000000)

// I2C_1_PORT_CTRL
#define PCTL_I2C_2_CLK           (3U<<16)  // Pg.688, 1351 GPIO Port control - PB4
#define PCTL_I2C_2_SDA           (3U<<20)  // Pg.688, 1351 GPIO Port control - PB5
#define PCTL_I2C_2_CLK_SDA       (PCTL_I2C_2_CLK | PCTL_I2C_2_SDA)
#define PCTL_I2C_2_CLK_SDA_BITS  (0x00FF0000)

// I2C_3_PORT_CTRL
#define PCTL_I2C_3_CLK           (3U<<0)   // Pg.688 GPIO Port control
#define PCTL_I2C_3_SDA           (3U<<4)   // Pg.688 GPIO Port control
#define PCTL_I2C_3_CLK_SDA       (PCTL_I2C_3_CLK | PCTL_I2C_3_SDA)
#define PCTL_I2C_3_CLK_SDA_BITS  (0x00000FF)


// I2C MASTER DATA REGISTER
#define I2C_0_MDR                        (*((volatile unsigned long  *)0x40020008))
#define I2C_1_MDR                        (*((volatile unsigned long  *)0x40021008))
#define I2C_2_MDR                        (*((volatile unsigned long  *)0x40022008))
#define I2C_3_MDR                        (*((volatile unsigned long  *)0x40023008))


/* I2C CONFIGURATION BITS */
#define I2C_GPIO_PORT            I2C_1_GPIO_PORT
#define I2C_GPIO_PORT_CLK_EN     I2C_1_GPIO_PORT_CLK_EN
#define RCGC1_I2C_CLK_EN         RCGC1_I2C_1_CLK_EN_BIT
#define PIN_I2C_CLK              PIN_I2C_1_CLK
#define PIN_I2C_SDA              PIN_I2C_1_SDA
#define I2C_ALL_PINS             I2C_1_ALL_PINS
#define PCTL_I2C_CLK             PCTL_I2C_1_CLK
#define PCTL_I2C_SDA             PCTL_I2C_1_SDA
#define PCTL_I2C_CLK_SDA         PCTL_I2C_1_CLK_SDA
#define PCTL_I2C_CLK_SDA_BITS    PCTL_I2C_1_CLK_SDA_BITS
#define I2C_MDR_R                I2C_1_MDR

#define I2C_MCR_MFE_BIT          (1U<<4)   // I2C Master Configuration register - Master Function Enable bit
#define I2C_MTPR_HS              (1U<<7)   // I2C High speed Enable bit - Pg.1026
#define I2C_MCS_BUSY             (1U<<0)   // I2C Master control status register - busy bit - Pg.1020
#define I2C_MCS_ERR_BITS         (0x0E)    // I2C Master control status register - bits[3:1] - pg.1021
#define I2C_MCS_BUSBSY_BIT       (1U<<6)   // I2C Master control status register - Bus Busy bit - pg.1020/1
#define I2C_MSA_RS_BIT           (1U<<0)   // I2C Master slave address register - Receive/send bit Pg.1019



extern uint32_t sysClkFreq;   // system clock frequency variable defined in php.h file

void PHP_I2C_Init(I2C0_Type *);


/*------------------------------------------ PHP_I2C_Wait  -------------------------------------*
 * This function polls busy bit of the master control status during data transmission
 * Therefore, waiting till the ongoing I2C transmission is complete.
 * The error bits are returned.
 *
 * Note that the data ack and address ack are included
 * data ack = 1; means that there is no ack. No response from slave or slave wants no more data
 * addr ack = 1; means that there is no ack. No response from slave at all - no connection
 * error    = 1; means that there was an error during transmission
 *
 * @input : pointer to I2C Type
 *
 * @return : returns an int type which indicate an Error or NACK
 *
 * ----------------------------------------------------------------------------------------------*/
int PHP_I2C_Wait(I2C0_Type *);



/*-----------------------------------   PHP_I2C_ByteWrite  ---------------------------------------*
 * This function transmitted one byte of data given the slave address and data to be transmitted
 * one byte of data is transmitted to the slave at any given time
 * it also check for error by polling bits as defined in PHP_I2C_Wait function.
 * @input : (I2C0_Type *) -> pointer to I2C Type
 * @input : int           -> slave address
 * @input : char          -> memory address
 * @input : char          -> data (byte to be transmitted)
 *
 * @return: char          -> return ERROR
 * ----------------------------------------------------------------------------------------------*/
char PHP_I2C_ByteWrite(I2C0_Type *, int, char, char);



/*-----------------------------------   PHP_I2C_BurstWrite  ---------------------------------------*
 * This function transmitted bytes of data consecutively to the slave device memory locations given
 * the slave address and first memory location on slave device.
 * Address of the first memory location on slave device is provided, followed by the data for that
 * location.
 *
 * Master generate START condition
 * Master transmits slave address and indicates write operation bit[0] = 0
 * Master transmits the address of the first memory location on slave device
 * Master transmits data for first location and subsequently provides byte data for the consecutive
 *     memory location
 * The master generate STOP condition
 *
 * it also check for error by polling bits as defined in PHP_I2C_Wait function.
 *
 * DATA-FRAME: START-(SLAVE_ADDR + W)-ACK-MEM_ADDR-ACK-DATA-ACK...-DATA-NACK-STP
 * ----------
 *
 * @input : (I2C0_Type *) -> pointer to I2C Type
 * @input : int           -> slave address
 * @input : char          -> mem address
 * @input : int           -> byteCount (no of memory location to transmit data to)
 * @input : char          -> data (byte to be transmitted)
 *
 * @return: char          -> return ERROR
 * ----------------------------------------------------------------------------------------------*/
char PHP_I2C_BurstWrite(I2C0_Type * I2C, int, char, int, char *);



/*-----------------------------------   PHP_I2C_ByteRead  ---------------------------------------*
 * This function receive one byte of data given the slave address and data pointer to store data
 * one byte of data is received from the slave at any given time
 * it also check for error by polling bits as defined in PHP_I2C_Wait function.
 * @input : (I2C0_Type *) -> pointer to I2C Type
 * @input : int           -> slave address
 * @input : char*          -> Pointer to store received data
 *
 * @return: char          -> return ERROR
 * ----------------------------------------------------------------------------------------------*/
char PHP_I2C_ByteRead(I2C0_Type * I2C, int slaveAddr, char memAddr, char *data);



/*-----------------------------------   PHP_I2C_BurstRead  ---------------------------------------*
 * This function receives bytes of data consecutively from the slave device memory locations given
 * the slave address and first memory location on slave device.
 * Address of the first memory location on slave device is provided, followed by the data for that
 * location.
 *
 * It also check for error by polling bits as defined in PHP_I2C_Wait function.
 *
 * Master generate START condition
 * Master transmits slave address and indicates write operation bit[0] = 0 (Write address)
 * Master transmits the address of the first memory location on slave device
 * Master generates a restart condition to switch bus direction from write to read
 * Master clocks and wait for slave to provide data from first location
 * Master provides ACK
 * Master reads the consecutive locations and provides ACK for each byte
 * Master provides NACK after receiving the last byte to signal that slave read is completed
 * The master generate STOP condition
 *
 * DATA-FRAME:
 * START-(SLAVE_ADDR + W)-ACK-MEM_ADDR-ACK-(RPT_START)-(SLAVE_ADDR + R)-ACK-DATA-ACK...-DATA-NACK-STP
 *
 * @input : (I2C0_Type *) -> pointer to I2C Type
 * @input : int           -> slave address
 * @input : char          -> mem address
 * @input : int           -> byteCount (no of memory location to transmit data to)
 * @input : char *        -> Pointer to store received data
 *
 * @return: char          -> return ERROR
 * ----------------------------------------------------------------------------------------------*/
char PHP_I2C_BurstRead(I2C0_Type * I2C, int slaveAddr, char memAddr, int byteCount, char *data);


#endif // PHP_I2C_H



