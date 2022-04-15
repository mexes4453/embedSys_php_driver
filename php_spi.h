#ifndef PHP_SPI_H
#define PHP_SPI_H

/*
 * Author: Udoh Chiemezie Albert
 * Date  : 12.09.2021
 *
 * This code runs on TM4C Tiva Launchpad from Texas instrument
 * it configures the launchpad to use its synchronous serial interface (SSI)
 * to communicate with external devices that communicate using Serial Protocol
 * Interface (SPI). Example. Nokia_5110 display screen.
 *
 * All functions declarations are defined in the .c file
 *
 * */

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"
#include "misc.h"

// SPI_SSI - SYNCHRONOUS SERIAL INTERFACE DATA TRANSMISSION -  SSI1/3 Pg.952
/*
	spi Initialisation	- PORT_D on Launch pad PD0-3


*/

// Pg.236 -> SYSTEM CONTROL REGISTER MAP
// Pg.965 -> Initialisation and configuration

// SSI- PORT Configuration SSI1
/*
#define SPI_GPIO_PORT GPIOD
#define GPIO_PORT_D_CLK_EN_BIT (1U<<3)

#define SSI_BLK                 (SSI1)
#define RCGC1_SSI1_CLK_EN_BIT   (1U<<5)
#define PIN_SSI1_CLK            (1U<<0)   // PD0 - CLOCK
#define PIN_SSI1_SS             (1U<<1)   // PD1 - SLAVE SELECT
#define PIN_SSI1_RX             (1U<<2)   // PD2 - MISO: MASTER IN SLAVE OUT
#define PIN_SSI1_TX             (1U<<3)   // PD3 - MOSI: MASTER OUT SLAVE IN
#define SSI1_ALL_PINS           (PIN_SSI1_CLK | PIN_SSI1_TX | PIN_SSI1_RX | PIN_SSI1_SS)

#define PCTL_SSI1_CLK           (2U<<0)   // Pg.688 GPIO Port control
#define PCTL_SSI1_RX            (2U<<8)   // Pg.688 GPIO Port control
#define PCTL_SSI1_TX            (2U<<12)  // Pg.688 GPIO Port control
#define PCTL_SSI1_CLK_RX_TX     (PCTL_SSI1_CLK | PCTL_SSI1_RX | PCTL_SSI1_TX)
#define PCTL_SSI1_CLK_RX_TX_BIT (0x0000FF0F)
*/

// SSI- PORT Configuration SSI0
#define SPI_GPIO_PORT GPIOA
#define GPIO_PORT_A_CLK_EN_BIT (1U<<0)

#define SSI_BLK                 (SSI0)
#define RCGC1_SSI0_CLK_EN_BIT   (1U<<4)
#define PIN_SSI0_CLK            (1U<<2)   // PA2 - CLOCK
#define PIN_SSI0_SS             (1U<<3)   // PA3 - SLAVE SELECT
#define PIN_SSI0_RX             (1U<<4)   // PA4 - MISO: MASTER IN SLAVE OUT
#define PIN_SSI0_TX             (1U<<5)   // PA5 - MOSI: MASTER OUT SLAVE IN
#define SSI0_ALL_PINS           (PIN_SSI0_CLK | PIN_SSI0_TX | PIN_SSI0_RX | PIN_SSI0_SS)

#define PCTL_SSI0_CLK           (2U<<8)   // Pg.688 GPIO Port control
#define PCTL_SSI0_SS            (2U<<12)  // Pg.688 GPIO Port control
#define PCTL_SSI0_RX            (2U<<16)   // Pg.688 GPIO Port control
#define PCTL_SSI0_TX            (2U<<20)  // Pg.688 GPIO Port control
#define PCTL_SSI0_CLK_RX_TX     (PCTL_SSI0_CLK | PCTL_SSI0_RX | PCTL_SSI0_TX )
#define PCTL_SSI0_CLK_RX_TX_BIT (0x00FF0F00)


// SSI- GENERAL Configuration
#define SSI_CR_SSE_BIT          (1U<<1)   // SSI Synchronous Serial Port Enable bit
#define SSI_CR_MS_BIT           (1U<<2)   // SSI Master/Slave select bit

#define SSI_CC_CS_BITS          (0x0F)    // SSI Baud clock source bits [3:0]. Pg.984
#define SSI_CC_SYS_CLK_SEL      (0)       // System clock (based on clock source and divisor factor)

#define SSI_CPSR_CPSDVSR_BITS   (0xFF)    // SSI Clock Prescale divisor bits[7:0] Pg.976

#define SSI_CR0_SCR_BITS        (0x0000FF00) // SSI Serial Clock rate bits[15:8] Pg.969
#define SSI_CR0_SPH_BIT         (1U<<7)      // SSI Serial Clock phase bits[7] Pg.969
#define SSI_CR0_SPO_BIT         (1U<<6)      // SSI Serial Clock polarity bits[6] Pg.969
#define SSI_CR0_FRF_BITS        (3U<<4)      // SSI Frame format select bits[5:4] Pg.970
#define SSI_CR0_DSS_BITS        (0x0F)       // SSI Data size select bits[3:0] Pg.970
#define SSI_CR0_DSS_8BIT_DATA   (0x07)


#define SSIX_SR_BSY_BIT         (1U<<4)      // Pg.974 SSI status reg, busy bit[4]
#define SSIX_SR_RNE_BIT         (1U<<2)      // Pg.974 SSI status reg, RECEIVE  FIFO not empty bit[4]
#define SSIX_SR_TFE_BIT         (1U<<0)      // Pg.974 SSI status reg, TRANSMIT FIFO empty bit[0]

typedef struct {
    // SSInClk = SysClk / (CPSDVSR * (1 + SCR)) Pg.954
    // SSInClk : Output clock frequency for SSI: Example, 3.67Mbps
    //         :SYSnCLK_Int       = 3
    //         :SYSnCLK_Frac      = 67
    //         :SYSnCLK_Frac_size = 2
    uint32_t SYSCLK;                     // Sysclk for the microcontroller
    uint32_t SYSnCLK_Int;                 // Integer value of SSInClk
    uint32_t SYSnCLK_Frac;                // Fraction value of SSInClk
    uint32_t SYSnCLK_Frac_size;           // Fraction depth (tenth, hundredth)
    uint32_t CPSDVSR;                     // Pg.976 Must be even number (2-255)
    uint32_t SCR;                         // Pg.969 SSI Serial clock rate have value from 0-255
}SSInClk_CFG_DATA_TYPE;


void PHP_SPI_SSInClk_Configure(SSInClk_CFG_DATA_TYPE *, uint32_t, uint32_t, uint32_t);

void PHP_SPI_SSI_Init(uint32_t, uint32_t, uint32_t);

uint8_t PHP_SPI_SSI_WriteData(SSI0_Type *, uint8_t data);

/*-----------------------------------   PHP_SPI_RegByteWrite  ---------------------------------------*
 * This function transmitted bytes of data consecutively to the slave device memory locations given
 * the first memory location on slave device.
 * Address of the first memory location on slave device is provided, followed by the data array for 
 * consecutive transfer starting from the given memory address.
 *
 * @input : (SSI0_Type *) -> pointer to SSI Type
 * @input : uint8_t       -> reg                  (starting register/mem address)
 * @input : uint8_t       -> data                 (data to be transmitted)
 * ---------------------------------------------------------------------------------------------------*/
uint8_t PHP_SPI_SSI_RegByteWrite(SSI0_Type * SSI_BLK_PTR, uint8_t reg,  uint8_t data);




/*-----------------------------------   PHP_SPI_RegBurstWrite  ---------------------------------------*
 * This function transmitted bytes of data consecutively to the slave device memory locations given
 * the first memory location on slave device.
 * Address of the first memory location on slave device is provided, followed by the data array for 
 * consecutive transfer starting from the given memory address.
 *
 * @input : (SSI0_Type *) -> pointer to SSI Type
 * @input : uint8_t       -> reg                  (starting register/mem address)
 * @input : uint8_t       -> byteCount            (data size to be transmitted)
 * @input : uint8_t *     -> byteData             (Data array / pointer to data array)

 * ---------------------------------------------------------------------------------------------------*/
uint8_t PHP_SPI_SSI_RegBurstWrite(SSI0_Type * SSI_BLK_PTR, uint8_t reg,
                                                   uint8_t byteCount,
                                                   uint8_t *byteData);



//uint8_t PHP_SPI_SSI_ReadData(SSI0_Type * SSI_BLK_PTR);
void PHP_SPI_SSI_RegByteRead(SSI0_Type * SSI_BLK_PTR, uint8_t reg, uint8_t *valPtr);
void PHP_SPI_SSI_RegBurstRead(SSI0_Type * SSI_BLK_PTR, uint8_t reg,
	                                                     uint8_t byteCount, 
                                                       uint8_t *byteData);

#endif // PHP_SPI_H

