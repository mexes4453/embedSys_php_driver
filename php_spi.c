/*
 * Author: Udoh Chiemezie Albert
 * Date  : 12.09.2021
 *
 * This code runs on TM4C Tiva Launchpad from Texas instrument
 * it configures the launchpad to use its synchronous serial interface (SSI)
 * to communicate with external devices that communicate using Serial Protocol
 * Interface (SPI). Example. Nokia_5110 display screen.
 *
 * */
#include "php_spi.h"


void PHP_SPI_SSInClk_Configure(SSInClk_CFG_DATA_TYPE * cfgData, uint32_t clkInt,
                               uint32_t clkFrac, uint32_t clkFracSize){
    // SSInClk = SysClk / (CPSDVSR * (1 + SCR)) Pg.954
    uint32_t quotient;


    // (CPSDVSR * (1 + SCR)) =    SysClk /  SSInClk
    quotient = MISC_DivByFloat(cfgData->SYSCLK, clkInt, clkFrac, clkFracSize);


    // Optimize CPSDVSR & SCR Values
    if (quotient <= 255){
        cfgData->CPSDVSR = quotient;
    }else{
        for (int x=2; x<=255; (x=+2)){    // CPSDVSR MUST BE EVEN
            for (int y=0; y<=255; y++){   // SCR
                if ((cfgData->CPSDVSR * (1 + cfgData->SCR)) > quotient){
                    (cfgData->SCR)--;
                    break;
                }
                (cfgData->SCR)++;
            }
            (cfgData->CPSDVSR) += 2;
        }
    }
}




// Pg.236 -> SYSTEM CONTROL REGISTER MAP
// Pg.965 -> Initialisation and configuration
void PHP_SPI_SSI_Init(uint32_t clkInt, uint32_t clkFrac, uint32_t clkFracSize){
	uint32_t delay = 0;
	SYSCTL->RCGC2  |= GPIO_PORT_A_CLK_EN_BIT;	     // 01. Pg.340 - enable CLK Run mode for GPIOD */
	//PHP_DelayMs(2);                                  //     Allow clock to stabilize;
	delay = (SYSCTL->RCGC2)&(0xFFFFFFFF);
	SYSCTL->RCGC1  |= RCGC1_SSI0_CLK_EN_BIT;         // 02. Pg.460 - enable CLK Run mode for SSI_SPI */
	//PHP_DelayMs(2);                                  //     Allow clock to stabilize;
  delay = (SYSCTL->RCGC1)&(0xFFFFFFFF);
	
	// Configure GPIO PIN for SSI_SPI
	SPI_GPIO_PORT->AFSEL |=  (PIN_SSI0_CLK | PIN_SSI0_TX | PIN_SSI0_RX );                    // 03. Enable alternate function for CLK & TX
	SPI_GPIO_PORT->AMSEL &= ~(SSI0_ALL_PINS);                  // 04. Disable analog mode for all pins
	SPI_GPIO_PORT->DIR   |=  PIN_SSI0_SS;                          // 05. Configure SSI1_SS as output
	//SPI_GPIO_PORT->PUR   |=  PIN_SSI0_SS;                          // 05. Configure SSI1_SS as output
    SPI_GPIO_PORT->DEN   |=  SSI0_ALL_PINS;                    // 06. Configure all SSI pin on port d as digital
    SPI_GPIO_PORT->PCTL  &=  ~PCTL_SSI0_CLK_RX_TX_BIT;         // 07. Pg.688 Clear PCTL bits for CLK; RX; TX
    SPI_GPIO_PORT->PCTL  |=  PCTL_SSI0_CLK_RX_TX;              // 08. Pg.688 Set PCTL bits for CLK; RX; TX (pg.1351)
    SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = PIN_SSI0_SS;               // 09. Set SLAVE SELECT High - idle state

    // Configure SSI_SPI Frame format settings
    // CPOL:=0 CPHA:=0 Master
    SSI_BLK->CR1 &= ~SSI_CR_SSE_BIT;                              // 10. Pg.972 Disable SSI_SPI_before programming
    SSI_BLK->CR1 &= ~SSI_CR_MS_BIT;                               // 11. Pg.971 Configure as Master
    SSI_BLK->CC   = (SSI_BLK->CC & SSI_CC_CS_BITS) + SSI_CC_SYS_CLK_SEL;  // 12. Pg.984 Clear bit field and select clock


    // SSInClk = SysClk / (CPSDVSR * (1 + SCR)) Pg.954
    // Initialise SSInCLK configuration parameter (CPSDVSR, SCR) to minimum default values
    SSInClk_CFG_DATA_TYPE SSInClkCfgData;

    SSInClkCfgData.SYSCLK = PHP_SysClock_GetFreq()/1000000;  // Sysclk for the launchpad in MHz
    SSInClkCfgData.SYSnCLK_Int  = clkInt;                    // Integer value of SSInClk
    SSInClkCfgData.SYSnCLK_Frac = clkFrac;                   // Fraction value of SSInClk
    SSInClkCfgData.SYSnCLK_Frac_size = clkFracSize;          // Fraction depth (tenth, hundredth)
    //SSInClkCfgData.CPSDVSR = 24;                              // Pg.976 clock prescale divisor must be even number (2-255)
    //SSInClkCfgData.SCR = 0x00;                                  // Pg.969 SSI Serial clock rate have value from 0-255
    PHP_SPI_SSInClk_Configure(&SSInClkCfgData, clkInt, clkFrac, clkFracSize);

//    SSI1->CPSR = (SSI1->CPSR & SSI_CPSR_CPSDVSR_BITS) +                    // Pg.976 Clear prescale divisor
//                  SSInClkCfgData.CPSDVSR;                                  // Set prescale divisor
    SSI_BLK->CPSR = SSInClkCfgData.CPSDVSR;                                  // Set prescale divisor

    SSI_BLK->CR0  = (SSI_BLK->CR0 & ~SSI_CR0_SCR_BITS) +                      // Pg.969 Clear SCR[15:8]
                    (SSInClkCfgData.SCR << 8);                                // Set SCR bits[15:8]
										
    SSI_BLK->CR0  = (SSI_BLK->CR0 & ~SSI_CR0_SCR_BITS) +                      // Pg.969 Clear SCR[15:8]
                    (SSInClkCfgData.SCR << 8);                                // Set SCR bits[15:8]
										
    SSI_BLK->CR0 &= ~(SSI_CR0_SPO_BIT |                                       // SPO = 0; Clear bit
                      SSI_CR0_SPH_BIT |                                       // SPH = 0; Clear bit
                      SSI_CR0_FRF_BITS);                                      // Frame format select - FRF (0x00)

    SSI_BLK->CR0  = (SSI_BLK->CR0 & ~SSI_CR0_DSS_BITS) + SSI_CR0_DSS_8BIT_DATA;  // Configure Data size as 8 bit
    SSI_BLK->CR1 |= SSI_CR_SSE_BIT;                                           // enable SSI
}




uint8_t PHP_SPI_SSI_WriteData(SSI0_Type * SSI_BLK_PTR, uint8_t data){
    uint8_t dataRx;
	
	  SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = ~PIN_SSI0_SS;              // Activate chip for data transmission
    while ((SSI_BLK_PTR->SR & SSIX_SR_BSY_BIT) == SSIX_SR_BSY_BIT) {}  // Wait till SSI is idle before transmission start
    SSI_BLK_PTR->DR = data;                                            // store data in SSI data register for transmission
		
    while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == 0){}                 // wait till response from slave device
    

		SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = PIN_SSI0_SS;               // Deactivate chip after transmission
    dataRx = (uint8_t)((SSI_BLK_PTR->DR)&0xFF);                                          // read received data from slave 
	  return dataRx;
}




uint8_t PHP_SPI_SSI_RegByteWrite(SSI0_Type * SSI_BLK_PTR, uint8_t reg,  
	                                                        uint8_t data)
{   
	
	  SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = ~PIN_SSI0_SS;               // Activate chip for data transmission
    while ((SSI_BLK_PTR->SR & SSIX_SR_BSY_BIT) == SSIX_SR_BSY_BIT) {}   // Wait till SSI is idle before transmission start
    SSI_BLK_PTR->DR = reg;                                              // store Reg addr in SSI data register for transmission
 	
		while ((SSI_BLK_PTR->SR & SSIX_SR_TFE_BIT) != SSIX_SR_TFE_BIT) {};  // Wait till Transmit FIFO is empty
		while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == 0){};	                // wait till response from slave device


    // Transmit data 
		SSI_BLK_PTR->DR = data; 
    while ((SSI_BLK_PTR->SR & SSIX_SR_TFE_BIT) != SSIX_SR_TFE_BIT) {};			
    while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == 0){}                 // wait till response from slave device
    
			
		PHP_DelayMs(1);	
		SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = PIN_SSI0_SS;               // Deactivate chip after transmission
    return (uint8_t)(SSI_BLK_PTR->DR);                                 // read received data from slave 
	  //return dataRx;
	
	  //  Since this function performs write operation, dataRx is returned only in read operation
}



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
                                                   uint8_t *byteData)
{
	  uint8_t idx, dataRx;
	
	for (idx=0; idx<byteCount; idx++){
	  dataRx = PHP_SPI_SSI_RegByteWrite(SSI_BLK_PTR, reg, *byteData);
		reg = (((reg>>1) + 1)<<1);
		byteData++;
		
  }
	
	/*
	  SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = ~PIN_SSI0_SS;                   // Activate chip for data transmission
	
	  while ((SSI_BLK_PTR->SR & SSIX_SR_BSY_BIT) == SSIX_SR_BSY_BIT) {}       // Wait till SSI is idle before transmission start
    SSI_BLK_PTR->DR = reg;                                                  // store Reg addr in SSI data register for transmission
																																				
	
    for (idx=0; idx<byteCount; idx++){
			  while ((SSI_BLK_PTR->SR & SSIX_SR_TFE_BIT) != SSIX_SR_TFE_BIT) {};  // Wait till Transmit FIFO is empty
		    while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == 0){};	                // wait till response from slave device
				dataRx = (uint8_t)(SSI_BLK_PTR->DR);                                // read received data from to prevent Master FIFO Full state
	      SSI_BLK_PTR->DR = byteData[idx];                                    // Transmit data 					
        PHP_DelayMs(10);	                                                      // Timing requirement for SS Select to go high
		    SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = PIN_SSI0_SS;	
        PHP_DelayMs(5);	
        SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = ~PIN_SSI0_SS;					
		}
		
		PHP_DelayMs(10);	                                                      // Timing requirement for SS Select to go high
		SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = PIN_SSI0_SS;                    // Deactivate chip after transmission
    return (uint8_t)(SSI_BLK_PTR->DR);                                      // read received data from slave 
		*/
}





void PHP_SPI_SSI_RegByteRead(SSI0_Type * SSI_BLK_PTR, uint8_t reg, uint8_t * valPtr)
{


	// clear the receive buffer 	
	  while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == SSIX_SR_RNE_BIT){
		    *(valPtr) = SSI_BLK_PTR->DR; 
		}
	
	  SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = ~PIN_SSI0_SS;              // Activate chip for data transmission
    while ((SSI_BLK_PTR->SR & SSIX_SR_BSY_BIT) == SSIX_SR_BSY_BIT) {}  // Wait till SSI is idle before transmission start
    SSI_BLK_PTR->DR = reg;                                             // store data in SSI data register for transmission
		
		while ((SSI_BLK_PTR->SR & SSIX_SR_TFE_BIT) != SSIX_SR_TFE_BIT) {}; // Wait till Transmit FIFO is empty
		while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == 0){};                // Wait till slave data is received
		*(valPtr) = (uint8_t)(SSI_BLK_PTR->DR);
		
		
    // starting transmitting dummy data to read data from given register address
		SSI_BLK_PTR->DR = 0x00; 	
		while ((SSI_BLK_PTR->SR & SSIX_SR_TFE_BIT) != SSIX_SR_TFE_BIT) {}; // Wait till Transmit FIFO is empty
    while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == 0){}                 // wait till response from slave device
    
		PHP_DelayMs(1);
		*(valPtr) = (uint8_t)(SSI_BLK_PTR->DR);                            // read received data from slave

		SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = PIN_SSI0_SS;               // Deactivate chip after transmission
	
}



/*-----------------------------------   PHP_SPI_RegBurstRead  ---------------------------------------*
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
void PHP_SPI_SSI_RegBurstRead(SSI0_Type * SSI_BLK_PTR, uint8_t reg,
	                                                     uint8_t byteCount, 
                                                       uint8_t *byteData)
{
	  uint8_t dataRx;
	  uint16_t idx;
    // clear (flush) the receive buffer by reading data register repeatedly until Recv FIFO is empty 	
	  while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == SSIX_SR_RNE_BIT){
		    dataRx = SSI_BLK_PTR->DR; 
		}
		
		// Transmit start register address to commence reading from
		SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = ~PIN_SSI0_SS;                  // Activate chip for data transmission
    while ((SSI_BLK_PTR->SR & SSIX_SR_BSY_BIT) == SSIX_SR_BSY_BIT) {}      // Wait till SSI is idle before transmission start
    SSI_BLK_PTR->DR = reg;                                                 // store data in SSI data register for transmission
		while ((SSI_BLK_PTR->SR & SSIX_SR_TFE_BIT) != SSIX_SR_TFE_BIT) {};     // Wait till Transmit FIFO is empty
		while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == 0){};                    // Wait till slave data is received
		dataRx = (uint8_t)(SSI_BLK_PTR->DR);                                   // First received byte is always useless
			
		// Transmit remaning register address for reading
		for (idx=0; idx<(byteCount-1); idx++){
			  reg = (((reg>>1) + 1)<<1);                                         // shift reg addr right before adding and shift back left
			  SSI_BLK_PTR->DR = reg;
		    while ((SSI_BLK_PTR->SR & SSIX_SR_TFE_BIT) != SSIX_SR_TFE_BIT) {}; // Wait till Transmit FIFO is empty
		    while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == 0){};                // Wait till slave data is received
        (*byteData) = (uint8_t)(SSI_BLK_PTR->DR);
				byteData++;
		}
			  
					
	  // Starting transmitting dummy data to read data from last given register address
		SSI_BLK_PTR->DR = 0x00; 	
		while ((SSI_BLK_PTR->SR & SSIX_SR_TFE_BIT) != SSIX_SR_TFE_BIT) {};    // Wait till Transmit FIFO is empty
    while ((SSI_BLK_PTR->SR & SSIX_SR_RNE_BIT) == 0){}                    // wait till response from slave device
		(*byteData) = (uint8_t)(SSI_BLK_PTR->DR);
			
		PHP_DelayMs(1);                                                       // Timing requirement to set SS Select high
		SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = PIN_SSI0_SS;                  // Deactivate chip after transmission
}


