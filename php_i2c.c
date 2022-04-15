/*
 * Author: Udoh Chiemezie Albert
 * Date  : 12.09.2021
 *
 * This code runs on TM4C Tiva Launchpad from Texas instrument
 * it configures the launchpad to use its i2c to communicate with
 * external devices.
 *
 * Example. Accelerometer.
 *
 * */
#include "php_i2c.h"

extern uint32_t sysClkFreq;                     // Variable created in php.c

// PD0 - CLOCK LINE
// PD1 - DATA LINE
// Pg.236  -> SYSTEM CONTROL REGISTER MAP
// Pg.1015 -> Initialisation and configuration
void PHP_I2C_Init(I2C0_Type * I2C){
    uint32_t sysClkFreq;                       // system clock freq

	SYSCTL->RCGC2    |= I2C_GPIO_PORT_CLK_EN;  // 01. Pg.340 - enable CLK Run mode for GPIOD */
	PHP_DelayUs(2);                            //     Allow clock to stabilize;
	SYSCTL->RCGCI2C  |= RCGC1_I2C_CLK_EN;      // 02. Pg.460 - enable CLK Run mode for SSI_SPI */
	PHP_DelayUs(2);                            //     Allow clock to stabilize;

	// Configure GPIO PIN for SSI_SPI
	I2C_GPIO_PORT->AFSEL |=  (I2C_ALL_PINS);  // 03. Enable alternate function for CLK & SDA
	I2C_GPIO_PORT->AMSEL &= ~(I2C_ALL_PINS);               // 04. Disable analog mode for all pins
	I2C_GPIO_PORT->DEN   |=  I2C_ALL_PINS;                 // 05. Configure all SSI pin on port d as digital
	//I2C_GPIO_PORT->PCTL  &=  ~PCTL_I2C_CLK_SDA_BITS;       // 06. Pg.688 Clear PCTL bits for CLK; RX; TX
	I2C_GPIO_PORT->PCTL  |=  PCTL_I2C_CLK_SDA;             // 07. Pg.688 Set PCTL bits for CLK; RX; TX (pg.1351)
	I2C_GPIO_PORT->ODR   |=  PIN_I2C_SDA;                  // 08. Pg.676 Enable Open Drain
	//I2C_GPIO_PORT->PUR   |=  PIN_I2C_SDA;
	//I2C_GPIO_PORT->DATA_Bits[PIN_I2C_SDA] &= ~PIN_I2C_SDA;

	I2C->MCR  = I2C_MCR_MFE_BIT;                         // 09. Pg.1031 configure Launchpad as master

	/* Pg.1026 - Master Timer Period computation (Data Transmisson speed period)
	 * TPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1;
       TPR = (80MHz/(2*(6+4)*100000))-1;
       TPR = 9
	 */
	sysClkFreq = PHP_SysClock_GetFreq();                  // Get current System clock frequency
	I2C->MTPR = (sysClkFreq/(20 *I2C_BUS_FREQ))-1;       // Pg.1026 - Compute timer period for data transmission
	I2C->MTPR &= ~(I2C_MTPR_HS);                         // Pg.1026 - Configure also for standard mode, bit[7] = 0

}


/*------------------------------------------ PHP_I2C_Wait  -------------------------------------*
 * This function polls busy bit of the master control status during data transmission
 * Therefore, waiting till the ongoing transmission is complete.
 * The error bits are returned bits[3:1].
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
int PHP_I2C_Wait(I2C0_Type * I2C){
    while (I2C->MCS & I2C_MCS_BUSY);               // Pg.1021 wait until I2C master is not busy (0)
    return I2C->MCS & I2C_MCS_ERR_BITS;            // error bit return (DATA_NACK; ADR_NACK; ERROR)

}


/*-----------------------------------   PHP_I2C_ByteWrite  ---------------------------------------*
 * This function transmitted one byte of data given the slave address and data to be transmitted
 * one byte of data is transmitted to the slave at any given time
 * it also check for error by polling bits as defined in PHP_I2C_Wait function.
 * @input : (I2C0_Type *) -> pointer to I2C Type
 * @input : int           -> slave address
 * @input : char          -> data (byte to be transmitted)
 *
 * @return: char          -> return ERROR
 * ----------------------------------------------------------------------------------------------*/
char PHP_I2C_ByteWrite(I2C0_Type * I2C, int slaveAddr, char memAddr, char data){
    char ERROR;

    I2C->MSA  =  (slaveAddr<<1);       // Pg. 1019 update master slave addr register bits[7:1]
    I2C->MSA &= ~(I2C_MSA_RS_BIT);       // Pg. 1019 specify write operation bit[0] = 0
    I2C->MDR = memAddr & 0xFF;          // Pg. 1025 place byte data to be transmitted in data register
    //I2C_MDR_R = I2C_MDR_R&0xFF;          // Read back
    I2C->MCS  = 0x03;//|(1U<<3);                    // pg. 1061 Initiate a single byte transmit of the data from
                                         //          Master to Slave (0; START; RUN) ->
                                         //          START-(SLAVE_ADDR + W)-ACK-MEM_ADDR-ACK
    ERROR = I2C_MDR_R&0xFF;          // Read back
    ERROR = PHP_I2C_Wait(I2C);           // Wait for I2C to complete transmission and check for error
    if (ERROR) return ERROR;             // Return Error if any

    // Transmit Register addr
//    I2C->MDR = memAddr & 0xFF;          // Pg. 1025 place byte data to be transmitted in data register
//    I2C->MCS  = 0x03; //|(1U<<3);                    // pg. 1061 Initiate a single byte transmit of the data from
//    ERROR = PHP_I2C_Wait(I2C);           // Wait for I2C to complete transmission and check for error
//    if (ERROR) return ERROR;             // Return Error if any


    // Transmit Data
    I2C->MDR  = data;                    // Pg. 1025 place byte data to be transmitted in data register
    I2C->MCS  = 0x05;                    // pg. 1061 Initiate a single byte transmit of the data from
                                         //          Master to Slave (STOP; 0; RUN) -> -Data-NACK-STP
    ERROR = PHP_I2C_Wait(I2C);           // Wait for I2C to complete transmission and check for error
    if (ERROR) return ERROR;             // Return Error if any

    while(I2C->MCS & I2C_MCS_BUSBSY_BIT);
    return ERROR;
}



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
char PHP_I2C_BurstWrite(I2C0_Type * I2C, int slaveAddr, char memAddr, int byteCount, char *data){
    char ERROR;

    if (byteCount <= 0) return ERROR; // No Data transmission was performed

    /* Send slave addr and first memory address */
    I2C->MSA  =  (slaveAddr << 1);       // Pg. 1019 update master slave addr register bits[7:1]
    I2C->MSA &= ~(I2C_MSA_RS_BIT);       // Pg. 1019 specify write operation bit[0] = 0
    I2C->MDR  = memAddr;                 // Pg. 1025 place byte data to be transmitted in data register
    I2C->MCS  = 0x03;                    // pg. 1061 Initiate a single byte transmit of the data from

    ERROR = PHP_I2C_Wait(I2C);           // Wait for I2C to complete transmission and check for error
    if (ERROR) return ERROR;             // Return Error if any

    /* Transmit data excluding last data */
    while(byteCount > 1){
        I2C->MDR  = *(data++);           // Pg. 1025 place byte data to be transmitted in data register
        I2C->MCS  = 0x01;                // pg. 1061 Initiate a byte transmit  -data-ACK-
        ERROR = PHP_I2C_Wait(I2C);       // Wait for I2C to complete transmission and check for error
        if (ERROR) return ERROR;         // Return Error if any
        byteCount--;
    }

    /* Transmit last data and include STOP in frame */
    I2C->MDR  = *(data++);                 // Pg. 1025 place byte data to be transmitted in data register
    I2C->MCS  = 0x05;                      // pg. 1061 Initiate a byte transmit  -data-NACK-STP
    ERROR = PHP_I2C_Wait(I2C);             // Wait for I2C to complete transmission and check for error
    if (ERROR) return ERROR;               // Return Error if any

    while(I2C->MCS & I2C_MCS_BUSBSY_BIT); // Wait till I2C bus is not idle (bit[6] = 0)
    return 0;


}



/*-----------------------------------   PHP_I2C_ByteRead  ---------------------------------------*
 * This function receive one byte of data given the slave address and data pointer to store data
 * one byte of data is received from the slave at any given time
 * it also check for error by polling bits as defined in PHP_I2C_Wait function.
 * @input : (I2C0_Type *) -> pointer to I2C Type
 * @input : int           -> slave address
 * @input : char*         -> Pointer to store received data
 *
 * @return: char          -> return ERROR
 * ----------------------------------------------------------------------------------------------*/
char PHP_I2C_ByteRead(I2C0_Type * I2C, int slaveAddr, char memAddr, char *data){
    char ERROR;


    /* Send slave addr and first memory address */
    I2C->MSA  =  (slaveAddr << 1);       // Pg. 1019 update master slave addr register bits[7:1]
    //I2C->MSA &=  ~I2C_MSA_RS_BIT;         // Pg. 1019 specify write operation bit[0] = 1
    I2C->MDR  = memAddr;                 // Pg. 1025 place byte data to be transmitted in data register
    I2C->MCS  = 0x03;                    // pg. 1016 Initiate a single byte transmit of the data from
                                         //     START-(SLAVE_ADDR + W)-ACK-MEM_ADDR-ACK
    ERROR = PHP_I2C_Wait(I2C);           // Wait for I2C to complete transmission and check for error
    if (ERROR) return ERROR;             // Return Error if any


    I2C->MSA  =  (slaveAddr << 1);       // Pg. 1019 update master slave addr register bits[7:1]
    I2C->MSA |= I2C_MSA_RS_BIT;          // Pg. 1019 specify read operation bit[0] = 1
    I2C->MCS  = 0x07;                    // pg. 1016 Initiate a single byte transmit of the data from
                                         // pg. 1022 Master to Slave (STOP; START; RUN)

    ERROR = PHP_I2C_Wait(I2C);           // Wait for I2C to complete transmission and check for error
    if (ERROR) return ERROR;             // Return Error if any
    *data = I2C->MDR;                    // Pg. 1025 store byte data to given pointer

    return ERROR;
}



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
char PHP_I2C_BurstRead(I2C0_Type * I2C, int slaveAddr, char memAddr, int byteCount, char *data){
    char ERROR;

    if (byteCount <= 0) return ERROR; // No Data transmission was performed

    /* Send slave addr and first memory address */
    I2C->MSA  =  (slaveAddr << 1);       // Pg. 1019 update master slave addr register bits[7:1]
    //I2C->MSA &=  ~I2C_MSA_RS_BIT;         // Pg. 1019 specify write operation bit[0] = 1
    I2C->MDR  = memAddr;                 // Pg. 1025 place byte data to be transmitted in data register
    I2C->MCS  = 0x03;                    // pg. 1016 Initiate a single byte transmit of the data from
                                         //     START-(SLAVE_ADDR + W)-ACK-MEM_ADDR-ACK
    ERROR = PHP_I2C_Wait(I2C);           // Wait for I2C to complete transmission and check for error
    if (ERROR) return ERROR;             // Return Error if any

    /* Change the I2C bus direction from write to read */
    I2C->MSA  =  (slaveAddr << 1);       // Pg. 1019 update master slave addr register bits[7:1]
    I2C->MSA |= I2C_MSA_RS_BIT;          // Pg. 1019 specify read operation bit[0] = 1
                                         // (RPT_START)-(SLAVE_ADDR + R)-ACK


    /* Transmit read data */
    if (byteCount == 1){                 // No ACK if last byte is being read
        I2C->MCS = 0x07;                 // -DATA-NACK-STP

    }else{
        I2C->MCS = 0x0B;                 // -ACK-DATA-ACK
    }
    ERROR = PHP_I2C_Wait(I2C);           // Wait for I2C to complete transmission and check for error
    if (ERROR) return ERROR;             // Return Error if any

    *(data++) = I2C->MDR;                // Store received data


    /* Transmit read remaining byte */
    while(byteCount > 1){
        I2C->MCS  = 0x09;                // pg. 1061 Initiate a byte transmit  -data-ACK-
        ERROR = PHP_I2C_Wait(I2C);       // Wait for I2C to complete transmission and check for error
        if (ERROR) return ERROR;         // Return Error if any
        byteCount--;
        *(data++) = I2C->MDR;           // Pg. 1025 place byte data to be transmitted in data register
    }

    /* Transmit last data and include STOP in frame */
    I2C->MCS  = 0x05;                      // pg. 1022 Initiate a byte transmit  -data-ACK-STP
    ERROR = PHP_I2C_Wait(I2C);             // Wait for I2C to complete transmission and check for error
    *(data++) = I2C->MDR;                  // Pg. 1025 place byte data to be transmitted in data register
    if (ERROR) return ERROR;               // Return Error if any

    while(I2C->MCS & I2C_MCS_BUSBSY_BIT); // Wait till I2C bus is not idle (bit[6] = 0)
    return 0;


}
