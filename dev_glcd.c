/*
 * dev_glcd.c
 *
 *  Created on: 5 Dec 2021
 *      Author: chime
 */


#include "dev_glcd.h"


void DEV_GLCD_Init(void){


    // initialise SPI Peripheral with clk speed 4.00Mbps
    PHP_SPI_SSI_Init(GLCD_SSI_CLK_SPD_INT,
                     GLCD_SSI_CLK_SPD_FRAC,
                     GLCD_SSI_CLK_SPD_FRAC_SIZE);

    // Configure other LCD pins
    PHP_GPIO_Init(GLCD_PORT, GLCD_PORT_CLK_EN_BIT, GLCD_DC, 1U, 0U, 0U);
	  PHP_GPIO_Init(GLCD_PORT, GLCD_PORT_CLK_EN_BIT, GLCD_RST, 1U, 0U, 0U);


    // Reset GLCD Controller
	  GLCD_PORT->DATA_Bits[GLCD_RST] = GLCD_RST;   // set reset pin (high)
	  PHP_DelayMs(100);
    GLCD_PORT->DATA_Bits[GLCD_RST] = ~GLCD_RST;  // set reset pin low (active low)
    PHP_DelayMs(100);
    GLCD_PORT->DATA_Bits[GLCD_RST] = GLCD_RST;   // set reset pin (high)

    // Configure GLCD controller
    DEV_GLCD_DC_Write(GLCD_COMMAND, GLCD_CFG_EXT_INST);       // Set extended command mode bit[0]. Pg.14, 22 datasht
    DEV_GLCD_DC_Write(GLCD_COMMAND, GLCD_CFG_VOP_CONTRAST);
    DEV_GLCD_DC_Write(GLCD_COMMAND, GLCD_CFG_TEMP_COEFF);
    DEV_GLCD_DC_Write(GLCD_COMMAND, GLCD_CFG_BIAS_MODE);
    DEV_GLCD_DC_Write(GLCD_COMMAND, GLCD_CFG_NORM_MODE);
    DEV_GLCD_DC_Write(GLCD_COMMAND, GLCD_CFG_DISP_NORM_MODE);
}



void DEV_GLCD_DC_Write(enum DEV_GLCD_DC DC_FLAG, uint8_t data){
	
	  // Activate chip for data transmission (slave select is active low)
	  SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = ~PIN_SSI0_SS;

    // Select register to write to (Data or command register)

    switch (DC_FLAG){

        case GLCD_COMMAND:
            GLCD_PORT->DATA_Bits[GLCD_DC] = ~GLCD_DC;
            PHP_SPI_SSI_WriteData(GLCD_SSI_BLK, data);
            break;

        case GLCD_DATA: // Selects GLCD_DATA by default
            GLCD_PORT->DATA_Bits[GLCD_DC] = GLCD_DC;
            PHP_SPI_SSI_WriteData(GLCD_SSI_BLK, data);
						break;
					
    }

		// deactivate chip for data transmission (slave select is active low)
    SPI_GPIO_PORT->DATA_Bits[PIN_SSI0_SS] = PIN_SSI0_SS;
}




void DEV_GLCD_SetCursor(uint8_t x, uint8_t y){

    DEV_GLCD_DC_Write(GLCD_COMMAND, 0x80 | x); // Pg.14 datasht: set column (x) address
    DEV_GLCD_DC_Write(GLCD_COMMAND, 0x40 | y); // Pg.14 datasht: set row bank (y) address
}

// This function clears the entire screen by writing
// zero in all the buffer memory
void DEV_GLCD_ClearScreen(void){

    int32_t idx;
    for (idx=0; idx< ((GLCD_WIDTH * GLCD_HEIGHT) / 8); idx++){
        DEV_GLCD_DC_Write(GLCD_DATA, 0x00);

    }

    // Return cursor to home position after screen clear
    DEV_GLCD_SetCursor(0, 0);
}


// This function clears the entire screen by writing
// 1's in all the buffer memory
void DEV_GLCD_FillScreen(void){

    int32_t idx;
    for (idx=0; idx< ((GLCD_WIDTH * GLCD_HEIGHT) / 8); idx++){
        DEV_GLCD_DC_Write(GLCD_DATA, 0xFF);

    }

    // Return cursor to home position after screen clear
    DEV_GLCD_SetCursor(0, 0);
}







