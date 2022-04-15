/*
 * dev_glcd.h
 *
 *  Created on: 5 Dec 2021
 *      Author: chime
 */

#ifndef DEV_GLCD_H_
#define DEV_GLCD_H_

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"
#include "php_gpio.h"
#include "php_spi.h"
#include "misc.h"


// SSI
#define GLCD_SSI_BLK SSI0
#define GLCD_SSI_CLK_SPD_INT        (3)
#define GLCD_SSI_CLK_SPD_FRAC       (33/100)
#define GLCD_SSI_CLK_SPD_FRAC_SIZE  (2)

// Other Pin configuration except for SSI
#define GLCD_PORT GPIOB
#define GLCD_PORT_CLK_EN_BIT  (1)
#define GLCD_DC  (1U<<2)         // PB2
#define GLCD_RST (1U<<3)         // PB3

#define GLCD_WIDTH               (84)
#define GLCD_HEIGHT              (48)
#define GLCD_CFG_EXT_INST        (0x21)  // Pg.14, 22 (datasheet) PD:0 active, V:1 horizontal entry mode, H:0 basic instruction
#define GLCD_CFG_VOP_CONTRAST    (0xB9)
#define GLCD_CFG_TEMP_COEFF      (0x04)
#define GLCD_CFG_BIAS_MODE       (0x14)
#define GLCD_CFG_NORM_MODE       (0x20)
#define GLCD_CFG_DISP_NORM_MODE  (0x0C)

enum DEV_GLCD_DC {
    GLCD_COMMAND,
    GLCD_DATA
};


// Function Declarations

void DEV_GLCD_Init(void);

void DEV_GLCD_DC_Write(enum DEV_GLCD_DC, uint8_t);

void DEV_GLCD_SetCursor(uint8_t x, uint8_t y);

// This function clears the entire screen by writing
// zero in all the buffer memory
void DEV_GLCD_ClearScreen(void);

// This function clears the entire screen by writing
// 1's in all the buffer memory
void DEV_GLCD_FillScreen(void);

#endif /* DEV_GLCD_H_ */
