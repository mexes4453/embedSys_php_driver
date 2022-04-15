/*
 * php_gpio.h
 *
 *  Created on: 5 Dec 2021
 *      Author: chime
 */

#ifndef PHP_GPIO_H_
#define PHP_GPIO_H_

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"
#include "misc.h"

/*
 * This function configures the gpio pin
 * Inputs: GPIOA_Type * GPIO_PORT -> GPIOB
 *         int portClkEnableBit   -> 1            // (PORTA:0; PORTB:1; PORTC:2, ..)
 *         uint32_t pinBit        -> (1U<<2)      // PB2
 *         int IO_FLAG,           -> 1            // (output:1, input:0)
 *         uint32_t PCTL_SEL,     -> 0x00440000   // port control for pin see Pg.1351, 688
 *         int AMSEL_FLAG         -> 0            // (digital:0, analog:1)
 *
 *
 *  Note that the following pins need to be unlocked (Pg.688) - PF0 & PD7
 *  They can be reprogrammed by unlocking the pin in the GPIOLOCK register and
 *  uncommitting it by setting the GPIOCR register
 */
void PHP_GPIO_Init(GPIOA_Type * , uint32_t portClkEnableBit,
                                  uint32_t pinBit,
                                  uint32_t IO_FLAG,
                                  uint32_t PCTL_SEL,
                                  uint32_t AMSEL_FLAG);


#endif /* PHP_GPIO_H_ */
