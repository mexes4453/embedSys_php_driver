/*
 * php_switch.h
 *
 *  Created on: 20 Aug 2021
 *      Author: chime
 */

#ifndef PHP_SWITCH_H_
#define PHP_SWITCH_H_

#include "TM4C123GH6PM.h"
#include "php.h"

#define GPIO_PORTF_CR_R    (*((volatile unsigned long *)0x40025524))
#define SWITCH_PORT GPIOF

/*This function initialises the on-board switch on port f (Interrupt enabled - falling edge)
 * SW1 PORT.F4
 * SW2 PORT.F0 - is locked and requires unlocking (See Pg.688, 684)
 * */
void PHP_SWITCH_Init(void);


#endif /* PHP_SWITCH_H_ */
