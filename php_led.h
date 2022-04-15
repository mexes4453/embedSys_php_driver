/*
 * php_led.h
 *
 *  Created on: 20 Aug 2021
 *      Author: chime
 */

#ifndef PHP_LED_H_
#define PHP_LED_H_

#define LED_RED   (1U << 1)
#define LED_BLUE  (1U << 2)
#define LED_GREEN (1U << 3)

#include "TM4C123GH6PM.h"
#include "php.h"

#define LED_PORT GPIOF



void PHP_LED_Init(void);
void PHP_LED_Toggle_Green(void);
void PHP_LED_Toggle_Blue(void);
void PHP_LED_Toggle_Red(void);

#endif /* PHP_LED_H_ */
