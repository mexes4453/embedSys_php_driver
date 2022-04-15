/*
 * php_led.c
 *
 *  Created on: 20 Aug 2021
 *      Author: chime
 */

#include "php_led.h"

void PHP_LED_Init(void){
	  uint32_t delay=0;
    SYSCTL->RCGC2  |= (1U<<5);                              // 01. Pg.340 - enable Run mode for GPIOF */
    delay = (SYSCTL->RCGC2)&(0xFFFFFFFF);
	  LED_PORT->DIR   |= 0x0000000E;                                    // 02. Configure all pin on port B as output
    LED_PORT->DEN   |= 0x0000000E;                                    // 03. Configure all pin on port B as digital
    LED_PORT->AFSEL &= ~0x0000000E;
    LED_PORT->AMSEL &= ~0x0000000E;
}


void PHP_LED_Toggle_Green(void){

    LED_PORT->DATA_Bits[LED_GREEN] |= LED_GREEN;          // Turn on LED_GREEN
    PHP_DelayMs(10);                                      // timer0A - delay 10ms
    LED_PORT->DATA_Bits[LED_GREEN] &= ~LED_GREEN;         // Turn off LED_GREEN
}


void PHP_LED_Toggle_Red(void){

    LED_PORT->DATA_Bits[LED_RED] |= LED_RED;            // Turn on LED_RED
    PHP_DelayMs(10);                                    // timer0A - delay 10ms
    LED_PORT->DATA_Bits[LED_RED] &= ~LED_RED;           // Turn off LED_RED
}


void PHP_LED_Toggle_Blue(void){

    LED_PORT->DATA_Bits[LED_BLUE] |= LED_BLUE;          // Turn on LED_BLUE
    PHP_DelayMs(10);                                    // timer0A - delay 10ms
    LED_PORT->DATA_Bits[LED_BLUE] &= ~LED_BLUE;         // Turn off LED_BLUE
}
