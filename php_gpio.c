/*
 * php_gpio.c
 *
 *  Created on: 5 Dec 2021
 *      Author: chime
 */


#include "php_gpio.h"


/*
 * This function configures the gpio pin
 * Inputs: GPIOA_Type * GPIO_PORT -> GPIOB
 *         int portClkEnableBit   -> 1            // (PORTA:0; PORTB:1; PORTC:2, ..)
 *         uint32_t pinBit        -> (1U<<2)      // PB2
 *         int IO_FLAG,           -> 1            // (output:1, input:0)
 *         uint32_t PCTL_SEL,     -> 0x00440000   // port control for pin see Pg.1351, 688
 *         int AMSEL_FLAG         -> 0            // (digital:0, analog:1)
 */
void PHP_GPIO_Init(GPIOA_Type * GPIO_PORT, uint32_t portClkEnableBit,
                                           uint32_t pinBit,
                                           uint32_t IO_FLAG,
                                           uint32_t PCTL_SEL,
                                           uint32_t AMSEL_FLAG){

    uint32_t delay=0;
    SYSCTL->RCGC2  |= (1U<<portClkEnableBit);        // 01. Pg.340 - enable Run mode for GPIOB */
    delay = (SYSCTL->RCGC2)&(0xFFFFFFFF);
		//PHP_DelayMs(10);

    (IO_FLAG)? (GPIO_PORT->DIR |= pinBit) :           // 02. Configure pin as output or Input
               (GPIO_PORT->DIR &= ~pinBit);



    //-- 03. Configure pin for alternative function and port control
    if (PCTL_SEL) {
       GPIO_PORT->AFSEL |=  pinBit;
       GPIO_PORT->PCTL  |=  ((GPIO_PORT->PCTL)& ~PCTL_SEL) + PCTL_SEL; //(pg.1351, 688)
    }else{
        GPIO_PORT->AFSEL &= ~pinBit;
    }



    //-- 04. Configure Port pin as digital or analog
    switch (AMSEL_FLAG) {
        case 1:
            GPIO_PORT->AMSEL |=  pinBit;
            GPIO_PORT->DEN   &= ~pinBit;
            break;

        default:
            GPIO_PORT->AMSEL &= ~pinBit;
            GPIO_PORT->DEN   |=  pinBit;
				    break;

    }



}
