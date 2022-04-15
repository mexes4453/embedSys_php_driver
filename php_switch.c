/*
 * php_switch.c
 *
 *  Created on: 20 Aug 2021
 *      Author: chime
 */

#include "php_switch.h"


/*This function initialises the on-board switch on port f (Interrupt enabled - falling edge)
 * SW1 PORT.F4
 * SW2 PORT.F0 - is locked and requires unlocking (See Pg.688, 684)
 * */
void PHP_SWITCH_Init(void){

    SYSCTL->RCGC2  |= (1U<<5);                  // 01. Pg.340 - enable Run mode for GPIOF */

    // PORT.F0 is locked and requires unlocking (See Pg.688)
    SWITCH_PORT->LOCK = 0x4C4F434B;             // 02. Pg.684 - Writing value unlocks GPIO.CR commit register
    //SWITCH_PORT->CR = 1;           // 03. Pg.685 - allow PORT.F0 to be configurable
    GPIO_PORTF_CR_R |= 0x01;         // 03. Pg.685 - allow PORT.F0 to be configurable - the line above results in error (wrong CR address prhaps)

    SWITCH_PORT->DIR  &= (~(1U<<4)) | (~(1U<<0)); // 04. Configure pin 0 & 4 on port F as input
    SWITCH_PORT->DEN   |= (1U<<4) | (1U<<0);      // 05. Configure pin 0 & 4 on port F as digital
    SWITCH_PORT->AFSEL &= (~(1U<<4)) | (~(1U<<0));// 06. Disable alternate function
    SWITCH_PORT->AMSEL &= (~(1U<<4)) | (~(1U<<0));// 07. Disable analog mode

    // Configure interrupt trigger -> falling edge trigger
    SWITCH_PORT->IS  &= (~(1U<<4)) | (~(1U<<0));   // 08. Configure interrupt as edge sensitive
    SWITCH_PORT->IBE &= (~(1U<<4)) | (~(1U<<0));   // 09. Disable the use of both edge as the
                                                   //     one edge is configure by IEV register
    SWITCH_PORT->IEV &= (~(1U<<4)) | (~(1U<<0));   // 09. Configure both switch trigger - falling edge
    SWITCH_PORT->ICR |= (1U<<4) | (1U<<0);         // 10. Clear all previous interrupt
    SWITCH_PORT->IM  |= (1U<<4) | (1U<<0);         // 11. Unmask (enable) interrupt

    // Configure the interrupt priority (See Page.105 on datasheet)
    // PORT F -> Vector no:46, Interrupt no (IRQ):30
    /* Review files : C1203_05.PNG, C1203_06.PNG, C1203_07.PNG
     * m = (IRQ:30)/4 = 7 => Priority register number is 7 (NVIC_PRI7_R)
     * p = (IRQ:30)%4 = 2 => Priority bits are (8*p)+7, (8*p)+6, (8*p)+5 => 23, 22, 21; bits[23:21]
     * a = IRQ/32     = 0 => Interrupt enable register NVIC_EN0_R
     * b = IRQ%32     = 30 => Enable bit is 30 bit[30]
     * */
    //__NVIC_SetPriority(GPIOF_IRQn, 3);              // 12. Set priority to 3
    NVIC->IP[30]  |= (3U<<5);                     //     Alternative Pg.152

    //__NVIC_EnableIRQ(GPIOF_IRQn);                   // 13. Pg.142 set Enable (EN0) - bit 30 (INT.NO)
    NVIC->ISER[0] |= (1U<<30);                    //     Alternative Pg.142
}
