/*
 * php_pwm.c
 *
 *  Created on: 28 Nov 2021
 *      Author: chime
 */

#include "php_pwm.h"

//----- PWM Init -------------
//----- See User Manual page 1239 for configuration steps
//----- https://microcontrollerslab.com/pwm-tm4c123-example-codes-tiva-c-launchpad/
//----- PWM CLOCK FREQ : 250kHz
//----- PWM Freq : 50Hz (desired)
//----- PWM Period : PWM CLOCK FREQ/PWM Freq = 250kHz/50Hz = 5000
//----- PWM Period -> load value = 5000-1 = 4999 (Counter is zero based)
//----- PWM default duty cycles = 50% (setting comparators to PWM Period/2 = 2499)
void PHP_PWM_Init(int periodFreq){

    uint32_t sysClkFreq = PHP_SysClock_GetFreq();                               // Get system clock current freq
    int counterValue;
    // Initialise GPIO PORT PINS

    SYSCTL->RCGC2 |= GPIO_PORT_CLK_EN_BIT;                                    // 01. Pg.340 - enable Run mode for GPIO Port *;                              // 01. Pg.340 - enable Run mode for GPIOF *
    PHP_DelayMs(10);                                                            // Activate clock for Port B -> 0x02 (bit 2)
    PWM_GPIO_PORT->AFSEL |= GPIO_PORT_PIN_ALL;                                  // 03 Enable alt function for bit 6 and 7
    PWM_GPIO_PORT->DIR   |= GPIO_PORT_PIN_ALL;                                  // 04 Configure pins for output PB6 & PB7
    PWM_GPIO_PORT->DEN   |= GPIO_PORT_PIN_ALL;                                  // 05 Configure pins as digital pins PB6 & PB7

    PWM_GPIO_PORT->PCTL = ((PWM_GPIO_PORT->PCTL)&GPIO_PCTL_RST) | GPIO_PCTL_SET;// 06 Select peripheral function PB6 & PB7 (pg.1351, 688)
                                                                                // Pin PB6 -> bit field [27:23] PMCx bit value ->(4)
                                                                                // Pin PB7 -> bit field [31:28] PMCx bit value ->(4)


    // Initialise PWM MODULE
    SYSCTL->RCGC0 |= SYSCTL_RCGC0_PWMX;                    // 01 Pg. 236,457,354 -- Activate run-mode clock gating Ctrl reg
                                                           //                    -> for PWM0 -> 0x00100000 (bit 20).
    PHP_DelayMs(10);                                       //                    -> Let clock stabilize
    SYSCTL->RCC |= SYSCTL_RCC_USEPWMDIV;                   // 07 pg.254 -> Select clock source for PWM
                                                           //           -> Enable PWM Clock divisor -> 0x00100000 (bit 20)

    SYSCTL->RCC = ((SYSCTL->RCC) & SYSCTL_RCC_PWMDIV_RST) |
                                   SYSCTL_RCC_PWMDIV_SET;  // 08 (Pg.254, 255) -> Select PWM Unit Clock divisor
                                                           //                  -> Clock source freq = 16MHz; 0x7 <-> E (selects 64 unit divisor)
                                                                                                                // PWM clock freq    = 16MHz/64 = 250kHz

    PWM_BLOCK->_0_CTL   &= ~PWMX_CTL_EN;                    // 09 Pg.1266 -> Disable PWM during configuration Pg. 1266, 1270 disable PWM Block
    PWM_BLOCK->_0_CTL   &= ~PWMX_CTL_MODE;                  // 10 Pg.1270 -> select countdown mode by clearing the bit field [1]
    PWM_BLOCK->_0_GENA = PWMX_GEN_A_CTL;                    // 11 Pg.1282 -> PWM generator for countdown mode with immediate action
                                                            //            -> bits[7:6] = 0x02 - signal down when comparator matches
                                                            //            -> bits[3:2] = 0x0C - signal up when load register reloads

    PWM_BLOCK->_0_GENB = PWMX_GEN_B_CTL;                    // 12 Pg.1282 -> PWM generator for countdown mode with immediate action
                                                            //            -> bits[11:10] = 0x02 - signal down when comparator matches
                                                            //            -> bits[3:2] = 0x0C - signal up when load register reloads
    PWM_BLOCK->INVERT |= PWMX_INVERT_PWM_BLK_INV;           // 12b Pg.1249-> Invert PWM output signal
    PWM_BLOCK->_0_LOAD = ((sysClkFreq/64)/(periodFreq))-1;  // 13 Pg.1278 -> set period in load register -> see header comment above (#08)
                                                            //            -> Desired PWM Freq (50Hz)
                                                            //            -> PWM Clock source freq (system clock/64) -> step 06
                                                            //            -> PWM Clock source freq (16MHz/64 = 250KHz)
                                                            //            -> load value = 250kHz/50Hz  = 5000 ticks per sec
                                                            //            -> load value = 5000-1(4999) zero based counter
    // continue from here
    counterValue = (PWM_BLOCK->_0_LOAD)&0xFFFF;
    PWM_BLOCK->_0_CMPA = counterValue/2;                    // 14 Pg.1280 -> Sets the value to compared against the counter
                                                            //            -> Set pulse width in comparator 1 -> 2499 (50% duty cycle)
    PWM_BLOCK->_0_CMPB = counterValue/2;                    // 15 Pg.1281 -> Sets the value to compared against the counter
                                                            //            -> Set pulse width in comparator 1 -> 2499 (50% duty cycle)

    PWM_BLOCK->_0_CTL    = PWMX_CTL_EN;                     // 16 Pg.1266 -> Start the timers in PWM generator 0
    PWM_BLOCK->ENABLE |= PWMX_ENABLE_PWM_BLK_EN;            // 17 Pg.1247, 1248 -> Enable PWM Output (MnPMW0 & MnPWM1)
}




//---- PWM_SetDutyCycle
//---- Set the duty cycle of both comparators with the flag option (selector)
//---- duty cycle is set based on the PWM initialised load counter value.
//---- selector = 0, updates the dutycycle of both comparators
//---- selector = 1, updates the dutycycle of only comparator A -> left wheel
//---- selector = 2, updates the dutycycle of only comparator B -> right wheel
void PHP_PWM_SetDutyCycle(PWM0_Type *PWM_Block, int dutyCycle, int selector){

    int counterValue = (PWM_Block->_0_LOAD)&0xFFFF;        // load register is 16bit register other bit fields are reserved
    counterValue = (dutyCycle * counterValue)/100;  // calculate new period value for duty cycle based on counter load val

    switch (selector) {

        case 0:                                     // Update both comparator to have same dutycycle
            PWM_Block->_0_CMPA = counterValue;
            PWM_Block->_0_CMPB = counterValue;
            break;

        case 1:                                     // Update both comparator A only with new dutycycle
            PWM_Block->_0_CMPA = counterValue;
            break;

        case 2:
            PWM_Block->_0_CMPB = counterValue;           // Update both comparator B only with new dutycycle
            break;

        default:                                    // do nothing
            break;
    }


}


// This function deactivate the PWM generator output
void PHP_PWM_Disable(PWM0_Type *PWM_Block){

    PWM_BLOCK->ENABLE &= ~PWMX_ENABLE_PWM_BLK_EN;            // 17 Pg.1247, 1248 -> Enable PWM Output (MnPMW0 & MnPWM1)
}


// This function activates the PWM generator output
void PHP_PWM_Enable(PWM0_Type *PWM_Block){

    PWM_BLOCK->ENABLE |= PWMX_ENABLE_PWM_BLK_EN;            // 17 Pg.1247, 1248 -> Enable PWM Output (MnPMW0 & MnPWM1)
}

