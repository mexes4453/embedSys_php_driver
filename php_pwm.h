/*
 * php_pwm.h
 *
 *  Created on: 28 Nov 2021
 *      Author: chime
 */

#ifndef PHP_PWM_H_
#define PHP_PWM_H_

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"

// PORT
#define PWM_GPIO_PORT GPIOB  // PB6 PB7 - PWM-MODULE 0/4 Pg. 1351
#define GPIO_PORT_CLK_EN_BIT (1U<<1)   // PORT B (1U<<1)
#define GPIO_PORT_PIN_6      (1U<<6)   // PIN 6
#define GPIO_PORT_PIN_7      (1U<<7)   // PIN 6
#define GPIO_PORT_PIN_ALL    (GPIO_PORT_PIN_6 | GPIO_PORT_PIN_7)   // PIN ALL
#define GPIO_PCTL_SET        (0x44000000)   // Pg.1351, 688
#define GPIO_PCTL_RST        (0xFF000000)   // Pg.1351, 688

// PWM
#define PWM_BLOCK               PWM0
#define SYSCTL_RCGC0_PWMX       (1U<<20)     // PWM0
#define SYSCTL_RCC_USEPWMDIV    (1U<<20)     // pg.254
#define SYSCTL_RCC_PWMDIV_RST   (~(3U<<17))  // Pg.255 bits[19:17] = 0
#define SYSCTL_RCC_PWMDIV_SET   (7U<<17)     // Pg.255 bits[19:17] = 0x07
#define PWMX_CTL_EN             (1U<<0)      // Pg.1266 bit[0]
#define PWMX_CTL_MODE           (1U<<1)      // Pg.1266 bit[1]  MODE SELECT BIT
#define PWMX_INVERT_PWM0_INV    (1U<<0)      // Pg.1249 bit[0] invert PWM0A output
#define PWMX_INVERT_PWM1_INV    (1U<<1)      // Pg.1249 bit[1] invert PWM0B output
#define PWMX_INVERT_PWM_BLK_INV  (PWMX_INVERT_PWM0_INV | PWMX_INVERT_PWM1_INV)
#define PWMX_GEN_A_CTL_ACTCMPAD (1U<<7)     // Pg.1282 bits[7:6] = 0x02 - signal down
#define PWMX_GEN_A_CTL_ACTLOAD  (3U<<2)      // Pg.1282 bits[3:2] = 0x0C - signal up
#define PWMX_GEN_A_CTL          (PWMX_GEN_A_CTL_ACTCMPAD | PWMX_GEN_A_CTL_ACTLOAD )
#define PWMX_GEN_B_CTL_ACTCMPBD (1U<<11)     // Pg.1282 bits[11:10] = 0x02 - signal down
#define PWMX_GEN_B_CTL_ACTLOAD  (3U<<2)      // Pg.1282 bits[3:2] = 0x0C - signal up
#define PWMX_GEN_B_CTL          (PWMX_GEN_B_CTL_ACTCMPBD | PWMX_GEN_B_CTL_ACTLOAD)
#define PWMX_ENABLE_PWM0_EN     (1U<<0)      // Pg.1247, 1248 bit[0] enable PWM0A output
#define PWMX_ENABLE_PWM1_EN     (1U<<1)      // Pg.1247, 1248 bit[1] enable PWM0B output
#define PWMX_ENABLE_PWM_BLK_EN  (PWMX_ENABLE_PWM0_EN | PWMX_ENABLE_PWM1_EN)
                                // Pg.1247, 1248 bit[1:0] enable PWM0 output


//----- PWM Init -------------
//----- See User Manual page 1239 for configuration steps
//----- https://microcontrollerslab.com/pwm-tm4c123-example-codes-tiva-c-launchpad/
//----- PWM CLOCK FREQ : 250kHz
//----- PWM Freq : 50Hz (desired)
//----- PWM Period : PWM CLOCK FREQ/PWM Freq = 250kHz/50Hz = 5000
//----- PWM Period -> load value = 5000-1 = 4999 (Counter is zero based)
//----- PWM default duty cycles = 50% (setting comparators to PWM Period/2 = 2499)
void PHP_PWM_Init(int);


//---- PWM_SetDutyCycle
//---- Set the duty cycle of both comparators with the flag option (selector)
//---- duty cycle is set based on the PWM initialised load counter value.
//---- selector = 0, updates the dutycycle of both comparators
//---- selector = 1, updates the dutycycle of only comparator A -> left wheel
//---- selector = 2, updates the dutycycle of only comparator B -> right wheel
void PHP_PWM_SetDutyCycle(PWM0_Type *, int, int);

// This function deactivate the PWM generator output
void PHP_PWM_Disable(PWM0_Type *);

// This function activates the PWM generator output
void PHP_PWM_Enable(PWM0_Type *);

#endif /* PHP_PWM_H_ */
