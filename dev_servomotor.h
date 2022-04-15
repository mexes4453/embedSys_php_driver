/*
 * dev_servomotor.h
 * The SG90 servo-motor is control by single pulse width modulation (PWM) signal
 * PWM Signal specification
 * ------------------------
 * Period : 20ms
 * duty   : 0.5ms (high output) -> 0   deg position
 * duty   : 1.0ms (high output) -> 45  deg position
 * duty   : 1.5ms (high output) -> 90  deg position
 * duty   : 2.0ms (high output) -> 135 deg position
 * duty   : 2.5ms (high output) -> 180 deg position
 *
 * Note that the duty value can be slightly adjusted for calibration purpose
 *
 *    duty (0.5, 1, 1.5, 2, 2.5ms)
 *    <--->
 *    ____
 *    |   |____________|
 *    <---------------->
 *         period (20ms)
 */

#ifndef DEV_SERVOMOTOR_H_
#define DEV_SERVOMOTOR_H_



#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"
#include "misc.h"
#include "php_timer.h"
#include "php_pwm.h"

//
// Define pins
// DEFINITIONS FOR INITIALISATION WITHOUT PWM
#define SERVOMOTOR_PORT GPIOE
#define SERVOMOTOR_PORT_GPIO_PORT_CLK_EN   (1U<<4)   // PORT E
#define SERVOMOTOR_CTRL_PIN                (1U<<3) //(1U << 3)


// DEFINITIONS FOR INITIALISATION WITH PWM
#define SERVOMOTOR_PWM_PORT GPIOB
#define SERVOMOTOR_PWM_PORT_GPIO_PORT_CLK_EN   (1U<<1)   // PORT B
#define SERVOMOTOR_PWM_CTRL_PIN   (1U << 2) //(1U << 2) PB2


// GENERAL DEFINITION


#define SERVOMOTOR_PWM_PERIOD (20000)      // 20ms (20000us)

#define SERVOMOTOR_ROTATE_WAIT (2000)      // ms (2)


void SERVOMOTOR_Init(void);
void SERVOMOTOR_PWM_Init(void);
void SERVOMOTOR_Rotate(unsigned int *);
void SERVOMOTOR_Rotate_PWM(int *);





#endif /* DEV_SERVOMOTOR_H_ */
