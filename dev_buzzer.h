/*
 * dev_buzzer.h
 * The buzzer is control by pulse width modulation (PWM) signal
 * The volume of the buzzer is control by adjusting the dutycycle of the PWM
 */

#ifndef DEV_BUZZER_H_
#define DEV_BUZZER_H_



#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"
#include "misc.h"
#include "php_timer.h"
#include "php_pwm.h"
#include "php_switch.h"




// GENERAL DEFINITION
#define BUZZER_PULSE_DURATION (2000)      // ms (2sec)
#define BUZZER_FREQ           (100)
#define BUZZER_DUTY           (70)


void DEV_BUZZER_Init(void);
void DEV_BUZZER_SetVolume(int);
void DEV_BUZZER_Play(void);




#endif /* DEV_BUZZER_H_ */
