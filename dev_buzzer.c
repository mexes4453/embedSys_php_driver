/*
 * dev_buzzer.c
 *
 *  Created on: 9 Oct 2021
 *      Author: chime
 */


#include "dev_buzzer.h"

void DEV_BUZZER_Init(void){

    PHP_PWM_Init(BUZZER_FREQ);                      // configured for PWM0 PB6 & PB7; Desire freq = 50Hz (20ms)
    PHP_PWM_Disable(PWM0);                          // Turn off buzzer after intialisation
    PHP_PWM_SetDutyCycle(PWM0, BUZZER_DUTY, 0);     // Set duty cycle for default loudness
}


void DEV_BUZZER_SetVolume(int duty_percent){

    PHP_PWM_SetDutyCycle(PWM0, duty_percent, 0);    // Set duty cycle for buzzer loudness

}



void DEV_BUZZER_Play(void){

    PHP_PWM_Enable(PWM0);                            // Turn on buzzer
    PHP_DelayMs(BUZZER_PULSE_DURATION);          // Sound duration
    PHP_PWM_Disable(PWM0);                           // Turn off buzzer


}





/*
void SERVOMOTOR_Rotate(unsigned int * duty_us){
    // Rotate to angle 0
    int i = 0;

    for (i=0; i<5; i++){
        SERVOMOTOR_PORT->DATA_Bits[SERVOMOTOR_CTRL_PIN] |= SERVOMOTOR_CTRL_PIN;
        PHP_DelayUs((*duty_us));
        SERVOMOTOR_PORT->DATA_Bits[SERVOMOTOR_CTRL_PIN] = ~SERVOMOTOR_CTRL_PIN;
        PHP_DelayUs(SERVOMOTOR_PWM_PERIOD - (*duty_us));
    }

    PHP_DelayMs(SERVOMOTOR_ROTATE_WAIT); // wait for motor arm to rotate to position

}
*/

