/*
 * dev_servomotor.c
 *
 *  Created on: 9 Oct 2021
 *      Author: chime
 */


#include "dev_servomotor.h"



void SERVOMOTOR_Init(void){

    SYSCTL->RCGC2  |= SERVOMOTOR_PORT_GPIO_PORT_CLK_EN;          // 01. Pg.340 - enable Run mode for GPIO
    SERVOMOTOR_PORT->DIR   |= SERVOMOTOR_CTRL_PIN;               // 02. Configure all pin on port e as output
    SERVOMOTOR_PORT->DEN   |= SERVOMOTOR_CTRL_PIN;               // 03. Configure all pin on port e as digital
    SERVOMOTOR_PORT->AFSEL &= ~SERVOMOTOR_CTRL_PIN;
    SERVOMOTOR_PORT->AMSEL &= ~SERVOMOTOR_CTRL_PIN;



}


void SERVOMOTOR_PWM_Init(void){

#ifdef __SWITCH_ON_CODE
/* Device input configuration PB3  for TIMER3*/
    SYSCTL->RCGC2  |=  SERVOMOTOR_PWM_PORT_GPIO_PORT_CLK_EN;    // 01. Pg.340, 460 - enable Run mode for GPIOB */
    //GPIOB->DIR     &= ~(1U<<2);                               // 02. Configure all pin on port B as input PB3
    SERVOMOTOR_PWM_PORT->DEN     |=  SERVOMOTOR_PWM_CTRL_PIN;       // 03. Configure all pin on port B as digital
    SERVOMOTOR_PWM_PORT->AFSEL   |=  SERVOMOTOR_PWM_CTRL_PIN;       // 04. Use alternate function
    SERVOMOTOR_PWM_PORT->AMSEL   &= ~SERVOMOTOR_PWM_CTRL_PIN;       // 05. Disable analog function
    SERVOMOTOR_PWM_PORT->PCTL    =  (SERVOMOTOR_PWM_PORT->PCTL&0xFFFFF0FF)|0x0000700; // 04. Pin PB3 -> bit field [15:12] PMCx bit value ->(7) pg.688

    // configure the timer mode for PWM
    PHP_TIMER_A_PWM_Init(TIMER3, (1U<<3), 0, 20000, 10000);       // timer3, clockEnable, timeunit(us), period(20000), duty(500us)
#endif

    PHP_PWM_Init(50);  // configured for PWM0 PB6 & PB7; Desire freq = 50Hz (20ms)
}


void SERVOMOTOR_Rotate_PWM(int * duty_percent){

    PHP_PWM_SetDutyCycle(PWM0, *duty_percent, 0);     // Set duty cycle for rotate angle
    PHP_DelayMs(SERVOMOTOR_ROTATE_WAIT);         // wait for motor arm to rotate to position

}



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
