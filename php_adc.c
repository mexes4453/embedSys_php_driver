/*
 * php_timer.c
 *
 *  Created on: 11 Aug 2021
 *      Author: chime
 */

#include "php_adc.h"

extern uint32_t sysClkFreq;                     // Variable created in php.c

/*
 * The ADC is configured to convert analog input from AIN0 with the
 * sample sequencer 3 and triggered by the software continously
 * AIN0 channel corresponds to PE3
 * */

void PHP_ADC_Init_SW_Trig(void){

    /* Device configuration PE3 - For analog input */
    SYSCTL->RCGC2  |=  (1U<<4);                   // 01. Pg.340, 460 - enable Run mode for GPIOE */
    GPIOE->DIR     &= ~(1U<<3);                   // 02. Configure pin on port E as input PE3
    GPIOE->DEN     &= ~(1U<<3);                   // 03. Disable digital configuration (Pin use for analog input)
    GPIOE->AMSEL   |=  (1U<<3);                   // 05. Enable analog Mode
    GPIOE->AFSEL   |=  (1U<<3);                   // 05. Enable analog function

    /* Initialise ADC Pg.817 */
    SYSCTL->RCGC0  |=  (1U<<16);                // 01. Pg.352, 457 - Enable Run mode for ADC Module 0
    ADC0->ACTSS    &= ~ADC_SS3;                 // 02. Pg.821 - disable sample sequencer 3 during configuration
    ADC0->EMUX     &= ~0xF000;                  // 03. Pg.833 - select SS3 trigger - default (Software)
    ADC0->SSMUX3    =0;                         // 04. Pg.875 - Select Analog Input Channel (AIN0 - PORT.E3)
    ADC0->SSCTL3   |= ((1U<<2)|(1U<<1));        // 05. Pg.876 - select Sample control option (interrupt & end of sequence)
    ADC0->ACTSS    |= ADC_SS3;                  // 06. Pg.821 - Enable sample sequencer 3 after configuration

}

/**
 * To start a conversion sequence on any of the sequencer (software trigger)
 *
 * volatile int result;
 *
 * while(1){
 *
 * ADC0->PSSI |= 8; // Pg.845 Using sample sequencer 3
 * while((ADC0->RIS & 8)==0); // Polling -> wait till conversion is completed
 * result = ADC0->SSFIFO3;    // Read converted data
 * ADC0->ISC = 8;             // Acknowledge flag -> reset for next conversion
 * }
 * */



/*
 * The ADC is configured to sample the launchpad in-built temperature sensor
 * using the wide timer as sample trigger which configured in periodic mode
 * with milliseconds time unit.
 *
 * WIDE TIMER A is configured in periodic mode.@16MHz clock freq
 * PARAMETERS
 *     TIMER0_Type * : Pointer to timer struct. (TIMER0, TIMER1, ... etc)
 *     uint32_t      : clockEnableBit           (TIMER0(1<<0); TIMER1(1<<1))
 *     uint32_t      : period - timer interval load value  (10 -> 10millisec)
 *
 **/
void PHP_ADC_Init_WTIMER_A_Ms_Trig(WTIMER0_Type *TimerBlock, uint32_t clockEnableBit,uint32_t period){

    //uint32_t delay;
    uint32_t sysClkFreq;
    sysClkFreq = PHP_SysClock_GetFreq();
    uint16_t timerPrescaler = 250;
    uint16_t intLoadValue = sysClkFreq/(timerPrescaler*1000); // freq

    SYSCTL->RCGCWTIMER |= clockEnableBit;    // Enable clock to wide timer block
    /* Initialise ADC Pg.817 */
    SYSCTL->RCGC0  |=  (1U<<16);                     // 01. Pg.352, 457 - Enable Run mode for ADC Module 0
    PHP_DelayUs(2);                                  //     Allow clock to stabilize;

    ADC0->ACTSS    &= ~ADC_SS3;                      // 02. Pg.821 - disable sample sequencer 3 during configuration
    ADC0->EMUX     &= ~0xF000;                       // 03. Pg.833 - select SS3 trigger - default (Software) - clear the bits
    ADC0->EMUX     |= 0x5000;                        // 03. Pg.833 - select SS3 trigger - timer trigger (0x5)
    ADC0->SSMUX3    =0;                              // 04. Pg.875 - Select Analog Input Channel (AIN0 - PORT.E3)
    ADC0->SSCTL3   |= ((1U<<3)|(1U<<2)|(1U<<1));     // 05. Pg.876 - select Sample control option (temp sensor, interrupt & end of sequence)
    ADC0->ACTSS    |= ADC_SS3;                       // 06. Pg.821 - Enable sample sequencer 3


    /* Initialise Wtimer for triggering ADC at given period Pg.722 */
    //SYSCTL->RCGCWTIMER |= clockEnableBit;    // Enable clock to wide timer block
    //delay = 1;
    PHP_DelayUs(2);                                  //     Allow clock to stabilize;
    TimerBlock->CTL   &= ~1U;                // Disable timer before config
    TimerBlock->CFG   |= 0x04;               // 32-bit mode
    TimerBlock->TAMR  |= 0x02;               // periodic mode & counter down
    TimerBlock->TAILR = (intLoadValue*period)-1;       // example: Timer A interval load reg @16MHZ clock freq
    TimerBlock->TAPR  = timerPrescaler-1;               // example: Timer A prescaler 16MHz/250 = 64000Hz
    TimerBlock->ICR  |= 0x01;                // Clear the timeout flag. pg.754
    TimerBlock->CTL  |= ((1U<<5)|(1U));      // Pg.793 - Enable ADC trigger and timer
}







