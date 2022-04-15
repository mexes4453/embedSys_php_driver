/*
 * php_timer.c
 *
 *  Created on: 11 Aug 2021
 *      Author: chime
 */

#include "php_timer.h"




extern uint32_t sysClkFreq;       // Variable created in php.c


/* TIMER is configured in one shot mode.@16MHz clock freq
 * PARAMETERS
 *     TIMER0_Type * : Pointer to timer struct. (TIMER0, TIMER1, ... etc)
 *     int           : clockEnableBit           (TIMER0(1<<0); TIMER1(1<<1))
 *     int           : timeUnitSel              (0: us; 1: ms) - select time unit
 *     uint32_t      : period - timer interval load value  (10 -> 10millisec or microsec)
 *
 **/
void PHP_TIMER_A_OneShotMode_Delay_Us_Ms(TIMER0_Type *TimerBlock, int clockEnableBit, int timeUnit, uint32_t period){
    uint16_t intLoadValue = 0;
    uint16_t timerPrescaler = 0;
    uint32_t sysClkFreq = PHP_SysClock_GetFreq();

    // timeUnit 0 for us; 1 for ms
    if (timeUnit){                                            // Configure for Milliseconds
        timerPrescaler = 250;
        intLoadValue   = sysClkFreq/(timerPrescaler*1000);    // example 16000000/(250*1000) = 64


    }else{                                                    // Configure for microseconds
        timerPrescaler = 1;
        intLoadValue   = sysClkFreq/(timerPrescaler*1000000); // example 16000000/(1*1000000) = 16

    }

    SYSCTL->RCGCTIMER |= (uint32_t)clockEnableBit;// Enable clock to timer block 0

    TimerBlock->CTL   &= ~1U;               // Disable timer before config
    TimerBlock->CFG   |= 0x04;              // 16-bit mode
    TimerBlock->TAMR  |= 0x01;              // periodic mode & counter down
    TimerBlock->TAILR = (intLoadValue*period)-1;       // Timer A interval load reg @16MHZ clock freq
    TimerBlock->TAPR  = timerPrescaler-1;              // Timer A prescaler 16MHz/250 = 64000Hz
    TimerBlock->ICR  |= 0x01;               // Clear the timeout flag
    TimerBlock->CTL  |= 0x01;               // Enable timer
    while((TimerBlock->RIS&0x01) == 0){}    // Wait till time out flag is set


}



/* TIMERB is configured in one shot mode.@16MHz clock freq (use sysClkFreq = PHP_SysClock_GetFreq();)
 * PARAMETERS
 *     TIMER0_Type * : Pointer to timer struct. (TIMER0, TIMER1, ... etc)
 *     int           : clockEnableBit           (TIMER0(1<<0); TIMER1(1<<1))
 *     int           : timeUnitSel              (0: us; 1: ms) - select time unit
 *     uint32_t      : period - timer interval load value  (10 -> 10millisec or microsec)
 *
 **/
void PHP_TIMER_B_OneShotMode_Delay_Us_Ms(TIMER0_Type *TimerBlock, int clockEnableBit, int timeUnit, uint32_t period){

    uint16_t intLoadValue = 0;
    uint16_t timerPrescaler = 0;
    uint32_t sysClkFreq = PHP_SysClock_GetFreq();

    // timeUnit 0 for us; 1 for ms
    if (timeUnit){                                            // Configure for Milliseconds
        timerPrescaler = 250;
        intLoadValue   = sysClkFreq/(timerPrescaler*1000);    // example 16000000/(250*1000) = 64


    }else{                                                    // Configure for microseconds
        timerPrescaler = 1;
        intLoadValue   = sysClkFreq/(timerPrescaler*1000000); // example 16000000/(1*1000000) = 16

    }
    SYSCTL->RCGCTIMER |= clockEnableBit;// Enable clock to timer block 0

    TimerBlock->CTL   &= ~1U;               // Disable timer before config
    TimerBlock->CFG   |= 0x04;              // 16-bit mode
    TimerBlock->TBMR  |= 0x01;              // periodic mode & counter down
    TimerBlock->TBILR = (intLoadValue*period)-1;      // Timer B interval load reg @16MHZ clock freq
    TimerBlock->TBPR  = timerPrescaler-1;             // Timer B prescaler 16MHz/250 = 64000Hz
    TimerBlock->ICR  |= 0x100;              // Clear the timeout flag. pg.754
    TimerBlock->CTL  |= 0x01;               // Enable timer
    while((TimerBlock->RIS&0x01) == 0){}    // Wait till time out flag is set

}


/* TIMERA is configured in periodic mode.@16MHz clock freq (use )
 * PARAMETERS
 *     TIMER0_Type * : Pointer to timer struct. (TIMER0, TIMER1, ... etc)
 *     int           : clockEnableBit           (TIMER0(1<<0); TIMER1(1<<1))
 *     int           : timeUnitSel              (0: us; 1: ms) - select time unit
 *     uint32_t      : period - timer interval load value  (10 -> 10millisec)
 *
 **/
void PHP_TIMER_A_PeriodicMode_Delay_Us_Ms(TIMER0_Type *TimerBlock, int clockEnableBit, int timeUnit, uint32_t period){
    int i;
	  uint32_t delay;
    uint16_t intLoadValue = 0;
    uint16_t timerPrescaler = 0;
    uint32_t sysClkFreq = PHP_SysClock_GetFreq();

    // timeUnit 0 for us; 1 for ms
    if (timeUnit){                                            // Configure for Milliseconds
        timerPrescaler = 250;
        intLoadValue   = sysClkFreq/(timerPrescaler*1000);    // example 16000000/(250*1000) = 64


    }else{                                                    // Configure for microseconds
        timerPrescaler = 1;
        intLoadValue   = sysClkFreq/(timerPrescaler*1000000); // example 16000000/(1*1000000) = 16

    }

    SYSCTL->RCGCTIMER |= clockEnableBit;// Enable clock to timer block 0
    delay = SYSCTL->RCGCTIMER;              // allow clock to stabilize
		
    TimerBlock->CTL   &= ~1U;               // Disable timer before config
    TimerBlock->CFG   |= 0x04;              // 16-bit mode
    TimerBlock->TAMR  |= 0x02;              // periodic mode & counter down
    TimerBlock->TAILR = (intLoadValue)-1;   // example: Timer A interval load reg @16MHZ clock freq
    TimerBlock->TAPR  = timerPrescaler-1;   // example: Timer A prescaler 16MHz/250 = 64000Hz
    TimerBlock->ICR  |= 0x01;               // Clear the timeout flag
    TimerBlock->CTL  |= 0x01;               // Enable timer

    for (i=0;i<period;i++){

         while((TimerBlock->RIS&0x01) == 0){}    // Wait till time out flag is set
        TimerBlock->ICR  |= 0x01;               // Clear the timeout flag
    }


}



/* TIMERB is configured in periodic mode.@16MHz clock freq
 * PARAMETERS
 *     TIMER0_Type * : Pointer to timer struct. (TIMER0, TIMER1, ... etc)
 *     int           : clockEnableBit           (TIMER0(1<<0); TIMER1(1<<1))
 *     int           : timeUnitSel              (0: us; 1: ms) - select time unit
 *     uint32_t      : period - timer interval load value  (10 -> 10millisec)
 *
 **/
void PHP_TIMER_B_PeriodicMode_Delay_Us_Ms(TIMER0_Type *TimerBlock, int clockEnableBit, int timeUnit, uint32_t period){
    int i;
    uint16_t intLoadValue = 0;
    uint16_t timerPrescaler = 0;
    uint32_t sysClkFreq = PHP_SysClock_GetFreq();

    // timeUnit 0 for us; 1 for ms
    if (timeUnit){                                            // Configure for Milliseconds
        timerPrescaler = 250;
        intLoadValue   = sysClkFreq/(timerPrescaler*1000);    // example 16000000/(250*1000) = 64


    }else{                                                    // Configure for microseconds
        timerPrescaler = 1;
        intLoadValue   = sysClkFreq/(timerPrescaler*1000000); // example 16000000/(1*1000000) = 16

    }

    SYSCTL->RCGCTIMER |= clockEnableBit;    // Enable clock to timer block 0

    TimerBlock->CTL   &= ~1U;               // Disable timer before config
    TimerBlock->CFG   |= 0x04;              // 16-bit mode
    TimerBlock->TBMR  |= 0x02;              // periodic mode & counter down
    TimerBlock->TBILR = (intLoadValue)-1;   // example: Timer B interval load reg @16MHZ clock freq
    TimerBlock->TBPR  = timerPrescaler-1;   // example: Timer B prescaler 16MHz/250 = 64000Hz
    TimerBlock->ICR  |= 0x100;              // Clear the timeout flag
    TimerBlock->CTL  |= 0x01;               // Enable timer

    for (i=0;i<period;i++){

        while((TimerBlock->RIS&0x01) == 0){}    // Wait till time out flag is set
        TimerBlock->ICR  |= 0x100;              // Clear the timeout flag
    }


}

/* This function initialise the port and timer peripheral
 * for capturing two consecutive positive rising edges of a periodic signal
 * */
void PHP_TIMER0_A_CaptureMode_EdgeTime_Init(void){

    /* Device input configuration PB6 */
    SYSCTL->RCGC2  |=  (1U<<1);                   // 01. Pg.340, 460 - enable Run mode for GPIOB */
    GPIOB->DIR     &= ~(1U<<6);                   // 02. Configure all pin on port B as input PB6
    GPIOB->DEN     |=  (1U<<6);                   // 03. Configure all pin on port B as digital
    GPIOB->AFSEL   |=  (1U<<6);                   // 04. Use alternate function
    GPIOB->AMSEL   &= ~(1U<<6);                   // 05. Disable analog function
    GPIOB->PCTL     =  (GPIOB->PCTL&0xF0FFFFFF)|0x0700000;// 04. Pin PB6 -> bit field [27:24] PMCx bit value ->(7) pg.688

    SYSCTL->RCGCTIMER |= 1U;                      // Enable clock to timer block 0
    TIMER0->CTL &= ~1U;                           // Disable timer before config
    TIMER0->CFG   |= 0x04;                        // 16-bit mode
    TIMER0->TAMR  |= 0x17;                        // capture mode & count up; edge time mode
    TIMER0->CTL   &= ~(0x0C);                     // capture positive (rising) edge
    TIMER0->CTL   |= 1U;                          // Enable timer

}

/* This function captures two consecutive positive rising edges of a periodic
 * signal using the timer0A.
 * It returns the time difference between the capture edges as the wave period
 * */
int PHP_TIMER0_A_EdgeTimeCapture(void){
    int lastEdge, thisEdge;

    // capture the first edge
    TIMER0->ICR |= (1U<<2);
    while(((TIMER0->RIS)&(1U<<2)) == 0);       // poll the bit till capture occurs
    lastEdge = TIMER0->TAR;                    // Timestamp saved

    // capture the first edge
    TIMER0->ICR |= (1U<<2);
    while(((TIMER0->RIS)&(1U<<2)) == 0);       // poll the bit till next capture occurs
    thisEdge = TIMER0->TAR;                    // Timestamp saved

    return (thisEdge - lastEdge)&0x00FFFFFF;   // Return the period as time difference
}


/* This function initialise the port and timer peripheral in edge count mode
* for capturing the positive rising edges counts using TIMER3A, PORTB (PB2)
* */
void PHP_TIMER3_A_CaptureMode_EdgeCount_Init(void){

    /* Device input configuration PB2 */
    SYSCTL->RCGC2  |=  (1U<<1);                   // 01. Pg.340, 460 - enable Run mode for GPIOB */
    GPIOB->DIR     &= ~(1U<<2);                   // 02. Configure all pin on port B as input PB2
    GPIOB->DEN     |=  (1U<<2);                   // 03. Configure all pin on port B as digital
    GPIOB->AFSEL   |=  (1U<<2);                   // 04. Use alternate function
    GPIOB->AMSEL   &= ~(1U<<2);                   // 05. Disable analog function
    GPIOB->PCTL     =  (GPIOB->PCTL&0xFFFFF0FF)|0x0000700;// 04. Pin PB6 -> bit field [11:8] PMCx bit value ->(7) pg.688

    SYSCTL->RCGCTIMER |= 1U<<3;             // Enable clock to timer block 3
    TIMER3->CTL   &= ~1U;                   // Disable timer before config
    TIMER3->CFG   |= 0x04;                  // 16-bit mode
    TIMER3->TAMR  |= 0x13;                  // capture mode & count up; edge count mode
    TIMER3->TAMATCHR = 0xFFFF;              // Setting the count limit (max 16bit value)
    TIMER3->TAPMR    = 0xFF;                // Using the match prescaler(max 8bit value) extends match to (max 24bits value)
    TIMER3->CTL   &= ~(0x0C);               // capture positive (rising) edge
    TIMER3->CTL   |= 1U;                    // Enable timer


}

/*  This function returns the number of edge count
 *  TIMER3A, PORTB (PB2)
 *  */
int PHP_TIMER3_A_EdgeCountCapture(void){

    return TIMER3->TAR;

}


void PHP_TIMER_WIDE_A_OneShotMode_DelaySec(WTIMER0_Type *TimerBlock, uint32_t clockEnableBit, uint32_t period){

    uint32_t sysClkFreq = PHP_SysClock_GetFreq();
    uint16_t timerPrescaler = 250;
    uint16_t intLoadValue = sysClkFreq/(timerPrescaler);


    SYSCTL->RCGCWTIMER |= clockEnableBit;// Enable clock to wide timer block

    TimerBlock->CTL   &= ~1U;                       // Disable timer before config
    TimerBlock->CFG   |= 0x04;                      // 32-bit mode
    TimerBlock->TAMR  |= 0x01;                      // One shot mode & counter down
    TimerBlock->TAILR = (intLoadValue*period)-1;    // example: Timer A interval load reg @16MHZ clock freq
    TimerBlock->TAPR  = timerPrescaler-1;           // example: Timer A prescaler 16MHz/250 = 64000Hz
    TimerBlock->ICR  |= 0x01;                       // Clear the timeout flag. pg.754
    TimerBlock->CTL  |= 0x01;                       // Enable timer
    while((TimerBlock->RIS&0x01) == 0){}            // Wait till time out flag is set

}


/* This function initialises the timer in PWM mode
 * TIMER is configured in periodic mode.@16MHz or 80MHz clock freq
 * (use sysClkFreq = PHP_SysClock_GetFreq(); to get the lastest clock freq
 * PARAMETERS
 *     TIMER0_Type * : Pointer to timer struct. (TIMER0, TIMER1, ... etc)
 *     int           : clockEnableBit           (TIMER0(1<<0); TIMER1(1<<1))
 *     int           : timeUnitSel              (0: us; 1: ms) - select time unit
 *     uint32_t      : period - timer interval load value  (10 -> 10millisec or microsec) */
void PHP_TIMER_A_PWM_Init(WTIMER0_Type *TimerBlock,
                          uint32_t clockEnableBit,
                          int timeUnit,
                          uint32_t period,
                          uint32_t duty){

    uint32_t intLoadValue = 0;
    uint32_t timerPrescaler = 0;
    uint32_t sysClkFreq = PHP_SysClock_GetFreq();

    // PWM output pin should be configured at device level by enabling the alternate function
    /* Device input configuration PB3 */
    /*
    SYSCTL->RCGC2  |=  (1U<<1);                   // 01. Pg.340, 460 - enable Run mode for GPIOB
    GPIOB->DIR     &= ~(1U<<2);                   // 02. Configure all pin on port B as input PB2
    GPIOB->DEN     |=  (1U<<2);                   // 03. Configure all pin on port B as digital
    GPIOB->AFSEL   |=  (1U<<2);                   // 04. Use alternate function
    GPIOB->AMSEL   &= ~(1U<<2);                   // 05. Disable analog function
    GPIOB->PCTL     =  (GPIOB->PCTL&0xFFFFF0FF)|0x0000700;// 04. Pin PB6 -> bit field [11:8] PMCx bit value ->(7) pg.688
     */

    // Pg.725
    SYSCTL->RCGCTIMER |= clockEnableBit;         // Enable clock to timer block 3
    PHP_DelayMs(10);
    TimerBlock->CTL   &= ~TIMER_CTL_TAEN_BIT;     // Pg.737 Disable timer before config
    TimerBlock->CFG   |= 0x04;                    // Pg.728 : 16-bit mode
    TimerBlock->TAMR  |= ( TIMER_TXMR_TXAMS_BIT  |         // Pg.733,735 : Enable PWM Mode
                           //(~TIMER_TXMR_TXCMR_BIT)  |         // Pg.733,735 : Enable Edge Count Mode
                           TIMER_TXMR_TXMR_BITS            // Pg.733,736 : Enable Periodic Timer Mode
                         );




    // timeUnit 0 for us; 1 for ms
    if (timeUnit){                                            // Configure for Milliseconds
        timerPrescaler = 250;
        intLoadValue   = ((sysClkFreq/(timerPrescaler*1000))*period)-1;    // example 16000000/(250*1000) = 64

    }else{                                                    // Configure for microseconds
        timerPrescaler = 100;
        intLoadValue   = (sysClkFreq/1000000);
        intLoadValue   = intLoadValue * period;
        intLoadValue   = intLoadValue/timerPrescaler; // example 16000000/(1*1000000) = 16
        intLoadValue   = intLoadValue-1;

    }
    //TimerBlock->TAPR  = timerPrescaler-1;               // example: Timer A prescaler 16MHz/250 = 64000Hz
    //TimerBlock->TAILR = intLoadValue;
    TimerBlock->TAILR = (sysClkFreq/2048)-1;
    //TimerBlock->TAMATCHR = (duty*intLoadValue)/period;  // Setting the count limit (max 16bit value)
    TimerBlock->TAMATCHR = (duty*TimerBlock->TAILR)>>10;  // Setting the count limit (max 16bit value)
    //TimerBlock->TAPMR    = 0xFF;                      // Using the match prescaler(max 8bit value) extends match to (max 24bits value)
    TimerBlock->CTL |= TIMER_CTL_TXPWML_BIT;            // Pg.737 : Configure PWM Output (inverted)
    TimerBlock->CTL |= TIMER_CTL_TAEN_BIT;              // Enable timer


}

