/*
 * php_timer.h
 *
 *  Created on: 11 Aug 2021
 *      Author: chime
 */

#ifndef PHP_TIMER_H_
#define PHP_TIMER_H_

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"


//extern static uint32_t sysClkFreq;

#define TIMER_TXMR_TXAMS_BIT   (1U<<3)
#define TIMER_TXMR_TXCMR_BIT   (1U<<2)
#define TIMER_TXMR_TXMR_BITS   (1U<<1)
#define TIMER_CTL_TXPWML_BIT   (1U<<6)
#define TIMER_CTL_TBEN_BIT     (1U<<8)
#define TIMER_CTL_TAEN_BIT     (1U<<0)

/* TIMER is configured in one shot mode.@16MHz clock freq
 * (use sysClkFreq = PHP_SysClock_GetFreq(); to get the lastest clock freq
 * PARAMETERS
 *     TIMER0_Type * : Pointer to timer struct. (TIMER0, TIMER1, ... etc)
 *     int           : clockEnableBit           (TIMER0(1<<0); TIMER1(1<<1))
 *     int           : timeUnitSel              (0: us; 1: ms) - select time unit
 *     uint32_t      : period - timer interval load value  (10 -> 10millisec or microsec)
 *
 **/
void PHP_TIMER_A_OneShotMode_Delay_Us_Ms(TIMER0_Type *, int, int, uint32_t);
void PHP_TIMER_B_OneShotMode_Delay_Us_Ms(TIMER0_Type *, int, int, uint32_t);

void PHP_TIMER_WIDE_A_OneShotMode_DelaySec(WTIMER0_Type *, uint32_t, uint32_t);



/******************************************************************************/
/* TIMER is configured in periodic mode.@16MHz or 80MHz clock freq
 * (use sysClkFreq = PHP_SysClock_GetFreq(); to get the lastest clock freq
 * PARAMETERS
 *     TIMER0_Type * : Pointer to timer struct. (TIMER0, TIMER1, ... etc)
 *     int           : clockEnableBit           (TIMER0(1<<0); TIMER1(1<<1))
 *     int           : timeUnitSel              (0: us; 1: ms) - select time unit
 *     uint32_t      : period - timer interval load value  (10 -> 10millisec or microsec)
 *
 **/
void PHP_TIMER_A_PeriodicMode_Delay_Us_Ms(TIMER0_Type *,  int, int, uint32_t);
void PHP_TIMER_B_PeriodicMode_Delay_Us_Ms(TIMER0_Type *,  int, int, uint32_t);



/******************************************************************************
 * This function initialise the port and timer peripheral
 * for capturing two consecutive positive rising edges of a periodic signal
 * */
void PHP_TIMER0_A_CaptureMode_EdgeTime_Init(void);

/* This function captures two consecutive positive rising edges of a periodic
 * signal using the timer0A.
 * It returns the time difference between the capture edges as the wave period
 * */
int PHP_TIMER0_A_EdgeTimeCapture(void);


/*****************************************************************************/

/* This function initialise the port and timer peripheral in edge count mode
 * for capturing the positive rising edges counts using TIMER3A, PORTB (PB2)
 * */
void PHP_TIMER3_A_CaptureMode_EdgeCount_Init(void);

/*  This function returns the number of edge count
 *  TIMER3A, PORTB (PB2)
 *  */
int PHP_TIMER3_A_EdgeCountCapture(void);
//**************************************************************************/

/* This function initialises the timer in PWM mode
 * TIMER is configured in periodic mode.@16MHz or 80MHz clock freq
 * (use sysClkFreq = PHP_SysClock_GetFreq(); to get the lastest clock freq
 * PARAMETERS
 *     TIMER0_Type * : Pointer to timer struct. (TIMER0, TIMER1, ... etc)
 *     int           : clockEnableBit           (TIMER0(1<<0); TIMER1(1<<1))
 *     int           : timeUnitSel              (0: us; 1: ms) - select time unit
 *     uint32_t      : period - timer interval load value  (10 -> 10millisec or microsec) */
void PHP_TIMER_A_PWM_Init(WTIMER0_Type *, uint32_t, int , uint32_t, uint32_t);


#endif /* PHP_TIMER_H_ */
