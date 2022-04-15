/*
 * php_timer.h
 *
 *  Created on: 11 Aug 2021
 *      Author: chime
 */

#ifndef PHP_ADC_H_
#define PHP_ADC_H_

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"

#define ADC_SS3 (1U<<3)

/*
 * The ADC is configured to convert analog input from AIN0 with the
 * sample sequencer 3 and triggered by the software continously
 * AIN0 channel corresponds to PE3
 * */

void PHP_ADC_Init_SW_Trig(void);


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
void PHP_ADC_Init_WTIMER_A_Ms_Trig(WTIMER0_Type *, uint32_t ,uint32_t);

#endif /* PHP_ADC_H_ */
