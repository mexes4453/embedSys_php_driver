#ifndef PHP_H
#define PHP_H

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php_timer.h"

// RCC_Register Pg.254
// RCC2_Register Pg.260
#define RCC2_USERCC2   (1U<<31)        // Bit to extend and override RCC reg fields
#define RCC2_BYPASS2   (1U<<11)        // Bit to ignore PLL
#define RCC_XTAL       (0x1F << 6)     // Bits to input crystal freq selection
#define RCC_XTAL_16MHZ (0x15 << 6)     // Value to set crystal freq selection to 16MHz
#define RCC2_OSCSRC2   (7U<<4)         // Bits to select clock source oscillator
#define RCC2_PWRDN2    (1U<<13)        // Bit to turn on or off PLL
#define RCC2_DIV400    (1U<<30)        // Bit to select DIV400 or DIV200 (DIV400/2)
#define RCC_USESYSDIV  (1U<<22)        // Bit to enable the system clock divider
#define RCC2_SYSDIV2   (0x7F<<22)      // Bits to select which divisor is used to generate the
                                       // system clock from either the PLL output or the oscillator source
                                       // (depending on how the BYPASS2 bit is configured)
#define SYSCTL_RIS_PLLLRIS (1U<<6)     // Pg.244 - Bit to check the PLL Timer is ready



#define CLK_FREQ_TEST_PORT     GPIOA
#define CLK_FREQ_TEST_PIN     (1U<<2)   // PIN PA2
#define CLK_FREQ_TEST_PORT_EN (1U<<0)   // Enable gating clock for PORT A


/* Global Variables */
//static uint32_t sysClkFreq = 16000000; // default clock Frequency 16MHz




/* Functions */

// ------------PHP_SysClock_InitFastest------------
// Configure the system clock to run at the fastest
// and most accurate settings.  For example, if the
// LaunchPad has a crystal, it should be used here.
// Call BSP_Clock_GetFreq() to get the current system
// clock frequency for the LaunchPad.
// Input: none
// Output: none
void PHP_SysClock_InitFastest(void);



// ------------PHP_SysClock_GetFreq------------
// Return the current system clock frequency for the
// LaunchPad.
// Input: none
// Output: system clock frequency in cycles/second
uint32_t PHP_SysClock_GetFreq(void);





// --------------  PHP_SysClock_Freq_Test_Init  ---------/
//
// To verify the system clock speed by toggle a pin
// PA2 is configured for this purpose
void PHP_SysClock_Freq_Test_Init(void);



// --------------  PHP_SysClock_Freq_Test_Exec  -------------------------/
//
// To verify the system clock speed by toggle a pin
// PA2 is toggle on and off using the systick count every 1ms
// This generates a square wave to be measure with an Oscilloscope or
// logic analyzer
void PHP_SysClock_Freq_Test_Exec(void);



void PHP_TIMER_SYSTICK_Init(uint32_t);
void PHP_TIMER_SYSTICK_DelayMs(uint32_t); // @16MHz or 80MHz clock freq



// Delay ticks microseconds (16MHz CPU Clock)
void PHP_DelayUs(uint32_t);


// Delay ticks Milliseconds (16MHz CPU Clock)
void PHP_DelayMs(uint32_t);

// Delay ticks Seconds (16MHz CPU Clock)
void PHP_DelaySec(uint32_t);




#endif // PHP_H

