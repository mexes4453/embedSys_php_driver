
#include "php.h"


static uint32_t sysClkFreq = 16000000; // default clock Frequency 16MHz

// ------------PHP_SysClock_InitFastest------------
// Configure the system clock to run at the fastest
// and most accurate settings.  For example, if the
// LaunchPad has a crystal, it should be used here.
// Call BSP_Clock_GetFreq() to get the current system
// clock frequency for the LaunchPad.
// Input: none
// Output: none


void PHP_SysClock_InitFastest(void){
    // pg. 231
    // 0) configure the system to use RCC2 for advanced features
    //    such as 400 MHz PLL and non-integer System Clock Divisor
    SYSCTL->RCC2 |= RCC2_USERCC2;                                       // 01. Pg.260 - Enable override of RCC register fields

    // 1) PLL is bypassed during initialization;
    SYSCTL->RCC2 |= RCC2_BYPASS2;                                       // 02. Pg.260 - PLL is bypassed during initialization;
                                                                        //              clock is derived from osc and divided
                                                                        //              by SYSDIV2
    // 2) select the crystal value and oscillator source. Pg.231
    SYSCTL->RCC  &= ~RCC_XTAL;                                          // 03. Pg.254 - Clear XTAL field, bit[10:6]
    SYSCTL->RCC  +=  RCC_XTAL_16MHZ;                                    // 04. Pg.256 - Configure for 16 MHz crystal bit[10:6] = (0x15)
    SYSCTL->RCC2 &= ~RCC2_OSCSRC2;                                      // 05. Pg.262 - Clear oscillator source field
                                                                        //              When cleared, bit[6:4] = 0 (MainOsc selected)
                                                                        //              Clock source configured as Main Oscillator
    // 3) Activate PLL by clearing PWRDN. Pg.231
    SYSCTL->RCC2 &= ~RCC2_PWRDN2;                                       // 06. Pg.261 - activate PLL by clearing PWRDN bit[13]

    // 4) set the desired system divider and the
    //    system divider least significant bit. Pg.231
    SYSCTL->RCC2 |= RCC2_DIV400;                                        // 07. Pg.260 - Use 400 MHz PLL bit[30] = 1
    SYSCTL->RCC  |= RCC_USESYSDIV;                                      // 08. Pg.255 - Enable the use of the system clock divider
    SYSCTL->RCC2 = (SYSCTL->RCC2&~RCC2_SYSDIV2)                         // 09. Pg.260 - Clear system clock divider field bits[28:22]
                  + (4<<22);                                            // 10. Pg.224 - Configure for 80 MHz clock

    // 5) wait for the PLL to lock by polling PLLLRIS. Pg.231
    while((SYSCTL->RIS & SYSCTL_RIS_PLLLRIS)==0){};                     // 11. Pg.244 - Poll until the bit[6] = 1; PLL TIMER is ready

    // 6) enable use of PLL by clearing BYPASS
    SYSCTL->RCC2 &= ~RCC2_BYPASS2;                                      // 12. Pg.260 - Disable PLL bypass for use as clock source
    sysClkFreq = 80000000;                                              // 13. Update system clock frequency value to 80MHz
}


// ------------PHP_SysClock_GetFreq------------
// Return the current system clock frequency for the
// LaunchPad.
// Input: none
// Output: system clock frequency in cycles/second
uint32_t PHP_SysClock_GetFreq(void){
  return sysClkFreq;

}




void PHP_TIMER_SYSTICK_Init(uint32_t CountValue){
    SysTick->CTRL = 0U;              // Disable systick during configuration
    SysTick->LOAD = CountValue;      // Load count value to timer
    SysTick->CTRL |= ((1U<<2)|(1U)); // Enable clk_src & Systick timer (no Interrupt)
}


void PHP_TIMER_SYSTICK_DelayMs(uint32_t countValue){
    // @16MHz clock freq
    // load count value for 1ms @16MHz clock frequency (use sysClkFreq = PHP_SysClock_GetFreq();)
    //uint32_t sysClkFreq = PHP_SysClock_GetFreq();
    PHP_TIMER_SYSTICK_Init(((countValue*sysClkFreq)/1000)-1); // example: countValue = (delay[1ms] * sysclk[16MHz]) - 1
    while((SysTick->CTRL & 0x10000) == 0){}                 // wait till the count bit is set
    SysTick->CTRL = 0U;                                     // Disable Systick timer


}



// --------------  PHP_SysClock_Freq_Test_Init  ---------/
//
// To verify the system clock speed by toggle a pin
// PA2 is configured for this purpose
void PHP_SysClock_Freq_Test_Init(void){

    // Configure Pin PA2
    SYSCTL->RCGC2  |= CLK_FREQ_TEST_PORT_EN;              // 01. Pg.340 - enable Run mode for GPIOA */
    CLK_FREQ_TEST_PORT->DIR   |=  CLK_FREQ_TEST_PIN;      // 02. Configure pin as output
    CLK_FREQ_TEST_PORT->DEN   |=  CLK_FREQ_TEST_PIN;      // 03. Configure pin as digital
    CLK_FREQ_TEST_PORT->AFSEL &= ~CLK_FREQ_TEST_PIN;
    CLK_FREQ_TEST_PORT->AMSEL &= ~CLK_FREQ_TEST_PIN;

}



// --------------  PHP_SysClock_Freq_Test_Exec  -------------------------/
//
// To verify the system clock speed by toggle a pin
// PA2 is toggle on and off using the systick count every 1ms
// This generates a square wave to be measure with an Oscilloscope or
// logic analyzer
void PHP_SysClock_Freq_Test_Exec(void){

    PHP_TIMER_SYSTICK_DelayMs(1);                                            // Delay 1ms
    CLK_FREQ_TEST_PORT->DATA_Bits[CLK_FREQ_TEST_PIN] |=  CLK_FREQ_TEST_PIN;  // Set pin output high
    PHP_TIMER_SYSTICK_DelayMs(1);                                            // Delay 1ms
    CLK_FREQ_TEST_PORT->DATA_Bits[CLK_FREQ_TEST_PIN] &= ~CLK_FREQ_TEST_PIN;  // Set pin output low
}






// Delay ticks microseconds (16MHz CPU Clock)
void PHP_DelayUs(uint32_t ticks){

    PHP_TIMER_A_PeriodicMode_Delay_Us_Ms(TIMER0,  (1U<<0), 0, ticks); // configured for usec
/*      uint32_t i, j;

      for (i=0;i<ticks;i++){

            for(j=0;j<3;j++){}  // do Nothing for 1 us
      }
*/
}


// Delay ticks Milliseconds (16MHz CPU Clock)
void PHP_DelayMs(uint32_t ticks){

    PHP_TIMER_A_PeriodicMode_Delay_Us_Ms(TIMER0,  (1U<<0), 1, ticks); // configured for msec
    /*
      uint32_t i, j;

      for (i=0;i<ticks;i++){

            for(j=0;j<3180;j++){}   // do Nothing for 1 ms
    }
    */
}

void PHP_DelaySec(uint32_t ticks){

    PHP_TIMER_WIDE_A_OneShotMode_DelaySec(WTIMER0, (1U<<0), ticks); // configured for secs

}
