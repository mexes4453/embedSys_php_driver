#ifndef __BSP_H__
#define __BSP_H__

#include <stdint.h>         /* Standard integers. WG14/N843 C99 Standard */
#include "TM4C123GH6PM.h"
#include "php.h"
#include "php_lcd.h"
#include "php_keypad.h"
#include "php_uart.h"
#include "php_timer.h"
#include "php_led.h"
#include "php_switch.h"
#include "php_adc.h"
#include "php_spi.h"
#include "php_i2c.h"
#include "php_pwm.h"
#include "dev_mpu.h"
#include "dev_steppermotor.h"
#include "dev_servomotor.h"
#include "dev_buzzer.h"
#include "dev_glcd.h"
#include "dev_rfid.h"
//#include "qassert.h"


//char STEPPERMOTOR_Steps[4] = {0x9, 0x1, 0x5, 0x3}; //STEPPERMOTOR_PIN_PINK, STEPPERMOTOR_PIN_YELLOW, STEPPERMOTOR_PIN_ORANGE};
//extern int STEPPERMOTOR_Step_Counter;
//extern int STEPPERMOTOR_Dir;


/* system clock tick [Hz] */
//#define BSP_TICKS_PER_SEC 100U

//void BSP_init(void);
void BSP_init(void);

// LCD
void BSP_LCD_PrintStringData(char *);
void BSP_LCD_ReturnHome(void);
void BSP_LCD_ClearDisplay(void);
void BSP_LCD_DisplayOff(void);
void BSP_LCD_NibbleWrite(uint8_t, uint8_t);
void BSP_LCD_Command(uint8_t);
void BSP_LCD_Data(char);
void BSP_LCD_Init(void);
void BSP_LCD_Test(void);


// UART
void BSP_UART_Test(UART0_Type *);


// TIMER
void BSP_TIMER_SYSTICK_Test_01(void);    // flash all LED
void BSP_TIMER_SYSTICK_Test_02(void);    // Toggle Red LED every 1sec
void BSP_TIMER_A_Test_03(void);          // Toggle Red LED using timer in one shot mode
void BSP_TIMER_A_Test_04(void);          // Toggle Red LED using timer in periodic mode
void BSP_TIMER_A_Test_05(void);          // Toggle green LED using Wide timer in one shot mode


// INTERRUPTS
void BSP_INTERRUPT_Test_01(void);        // Toggle LED using on-board switch configure with interrupt - falling edge


// ADC
/* Display Temp data on LCD & Computer from in-built temp sensor
 * Also indication end of data conversion by ADC using LED */
void BSP_ADC_Test_01(void);



// MPU & I2C
// Read data from Motion processing Unit and display on computer screen
void BSP_MPU_I2C_Test(I2C0_Type * I2C);


void BSP_STEPPERMOTOR_Test(void);       // STEPPER MOTOR
void BSP_SERVOMOTOR_Test(void);         // SERVO MOTOR
void BSP_BUZZER_Test(void);             // BUZZER
void BSP_GLCD_Test(char [][6]);         // LCD GRAPHIC NOKIA
void BSP_RFID_TestRegisters(uint8_t *DataPtr);      // RFID

//------------------------------------------------------------------------
/* delay for a specified number of system clock ticks (polling) */
// Delay ticks microseconds (16MHz CPU Clock)
void BSP_DelayUs(uint32_t);


// Delay ticks Milliseconds (16MHz CPU Clock)
void BSP_DelayMs(uint32_t);

// Delay ticks Seconds (16MHz CPU Clock)
void BSP_DelaySec(uint32_t ticks);

// convert integer to char string a
char* BSP_Int2CharStr(int intVal);


// --------------  BSP_SysClock_Freq_Test_Exec  -------------------------/
//
// To verify the system clock speed by toggle a pin
// PA2 is toggle on and off using the systick count every 1ms
// This generates a square wave to be measure with an Oscilloscope or
// logic analyzer
void BSP_SysClock_Freq_Test_Exec(void);


// Interrupt handlers
void GPIOPortF_IRQHandler(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);

#endif // __BSP_H__
