
#include "php_lcd.h"

void PHP_LCD_Init(void){
	SYSCTL->RCGC2  |= (1U<<1);								// 01. Pg.340 - enable Run mode for GPIOB */
	LCD_PORT->DIR	|= 0xFF;									// 02. Configure all pin on port B as output
	LCD_PORT->DEN	|= 0xFF;									// 03. Configure all pin on port B as digital
	LCD_PORT->AFSEL &= ~0xFF;
	LCD_PORT->AMSEL &= ~0xFF;

}


