#ifndef PHP_LCD_H
#define PHP_LCD_H

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"

// LCD
/*
	LCD Initialisation	- PORT_B on Launch pad
	RS	- PB0	(RS -> Register Select )
								-> 0 - Command Register
								-> 1 - Data Register

	RW	- PB1 (RW -> Read/Write)
								-> 1 - Read
								-> 0 - Write

	E	- PB2	(E -> Enable -> high to low Pulse width : 230ns )
	D4	- PB4	(4 Bit data option - only 4 data pins on LCD is utilised)
	D5	- PB5	(4 Bit data option - only 4 data pins on LCD is utilised)
	D6	- PB6	(4 Bit data option - only 4 data pins on LCD is utilised)
	D7	- PB7	(4 Bit data option - only 4 data pins on LCD is utilised)

*/
#define LCD_PORT GPIOB
#define LCD_RS (1U<<0)
#define LCD_RW (1U<<1)
#define LCD_EN (1U<<2)

void PHP_LCD_Init(void);



#endif // PHP_LCD_H

