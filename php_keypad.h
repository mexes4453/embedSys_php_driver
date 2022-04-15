#ifndef PHP_KEYPAD_H
#define PHP_KEYPAD_H

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"

#define KEYPAD_ROW GPIOE
#define KEYPAD_COL GPIOC


void PHP_KEYPAD_Init(void);



/* This function reads the keypad to check if a button is pressed
 * An ASCII encoding of the pressed key is return if a key is pressed
 * else returns 0
 */
unsigned char PHP_KEYPAD_GetKey(void);

/* This function is an extended version of PHP_KEYPAD_GetKey()
 * its extended feature includes: keypress debouncing
 * */
unsigned char PHP_KEYPAD_GetChar(void);

#endif // PHP_KEYPAD_H

