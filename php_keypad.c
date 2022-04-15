/*
 * php_keypad.c
 *
 *  Created on: 7 Aug 2021
 *  Author: chime
 */

#include "php_keypad.h"


void PHP_KEYPAD_Init(void){
    SYSCTL->RCGC2  |= (1U<<4) | (1U<<2);    // 01. Pg.340 - enable Run mode for GPIOE & GPIOC */

    KEYPAD_ROW->DIR   |= 0x0F;                 // 02. Configure pins 0-3 on port E as output
    KEYPAD_ROW->DEN   |= 0x0F;                 // 03. Configure pins 0-3 on port E as digital
    KEYPAD_ROW->ODR   |= 0x0F;                 // 04. Configure pins 0-3 on port E as open drain
    KEYPAD_ROW->AFSEL &= ~0x0F;                // 05. Disable alternate function
    KEYPAD_ROW->AMSEL &= ~0x0F;                // 06. Disable analog function

    KEYPAD_COL->DIR   &= ~0xF0;                // 07. Configure pins 4-7 on port C as input
    KEYPAD_COL->DEN   |= 0xF0;                 // 08. Configure pins 4-7 on port C as digital
    KEYPAD_COL->PUR   |= 0xF0;                 // 09. Enable pull ups for pins 4-7 port C
    KEYPAD_COL->AFSEL &= ~0xF0;                // 10. Disable alternate function
    KEYPAD_COL->AMSEL &= ~0xF0;                // 11. Disable analog function

}


/* This function reads the keypad to check if a button is pressed
 * An ASCII encoding of the pressed key is return if a key is pressed
 * else returns 0
 */
unsigned char PHP_KEYPAD_GetKey(void){
    const unsigned char keymap[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'},
    };

    int row, col;

    KEYPAD_ROW->DATA = 0;           // enable all rows;
    col = KEYPAD_COL->DATA&0xF0;    // read all columns
    if (col == 0xF0) return 0;      // no key pressed

    while (1){
        row = 0;
        KEYPAD_ROW->DATA = 0x0E;
        PHP_DelayUs(2);
        col = KEYPAD_COL->DATA & 0xF0;
        if (col != 0xF0) break;

        row = 1;
        KEYPAD_ROW->DATA = 0x0D;
        PHP_DelayUs(2);
        col = KEYPAD_COL->DATA & 0xF0;
        if (col != 0xF0) break;

        row = 2;
        KEYPAD_ROW->DATA = 0x0B;
        PHP_DelayUs(2);
        col = KEYPAD_COL->DATA & 0xF0;
        if (col != 0xF0) break;

        row = 3;
        KEYPAD_ROW->DATA = 0x07;
        PHP_DelayUs(2);
        col = KEYPAD_COL->DATA & 0xF0;
        if (col != 0xF0) break;

        return 0;                   // if no key is pressed
    }

    // program reaches here to find row if col exist by key pressed.
    if (col == 0xE0) return keymap[row][0]; // key in column 0
    if (col == 0xD0) return keymap[row][1]; // key in column 1
    if (col == 0xB0) return keymap[row][2]; // key in column 2
    if (col == 0x70) return keymap[row][3]; // key in column 3
    return 0;  // safe exit
}

unsigned char PHP_KEYPAD_GetChar(void){
    unsigned char key;

    // fetch key after release of last pressed key
    do{
        key = PHP_KEYPAD_GetKey();
        PHP_DelayMs(20);                    // wait for debounce

    }while(PHP_KEYPAD_GetKey() != key);

    // wait until the previous key is released
    do{
        while(PHP_KEYPAD_GetKey() != 0);
        //PHP_DelayMs(20);                    // wait for debounce

    }while(PHP_KEYPAD_GetKey() != 0);

    return key;
}
