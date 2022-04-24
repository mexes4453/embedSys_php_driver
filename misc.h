/*
 * misc.h
 *
 *  Created on: 12 Sep 2021
 *      Author: chime
 */

#ifndef MISC_H_
#define MISC_H_

#include <stdint.h>
/*------------------- my_pow ------------------------------------------------*
 *
 * This function is private - not visible to public - not declared in header file
 *
 * This function performs the power multiplication of a given base (2^^3) = 8
 *
 *
 * @param base  : int32_t (2)
 * @param power : int32_t (3)
 *
 * @return      : int32_t (8)
 *
 *-------------------------------------------------------------------------------*/
uint32_t MISC_MyPow(uint32_t, uint32_t );
uint32_t MISC_DivByFloat(uint32_t,    uint32_t, uint32_t, uint32_t);


/*------------------- MISC_MEM_Copy ------------------------------------------------*
 *
 *
 * This function copys data from one data structure to another using pointers. which
 * points to start address of both arrays (source, destination)
 *
 *
 * @param src       : *int8_t  () - pointer
 * @param dest      : *int8_t  () - pointer
 * @param byteCount : int16_t (3) 
 *
 * @return      : int32_t (8)
 *
 *-------------------------------------------------------------------------------*/
void MISC_MEM_Copy(uint8_t * dest,       // start address of destination data array
	                 uint8_t * src,        // start address of source data array                 
                   uint16_t byteCount);   // no of byte to copy



/*------------------- MISC_MEM_Move ------------------------------------------------*
 *
 *
 * This function moves data from one data structure to another using pointers. which
 * points to start address of both arrays (source, destination)
 *
 *
 * @param src       : *int8_t  () - pointer
 * @param dest      : *int8_t  () - pointer
 * @param byteCount : int16_t (3) 
 *
 * @return      : int32_t (8)
 *
 *-------------------------------------------------------------------------------*/
void MISC_MEM_Move(uint8_t * dest,       // start address of destination data array
	                 uint8_t * src,        // start address of source data array 
                   uint16_t byteCount);   // no of byte to move


/*------------------- MISC_Nbr2CharStr --------------------*
 *
 * This function converts unsigned integer to strings
 * with 10 digit limit : 1234567890 -> "1234567890"
 *
 * @param intVal : unsigned int - pointer 
 *
 * @return       : char* - pointer to character (strings)
 *---------------------------------------------------------*/
char* MISC_Nbr2CharStr(unsigned int nbr, uint8_t base);


//This function returns the number of characters in a string.
int MISC_StrLen(char *str);

#endif /* MISC_H_ */
