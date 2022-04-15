/*
 * misc.c
 *
 *  Created on: 12 Sep 2021
 *      Author: chime
 */

#include "misc.h"
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
uint32_t MISC_MyPow(uint32_t base, uint32_t power){
    uint32_t res=0;               // store the result of multiplication

    if (power==0){               // example: 2^^0 = 1
        res = 1;
    }else if (power==1){         // example: 2^^1 = 2
        res = base;
    }else{
        res = 1;                 // set result to 1 for multiplication
        while(power != 0){       // example: 2^^2 = 4
            res *= base;     // add multiplication result until power value is 0
            power--;             // update power value after multiplication
        }
    }
    return res;

}


uint32_t MISC_DivByFloat(uint32_t dividend,    uint32_t divisorInt,
                         uint32_t divisorFrac, uint32_t divisorFracSize)
{
    uint32_t quotient, divisor;

    divisorFracSize = MISC_MyPow(10, divisorFracSize);  // update fraction part (tenth, hundredth)
    divisor = (divisorInt*divisorFracSize) + divisorFrac;
    dividend = dividend * divisorFracSize;
    quotient = dividend / divisor;
    if ((dividend % divisor) >= 5) quotient++;
    return quotient;

}



void MISC_MEM_Copy(uint8_t * dest,       // start address of destination data array
	                 uint8_t * src,        // start address of source data array                 
                   uint16_t byteCount)   // no of byte to copy
{
    while (byteCount > 0){
			  *(dest) = *(src);               // copy data from current src to dest addr
			  dest++;                         // next destination addr for storage
			  src++;                          // next source addr to move data from
			  byteCount--;                    // decrement the bytecounter
		    
		}
}




void MISC_MEM_Move(uint8_t * dest,       // start address of destination data array
	                 uint8_t * src,        // start address of source data array 
                   uint16_t byteCount)   // no of byte to move
{
    while (byteCount > 0){
			  *(dest) = *(src);                // copy data from current src to dest addr
			  *(src)= 0;                       // clear data in current source pointer addr
			  dest++;                          // next destination addr for storage
			  src++;                           // next source addr to move data from
			  byteCount--;                     // decrement the bytecounter
		    
		}
}

