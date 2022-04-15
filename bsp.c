/* Board Support Package (BSP) for the EK-TM4C123GXL board */

//#include <stdio.h>    // use for mpu and i2c test
//#include <stdlib.h>   // use for mpu and i2c test
#include "bsp.h"

void BSP_init() {

    PHP_SysClock_InitFastest();                              // Initialise clock to fastest Frequency 80MHz
    //PHP_SysClock_Freq_Test_Init();                           // Initialise port for system clock speed test
    //PHP_UART0_Init(0, 3);                                    // Port A (enable interrupt, Priority 3)
	  //BSP_LCD_Init();                                          // Port B
	  //PHP_KEYPAD_Init();                                       // Port E & C
	  PHP_UART1_Init();                                        // Port B
	  PHP_LED_Init();
	  //PHP_TIMER_SYSTICK_Init(0xFFFFFF);
	  //PHP_SWITCH_Init();
	  //PHP_ADC_Init_WTIMER_A_Ms_Trig(WTIMER0, (1U<<0), 2000);   // 2000ms -> 2sec
    //SystemCoreClockUpdate();
    //SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);
    //PHP_I2C_Init(I2C1);
    //DEV_MPU_Init(I2C1);                                      // Initialise MPU and I2C
    //StepperMotor_Init();
    //SERVOMOTOR_Init();
    //SERVOMOTOR_PWM_Init();
	  //DEV_BUZZER_Init();
	  //DEV_GLCD_Init();
	  DEV_RFID_Init();
}

#ifdef __SWITCH_CODE
void BSP_LCD_ReturnHome(void)  { BSP_LCD_Command(0x02);  /* Return Cursor home*/}
void BSP_LCD_ClearDisplay(void){ BSP_LCD_Command(1);    /* Clear Screen*/}
void BSP_LCD_DisplayOff(void)  { BSP_LCD_Command(0x08); /* Turn off Display*/}


void BSP_LCD_PrintStringData(char *dataTxt){
    while((*dataTxt) != 0){
        BSP_LCD_Data(*(dataTxt++));
    }
}



void BSP_LCD_NibbleWrite(uint8_t data, uint8_t control){
	  data 	  &= 0xF0; 		// clear lower nibble for control
	  control &= 0x0F;		// clear upper nibble for data
	
	  LCD_PORT->DATA = (data | control);					    // RS = 0; R/W = 0
	  LCD_PORT->DATA = (data | control | LCD_EN);			    // Pulse E
	  BSP_DelayUs(0);									        // allow Pulse width 230ns for data latching
	  LCD_PORT->DATA = data;
	  LCD_PORT->DATA = 0;

}

void BSP_LCD_Command(uint8_t command){
    //BSP_LCD_NibbleWrite((command>>4), 0);             // Lower nibble second
	  BSP_LCD_NibbleWrite(command&0xF0, 0);						    // Upper nibble first
	  BSP_LCD_NibbleWrite((uint8_t)(command<<4), 0);			// Lower nibble second
    (command < 4U)? BSP_DelayMs(2) : BSP_DelayUs(40);   // Commands below 4 requires 1.64ms
															                          // Other commands require at least 40us
}

void BSP_LCD_Data(char data){
	  BSP_LCD_NibbleWrite(data&0xF0, LCD_RS);						  // Upper nibble first
	  BSP_LCD_NibbleWrite((uint8_t)(data<<4), LCD_RS);						  // Lower nibble second
      BSP_DelayUs(40);  										  // Commands below 4 requires 1.64ms
}


void BSP_LCD_Init(void){
	  PHP_LCD_Init();					         // Configure LCD - Launchpad hardware pins
		
	  // Start initialisation sequence
	  BSP_DelayMs(15);
	  BSP_LCD_NibbleWrite(0x30, 0);             // Command Function set -> 8 bit data mode
	  BSP_DelayMs(20);
	  BSP_LCD_NibbleWrite(0x30, 0);             // Command Function set -> 8 bit data mode
	  BSP_DelayUs(100);
	  BSP_LCD_NibbleWrite(0x30, 0);             // Command Function set -> 8 bit data mode
	  BSP_DelayUs(40);
	
	  BSP_LCD_NibbleWrite(0x20, 0);				// Command Function set -> 4 bit data mode */
	  BSP_DelayUs(40);
	
	  BSP_LCD_Command(0x28);					// Set 4-bit data, 2-line, 5x8 font
	  BSP_LCD_Command(0x08);                    // Set -> Turn off Display
	  BSP_DelayMs(1000);
	  BSP_LCD_Command(0x0C);                    // Set -> Turn on Display
	  BSP_LCD_Command(0x0B);                    // Set -> Turn on Cursor
	  BSP_LCD_Command(0x06);					// Entry Mode Set -> Move cursor right
	  BSP_LCD_Command(0x01);					// Clear screen move cursor to home
      BSP_LCD_Command(0x0F);					// Turn on display, cursor blinking

	
}





void BSP_LCD_Test(void){
	
    char *testTxt= "Welcome Mexes";
	
    BSP_LCD_ClearDisplay();						 // Clear Screen
    BSP_LCD_ReturnHome();						 // LCD cursor location
	
	BSP_DelayMs(500);
	  
	BSP_LCD_PrintStringData(testTxt);
	BSP_DelayMs(5000);
	BSP_LCD_ClearDisplay();                      // Clear Screen
	BSP_LCD_ReturnHome();                        // Set -> Return cursor to home
}


void BSP_UART_Test(UART0_Type * UART){
    
	  // Test should be done with UART0 passed in as the argument
    PHP_UART_Tx(UART, 'h');    // Transmit Data to PC
    PHP_UART_Tx(UART, 'e');    // Transmit Data to PC
    BSP_DelayMs(1000);
    PHP_UART_Tx(UART, '\n');    // Transmit Data to PC
    PHP_UART_Tx(UART, '\r');    // Transmit Data to PC
    PHP_UART_TxString(UART, "Hello\n");    // Transmit Data to PC
    PHP_UART_Tx(UART, '\r');    // Transmit Data to PC
}


// flash all LED
void BSP_TIMER_SYSTICK_Test_01(void){

    LED_PORT->DATA = (SysTick->VAL)>>20; // read current value and shift by 20

}



// Toggle Red LED every 1sec
void BSP_TIMER_SYSTICK_Test_02(void){

    PHP_TIMER_SYSTICK_DelayMs(1000); // initiate the systick
    LED_PORT->DATA ^= 2U;            // Toggle Red LED every 1sec

}



// Toggle LED using timer in one shot mode
void BSP_TIMER_A_Test_03(void){

    LED_PORT->DATA = 2U;
    PHP_TIMER_A_OneShotMode_Delay_Us_Ms(TIMER0, (1U<<0), 1, 2); // configure as msec
    LED_PORT->DATA = 0;
    PHP_TIMER_A_OneShotMode_Delay_Us_Ms(TIMER0, (1U<<0), 1, 5); // configure as msec

}


// Toggle LED using timer in periodic mode
void BSP_TIMER_A_Test_04(void){

    LED_PORT->DATA = 2U;
    PHP_DelayMs(2000); // configure as msec
    LED_PORT->DATA = 0;
    PHP_DelayMs(5000); // configure as msec

}


// Toggle LED using wide timer in one shot mode
void BSP_TIMER_A_Test_05(void){

    LED_PORT->DATA = 8U;     // Turn on green LED
    BSP_DelaySec(2);         // wide timer1A
    LED_PORT->DATA = 0;      // Turn off green LED
    BSP_DelaySec(2);         // wide timer1A

}


// Toggle LED using on-board switch configure with interrupt - falling edge
void BSP_INTERRUPT_Test_01(void){

    LED_PORT->DATA |= 0x02;     // Turn on red LED
    BSP_DelayMs(500);           // timer0A
    LED_PORT->DATA &= (uint32_t)~0x02;    // Turn off red LED
    BSP_DelayMs(500);           // timer0A

}





// --------------  BSP_SysClock_Freq_Test_Exec  -------------------------/
//
// To verify the system clock speed by toggle a pin
// PA2 is toggle on and off using the systick count every 1ms
// This generates a square wave to be measure with an Oscilloscope or
// logic analyzer
void BSP_SysClock_Freq_Test_Exec(void){
    PHP_SysClock_Freq_Test_Exec();

}


/* Display Temp data on LCD & Computer from in-built temp sensor
 * Also indication end of data conversion by ADC using LED */
void BSP_ADC_Test_01(void){

    volatile int tempData;
    char *txt1 = " Temp: ";
    char *txt2;
    char *txt3 = "c.deg";

    while ((ADC0->RIS & ADC_SS3) == 0);               // Pg. 823 - Polling -> waiting till conversion is completed
    tempData = (147 - (247 * ADC0->SSFIFO3)/4096);    // 12 bit (4096) ADC


    /* indicate conversion completion with RED LED */
    LED_PORT->DATA_Bits[LED_RED] |= LED_RED;          // Turn on red LED
    BSP_DelayMs(100);                                 // timer0A - delay 100ms
    LED_PORT->DATA_Bits[LED_RED] &= ~LED_RED;         // Turn off red LED

    /* Display Data on LCD & Computer screen */

    txt2 = BSP_Int2CharStr(tempData);
    BSP_LCD_PrintStringData(txt1); PHP_UART_TxString(UART0, txt1);
    BSP_LCD_PrintStringData(txt2); PHP_UART_TxString(UART0, txt2);
    BSP_LCD_PrintStringData(txt3); PHP_UART_TxString(UART0, txt3);
    BSP_LCD_ReturnHome(); PHP_UART_TxString(UART0, "\n\r");

    ADC0->ISC |= ADC_SS3;                             // Pg. 828 - Acknowledge conversion completion clear interrupt
}




void BSP_MPU_I2C_Test(I2C0_Type * I2C){
    char ERROR;
    int  accX, accY, accZ, GyroX, GyroY, GyroZ, Temper;
    int AX, AY, AZ, t, GX, GY, GZ;
    char SampleDataBuff[14];
    //static char msg[20];
    char *txt1 = "\n\rGX: ";
    char *txt2;
    /* Populate buffer with MPU sensor data sample*/
    ERROR = PHP_I2C_BurstRead(I2C, DEV_MPU_ADDR,
                              DEV_MPU_ACCEL_XOUT_H_ADDR,
                              MPU_SAMPLE_DATA_SIZE,
                              SampleDataBuff);

    /* Merge the byte data for each sample data into 16 bits */
    accX   = (int) ( (SampleDataBuff[0]  << 8 ) |SampleDataBuff[1] );
    accY   = (int) ( (SampleDataBuff[2]  << 8 ) |SampleDataBuff[3] );
    accZ   = (int) ( (SampleDataBuff[4]  << 8 ) |SampleDataBuff[5] );
    Temper = (int) ( (SampleDataBuff[6]  << 8 ) |SampleDataBuff[7] );
    GyroX  = (int) ( (SampleDataBuff[8]  << 8 ) |SampleDataBuff[9] );
    GyroY  = (int) ( (SampleDataBuff[10] << 8 ) |SampleDataBuff[11] );
    GyroZ  = (int) ( (SampleDataBuff[12] << 8 ) |SampleDataBuff[13] );


    // Convert The Readings
    AX = (int)accX; ///16384.0;
    AY = (int)accY; ///16384.0;
    AZ = (int)accZ; ///16384.0;
    GX = (int)GyroX; ///131.0;
    GY = (int)GyroY; //131.0;
    GZ = (int)GyroZ; ///131.0;
    t = ((int)Temper/340)+(3653/100);

    PHP_UART_TxString(UART0, "\n\n\r*** NEW DATA *** ");

    txt1="\n\rAX: ";
    txt2 = BSP_Int2CharStr(AX);
    PHP_UART_TxString(UART0, txt1);
    PHP_UART_TxString(UART0, txt2);


    txt1="\n\rAY: ";
    txt2 = BSP_Int2CharStr(AY);
    PHP_UART_TxString(UART0, txt1);
    PHP_UART_TxString(UART0, txt2);



    txt1="\n\rAZ: ";
    txt2 = BSP_Int2CharStr(AZ);
    PHP_UART_TxString(UART0, txt1);
    PHP_UART_TxString(UART0, txt2);


    txt1="\n\rGX: ";
    txt2 = BSP_Int2CharStr(GX);
    // Display data on PC Screen /
     //sprintf(msg,"Gx = %d \t",GX);
     PHP_UART_TxString(UART0, txt1);
     //BSP_LCD_PrintStringData(msg);
     //sprintf(msg,"Gy = %d \t",GY);
     PHP_UART_TxString(UART0, txt2);


     txt1="\n\rGY: ";
     txt2 = BSP_Int2CharStr(GY);
     PHP_UART_TxString(UART0, txt1);
     PHP_UART_TxString(UART0, txt2);

     txt1="\n\rGZ: ";
     txt2 = BSP_Int2CharStr(GZ);
     PHP_UART_TxString(UART0, txt1);
     PHP_UART_TxString(UART0, txt2);


     //BSP_LCD_PrintStringData(msg);
     //sprintf(msg,"Gz = %d \t",GZ);    PHP_UART_TxString(UART0, msg);  BSP_LCD_PrintStringData(msg);
     //sprintf(msg,"Ax = %d \t",AX);    PHP_UART_TxString(UART0, msg);  BSP_LCD_PrintStringData(msg);
     //sprintf(msg,"Ay = %d \t",AY);    PHP_UART_TxString(UART0, msg);  BSP_LCD_PrintStringData(msg);
     //sprintf(msg,"Ax = %d \r\n",AZ);  PHP_UART_TxString(UART0, msg);  BSP_LCD_PrintStringData(msg);

     /* indicate reading completion with RED LED */
     LED_PORT->DATA_Bits[LED_GREEN] |= LED_GREEN;          // Turn on LED_GREEN
     BSP_DelayMs(100);                                     // timer0A - delay 100ms
     LED_PORT->DATA_Bits[LED_GREEN] &= ~LED_GREEN;         // Turn off LED_GREEN


}





// BSP - STEPPER MOTOR



//void BSP_STEPPERMOTOR_Test(int * counter, int * motor_dir){
void BSP_STEPPERMOTOR_Test(void){
    //static int rotor_step_counter;
    extern int STEPPERMOTOR_Step_Counter;
    extern int STEPPERMOTOR_Dir;

    if (STEPPERMOTOR_Step_Counter > 2048){
        STEPPERMOTOR_Dir = 0;
        PHP_LED_Toggle_Green();
    }else if (STEPPERMOTOR_Step_Counter < 1){
        STEPPERMOTOR_Dir = 1;
        PHP_LED_Toggle_Green();
    }

    STEPPERMOTOR_Rotate();


}




void BSP_SERVOMOTOR_Test(void){
    int duty = 0;

    //duty = 500; SERVOMOTOR_Rotate(&duty);  // right position (0deg)
    duty = (25/10); SERVOMOTOR_Rotate_PWM(&duty);
    PHP_LED_Toggle_Red();

    //duty = 1030; SERVOMOTOR_Rotate(&duty);  // right position (45deg)
    duty = (515/100); SERVOMOTOR_Rotate_PWM(&duty);
    PHP_LED_Toggle_Red();

    //duty = 1560; SERVOMOTOR_Rotate(&duty);  // middle position (90deg)
    duty = (78/10); SERVOMOTOR_Rotate_PWM(&duty);
    PHP_LED_Toggle_Blue();

    //duty = 2090; SERVOMOTOR_Rotate(&duty);  // left position (135deg)
    duty = (1045/100); SERVOMOTOR_Rotate_PWM(&duty);
    PHP_LED_Toggle_Green();

    //duty = 2560; SERVOMOTOR_Rotate(&duty);  // left position (180deg)
    duty = (128/10); SERVOMOTOR_Rotate_PWM(&duty);
    PHP_LED_Toggle_Green();

}



void BSP_BUZZER_Test(void){

    DEV_BUZZER_Play();
    PHP_LED_Toggle_Green();
    BSP_DelayMs(2000);
}



void BSP_GLCD_Test(char font_table[][6]){

    int i;
	
    PHP_LED_Toggle_Blue();
    DEV_GLCD_FillScreen();
    BSP_DelayMs(1000);
    DEV_GLCD_ClearScreen();
    BSP_DelayMs(1000);
    PHP_LED_Toggle_Blue();
    BSP_DelayMs(2000);

    PHP_LED_Toggle_Green();
    for (i=0; i<6; i++) DEV_GLCD_DC_Write(GLCD_DATA, font_table[0][i]);  // Display A
    BSP_DelayMs(2000);

    PHP_LED_Toggle_Green();
    for (i=0; i<6; i++) DEV_GLCD_DC_Write(GLCD_DATA, font_table[1][i]);  // Display B
    BSP_DelayMs(2000);

    PHP_LED_Toggle_Green();
    for (i=0; i<6; i++) DEV_GLCD_DC_Write(GLCD_DATA, font_table[2][i]);  // Display C
    BSP_DelayMs(2000);
}
#endif


void BSP_RFID_TestRegisters(uint8_t *DataPtr){
    enum RFID_StatusCode RFID_SRC;
	  uint8_t buff_size = 2;
	  devRfidPiccUid_type rfidPiccUid; // GLOBAL declaration
    devRfidPiccUidIdSelState_type devRfidPiccUidIdSelState={1,0,0,0,0};
/*		 cascadeLevel;
		   uidIndex;
		   identDone;
		   selectDone;
		   stateActive;
*/
    DEV_RFID_PiccUidInit(&rfidPiccUid);
	  DEV_RFID_RegByteRead(CommandReg, &DataPtr[0]);
/*  
	  
		DEV_RFID_RegByteRead(TReloadRegL, &DataPtr[0]); //
		DEV_RFID_RegByteRead(TReloadRegL, &DataPtr[0]); //
			
	  DEV_RFID_RegByteWrite(TReloadRegL, 0x78);
		DEV_RFID_RegByteRead(TReloadRegL, &DataPtr[0]); //
	  DEV_RFID_RegSetBitMask(TReloadRegL, 0x80);
		DEV_RFID_RegByteRead(TReloadRegL, &DataPtr[0]); //0xF8

	
		
	  DEV_RFID_RegByteWrite(TReloadRegL, 0x8F);
		DEV_RFID_RegByteRead(TReloadRegL, &DataPtr[0]); //		
		DEV_RFID_RegClearBitMask(TReloadRegL, 0x08);
		DEV_RFID_RegByteRead(TReloadRegL, &DataPtr[0]); //0x87

		DEV_RFID_RegByteRead(VersionReg, &DataPtr[0]); //0x92
  
	 DataPtr[2] = 0x87;
	 DataPtr[3] = 0x88;
	 
	 DEV_RFID_RegByteRead(TReloadRegH, &DataPtr[0]); //
	 DEV_RFID_RegByteRead(TReloadRegL, &DataPtr[1]); //
	 DEV_RFID_RegBurstRead(TReloadRegH, 2, &DataPtr[0]);
	 DEV_RFID_RegBurstWrite(TReloadRegH, 2, &DataPtr[2]);
	 DEV_RFID_RegBurstRead(TReloadRegH, 2, &DataPtr[4]);

   DataPtr[0] = 0x11;
	 DataPtr[1] = 0x22;
	 DataPtr[2] = 0x33;
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]); 
   DEV_RFID_FifoRegBurstRead(FIFODataReg, 3, &DataPtr[3]);
   DEV_RFID_RegSetBitMask(FIFOLevelReg, RFID_PCD_R_FlushFIFO);
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]); 
   DEV_RFID_FifoRegBurstRead(FIFODataReg, 3, &DataPtr[3]);


   DEV_RFID_FifoRegBurstWrite(FIFODataReg, 3, &DataPtr[0]);
   
	 DEV_RFID_RegByteWrite(FIFODataReg, DataPtr[0]);
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]);
	 DEV_RFID_RegByteWrite(FIFODataReg, DataPtr[1]);
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]);
	 DEV_RFID_RegByteWrite(FIFODataReg, DataPtr[2]);
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]);
	 
	 
	 DEV_RFID_FifoRegBurstRead(FIFODataReg, 3, &DataPtr[3]);
	 
	 DEV_RFID_RegByteRead(FIFODataReg, &DataPtr[3]);
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]);
	 DEV_RFID_RegByteRead(FIFODataReg, &DataPtr[4]);
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]);
	 DEV_RFID_RegByteRead(FIFODataReg, &DataPtr[5]); 
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]);
	 
	
	 DEV_RFID_RegByteRead(FIFODataReg, &DataPtr[7]); 
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]);
	 
	 DEV_RFID_RegByteRead(FIFODataReg, &DataPtr[7]); 
	 DEV_RFID_RegByteRead(FIFOLevelReg, &DataPtr[6]);
	 */

  	RFID_SRC = DEV_RFID_PICC_RequestA(&DataPtr[0], &buff_size);	// The buffer to store the ATQA (Answer to request) in
		//RFID_SRC = DEV_RFID_PICC_WakeupA (&DataPtr[0], &buff_size);				                                                                                                                                 BSP_DelayUs(25);
    RFID_SRC = DEV_RFID_PICC_Select(&rfidPiccUid,		          // Pointer to Uid struct.
		                                &devRfidPiccUidIdSelState,
											              0); // ValidBits = 0               


}
//---------------------------------------------------------------------------------------


// Delay ticks microseconds (16MHz CPU Clock)
void BSP_DelayUs(uint32_t ticks){

    PHP_DelayUs(ticks);
    /*
	  uint32_t i, j;
	
	  for (i=0;i<ticks;i++){
		
		    for(j=0;j<3;j++){}	// do Nothing for 1 us
	  }
	  */
}


// Delay ticks Milliseconds (16MHz CPU Clock)
void BSP_DelayMs(uint32_t ticks){

    PHP_DelayMs(ticks);
    /*
	  uint32_t i, j;
	
	  for (i=0;i<ticks;i++){
		
		    for(j=0;j<3180;j++){}	// do Nothing for 1 ms
	}
	*/
}


// Delay ticks Seconds (16MHz CPU Clock)
void BSP_DelaySec(uint32_t ticks){
    PHP_TIMER_WIDE_A_OneShotMode_DelaySec(WTIMER1, (1U<<1), ticks);
}


#ifdef __SWITCH_CODE
void Q_onAssert(char const *module, int loc) {
    /* TBD: damage control */
    (void)module; // avoid the "unused parameter" compiler warning 
    (void)loc;    // avoid the "unused parameter" compiler warning 
    NVIC_SystemReset();
}


__attribute__((naked)) void assert_failed (char const *file, int line){
    /* TBD - Damage control function*/
    NVIC_SystemReset(); // Reset launchpad
}

#endif


char* BSP_Int2CharStr(int intVal){

    static char outBuffer[4];
    char inBuffer[4];
    int in_idx = 0, out_idx=0;
    char res = 0;


    do{                               // 6542 -> {'2', '4', '5', '6'}
        res = (char)(intVal % 10);
        inBuffer[in_idx] = res + 0x30;  // convert to ascii char and store in buffer
        in_idx++;

        intVal = intVal/10;

    }while(intVal);


    do{
        in_idx--;
        outBuffer[out_idx] = inBuffer[in_idx];
        out_idx++;

    }while(in_idx >= 0);

    return outBuffer;
}


// Interrupt handlers

void GPIOPortF_IRQHandler(void){

    int i=0;

    if (SWITCH_PORT->RIS & (1U<<4)){ // PORT.F4 (On-Board switch SW0) - is interrupt triggered?

        // Toggle green LED two times
        while (i < 2){
            LED_PORT->DATA_Bits[LED_GREEN] = LED_GREEN;
            BSP_DelayMs(500);
            LED_PORT->DATA_Bits[LED_GREEN] &= ~LED_GREEN;
            BSP_DelayMs(500);
            i++;
        }

        SWITCH_PORT->ICR |= (1U<<4);        // Clear interrupt flag for SW0


    }else if (SWITCH_PORT->RIS & (1U<<0)){ // PORT.F0 (On-Board switch SW4) - is interrupt triggered?

        // Toggle blue LED two times
        while (i < 2){
            LED_PORT->DATA_Bits[LED_BLUE] = LED_BLUE;
            BSP_DelayMs(500);
            LED_PORT->DATA_Bits[LED_BLUE] &= ~LED_BLUE;
            BSP_DelayMs(500);
            i++;
        }
        SWITCH_PORT->ICR |= (1U<<0);        // Clear interrupt flag for SW4


    }
    i=0; // reset toggle counter


}


void UART0_IRQHandler(void){
    char recvData;

    if (UART0->RIS & (1U<<4)){                       // interrupt trigger due to received data
        recvData = (char)UART0->DR;                  // retrieve data from uart data register
        LED_PORT->DATA = (uint32_t) (recvData << 1); // Used recv data to light up LEDs
        UART0->ICR = (1U<<4);                        // Acknowledge interupt


    }

}


void UART1_IRQHandler(void){
    char recvData;

        if (UART1->RIS & (1U<<4)){                        // interrupt trigger due to received data
            recvData = (char) UART1->DR;                  // retrieve data from uart data register
            LED_PORT->DATA = (uint32_t) (recvData << 1);  // Used recv data to light up LEDs
            UART1->ICR = (1U<<4);                         // Acknowledge interupt


        }
}
