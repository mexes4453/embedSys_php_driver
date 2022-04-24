
#include <stdint.h>
#include "bsp.h"


int main() {
    //char Error;
    //char mem_data;
    //char *txt2;
    //int counter = 1;
	  // font table for test graphic nokia lcd screen
	
	  /*
    static char font_table[][6] = {
                                   {0x7E, 0x11, 0x11, 0x11, 0x7E, 0}, // A
                                   {0x7F, 0x49, 0x49, 0x49, 0x36, 0}, // B
                                   {0x3E, 0x41, 0x41, 0x41, 0x22, 0}, // C


    };
	  */
	   
	   static uint8_t data[7] = {0,0,0,0,0,0,0};
	
    //PHP_SysClock_InitFastest();                              // Initialise clock to fastest Frequency 80MHz
    //PHP_LED_Init();

  	//DEV_GLCD_Init();
    BSP_init();

    //__enable_irq();   // enable interrupts globally CSPIE I

    while(1){
        //BSP_SysClock_Freq_Test_Exec();   // Toggle test Pin PA2 @1kHz
        //BSP_TIMER_SYSTICK_Test_01();   // flash all LED
        //BSP_TIMER_SYSTICK_Test_02();   // Toggle Red LED every 1sec
        //BSP_TIMER_A_Test_03();         // Toggle RED LED using timer in one shot mode
        //BSP_TIMER_A_Test_04();         // Toggle RED LED using timer in periodic mode
        //BSP_TIMER_A_Test_05();         // Toggle BLUE LED using Wide timer in one shot mode
        //BSP_INTERRUPT_Test_01();       // Toggle LED using on-board switch configure with interrupt - falling edge

        /* Display Temp data on LCD & Computer from in-built temp sensor
         * Also indication end of data conversion by ADC using LED */
        //BSP_ADC_Test_01();
        //BSP_MPU_I2C_Test(I2C1);

        /*
        Error = PHP_I2C_ByteWrite(I2C1,
                                      DEV_MPU_ADDR,
                                      MPU_REG_PWR_MGMT_1_ADDR,
                                      MPU_REG_PWR_MGMT_1_DATA);


        Error = PHP_I2C_ByteWrite(I2C1,0x68, 0x6B, 0x02);
        BSP_DelaySec(2);
        PHP_I2C_ByteRead(I2C1, 0x68, 0x6B, &mem_data);
        txt2 = BSP_Int2CharStr((uint8_t)mem_data);
        PHP_UART_TxString(UART0, txt2);
        */
        //BSP_STEPPERMOTOR_Test((&STEPPERMOTOR_Step_Counter), (&STEPPERMOTOR_Dir));
        //BSP_STEPPERMOTOR_Test();
        //BSP_SERVOMOTOR_Test();
        //BSP_BUZZER_Test();
        //BSP_GLCD_Test(font_table);
        BSP_RFID_TestRegisters(data);
				
				// Print the header section for reading tag memory sectors
				//DEV_RFID_PrintSectorHeader();
	}
}

/*
void SystemInit(void){
    // To be called by start up assembly code during initialisation
    SCB->CPACR |= 0x00F00000;       // Grant access to floating point coprocessor
}
*/
