#include "php_uart.h"


//--- User Manual: See pg 893-897
//--- pg. 918, 916
//--- pg.911 - flag reg
//--- pg.906 data reg
//--- initialization and configuration pg 902

/* UART is configured with flag to indicated interrupt mode
 * @param int: interrupt mode flag (0 -> disable interrupt configuration)
 *                                 (1 -> enable interrupt configuration)
 *
 * @paran int: interrupt priority (0-7)
 * @return: void
 * */
void PHP_UART0_Init(int intFlag, int priority){
    uint32_t sysClkFreq = PHP_SysClock_GetFreq();             // Get the latest system clock frequency
    uint32_t integer    = sysClkFreq/(UART_BAUDRATE_PC * 16); // int(16000000/(16*115200)) = int(8.68055) -> 8    // pg. 914
    uint32_t remainder  = sysClkFreq%(UART_BAUDRATE_PC * 16); // int((0.68055 * 64) + 0.5)    = 44    // pg. 915
    remainder = ((remainder/UART_BAUDRATE_PC * 16)*64)+ (5/10);



    SYSCTL->RCGC1 |= (1U << 0);                           // 01. Enable UART0 Module from RunMode Clock Gating Ctrl Reg  pg.344, 460
    SYSCTL->RCGC2 |= (1U << 0);                           // 02. Enable GPIO A RunMode Clock from Gating ctrl reg pg.340
    PHP_DelayMs(10);                                      //     allow clock to stabilize
    //       Port A (UART0)
    UART0_PORT->AFSEL |= 0x03;                            // 03. Enable alt function for PA0 & PA1 (bit 0 & 1) pg.1351
    UART0_PORT->AMSEL &= ~0x03;                           //     Disable analog functions
    UART0_PORT->PCTL = (UART0_PORT->PCTL&0xFFFFFF00)+0x00000011;   // 04. Pin PA0 -> bit field [3:0] PMCx bit value ->(1) pg.688
    UART0_PORT->DEN |= 0x03;                              // 05. Configure pins as digital pins PA0 & PA1                                                                                                                                // 05. Pin PA1 -> bit field [7:4] PMCx bit value ->(1)

    // Setting up UART BAUD RATE
    UART0->CTL &= ~UART_CTL_UARTEN;                       // 06. Disable UART (0x00000001) pg. 918
    UART0->IBRD  = integer;                               // 07. int(16000000/(16*115200)) = int(8.68055) -> 8    // pg. 914
    UART0->FBRD  = remainder;                             // 08. int((0.68055 * 64) + 0.5)    = 44    // pg. 915

    UART0->LCRH |= UART_LCRH_WLEN_8;                     // 09. Configure word length (8bits)
    //UART0->LCRH |= UART_LCRH_FEN;                    // 10. Enable FIFO (0x10)

    // Configure interrupt
    // Configure the interrupt priority (See Page.105 on datasheet)
    // UART 0 -> Vector no:21, Interrupt no (IRQ):5
    /* Review files : C1203_05.PNG, C1203_06.PNG, C1203_07.PNG
     * m = (IRQ:5)/4 = 1 => Priority register number is 1 (NVIC_PRI1_R)
     * p = (IRQ:5)%4 = 1 => Priority bits are (8*p)+7, (8*p)+6, (8*p)+5 => 15, 14, 13; bits[15:13]
     * a = IRQ/32     = 0 => Interrupt enable register NVIC_EN0_R
     * b = IRQ%32     = 5 => Enable bit is 5 bit[5]
     * */
    if (intFlag){
        UART0->IM |= ((1U << 5) | (1U << 4));           // 11. Enable Transmit (TXIM bit 5) & Receive (RXIM bit 4) Interrupt => 0x30,

        //__NVIC_SetPriority(UART0_IRQn, priority);       // 12. Set priority to 3
         NVIC->IP[5] = (3<<5);                        //     bits 15-13 priority 3   Pg.152 (Alternative)
        // NVIC_PRI1_R |= ((1U<<14) | (1U<<13));        //     bits 15-13 priority 3   Pg.152 (Alternative)

        //__NVIC_EnableIRQ(UART0_IRQn);                   // 13. Pg.142 set Enable (EN0) - bit 30 (INT.NO)
         NVIC->ISER[0] |= (1U<<5);                    //     Alternative Pg.142
        // NVIC_EN0_R |= (1U<<5)                        //     Alternative Pg.142
    }

    UART0->CC |= 0x00;                                   // Configure the UART clock source by writing to the UARTCC register
    UART0->CTL|= 0x301;                                  // 11. Enable RXE, TXE, UART pg.918
}



/* UART is configured with flag to indicated interrupt mode
 * @param int: interrupt mode flag (0 -> disable interrupt configuration)
 *                                 (1 -> enable interrupt configuration)
 *
 * @paran int: interrupt priority (0-7)
 * @return: void
 *
 * */
void PHP_UART1_Init(int intFlag, int priority){
    uint32_t sysClkFreq = PHP_SysClock_GetFreq();               // Get the latest system clock frequency
    uint32_t integer    = sysClkFreq/(UART_BAUDRATE_WIFI * 16); // int(16000000/(16*115200)) = int(8.68055) -> 8    // pg. 914
    uint32_t remainder  = sysClkFreq%(UART_BAUDRATE_WIFI * 16); // int((0.68055 * 64) + 0.5)    = 44    // pg. 915
    remainder = ((remainder/UART_BAUDRATE_WIFI * 16)*64)+0.5;


	SYSCTL->RCGC1 |= (1U << 1);							  // 01. Enable UART Module from RunMode Clock Gating Ctrl Reg pg.344 or pg.460 (1U < 1) or 0x02;
	SYSCTL->RCGC2 |= (1U << 1); 						  // 02. Enable GPIO B RunMode Clock from Gating ctrl reg pg.340 or pg.464 (1U < 1) or 0x02;  Port B (UART1)
	
	UART1_PORT->AFSEL |= 0x03;	    					  // 03. Enable alt function for PB0 & PB1 (bit 0 & 1) pg.1351
	UART1_PORT->AMSEL &= ~0x03;                           //     Disable analog functions
	UART1_PORT->PCTL   = (UART1_PORT->PCTL&0xFFFFFF00)+0x00000011;  // 04. Pin PB0 -> bit field [3:0] PMCx bit value ->(1) pg.688
	UART1_PORT->DEN   |= 0x03;						      // 05. Configure pins as digital pins PB0 & PB1	((1U < 0) | (1U < 1))
														  // 05. Pin PA1 -> bit field [7:4] PMCx bit value ->(1)
	
	// Setting up UART BAUD RATE
	UART1->CTL &= ~(1U << 0 );							  // 06. Disable UART (0x00000001) pg. 918
    UART1->IBRD	= integer;							      // 07. int(16000000/(16*9600)) = int(8.68055) -> 8	// pg. 914
	UART1->FBRD	= remainder;					     	  // 08. int((0.166667 * 64) + 0.5)	= 11	// pg. 915
																						
	UART1->LCRH |= ((1U << 5) | (1U << 6));  	          // 09. Configure word length (8bits) pg.916 -> set bits 5 and 6
	//UART1_LCRH_R |= UART_LCRH_FEN;  				        // 10. Enable FIFO (0x10)
	//UART1_IFLS_R &= ~0x3F;                		        // clear TX and RX interrupt FIFO level fields

    // Configure interrupt
    // Configure the interrupt priority (See Page.105 on datasheet)
    // UART 0 -> Vector no:21, Interrupt no (IRQ):5
    /* Review files : C1203_05.PNG, C1203_06.PNG, C1203_07.PNG
     * m = (IRQ:6)/4 = 1 => Priority register number is 1 (NVIC_PRI1_R)
     * p = (IRQ:6)%4 = 2 => Priority bits are (8*p)+7, (8*p)+6, (8*p)+5 => 23, 22, 21; bits[23:21]
     * a = IRQ/32     = 0 => Interrupt enable register NVIC_EN0_R
     * b = IRQ%32     = 6 => Enable bit is 6 bit[6]
     * */
    if (intFlag){
        UART1->IM |= ((1U << 5) | (1U << 4));           // 11. Enable Transmit (TXIM bit 5) & Receive (RXIM bit 4) Interrupt => 0x30,
                                                        //     configure interrupt for TX FIFO <= 1/8 full
                                                        //     configure interrupt for RX FIFO >= 1/8 full
        //__NVIC_SetPriority(UART1_IRQn, priority);       // 12. Set priority to 3
         NVIC->IP[5] = (3<<5);                        //     bits 15-13 priority 3   Pg.152 (Alternative)
        // NVIC_PRI1_R |= ((1U<<14) | (1U<<13));        //     bits 15-13 priority 3   Pg.152 (Alternative)

        //__NVIC_EnableIRQ(UART1_IRQn);                   // 13. Pg.142 set Enable (EN0) - bit 30 (INT.NO)
         NVIC->ISER[0] |= (1U<<6);                    //     Alternative Pg.142
        // NVIC_EN0_R |= (1U<<6)                        //     Alternative Pg.142
    }


	UART1->CC  |= 0x00;								      // Configure the UART clock source by writing to the UARTCC register
	UART1->CTL |= 0x301;								  // 11. Enable RXE, TXE, UART pg.918
}




char PHP_UART_Rx(UART0_Type *UART){
    char data;

    while((UART->FR & UART_RXFE) != 0){       // Flag Reg -> RXFE (bit 4)Pg 911
                                                // Wait until Receiver register is not empty
        data = UART->DR&0xFF;                   // Get Data from Data register pg.906
    }
    return (unsigned char)data;

}


void PHP_UART_Tx(UART0_Type *UART, unsigned char data){

    while((UART->FR & UART_TXFF) !=0);         // Flag reg -> TXFF (bit 5) Pg 911
                                               // Wait until Transmitter register is not full
    UART->DR = data;

}

void PHP_UART_TxString(UART0_Type *UART, char *str){

    while (*str){
        PHP_UART_Tx(UART, *(str++));
    }

}


