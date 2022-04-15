/*
 * dev_rfid.c
 *
 *  Created on: 5 Dec 2021
 *      Author: chime
 */


#include "dev_rfid.h"

	

void DEV_RFID_Init(void){


    //---- initialise SPI Peripheral with clk speed 4.00Mbps----
    PHP_SPI_SSI_Init(RFID_SSI_CLK_SPD_INT,
                     RFID_SSI_CLK_SPD_FRAC,
                     RFID_SSI_CLK_SPD_FRAC_SIZE);

    // ----Configure other RFID pins------
    //PHP_GPIO_Init(RFID_PORT, RFID_PORT_CLK_EN_BIT, RFID_DC, 1U, 0U, 0U);
	  PHP_GPIO_Init(RFID_PORT, RFID_PORT_CLK_EN_BIT, RFID_RST, 1U, 0U, 0U);    // PB3


    //----- Reset RFID Controller--------
	  // Hard Reset
	  if (RFID_PORT->DATA_Bits[RFID_RST] != RFID_RST)  // Check RFID MFRC522 chip status (0) off (1) on
		{
			RFID_PORT->DATA_Bits[RFID_RST] = RFID_RST;     // Turn on chip if off
			PHP_DelayMs(100);
		}
		else
		{ // Soft Reset
		    DEV_RFID_RegByteWrite(CommandReg, RFID_PCD_CMD_SoftReset);
		    PHP_DelayMs(100);
		}
		
	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	DEV_RFID_RegByteWrite(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	DEV_RFID_RegByteWrite(TPrescalerReg, 0xA9);	// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25us.
	DEV_RFID_RegByteWrite(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = ,, ie 25ms before timeout.
	DEV_RFID_RegByteWrite(TReloadRegL, 0xE8);
	DEV_RFID_RegByteWrite(TxASKReg, 0x40);		  // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	DEV_RFID_RegByteWrite(ModeReg, 0x3D);		    // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	DEV_RFID_SwitchOnAntenna();						      // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
		
}



void DEV_RFID_RegByteWrite(uint8_t reg, uint8_t data){
    PHP_SPI_SSI_RegByteWrite(RFID_SSI_BLK, reg, data);      // Transmit data to given register                                                      
    
}



void DEV_RFID_RegBurstWrite(uint8_t reg, uint8_t byteCount, uint8_t *byteData)
{
    PHP_SPI_SSI_RegBurstWrite(RFID_SSI_BLK, reg, byteCount, byteData); // Transmit data to given register
}

void DEV_RFID_FifoRegBurstWrite(uint8_t reg, uint8_t byteCount, uint8_t *byteDataTx)
{
		/* default mode is write mode (reg[7]   = 0);
     *                 write mode  reg[7]   = 0 
	   *                             reg[6:1] = reg addr
	   *                             reg[0]   = 0  
     */
	  while (byteCount > 0){                                      // read FIFO for bytecount
	      PHP_SPI_SSI_RegByteWrite(RFID_SSI_BLK, reg, *byteDataTx); // read FIFO register
			  byteCount--;																						// update bytecount to be read
			  byteDataTx++;																						// update data addr for next storage
		}
}

/* 
 * ---------------------------------- DEV_RFID_RegByteRead -------------------------------------/
 * Pg.10,11 Datasheet MFRC522 Contactless reader IC
 * Section: 8.1.2.3
 * The MSB of the first byte defines the mode used.
 * To read data from the MFRC522 the MSB (bit 7) is set to logic 1. 
 * To write data to the MFRC522 the MSB (bit 7) must be set to logic 0.
 */
//uint8_t DEV_RFID_RegByteRead(uint8_t reg, uint8_t *valPtr)
void DEV_RFID_RegByteRead(uint8_t reg, uint8_t *valPtr)
{
		/* default mode is write mode (reg[7]   = 0);
     *                 read  mode  reg[7]   = 1 
	   *                             reg[6:1] = reg addr
	   *                             reg[0]   = 0  
     */
	  reg |= ((1U<<7)&0xFF);                                // Set mode bit to read´mode in reg addr
	  PHP_SPI_SSI_RegByteRead(RFID_SSI_BLK, reg, valPtr);   // read data from given reg addr
	  
}


/* 
 * ---------------------------------- DEV_RFID_RegBurstRead -------------------------------------/
 * Pg.10,11 Datasheet MFRC522 Contactless reader IC
 * Section: 8.1.2.3
 * The MSB of the first byte defines the mode used.
 * To read data from the MFRC522 the MSB (bit 7) is set to logic 1. 
 * To write data to the MFRC522 the MSB (bit 7) must be set to logic 0.
 *
 * This function repeatedly reads a given address a specified number of times to obtain the value
 * stored in the address. The number of reads corresponds to the bytecount or array size where the
 * the value stored in the address in stored into as return data.
 */
void DEV_RFID_RegBurstRead(uint8_t reg, uint8_t byteCount, uint8_t *byteDataRx)
{
		/* default mode is write mode (reg[7]   = 0);
     *                 read  mode  reg[7]   = 1 
	   *                             reg[6:1] = reg addr
	   *                             reg[0]   = 0  
     */
	  reg |= ((1U<<7)&0xFF);                                      // Set mode bit to read´mode in reg addr
	  PHP_SPI_SSI_RegBurstRead(RFID_SSI_BLK, reg, byteCount, byteDataRx);
}

void DEV_RFID_FifoRegBurstRead(uint8_t reg, uint8_t byteCount, uint8_t *byteDataRx)
{
		/* default mode is write mode (reg[7]   = 0);
     *                 read  mode  reg[7]   = 1 
	   *                             reg[6:1] = reg addr
	   *                             reg[0]   = 0  
     */
	  reg |= ((1U<<7)&0xFF);                                      // Set mode bit to read´mode in reg addr
	  
	  while (byteCount > 0){                                      // read FIFO for bytecount
	      PHP_SPI_SSI_RegByteRead(RFID_SSI_BLK, reg, byteDataRx); // read FIFO register
			  byteCount--;																						// update bytecount to be read
			  byteDataRx++;																						// update data addr for next storage
		}
}

void DEV_RFID_RegSetBitMask(uint8_t reg, uint8_t bitMask){
	
    //PHP_SPI_SSI_RegSetBitMask(RFID_SSI_BLK, reg, bitMask);
    uint8_t value;
    
    DEV_RFID_RegByteRead(reg, &value);  // Read data from register
	  value |= bitMask;                   // Modify data by setting given bits
		DEV_RFID_RegByteWrite(reg, value);  // Write Modified data back to reg

}



void DEV_RFID_RegClearBitMask(uint8_t reg, uint8_t bitMask){
	
    uint8_t value;
    
    DEV_RFID_RegByteRead(reg, &value);  // Read data from register
	  value &= ~bitMask;                  // Modify data by setting given bits
		DEV_RFID_RegByteWrite(reg, value);  // Write Modified data back to reg

}



void DEV_RFID_SwitchOnAntenna(void)
{ 
    // Set the antenna bits in the TxControl Reg active
    DEV_RFID_RegSetBitMask(TxControlReg,
	                         (RFID_TXCTL_R_TX1RFEN | RFID_TXCTL_R_TX2RFEN )&0xFF);
}







/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum RFID_StatusCode DEV_RFID_PCD_CalculateCRC(uint8_t *data,		// In: Pointer to the data to transfer to the FIFO for CRC calculation.
                                               uint8_t length,	// In: The number of bytes to transfer.
                                               uint8_t *result	// Out: Pointer to result buffer.
                                                                // Result is written to result[0..1], low byte first.
					                                    )
{ 
	uint8_t regValue;
	uint16_t timeOutCounter = 5000;
	enum RFID_StatusCode RFID_SRC;                                  // For status return code (Error return check)
	
	DEV_RFID_RegByteWrite(CommandReg, RFID_PCD_CMD_Idle);		        // Stop any active command.
	DEV_RFID_RegByteWrite(DivIrqReg, RFID_PCD_DivIrqReg_B_CRCIRq);	// Clear the CRCIRq interrupt request bit
	DEV_RFID_RegSetBitMask(FIFOLevelReg, RFID_PCD_FIFOLevelReg_B_FlushBuffer); // FlushBuffer = 1, FIFO initialization
	DEV_RFID_FifoRegBurstWrite(FIFODataReg, length, data);	                   // Write data to the FIFO
	DEV_RFID_RegByteWrite(CommandReg, RFID_PCD_CMD_CalcCRC);		               // Start the calculation
	
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73?s.
	while (1) {
		DEV_RFID_RegByteRead(DivIrqReg, &regValue);	    // DivIrqReg[7..0] bits are:   
                                                    // [Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved]
		if (regValue & RFID_PCD_DivIrqReg_B_CRCIRq) {		// Wait until CRCIRq bit set - calculation done
			break;
		}
		if (--timeOutCounter == 0)                      // The emergency break. We will eventually terminate on this one after 89ms.
		{						                                    // Communication with the MFRC522 might be down.
			RFID_SRC = STATUS_TIMEOUT;
			return RFID_SRC;
		}
	}
	DEV_RFID_RegByteWrite(CommandReg, RFID_PCD_CMD_Idle);		// Stop calculating CRC for new content in the FIFO.
	
	// Transfer the result from the registers to the result buffer
	//result[0] = DEV_RFID_RegByteRead(CRCResultRegL);
	DEV_RFID_RegByteRead(CRCResultRegL, &result[0]);
	
	//result[1] = DEV_RFID_RegByteRead(CRCResultRegH);
	DEV_RFID_RegByteRead(CRCResultRegH, &result[1]);
	RFID_SRC = STATUS_OK;
	return RFID_SRC;
} 




/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if recvData and recvLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
int RFID_PCDCommunicateWithPICC(	uint8_t command,		// The command to execute. One of the PCD_Command enums.
																	uint8_t waitIRq,		// The bits in the ComIrqReg register that signals successful completion of the command.
																	uint8_t *sendData,  // Pointer to the data to transfer to the FIFO.
																	uint8_t sendLen,		// Number of bytes to transfer to the FIFO.
																	uint8_t *recvData,  // NULL or pointer to buffer if data should be read back after executing the command.
																	uint8_t *recvLen,		// In: Max number of bytes to write to *backData. Out: The number of bytes returned.
																  uint8_t *validBits,	// In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
																	uint8_t rxAlign,		// In: Defines the bit position in recvData[0] for the first bit received. Default 0.
																	uint8_t checkCRC)		// In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
{
	uint8_t regValue, ErrorRegValue, FIFOLevelRegValue, validBitsRx;
	uint16_t timeOutCounter = 2000;
	uint8_t controlBuffer[2];        // For CRC calculation
	enum RFID_StatusCode RFID_SRC;   // For status return code (Error return check)
	unsigned int i;
	
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		            // Pg.45 RxAlign = BitFramingReg[6..4].
                                                                  //       => (0): store LSB in position 0
	                                                                // Pg.45 TxLastBits = BitFramingReg[2..0] 
	                                                                //       => (0) : all bits in last byte will be transmitted
	
	DEV_RFID_RegByteWrite(CommandReg, RFID_PCD_CMD_Idle);			      // Pg.37 PCD Stop any active command.
	DEV_RFID_RegByteWrite(ComIrqReg,  RFID_PCD_R_ClearIntRqBits);   // Pg.38 Clear all seven interrupt request bits
	DEV_RFID_RegSetBitMask(FIFOLevelReg, RFID_PCD_R_FlushFIFO);			// Pg.43 FlushBuffer = 1, FIFO initialization bit[7]
	//DEV_RFID_RegBurstWrite(FIFODataReg, sendLen, sendData);	        // Pg.43 Write sendData to the FIFO (FIFO Capacity: 64 bytes)
	DEV_RFID_FifoRegBurstWrite(FIFODataReg, sendLen, sendData);	        // Pg.43 Write sendData to the FIFO (FIFO Capacity: 64 bytes)
	DEV_RFID_RegByteWrite(BitFramingReg, bitFraming);		            // Pg.45 Bit adjustments
	DEV_RFID_RegByteWrite(CommandReg, command);				              // Execute the command
	
	if (command == RFID_PCD_CMD_Transceive) {
		DEV_RFID_RegSetBitMask(BitFramingReg, RFID_PCD_R_StartSend);	// Pg.45 StartSend=1, transmission of data starts bit[7]
	}
	
	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. 
	// This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86?s.
	while (1) {
		DEV_RFID_RegByteRead(ComIrqReg, &regValue); // ComIrqReg[7..0] bits are: 
		                                            // Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		
		if (regValue & waitIRq) {					          // One of the interrupts that signal success has been set.
			break;
		}
		if (regValue & 0x01) {						          // Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
		if (--timeOutCounter == 0) {				        // The emergency break. If all other conditions fail we will 
			                                          // eventually terminate on this one after 35.7ms.
                                                // Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	
	// Stop now if any errors except collisions were detected.
	DEV_RFID_RegByteRead(ErrorReg, &ErrorRegValue);
	// Pg.40 Error Register bits are as written below bit[7:0]
	// [WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr]
	if (ErrorRegValue & (RFID_PCD_ErrorReg_B_BuffOvfl    | // BufferOvfl 
		                   RFID_PCD_ErrorReg_B_ParityErr   | // ParityErr 
	                     RFID_PCD_ErrorReg_B_ProtocolErr   // ProtocolErr
	              ))
	{	 
		return STATUS_ERROR;
	}	


	// If the caller wants data back, get it from the MFRC522.
	if (recvData && recvLen) {
		DEV_RFID_RegByteRead(FIFOLevelReg, &FIFOLevelRegValue);  // Number of bytes in the FIFO
		
		if (FIFOLevelRegValue > *recvLen) {
			return STATUS_NO_ROOM;
		}
		*recvLen = FIFOLevelRegValue;							               // Number of bytes returned
		//DEV_RFID_RegBurstRead(FIFODataReg, *recvLen, recvData);  // Get received data from FIFO
		DEV_RFID_FifoRegBurstRead(FIFODataReg, *recvLen, recvData);  // Get received data from FIFO
		DEV_RFID_RegByteRead(ControlReg, &regValue);             // Pg.44 return no of valid bits in last received byte 
    validBitsRx = regValue	& 0x07;               			     // Pg.44 RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = validBitsRx;
		}
	}
	
	// Check for collisions
	if (ErrorRegValue & RFID_PCD_ErrorReg_B_CollErr) {		// CollErr
			RFID_SRC = STATUS_COLLISION;
			return RFID_SRC;
	}
	
	// Perform CRC_A validation if requested.
	if (recvData && recvLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*recvLen == 1 && validBitsRx == 4) {
			RFID_SRC = STATUS_MIFARE_NACK;
			return RFID_SRC;
		}
		
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*recvLen < 2 || validBitsRx != 0) {
			RFID_SRC = STATUS_CRC_WRONG;
			return RFID_SRC;
		}
		
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		RFID_SRC = DEV_RFID_PCD_CalculateCRC(&recvData[0], (*recvLen - 2), &controlBuffer[0]);
		if (RFID_SRC != STATUS_OK) {
			return RFID_SRC;
		}
		// Compare received data with CRC computation result
		if ((recvData[*recvLen - 2] != controlBuffer[0]) || 
			  (recvData[*recvLen - 1] != controlBuffer[1])) 
		{
			RFID_SRC = STATUS_CRC_WRONG;
			return RFID_SRC;
		}
	}
	
	RFID_SRC = STATUS_OK;
	return RFID_SRC;
} // End PCD_CommunicateWithPICC()



/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum RFID_StatusCode DEV_RFID_PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
													uint8_t sendLen,		  // Number of bytes to transfer to the FIFO.
													uint8_t *backData,		// NULL or pointer to buffer if data should be read back after executing the command.
													uint8_t *backLen,		  // In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													uint8_t *validBits,	  // In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
													uint8_t rxAlign,		  // In: Defines the bit position in backData[0] for the first bit received. Default 0.
													uint8_t checkCRC		  // In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = (RFID_PCD_ComIrqReg_B_IdleIRq | RFID_PCD_ComIrqReg_B_TxIRq);		// pg.39 RxIRq and IdleIRq
	return RFID_PCDCommunicateWithPICC(RFID_PCD_CMD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()




/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
enum RFID_StatusCode DEV_RFID_PICC_REQA_or_WUPA( uint8_t command, 		// The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
												                         uint8_t *bufferATQA,	// The buffer to store the ATQA (Answer to request) in
												                         uint8_t *bufferSize)	// Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
{
	uint8_t validBits;
	enum RFID_StatusCode RFID_SRC;        // For status return code (Error return check) 
	
	if (!bufferATQA || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	DEV_RFID_RegClearBitMask(CollReg, RFID_PCD_CollReg_B_ValuesAfterColl);	// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									            // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	RFID_SRC = DEV_RFID_PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0, 0);
	if (RFID_SRC != STATUS_OK) {
		return RFID_SRC;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()



/**------------------------------------ DEV_RFID_PICC_RequestA ------------------------------------------/
 * Transmits a REQuest command, Type A. 
 * Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time it often get STATUS_TIMEOUT
 * - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum RFID_StatusCode DEV_RFID_PICC_RequestA(uint8_t *bufferATQA,	// The buffer to store the ATQA (Answer to request) in
											                      uint8_t *bufferSize)  // Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
{
	return DEV_RFID_PICC_REQA_or_WUPA(RFID_PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()




/**------------------------------------ DEV_RFID_PICC_WakeupA ------------------------------------------/
 * Transmits a Wake-UP command, Type A. 
 * Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection.
 * - (7 bit frame).
 *
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT 
 * - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum RFID_StatusCode DEV_RFID_PICC_WakeupA(	uint8_t *bufferATQA,	// The buffer to store the ATQA (Answer to request) in
											                      uint8_t *bufferSize)	// Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
{
	return DEV_RFID_PICC_REQA_or_WUPA(RFID_PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()




/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum RFID_StatusCode DEV_RFID_PICC_Select( devRfidPiccUid_type *devRfidPiccUid,		// Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											                     devRfidPiccUidIdSelState_type *devRfidPiccUidIdSelState,
                                           uint8_t validBits)	                      // The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
{
	uint8_t loopCount=0;
	enum RFID_StatusCode RFID_SRC;  // For status return code (Error return check)
	uint8_t uidIndex;					      // The first index in uid->uidByte[] that is used in the current Cascade Level.
	uint8_t currentLevelKnownBits;	// The number of known UID bits in the current Cascade Level.
	uint8_t txBuffer[9] = {0};				// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t txByteCount = 2;				      // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign=0;					      // Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits=0;				      // Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	uint8_t rxBuffer[8] = {0};
	uint8_t rxByteCount = 8;
	uint8_t collRegVal;

	
	// Description of send buffer (txBuffer) structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						    2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						    2			CT		uid3	uid4	uid5
	//						    3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80){return STATUS_INVALID;}
	
	
	// Prepare MFRC522
	DEV_RFID_RegClearBitMask(CollReg, RFID_PCD_CollReg_B_ValuesAfterColl);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	DEV_RFID_RegByteRead(CollReg, &collRegVal);
	while (!devRfidPiccUidIdSelState->identDone) {
	    // Repeat Cascade Level loop until we have a complete UID.
		  /* 01. Assign SEL -> Select cascade level */
		  RFID_SRC = DEV_RFID_PCD_PICC_SelectCascadeLevel(txBuffer,
                                                      devRfidPiccUidIdSelState); // index:0
		  
		  /* 02. Assign NVB -> Number of Valid Bits */
		  txBuffer[1] = 0x20; // NVB - Number of Valid Bits: Seven whole bytes
      
		  // Pg.45 Set bit adjustments 
			rxAlign = txLastBits; // Having a separate variable is overkill. But it makes the next line easier to read.
			DEV_RFID_RegByteWrite(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			RFID_SRC = DEV_RFID_PCD_TransceiveData(txBuffer, txByteCount, rxBuffer, &rxByteCount, &txLastBits, rxAlign, 0);
		
		  // Copy received data (5bytes) to RFID_PICC_UID->uidbyte 
		  RFID_SRC = DEV_RFID_IsUidComplete(devRfidPiccUid,
		                                    rxBuffer, 
		                                    &rxByteCount, 
		                                    devRfidPiccUidIdSelState);
														 
		  // Select PICC
			RFID_SRC = DEV_RFID_SendSelect(txBuffer, rxBuffer, &rxByteCount);			
			if (RFID_SRC != STATUS_OK) return RFID_SRC;
			
			// Check SAK
			RFID_SRC = DEV_RFID_CheckSak(rxBuffer, &rxByteCount, devRfidPiccUidIdSelState);
		  /*
		  buffer[1]=0x70;
		  buffer[2]=responseBuffer[0];
		  bufferUsed = 3;
		  RFID_SRC = DEV_RFID_PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0);
		  DEV_RFID_RegByteRead(CollReg, &collRegVal);
			*/
//			uidComplete=1;
	}
	return RFID_SRC;
}


/*

						// How many UID bits are known in this Cascade Level?
						currentLevelKnownBits = validBits - (8 * uidIndex);
						if (currentLevelKnownBits < 0) {currentLevelKnownBits = 0;}
						// Copy the known bits from uid->uidByte[] to buffer[]
						index = 2; // destination index in buffer[]
						if (useCascadeTag) {
							buffer[index++] = RFID_PICC_CMD_CT;
						}
						byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
						if (bytesToCopy) {
							byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
							if (bytesToCopy > maxBytes) {
								bytesToCopy = maxBytes;
							}
							for (count = 0; count < bytesToCopy; count++) {
								buffer[index++] = uid->uidByte[uidIndex + count];
							}
						}
						// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
						if (useCascadeTag) {
							currentLevelKnownBits += 8;
						}
						
						// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
						selectDone = false;
						while (!selectDone) {
							// Find out how many bits and bytes to send and receive.
							if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
								//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
								buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
								// Calculate BCC - Block Check Character
								buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
								// Calculate CRC_A
								RFID_SRC = DEV_RFID_PCD_CalculateCRC(buffer, 7, &buffer[7]);
								if (RFID_SRC != STATUS_OK) {
									return RFID_SRC;
								}
								txLastBits		= 0; // 0 => All 8 bits are valid.
								bufferUsed		= 9;
								// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
								responseBuffer	= &buffer[6];
								responseLength	= 3;
							}
							else { // This is an ANTICOLLISION.
								//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
								txLastBits		= currentLevelKnownBits % 8;
								count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
								index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
								buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
								bufferUsed		= index + (txLastBits ? 1 : 0);
								// Store response in the unused part of buffer
								responseBuffer	= &buffer[index];
								responseLength	= sizeof(buffer) - index;
							}
							
							// Set bit adjustments
							rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
							PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
							
							// Transmit the buffer and receive the response.
							result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
							if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
								byte valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
								if (valueOfCollReg & 0x20) { // CollPosNotValid
									return STATUS_COLLISION; // Without a valid collision position we cannot continue
								}
								byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
								if (collisionPos == 0) {
									collisionPos = 32;
								}
								if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
									return STATUS_INTERNAL_ERROR;
								}
								// Choose the PICC with the bit set.
								currentLevelKnownBits = collisionPos;
								count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
								index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
								buffer[index]	|= (1 << count);
							}
							else if (result != STATUS_OK) {
								return result;
							}
							else { // STATUS_OK
								if (currentLevelKnownBits >= 32) { // This was a SELECT.
									selectDone = true; // No more anticollision 
									// We continue below outside the while.
								}
								else { // This was an ANTICOLLISION.
									// We now have all 32 bits of the UID in this Cascade Level
									currentLevelKnownBits = 32;
									// Run loop again to do the SELECT.
								}
							}
						} // End of while (!selectDone)
						
						// We do not check the CBB - it was constructed by us above.
						
						// Copy the found UID bytes from buffer[] to uid->uidByte[]
						index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
						bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
						for (count = 0; count < bytesToCopy; count++) {
							uid->uidByte[uidIndex + count] = buffer[index++];
						}
						
						// Check response SAK (Select Acknowledge)
						if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
							return STATUS_ERROR;
						}
						// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
						RFID_SRC = DEV_RFID_PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
						if (RFID_SRC != STATUS_OK) {
							return RFID_SRC;
						}
						if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
							return STATUS_CRC_WRONG;
						}
						if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
							cascadeLevel++;
						}
						else {
							uidComplete = true;
							uid->sak = responseBuffer[0];
						}
					} // End of while (!uidComplete)
					
					// Set correct uid->size
					uid->size = 3 * cascadeLevel + 1;
					
					return STATUS_OK;
				} // End PICC_Select()
*/

enum RFID_StatusCode DEV_RFID_PCD_PICC_SelectCascadeLevel(uint8_t *buffer,
	                                                        devRfidPiccUidIdSelState_type *rfidPiccUidSelState)
{
	  // Repeat Cascade Level loop until we have a complete UID.
		// Set the Cascade Level in the SEL byte, 
		switch (rfidPiccUidSelState->cascadeLevel) {
			case 1:
				buffer[0] = RFID_PICC_CMD_SEL_CL1;
				break;
			
			case 2:
				buffer[0] = RFID_PICC_CMD_SEL_CL2;
				break;
			
			case 3:
				buffer[0] = RFID_PICC_CMD_SEL_CL3;
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				
		}		
		return STATUS_OK;
 
}



void DEV_RFID_PiccUidInit(devRfidPiccUid_type *devRfidPiccUid){
     uint8_t byteSize = 15;
	   devRfidPiccUid->size = 0;				// Initialise size to zero
	   devRfidPiccUid->sak  = 0;        // Initialise sak to zero
	
	   // Initialise the uid byte arrays to all zeros
	   for (int i=0; i<byteSize; i++){
		     devRfidPiccUid->uidByte[i] = 0;
		 }

}


enum RFID_StatusCode DEV_RFID_IsUidComplete(devRfidPiccUid_type *rfidPiccUid,
	                                          uint8_t * rxBuffer,
                                            uint8_t * rxByteCount,
                                            devRfidPiccUidIdSelState_type *rfidPiccUidSelState)
{     
	  uint8_t BCC;
	  uint8_t uidIdx=0; // Initial value is 0

	  // Copy data from recv buffer to UID data structure
    MISC_MEM_Copy(&(rfidPiccUid->uidByte[uidIdx]),                   // data destination addr
                 	rxBuffer,                                          // data source addr
	                *rxByteCount);                                     // no of byte to copy
	
	  rfidPiccUidSelState->uidIndex += (*rxByteCount);                 // update the uidIndex
	  uidIdx = rfidPiccUidSelState->uidIndex;                          // update the uidIdx local var
	
		BCC = rxBuffer[0]^ rxBuffer[1]^ rxBuffer[2]^ rxBuffer[3];        // Exclusive OR of first 4bytes received
	  if (rxBuffer[4] != BCC) return STATUS_ERROR;                     // Index 4 of recv buffer always hold bcc from PICC
			
    switch (uidIdx){
		    case 5:
             if (rfidPiccUid->uidByte[0] != RFID_PICC_CMD_CT){ 	     // 4 Byte UID Required CL1 -> [uid0-3, bcc]
                 rfidPiccUidSelState->identDone=1;
                 rfidPiccUid->size = 4;							 
                 break;
							 
						 }else if (rfidPiccUid->uidByte[0] == RFID_PICC_CMD_CT){ // 7 Byte UID Required CL1 -> [CT,uid0-2, bcc]
						     rfidPiccUidSelState->cascadeLevel++;                // repeat uid protocol with next cascade level
				         return STATUS_UID_INCOMPLETE;
						 
			       }
		    case 10:
				    if (rfidPiccUid->uidByte[5] != RFID_PICC_CMD_CT){        // 7 Byte UID Required CL2 -> [uid3-6, bcc]
								rfidPiccUidSelState->identDone=1;
							  rfidPiccUid->size = 7;
								break;
									
						}else if (rfidPiccUid->uidByte[5] == RFID_PICC_CMD_CT){ // 10 Byte UID Required CL2 -> [CT,uid3-5, bcc]
						    rfidPiccUidSelState->cascadeLevel++;                // repeat uid protocol with next cascade level
							  return STATUS_UID_INCOMPLETE;
						
						}
				case 15:
					  if (rfidPiccUid->uidByte[10] != RFID_PICC_CMD_CT){      // 10 Byte UID Required CL2 -> [uid6-9, bcc]
							rfidPiccUidSelState->identDone=1;
							rfidPiccUid->size = 10;
							break;
				}
				
				default:
				    return STATUS_INTERNAL_ERROR;
				    
		
		
		}
    return STATUS_OK;
}


enum RFID_StatusCode DEV_RFID_SendSelect(uint8_t *txBuffer,
                                         uint8_t *rxBuffer,
                                         uint8_t *rxByteCount){
    
		enum RFID_StatusCode	RFID_SRC;	
    uint8_t txByteCount = 	(2+(*rxByteCount));                  // No of bytes to transmit {CL, 70, (5bytes from recv buffer)}																				 
																					                       //txBuffer[0], already updated based on current CL
    txBuffer[1] = 0x70;                                          // code for sending information to the PICC
    
		// Move data from recv buffer to transmit buffer for select transmit
    MISC_MEM_Move(&(txBuffer[2]),                           // data destination addr
                 	rxBuffer,                                 // data source addr
	                *rxByteCount);                            // no of byte to copy
		
    // compute CRC
    RFID_SRC = DEV_RFID_PCD_CalculateCRC(txBuffer,		      // In: Pointer to the data to transfer to the FIFO for CRC calculation.
                                         txByteCount,    	  // In: The number of bytes to transfer.
                                         rxBuffer	          // Out: Pointer to result buffer.
                                        );                  // Result is written to result[0..1], low byte first.

    // Move the CRC in the recv buffer to the Transmit buffer for select transmission to PICC																					 
		if (RFID_SRC == STATUS_OK){                             // Check if CRC Calc was ok
			MISC_MEM_Move(&txBuffer[txByteCount],
			              rxBuffer,
			              2);                                     // CRC result is always 2 bytes
			txByteCount += 2;                                     // Transmit bytes now includes crc (2bytes) 
		
		}
    RFID_SRC = DEV_RFID_PCD_TransceiveData(txBuffer,        // Updated buffer for select transmission to PICC
																					 txByteCount,
																					 rxBuffer,        // buffer array to recv bytes from PICC
																					 rxByteCount,     // ptr to recv byte count
																					 0, 0, 1);				// validBits = 0; rxAlign=0; crc=1															
   return RFID_SRC;
}



enum RFID_StatusCode DEV_RFID_CheckSak(uint8_t *rxBuffer,
                                       uint8_t *rxByteCount,
                                       devRfidPiccUidIdSelState_type *devRfidPiccUidIdSelState)
{
	  enum RFID_StatusCode	RFID_SRC;
    uint8_t ByteCount = *rxByteCount;
    uint8_t sakByte;		
    
    if (ByteCount < 3){            // The select command always return 3bytes (sak, crc1, crc2)
		    return STATUS_SAK_WRONG;     // The select command response is not complete
		}
		
    *(rxByteCount) = 5;              // Reset byteCount to default value from PICC 
		sakByte = rxBuffer[0];           // Fetch sak response from first byte received
		

		if (sakByte & ((1U<<2)&0xFF)){ 
		    RFID_SRC = STATUS_UID_INCOMPLETE;
		
		}else{ // UID is completed
			   devRfidPiccUidIdSelState->stateActive=1;
			
			   if (sakByte & ((1U<<5)&0xFF)){           // check for ISO/IEC 14443-3 Compliance
			       RFID_SRC = STATUS_UID_COMPLAINT;
				 }else{
             RFID_SRC = STATUS_UID_NO_COMPLAINT;
         }	
		}				
		
		return RFID_SRC;
		
}

