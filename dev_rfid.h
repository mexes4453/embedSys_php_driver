/*
 * dev_rfid.h
 *
 *  Created on: 5 Dec 2021
 *      Author: chime
 *
 * Datasheets
 * ----------
 * DS1 https://www.mouser.com/datasheet/2/302/MF1S503x-89574.pdf
 * DS2 http://wg8.de/wg8n1496_17n3613_Ballot_FCD14443-3.pdf
 * DS3 Datatsheet MFRC522.pdf Contactless reader IC 112136
 * Tutorial -> https://www.youtube.com/watch?v=QDJMJ_INX3w
 *
 * TIVA TM4C123 Launchpad 
 * SPI-SSI Communcation data speed -> 10Mbit/s  DS1: Pg.10
 * RST/Reset   RST          PB3 - BLUE   - RESET   
 * SPI SS      SDA(SS)      PA3 - YELLOW - SLAVE SELECT  
 * SPI MOSI    MOSI         PA5 - GREEN  - MASTER OUT SLAVE IN 
 * SPI MISO    MISO         PA4 - ORANGE - MASTER IN SLAVE OUT 
 * SPI SCK     SCK          PA2 - WHITE  - CLOCK  
 * 
 * _____
 * ATQA
 * -----
 * Tag - ELEGOO - Blue  - 04 00 -> (00 04)
 * Tag - ELEGOO - White - 04 00 -> (00 04)
 * Tag - CATEEN - Red   - 02 00 -> (00 02)
 * Tag - IDCARD - White - 44 03 -> (03 44) 
 */

#ifndef DEV_RFID_H_
#define DEV_RFID_H_

#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "php.h"
#include "php_gpio.h"
#include "php_spi.h"
#include "php_uart.h"
#include "php_led.h"
#include "misc.h"


// SSI
#define RFID_SSI_BLK SSI0
#define RFID_SSI_CLK_SPD_INT        (1)
#define RFID_SSI_CLK_SPD_FRAC       (0)
#define RFID_SSI_CLK_SPD_FRAC_SIZE  (0)

// Other Pin configuration except for SSI
#define RFID_PORT GPIOB
#define RFID_PORT_CLK_EN_BIT  (1)

#define RFID_RST (1U<<3)         // PB3

#define RFID_TXCTL_R_TX1RFEN     (1U<<0)  // Pg.49 TxControlReg bit 0 Antenna control bit TX1
#define RFID_TXCTL_R_TX2RFEN     (1U<<1)  // Pg.49 TxControlReg bit 1 Antenna control bit TX2

// User defined PCD operations commands 
#define	RFID_PCD_R_ClearIntRqBits  (0x7F)   // Clear all interrupt request bits in ComIrqReg
#define	RFID_PCD_R_FlushFIFO       (0x80)   // Clear internal FIFO buffer read write pointer & Buffer overflow (OVFL) bit in ErrorReg
#define	RFID_PCD_R_StartSend       (0x80)   // Pg.45 Set StartSend bit in BitFrammingReg


// RFID PCD Register bit definition
// Pg.40 ErrorReg[7..0] bits are: 
// Pg.40 [WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr]
#define RFID_PCD_ErrorReg_B_BuffOvfl    (0x01<<4)
#define RFID_PCD_ErrorReg_B_CollErr     (0x01<<3)
#define RFID_PCD_ErrorReg_B_ParityErr   (0x01<<1)
#define RFID_PCD_ErrorReg_B_ProtocolErr (0x01)

// Pg.39 DivIrqReg[7..0] bits are: 
// Pg.39 [ Set2 reserved MfinActIRq reserved CRCIRq(2) reserved]
#define RFID_PCD_DivIrqReg_B_CRCIRq     (0x01<<2)
#define RFID_PCD_ComIrqReg_B_IdleIRq    (0x01<<4)
#define RFID_PCD_ComIrqReg_B_TxIRq      (0x01<<6)
#define RFID_PCD_CollReg_B_ValuesAfterColl  (0x01<<7)   // Pg.45 
#define RFID_PCD_CollReg_B_CollPosNotValid  (0x01<<5)
#define	RFID_PCD_FIFOLevelReg_B_FlushBuffer (0x01<<7)   // Pg.43 Clear internal FIFO buffer read write pointer & Buffer overflow (OVFL) bit in ErrorReg

// Authentication constants
#define MF_KEY_SIZE (6)				 // Key A and Key B are both 6 bytes stored in the sector trailer
#define MF_AUTH_BUF_SIZE (12)	 // Pg.71 DS3 : 12bytes of data to be transmitted to FIFO for AUTH
#define MF_BLK_DATA_SIZE  (18) // Pg.9/10 DS1 : Each memory block contains 16bytes 
															 // Pg.9/10 DS1 : Each memory block contains 16bytes 
/* Pg.11 Datasheet MFRC522 Contactless reader IC
 * Section: 8.1.2.3
 * The MSB of the first byte defines the mode used. To read data from the MFRC522 the
 * MSB is set to logic 1. To write data to the MFRC522 the MSB must be set to logic 0. Bits 6
 * to 1 define the address and the LSB is set to logic 0.
 * Therefore all register addresses are (0xXX<<1) are shifted left by 1 bit
*/
enum RFID_Register {
	// Page 0: Command and status
	CommandReg			= 0x01 << 1,	// starts and stops command execution
	ComIEnReg				= 0x02 << 1,	// enable and disable interrupt request control bits
	DivIEnReg				= 0x03 << 1,	// enable and disable interrupt request control bits
	ComIrqReg				= 0x04 << 1,	// interrupt request bits
	DivIrqReg				= 0x05 << 1,	// interrupt request bits
	ErrorReg				= 0x06 << 1,	// error bits showing the error status of the last command executed 
	Status1Reg			= 0x07 << 1,	// communication status bits
	Status2Reg			= 0x08 << 1,	// receiver and transmitter status bits
	FIFODataReg			= 0x09 << 1,	// input and output of 64 byte FIFO buffer
	FIFOLevelReg		= 0x0A << 1,	// number of bytes stored in the FIFO buffer
	WaterLevelReg		= 0x0B << 1,	// level for FIFO underflow and overflow warning
	ControlReg			= 0x0C << 1,	// miscellaneous control registers
	BitFramingReg		= 0x0D << 1,	// adjustments for bit-oriented frames
	CollReg					= 0x0E << 1,	// bit position of the first bit-collision detected on the RF interface

	
	// Page 1: Command
	ModeReg					= 0x11 << 1,	// defines general modes for transmitting and receiving 
	TxModeReg				= 0x12 << 1,	// defines transmission data rate and framing
	RxModeReg				= 0x13 << 1,	// defines reception data rate and framing
	TxControlReg		= 0x14 << 1,	// controls the logical behavior of the antenna driver pins TX1 and TX2
	TxASKReg				= 0x15 << 1,	// controls the setting of the transmission modulation
	TxSelReg				= 0x16 << 1,	// selects the internal sources for the antenna driver
	RxSelReg				= 0x17 << 1,	// selects internal receiver settings
	RxThresholdReg	= 0x18 << 1,	// selects thresholds for the bit decoder
	DemodReg				= 0x19 << 1,	// defines demodulator settings
	MfTxReg					= 0x1C << 1,	// controls some MIFARE communication transmit parameters
	MfRxReg					= 0x1D << 1,	// controls some MIFARE communication receive parameters
	SerialSpeedReg	= 0x1F << 1,	// selects the speed of the serial UART interface
	
	// Page 2: Configuration
	CRCResultRegH			  = 0x21 << 1,	// shows the MSB and LSB values of the CRC calculation
	CRCResultRegL			  = 0x22 << 1,
	ModWidthReg				  = 0x24 << 1,	// controls the ModWidth setting?
	RFCfgReg			      = 0x26 << 1,	// configures the receiver gain
	GsNReg				    	= 0x27 << 1,	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
	CWGsPReg			    	= 0x28 << 1,	// defines the conductance of the p-driver output during periods of no modulation
	ModGsPReg			    	= 0x29 << 1,	// defines the conductance of the p-driver output during periods of modulation
	TModeReg				    = 0x2A << 1,	// defines settings for the internal timer
	TPrescalerReg			  = 0x2B << 1,	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
	TReloadRegH				  = 0x2C << 1,	// defines the 16-bit timer reload value
	TReloadRegL				  = 0x2D << 1,
	TCounterValueRegH		= 0x2E << 1,	// shows the 16-bit timer value
	TCounterValueRegL		= 0x2F << 1,
	
	// Page 3: Test Registers

	TestSel1Reg				= 0x31 << 1,	// general test signal configuration
	TestSel2Reg				= 0x32 << 1,	// general test signal configuration
	TestPinEnReg			= 0x33 << 1,	// enables pin output driver on pins D1 to D7
	TestPinValueReg		= 0x34 << 1,	// defines the values for D1 to D7 when it is used as an I/O bus
	TestBusReg				= 0x35 << 1,	// shows the status of the internal test bus
	AutoTestReg				= 0x36 << 1,	// controls the digital self test
	VersionReg				= 0x37 << 1,	// shows the software version
	AnalogTestReg			= 0x38 << 1,	// controls the pins AUX1 and AUX2
	TestDAC1Reg				= 0x39 << 1,	// defines the test value for TestDAC1
	TestDAC2Reg				= 0x3A << 1,	// defines the test value for TestDAC2
	TestADCReg				= 0x3B << 1		// shows the value of ADC I and Q channels

};

// MFRC522 (PCD) commands. Described in chapter 10 of the datasheet.
enum RFID_PCD_Command {
	RFID_PCD_CMD_Idle				      = 0x00,		// no action, cancels current command execution
	RFID_PCD_CMD_Mem				    	= 0x01,		// stores 25 bytes into the internal buffer
	RFID_PCD_CMD_GenerateRandomID	= 0x02,		// generates a 10-byte random ID number
	RFID_PCD_CMD_CalcCRC				  = 0x03,		// activates the CRC coprocessor or performs a self test
	RFID_PCD_CMD_Transmit		    	= 0x04,		// transmits data from the FIFO buffer
	RFID_PCD_CMD_NoCmdChange 			= 0x07,		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
	RFID_PCD_CMD_Receive		  		= 0x08,		// activates the receiver circuits
	RFID_PCD_CMD_Transceive 			= 0x0C,		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
	RFID_PCD_CMD_MFAuthent 		  	= 0x0E,		// performs the MIFARE standard authentication as a reader
	RFID_PCD_CMD_SoftReset		  	= 0x0F,		// resets the MFRC522
	
};


// Commands sent to the RFID_PICC.
enum RFID_PICC_Command {
	// The commands used by the PCD to manage communication with several RFID_PICCs (ISO 14443-3, Type A, section 6.4)
	RFID_PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites RFID_PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	RFID_PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites RFID_PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	RFID_PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
	RFID_PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
	RFID_PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
	RFID_PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
	RFID_PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE RFID_PICC to go to state HALT.

	// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
	// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
	// The read/write commands can also be used for MIFARE Ultralight.
	RFID_PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
	RFID_PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
	RFID_PICC_CMD_MF_READ	    	= 0x30,		// Reads one 16 byte block from the authenticated sector of the RFID_PICC. Also used for MIFARE Ultralight.
	RFID_PICC_CMD_MF_WRITE	  	= 0xA0,		// Writes one 16 byte block to the authenticated sector of the RFID_PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	RFID_PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
	RFID_PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
	RFID_PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
	RFID_PICC_CMD_MF_TRANSFER	  = 0xB0,		// Writes the contents of the internal data register to a block.
	
	// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	// The RFID_PICC_CMD_MF_READ and RFID_PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
	RFID_PICC_CMD_UL_WRITE	  	= 0xA2		// Writes one 4 byte page to the RFID_PICC.
};

enum RFID_StatusCode {
	STATUS_OK				,	      // Success
	STATUS_ERROR			,	    // Error in communication
	STATUS_COLLISION		,	  // Collission detected
	STATUS_TIMEOUT			,	  // Timeout in communication.
	STATUS_NO_ROOM			,	  // A buffer is not big enough.
	STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
	STATUS_INVALID			,	  // Invalid argument.
	STATUS_CRC_WRONG		,	  // The CRC_A does not match
	STATUS_UID_INCOMPLETE,  // The UID is incomplete
	STATUS_UID_COMPLAINT,   // The UID is complete and ISO/IEC 14443-3 Compliance
	STATUS_UID_NO_COMPLAINT,// The UID is complete and NOT ISO/IEC 14443-3 Compliance
	STATUS_SAK_WRONG,       // The Select command response is not complete
	STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
};


	// A struct used for passing the UID of a PICC.
	typedef struct {
		uint8_t		size;			     // Number of bytes in the UID. 4, 7 or 10.
		uint8_t		uidByte[15];   // // Buffer to store PICC card UID bytes
		uint8_t		sak;			     // The SAK (Select acknowledge) byte returned from the PICC after successful selection.
		uint8_t 	blkAddr;			 // Current block address for data memory access operation
		uint8_t		dataBuf[18];   // Buffer to store data byte (16) + crc byte (2)
		uint8_t   dataBufSz;		 // Should not be more than 18	
	} devRfidPiccUid_type;


// A struct for passing the "identification" & "select" phase
	typedef struct {
	    uint8_t cascadeLevel;
		  uint8_t uidIndex;
		  uint8_t identDone;
		  uint8_t selectDone;
		  uint8_t stateActive;
	} devRfidPiccUidIdSelState_type;


// Function Declarations
void DEV_RFID_Init(void);
void DEV_RFID_RegByteWrite(uint8_t reg, uint8_t data);
void DEV_RFID_RegBurstWrite(uint8_t reg, uint8_t byteCount, uint8_t *byteData);
void DEV_RFID_RegByteRead(uint8_t reg, uint8_t * valPtr);
void DEV_RFID_RegBurstRead(uint8_t reg, uint8_t byteCount, uint8_t *byteDataRx);
void DEV_RFID_RegSetBitMask(uint8_t reg, uint8_t bitMask);
void DEV_RFID_RegClearBitMask(uint8_t reg, uint8_t bitMask);
void DEV_RFID_FifoRegBurstRead(uint8_t reg, uint8_t byteCount, uint8_t *byteDataRx);
void DEV_RFID_FifoRegBurstWrite(uint8_t reg, uint8_t byteCount, uint8_t *byteDataTx);
void DEV_RFID_SwitchOnAntenna(void);
void DEV_RFID_PiccUidInit(devRfidPiccUid_type *);
	
	
/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum RFID_StatusCode DEV_RFID_PCD_CalculateCRC(uint8_t *data,		// In: Pointer to the data to transfer to the FIFO for CRC calculation.
                                               uint8_t length,	// In: The number of bytes to transfer.
                                               uint8_t *result	// Out: Pointer to result buffer.
                                                                // Result is written to result[0..1], low byte first.
);



enum RFID_StatusCode DEV_RFID_PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
													uint8_t sendLen,		  // Number of bytes to transfer to the FIFO.
													uint8_t *backData,		// NULL or pointer to buffer if data should be read back after executing the command.
													uint8_t *backLen,		  // In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													uint8_t *validBits,	  // In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
													uint8_t rxAlign,		  // In: Defines the bit position in backData[0] for the first bit received. Default 0.
													uint8_t checkCRC		  // In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 );




enum RFID_StatusCode DEV_RFID_PICC_RequestA(uint8_t *bufferATQA,	// The buffer to store the ATQA (Answer to request) in
											                      uint8_t *bufferSize);  // Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.

									 
		

enum RFID_StatusCode DEV_RFID_PICC_WakeupA(	uint8_t *bufferATQA,	// The buffer to store the ATQA (Answer to request) in
											                      uint8_t *bufferSize);	// Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.

enum RFID_StatusCode DEV_RFID_PCD_PICC_SelectCascadeLevel(uint8_t *buffer,	                                                        
                                                          devRfidPiccUidIdSelState_type *);



/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if recvData and recvLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
enum RFID_StatusCode RFID_PCDCommunicateWithPICC(	uint8_t command,		// The command to execute. One of the PCD_Command enums.
																	uint8_t waitIRq,		// The bits in the ComIrqReg register that signals successful completion of the command.
																	uint8_t *sendData,  // Pointer to the data to transfer to the FIFO.
																	uint8_t sendLen,		// Number of bytes to transfer to the FIFO.
																	uint8_t *recvData,  // NULL or pointer to buffer if data should be read back after executing the command.
																	uint8_t *recvLen,		// In: Max number of bytes to write to *backData. Out: The number of bytes returned.
																  uint8_t *validBits,	// In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
																	uint8_t rxAlign,		// In: Defines the bit position in backData[0] for the first bit received. Default 0.
																	uint8_t checkCRC);	// In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.



enum RFID_StatusCode DEV_RFID_IsUidComplete(devRfidPiccUid_type *RFID_PICC_UID,
	                                          uint8_t * rxBuffer,
                                            uint8_t * rxByteCount,
                                            devRfidPiccUidIdSelState_type *DEV_RFID_PICC_UID_ID_SEL_STATE);




enum RFID_StatusCode DEV_RFID_PICC_Select(devRfidPiccUid_type *,		      // Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											                    devRfidPiccUidIdSelState_type *,
                                          uint8_t validBits);	


enum RFID_StatusCode DEV_RFID_SendSelect(uint8_t *txBuffer,
                                         uint8_t *rxBuffer,
                                         uint8_t *rxByteCount);

/**/
enum RFID_StatusCode DEV_RFID_CheckSak(uint8_t *rxBuffer,
                                       uint8_t *rxByteCount,
                                       devRfidPiccUidIdSelState_type *devRfidPiccUidIdSelState);

enum RFID_StatusCode DEV_RFID_ProcessUID(devRfidPiccUid_type *,
                                         devRfidPiccUidIdSelState_type *devRfidPiccUidIdSelState);


// This function authenticates a sector block for data operation 
// Operations include(read, write, increment, decrement, restore, transfer)
enum RFID_StatusCode DEV_RFID_PCD_AuthenticateSector(uint8_t cmd,
																										 uint8_t blockAddr,
																										 uint8_t *keyBytes,
																										 devRfidPiccUid_type *uid);
/*
 * This function reads the data store in a block memory after authenticating
 * input : uint8_t blockAddr        -> block address 0-63 (64 blocks for 1kb RFID card)
 *																		 Range varies depending on the card type					
 * input : devRfidPiccUid_type *uid -> Pointer to the uid struct
 *
 * @return RFID_StatusCode						-> Read operation status
 */
enum RFID_StatusCode DEV_RFID_PICC_Read(uint8_t blockAddr,
																				devRfidPiccUid_type *uid,
																				uint8_t *AuthKey);
// This function prints the header title for the memory 
void DEV_RFID_PrintSectorHeader(void);
enum RFID_StatusCode DEV_RFID_PrintBlockAddrData(enum RFID_StatusCode RFID_SRC,
																								 devRfidPiccUid_type *uid);


// This function indicates execution status with LED
void RFID_SRC_LEDStatus(enum RFID_StatusCode RFID_SRC);

#endif /* DEV_RFID_H_ */
