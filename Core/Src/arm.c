#include <stdint.h>
#include <stdio.h> // For printf, for debugging

// Assuming these are available from your JTAG bit-banging implementation
#include "jtag.h"

// extern void JTAG_Chain_Init(uint8_t num_devices);
// extern void JTAG_Chain_ConfigureDevice(uint8_t index, JTAG_DeviceType_TypeDef type, uint8_t default_dr_length);
// extern void JTAG_SetBypassState(uint8_t index, uint8_t bypassed); // Only if multi-device chain is used

// Also, ensure your delay function is available, e.g.:
// #define HAL_Delay(ms)  // Your actual delay implementation

// --- Internal Helper Function (Reads/Writes to DP or AP) ---

/**
 * @brief Performs a JTAG-DP or AHB-AP register access.
 * This is the fundamental function for interacting with the Cortex-M3.
 * It handles the JTAG instruction and data register shifts.
 *
 * @param dp_instruction The JTAG IR instruction for the DP (e.g., CM3_DP_SELECT, CM3_DP_CTRLSTAT).
 * @param data_out The 32-bit data to write. Ignored for reads.
 * @param data_in Pointer to store the 32-bit data read. Can be NULL for writes.
 * @param is_ap_access Flag: 1 if this is an AHB-AP access (requires DP_SELECT then DP_RDBUF/DRW logic), 0 for direct DP access.
 * @param ap_addr If is_ap_access is 1, this is the AHB-AP register address (e.g., CM3_AP_TAR_ADDR).
 * @return CM3_DBG_OK on success, error code otherwise.
 */


uint32_t DPACC (uint32_t data_out, uint32_t *data_in, uint32_t dp_reg ,uint32_t read)
{
	uint32_t retval = 0;
	uint64_t tempWriteVal = 0;
	uint64_t data_in_temp;

	tempWriteVal |= read & 0x1;

	tempWriteVal |= (dp_reg & 0x3) << 1;

	tempWriteVal |= (uint64_t)data_out << 3;

	// Direct DP access (e.g., CTRLSTAT, IDCODE, ABORT)
	JTAG_ShiftIRSPI(0b1010, CM3_JTAG_IR_LENGTH);

	JTAG_ShiftDR_SPI(tempWriteVal, 35, &data_in_temp);


	if(read)
	{

		//JTAG_ShiftDR(data_out, 35, &data_in_temp);
		JTAG_ShiftDR_SPI(data_out, 35, &data_in_temp);

		if( (data_in_temp & 0x7) == 0b010) // 0b010 = OK/FAULT
		{
			retval = 1;
			*data_in = ( (uint64_t)data_in_temp >> 3) & 0xFFFFFFFF;
		}
		else  // 0b001 = WAIT
		{
			retval = 0;
		}


	}

    return retval;
}

uint32_t APACC (uint32_t data_out, uint32_t *data_in, uint32_t dp_reg ,uint32_t read)
{
	uint32_t retval = 0;
	uint64_t tempWriteVal = 0;
	uint64_t data_in_temp;

	tempWriteVal |= read & 0x1;

	tempWriteVal |= (dp_reg & 0x3) << 1;

	tempWriteVal |= (uint64_t)data_out << 3;

	// Direct AP access (e.g., CTRLSTAT, IDCODE, ABORT)
	JTAG_ShiftIRSPI(0b1011, CM3_JTAG_IR_LENGTH);

	JTAG_ShiftDR_SPI(tempWriteVal, 35, &data_in_temp);


	if(read)
	{

		//JTAG_ShiftDR(data_out, 35, &data_in_temp);
		JTAG_ShiftDR_SPI(data_out, 35, &data_in_temp);

		if( (data_in_temp & 0x7) == 0b010) // 0b010 = OK/FAULT
		{
			retval = 1;
			*data_in = ( (uint64_t)data_in_temp >> 3) & 0xFFFFFFFF;
		}
		else  // 0b001 = WAIT
		{
			retval = 0;
		}


	}

    return retval;
}


#ifdef TRY_CM3_BLUEPILL

uint32_t idCode = JTAG_ReadIDCODE();


  //JTAG_ShiftIRSPI(0xabab, 13);
  //JTAG_ShiftIRSPI(0xdead, 14);
  //JTAG_ShiftIRSPI(0x72D1, 15);
  //JTAG_ShiftDR_SPI(0, 32, &idCode);
  //idCode = JTAG_ReadIDCODE();
  //JTAG_ShiftIRSPI(0xe6AD, 16);
  //JTAG_ShiftDR_SPI(0, 32, &idCode);
  //idCode = JTAG_ReadIDCODE();

  uint64_t denemeVal = 0;


  JTAG_ShiftIRSPI(0xf, 4);
  JTAG_ShiftDR_SPI( (uint64_t) 0xDeadbeef, (uint64_t) 33, &denemeVal);

  JTAG_ShiftIRSPI(0xe0a0dbad, 32);
  JTAG_ShiftDR_SPI(0, 32, &idCode);

  JTAG_ShiftIRSPI(0x70A0DBAD, 31);
  JTAG_ShiftDR_SPI(0, 32, &idCode);

  JTAG_ShiftIRSPI(0x38A0DBAD, 30);
  JTAG_ShiftDR_SPI(0, 32, &idCode);

  uint8_t irLen;
  irLen = JTAG_MeasureIRLength();



  uint32_t dpacc_reg;

  uint32_t apacc_reg;

  uint32_t writeVal;


  writeVal = (1 << 30) | (1 << 28) | (1 << 5);
  //writeVal = 0xFFffFFff;

  DPACC(writeVal, &dpacc_reg, 1, WRITE);

  DPACC(writeVal, &dpacc_reg, 1, READ);

  idCode = JTAG_ReadIDCODE();



  /* read normal data */
  writeVal = 0x00;

  DPACC(writeVal, &dpacc_reg, 2, WRITE);

  APACC(0x00000002, &apacc_reg, 0,READ);

  APACC( (0x2 | 1 << 29 | 1 << 25) , &apacc_reg, 0,WRITE);

  APACC(0x00000002, &apacc_reg, 0,READ);


  APACC(0xE000EDF0, &apacc_reg, 1,WRITE);

  APACC(DUMMY_WRITE_VAL, &apacc_reg, 1,READ);

  /* this command halts the core */
  APACC(0xA05F0003, &apacc_reg, 3,WRITE);


  APACC(0xAA55AA55, &apacc_reg, 3,READ);

  DPACC(writeVal, &dpacc_reg, 1, READ);

  APACC(0xDEADBEEF, &apacc_reg, 3,WRITE);

  APACC(0xAA55AA55, &apacc_reg, 3,READ);

  DPACC(writeVal, &dpacc_reg, 1, READ);

  /* read banked data */
  writeVal = 0x10;

  DPACC(writeVal, &dpacc_reg, 2, WRITE);

  APACC(0xAA55AA55, &apacc_reg, 0,READ);

  APACC(0xAA55AA55, &apacc_reg, 1,READ);

  APACC(0xAA55AA55, &apacc_reg, 2,READ);

  APACC(0xAA55AA55, &apacc_reg, 3,READ);


  /* read ID */

  writeVal = 0xf0;

  DPACC(writeVal, &dpacc_reg, 2, WRITE);

  APACC(0xAA55AA55, &apacc_reg, 3,READ);

  APACC(0xAA55AA55, &apacc_reg, 2,READ);









  idCode = JTAG_ReadIDCODE();





  //CM3_Debug_Init();

#endif

#ifdef TRY_BLUE_PILL


  uint64_t data_in;
  uint8_t irLen;
  uint32_t writeVal;
  uint64_t tempWriteVal = 0;

  writeVal = (1 << 30) | (1 << 28) | (1 << 5);


  /* enable connect */

  JTAG_ShiftIRSPI(0x7, 6);
  JTAG_ShiftDR_SPI(0x89, 8, &data_in);


  JTAG_ShiftIRSPI(0x2, 6);


  for(int i = 0; i < 16; i++)
  {
	  uint32_t val = (0xa0 + i);
	  uint32_t read_val;
	  val <<= 24;
	  val |= 0x120;
	  if(i == 11)
	  {
		  JTAG_ShiftDR_SPI(val, 32, &data_in);
	  }

	  val = (0x20 + i);
	  val <<= 24;
	  JTAG_ShiftDR_SPI(val, 32, &data_in);
	  JTAG_ShiftDR_SPI(val, 32, &data_in);
	  read_val = data_in;
  }



  JTAG_ShiftDR_SPI(0xFFFFFFFFFFFFFFFF, 64, &data_in);

  irLen = JTAG_MeasureIRLength();

  JTAG_ShiftIRSPI_BypassOthers(0b1010, 4,
  		6, 0, &data_in);


  JTAG_ShiftDR_SPI(0x280000002, 35, &data_in);
  JTAG_ShiftDR_SPI(0x280000003, 35, &data_in);


#endif
