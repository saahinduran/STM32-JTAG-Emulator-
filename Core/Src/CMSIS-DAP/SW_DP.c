/*
 * Copyright (c) 2013-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * $Date:        1. December 2017
 * $Revision:    V2.0.0
 *
 * Project:      CMSIS-DAP Source
 * Title:        SW_DP.c CMSIS-DAP SW DP I/O
 *
 *---------------------------------------------------------------------------*/

#include "DAP_config.h"
#include "DAP.h"
#include "helper.h"
#include "port.h"

/* these variables are for debug purposes */
uint32_t wait_ctr = 0;
uint32_t err_ctr = 0;
uint32_t success_ctr = 0;

// Generate SWJ Sequence
//   count:  sequence bit count
//   data:   pointer to sequence bit data
//   return: none
#if ((DAP_SWD != 0) || (DAP_JTAG != 0))
void SWJ_Sequence (uint32_t count, const uint8_t *data) {

  uint8_t xFerSizes[3];

  	SPI_SwitchPhaseToWrite();
  	calculate_xfer_sizes(count, xFerSizes);

  	uint32_t currentBit = 0;

  	/* Value is experimental. May differ from target(debugger) to target. */
  	uint32_t delay_cnt = 2500;

  	  		while(delay_cnt--)
  	  	    {
  	  		  __asm("nop");
  	  	    }

  	while(xFerSizes[IDX_8_BIT])
  	{
  		uint8_t tms_val = *data;
  		uint64_t tdo_val;

  		SPI_TMS_Transfer(tms_val, 8);
  		SPI_Transfer(&tdo_val, 0 , 8);

  		data++;
  		xFerSizes[IDX_8_BIT]--;
  		currentBit+= 8;
  	}

  	while(xFerSizes[IDX_RM1_BIT])
  	{
  		uint32_t delay_cnt = 2500;

  		while(delay_cnt--)
  	    {
  		  __asm("nop");
  	    }

  		uint16_t tms_val = extract_nbits_lsb(data, currentBit, xFerSizes[IDX_RM1_BIT]);
  		uint64_t tdo_val;

  		SPI_TMS_Transfer(tms_val, xFerSizes[IDX_RM1_BIT]);
  		SPI_Transfer(&tdo_val, 0 , xFerSizes[IDX_RM1_BIT]);


  		currentBit+= xFerSizes[IDX_RM1_BIT];

  		xFerSizes[IDX_RM1_BIT] = 0;

  	}

  	while(xFerSizes[IDX_RM2_BIT])
  	{
  		uint32_t delay_cnt = 2000;

  		while(delay_cnt--)
  		{
  		  __asm("nop");
  		}

  		uint16_t tms_val = extract_nbits_lsb(data, currentBit, xFerSizes[IDX_RM2_BIT]);
  		uint64_t tdo_val;

  		SPI_TMS_Transfer(tms_val, xFerSizes[IDX_RM2_BIT]);
  		SPI_Transfer(&tdo_val, 0 , xFerSizes[IDX_RM2_BIT]);


  		currentBit+= xFerSizes[IDX_RM2_BIT];

  		xFerSizes[IDX_RM2_BIT] = 0;

  	}

}
#endif


// Generate SWD Sequence
//   info:   sequence information
//   swdo:   pointer to SWDIO generated data
//   swdi:   pointer to SWDIO captured data
//   return: none
#if (DAP_SWD != 0)
void SWD_Sequence (uint32_t info, const uint8_t *swdo, uint8_t *swdi)
{
  uint32_t n;
  uint64_t dummyRead;

  n = info & SWD_SEQUENCE_CLK;
  if (n == 0U) {
    n = 64U;
  }

  if (info & SWD_SEQUENCE_DIN)
  {
	  SPI_SwitchPhaseToListen();

	  while(n > 8)
	  {
		  SPI_Transfer(*swdo, 0, 8);
		  SPI_TMSRead(swdi, 8);
		  swdo++;
		  swdi++;
		  n-= 8;
	  }

	  ChangeTMS_Size(n);

	  SPI_Transfer(&dummyRead, 0, n);
	  SPI_TMSRead(&dummyRead, n);
	  *swdi = dummyRead;

  }
  else
  {
	  SPI_SwitchPhaseToWrite();
	  if(n == 33)
	  {
		  uint16_t writeVal = 0;

		  writeVal = (*swdo);
		  swdo++;
		  writeVal |= (*swdo) << 8;
		  swdo++;
		  n-= 16;

		  SPI_TMS_Transfer(writeVal, 16);
		  SPI_Transfer(&dummyRead, 0, 16);
		  SPI_TMSRead(&dummyRead, 16);

		  writeVal = *swdo;

		  SPI_TMS_Transfer(writeVal, 8);
		  SPI_Transfer(&dummyRead, 0, 8);
		  SPI_TMSRead(&dummyRead, 8);

		  swdo++;

		  n-= 8;

		  writeVal = *swdo;
		  swdo++;

		  writeVal |=  ( (*swdo) & 0x1) << 8;

		  SPI_TMS_Transfer(writeVal, 9);
		  SPI_Transfer(&dummyRead, 0, 9);
		  SPI_TMSRead(&dummyRead, 9);

		  n-= 9;

	  }
	  while(n > 8)
	  {
		  SPI_TMS_Transfer(*swdo, 8);
		  SPI_Transfer(&dummyRead, 0, 8);
		  SPI_TMSRead(&dummyRead, 8);
		  swdo++;
		  n-= 8;
	  }
	  if(n == 1)
	  {
		  SPI_TMS_Transfer(*swdo, 4);
		  SPI_Transfer(&dummyRead, 0, 4);
		  SPI_TMSRead(&dummyRead, 4);
	  }
	  else if(n != 0)
	  {
		  SPI_TMS_Transfer(*swdo, n);
		  SPI_Transfer(&dummyRead, 0, n);
		  SPI_TMSRead(&dummyRead, n);
	  }



  }

}
#endif


#if (DAP_SWD != 0)
// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
uint8_t SWD_Transfer_LL(uint32_t request, uint32_t *data)
/* this function is the HEART of the SWD protocol*/
{
  uint32_t ack;
  uint8_t writeReq = 1;

  uint32_t parity;
  uint32_t bit;
  uint32_t parityIdx;

  uint64_t dummyRead;
  uint32_t delay_cnt = 2000;
  static int i = 0;

  i++;
                                                                                
  uint32_t n;
  uint32_t read_data = 0;

  uint32_t write_data = 0;
  write_nbits_lsb(&writeReq, 1, 4, request);

  parity = generate_even_parity(request & 0xf);

  write_nbits_lsb(&writeReq, 5, 1, parity);

  write_nbits_lsb(&writeReq, 7, 1, 1);

  SPI_SwitchPhaseToWrite();

  /* ISSUE the request phase which is 8 bits long */
  SPI_TMS_Transfer(writeReq, 8);
  SPI_Transfer(&dummyRead, 0, 8);


  if (request & DAP_TRANSFER_RnW)
  {
	  /* HANDLE acknowledge phase */
	  SPI_SwitchPhaseToListen();

	  n = DAP_Data.swd_conf.turnaround;

	  ChangeTMS_Size(4+n);

	  SPI_Transfer(&dummyRead, 0, 4 +n);

	  SPI_TMSRead(&dummyRead, 4 +n);

	  dummyRead >>= 1;
	  ack = dummyRead & 0x7;
	  dummyRead >>= 3;

	  /* if target responds ack, continue reading
	   * what I do here is dividing 34 bits (32 for data,
	   * 1 for parity and 1 for turnaround) into smaller chunks
	   * because this SPI can issue transactions from 4 bits to 16 bits*/
	  if(DAP_TRANSFER_OK == ack)
	  {
		  /* iterate 34 cycles */

		  read_data |= (dummyRead);

		  ChangeTMS_Size(16);
		  SPI_Transfer(&dummyRead, 0, 16);
		  SPI_TMSRead(&dummyRead, 16);

		  read_data |= (dummyRead) <<1;
		  ChangeTMS_Size(8);

		  SPI_Transfer(&dummyRead, 0, 8);
		  SPI_TMSRead(&dummyRead, 8);

		  read_data |= (dummyRead) << 17;

		  ChangeTMS_Size(33 + n -24 -1);

		  SPI_Transfer(&dummyRead, 0, 33 + n -25);
		  SPI_TMSRead(&dummyRead, 33 + n -25);

		  read_data |= (dummyRead & 0xFF) << 25;

		  parityIdx = (33 + n -25 -2);

		  parity = (dummyRead >> parityIdx) & 0x1;

		  if( !check_even_parity(read_data, parity) && DAP_TRANSFER_OK == ack)
		  {
			  ack = DAP_TRANSFER_ERROR;
		  }

		  *data = read_data;
		  SPI4->CR1 &= ~(0x1 << 6);

	  }

	  else
	  {
		  /* TODO: handle when data phase exist (this else block is useless).
		   * please refer to:
		   * https://developer.arm.com/documentation/ihi0031/a/Debug-Port-Registers/Debug-Port--DP--register-descriptions/The-Control-Status-Register--CTRL-STAT?lang=en
		   * it basically says that when sticky overrun is enabled and the target
		   * responds either fault or wait, there must be a data phase
		   * in order to skip current transaction. thankfully, openocd handles that and reports us
		   * via DAP_Data.swd_conf.data_phase. However, I did not implement it because OpenOCD does
		   * not enable overrun detection.
		   * at high SWCLK frequencies, target may respond WAIT sometime.
		   */
		  if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) != 0U)) // write xfer

		  {
			  bit = 0;

		  }

		  if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) == 0U)) // read xfer
		  {
			  bit = 0;
		  }
	  }



  }
  else
  {
	  SPI_SwitchPhaseToListen();
	  /* Turnaround */
	  /* get acknowledge and release the control of the line*/
	  n = DAP_Data.swd_conf.turnaround;


	  SPI4->CR2 = ( ( 2*n +3 -1) << 8);
	  SPI_Transfer(&dummyRead, 0, 2*n+3);
	  SPI_TMSRead(&dummyRead, 2*n+3);

	  SPI_SwitchPhaseToWrite();

	  dummyRead >>= 1;
	  ack = (dummyRead & 0x7);

	  /* Using the same logic to read, drive the line for data
	   * it is OKAY to drive the line more than 33 bits during WRITE
	   * phase. After the 33rd bit, target SWD controller goes to idle
	   * state and as long as we keep driving 0s, it will stay in IDLE */

	  if(DAP_TRANSFER_OK == ack)
	  {
		  parity = generate_even_parity(*data);
		  write_data = *data;
		  SPI4->CR1 |= (0x1 << 6);

		  SPI_TMS_Transfer(write_data, 16);
		  SPI_Transfer(&dummyRead, 0, 16);
		  SPI_TMSRead(&dummyRead, 16);

		  write_data >>= 16;
		  delay_cnt = 2; //TODO: delay_cnt value is experimental!
		  		  while(delay_cnt--)
		  		  {
		  			  __asm("nop");
		  		  }

		  SPI_TMS_Transfer(write_data, 8);
		  SPI_Transfer(&dummyRead, 0, 8);
		  SPI_TMSRead(&dummyRead, 8);


		  delay_cnt = 2;
		  while(delay_cnt--)
		  {
			  __asm("nop");
		  }

		  write_data >>= 8;

		  write_data |= parity << 8;

		  SPI_TMS_Transfer(write_data, 16);
		  SPI_Transfer(&dummyRead, 0, 16);
		  SPI_TMSRead(&dummyRead, 16);



	  }
	  else
	  {

		  /* TODO: handle when data phase exist.
		   * Since OpenOCD does not enable sticky overrun detection, there is no data phase,
		   * we already performed request, turnaround, acknowledge and another turnaround phase
		   * we can skip the transaction and return WAIT
		   * please refer to:
		   * https://developer.arm.com/documentation/ihi0031/a/Debug-Port-Registers/Debug-Port--DP--register-descriptions/The-Control-Status-Register--CTRL-STAT?lang=en
		   */
	  }

  }

  /* Capture Timestamp */
      if (request & DAP_TRANSFER_TIMESTAMP) {
        DAP_Data.timestamp = TIMESTAMP_GET();
      }

      /* below lines are for debug purposes */
      if(ack == 0x2)
      {
    	  wait_ctr++;
      }

      if(ack == 0x4 || ack == 0x7)
      {
    	  err_ctr++;
      }

      if(ack == 0x1)
      {
    	  success_ctr++;
      }

  return ((uint8_t)ack);
}




// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
uint8_t  SWD_Transfer(uint32_t request, uint32_t *data) {
  if (DAP_Data.fast_clock) {
    return SWD_Transfer_LL(request, data);
  } else {
    return SWD_Transfer_LL(request, data);
  }
}


#endif  /* (DAP_SWD != 0) */
