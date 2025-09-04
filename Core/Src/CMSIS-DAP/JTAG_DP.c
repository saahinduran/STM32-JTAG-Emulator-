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
 * Title:        JTAG_DP.c CMSIS-DAP JTAG DP I/O
 *
 *---------------------------------------------------------------------------*/

#include "DAP_config.h"
#include "DAP.h"
#include "helper.h"
#include "string.h"
#include "port.h"


extern uint8_t TMS_SEQ_ARR[];
extern uint8_t TDI_SEQ_ARR[];
extern uint8_t TDO_SEQ_ARR[];
extern uint8_t TDO_PROCESSED_SEQ_ARR[];



/* This function executes the jtag transfer that is stated by TDI and TMS sequences */
static void apply_jtag_xfer(const uint8_t *tdi, const uint8_t *tms, uint8_t *tdo, uint32_t cnt)
{
	uint8_t xFerSizes[3];
	/* divide the transfer into chunks, we don't want the remainder clock cycle to be less
	 * than 4 since SPI peripheral does not support less than 4 clock cycle transfer.
	 */
	calculate_xfer_sizes(cnt, xFerSizes);

	uint32_t currentBit = 0;

	uint8_t *tms_seq_arr = tms;

	uint8_t *tdi_seq_arr = tdi;


	while(xFerSizes[IDX_8_BIT])
	{
		uint8_t tms_val = *tms;
		uint8_t tdi_val = *tdi;
		uint64_t tdo_val;


		SPI_TMS_Transfer(tms_val, 8);
		SPI_Transfer(&tdo_val, tdi_val , 8);

		*tdo = (uint8_t)tdo_val;

		tms++;
		tdi++;
		tdo++;

		xFerSizes[IDX_8_BIT]--;

		currentBit+= 8;
	}

	while(xFerSizes[IDX_RM1_BIT])
	{

		uint16_t tms_val = extract_nbits_lsb(tms_seq_arr, currentBit, xFerSizes[IDX_RM1_BIT]);
		uint16_t tdi_val = extract_nbits_lsb(tdi_seq_arr, currentBit, xFerSizes[IDX_RM1_BIT]);;
		uint64_t tdo_val;

		SPI_TMS_Transfer(tms_val, xFerSizes[IDX_RM1_BIT]);
		SPI_Transfer(&tdo_val, tdi_val , xFerSizes[IDX_RM1_BIT]);

		write_nbits_lsb(TDO_SEQ_ARR, currentBit, xFerSizes[IDX_RM1_BIT], tdo_val);

		currentBit+= xFerSizes[IDX_RM1_BIT];

		xFerSizes[IDX_RM1_BIT] = 0;

	}

	while(xFerSizes[IDX_RM2_BIT])
	{

		uint16_t tms_val = extract_nbits_lsb(tms_seq_arr, currentBit, xFerSizes[IDX_RM2_BIT]);
		uint16_t tdi_val = extract_nbits_lsb(tdi_seq_arr, currentBit, xFerSizes[IDX_RM2_BIT]);;
		uint64_t tdo_val;

		SPI_TMS_Transfer(tms_val, xFerSizes[IDX_RM2_BIT]);
		SPI_Transfer(&tdo_val, tdi_val , xFerSizes[IDX_RM2_BIT]);

		write_nbits_lsb(TDO_SEQ_ARR, currentBit, xFerSizes[IDX_RM2_BIT], tdo_val);

		currentBit+= xFerSizes[IDX_RM2_BIT];

		xFerSizes[IDX_RM2_BIT] = 0;

	}


}


#if (DAP_JTAG != 0)

// Generate JTAG Sequence
//   info:   sequence information
//   tdi:    pointer to TDI generated data
//   tdo:    pointer to TDO captured data
//   return: none
uint32_t JTAG_Sequence (uint32_t count, const uint8_t *request, uint8_t *response)
{

  uint32_t total_write_bit_cnt = 0;
  uint32_t total_read_bit_cnt = 0;
  uint32_t bytes_needed = 0;

  uint32_t i;

  uint8_t *req_base = request;

  memset(TMS_SEQ_ARR, 0, 256);

  memset(TDI_SEQ_ARR, 0, 256);

  memset(TDO_SEQ_ARR, 0, 256);

  memset(TDO_PROCESSED_SEQ_ARR, 0, 256);



  for(i = 0; i < count; i++)
  {
	  uint32_t n;

	  uint8_t tms_val = (*request & JTAG_SEQUENCE_TMS) >> 6;

	  n = *request & JTAG_SEQUENCE_TCK;

	  if (n == 0U)
	  {
		  n = 64U;
	  }

	  if(tms_val)
	  {
		  fill_tms_buffer(total_write_bit_cnt, n, tms_val);
	  }


	  fill_tdi_buffer(total_write_bit_cnt, n, (request +1));


	  total_write_bit_cnt += n;

	  if(*request & JTAG_SEQUENCE_TDO)
	  {
		  total_read_bit_cnt += n;
	  }


	  request += ( (n + 7U) /8U ) + 1;

  }

  apply_jtag_xfer(TDI_SEQ_ARR, TMS_SEQ_ARR, TDO_SEQ_ARR, total_write_bit_cnt);

  total_write_bit_cnt = 0;

  total_read_bit_cnt = 0;

  for(i = 0; i < count; i++)
  {
	  uint32_t n;



	  n = *req_base & JTAG_SEQUENCE_TCK;

	  if (n == 0U)
  	  {
  		  n = 64U;
  	  }



	  if(*req_base & JTAG_SEQUENCE_TDO)
	  {

		  copy_bits_lsb(TDO_SEQ_ARR, total_write_bit_cnt, n, TDO_PROCESSED_SEQ_ARR, total_read_bit_cnt);
		  total_read_bit_cnt += n;

		  if(total_read_bit_cnt % 8 )
		  {
			  total_read_bit_cnt = ( (total_read_bit_cnt / 8) +1) *8;
		  }


	  }

	  total_write_bit_cnt += n;


	  req_base += ( (n + 7U) /8U ) + 1;

    }


  memcpy(response, TDO_PROCESSED_SEQ_ARR, total_read_bit_cnt /8);

  return total_read_bit_cnt / 8;

}



// JTAG Read IDCODE register
//   return: value read
uint32_t JTAG_ReadIDCode (void)
{

  return (0);
}


// JTAG Write ABORT register
//   data:   value to write
//   return: none
void JTAG_WriteAbort (uint32_t data)
{
	/*TODO: implement this function */
}


// JTAG Set IR
//   ir:     IR value
//   return: none
void JTAG_IR (uint32_t ir)
{
/*TODO: implement this function */
}


// JTAG Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
uint8_t  JTAG_Transfer(uint32_t request, uint32_t *data)
{
	/*TODO: implement this function */
	return (0);
}


#endif  /* (DAP_JTAG != 0) */
