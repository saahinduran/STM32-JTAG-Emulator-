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
static inline void apply_jtag_xfer(const uint8_t *tdi, const uint8_t *tms, uint8_t *tdo, uint32_t cnt)
{
	uint8_t xFerSizes[3];
	/* divide the transfer into chunks, we don't want the remainder clock cycle to be less
	 * than 4 since SPI peripheral does not support less than 4 clock cycle transfer.
	 */
	calculate_xfer_sizes(cnt, xFerSizes);

	uint32_t currentBit = 0;

	uint8_t *tms_seq_arr = tms;

	uint8_t *tdi_seq_arr = tdi;

	uint8_t *tdo_seq_arr = tdo;


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

		write_nbits_lsb(tdo_seq_arr, currentBit, xFerSizes[IDX_RM1_BIT], tdo_val);

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

		write_nbits_lsb(tdo_seq_arr, currentBit, xFerSizes[IDX_RM2_BIT], tdo_val);

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

		  copy_bits_lsb(TDO_SEQ_ARR, total_write_bit_cnt, n, response, total_read_bit_cnt);
		  total_read_bit_cnt += n;

		  if(total_read_bit_cnt % 8 )
		  {
			  total_read_bit_cnt = ( (total_read_bit_cnt / 8) +1) *8;
		  }


	  }

	  total_write_bit_cnt += n;


	  req_base += ( (n + 7U) /8U ) + 1;

    }

  return total_read_bit_cnt / 8;

}



// JTAG Read IDCODE register
//   return: value read
uint32_t JTAG_ReadIDCode (void)
{

	uint32_t n;
	uint32_t total_bit_cnt = 0;
	uint8_t tms_buff[8] = {0};
	uint8_t tdi_buff[2] = {0};
	uint8_t tdo_buff[8] = {0};

	/* Move the TAP controller to SHIFT-DR state */
	write_nbits_lsb(tms_buff, 0, 4, 0x2);
	total_bit_cnt += 4;

	/* Bypass before data */
	n = DAP_Data.jtag_dev.index;

	while(n > 8)
	{
		write_nbits_lsb(tdi_buff, total_bit_cnt, 0xff, 8);
		total_bit_cnt +=8;
		n -= 8;
	}

	total_bit_cnt += n;

	apply_jtag_xfer(tdi_buff, tms_buff, tdo_buff, total_bit_cnt);

	/* We are ready to shift IDCODE in */
	total_bit_cnt = 32;

	memset(tms_buff, 0, total_bit_cnt / 8 +1);

	write_nbits_lsb(tms_buff, total_bit_cnt -1, 8, 0x03);
	total_bit_cnt += 8;

	apply_jtag_xfer(tdi_buff, tms_buff, tdo_buff, total_bit_cnt);

	return (uint32_t )(tdo_buff[3] << 24) | (tdo_buff[2] << 16) | (tdo_buff[1] << 8) | (tdo_buff[0]) ;
}


// JTAG Write ABORT register
//   data:   value to write
//   return: none
void JTAG_WriteAbort (uint32_t data)
{
	uint32_t n;

	uint32_t total_bit_cnt = 0;
	uint8_t tms_buff[64] = {0};
	uint8_t tdi_buff[64] = {0};
	uint8_t tdo_buff[64] = {0};


	write_nbits_lsb(tms_buff, 0, 5, 0x04);
	total_bit_cnt += 5;

	//TODO: There might be more than 8 devices on the chain!!! (Very odd.)
	n = DAP_Data.jtag_dev.index;

	/* BYPASS BEFORE DEVICE, TARGET DEVICE, DEVICE AT TDO HAS INDEX 0 */
	total_bit_cnt += n;
	write_nbits_lsb(tdi_buff, total_bit_cnt, 3, 0x0);

	/* APPLY JTAG TRANSFER */
	total_bit_cnt += 3;
	apply_jtag_xfer(tdi_buff, tms_buff, tdo_buff, total_bit_cnt);


	/* Write Transfer */
	total_bit_cnt = 32;
	n = DAP_Data.jtag_dev.count - DAP_Data.jtag_dev.index - 1U;

	if( (int)n > 0)
		total_bit_cnt += n;


	memset(tms_buff, 0x0, total_bit_cnt /8 +1);

	tdi_buff[0] = (data & 0xFF);
	tdi_buff[1] = (data & 0xFF00) >> 8;
	tdi_buff[2] = (data & 0xFF0000) >> 16;
	tdi_buff[3] = (data & 0xFF000000) >> 24;

	write_nbits_lsb(tms_buff, total_bit_cnt -1 , 0x4, 0x3);
	total_bit_cnt += 4;

	/* Insert Idle cycles */
	n = DAP_Data.transfer.idle_cycles;
	total_bit_cnt += n;

	apply_jtag_xfer(tdi_buff, tms_buff, tdo_buff, total_bit_cnt);

}


// JTAG Set IR
//   ir:     IR value
//   return: none
void JTAG_IR (uint32_t ir)
{
	uint32_t total_bit_cnt = 0;
	uint32_t n;
	uint8_t tms_buff[64] = {0};
	uint8_t tdi_buff[64] = {0};
	uint8_t tdo_buff[64];

	write_nbits_lsb(tms_buff, 0, 4, 0x3);

	total_bit_cnt += 4;

	n = DAP_Data.jtag_dev.ir_before[DAP_Data.jtag_dev.index];


	while(n > 8)

	{
		write_nbits_lsb(tdi_buff, total_bit_cnt, 8, 0xff);
		total_bit_cnt +=8;
		n -= 8;
	}

	write_nbits_lsb(tdi_buff, total_bit_cnt, n, 0xff);

	total_bit_cnt +=n;

	n = DAP_Data.jtag_dev.ir_length[DAP_Data.jtag_dev.index];


	while(n > 8)

	{
		write_nbits_lsb(tdi_buff, total_bit_cnt, 8, ir);
		ir >>= 8;
		total_bit_cnt +=8;
		n -= 8;
	}

	write_nbits_lsb(tdi_buff, total_bit_cnt, n, ir);


	total_bit_cnt += n;

	n = DAP_Data.jtag_dev.ir_after[DAP_Data.jtag_dev.index];

	if (n)
	{
		while(n > 8)
		{
			write_nbits_lsb(tdi_buff, total_bit_cnt, 8, 0xff);

			total_bit_cnt +=8;
			n -= 8;
		}

		write_nbits_lsb(tdi_buff, total_bit_cnt, n, 0xff);
		total_bit_cnt += n;

	}


	/* add return path to IDLE state */
	write_nbits_lsb(tms_buff, total_bit_cnt-1, 4, 0x3);
	total_bit_cnt += 4;

	apply_jtag_xfer(tdi_buff, tms_buff, tdo_buff, total_bit_cnt);


}


// JTAG Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
uint8_t  JTAG_Transfer(uint32_t request, uint32_t *data)
{
	uint8_t ack = 0;
	uint32_t n;

	uint32_t total_bit_cnt = 0;
	uint8_t tms_buff[64] = {0};
	uint8_t tdi_buff[64] = {0};
	uint8_t tdo_buff[64] = {0};

	write_nbits_lsb(tms_buff, 0, 5, 0x04);
	total_bit_cnt += 5;

	//TODO: There might be more than 8 devices on the chain!!! (Very odd.)
	n = DAP_Data.jtag_dev.index;

	/* BYPASS BEFORE DEVICE, TARGET DEVICE, DEVICE AT TDO HAS INDEX 0 */
	total_bit_cnt += n;

	write_nbits_lsb(tdi_buff, total_bit_cnt, 3, request >> 1);

	/* APPLY JTAG TRANSFER */
	total_bit_cnt += 3;
	apply_jtag_xfer(tdi_buff, tms_buff, tdo_buff, total_bit_cnt);


	copy_bits_lsb(tdo_buff, total_bit_cnt -3,
	                     3,
	                     &ack , 0);


	  if (ack != 0x2)
	  {
		  /* Exit on error */
		  total_bit_cnt = 8;
		  write_nbits_lsb(tms_buff, total_bit_cnt, 0x8, 0x3);
		  apply_jtag_xfer(tdi_buff, tms_buff, tdo_buff, total_bit_cnt);
	  }
	  else if (ack == 0x2 && (request & DAP_TRANSFER_RnW) )
	  {
		  /* Read Transfer */
		  total_bit_cnt = 32;
		  n = DAP_Data.jtag_dev.count - DAP_Data.jtag_dev.index - 1U;
		  if( (int)n > 0)
			  total_bit_cnt += n;

		  memset(tms_buff, 0x0, total_bit_cnt /8 +1);

		  write_nbits_lsb(tms_buff, total_bit_cnt -1 , 0x4, 0x3);

		  total_bit_cnt += 4;

		  /* Insert Idle cycles */
		  n = DAP_Data.transfer.idle_cycles;
		  total_bit_cnt += n;

		  apply_jtag_xfer(tdi_buff, tms_buff, tdo_buff, total_bit_cnt);

		  *data = 0;
		  *data |= (tdo_buff[3] << 24) | (tdo_buff[2] << 16) | (tdo_buff[1] << 8) | (tdo_buff[0]) ;


	  }
	  else if(ack == 0x2 && !(request & DAP_TRANSFER_RnW) )
	  {
		  /* Write Transfer */
		  uint32_t xFerData = *data;

		  total_bit_cnt = 32;
		  n = DAP_Data.jtag_dev.count - DAP_Data.jtag_dev.index - 1U;
		  if( (int)n > 0)
			  total_bit_cnt += n;

		  memset(tms_buff, 0x0, total_bit_cnt /8 +1);

		  tdi_buff[0] = (xFerData & 0xFF);
		  tdi_buff[1] = (xFerData & 0xFF00) >> 8;
		  tdi_buff[2] = (xFerData & 0xFF0000) >> 16;
		  tdi_buff[3] = (xFerData & 0xFF000000) >> 24;

		  write_nbits_lsb(tms_buff, total_bit_cnt -1 , 0x4, 0x3);

		  total_bit_cnt += 4;

		  /* Insert Idle cycles */
		  n = DAP_Data.transfer.idle_cycles;
		  total_bit_cnt += n;


		  apply_jtag_xfer(tdi_buff, tms_buff, tdo_buff, total_bit_cnt);

	  }

	  /* Capture Timestamp */
	  if (request & DAP_TRANSFER_TIMESTAMP) {
	    DAP_Data.timestamp = TIMESTAMP_GET();
	  }


	  /* JTAG ACK and SW-DP ACK bit indexes are not the same!! */
	  if(0x02 == ack)
	  {
		  ack = DAP_TRANSFER_OK;
	  }
	  else if(0x1 == ack)
	  {
		  ack = DAP_TRANSFER_WAIT;
	  }

	  return ((uint8_t)ack);
}


#endif  /* (DAP_JTAG != 0) */
