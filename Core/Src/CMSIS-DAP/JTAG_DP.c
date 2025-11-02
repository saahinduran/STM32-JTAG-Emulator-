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

static inline uint32_t round_up_bits_to_bytes(uint32_t bits) { return (bits + 7u) >> 3; }
static inline uint32_t round_up_to_8(uint32_t bits)          { return (bits + 7u) & ~7u; }

uint32_t JTAG_Sequence (uint32_t count, const uint8_t *request, uint8_t *response)
{
  const uint8_t *req = request;

  uint32_t total_write_bits = 0;
  uint32_t total_read_bits  = 0;
  uint32_t tdi_tms_bits     = 0;   // total bits we’ll need for TDI/TMS buffers
  uint32_t tdo_bits_rounded = 0;   // rounded-up total bits we’ll need for TDO buffer

  // ---------- Pass 1: size calculation (no heavy work) ----------
  for (uint32_t i = 0; i < count; i++) {
    uint32_t n = req[0] & JTAG_SEQUENCE_TCK;
    if (n == 0u) n = 64u;

    tdi_tms_bits += n;

    if (req[0] & JTAG_SEQUENCE_TDO) {
      total_read_bits += n;
      tdo_bits_rounded = round_up_to_8(total_read_bits);
    }

    // advance: header + payload bytes
    req += 1 + ((n + 7u) >> 3);
  }

  // Buffers are globals; clear only what we’ll write.
  memset(TMS_SEQ_ARR, 0x00, round_up_bits_to_bytes(tdi_tms_bits));
  memset(TDI_SEQ_ARR, 0x00, round_up_bits_to_bytes(tdi_tms_bits));
  memset(TDO_SEQ_ARR, 0x00, round_up_bits_to_bytes(tdo_bits_rounded));

  // ---------- Pass 2: build TMS/TDI bitstreams ----------
  req = request;
  uint32_t wr_bit_cursor = 0;

  for (uint32_t i = 0; i < count; i++) {
    const uint8_t hdr      = req[0];
    uint32_t       n       = hdr & JTAG_SEQUENCE_TCK;
    if (n == 0u) n = 64u;

    const uint32_t byte_len = (n + 7u) >> 3;
    const uint8_t  tms_bit  = (uint8_t)((hdr & JTAG_SEQUENCE_TMS) >> 6);

    if (tms_bit) {
      // Fill 'n' ones at the proper places for TMS (your helper already handles packing)
      fill_tms_buffer(wr_bit_cursor, n, 1);
    }

    // Quick “all-zero” check on payload to skip fill_tdi_buffer when possible
    uint32_t or_acc = 0;
    const uint8_t *payload = req + 1;
    for (uint32_t k = 0; k < byte_len; k++) {
      or_acc |= payload[k];
    }
    if (or_acc) {
      // Only write when there is at least one '1' in payload
      fill_tdi_buffer(wr_bit_cursor, n, payload);
    }

    wr_bit_cursor += n;
    req += 1 + byte_len;
  }

  // ---------- Transfer ----------
  apply_jtag_xfer(TDI_SEQ_ARR, TMS_SEQ_ARR, TDO_SEQ_ARR, wr_bit_cursor);

  // ---------- Pass 3: extract TDO back to response ----------
  const uint8_t * __restrict req2 = request;
  uint32_t rd_bit_cursor = 0;     // where we write into 'response' (bit index)
  uint32_t scan_bit_off  = 0;     // where we read from TDO_SEQ_ARR   (bit index), follows write order

  for (uint32_t i = 0; i < count; i++) {
    uint32_t n = req2[0] & JTAG_SEQUENCE_TCK;
    if (n == 0u) n = 64u;

    const uint32_t byte_len = (n + 7u) >> 3;

    if (req2[0] & JTAG_SEQUENCE_TDO) {
      // copy the n bits starting at scan_bit_off into 'response' at rd_bit_cursor
      copy_bits_lsb(TDO_SEQ_ARR, scan_bit_off, n, response, rd_bit_cursor);
      rd_bit_cursor = round_up_to_8(rd_bit_cursor + n);  // maintain byte alignment between segments
    }

    scan_bit_off += n;
    req2 += 1 + byte_len;
  }

  return rd_bit_cursor >> 3; // bytes produced
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
	uint32_t required_buf_size = 0;
	uint32_t n;
	uint8_t tms_buff[64];
	uint8_t tdi_buff[64];
	uint8_t tdo_buff[64];

	required_buf_size += 4;
	required_buf_size += DAP_Data.jtag_dev.ir_before[DAP_Data.jtag_dev.index];
	required_buf_size += DAP_Data.jtag_dev.ir_length[DAP_Data.jtag_dev.index];
	required_buf_size += DAP_Data.jtag_dev.ir_after[DAP_Data.jtag_dev.index];
	required_buf_size += 4;

	memset(tms_buff, 0x0, required_buf_size / 8 +1);
	memset(tdi_buff, 0x0, required_buf_size / 8 +1);


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
	uint32_t required_buf_size = 0;
	uint8_t tms_buff[64];
	uint8_t tdi_buff[64];
	uint8_t tdo_buff[64];

	required_buf_size += 5;
	required_buf_size += DAP_Data.jtag_dev.index;
	required_buf_size += 3;


	memset(tms_buff, 0x0, required_buf_size / 8 +1);
	memset(tdi_buff, 0x0, required_buf_size / 8 +1);


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
