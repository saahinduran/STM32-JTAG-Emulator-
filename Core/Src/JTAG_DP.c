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


uint8_t TMS_SEQ_ARR[1024];
uint8_t TDI_SEQ_ARR[1024];
uint8_t TDO_SEQ_ARR[1024];
uint8_t TDO_PROCESSED_SEQ_ARR[1024];

int SPI_Transfer(uint64_t *rdData, uint64_t wrData, uint8_t bitSize);


void SPI_TMS_Transfer(uint64_t data, uint8_t bits);

void copy_bits_lsb(const uint8_t *src, uint32_t srcBitIndex,
                   uint32_t bitLen,
                   uint8_t *dst, uint32_t dstBitIndex)
{
    for (uint32_t i = 0; i < bitLen; i++)
    {
        // Locate the bit in the source
        uint32_t sByte = (srcBitIndex + i) / 8;
        uint32_t sBit  = (srcBitIndex + i) % 8; // LSB-first
        uint8_t  bit   = (src[sByte] >> sBit) & 1;

        // Locate where to put it in the destination
        uint32_t dByte = (dstBitIndex + i) / 8;
        uint32_t dBit  = (dstBitIndex + i) % 8; // LSB-first

        // Clear and set the destination bit
        dst[dByte] &= ~(1U << dBit);
        dst[dByte] |=  (bit << dBit);
    }
}


// Extracts 8 bits from a uint8_t array (LSB-first), starting at `bit_index`
// Returns the extracted bits right-aligned
// Extract up to 16 bits from a bitstream (LSB-first)
uint16_t extract_nbits_lsb(const uint8_t *buf, size_t bit_offset, size_t n) {
    if (n == 0 || n > 16) return 0;

    size_t byte_offset = bit_offset / 8;
    size_t bit_in_byte = bit_offset % 8;

    // Read 3 bytes to cover the worst case (bits span across 3 bytes)
    uint32_t temp = buf[byte_offset] |
                   ((uint32_t)buf[byte_offset + 1] << 8) |
                   ((uint32_t)buf[byte_offset + 2] << 16);

    // Right shift to the relevant bit, then mask
    return (temp >> bit_in_byte) & ((1U << n) - 1);
}

#if 0
void write_nbits_lsb(uint8_t *dst, size_t bit_index, size_t n, uint8_t value) {
    size_t byte_index = bit_index / 8;
    size_t bit_offset = bit_index % 8;

    // Load 16-bit window
    uint16_t window = ((uint16_t)dst[byte_index]) |
                      ((uint16_t)dst[byte_index + 1] << 8);

    // Clear target bits
    uint16_t mask = ((1U << n) - 1) << bit_offset;
    window = (window & ~mask) | ((uint16_t)(value & ((1U << n) - 1)) << bit_offset);

    // Store back
    dst[byte_index] = window & 0xFF;
    dst[byte_index + 1] = (window >> 8) & 0xFF;
}
#endif
void write_nbits_lsb(uint8_t *buf, size_t bit_offset, size_t n, uint16_t value) {
    if (n == 0 || n > 16) return;

    size_t byte_offset = bit_offset / 8;
    size_t bit_in_byte = bit_offset % 8;

    // Read 3 bytes to cover the worst case (spanning across 3 bytes)
    uint32_t temp = buf[byte_offset] |
                   ((uint32_t)buf[byte_offset + 1] << 8) |
                   ((uint32_t)buf[byte_offset + 2] << 16);

    // Mask and insert new value
    uint32_t mask = ((1U << n) - 1U) << bit_in_byte;
    temp = (temp & ~mask) | (((uint32_t)(value & ((1U << n) - 1))) << bit_in_byte);

    // Write the result back
    buf[byte_offset]     = temp & 0xFF;
    buf[byte_offset + 1] = (temp >> 8) & 0xFF;
    buf[byte_offset + 2] = (temp >> 16) & 0xFF;
}

void fill_tms_buffer(uint32_t total_write_bit_cnt, uint32_t n, uint8_t tms_val)
{
	//TODO: optimize here!!
	while(n)
	{
		if(n > 8)
		{
			if(tms_val)
			{
				tms_val = 0xFF;
				write_nbits_lsb(TMS_SEQ_ARR, total_write_bit_cnt, 8, tms_val);
				total_write_bit_cnt += 8;
			}

			n -= 8;
		}
		else
		{
			write_nbits_lsb(TMS_SEQ_ARR, total_write_bit_cnt, n, tms_val);
			break;
		}

	}
}

void fill_tdi_buffer(uint32_t total_write_bit_cnt, uint32_t n, uint8_t *tdi_val_ptr)
{
	//TODO: optimize here!!
	while(n)
	{
		if(n > 8)
		{
			write_nbits_lsb(TDI_SEQ_ARR, total_write_bit_cnt, 8, *tdi_val_ptr);
			n -= 8;
			tdi_val_ptr++;
			total_write_bit_cnt += 8;
		}
		else
		{
			write_nbits_lsb(TDI_SEQ_ARR, total_write_bit_cnt, n, *tdi_val_ptr);
			break;
		}

	}
}

#define IDX_8_BIT 0
#define IDX_RM1_BIT 1
#define IDX_RM2_BIT 2
void calculate_xfer_sizes(uint16_t input_len, uint8_t *buff)
{
	int isunAligned = input_len % 8 < 4 && input_len % 8 != 0;
	int isGreaterThan8 = input_len > 8;

	if(isunAligned && isGreaterThan8)
	{
		buff[IDX_8_BIT] = input_len / 8 -2;
		buff[IDX_RM1_BIT] = 4;
		buff[IDX_RM2_BIT] = input_len - buff[IDX_8_BIT]*8 - buff[IDX_RM1_BIT];
	}

	else if (input_len < 8)
	{
		buff[IDX_8_BIT] = 0;
		buff[IDX_RM1_BIT] = input_len % 8;
		buff[IDX_RM2_BIT] = 0;
	}
	else
	{
		buff[IDX_8_BIT] = input_len /8;
		buff[IDX_RM1_BIT] = input_len % 8;
		buff[IDX_RM2_BIT] = 0;
	}

}

void apply_jtag_xfer(const uint8_t *tdi, const uint8_t *tms, uint8_t *tdo, uint32_t cnt)
{
	uint8_t xFerSizes[3];

	uint8_t dummyVal = 0;
	calculate_xfer_sizes(cnt, xFerSizes);

	uint32_t currentBit = 0;

	uint8_t *tms_seq_arr = tms;

	uint8_t *tdi_seq_arr = tdi;


	while(xFerSizes[IDX_8_BIT])
	{
		uint8_t tms_val = *tms;
		uint8_t tdi_val = *tdi;
		uint64_t tdo_val;
		if(tms_val != 0 && currentBit != 0)
		{
			dummyVal = 1;
		}

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
		uint32_t delay_cnt = 2500;

		while(delay_cnt--)
	    {
		  __asm("nop");
	    }

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
		//TODO: extract bits!!
		uint32_t delay_cnt = 2000;

		while(delay_cnt--)
		{
		  __asm("nop");
		}

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
void shift_right_bitstream_lsb(uint8_t *data, size_t num_bits, size_t n) {
    if (n == 0 || num_bits == 0 || n >= num_bits)
    {
        // If n >= num_bits, clear all bits
        size_t num_bytes = (num_bits + 7) / 8;
        for (size_t i = 0; i < num_bytes; i++) {
            data[i] = 0;
        }
        return;
    }

    for (size_t i = 0; i < num_bits - n; i++) {
        // Read source bit
        size_t from_bit = i + n;
        size_t from_byte = from_bit / 8;
        size_t from_off = from_bit % 8;
        uint8_t bit = (data[from_byte] >> from_off) & 1;

        // Write destination bit
        size_t to_byte = i / 8;
        size_t to_off = i % 8;
        data[to_byte] &= ~(1 << to_off);
        data[to_byte] |= (bit << to_off);
    }

    // Zero the remaining n bits at the top
    for (size_t i = num_bits - n; i < num_bits; i++) {
        size_t byte = i / 8;
        size_t bit = i % 8;
        data[byte] &= ~(1 << bit);
    }
}

// JTAG Macros

#define PIN_TCK_SET PIN_SWCLK_TCK_SET
#define PIN_TCK_CLR PIN_SWCLK_TCK_CLR
#define PIN_TMS_SET PIN_SWDIO_TMS_SET
#define PIN_TMS_CLR PIN_SWDIO_TMS_CLR

#define JTAG_CYCLE_TCK()                \
  PIN_TCK_CLR();                        \
  PIN_DELAY();                          \
  PIN_TCK_SET();                        \
  PIN_DELAY()

#define JTAG_CYCLE_TDI(tdi)             \
  PIN_TDI_OUT(tdi);                     \
  PIN_TCK_CLR();                        \
  PIN_DELAY();                          \
  PIN_TCK_SET();                        \
  PIN_DELAY()

#define JTAG_CYCLE_TDO(tdo)             \
  PIN_TCK_CLR();                        \
  PIN_DELAY();                          \
  tdo = PIN_TDO_IN();                   \
  PIN_TCK_SET();                        \
  PIN_DELAY()

#define JTAG_CYCLE_TDIO(tdi,tdo)        \
  PIN_TDI_OUT(tdi);                     \
  PIN_TCK_CLR();                        \
  PIN_DELAY();                          \
  tdo = PIN_TDO_IN();                   \
  PIN_TCK_SET();                        \
  PIN_DELAY()

#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)


#if (DAP_JTAG != 0)

// Generate JTAG Sequence
//   info:   sequence information
//   tdi:    pointer to TDI generated data
//   tdo:    pointer to TDO captured data
//   return: none
uint32_t JTAG_Sequence (uint32_t count, const uint8_t *request, uint8_t *response)
{
	static volatile uint32_t firstTime;
	static volatile uint32_t elapsed_time;

	static volatile uint32_t memcpy_elapsed_time;
	static volatile uint32_t memcpy_first;

	firstTime = DWT->CYCCNT;


	static int cnt = 0;
	int dummyVal = 31;
  uint32_t total_write_bit_cnt = 0;
  uint32_t total_read_bit_cnt = 0;

  uint32_t total_write_word_cnt = 0;
  uint32_t total_read_word_cnt = 0;

  uint32_t tdo_capture_index = 0;

  uint32_t tms_seq_val = 0;

  uint32_t i;

  uint8_t *req_base = request;

  memset(TMS_SEQ_ARR, 0, 256);

  memset(TDI_SEQ_ARR, 0, 256);

  memset(TDO_SEQ_ARR, 0, 256);

  memset(TDO_PROCESSED_SEQ_ARR, 0, 256);


  cnt++;


  for(i = 0; i < count; i++)
  {
	  uint32_t n;

	  uint8_t tms_val = (*request & JTAG_SEQUENCE_TMS) >> 6;

	  uint8_t tdi_val = *(request +1);

	  n = *request & JTAG_SEQUENCE_TCK;

	  if (n == 0U)
	  {
		  n = 64U;
	  }

	  fill_tms_buffer(total_write_bit_cnt, n, tms_val);
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


  //shift_right_bitstream_lsb(TDO_SEQ_ARR, total_write_bit_cnt, total_write_bit_cnt - total_read_bit_cnt);

  memcpy_first = DWT->CYCCNT;

  memcpy(response, TDO_PROCESSED_SEQ_ARR, total_read_bit_cnt /8);

  memcpy_elapsed_time = DWT ->CYCCNT - memcpy_first;

  elapsed_time = DWT->CYCCNT - firstTime;

  return total_read_bit_cnt / 8;

}


// JTAG Set IR
//   ir:     IR value
//   return: none
#define JTAG_IR_Function(speed) /**/                                            \
static void JTAG_IR_##speed (uint32_t ir) {                                     \
  uint32_t n;                                                                   \
                                                                                \
  PIN_TMS_SET();                                                                \
  JTAG_CYCLE_TCK();                         /* Select-DR-Scan */                \
  JTAG_CYCLE_TCK();                         /* Select-IR-Scan */                \
  PIN_TMS_CLR();                                                                \
  JTAG_CYCLE_TCK();                         /* Capture-IR */                    \
  JTAG_CYCLE_TCK();                         /* Shift-IR */                      \
                                                                                \
  PIN_TDI_OUT(1U);                                                              \
  for (n = DAP_Data.jtag_dev.ir_before[DAP_Data.jtag_dev.index]; n; n--) {      \
    JTAG_CYCLE_TCK();                       /* Bypass before data */            \
  }                                                                             \
  for (n = DAP_Data.jtag_dev.ir_length[DAP_Data.jtag_dev.index] - 1U; n; n--) { \
    JTAG_CYCLE_TDI(ir);                     /* Set IR bits (except last) */     \
    ir >>= 1;                                                                   \
  }                                                                             \
  n = DAP_Data.jtag_dev.ir_after[DAP_Data.jtag_dev.index];                      \
  if (n) {                                                                      \
    JTAG_CYCLE_TDI(ir);                     /* Set last IR bit */               \
    PIN_TDI_OUT(1U);                                                            \
    for (--n; n; n--) {                                                         \
      JTAG_CYCLE_TCK();                     /* Bypass after data */             \
    }                                                                           \
    PIN_TMS_SET();                                                              \
    JTAG_CYCLE_TCK();                       /* Bypass & Exit1-IR */             \
  } else {                                                                      \
    PIN_TMS_SET();                                                              \
    JTAG_CYCLE_TDI(ir);                     /* Set last IR bit & Exit1-IR */    \
  }                                                                             \
                                                                                \
  JTAG_CYCLE_TCK();                         /* Update-IR */                     \
  PIN_TMS_CLR();                                                                \
  JTAG_CYCLE_TCK();                         /* Idle */                          \
  PIN_TDI_OUT(1U);                                                              \
}

static void JTAG_IR_Benim (uint32_t ir) {
  uint32_t total_bit_cnt = 0;
  uint32_t n;

  uint8_t tms_buff[64] = {0};
  uint8_t tdi_buff[64] = {0};

  write_nbits_lsb(tms_buff, 0, 4, 0x03);

  total_bit_cnt += 4;

  write_nbits_lsb(tdi_buff, 0, 4, 0x00);


  n = DAP_Data.jtag_dev.ir_before[DAP_Data.jtag_dev.index];

  write_nbits_lsb(tms_buff, total_bit_cnt, n, 0x00);

  write_nbits_lsb(tdi_buff, total_bit_cnt, n, 0xff);


  n = DAP_Data.jtag_dev.ir_length[DAP_Data.jtag_dev.index];

  write_nbits_lsb(tms_buff, total_bit_cnt, n, 0);

  write_nbits_lsb(tdi_buff, total_bit_cnt, n, ir);

  total_bit_cnt += n;


  n = DAP_Data.jtag_dev.ir_after[DAP_Data.jtag_dev.index];
  if (n)
  {

	  write_nbits_lsb(tdi_buff, total_bit_cnt, n, 0xff);
	  write_nbits_lsb(tms_buff, total_bit_cnt, n, 0x0);

	  total_bit_cnt += n;

	  write_nbits_lsb(tms_buff, total_bit_cnt, 1, 1);

	  total_bit_cnt++;

  }
  else
  {
	  write_nbits_lsb(tms_buff, total_bit_cnt, 1, 1);

	  total_bit_cnt++;
  }

  write_nbits_lsb(tms_buff, total_bit_cnt, 3, 0x01);

  total_bit_cnt += 3;

}


// JTAG Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
#define JTAG_TransferFunction(speed)        /**/                                \
static uint8_t JTAG_Transfer##speed (uint32_t request, uint32_t *data) {        \
  uint32_t ack;                                                                 \
  uint32_t bit;                                                                 \
  uint32_t val;                                                                 \
  uint32_t n;                                                                   \
                                                                                \
  PIN_TMS_SET();                                                                \
  JTAG_CYCLE_TCK();                         /* Select-DR-Scan */                \
  PIN_TMS_CLR();                                                                \
  JTAG_CYCLE_TCK();                         /* Capture-DR */                    \
  JTAG_CYCLE_TCK();                         /* Shift-DR */                      \
                                                                                \
  for (n = DAP_Data.jtag_dev.index; n; n--) {                                   \
    JTAG_CYCLE_TCK();                       /* Bypass before data */            \
  }                                                                             \
                                                                                \
  JTAG_CYCLE_TDIO(request >> 1, bit);       /* Set RnW, Get ACK.0 */            \
  ack  = bit << 1;                                                              \
  JTAG_CYCLE_TDIO(request >> 2, bit);       /* Set A2,  Get ACK.1 */            \
  ack |= bit << 0;                                                              \
  JTAG_CYCLE_TDIO(request >> 3, bit);       /* Set A3,  Get ACK.2 */            \
  ack |= bit << 2;                                                              \
                                                                                \
  if (ack != DAP_TRANSFER_OK) {                                                 \
    /* Exit on error */                                                         \
    PIN_TMS_SET();                                                              \
    JTAG_CYCLE_TCK();                       /* Exit1-DR */                      \
    goto exit;                                                                  \
  }                                                                             \
                                                                                \
  if (request & DAP_TRANSFER_RnW) {                                             \
    /* Read Transfer */                                                         \
    val = 0U;                                                                   \
    for (n = 31U; n; n--) {                                                     \
      JTAG_CYCLE_TDO(bit);                  /* Get D0..D30 */                   \
      val  |= bit << 31;                                                        \
      val >>= 1;                                                                \
    }                                                                           \
    n = DAP_Data.jtag_dev.count - DAP_Data.jtag_dev.index - 1U;                 \
    if (n) {                                                                    \
      JTAG_CYCLE_TDO(bit);                  /* Get D31 */                       \
      for (--n; n; n--) {                                                       \
        JTAG_CYCLE_TCK();                   /* Bypass after data */             \
      }                                                                         \
      PIN_TMS_SET();                                                            \
      JTAG_CYCLE_TCK();                     /* Bypass & Exit1-DR */             \
    } else {                                                                    \
      PIN_TMS_SET();                                                            \
      JTAG_CYCLE_TDO(bit);                  /* Get D31 & Exit1-DR */            \
    }                                                                           \
    val |= bit << 31;                                                           \
    if (data) { *data = val; }                                                  \
  } else {                                                                      \
    /* Write Transfer */                                                        \
    val = *data;                                                                \
    for (n = 31U; n; n--) {                                                     \
      JTAG_CYCLE_TDI(val);                  /* Set D0..D30 */                   \
      val >>= 1;                                                                \
    }                                                                           \
    n = DAP_Data.jtag_dev.count - DAP_Data.jtag_dev.index - 1U;                 \
    if (n) {                                                                    \
      JTAG_CYCLE_TDI(val);                  /* Set D31 */                       \
      for (--n; n; n--) {                                                       \
        JTAG_CYCLE_TCK();                   /* Bypass after data */             \
      }                                                                         \
      PIN_TMS_SET();                                                            \
      JTAG_CYCLE_TCK();                     /* Bypass & Exit1-DR */             \
    } else {                                                                    \
      PIN_TMS_SET();                                                            \
      JTAG_CYCLE_TDI(val);                  /* Set D31 & Exit1-DR */            \
    }                                                                           \
  }                                                                             \
                                                                                \
exit:                                                                           \
  JTAG_CYCLE_TCK();                         /* Update-DR */                     \
  PIN_TMS_CLR();                                                                \
  JTAG_CYCLE_TCK();                         /* Idle */                          \
  PIN_TDI_OUT(1U);                                                              \
                                                                                \
  /* Capture Timestamp */                                                       \
  if (request & DAP_TRANSFER_TIMESTAMP) {                                       \
    DAP_Data.timestamp = TIMESTAMP_GET();                                       \
  }                                                                             \
                                                                                \
  /* Idle cycles */                                                             \
  n = DAP_Data.transfer.idle_cycles;                                            \
  while (n--) {                                                                 \
    JTAG_CYCLE_TCK();                       /* Idle */                          \
  }                                                                             \
                                                                                \
  return ((uint8_t)ack);                                                        \
}


#undef  PIN_DELAY
#define PIN_DELAY() PIN_DELAY_FAST()
JTAG_IR_Function(Fast)
JTAG_TransferFunction(Fast)

#undef  PIN_DELAY
#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)
JTAG_IR_Function(Slow)
JTAG_TransferFunction(Slow)


// JTAG Read IDCODE register
//   return: value read
uint32_t JTAG_ReadIDCode (void) {
  uint32_t bit;
  uint32_t val;
  uint32_t n;

  PIN_TMS_SET();
  JTAG_CYCLE_TCK();                         /* Select-DR-Scan */
  PIN_TMS_CLR();
  JTAG_CYCLE_TCK();                         /* Capture-DR */
  JTAG_CYCLE_TCK();                         /* Shift-DR */

  for (n = DAP_Data.jtag_dev.index; n; n--) {
    JTAG_CYCLE_TCK();                       /* Bypass before data */
  }

  val = 0U;
  for (n = 31U; n; n--) {
    JTAG_CYCLE_TDO(bit);                    /* Get D0..D30 */
    val  |= bit << 31;
    val >>= 1;
  }
  PIN_TMS_SET();
  JTAG_CYCLE_TDO(bit);                      /* Get D31 & Exit1-DR */
  val |= bit << 31;

  JTAG_CYCLE_TCK();                         /* Update-DR */
  PIN_TMS_CLR();
  JTAG_CYCLE_TCK();                         /* Idle */

  return (val);
}


// JTAG Write ABORT register
//   data:   value to write
//   return: none
void JTAG_WriteAbort (uint32_t data) {
  uint32_t n;

  PIN_TMS_SET();
  JTAG_CYCLE_TCK();                         /* Select-DR-Scan */
  PIN_TMS_CLR();
  JTAG_CYCLE_TCK();                         /* Capture-DR */
  JTAG_CYCLE_TCK();                         /* Shift-DR */

  for (n = DAP_Data.jtag_dev.index; n; n--) {
    JTAG_CYCLE_TCK();                       /* Bypass before data */
  }

  PIN_TDI_OUT(0U);
  JTAG_CYCLE_TCK();                         /* Set RnW=0 (Write) */
  JTAG_CYCLE_TCK();                         /* Set A2=0 */
  JTAG_CYCLE_TCK();                         /* Set A3=0 */

  for (n = 31U; n; n--) {
    JTAG_CYCLE_TDI(data);                   /* Set D0..D30 */
    data >>= 1;
  }
  n = DAP_Data.jtag_dev.count - DAP_Data.jtag_dev.index - 1U;
  if (n) {
    JTAG_CYCLE_TDI(data);                   /* Set D31 */
    for (--n; n; n--) {
      JTAG_CYCLE_TCK();                     /* Bypass after data */
    }
    PIN_TMS_SET();
    JTAG_CYCLE_TCK();                       /* Bypass & Exit1-DR */
  } else {
    PIN_TMS_SET();
    JTAG_CYCLE_TDI(data);                   /* Set D31 & Exit1-DR */
  }

  JTAG_CYCLE_TCK();                         /* Update-DR */
  PIN_TMS_CLR();
  JTAG_CYCLE_TCK();                         /* Idle */
  PIN_TDI_OUT(1U);
}


// JTAG Set IR
//   ir:     IR value
//   return: none
void JTAG_IR (uint32_t ir) {
  if (DAP_Data.fast_clock) {
    JTAG_IR_Fast(ir);
  } else {
    JTAG_IR_Slow(ir);
  }
}


// JTAG Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
uint8_t  JTAG_Transfer(uint32_t request, uint32_t *data) {
  if (DAP_Data.fast_clock) {
    return JTAG_TransferFast(request, data);
  } else {
    return JTAG_TransferSlow(request, data);
  }
}


#endif  /* (DAP_JTAG != 0) */
