/*
 * helper.h
 *
 *  Created on: Aug 31, 2025
 *      Author: sahin
 */

#ifndef INC_HELPER_H_
#define INC_HELPER_H_

#include "main.h"

#define IDX_8_BIT 0
#define IDX_RM1_BIT 1
#define IDX_RM2_BIT 2



void copy_bits_lsb(const uint8_t *src, uint32_t srcBitIndex,
                   uint32_t bitLen,
                   uint8_t *dst, uint32_t dstBitIndex);


void write_nbits_lsb(uint8_t *buf, size_t bit_offset, size_t n, uint16_t value);

uint16_t extract_nbits_lsb(const uint8_t *buf, size_t bit_offset, size_t n);

void fill_tdi_buffer(uint32_t total_write_bit_cnt, uint32_t n, uint8_t *tdi_val_ptr);

void fill_tms_buffer(uint32_t total_write_bit_cnt, uint32_t n, uint8_t tms_val);

void calculate_xfer_sizes(uint16_t input_len, uint8_t *buff);



void calculate_xfer_sizes(uint16_t input_len, uint8_t *buff);


uint8_t check_even_parity(uint32_t data, uint8_t parity);
uint8_t generate_even_parity(uint32_t data);

#endif /* INC_HELPER_H_ */
