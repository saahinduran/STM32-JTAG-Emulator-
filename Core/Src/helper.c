

#include "helper.h"
#include "port.h"

extern uint8_t TMS_SEQ_ARR[];
extern uint8_t TDI_SEQ_ARR[];
extern uint8_t TDO_SEQ_ARR[];
extern uint8_t TDO_PROCESSED_SEQ_ARR[];


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

void inline calculate_xfer_sizes(uint16_t input_len, uint8_t *buff)
{
	/* divide the transfer into chunks, we don't want the remainder clock cycle to be less
	 * than 4 since SPI peripheral does not support less than 4 clock cycle transfer.
	 */

	int isunAligned = input_len % 8 < 4 && input_len % 8 != 0;
	int isGreaterThan8 = input_len > 8;

	if(isunAligned && isGreaterThan8)
	{
		if( (input_len / 8 -2) > 0)
		{
			buff[IDX_8_BIT] = input_len / 8 -2;
		}
		else
		{
			buff[IDX_8_BIT] = 0;
		}

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


// Check if data + parity bit satisfies even parity
uint8_t check_even_parity(uint32_t data, uint8_t parity) {
    uint8_t total_ones = parity;
    uint32_t temp = data;

    while (temp) {
        total_ones ^= (temp & 1);
        temp >>= 1;
    }

    // If XOR result is 0, then even parity is satisfied
    return (total_ones == 0);
}

// Generate even parity bit for given data
uint8_t generate_even_parity(uint32_t data)
{
    uint8_t count = 0;
    uint32_t temp = data;

    // Count number of 1s
    while (temp) {
        count ^= (temp & 1);
        temp >>= 1;
    }

    // For even parity: if count of 1s is odd, parity bit must be 1
    // if count is even, parity bit is 0
    return count;
}

