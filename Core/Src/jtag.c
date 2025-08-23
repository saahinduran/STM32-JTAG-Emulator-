// Define your GPIO pins for JTAG
// This will depend on your specific STM32F746 board and how you've wired it.

#include "main.h"
#include <stdint.h>
#include "jtag.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

int SPI_Transfer(uint64_t *rdData, uint64_t wrData, uint8_t bitSize);

void SPI_TMS_Transfer(uint64_t data, uint8_t bits );


// --- Low-Level GPIO Control Functions (STM32 HAL/LL equivalents) ---

/**
 * @brief Sets the state of a GPIO pin.
 * @param port GPIO_TypeDef* The GPIO port (e.g., GPIOA, GPIOB).
 * @param pin uint16_t The pin number (e.g., GPIO_PIN_5).
 * @param state GPIO_PinState The desired state (GPIO_PIN_SET or GPIO_PIN_RESET).
 */
void JTAG_GPIO_Write(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state) {
    HAL_GPIO_WritePin(port, pin, state); // Using STM32 HAL library function
}

void JTAG_Reset_Target(void)
{
	HAL_GPIO_WritePin(JTAG_SRST_PORT, JTAG_SRST_PIN, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(JTAG_SRST_PORT, JTAG_SRST_PIN, GPIO_PIN_SET);

}


/**
 * @brief Introduces a small delay. Crucial for JTAG timing.
 * This needs to be carefully tuned. A simple loop or a timer-based delay.
 */
void JTAG_Delay_ns(uint32_t nanoseconds) {
    // Implement a precise delay here.
    // For bit-banging, even a few clock cycles can matter.
    // Consider using a DWT (Data Watchpoint and Trace) cycle counter for very precise delays,
    // or a TIM (Timer) in one-shot mode. A simple NOP loop might be too slow or imprecise.
    // Example (very rough):
    //for (volatile uint32_t i = 0; i < (nanoseconds / 10); i++); // Adjust divisor based on CPU freq
	HAL_Delay(nanoseconds);
}


// --- JTAG API Wrapper Functions ---

/**
 * @brief Initializes JTAG GPIOs.
 * Call this once at the start of your program.
 */
void JTAG_Init(void) {
	uint32_t dummyInVal;

    JTAG_Reset();

    // Go to run-test/idle state
    SPI_TMS_Transfer((uint64_t)0x001f, (uint8_t)16);
    SPI_Transfer(&dummyInVal, 0, (uint8_t)16);
}


/**
 * @brief Resets the JTAG state machine to Test-Logic-Reset.
 * Can use TRST if available, or sequence TMS.
 */
void JTAG_Reset(void) {
    #ifdef JTAG_TRST_PORT
    JTAG_GPIO_Write(JTAG_TRST_PORT, JTAG_TRST_PIN, GPIO_PIN_RESET); // Assert TRST
    JTAG_Delay_ns(DELAY_MS); // Hold for a short period
    JTAG_GPIO_Write(JTAG_TRST_PORT, JTAG_TRST_PIN, GPIO_PIN_SET);   // De-assert TRST
    JTAG_Delay_ns(DELAY_MS);
    #else
    // Go to run-test/idle state
    SPI_TMS_Transfer(0x001f, 16);
    SPI_Transfer(&dummyInVal, 0, 16);
	#endif
}


int JTAG_ShiftIRSPI_BypassOthers(uint64_t instruction, uint8_t bits,
		uint32_t left, uint32_t right, uint64_t *instruction_return)
{

	uint64_t totalBits = left + right + bits;

	if(totalBits > 64)
	{
		return -1;
	}

	uint64_t numToWrite = 0;
	uint64_t rightVal = (1 << right) - 1;
	uint64_t midVal = (1 << (right + bits) ) -1;

	uint64_t leftVal = ( (1 << totalBits) -1)  & ~(midVal);


	numToWrite |= rightVal | leftVal;

	numToWrite = numToWrite | instruction << right;

	*instruction_return = JTAG_ShiftIRSPI(numToWrite, totalBits);

	return 0;



}
/**
 * @brief Shifts data into the JTAG Instruction Register (IR).
 * Assumes JTAG is in Run-Test/Idle state before call.
 * @param instruction The instruction value to shift in.
 * @param bits The number of bits in the instruction.
 * @return The data shifted out from TDO during the IR shift.
 */
uint64_t JTAG_ShiftIRSPI(uint64_t instruction, uint8_t bits) {
    uint64_t dummyInVal = 0;
    uint32_t remaining_bits = bits;
    uint64_t read_data = 0;
    //TODO: handle bits more than 64 bits!!!

    /* move to shift ir */
    uint16_t next_path = 0x3000;

	SPI_TMS_Transfer(next_path, 16);
	SPI_Transfer(&dummyInVal, 0, 16);


	int i = 0;

	while(remaining_bits)
	{
		uint16_t tms_bit_pos = remaining_bits -1;

		uint16_t tms_seq = 0x03 << tms_bit_pos;
		SPI_TMS_Transfer(tms_seq, 16);
		SPI_Transfer(&dummyInVal, instruction, 16);

		if(remaining_bits > 16)
		{
			remaining_bits -= 16;
			instruction >>= 16;
			read_data |= (dummyInVal << (16 *i) );
			i++;
		}
		else if(16 == remaining_bits)
		{

			read_data |= (dummyInVal & ( (1 << remaining_bits) -1) ) << (16 *i);
			tms_seq = 0x01;
			SPI_TMS_Transfer(tms_seq, 16);
			SPI_Transfer(&dummyInVal, instruction, 16);

			remaining_bits = 0;
		}
		else if(15 == remaining_bits)
		{
			read_data |= (dummyInVal & ( (1 << remaining_bits) -1) ) << (16 *i);

			tms_seq = 0x0;
			SPI_TMS_Transfer(tms_seq, 16);
			SPI_Transfer(&dummyInVal, 0, 16);

			remaining_bits = 0;

		}
		else
		{
			read_data |= (dummyInVal & ( (1 << remaining_bits) -1) ) << (16 *i);
			remaining_bits = 0;
		}


	}

    return read_data;
}




int JTAG_ShiftDR_SPI_ByPassed(uint64_t data_out, uint8_t bits, uint64_t* data_in,
		uint32_t left, uint32_t right)
{
	uint8_t new_bits_size = bits + MAX(left,right);

	if(new_bits_size > 64)
	{
		return -1;
	}

	uint8_t input_shift_amt = new_bits_size - left - bits;

	uint8_t out_shift_amt = right;

	uint64_t data_out_new = data_out << input_shift_amt;

	uint64_t data_in_new = 0;

	JTAG_ShiftDR_SPI(data_out_new, new_bits_size, &data_in_new);

	data_in_new = (data_in_new >> out_shift_amt);

	*data_in = data_in_new;

	return 0;

}


/**
 * @brief Shifts data into the JTAG Data Register (DR).
 * Assumes JTAG is in Run-Test/Idle state before call.
 * @param data_out The data value to shift out.
 * @param bits The number of bits in the data register.
 * @param data_in Pointer to store the data shifted in from TDO. Can be NULL if not needed.
 */
void JTAG_ShiftDR_SPI(uint64_t data_out, uint8_t bits, uint64_t* data_in) {

		uint64_t dummyInVal = 0;
	    uint32_t remaining_bits = bits;
	    uint64_t read_data = 0;
	    //TODO: handle bits more than 64 bits!!!

	    /* move to shift dr */
	    uint16_t next_path = 0x2000;

		SPI_TMS_Transfer(next_path, 16);
		SPI_Transfer(&dummyInVal, 0, 16);


		int i = 0;

		while(remaining_bits)
		{
			uint16_t tms_bit_pos = remaining_bits -1;

			uint16_t tms_seq = 0x03 << tms_bit_pos;
			SPI_TMS_Transfer(tms_seq, 16);
			SPI_Transfer(&dummyInVal, data_out, 16);

			if(remaining_bits > 16)
			{
				remaining_bits -= 16;
				data_out >>= 16;
				read_data |= (dummyInVal << (16 *i) );
				i++;
			}
			else if(16 == remaining_bits)
			{

				read_data |= (dummyInVal & ( (1 << remaining_bits) -1) ) << (16 *i);
				tms_seq = 0x01;
				SPI_TMS_Transfer(tms_seq, 16);
				SPI_Transfer(&dummyInVal, data_out, 16);

				remaining_bits = 0;
			}
			else if(15 == remaining_bits)
			{
				read_data |= (dummyInVal & ( (1 << remaining_bits) -1) ) << (16 *i);

				tms_seq = 0x0;
				SPI_TMS_Transfer(tms_seq, 16);
				SPI_Transfer(&dummyInVal, 0, 16);

				remaining_bits = 0;

			}
			else
			{
				read_data |= (dummyInVal & ( (1 << remaining_bits) -1) ) << (16 *i);
				remaining_bits = 0;
			}


		}

        *data_in = read_data;
        return;

}

// --- Example High-Level JTAG Operations ---

/**
 * @brief Reads a specific JTAG IDCODE from the target device.
 * @return The IDCODE value. Returns 0 if read fails or JTAG not initialized.
 */
uint32_t JTAG_ReadIDCODE(void) {
    uint32_t idcode = 0;
    // Assume JTAG is in Run-Test/Idle

    // 1. Shift in the IDCODE instruction (typically 0x01 for ARM/standard JTAG devices)
    // The instruction length is device-specific. Common is 5-bit for ARM.
    // You need to know the target device's IR length.
    uint8_t ir_length = 4; // Example for ARM
    JTAG_ShiftIRSPI(0xE, ir_length);
    //JTAG_ShiftIRSPI(0xF, ir_length);

    // 2. Shift out the 32-bit IDCODE data register
    uint64_t read_idcode = 0;
    //JTAG_ShiftDR(0x00000000, 32, &read_idcode); // TDI doesn't matter, shifting in 0s
    JTAG_ShiftDR_SPI(0x80000001, 32, &read_idcode); // TDI doesn't matter, shifting in 0s

    idcode = (uint32_t)read_idcode;
    return idcode;
}


/**
 * @brief Measures the Instruction Register (IR) length of a JTAG-compliant device.
 * This function works by shifting in a known pattern using the BYPASS instruction
 * (often a single-bit instruction of '1' followed by '0's) and observing TDO.
 * It assumes the device is in the Run-Test/Idle state before the call.
 * @return The measured IR length, or 0 if measurement fails or an unexpected TDO pattern is observed.
 * A return value of 0 might indicate an issue with JTAG communication or a non-standard device.
 */
/**
 * @brief Measures the Instruction Register (IR) length of a JTAG-compliant device.
 * This function works by shifting in a known pattern using the BYPASS instruction
 * (often a single-bit instruction of '1' followed by '0's) and observing TDO.
 * It assumes the device is in the Run-Test/Idle state before the call.
 * @return The measured IR length, or 0 if measurement fails or an unexpected TDO pattern is observed.
 * A return value of 0 might indicate an issue with JTAG communication or a non-standard device.
 */
uint8_t JTAG_MeasureIRLength(void)
{//TODO: handle IRlen try more than 32 bit.
	uint32_t return_bits;
	int ir_len = 1;
	int i;
	const uint32_t bit_pattern = 0xDEADBEEF;
	uint32_t try_bit_cnt = 32;

	return_bits = JTAG_ShiftIRSPI(bit_pattern, try_bit_cnt);

	for(i = 0; i < try_bit_cnt; i++)
	{
		uint32_t mask = (1 << try_bit_cnt) - 1;
		if( (return_bits & mask) == (bit_pattern & mask) )
		{
			ir_len = i;
			break;
		}
		else
		{
			return_bits >>= 1;
			try_bit_cnt--;

		}
	}
    return ir_len;
}
