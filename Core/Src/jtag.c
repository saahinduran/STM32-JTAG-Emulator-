// Define your GPIO pins for JTAG
// This will depend on your specific STM32F746 board and how you've wired it.

#include "main.h"
#include <stdint.h>
#define JTAG_TCK_PORT    GPIOC
#define JTAG_TCK_PIN     GPIO_PIN_10 // Example: PA5

#define JTAG_TMS_PORT    GPIOC
#define JTAG_TMS_PIN     GPIO_PIN_9 // Example: PA6

#define JTAG_TDI_PORT    GPIOC
#define JTAG_TDI_PIN     GPIO_PIN_12 // Example: PA7

#define JTAG_TDO_PORT    GPIOC
#define JTAG_TDO_PIN     GPIO_PIN_11 // Example: PB0

#define JTAG_TRST_PORT   GPIOC      // Optional
#define JTAG_TRST_PIN    GPIO_PIN_8 // Optional

#define JTAG_SRST_PORT	 GPIOD
#define JTAG_SRST_PIN    GPIO_PIN_2 // Optional

// --- JTAG State Machine Constants (for TMS manipulation) ---
// These are standard JTAG state transitions controlled by TMS
#define JTAG_TMS_0       0
#define JTAG_TMS_1       1

#define DELAY_MS 		 1

int SPI_Transfer(uint64_t *rdData, uint64_t wrData, uint8_t bitSize);

void SPI_TMS_Transfer(uint32_t data, uint8_t bits );

typedef enum IRScanState
{
    FIRST,
    MIDDLE,
    LAST,

}IRScanStateEnum;


// JTAG States (for reference, not directly used in API functions but good to understand)
typedef enum {
    JTAG_STATE_TEST_LOGIC_RESET,
    JTAG_STATE_RUN_TEST_IDLE,
    JTAG_STATE_SELECT_DR_SCAN,
    JTAG_STATE_CAPTURE_DR,
    JTAG_STATE_SHIFT_DR,
    JTAG_STATE_EXIT1_DR,
    JTAG_STATE_PAUSE_DR,
    JTAG_STATE_EXIT2_DR,
    JTAG_STATE_UPDATE_DR,
    JTAG_STATE_SELECT_IR_SCAN,
    JTAG_STATE_CAPTURE_IR,
    JTAG_STATE_SHIFT_IR,
    JTAG_STATE_EXIT1_IR,
    JTAG_STATE_PAUSE_IR,
    JTAG_STATE_EXIT2_IR,
    JTAG_STATE_UPDATE_IR
} JTAG_State_TypeDef;


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
 * @brief Reads the state of a GPIO pin.
 * @param port GPIO_TypeDef* The GPIO port.
 * @param pin uint16_t The pin number.
 * @return GPIO_PinState The current state (GPIO_PIN_SET or GPIO_PIN_RESET).
 */
GPIO_PinState JTAG_GPIO_Read(GPIO_TypeDef* port, uint16_t pin) {
    return HAL_GPIO_ReadPin(port, pin); // Using STM32 HAL library function
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


// --- Core JTAG Primitive Functions ---

/**
 * @brief Toggles TCK high and then low, performing a single JTAG clock cycle.
 * Data is typically sampled on the rising or falling edge of TCK.
 * For simplicity, we'll assume data is set before rising edge, read after falling.
 * @param tms_state The state of TMS for this clock cycle (0 or 1).
 * @param tdi_state The state of TDI for this clock cycle (0 or 1).
 * @return The state of TDO after the clock cycle.
 */
uint8_t JTAG_ClockCycle(uint8_t tms_state, uint8_t tdi_state) {
    JTAG_GPIO_Write(JTAG_TMS_PORT, JTAG_TMS_PIN, (tms_state == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    JTAG_GPIO_Write(JTAG_TDI_PORT, JTAG_TDI_PIN, (tdi_state == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // TCK low
    JTAG_GPIO_Write(JTAG_TCK_PORT, JTAG_TCK_PIN, GPIO_PIN_RESET);
    JTAG_Delay_ns(DELAY_MS); // Adjust delay based on target device's TCK frequency requirements

    // TCK high (data is stable here)
    JTAG_GPIO_Write(JTAG_TCK_PORT, JTAG_TCK_PIN, GPIO_PIN_SET);
    JTAG_Delay_ns(DELAY_MS); // Adjust delay

    uint8_t tdo_read = (JTAG_GPIO_Read(JTAG_TDO_PORT, JTAG_TDO_PIN) == GPIO_PIN_SET) ? 1 : 0;

    return tdo_read;
}

/**
 * @brief Transitions the JTAG state machine by manipulating TMS.
 * @param path_to_state Array of TMS values (0 or 1) representing the path to the desired state.
 * @param num_steps Number of steps in the path.
 */
void JTAG_GoToState(const uint8_t* path_to_state, uint32_t num_steps) {
    for (uint32_t i = 0; i < num_steps; i++) {
        JTAG_ClockCycle(path_to_state[i], 0); // TDI doesn't matter for state transitions
    }
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
    SPI_TMS_Transfer(0x001f, 16);
    SPI_Transfer(&dummyInVal, 0, 16);
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


/**
 * @brief Shifts data into the JTAG Instruction Register (IR).
 * Assumes JTAG is in Run-Test/Idle state before call.
 * @param instruction The instruction value to shift in.
 * @param bits The number of bits in the instruction.
 * @return The data shifted out from TDO during the IR shift.
 */
uint32_t JTAG_ShiftIR(uint32_t instruction, uint8_t bits) {
    uint32_t tdo_data = 0;

    // Go to Select-DR-Scan -> Select-IR-Scan -> Capture-IR
    uint8_t path_to_capture_ir[] = {JTAG_TMS_1, JTAG_TMS_1, JTAG_TMS_0, JTAG_TMS_0};
    JTAG_GoToState(path_to_capture_ir, sizeof(path_to_capture_ir) / sizeof(path_to_capture_ir[0]));

    // Shift-IR state (TMS=0 for all but the last bit)
    for (int i = 0; i < bits; i++) {
        uint8_t tms_val = (i == bits - 1) ? JTAG_TMS_1 : JTAG_TMS_0; // TMS=1 on last bit to transition
        uint8_t tdi_val = (instruction >> i) & 0x01;

        uint8_t tdo_bit = JTAG_ClockCycle(tms_val, tdi_val);
        tdo_data |= (tdo_bit << i);
    }

    // After last bit, we are in Exit1-IR, then transition to Update-IR -> Run-Test/Idle
    // The previous JTAG_ClockCycle already set TMS=1 for the last bit, so we are in Exit1-IR.
    // Now, go to Update-IR (TMS=1) and then Run-Test/Idle (TMS=0).
    // Note: The previous call to JTAG_ClockCycle already handled the transition to Exit1-IR on the last bit of shift.
    // Now we need to transition from Exit1-IR -> Update-IR (TMS=1) -> Run-Test/Idle (TMS=0)
    JTAG_GoToState((uint8_t[]){JTAG_TMS_1, JTAG_TMS_0}, 2); // Update-IR then Run-Test/Idle

    return tdo_data;
}

/**
 * @brief Shifts data into the JTAG Instruction Register (IR).
 * Assumes JTAG is in Run-Test/Idle state before call.
 * @param instruction The instruction value to shift in.
 * @param bits The number of bits in the instruction.
 * @return The data shifted out from TDO during the IR shift.
 */
uint32_t JTAG_ShiftIRSPI(uint32_t instruction, uint8_t bits) {
    uint64_t dummyInVal = 0;
    uint32_t remaining_bits = bits;
    uint32_t read_data = 0;
    //TODO: handle bits more than 32 bits!!!

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


#if 0

    if(remaining_bits > 10)
    {
    	uint32_t next_path = 0x03;
    	uint32_t shift_instruction = instruction;

    	SPI_TMS_Transfer(next_path, 16);
    	SPI_Transfer(&dummyInVal, shift_instruction << 4, 16);

    	shift_instruction >>= 13;
		remaining_bits -= 13;

		next_path = 0;
		next_path = 0x03 << remaining_bits;

		SPI_TMS_Transfer(next_path, 16);
		SPI_Transfer(&dummyInVal, shift_instruction & ( (1 << remaining_bits) -1), 16);

    }
    else
    {
    	uint32_t path_to_capture_ir = (0x3 << (3 +bits) ) | 0x03;
		SPI_TMS_Transfer(path_to_capture_ir, 16);

		SPI_Transfer(&dummyInVal, instruction << 4, 16);
		//for(int i = 0; i < 999; i++);


		dummyInVal = (dummyInVal >> 4) & ( (1 << bits) -1);

    }


    return dummyInVal;

#endif
    return read_data;
}

/**
 * @brief Shifts data into the JTAG Data Register (DR).
 * Assumes JTAG is in Run-Test/Idle state before call.
 * @param data_out The data value to shift out.
 * @param bits The number of bits in the data register.
 * @param data_in Pointer to store the data shifted in from TDO. Can be NULL if not needed.
 */
void JTAG_ShiftDR(uint64_t data_out, uint8_t bits, uint64_t* data_in) {
    uint64_t received_data = 0;

    // Go to Select-DR-Scan -> Capture-DR
    uint8_t path_to_capture_dr[] = {JTAG_TMS_1, JTAG_TMS_0, JTAG_TMS_0};
    JTAG_GoToState(path_to_capture_dr, sizeof(path_to_capture_dr) / sizeof(path_to_capture_dr[0]));

    // Shift-DR state (TMS=0 for all but the last bit)
    for (int i = 0; i < bits; i++) {
        uint8_t tms_val = (i == bits - 1) ? JTAG_TMS_1 : JTAG_TMS_0; // TMS=1 on last bit to transition
        uint8_t tdi_val = (data_out >> i) & 0x01;

        uint8_t tdo_bit = JTAG_ClockCycle(tms_val, tdi_val);
        received_data |= ((uint64_t)tdo_bit << i);
    }

    // After last bit, we are in Exit1-DR, then transition to Update-DR -> Run-Test/Idle
    JTAG_GoToState((uint8_t[]){JTAG_TMS_1, JTAG_TMS_0}, 2); // Update-DR then Run-Test/Idle

    if (data_in != NULL) {
        *data_in = received_data;
    }
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

#if 0

	uint64_t received_data = 0;
    uint64_t data_to_shift = data_out;
    uint64_t dummyInVal = 0;
    const uint32_t shift_dr_path_len = 3;
    uint32_t remaining_bits = bits;




        // Go to Select-DR-Scan -> Select-IR-Scan -> Capture-IR
        uint32_t path_to_capture_dr = 0x01;
        SPI_TMS_Transfer(path_to_capture_dr, 16);
        SPI_Transfer(&dummyInVal, (data_to_shift << 3) & 0xFFFF, 16);

        data_out = data_out >> 13;
        received_data |= dummyInVal >> 3;

        remaining_bits -= (16 - shift_dr_path_len);
        while(remaining_bits > 16)
        {
        	int i = 1;

        	SPI_TMS_Transfer(0, 16);
        	SPI_Transfer(&dummyInVal, data_out & 0xFFFFF, 8);

        	data_out = data_out >> 16;

        	received_data |= dummyInVal << 13 * i;

        	remaining_bits -= 16;
        	i++;

        }


        uint16_t path_to_reset = 0x03 << remaining_bits -1;
        uint16_t remaining_bit_mask = (1 << remaining_bits) -1;


        SPI_TMS_Transfer(path_to_reset, 16);
        SPI_Transfer(&dummyInVal, data_out & remaining_bit_mask, 16);


        received_data |= (dummyInVal & remaining_bit_mask) << (bits - remaining_bits);

        *data_in = received_data;

#endif
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

uint32_t JTAG_ReadIDCODE_GPIO(void) {
    uint32_t idcode = 0;
    // Assume JTAG is in Run-Test/Idle

    // 1. Shift in the IDCODE instruction (typically 0x01 for ARM/standard JTAG devices)
    // The instruction length is device-specific. Common is 5-bit for ARM.
    // You need to know the target device's IR length.
    uint8_t ir_length = 4; // Example for ARM
    JTAG_ShiftIR(0xE, ir_length); // IDCODE instruction
    //JTAG_ShiftIRSPI(0xE, ir_length);

    // 2. Shift out the 32-bit IDCODE data register
    uint64_t read_idcode = 0;
    JTAG_ShiftDR(0x00000000, 32, &read_idcode); // TDI doesn't matter, shifting in 0s
    //JTAG_ShiftDR_SPI(0x00000000, 32, &read_idcode); // TDI doesn't matter, shifting in 0s

    idcode = (uint32_t)read_idcode;
    return idcode;
}



void JTAG_ByPassTest(void) {

    uint32_t testPatternIn = 0x5a5a, testPatternOut = 0;
    // Assume JTAG is in Run-Test/Idle

    // 1. Shift in the IDCODE instruction (typically 0x01 for ARM/standard JTAG devices)
    // The instruction length is device-specific. Common is 5-bit for ARM.
    // You need to know the target device's IR length.
    uint8_t ir_length = 4; // Example for ARM
    //JTAG_ShiftIR(0xF, ir_length); // IDCODE instruction

    JTAG_ShiftDR(testPatternIn, 16, &testPatternOut);


    return ;
}


/**
 * @brief Writes to a specific JTAG Data Register (e.g., a specific memory location via boundary scan).
 * This is highly dependent on the target device's JTAG implementation and instructions.
 * @param instruction The instruction to select the DR (e.g., BYPASS, EXTEST, etc.)
 * @param ir_bits The length of the instruction register.
 * @param data_to_write The data to write to the DR.
 * @param dr_bits The length of the data register.
 * @param data_read Optional pointer to store data read back (if a read-back is expected).
 */
void JTAG_WriteDR(uint32_t instruction, uint8_t ir_bits, uint64_t data_to_write, uint8_t dr_bits, uint64_t* data_read) {
    JTAG_ShiftIR(instruction, ir_bits);
    JTAG_ShiftDR(data_to_write, dr_bits, data_read);
}

// Add more functions as needed:
// JTAG_SetBypass()
// JTAG_ScanChain() (for multiple devices in the chain)
// Functions for specific boundary scan operations (EXTEST, INTEST, SAMPLE/PRELOAD)
// Functions for accessing Debug Access Port (DAP) or other specific debug features.



typedef enum {
    JTAG_STATUS_OK = 0,
    JTAG_STATUS_ERROR_NO_RESPONSE,
    JTAG_STATUS_ERROR_IDCODE_MISMATCH,
    JTAG_STATUS_ERROR_INIT_FAILED
} JTAG_Status_TypeDef;

JTAG_Status_TypeDef JTAG_ConnectionTest(uint32_t expected_idcode, uint8_t ir_length) {
    uint32_t read_idcode = 0;

    // 1. Initialize JTAG (if not already done)
    // This is crucial. If JTAG_Init() has already been called and handled,
    // you might skip this or make sure it's idempotent.
    // For a standalone test, it's good to call it here.
    // JTAG_Init(); // Uncomment if you want to initialize within the test function

    // 2. Reset the JTAG state machine
    JTAG_Reset();

    // 3. Shift in the IDCODE instruction (typically 0x01 for ARM/standard JTAG devices)
    // The instruction length (ir_length) is crucial and device-specific.
    // Example: For ARM, the IDCODE instruction is usually 0x01 and IR length is 5.
    JTAG_ShiftIR(0x01, ir_length);

    // 4. Shift out the 32-bit IDCODE data register
    uint64_t temp_read_idcode_64 = 0;
    JTAG_ShiftDR(0x00000000, 32, &temp_read_idcode_64); // TDI doesn't matter for read, send 0s
    read_idcode = (uint32_t)temp_read_idcode_64;

    // 5. Check if the read IDCODE is plausible (not all zeros or all ones)
    // This is a basic sanity check before comparing to expected_idcode.
    // If TDO is stuck low or high, this can indicate a wiring issue or power problem.
    if (read_idcode == 0x00000000 || read_idcode == 0xFFFFFFFF) {
        // TDO line might be stuck, or no valid JTAG response
        return JTAG_STATUS_ERROR_NO_RESPONSE;
    }

    // 6. Compare with the expected IDCODE
    if (read_idcode == expected_idcode) {
        return JTAG_STATUS_OK; // Connection successful and IDCODE matches
    } else {
        // IDCODE mismatch
        // You might want to print or log the read_idcode here for debugging
        // printf("JTAG Connection Test Failed: Expected IDCODE 0x%08lX, Read 0x%08lX\n", expected_idcode, read_idcode);
        return JTAG_STATUS_ERROR_IDCODE_MISMATCH;
    }
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
{
	//TODO: handle IRlen try more than 32 bit.
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
