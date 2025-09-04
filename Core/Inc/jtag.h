/*
 * jtag.h
 *
 *  Created on: Jul 5, 2025
 *      Author: sahin
 */

#ifndef INC_JTAG_H_
#define INC_JTAG_H_



// --- JTAG State Machine Constants (for TMS manipulation) ---
// These are standard JTAG state transitions controlled by TMS
#define JTAG_TMS_0       0
#define JTAG_TMS_1       1

#define DELAY_MS 		 1

// Define your GPIO pins for JTAG
// This will depend on your specific STM32F746 board and how you've wired it.

#include "main.h"
#include <stdint.h>

#include "jtag.h"

// --- Low-Level GPIO Control Functions (STM32 HAL/LL equivalents) ---

/**
 * @brief Sets the state of a GPIO pin.
 * @param port GPIO_TypeDef* The GPIO port (e.g., GPIOA, GPIOB).
 * @param pin uint16_t The pin number (e.g., GPIO_PIN_5).
 * @param state GPIO_PinState The desired state (GPIO_PIN_SET or GPIO_PIN_RESET).
 */
void JTAG_GPIO_Write(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);

void JTAG_Reset_Target(void);

/**
 * @brief Reads the state of a GPIO pin.
 * @param port GPIO_TypeDef* The GPIO port.
 * @param pin uint16_t The pin number.
 * @return GPIO_PinState The current state (GPIO_PIN_SET or GPIO_PIN_RESET).
 */
GPIO_PinState JTAG_GPIO_Read(GPIO_TypeDef* port, uint16_t pin);
/**
 * @brief Introduces a small delay. Crucial for JTAG timing.
 * This needs to be carefully tuned. A simple loop or a timer-based delay.
 */
void JTAG_Delay_ns(uint32_t nanoseconds);

// --- Core JTAG Primitive Functions ---

/**
 * @brief Toggles TCK high and then low, performing a single JTAG clock cycle.
 * Data is typically sampled on the rising or falling edge of TCK.
 * For simplicity, we'll assume data is set before rising edge, read after falling.
 * @param tms_state The state of TMS for this clock cycle (0 or 1).
 * @param tdi_state The state of TDI for this clock cycle (0 or 1).
 * @return The state of TDO after the clock cycle.
 */
uint8_t JTAG_ClockCycle(uint8_t tms_state, uint8_t tdi_state);

/**
 * @brief Transitions the JTAG state machine by manipulating TMS.
 * @param path_to_state Array of TMS values (0 or 1) representing the path to the desired state.
 * @param num_steps Number of steps in the path.
 */
void JTAG_GoToState(const uint8_t* path_to_state, uint32_t num_steps);


// --- JTAG API Wrapper Functions ---

/**
 * @brief Initializes JTAG GPIOs.
 * Call this once at the start of your program.
 */
void JTAG_Init(void);

/**
 * @brief Resets the JTAG state machine to Test-Logic-Reset.
 * Can use TRST if available, or sequence TMS.
 */
void JTAG_Reset(void);
/**
 * @brief Shifts data into the JTAG Instruction Register (IR).
 * Assumes JTAG is in Run-Test/Idle state before call.
 * @param instruction The instruction value to shift in.
 * @param bits The number of bits in the instruction.
 * @return The data shifted out from TDO during the IR shift.
 */
uint32_t JTAG_ShiftIR(uint32_t instruction, uint8_t bits);

/**
 * @brief Shifts data into the JTAG Instruction Register (IR).
 * Assumes JTAG is in Run-Test/Idle state before call.
 * @param instruction The instruction value to shift in.
 * @param bits The number of bits in the instruction.
 * @return The data shifted out from TDO during the IR shift.
 */
uint64_t JTAG_ShiftIRSPI(uint64_t instruction, uint8_t bits);

JTAG_ShiftIRSPI_BypassOthers(uint64_t instruction, uint8_t bits,
		uint32_t left, uint32_t right, uint64_t *instruction_return);

/**
 * @brief Shifts data into the JTAG Data Register (DR).
 * Assumes JTAG is in Run-Test/Idle state before call.
 * @param data_out The data value to shift out.
 * @param bits The number of bits in the data register.
 * @param data_in Pointer to store the data shifted in from TDO. Can be NULL if not needed.
 */
void JTAG_ShiftDR_SPI(uint64_t data_out, uint8_t bits, uint64_t* data_in);

int JTAG_ShiftDR_SPI_ByPassed(uint64_t data_out, uint8_t bits, uint64_t* data_in, uint32_t left, uint32_t right);

// --- Example High-Level JTAG Operations ---

/**
 * @brief Reads a specific JTAG IDCODE from the target device.
 * @return The IDCODE value. Returns 0 if read fails or JTAG not initialized.
 */
uint32_t JTAG_ReadIDCODE(void);


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
uint8_t JTAG_MeasureIRLength(void);



#endif /* INC_JTAG_H_ */
