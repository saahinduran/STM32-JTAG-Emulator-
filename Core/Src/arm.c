#include <stdint.h>
#include <stdio.h> // For printf, for debugging

#define MAX_JTAG_DEVICES 4
// Assuming these are available from your JTAG bit-banging implementation
// #include "jtag_api.h"
 extern void JTAG_Init(void);
 extern void JTAG_Reset(void);
 extern void JTAG_ShiftIR(uint32_t instruction, uint8_t bits);
 extern void JTAG_ShiftDR(uint64_t data_out, uint8_t bits, uint64_t* data_in);
 extern void JTAG_GoToState(const uint8_t* path_to_state, uint32_t num_steps);

 void JTAG_ShiftDR_SPI(uint64_t data_out, uint8_t bits, uint64_t* data_in);
 uint32_t JTAG_ShiftIRSPI(uint32_t instruction, uint8_t bits);
// extern void JTAG_Chain_Init(uint8_t num_devices);
// extern void JTAG_Chain_ConfigureDevice(uint8_t index, JTAG_DeviceType_TypeDef type, uint8_t default_dr_length);
// extern void JTAG_SetBypassState(uint8_t index, uint8_t bypassed); // Only if multi-device chain is used

// Also, ensure your delay function is available, e.g.:
// #define HAL_Delay(ms)  // Your actual delay implementation


// --- Configuration for your Cortex-M3 ---
#define CM3_JTAG_IR_LENGTH 4       // Common for ARM JTAG-DPs, verify for your specific M3
#define CM3_EXPECTED_IDCODE 0x4BA00477 // Example: ARM Cortex-M3 (verify from datasheet)

// JTAG-DP Instructions
#define CM3_DP_IDCODE   0x0E // Read Debug Port IDCODE
#define CM3_DP_ABORT    0x0A // Abort Debug Port operations
#define CM3_DP_CTRLSTAT 0x09 // Access Debug Port Control/Status Register
#define CM3_DP_SELECT   0x0A // Select Access Port and register bank
#define CM3_DP_RDBUF    0x0B // Read the Debug Port Read Buffer (result of last AP read)
#define CM3_DP_BYPASS   0x0F // JTAG-DP Bypass instruction

// AHB-AP Register Addresses (these are the 'AP_ADDR' values within the selected AP)
// These are not JTAG IR instructions, but addresses within the AP's register space.
#define CM3_AP_CSW_ADDR 0x00 // Control/Status Word
#define CM3_AP_TAR_ADDR 0x04 // Transfer Address Register
#define CM3_AP_DRW_ADDR 0x0C // Data Read/Write Register
#define CM3_AP_IDR_ADDR 0xFC // AP ID Register


// AHB-AP CSW bit definitions (same as before)
#define CM3_AP_CSW_SIZE_8BIT  (0x00000000UL << 0)
#define CM3_AP_CSW_SIZE_16BIT (0x00000001UL << 0)
#define CM3_AP_CSW_SIZE_32BIT (0x00000002UL << 0) // Default word access
#define CM3_AP_CSW_ADDRINC_OFF  (0x00000000UL << 4) // No address increment
#define CM3_AP_CSW_ADDRINC_SINGLE (0x00000001UL << 4) // Increment by transfer size
#define CM3_AP_CSW_ADDRINC_PACKED (0x00000002UL << 4) // Packed transfers (advanced)
#define CM3_AP_CSW_HPROT_DATA_RW (0x00000000UL << 8) // Data read/write (typical)
#define CM3_AP_CSW_TRN_NORMAL   (0x00000000UL << 12) // Normal transfer
#define CM3_AP_CSW_DBGSTAT      (0x00000000UL << 16) // Debug Status (read-only)
#define CM3_AP_CSW_MSTRTYPE_AP  (0x00000000UL << 28) // Master Type: AP (default)
#define CM3_AP_CSW_PROT         (0x00000000UL)     // Protection bits (depends on system)
#define CM3_AP_CSW_DEFAULT      (CM3_AP_CSW_SIZE_32BIT | CM3_AP_CSW_ADDRINC_SINGLE | CM3_AP_CSW_MSTRTYPE_AP)

 void JTAG_ShiftDR_SPI(uint64_t data_out, uint8_t bits, uint64_t* data_in);


// --- Status/Error Codes for Debugging ---
typedef enum {
    CM3_DBG_OK = 0,
    CM3_DBG_ERROR_JTAG_INIT_FAILED,
    CM3_DBG_ERROR_IDCODE_MISMATCH,
    CM3_DBG_ERROR_DP_STUCK,
    CM3_DBG_ERROR_AP_UNAVAILABLE,
    CM3_DBG_ERROR_TIMEOUT,
    CM3_DBG_ERROR_INVALID_ARG,
    CM3_DBG_ERROR_READ_FAILED,
    CM3_DBG_ERROR_WRITE_FAILED
} CM3_DebugStatus_TypeDef;

// --- Internal Helper Function (Reads/Writes to DP or AP) ---

/**
 * @brief Performs a JTAG-DP or AHB-AP register access.
 * This is the fundamental function for interacting with the Cortex-M3.
 * It handles the JTAG instruction and data register shifts.
 *
 * @param dp_instruction The JTAG IR instruction for the DP (e.g., CM3_DP_SELECT, CM3_DP_CTRLSTAT).
 * @param data_out The 32-bit data to write. Ignored for reads.
 * @param data_in Pointer to store the 32-bit data read. Can be NULL for writes.
 * @param is_ap_access Flag: 1 if this is an AHB-AP access (requires DP_SELECT then DP_RDBUF/DRW logic), 0 for direct DP access.
 * @param ap_addr If is_ap_access is 1, this is the AHB-AP register address (e.g., CM3_AP_TAR_ADDR).
 * @return CM3_DBG_OK on success, error code otherwise.
 */


uint32_t read_identification(void)
{
	uint32_t retval = 0;
	uint64_t tempWriteVal = 0;
	uint64_t data_in_temp;

	//tempWriteVal |= read & 0x1;

	tempWriteVal |= (0b10 & 0x3) << 1;

	// Direct DP access (e.g., CTRLSTAT, IDCODE, ABORT)
	JTAG_ShiftIR(0b1010, CM3_JTAG_IR_LENGTH);

	JTAG_ShiftDR(tempWriteVal, 35, &data_in_temp);


}

uint32_t DPACC (uint32_t data_out, uint32_t *data_in, uint32_t dp_reg ,uint32_t read)
{
	uint32_t retval = 0;
	uint64_t tempWriteVal = 0;
	uint64_t data_in_temp;

	tempWriteVal |= read & 0x1;

	tempWriteVal |= (dp_reg & 0x3) << 1;

	tempWriteVal |= (uint64_t)data_out << 3;

	// Direct DP access (e.g., CTRLSTAT, IDCODE, ABORT)
	JTAG_ShiftIRSPI(0b1010, CM3_JTAG_IR_LENGTH);

	JTAG_ShiftDR_SPI(tempWriteVal, 35, &data_in_temp);


	if(read)
	{

		//JTAG_ShiftDR(data_out, 35, &data_in_temp);
		JTAG_ShiftDR_SPI(data_out, 35, &data_in_temp);

		if( (data_in_temp & 0x7) == 0b010) // 0b010 = OK/FAULT
		{
			retval = 1;
			*data_in = ( (uint64_t)data_in_temp >> 3) & 0xFFFFFFFF;
		}
		else  // 0b001 = WAIT
		{
			retval = 0;
		}


	}

    return retval;
}

uint32_t APACC (uint32_t data_out, uint32_t *data_in, uint32_t dp_reg ,uint32_t read)
{
	uint32_t retval = 0;
	uint64_t tempWriteVal = 0;
	uint64_t data_in_temp;

	tempWriteVal |= read & 0x1;

	tempWriteVal |= (dp_reg & 0x3) << 1;

	tempWriteVal |= (uint64_t)data_out << 3;

	// Direct DP access (e.g., CTRLSTAT, IDCODE, ABORT)
	JTAG_ShiftIRSPI(0b1011, CM3_JTAG_IR_LENGTH);

	JTAG_ShiftDR_SPI(tempWriteVal, 35, &data_in_temp);


	if(read)
	{

		//JTAG_ShiftDR(data_out, 35, &data_in_temp);
		JTAG_ShiftDR_SPI(data_out, 35, &data_in_temp);

		if( (data_in_temp & 0x7) == 0b010) // 0b010 = OK/FAULT
		{
			retval = 1;
			*data_in = ( (uint64_t)data_in_temp >> 3) & 0xFFFFFFFF;
		}
		else  // 0b001 = WAIT
		{
			retval = 0;
		}


	}

    return retval;
}

// --- Cortex-M3 Debug Abstraction Layer Functions ---

/**
 * @brief Initializes the JTAG connection and verifies the Cortex-M3 processor.
 * Assumes JTAG_Init() and JTAG_Chain_Init() have been called.
 * Configures the JTAG chain for a single Cortex-M3 device.
 * This version integrates the IDCODE read directly.
 * @return CM3_DBG_OK on success, error code otherwise.
 */
CM3_DebugStatus_TypeDef CM3_Debug_Init(void) {
    uint32_t read_idcode = 0;
    uint64_t rx_array[MAX_JTAG_DEVICES] = {0};
    uint64_t tx_array[MAX_JTAG_DEVICES] = {0}; // Dummy data for shifts

    // 1. Initialize and configure the JTAG chain (if not already done globally)
    // Assuming a single Cortex-M3 device is the ONLY device in the chain.
    //JTAG_Chain_Init(1); // One device in the chain
    //JTAG_Chain_ConfigureDevice(0, JTAG_DEV_TYPE_ARM_CORTEX_M, 32); // Default DR length for DP registers is 32

    // 2. Perform JTAG reset to bring the state machine to a known state
    JTAG_Reset();


    read_idcode = JTAG_ReadIDCODE(); // IDCODE is 32-bit for device 0

    // 4. Verify the read IDCODE
    if (read_idcode == 0x00000000 || read_idcode == 0xFFFFFFFF) {
        printf("JTAG Init Failed: No response from target (IDCODE 0x%08lX indicates stuck TDO/no activity).\n", read_idcode);
        return CM3_DBG_ERROR_JTAG_INIT_FAILED;
    }

    if (read_idcode != CM3_EXPECTED_IDCODE) {
        printf("JTAG Init Failed: IDCODE mismatch. Expected 0x%08lX, Read 0x%08lX\n", CM3_EXPECTED_IDCODE, read_idcode);
        return CM3_DBG_ERROR_IDCODE_MISMATCH;
    }
    printf("Cortex-M3 IDCODE (0x%08lX) verified successfully.\n", read_idcode);

    // 5. Power up the Debug Port (DP) and Access Port (AP)
    // Send DP ABORT with STKCMPST, STKERR, WDERR, ORUNERR set to clear sticky flags.
    CM3_Debug_AccessRegister(CM3_DP_ABORT, 0x1E, NULL, 0, 0); // ABORT with sticky bits set

    // CTRL/STAT register to power up (CSYSPWRUPREQ, CDBGPWRUPREQ)
    uint32_t ctrl_stat_val = (1UL << 28) | (1UL << 29); // CSYSPWRUPREQ | CDBGPWRUPREQ
    CM3_Debug_AccessRegister(CM3_DP_CTRLSTAT, ctrl_stat_val, NULL, 0, 0);

    // Wait for power up acknowledge (CSYSPWRUPACK, CDBGPWRUPACK)
    uint32_t read_ctrl_stat = 0;
    int timeout = 1000;
    do {
        CM3_Debug_AccessRegister(CM3_DP_CTRLSTAT, 0, &read_ctrl_stat, 0, 0);
        if (((read_ctrl_stat >> 31) & 0x01) && ((read_ctrl_stat >> 30) & 0x01)) {
            break; // Power-up acknowledged
        }
        // Replace HAL_Delay with your actual delay function (e.g., my_delay_us(1000))
        HAL_Delay(1); // Small delay, or your JTAG_Delay_ns
    } while (timeout-- > 0);

    if (timeout <= 0) {
        printf("Cortex-M3 Debug Power-up Timeout!\n");
        return CM3_DBG_ERROR_DP_STUCK;
    }

    // 6. Initialize AHB-AP CSW register (e.g., 32-bit access, auto-increment)
    // This needs to be done via CM3_DP_SELECT then a write operation via DP_CTRLSTAT.
    CM3_Debug_AccessRegister(CM3_DP_CTRLSTAT, CM3_AP_CSW_DEFAULT, NULL, 1, CM3_AP_CSW_ADDR);
    printf("Cortex-M3 Debug Init: SUCCESS!\n");
    return CM3_DBG_OK;
}

/**
 * @brief Reads a 32-bit word from the Cortex-M3's memory.
 * @param address The memory address to read from.
 * @param data_out Pointer to store the read 32-bit data.
 * @return CM3_DBG_OK on success, error code otherwise.
 */
CM3_DebugStatus_TypeDef CM3_Debug_ReadMemory32(uint32_t address, uint32_t *data_out) {
    if (data_out == NULL) return CM3_DBG_ERROR_INVALID_ARG;

    // 1. Write address to AHB-AP TAR
    CM3_DebugStatus_TypeDef status = CM3_Debug_AccessRegister(CM3_DP_CTRLSTAT, address, NULL, 1, CM3_AP_TAR_ADDR);
    if (status != CM3_DBG_OK) return status;

    // 2. Read from AHB-AP DRW (data will be available in RDBUF on next DP read)
    // The previous access to TAR will have placed its *read* result in RDBUF.
    // So, we immediately initiate a read from DRW, and the result of THIS read will be in RDBUF on the *next* cycle.
    status = CM3_Debug_AccessRegister(CM3_DP_CTRLSTAT, 0, data_out, 1, CM3_AP_DRW_ADDR);
    if (status != CM3_DBG_OK) return status;

    return CM3_DBG_OK;
}

/**
 * @brief Writes a 32-bit word to the Cortex-M3's memory.
 * @param address The memory address to write to.
 * @param data_in The 32-bit data to write.
 * @return CM3_DBG_OK on success, error code otherwise.
 */
CM3_DebugStatus_TypeDef CM3_Debug_WriteMemory32(uint32_t address, uint32_t data_in) {
    // 1. Write address to AHB-AP TAR
    CM3_DebugStatus_TypeDef status = CM3_Debug_AccessRegister(CM3_DP_CTRLSTAT, address, NULL, 1, CM3_AP_TAR_ADDR);
    if (status != CM3_DBG_OK) return status;

    // 2. Write data to AHB-AP DRW
    status = CM3_Debug_AccessRegister(CM3_DP_CTRLSTAT, data_in, NULL, 1, CM3_AP_DRW_ADDR);
    if (status != CM3_DBG_OK) return status;

    return CM3_DBG_OK;
}

// ... (rest of the CM3_Debug functions such as CM3_Debug_Halt, CM3_Debug_Run,
// CM3_Debug_Reset, CM3_Debug_ReadCoreRegister, CM3_Debug_WriteCoreRegister
// remain the same, as they call CM3_Debug_AccessRegister)
