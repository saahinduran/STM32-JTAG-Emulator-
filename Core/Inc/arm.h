/*
 * arm.h
 *
 *  Created on: Jul 5, 2025
 *      Author: sahin
 */

#ifndef INC_ARM_H_
#define INC_ARM_H_

#define MAX_JTAG_DEVICES 4




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


#endif /* INC_ARM_H_ */
