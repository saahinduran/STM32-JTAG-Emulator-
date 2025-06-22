/*
 * dpacc.c
 *
 *  Created on: Jun 1, 2025
 *      Author: sahin
 */
#if 0

Function Read_AHB_AP_IDR(DEBUG_PORT_BASE_ADDRESS):
    // This pseudocode assumes direct memory-mapped access to the Debug Port (DP)
    // and AHB Access Port (AP) registers.
    // Specific register addresses (offsets) will vary by ARM architecture
    // and silicon implementation, but the general flow remains similar.

    // Define known offsets for common Debug Port (DP) and AHB Access Port (AP) registers
    // These are examples; refer to ARM Architecture Reference Manual (ARM ARM)
    // or specific silicon vendor documentation for exact addresses.
    DP_SELECT_REGISTER_OFFSET = 0x8  // Example offset for DP SELECT register
    DP_RDBUFF_REGISTER_OFFSET = 0xC  // Example offset for DP RDBUFF register
    AP_CSW_REGISTER_OFFSET = 0x0     // Example offset for AP Control/Status Word register
    AP_TAR_REGISTER_OFFSET = 0x4     // Example offset for AP Transfer Address Register
    AP_DRW_REGISTER_OFFSET = 0xC     // Example offset for AP Data Read/Write register
    AP_IDR_REGISTER_OFFSET = 0xFC    // Example offset for AP Identification Register

    // Step 1: Select the AHB Access Port (AP)
    // Write to the DP SELECT register to select the desired AP.
    // The APSEL field (bits 24-31) in DP SELECT determines which AP is selected.
    // AHB-AP is usually AP #0, but verify this in documentation.
    // The APBANKSEL field (bits 4-7) is typically 0 for initial access.
    // Example: Select AP #0, bank 0
    AP_SELECT_VALUE = (0 << 24) | (0 << 4) // APSEL = 0, APBANKSEL = 0
    Write_Register(DEBUG_PORT_BASE_ADDRESS + DP_SELECT_REGISTER_OFFSET, AP_SELECT_VALUE)

    // Step 2: Ensure the AHB-AP is in a usable state (optional but good practice)
    // Read and verify the AHB-AP CSW register.
    // This often involves checking for power-up defaults or reset status.
    // For simplicity, we'll omit detailed CSW configuration here, assuming default.
    // In a real implementation, you might configure CSW for 32-bit transfers, auto-increment, etc.

    // Step 3: Write the address of the AP IDR to the AP TAR (Transfer Address Register)
    // The AP IDR address is usually an offset from the AP's base address.
    // AP IDR is typically at offset 0xFC within the AP register space.
    Write_Register(DEBUG_PORT_BASE_ADDRESS + AP_TAR_REGISTER_OFFSET, AP_IDR_REGISTER_OFFSET)

    // Step 4: Read the AP IDR by reading from the AP DRW (Data Read/Write) register
    // Reading from AP DRW after writing to AP TAR will initiate the read from the target AP register.
    IDR_VALUE = Read_Register(DEBUG_PORT_BASE_ADDRESS + AP_DRW_REGISTER_OFFSET)

    // Step 5: Read the RDBUFF register to get the actual data
    // The data from the previous AP transaction is typically buffered in DP RDBUFF.
    FINAL_IDR_VALUE = Read_Register(DEBUG_PORT_BASE_ADDRESS + DP_RDBUFF_REGISTER_OFFSET)

    Return FINAL_IDR_VALUE

// Helper function (simulated) for writing to a register
Function Write_Register(address, value):
    // In a real system, this would be a memory-mapped write operation
    // e.g., *(volatile uint32_t*)address = value;
    Print("Writing value " + Hex(value) + " to address " + Hex(address))

// Helper function (simulated) for reading from a register
Function Read_Register(address):
    // In a real system, this would be a memory-mapped read operation
    // e.g., return *(volatile uint32_t*)address;
    Simulated_Read_Value = Generate_Simulated_Read_Value(address) // Placeholder for actual read
    Print("Reading from address " + Hex(address) + ", returning " + Hex(Simulated_Read_Value))
    Return Simulated_Read_Value

// Example of how to use the function:
// Assuming your debug port base address is 0x50000000 (example)
// DEBUG_PORT_BASE = 0x50000000
// idr = Read_AHB_AP_IDR(DEBUG_PORT_BASE)
// Print("AHB-AP IDR value: " + Hex(idr))
#endif
