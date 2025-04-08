#include "pcm1865.h"
#include "stdio.h" // For printf in status function

/* USER CODE BEGIN Includes */
// Add any other necessary includes for your project if needed
/* USER CODE END Includes */

// --- Private Defines ---
#define I2C_TIMEOUT_MS      100
#define PCM1865_REG_COUNT   128

// Define configuration constants used during initialization
// These could be moved to the header or made parameters if more flexibility is needed
#define RESET_CMD           0xFE // Command to trigger reset via register 0x00
#define ADC1L_INPUT_SRC     0x41 // VIN1L -> ADC1_L
#define ADC1R_INPUT_SRC     0x41 // VIN1R -> ADC1_R (Matching L/R naming convention of PCM)
#define ADC2L_INPUT_SRC     0x42 // VIN2L -> ADC2_L
#define ADC2R_INPUT_SRC     0x42 // VIN2R -> ADC2_R
#define TDM_OFFSET_DEV0     0    // Device 0 starts at TDM slot 0 (0*32 bits offset)
#define TDM_OFFSET_DEV1     128  // Device 1 starts at TDM slot 4 (4*32 bits offset = 128 bits)
#define TDM_FORMAT_CONFIG   0x43 // 32-bit Word, 256-Fs TDM, Master Mode Clk Det, TDM Format, 32-bit Data
#define TDM_OUTPUT_SLOTS    0x01 // Enable ADC1L/R and ADC2L/R output (4 slots total per device)
#define CLOCK_CONFIG        0x01 // Auto Clock Config Mode
#define DEFAULT_GAIN        0x18 // +12dB default gain


// --- Private Helper Functions ---

/**
 * @brief Writes a single byte to a specific register of a PCM1865 device.
 * @param hi2c Pointer to the I2C handle.
 * @param device_addr The 8-bit I2C address (already shifted).
 * @param reg_addr The register address.
 * @param data The byte to write.
 * @retval HAL_StatusTypeDef HAL status.
 */
static HAL_StatusTypeDef PCM1865_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t device_addr, uint8_t reg_addr, uint8_t data)
{
    if (hi2c == NULL) {
        return HAL_ERROR;
    }
    return HAL_I2C_Mem_Write(hi2c, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
}

/**
 * @brief Reads a single byte from a specific register of a PCM1865 device.
 * @param hi2c Pointer to the I2C handle.
 * @param device_addr The 8-bit I2C address (already shifted).
 * @param reg_addr The register address.
 * @param pData Pointer to store the read byte.
 * @retval HAL_StatusTypeDef HAL status.
 */
static HAL_StatusTypeDef PCM1865_ReadReg(I2C_HandleTypeDef *hi2c, uint16_t device_addr, uint8_t reg_addr, uint8_t *pData)
{
    if (hi2c == NULL || pData == NULL) {
        return HAL_ERROR;
    }
    return HAL_I2C_Mem_Read(hi2c, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, pData, 1, I2C_TIMEOUT_MS);
}


// --- Public Function Implementations ---

/**
 * @brief Initializes both PCM1865 devices with a standard TDM configuration.
 */
HAL_StatusTypeDef PCM1865_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (hi2c == NULL) {
        return HAL_ERROR;
    }

    printf("PCM1865: Initializing...\r\n");

    // --- Reset both devices ---
    printf("PCM1865: Resetting devices...\r\n");
    uint8_t reset_cmd = RESET_CMD;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_RESET, reset_cmd);
    if (status != HAL_OK) { printf("PCM1865: Error Resetting Device 0 (Status: %d)\r\n", status); return status; }

    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_RESET, reset_cmd);
    if (status != HAL_OK) { printf("PCM1865: Error Resetting Device 1 (Status: %d)\r\n", status); return status; }
    HAL_Delay(10); // Short delay after reset

    // --- Configure Device 0 (Address PCM1865_I2C_ADDR_0) ---
    printf("PCM1865: Configuring Device 0 (Addr: 0x%X)...\r\n", PCM1865_I2C_ADDR_0);
    // Input Selection
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_ADC1_IP_SEL_L, ADC1L_INPUT_SRC);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_ADC1_IP_SEL_R, ADC1R_INPUT_SRC);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_ADC2_IP_SEL_L, ADC2L_INPUT_SRC);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_ADC2_IP_SEL_R, ADC2R_INPUT_SRC);
    if (status != HAL_OK) return status;

    // TDM Configuration
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_TX_TDM_OFFSET, TDM_OFFSET_DEV0);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_FMT, TDM_FORMAT_CONFIG);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_TDM_OSEL, TDM_OUTPUT_SLOTS);
    if (status != HAL_OK) return status;

    // Clock Configuration
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_CLK_CFG0, CLOCK_CONFIG);
    if (status != HAL_OK) return status;

    // Default Gain Settings
    status = PCM1865_SetGain(hi2c, PCM1865_DEVICE_0, PCM1865_CHANNEL_1_LEFT, DEFAULT_GAIN);
    if (status != HAL_OK) return status;
    status = PCM1865_SetGain(hi2c, PCM1865_DEVICE_0, PCM1865_CHANNEL_1_RIGHT, DEFAULT_GAIN);
    if (status != HAL_OK) return status;
    status = PCM1865_SetGain(hi2c, PCM1865_DEVICE_0, PCM1865_CHANNEL_2_LEFT, DEFAULT_GAIN);
    if (status != HAL_OK) return status;
    status = PCM1865_SetGain(hi2c, PCM1865_DEVICE_0, PCM1865_CHANNEL_2_RIGHT, DEFAULT_GAIN);
    if (status != HAL_OK) return status;

    // --- Configure Device 1 (Address PCM1865_I2C_ADDR_1) ---
    printf("PCM1865: Configuring Device 1 (Addr: 0x%X)...\r\n", PCM1865_I2C_ADDR_1);
    // Input Selection (Same as device 0 in this example, adjust if needed)
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_ADC1_IP_SEL_L, ADC1L_INPUT_SRC);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_ADC1_IP_SEL_R, ADC1R_INPUT_SRC);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_ADC2_IP_SEL_L, ADC2L_INPUT_SRC);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_ADC2_IP_SEL_R, ADC2R_INPUT_SRC);
    if (status != HAL_OK) return status;

    // TDM Configuration
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_TX_TDM_OFFSET, TDM_OFFSET_DEV1);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_FMT, TDM_FORMAT_CONFIG);
    if (status != HAL_OK) return status;
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_TDM_OSEL, TDM_OUTPUT_SLOTS);
    if (status != HAL_OK) return status;

    // Clock Configuration
    status = PCM1865_WriteReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_CLK_CFG0, CLOCK_CONFIG);
    if (status != HAL_OK) return status;

    // Default Gain Settings
    status = PCM1865_SetGain(hi2c, PCM1865_DEVICE_1, PCM1865_CHANNEL_1_LEFT, DEFAULT_GAIN);
    if (status != HAL_OK) return status;
    status = PCM1865_SetGain(hi2c, PCM1865_DEVICE_1, PCM1865_CHANNEL_1_RIGHT, DEFAULT_GAIN);
    if (status != HAL_OK) return status;
    status = PCM1865_SetGain(hi2c, PCM1865_DEVICE_1, PCM1865_CHANNEL_2_LEFT, DEFAULT_GAIN);
    if (status != HAL_OK) return status;
    status = PCM1865_SetGain(hi2c, PCM1865_DEVICE_1, PCM1865_CHANNEL_2_RIGHT, DEFAULT_GAIN);
    if (status != HAL_OK) return status;

    // Optional: Read back a key register to verify communication after config
    uint8_t readVal = 0;
    status = PCM1865_ReadReg(hi2c, PCM1865_I2C_ADDR_0, PCM1865_CLK_CFG0, &readVal);
    if (status != HAL_OK || readVal != CLOCK_CONFIG) {
        printf("PCM1865: Verification Read Failed for Device 0 CLK_CFG0 (Status: %d, Read: 0x%X)\r\n", status, readVal);
        // Decide if this should be a fatal error
    }
    status = PCM1865_ReadReg(hi2c, PCM1865_I2C_ADDR_1, PCM1865_CLK_CFG0, &readVal);
     if (status != HAL_OK || readVal != CLOCK_CONFIG) {
        printf("PCM1865: Verification Read Failed for Device 1 CLK_CFG0 (Status: %d, Read: 0x%X)\r\n", status, readVal);
        // Decide if this should be a fatal error
    }


    printf("PCM1865: Initialization Complete (Last Status: %d)\r\n", status);
    return status; // Return the status of the last operation
}


/**
 * @brief Sets the Programmable Gain Amplifier (PGA) value for a specific channel.
 */
HAL_StatusTypeDef PCM1865_SetGain(I2C_HandleTypeDef *hi2c, PCM1865_Device_t device, PCM1865_Channel_t channel, uint8_t gainValue)
{
    uint16_t deviceAddr;
    uint8_t registerAddr;

    if (hi2c == NULL) {
        return HAL_ERROR;
    }

    // Determine Device Address
    switch (device) {
        case PCM1865_DEVICE_0:
            deviceAddr = PCM1865_I2C_ADDR_0;
            break;
        case PCM1865_DEVICE_1:
            deviceAddr = PCM1865_I2C_ADDR_1;
            break;
        default:
            return HAL_ERROR; // Invalid device
    }

    // Determine Register Address based on Channel
    switch (channel) {
        case PCM1865_CHANNEL_1_LEFT:
            registerAddr = PCM1865_PGA_VAL_CH1_L;
            break;
        case PCM1865_CHANNEL_1_RIGHT:
            registerAddr = PCM1865_PGA_VAL_CH1_R;
            break;
        case PCM1865_CHANNEL_2_LEFT:
            registerAddr = PCM1865_PGA_VAL_CH2_L;
            break;
        case PCM1865_CHANNEL_2_RIGHT:
            registerAddr = PCM1865_PGA_VAL_CH2_R;
            break;
        default:
            return HAL_ERROR; // Invalid channel
    }

    // Write the gain value
    return PCM1865_WriteReg(hi2c, deviceAddr, registerAddr, gainValue);
}


/**
 * @brief Reads all registers from both PCM1865 devices and prints status info.
 */
void PCM1865_ReadAndPrintStatus(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;
    uint8_t PCM1865_0_Reg[PCM1865_REG_COUNT];
    uint8_t PCM1865_1_Reg[PCM1865_REG_COUNT];

    printf("\n--- Reading PCM1865 Status ---\n");

    if (hi2c == NULL) {
        printf("PCM1865: Error - I2C Handle is NULL.\r\n");
        return;
    }

    // Read all registers from both devices (Can be slow, use selectively)
    printf("PCM1865: Reading registers from Device 0 (Addr: 0x%X)...\r\n", PCM1865_I2C_ADDR_0);
    for (int Offset = 0; Offset < PCM1865_REG_COUNT; Offset++) {
        status = PCM1865_ReadReg(hi2c, PCM1865_I2C_ADDR_0, Offset, &PCM1865_0_Reg[Offset]);
        if (status != HAL_OK) {
            printf("PCM1865: Error reading Device 0 register 0x%02X (Status: %d)\r\n", Offset, status);
            // Optional: return or break here if a read fails
        }
    }

    printf("PCM1865: Reading registers from Device 1 (Addr: 0x%X)...\r\n", PCM1865_I2C_ADDR_1);
    for (int Offset = 0; Offset < PCM1865_REG_COUNT; Offset++) {
        status = PCM1865_ReadReg(hi2c, PCM1865_I2C_ADDR_1, Offset, &PCM1865_1_Reg[Offset]);
         if (status != HAL_OK) {
            printf("PCM1865: Error reading Device 1 register 0x%02X (Status: %d)\r\n", Offset, status);
            // Optional: return or break here if a read fails
        }
    }

    // Extract and Print specific status values
    printf("\n--- PCM1865 Status Summary ---\n");

    // Device 0 Status
    printf("Device 0 (Addr 0x%X):\n", PCM1865_I2C_ADDR_0);
    printf("  PLL Status (Reg 0x%02X): 0x%02X (Locked: %d)\n", PCM1865_PLL_STATE, PCM1865_0_Reg[PCM1865_PLL_STATE], (PCM1865_0_Reg[PCM1865_PLL_STATE] >> 4) & 0x01);
    printf("  System Status (Reg 0x72): 0x%02X\n", PCM1865_0_Reg[114]);
    printf("  Sample Rate (Reg 0x73): 0x%02X\n", PCM1865_0_Reg[115]);
    printf("  BCK Rate (Reg 0x74): %d\n", (PCM1865_0_Reg[116] >> 4) & 0x0F);
    printf("  SCK Rate (Reg 0x74): %d\n", PCM1865_0_Reg[116] & 0x0F);
    printf("  Error Status (Reg 0x75): 0x%02X\n", PCM1865_0_Reg[117]);
    printf("  Power Status (Reg 0x78): 0x%02X\n", PCM1865_0_Reg[120]);

    // Device 1 Status
    printf("Device 1 (Addr 0x%X):\n", PCM1865_I2C_ADDR_1);
    printf("  PLL Status (Reg 0x%02X): 0x%02X (Locked: %d)\n", PCM1865_PLL_STATE, PCM1865_1_Reg[PCM1865_PLL_STATE], (PCM1865_1_Reg[PCM1865_PLL_STATE] >> 4) & 0x01);
    printf("  System Status (Reg 0x72): 0x%02X\n", PCM1865_1_Reg[114]);
    printf("  Sample Rate (Reg 0x73): 0x%02X\n", PCM1865_1_Reg[115]);
    printf("  BCK Rate (Reg 0x74): %d\n", (PCM1865_1_Reg[116] >> 4) & 0x0F);
    printf("  SCK Rate (Reg 0x74): %d\n", PCM1865_1_Reg[116] & 0x0F);
    printf("  Error Status (Reg 0x75): 0x%02X\n", PCM1865_1_Reg[117]);
    printf("  Power Status (Reg 0x78): 0x%02X\n", PCM1865_1_Reg[120]);

    printf("--- End PCM1865 Status ---\n");
}


/* USER CODE BEGIN Function Implementations */
// Add any other necessary functions specific to this driver
/* USER CODE END Function Implementations */