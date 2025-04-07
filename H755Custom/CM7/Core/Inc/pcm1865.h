#ifndef INC_PCM1865_DRIVER_H_
#define INC_PCM1865_DRIVER_H_

#include "stm32h7xx_hal.h" // Include base HAL types
#include <stdint.h>         // For standard integer types

/* USER CODE BEGIN Includes */
// Add any other necessary includes for your project if needed
/* USER CODE END Includes */


// --- Public Defines ---

// I2C Addresses (Shifted left by 1, as expected by HAL)
#define PCM1865_I2C_ADDR_0        (0x4A << 1) // Primary Device Address (e.g., ADDR=0)
#define PCM1865_I2C_ADDR_1        (0x4B << 1) // Secondary Device Address (e.g., ADDR=1)

// PCM1865 Register Map (Commonly used subset)
#define PCM1865_RESET               (0x00)
#define PCM1865_PGA_VAL_CH1_L       (0x01)
#define PCM1865_PGA_VAL_CH1_R       (0x02)
#define PCM1865_PGA_VAL_CH2_L       (0x03)
#define PCM1865_PGA_VAL_CH2_R       (0x04)
#define PCM1865_ADC1_IP_SEL_L       (0x06) // Select input to route to ADC1 left input.
#define PCM1865_ADC1_IP_SEL_R       (0x07) // Select input to route to ADC1 right input.
#define PCM1865_ADC2_IP_SEL_L       (0x08) // Select input to route to ADC2 left input.
#define PCM1865_ADC2_IP_SEL_R       (0x09) // Select input to route to ADC2 right input.
#define PCM1865_FMT                 (0x0B) // RX_WLEN, TDM_LRCLK_MODE, TX_WLEN, FMT
#define PCM1865_TDM_OSEL            (0x0C) // TDM Output Slot Selection
#define PCM1865_TX_TDM_OFFSET       (0x0D) // TDM Transmit Offset
#define PCM1865_CLK_CFG0            (0x20) // Basic clock config.
#define PCM1865_PLL_STATE			(0x28) // PLL Control & Status
#define PCM1865_PWR_STATE           (0x70) // Power down, Sleep, Standby

// --- Public Typedefs ---

/**
 * @brief Enum to identify PCM1865 devices on the I2C bus.
 */
typedef enum {
    PCM1865_DEVICE_0 = 0, // Corresponds to PCM1865_I2C_ADDR_0
    PCM1865_DEVICE_1 = 1  // Corresponds to PCM1865_I2C_ADDR_1
} PCM1865_Device_t;

/**
 * @brief Enum to identify individual ADC channels within a PCM1865 device.
 */
typedef enum {
    PCM1865_CHANNEL_1_LEFT  = 0,
    PCM1865_CHANNEL_1_RIGHT = 1,
    PCM1865_CHANNEL_2_LEFT  = 2,
    PCM1865_CHANNEL_2_RIGHT = 3
} PCM1865_Channel_t;

/**
 * @brief Structure to hold initialization parameters (optional, for more flexibility).
 *        For now, we'll keep the original fixed configuration.
 */
// typedef struct {
//     uint8_t initial_gain;
//     // Add other parameters if you want to make init more configurable
// } PCM1865_InitTypeDef;


// --- Public Function Prototypes ---

/**
 * @brief Initializes both PCM1865 devices with a standard TDM configuration.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for communication.
 * @retval HAL_StatusTypeDef HAL_OK on success, HAL_ERROR or HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef PCM1865_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Sets the Programmable Gain Amplifier (PGA) value for a specific channel.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for communication.
 * @param device The target PCM1865 device (PCM1865_DEVICE_0 or PCM1865_DEVICE_1).
 * @param channel The target channel (PCM1865_CHANNEL_1_LEFT, ..., PCM1865_CHANNEL_2_RIGHT).
 * @param gainValue The 8-bit gain value to write to the register (e.g., 0x18 for +12dB).
 *                  Refer to the PCM1865 datasheet for gain value mapping.
 * @retval HAL_StatusTypeDef HAL_OK on success, HAL_ERROR or HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef PCM1865_SetGain(I2C_HandleTypeDef *hi2c, PCM1865_Device_t device, PCM1865_Channel_t channel, uint8_t gainValue);

/**
 * @brief Reads all registers from both PCM1865 devices and prints status info.
 *        (Kept similar to original for debugging purposes).
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for communication.
 * @retval None (Prints output via printf).
 */
void PCM1865_ReadAndPrintStatus(I2C_HandleTypeDef *hi2c);


HAL_StatusTypeDef PCM1865_SetGainDB_GlobalChannel(I2C_HandleTypeDef *hi2c, uint8_t global_channel_index, float gain_db);

#endif /* INC_PCM1865_DRIVER_H_ */