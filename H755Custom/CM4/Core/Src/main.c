/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mixer_state.h" // Assuming this includes your shared memory pointers/structs
#include <stdio.h>      // For printf
#include <string.h>     // For memcpy and memset
#include "pcm1865.h" // Include header for PCM1865_Init if used
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Structure to hold the decoded command message
typedef struct {
    uint32_t channel; // Corresponds to ChannelParameters index (0=main, 1-8=inputs)
    uint32_t effect_id; // Identifier for EQ, Comp, Reverb, Gain, Pan etc.
    uint32_t parameter_id; // Identifier for specific param within effect (e.g., EQ band freq)
    uint32_t raw_value; // The value (can be int or float bits cast to uint32_t)
} MixerCommand;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

// Define the exact size of the expected I2C message from the Pi
#define I2C_MESSAGE_SIZE 16 // (4 * sizeof(uint32_t))

// Define the I2C Slave address for the CM4 (MUST MATCH PI'S TARGET ADDRESS)
// Example: 0x42. The HAL expects te 8-bit address (7-bit shifted left)
#define CM4_I2C_ADDRESS (0x42 << 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;

/* USER CODE BEGIN PV */
// Buffer to receive the raw I2C message bytes
uint8_t i2c_rx_buffer[I2C_MESSAGE_SIZE];

// Flag to signal that a complete message has been received
volatile uint8_t i2c_message_received_flag = 0;

// Variable to hold the decoded command
MixerCommand decoded_command;

// --- Add printf retargeting if not already done ---
// Depends on your BSP/Setup. Assumes UART connected to ST-Link VCP exists
// Make sure the UART handle (e.g., huart3 or hcom_uart[COM1]) is correctly initialized
// extern UART_HandleTypeDef huart3; // Example: Adjust handle as needed
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  // Replace huart3 with the correct UART handle for your VCP
  // HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  // Or if using BSP:
  // BSP_COM_Transmit(COM1, (uint8_t *)&ch, 1, HAL_MAX_DELAY); // Requires BSP setup
  return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_I2C4_Init(void);
/* USER CODE BEGIN PFP */
// Function prototype for processing the decoded command
static void Process_Mixer_Command(const MixerCommand* cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config(); // Make sure MPU is configured correctly for shared RAM

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Initialize COM port for printf BEFORE first printf call
  // Example: if (BSP_COM_Init(COM1, ...) != BSP_ERROR_NONE) Error_Handler();
  printf("CM4 Booting...\r\n");
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_I2C4_Init(); // Ensure OwnAddress1 is set correctly in this function

  /* USER CODE BEGIN 2 */
  printf("CM4 I2C Address: 0x%X (7-bit: 0x%X)\r\n", CM4_I2C_ADDRESS, CM4_I2C_ADDRESS >> 1);

  // Conditional ADC Initialization
  #ifdef USE_CUSTOM_MISTER_MIXER_BOARD
      printf("--- Custom Mister Mixer Board Build ---\r\n");
      // HAL_StatusTypeDef status;
      // status = PCM1865_Init(&hi2c4); // Call your ADC init
      // if (status != HAL_OK) { /* handle error */ } else { /* success */ }
  #else
      printf("--- Nucleo Board Build - Skipping PCM1865 Init on CM4 ---\r\n");
  #endif

  // --- Start I2C Listening ---
  printf("Starting I2C Slave Listen...\r\n");
  // Clear buffer initially
  memset(i2c_rx_buffer, 0, I2C_MESSAGE_SIZE);
  // Start listening for exactly I2C_MESSAGE_SIZE bytes
  if (HAL_I2C_Slave_Receive_IT(&hi2c4, i2c_rx_buffer, I2C_MESSAGE_SIZE) != HAL_OK)
  {
      printf("!!! HAL_I2C_Slave_Receive_IT Failed !!!\r\n");
      Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Check if a full message was received
    if (i2c_message_received_flag)
    {
        printf("I2C Message Received. Decoding...\r\n");

        // --- Decode the message ---
        // Safely copy the bytes from the buffer into the structure fields
        // This handles potential alignment issues and avoids strict aliasing violations.
        // Assumes Pi and STM32 have the same endianness (likely both little-endian).
        memcpy(&decoded_command.channel, &i2c_rx_buffer[0], sizeof(uint32_t));
        memcpy(&decoded_command.effect_id, &i2c_rx_buffer[4], sizeof(uint32_t));
        memcpy(&decoded_command.parameter_id, &i2c_rx_buffer[8], sizeof(uint32_t));
        memcpy(&decoded_command.raw_value, &i2c_rx_buffer[12], sizeof(uint32_t));

        // Optional: Print decoded values for debugging
        printf("Decoded Cmd: Chan=%lu, FX=%lu, Param=%lu, Val=0x%08lX\r\n",
               decoded_command.channel,
               decoded_command.effect_id,
               decoded_command.parameter_id,
               decoded_command.raw_value);

        // --- Process the command ---
        Process_Mixer_Command(&decoded_command);

        // --- Cleanup ---
        // Clear the flag now that processing is done
        i2c_message_received_flag = 0;

        // IMPORTANT: The listen is re-armed in the RxCpltCallback below
        // No need to call HAL_I2C_Slave_Receive_IT here unless the callback failed

    } // end if(i2c_message_received_flag)

    // Other CM4 tasks can go here (e.g., sending data TO Pi via SPI/UART)
    // HAL_Delay(1); // Small delay to prevent tight loop if nothing else to do

  } // end while(1)
  /* USER CODE END 3 */
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 60;
  PeriphClkInitStruct.PLL2.PLL2P = 85;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.Sai23ClockSelection = RCC_SAI23CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C4 Initialization Function
  */
static void MX_I2C4_Init(void)
{
  /* USER CODE BEGIN I2C4_Init 0 */
  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */
  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x307075B1; // Keep your timing value
  // !!! SET THE SLAVE ADDRESS HERE !!!
  hi2c4.Init.OwnAddress1 = CM4_I2C_ADDRESS; // Use the defined address
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Stretching is usually required for slaves
  if (HAL_I2C_Init(&hi2c4) != HAL_OK) { Error_Handler(); }

  /** Configure Analogue filter */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }

  /** Configure Digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN I2C4_Init 2 */
  /* USER CODE END I2C4_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/**
  * @brief  This is where you will process the decoded I2C command.
  * @param  cmd Pointer to the decoded MixerCommand structure.
  * @retval None
  */
static void Process_Mixer_Command(const MixerCommand* cmd)
{

}