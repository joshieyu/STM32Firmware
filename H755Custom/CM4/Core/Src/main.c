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
#include <stdio.h>  // Needed for printf
#include <string.h> // Needed for memset
#include <stdbool.h>
#include "mixer_state.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t channel;
  uint32_t effect_id;
  uint32_t parameter_id;
  uint32_t raw_value;
} MixerCommand;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define I2C_COMMAND_SIZE 16                       // The actual command data size
#define I2C_RX_BUFFER_SIZE (I2C_COMMAND_SIZE + 1) // Receive 1 extra byte
#define CM4_I2C_ADDRESS (0x42 << 1)

// --- Protocol ID Definitions (Mirroring Python) ---
// Channel IDs
#define CH_ID_MASTER 0 // Master Channel index
// Input channels use index 1-8 directly
#define CH_ID_SOLOING_ACTIVE 9
#define CH_ID_INFERENCING_ACTIVE 10
#define CH_ID_HW_INIT_READY 11

// Effect IDs
#define FX_ID_DIRECT 0
#define FX_ID_EQ 1
#define FX_ID_COMP 2
#define FX_ID_DIST 3
#define FX_ID_PHASER 4
#define FX_ID_REVERB 5

// Parameter IDs for Direct Channel (FX_ID_DIRECT = 0)
#define PARAM_ID_DIRECT_MUTED 0
#define PARAM_ID_DIRECT_SOLOED 1
#define PARAM_ID_DIRECT_PANNING 2
#define PARAM_ID_DIRECT_DIGITAL_GAIN 3
#define PARAM_ID_DIRECT_ANALOG_GAIN 4 // Special handling
#define PARAM_ID_DIRECT_STEREO 5

// Parameter IDs for Equalizer (FX_ID_EQ = 1)
#define PARAM_ID_EQ_ENABLED 0
#define PARAM_ID_EQ_LS_GAIN 1
#define PARAM_ID_EQ_LS_FREQ 2
#define PARAM_ID_EQ_LS_Q 3
#define PARAM_ID_EQ_HS_GAIN 4
#define PARAM_ID_EQ_HS_FREQ 5
#define PARAM_ID_EQ_HS_Q 6
#define PARAM_ID_EQ_B0_GAIN 7
#define PARAM_ID_EQ_B0_FREQ 8
#define PARAM_ID_EQ_B0_Q 9
#define PARAM_ID_EQ_B1_GAIN 10
#define PARAM_ID_EQ_B1_FREQ 11
#define PARAM_ID_EQ_B1_Q 12
#define PARAM_ID_EQ_B2_GAIN 13
#define PARAM_ID_EQ_B2_FREQ 14
#define PARAM_ID_EQ_B2_Q 15
#define PARAM_ID_EQ_B3_GAIN 16
#define PARAM_ID_EQ_B3_FREQ 17
#define PARAM_ID_EQ_B3_Q 18

// Parameter IDs for Compressor (FX_ID_COMP = 2)
#define PARAM_ID_COMP_ENABLED 0
#define PARAM_ID_COMP_THRESH 1
#define PARAM_ID_COMP_RATIO 2
#define PARAM_ID_COMP_ATTACK 3
#define PARAM_ID_COMP_RELEASE 4
#define PARAM_ID_COMP_KNEE 5
#define PARAM_ID_COMP_MAKEUP 6

// Parameter IDs for Distortion (FX_ID_DIST = 3)
#define PARAM_ID_DIST_ENABLED 0
#define PARAM_ID_DIST_DRIVE 1
#define PARAM_ID_DIST_OUTPUT 2

// Parameter IDs for Phaser (FX_ID_PHASER = 4)
#define PARAM_ID_PHASER_ENABLED 0
#define PARAM_ID_PHASER_RATE 1
#define PARAM_ID_PHASER_DEPTH 2

// Parameter IDs for Reverb (FX_ID_REVERB = 5)
#define PARAM_ID_REVERB_ENABLED 0
#define PARAM_ID_REVERB_DECAY 1
#define PARAM_ID_REVERB_WET 2

// Parameter ID for Top-Level Bool Flags
#define PARAM_ID_TOPLEVEL_BOOL 0
// --- End Protocol ID Definitions ---

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DECODE_FLOAT(raw_val_ptr) (*((float *)(raw_val_ptr)))
#define DECODE_BOOL(raw_val) ((raw_val) != 0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */
int8_t i2c_rx_buffer_isr[I2C_RX_BUFFER_SIZE];   // Size 17 now
uint8_t i2c_process_buffer[I2C_RX_BUFFER_SIZE]; // Size 17 now
// --- Flags, decoded_command (remain the same) ---
volatile bool g_i2c_message_ready_to_process = false;
volatile bool g_i2c_error_flag = false;
volatile uint32_t g_i2c_last_error_code = 0;

MixerCommand decoded_command;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
static void Process_Mixer_Command(const MixerCommand *cmd);
static void Set_Analog_Gain(uint32_t channel_index, float gain_db);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- I2C Callbacks ---
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C3)
  {
    // Copy potentially 17 bytes
    memcpy(i2c_process_buffer, i2c_rx_buffer_isr, I2C_RX_BUFFER_SIZE);
    g_i2c_message_ready_to_process = true;
    printf("I2C Rx Cplt Callback: Success (%d bytes received).\r\n", I2C_RX_BUFFER_SIZE); // Log actual size received

    // Re-arm listening for 17 bytes
    memset(i2c_rx_buffer_isr, 0xCC, I2C_RX_BUFFER_SIZE);
    HAL_StatusTypeDef status = HAL_I2C_Slave_Receive_IT(&hi2c3, i2c_rx_buffer_isr, I2C_RX_BUFFER_SIZE); // Use new size
    if (status != HAL_OK)
    { /* ... Error Handling ... */
    }
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C3)
  {
    // ... (Error logging) ...
    // Re-arm listening for 17 bytes
    HAL_StatusTypeDef status = HAL_I2C_Slave_Receive_IT(&hi2c3, i2c_rx_buffer_isr, I2C_RX_BUFFER_SIZE); // Use new size
    if (status != HAL_OK)
    { /* ... Error Handling ... */
    }
  }
}

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
  MPU_Config();

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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1 | RCC_PERIPHCLK_SAI2;
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
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */
  __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x307075B1;
  hi2c3.Init.OwnAddress1 = 132;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */
  // --- CRITICAL: Enable I2C Interrupts in NVIC ---
  printf("Enabling I2C3 NVIC Interrupts...\r\n");
  // Set appropriate priorities (lower number = higher priority)
  HAL_NVIC_SetPriority(I2C3_EV_IRQn, 5, 0); // Example priority
  HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
  HAL_NVIC_SetPriority(I2C3_ER_IRQn, 5, 0); // Example priority
  HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
  printf("I2C3 NVIC Interrupts Enabled.\r\n");
  /* USER CODE END I2C3_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // --- !!! MANUALLY CONFIGURE I2C3 PINS (PA8 SCL, PC9 SDA) !!! ---
  printf("Manually configuring I2C3 GPIO Pins (PA8, PC9)...\r\n");

  // Configure PA8 as I2C3_SCL
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;       // Alternate Function Open Drain
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // Enable Internal Pull-up (if no external)
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // Suitable speed
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;    // *** VERIFY AF for PA8 -> I2C3_SCL in DATASHEET ***
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);       // Initialize PA8

  // Configure PC9 as I2C3_SDA
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;       // Alternate Function Open Drain
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // Enable Internal Pull-up (if no external)
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // Suitable speed
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;    // *** VERIFY AF for PC9 -> I2C3_SDA in DATASHEET ***
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);       // Initialize PC9

  printf("I2C3 GPIO Pins configured.\r\n");
  // --- END MANUAL I2C3 PIN CONFIG ---
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
  HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);
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

#ifdef USE_FULL_ASSERT
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
