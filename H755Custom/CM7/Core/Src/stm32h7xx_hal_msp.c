/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
    PeriphClkInitStruct.PLL3.PLL3M = 4;
    PeriphClkInitStruct.PLL3.PLL3N = 10;
    PeriphClkInitStruct.PLL3.PLL3P = 2;
    PeriphClkInitStruct.PLL3.PLL3Q = 2;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */

  }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }

}

extern DMA_HandleTypeDef hdma_sai1_b;

extern DMA_HandleTypeDef hdma_sai1_a;

extern DMA_HandleTypeDef hdma_sai2_b;

static uint32_t SAI1_client =0;
static uint32_t SAI2_client =0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* hsai)
{

  GPIO_InitTypeDef GPIO_InitStruct;
/* SAI1 */
    if(hsai->Instance==SAI1_Block_A)
    {
    /* Peripheral clock enable */
    if (SAI1_client == 0)
    {
       __HAL_RCC_SAI1_CLK_ENABLE();
    }
    SAI1_client ++;

    /**SAI1_A_Block_A GPIO Configuration
    PE2     ------> SAI1_MCLK_A
    PE4     ------> SAI1_FS_A
    PE5     ------> SAI1_SCK_A
    PE6     ------> SAI1_SD_A
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

      /* Peripheral DMA init*/

    hdma_sai1_a.Instance = DMA1_Stream2;
    hdma_sai1_a.Init.Request = DMA_REQUEST_SAI1_A;
    hdma_sai1_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sai1_a.Init.Mode = DMA_CIRCULAR;
    hdma_sai1_a.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_sai1_a.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai1_a);

    __HAL_LINKDMA(hsai,hdmatx,hdma_sai1_a);

    }
    if(hsai->Instance==SAI1_Block_B)
    {
      /* Peripheral clock enable */
      if (SAI1_client == 0)
      {
       __HAL_RCC_SAI1_CLK_ENABLE();
      }
    SAI1_client ++;

    /**SAI1_B_Block_B GPIO Configuration
    PE3     ------> SAI1_SD_B
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

      /* Peripheral DMA init*/

    hdma_sai1_b.Instance = DMA1_Stream0;
    hdma_sai1_b.Init.Request = DMA_REQUEST_SAI1_B;
    hdma_sai1_b.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_sai1_b.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai1_b.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai1_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai1_b.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sai1_b.Init.Mode = DMA_CIRCULAR;
    hdma_sai1_b.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_sai1_b.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai1_b) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai1_b);
    __HAL_LINKDMA(hsai,hdmatx,hdma_sai1_b);
    }
/* SAI2 */
    if(hsai->Instance==SAI2_Block_B)
    {
      /* Peripheral clock enable */
      if (SAI2_client == 0)
      {
       __HAL_RCC_SAI2_CLK_ENABLE();
      }
    SAI2_client ++;

    /**SAI2_B_Block_B GPIO Configuration
    PC0     ------> SAI2_FS_B
    PA0     ------> SAI2_SD_B
    PA2     ------> SAI2_SCK_B
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /* Peripheral DMA init*/

    hdma_sai2_b.Instance = DMA1_Stream1;
    hdma_sai2_b.Init.Request = DMA_REQUEST_SAI2_B;
    hdma_sai2_b.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sai2_b.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai2_b.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai2_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai2_b.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sai2_b.Init.Mode = DMA_CIRCULAR;
    hdma_sai2_b.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_sai2_b.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai2_b) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai2_b);
    __HAL_LINKDMA(hsai,hdmatx,hdma_sai2_b);
    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* hsai)
{
/* SAI1 */
    if(hsai->Instance==SAI1_Block_A)
    {
    SAI1_client --;
    if (SAI1_client == 0)
      {
      /* Peripheral clock disable */
       __HAL_RCC_SAI1_CLK_DISABLE();
      }

    /**SAI1_A_Block_A GPIO Configuration
    PE2     ------> SAI1_MCLK_A
    PE4     ------> SAI1_FS_A
    PE5     ------> SAI1_SCK_A
    PE6     ------> SAI1_SD_A
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

    /* SAI1 DMA Deinit */
    HAL_DMA_DeInit(hsai->hdmarx);
    HAL_DMA_DeInit(hsai->hdmatx);
    }
    if(hsai->Instance==SAI1_Block_B)
    {
    SAI1_client --;
      if (SAI1_client == 0)
      {
      /* Peripheral clock disable */
      __HAL_RCC_SAI1_CLK_DISABLE();
      }

    /**SAI1_B_Block_B GPIO Configuration
    PE3     ------> SAI1_SD_B
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_3);

    /* SAI1 DMA Deinit */
    HAL_DMA_DeInit(hsai->hdmarx);
    HAL_DMA_DeInit(hsai->hdmatx);
    }
/* SAI2 */
    if(hsai->Instance==SAI2_Block_B)
    {
    SAI2_client --;
      if (SAI2_client == 0)
      {
      /* Peripheral clock disable */
      __HAL_RCC_SAI2_CLK_DISABLE();
      }

    /**SAI2_B_Block_B GPIO Configuration
    PC0     ------> SAI2_FS_B
    PA0     ------> SAI2_SD_B
    PA2     ------> SAI2_SCK_B
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_2);

    /* SAI2 DMA Deinit */
    HAL_DMA_DeInit(hsai->hdmarx);
    HAL_DMA_DeInit(hsai->hdmatx);
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
