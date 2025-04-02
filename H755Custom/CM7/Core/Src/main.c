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
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define PCM1865_Reg_Size 128
#define AUDIO_BUFFER_SIZE  4096
#define TRIANGLE_BUFFER_SIZE 8192
#define MAX_AMPLITUDE_16BIT (32767)
#define MIN_AMPLITUDE_16BIT (-32768)

#define PCM1865_I2C_ADDR   (0x4A << 1)   // Adjust this based on your hardware strap
#define PCM1865_I2C_ADDR_2   (0x4B << 1)   // Adjust this based on your hardware strap
#define SINE_SAMPLES 440

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
#define PCM1865_TDM_OSEL            (0x0C)
#define PCM1865_TX_TDM_OFFSET       (0x0D)
#define PCM1865_CLK_CFG0            (0x20) // Basic clock config.
#define PCM1865_PLL_STATE			(0x28)
#define PCM1865_PWR_STATE           (0x70) // Power down, Sleep, Standby
#define DC_VALUE_TO_TRANSMIT ((int32_t)500000) << 8 // Example: Output a positive DC level

// --- Global or Static Buffer ---
// Buffer needs to hold exactly ONE stereo sample pair (Left, Right).
// Must be accessible by DMA (global or static).
int32_t single_value_dma_buffer[2]; // Size is 2 for L/R pair

// Global buffer to capture ADC data from SAI
int32_t audioRxBuffer[AUDIO_BUFFER_SIZE];
int32_t audioPlaybackBuffer[AUDIO_BUFFER_SIZE * 2];

int32_t triangleBuffer[TRIANGLE_BUFFER_SIZE * 2];

int16_t Wave_LUT[128] = {
    2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929, 3020, 3108, 3193, 3275, 3355,
    3431, 3504, 3574, 3639, 3701, 3759, 3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076,
    4087, 4094, 4095, 4091, 4082, 4069, 4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730,
    3671, 3607, 3539, 3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500,
    2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595, 1497, 1400, 1305, 1212, 1120, 1031,
    944, 860, 779, 701, 627, 556, 488, 424, 365, 309, 258, 211, 168, 130, 97,
    69, 45, 26, 13, 4, 0, 1, 8, 19, 35, 56, 82, 113, 149, 189,
    234, 283, 336, 394, 456, 521, 591, 664, 740, 820, 902, 987, 1075, 1166, 1258,
    1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047
};

int16_t sine_wave_table_128_samples[128 * 2] = {
        0,     0,    491,   491,    980,   980,   1467,  1467,   1951,  1951,
     2429,  2429,   2900,  2900,   3362,  3362,   3813,  3813,   4253,  4253,
     4679,  4679,   5091,  5091,   5487,  5487,   5867,  5867,   6229,  6229,
     6573,  6573,   6898,  6898,   7203,  7203,   7488,  7488,   7753,  7753,
     7997,  7997,   8219,  8219,   8420,  8420,   8600,  8600,   8758,  8758,
     8895,  8895,   9011,  9011,   9106,  9106,   9181,  9181,   9236,  9236,
     9271,  9271,   9287,  9287,   9283,  9283,   9261,  9261,   9219,  9219,
     9158,  9158,   9079,  9079,   8982,  8982,   8868,  8868,   8737,  8737,
     8590,  8590,   8428,  8428,   8251,  8251,   8061,  8061,   7857,  7857,
     7642,  7642,   7415,  7415,   7178,  7178,   6930,  6930,   6673,  6673,
     6407,  6407,   6133,  6133,   5851,  5851,   5562,  5562,   5267,  5267,
     4966,  4966,   4660,  4660,   4350,  4350,   4036,  4036,   3719,  3719,
     3400,  3400,   3079,  3079,   2757,  2757,   2435,  2435,   2113,  2113,
     1792,  1792,   1473,  1473,   1155,  1155,    839,   839,    526,   526,
      214,   214,   -95,   -95,   -403,  -403,   -708,  -708,  -1010, -1010,
    -1308, -1308,  -1602, -1602,  -1891, -1891,  -2175, -2175,  -2453, -2453,
    -2725, -2725,  -2990, -2990,  -3248, -3248,  -3499, -3499,  -3742, -3742,
    -3977, -3977,  -4204, -4204,  -4423, -4423,  -4633, -4633,  -4835, -4835,
    -5028, -5028,  -5212, -5212,  -5388, -5388,  -5554, -5554,  -5711, -5711,
    -5859, -5859,  -5997, -5997,  -6126, -6126,  -6245, -6245,  -6355, -6355,
    -6455, -6455,  -6545, -6545,  -6626, -6626,  -6697, -6697,  -6758, -6758,
    -6809, -6809,  -6850, -6850,  -6881, -6881,  -6902, -6902,  -6912, -6912,
    -6913, -6913,  -6903, -6903,  -6883, -6883,  -6853, -6853,  -6813, -6813,
    -6763, -6763,  -6703, -6703,  -6633, -6633,  -6553, -6553,  -6464, -6464,
    -6365, -6365,  -6257, -6257,  -6140, -6140,  -6014, -6014,  -5879, -5879,
    -5736, -5736,  -5584, -5584 // Note: Last sample pair omitted slightly to fit 128 points exactly starting from 0
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai2_b;

SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C4_Init(void);
static void MX_SAI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_SAI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int le, char *ptr, int len)
    {
    int DataIdx;
    for(DataIdx = 0; DataIdx < len; DataIdx++)
        {
        ITM_SendChar(*ptr++);
        }
    return len;
    }

void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    char msg[64];
    HAL_StatusTypeDef res;
    // I2C addresses are 7-bit. The HAL function expects the 7-bit address shifted left by 1.
    for (uint8_t addr = 1; addr < 128; addr++) {
        // Try up to 3 times with a small timeout.
        res = HAL_I2C_IsDeviceReady(hi2c, addr << 1, 3, 10);
        if (res == HAL_OK) {
            sprintf(msg, "Device found at 0x%02X\r\n", addr);
            // You can send this message over UART, SWV, or your debugger console.
            printf("%s", msg);
        }
    }
}

// Stereo 16-bit, so total array size = SINE_SAMPLES * 2
int16_t sineTable[SINE_SAMPLES * 2];

void GenerateSineWaveTable(int16_t amplitude)
{
    // --- Input Validation ---
    // Ensure amplitude is within valid int16_t positive range
    if (amplitude > MAX_AMPLITUDE_16BIT) {
        amplitude = MAX_AMPLITUDE_16BIT;
        // Consider adding a printf warning here if debugging is enabled
    }
    if (amplitude < 0) {
        // Amplitude should represent the peak, make it positive.
        amplitude = 0;
         // Consider adding a printf warning here if debugging is enabled
    }

    // --- Calculations ---
    // Use float for intermediate calculations to maintain precision
    float amplitude_f = (float)amplitude;
    const float twoPi = 2.0f * (float)M_PI;
    // Calculate the angle step needed for each sample point in the table
    const float phaseIncrement = twoPi / (float)SINE_SAMPLES;

    // --- Table Population ---
    for (int i = 0; i < SINE_SAMPLES; i++)
    {
        // 1. Calculate the angle for this sample point (0 to 2*PI)
        float angle = phaseIncrement * (float)i;

        // 2. Calculate the sine value (-1.0 to +1.0) and scale by amplitude
        float sampleValue_f = sinf(angle) * amplitude_f;

        // 3. **CRITICAL: Clamp** the float value to the int16_t range BEFORE casting.
        //    This prevents wrap-around artifacts from floating point inaccuracies.
        if (sampleValue_f >= (float)MAX_AMPLITUDE_16BIT) {
             sampleValue_f = (float)MAX_AMPLITUDE_16BIT; // Clamp to max positive
        } else if (sampleValue_f <= (float)MIN_AMPLITUDE_16BIT) {
             sampleValue_f = (float)MIN_AMPLITUDE_16BIT; // Clamp to max negative
        }

        // 4. Cast the clamped float value to the target signed integer type.
        //    This preserves the sign and centers the wave around 0.
        int16_t sample = (int16_t)sampleValue_f;

        // 5. --- Stereo Interleaving (Mono Output) ---
        //    Write the same signed sample to both Left and Right channels
        sineTable[i * 2 + 0] = sample; // Left channel sample for time point i
        sineTable[i * 2 + 1] = sample; // Right channel sample for time point i
    }
    // Optional: Add a printf here to indicate table generation is complete if needed
    // printf("Sine wave table generated.\n");
}

static void fill_buffer_with_square_wave(int32_t *buf, uint32_t num_samples)
{
    // Fill up a 100 Hz square wave
    // 44.1 kHz sample rate -> 441 samples in 100 Hz -> toggle every 220 samples
    int toggle_period = 440;
    int count = 0;
    int wave_state = 1;
    int32_t magnitude = 8000000;

    for(int i = 0; i < num_samples; i++)
    {
        buf[i] = magnitude * wave_state;
        count++;
        if(count >= toggle_period)
        {
            count = 0;
            wave_state = wave_state * (-1); // toggle
        }
    }
}

static void fill_buffer_with_triangle_wave(int32_t *buf, uint32_t num_samples)
{
	for (int32_t i = 0; i < num_samples; i++)
	{
		buf[(i)*2] = (i*1000);
		buf[(i)*2+1] = (i*1000);
	}
}


HAL_StatusTypeDef ADC_INIT() {
	HAL_StatusTypeDef status;

	uint8_t regRead1, regRead2;

	uint8_t reset = 0xFE;
	status = HAL_I2C_Mem_Write(&hi2c4, // reset
            PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
            0x00,                 // Register 0x00 is the page-select register
            I2C_MEMADD_SIZE_8BIT,
            &reset,
            1,                    // Writing 1 byte
            100);
	status = HAL_I2C_Mem_Write(&hi2c4, // Reset
	            PCM1865_I2C_ADDR,     // 7-bit device address << 1
	            0x00,                 // Register 0x00 is the page-select register
	            I2C_MEMADD_SIZE_8BIT,
	            &reset,
	            1,                    // Writing 1 byte
	            100);
	// status = HAL_I2C_Mem_Read(&hi2c4, // Read
	//             PCM1865_I2C_ADDR,     // 7-bit device address << 1
	//             0x00,                 // Register 0x00 is the page-select register
	//             I2C_MEMADD_SIZE_8BIT,
	//             &regRead1,
	//             1,                    // Writing 1 byte
	//             100);

	// status = HAL_I2C_Mem_Read(&hi2c4, // Read
	// 	            PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
	// 	            0x00,                 // Register 0x00 is the page-select register
	// 	            I2C_MEMADD_SIZE_8BIT,
	// 	            &regRead2,
	// 	            1,                    // Writing 1 byte
	// 	            100);

	// printf("page register 1: %x\r\n", regRead1);
	// printf("page register 2: %x\r\n", regRead2);



  uint8_t adc_select_1L = 0x41; // SET ADC_1_L source to VIN1L
  uint8_t adc_select_1R = 0x41; // SET ADC_1_R source to VIN1R
	uint8_t adc_select_2L = 0x42; // SET ADC_2_L source to VIN2L??
  uint8_t adc_select_2R = 0x42; // SET ADC_2_R source to VIN2R??


  // Set ADC1 input sources
	status = HAL_I2C_Mem_Write(&hi2c4, // Set ADC1 source
	            PCM1865_I2C_ADDR,     // 7-bit device address << 1
				PCM1865_ADC1_IP_SEL_L,                 // Register 0x00 is the page-select register
	            I2C_MEMADD_SIZE_8BIT,
	            &adc_select_1L,
	            1,                    // Writing 1 byte
	            100);
	status = HAL_I2C_Mem_Write(&hi2c4,
		            PCM1865_I2C_ADDR,     // 7-bit device address << 1
					PCM1865_ADC1_IP_SEL_L,                 // Register 0x00 is the page-select register
		            I2C_MEMADD_SIZE_8BIT,
		            &adc_select_1R,
		            1,                    // Writing 1 byte
		            100);

  status = HAL_I2C_Mem_Write(&hi2c4, // Set ADC1 source
	            PCM1865_I2C_ADDR,     // 7-bit device address << 1
				PCM1865_ADC2_IP_SEL_L,                 // Register 0x00 is the page-select register
	            I2C_MEMADD_SIZE_8BIT,
	            &adc_select_2L,
	            1,                    // Writing 1 byte
	            100);
	status = HAL_I2C_Mem_Write(&hi2c4,
		            PCM1865_I2C_ADDR,     // 7-bit device address << 1
					PCM1865_ADC2_IP_SEL_L,                 // Register 0x00 is the page-select register
		            I2C_MEMADD_SIZE_8BIT,
		            &adc_select_2R,
		            1,                    // Writing 1 byte
		            100);

  // Set ADC2 input sources
	status = HAL_I2C_Mem_Write(&hi2c4, // Set ADC1 source
	            PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
				PCM1865_ADC1_IP_SEL_L,                 // Register 0x00 is the page-select register
	            I2C_MEMADD_SIZE_8BIT,
	            &adc_select_1L,
	            1,                    // Writing 1 byte
	            100);
	status = HAL_I2C_Mem_Write(&hi2c4,
		            PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
					PCM1865_ADC1_IP_SEL_L,                 // Register 0x00 is the page-select register
		            I2C_MEMADD_SIZE_8BIT,
		            &adc_select_1R,
		            1,                    // Writing 1 byte
		            100);

  status = HAL_I2C_Mem_Write(&hi2c4, // Set ADC1 source
	            PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
				PCM1865_ADC2_IP_SEL_L,                 // Register 0x00 is the page-select register
	            I2C_MEMADD_SIZE_8BIT,
	            &adc_select_2L,
	            1,                    // Writing 1 byte
	            100);
	status = HAL_I2C_Mem_Write(&hi2c4,
		            PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
					PCM1865_ADC2_IP_SEL_L,                 // Register 0x00 is the page-select register
		            I2C_MEMADD_SIZE_8BIT,
		            &adc_select_2R,
		            1,                    // Writing 1 byte
		            100);

  
	// status = HAL_I2C_Mem_Read(&hi2c4, // Set ADC2 source
	// 	            PCM1865_I2C_ADDR,     // 7-bit device address << 1
	// 				PCM1865_ADC2_IP_SEL_L,                 // Register 0x00 is the page-select register
	// 	            I2C_MEMADD_SIZE_8BIT,
	// 	            &regRead1,
	// 	            1,                    // Writing 1 byte
	// 	            100);
	// 	status = HAL_I2C_Mem_Read(&hi2c4,
	// 		            PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
	// 					PCM1865_ADC2_IP_SEL_L,                 // Register 0x00 is the page-select register
	// 		            I2C_MEMADD_SIZE_8BIT,
	// 		            &regRead2,
	// 		            1,                    // Writing 1 byte
	// 		            100);

		printf("adc2 input select 1: %x\r\n", regRead1);
			printf("adc2 input select 2: %x\r\n", regRead2);



	uint8_t tdm_offset_1 = 0;
	uint8_t tdm_offset_2 = 128;

	status = HAL_I2C_Mem_Write(&hi2c4, // Set TDM offset
		            PCM1865_I2C_ADDR,     // 7-bit device address << 1
					PCM1865_TX_TDM_OFFSET ,                 // Register 0x00 is the page-select register
		            I2C_MEMADD_SIZE_8BIT,
		            &tdm_offset_1,
		            1,                    // Writing 1 byte
		            100);

	status = HAL_I2C_Mem_Write(&hi2c4,
					PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
					PCM1865_TX_TDM_OFFSET ,                 // Register 0x00 is the page-select register
					I2C_MEMADD_SIZE_8BIT,
					&tdm_offset_2,
					1,                    // Writing 1 byte
					100);

	status = HAL_I2C_Mem_Read(&hi2c4, // Set TDM offset
		            PCM1865_I2C_ADDR,     // 7-bit device address << 1
					PCM1865_TX_TDM_OFFSET ,                 // Register 0x00 is the page-select register
		            I2C_MEMADD_SIZE_8BIT,
		            &regRead1,
		            1,                    // Writing 1 byte
		            100);

	status = HAL_I2C_Mem_Read(&hi2c4,
					PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
					PCM1865_TX_TDM_OFFSET ,                 // Register 0x00 is the page-select register
					I2C_MEMADD_SIZE_8BIT,
					&regRead2,
					1,                    // Writing 1 byte
					100);

	printf("tdm offset select 1: %x\r\n", regRead1);
	printf("tdm offset select 2: %x\r\n", regRead2);




	uint8_t tdm_mode = 0x43;

	status = HAL_I2C_Mem_Write(&hi2c4, // Set TDM mode, 0x0B
			            PCM1865_I2C_ADDR,     // 7-bit device address << 1
						PCM1865_FMT,                 // Register 0x00 is the page-select register
			            I2C_MEMADD_SIZE_8BIT,
			            &tdm_mode,
			            1,                    // Writing 1 byte
			            100);
	status = HAL_I2C_Mem_Write(&hi2c4,
					PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
					PCM1865_FMT,                 // Register 0x00 is the page-select register
					I2C_MEMADD_SIZE_8BIT,
					&tdm_mode,
					1,                    // Writing 1 byte
					100);





	uint8_t tdm_channels = 1;   // 0x0C

	status = HAL_I2C_Mem_Write(&hi2c4, // Set 4 channel TDM
				            PCM1865_I2C_ADDR,     // 7-bit device address << 1
							PCM1865_TDM_OSEL,                 // Register 0x00 is the page-select register
				            I2C_MEMADD_SIZE_8BIT,
				            &tdm_channels,
				            1,                    // Writing 1 byte
				            100);
	status = HAL_I2C_Mem_Write(&hi2c4,
					PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
					PCM1865_TDM_OSEL ,                 // Register 0x00 is the page-select register
					I2C_MEMADD_SIZE_8BIT,
					&tdm_channels,
					1,                    // Writing 1 byte
					100);





	uint8_t clock_config = 0x01; // Set clock config to auto

		status = HAL_I2C_Mem_Write(&hi2c4,
					            PCM1865_I2C_ADDR,     // 7-bit device address << 1
								PCM1865_CLK_CFG0,                 // Register 0x00 is the page-select register
					            I2C_MEMADD_SIZE_8BIT,
					            &clock_config,
					            1,                    // Writing 1 byte
					            100);
		status = HAL_I2C_Mem_Write(&hi2c4,
						PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
						PCM1865_CLK_CFG0 ,                 // Register 0x00 is the page-select register
						I2C_MEMADD_SIZE_8BIT,
						&clock_config,
						1,                    // Writing 1 byte
						100);



		status = HAL_I2C_Mem_Read(&hi2c4, // Read
				            PCM1865_I2C_ADDR,     // 7-bit device address << 1
							PCM1865_CLK_CFG0,                 // Register 0x00 is the page-select register
				            I2C_MEMADD_SIZE_8BIT,
				            &regRead1,
				            1,                    // Writing 1 byte
				            100);

	status = HAL_I2C_Mem_Read(&hi2c4, // Read
		            PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
					PCM1865_CLK_CFG0,                 // Register 0x00 is the page-select register
		            I2C_MEMADD_SIZE_8BIT,
		            &regRead2,
		            1,                    // Writing 1 byte
		            100);
	printf("adc init complete\r\n");
	printf("clk config1 (hex): %x\r\n", regRead1);
	printf("clk config2 (hex): %x\r\n", regRead2);



	// Set PLL
	uint8_t PLL_config = 0x01; // Set clock config to auto

//			status = HAL_I2C_Mem_Write(&hi2c4,
//						            PCM1865_I2C_ADDR,     // 7-bit device address << 1
//									PCM1865_PLL_STATE,                 // Register 0x00 is the page-select register
//						            I2C_MEMADD_SIZE_8BIT,
//						            &PLL_config,
//						            1,                    // Writing 1 byte
//						            100);
	return status;

}


// Buffers to store register values
uint8_t PCM1865_0_Reg[PCM1865_Reg_Size];
uint8_t PCM1865_1_Reg[PCM1865_Reg_Size];

// Variables to store extracted register values
uint8_t PLL_Register_0;
uint8_t PLL_Locked_0;
uint8_t PLL_Locked_1;
uint8_t SYS_Status_0;
uint8_t SYS_Status_1;
uint8_t SampleRate_0;
uint8_t SampleRate_1;
uint8_t BCKRate_0;
uint8_t BCKRate_1;
uint8_t SCKRate_0;
uint8_t SCKRate_1;
uint8_t ERROR_0;
uint8_t ERROR_1;
uint8_t PWR_Status_0;
uint8_t PWR_Status_1;


void read_pcm1865_registers(void) {
    HAL_StatusTypeDef status;

    // Read all registers from both PCM1865 devices
    for (int Offset = 0; Offset < 128; Offset++) {
        status = HAL_I2C_Mem_Read(&hi2c4, PCM1865_I2C_ADDR, Offset, 1, &PCM1865_0_Reg[Offset], 1, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            // Handle error for PCM1865_0 (e.g., printf, error flag)
            printf("Error reading PCM1865_0 register %d\r\n", Offset);
            // ... error handling ...
        }

        status = HAL_I2C_Mem_Read(&hi2c4, PCM1865_I2C_ADDR_2, Offset, 1, &PCM1865_1_Reg[Offset], 1, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            // Handle error for PCM1865_1 (e.g., printf, error flag)
//            printf("Error reading PCM1865_1 register %d\r\n", Offset);
            // ... error handling ...
        }
    }

    // Extract specific register values

    PLL_Register_0 = PCM1865_0_Reg[40];

    PLL_Locked_0 = PCM1865_0_Reg[40] >> 4; // PLL locked
    PLL_Locked_1 = PCM1865_1_Reg[40] >> 4;

    SYS_Status_0 = PCM1865_0_Reg[114]; // System status
    SYS_Status_1 = PCM1865_1_Reg[114];

    SampleRate_0 = PCM1865_0_Reg[115]; // Sample rate
    SampleRate_1 = PCM1865_1_Reg[115];

    BCKRate_0 = PCM1865_0_Reg[116] >> 4; // BCK rate
    BCKRate_1 = PCM1865_1_Reg[116] >> 4;

    SCKRate_0 = PCM1865_0_Reg[116] & 0x0F; // SCK Rate
    SCKRate_1 = PCM1865_1_Reg[116] & 0x0F;

    ERROR_0 = PCM1865_0_Reg[117]; // Error code
    ERROR_1 = PCM1865_1_Reg[117];

    PWR_Status_0 = PCM1865_0_Reg[120]; // Power status
    PWR_Status_1 = PCM1865_1_Reg[120];

    // Print the extracted values


   printf("PLL_Register_0: %d\r\n", PLL_Register_0);


   printf("PLL_Locked_0: %d\r\n", PLL_Locked_0);
   printf("PLL_Locked_1: %d\r\n", PLL_Locked_1);
   printf("SYS_Status_0: %d\r\n", SYS_Status_0);
   printf("SYS_Status_1: %d\r\n", SYS_Status_1);
   printf("SampleRate_0: %d\r\n", SampleRate_0);
   printf("SampleRate_1: %d\r\n", SampleRate_1);
   printf("BCKRate_0: %d\r\n", BCKRate_0);
   printf("BCKRate_1: %d\r\n", BCKRate_1);
   printf("SCKRate_0: %d\r\n", SCKRate_0);
   printf("SCKRate_1: %d\r\n", SCKRate_1);
   printf("ERROR_0: %d\r\n", ERROR_0);
   printf("ERROR_1: %d\r\n", ERROR_1);
   printf("PWR_Status_0: %d\r\n", PWR_Status_0);
   printf("PWR_Status_1: %d\r\n", PWR_Status_1);

    // You can now use the extracted values (e.g., print them, use them for logic)
    // Example: printf("PLL_Locked_0: %d\r\n", PLL_Locked_0);
}

//HAL_StatusTypeDef write_ADC_registers();


volatile uint8_t bufferFull = 0;

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    // This callback is invoked when the DMA finishes a full buffer transfer.
    // You can process the data here, or set a flag for your main loop.
    printf("Full buffer received\r\n");
    bufferFull = 1;
    for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
          {
              printf("Sample %d: %d\r\n", i, audioRxBuffer[i] >> 8);
          }

    HAL_SAI_DMAPause(&hsai_BlockB1);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    // Called at half buffer, useful for double-buffered processing.
//    printf("Half buffer received\r\n");
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
//  if ( timeout < 0 )
//  {
//  Error_Handler();
//  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
//while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
//if ( timeout < 0 )
//{
//Error_Handler();
//}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C4_Init();
  MX_SAI1_Init();
  MX_SPI3_Init();
  MX_SAI2_Init();
  /* USER CODE BEGIN 2 */
	ADC_INIT();

	I2C_Scan(&hi2c4);

  int number = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t regData;
  uint8_t pageVal = 0;  // Suppose we want Page 3
  HAL_StatusTypeDef status2;
  status2 = HAL_I2C_Mem_Read(&hi2c4,
                    PCM1865_I2C_ADDR,     // 7-bit device address << 1
                    0x00,                 // Register 0x00 is the page-select register
                    I2C_MEMADD_SIZE_8BIT,
                    &regData,
                    1,                    // Writing 1 byte
                    100);


  if (status2 != HAL_OK)
    {
  	  printf("i2c read1 fail\r\n");
    }
  else
  {
	  printf("i2c read1 success\r\n");
  }

  status2 = HAL_I2C_Mem_Read(&hi2c4,
                      PCM1865_I2C_ADDR_2,     // 7-bit device address << 1
                      0x00,                 // Register 0x00 is the page-select register
                      I2C_MEMADD_SIZE_8BIT,
                      &regData,
                      1,                    // Writing 1 byte
                      100);
  if (status2 != HAL_OK)
  {
	  printf("i2c read2 fail\r\n");
  }
  else
    {
  	  printf("i2c read2 success\r\n");
    }
  uint32_t error = 0;

  static uint16_t dummyData[2] = {3, 3};
  if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, &dummyData, 2) != HAL_OK)
        {
            // Handle error if needed
  	  	  printf("HAL_SAI_Transmit_DMA failed A1\r\n");
  	  	  error = HAL_SAI_GetError(&hsai_BlockA1);
        }
  if (HAL_SAI_Receive_DMA(&hsai_BlockB1, &audioRxBuffer, AUDIO_BUFFER_SIZE) != HAL_OK)
      {
          // Handle error if needed
	  	  printf("HAL_SAI_Receive_DMA failed B1\r\n");
	  	  error = HAL_SAI_GetError(&hsai_BlockB1);
      }

//
//  GenerateSineWaveTable(15000);


//  if (HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)sineTable, 440) != HAL_OK)
//       {
//           // Handle error (e.g., call Error_Handler())
//           printf("HAL_SAI_Receive_DMA2 failed\r\n");
// //          error = HAL_SAI_GetError(&hsai_BlockB2);
//
// //          while(1);
//       }
 //
//
//  if (HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)Wave_LUT, 128) != HAL_OK)
//       {
//           // Handle error (e.g., call Error_Handler())
//           printf("HAL_SAI_Receive_DMA2 failed\r\n");
// //          error = HAL_SAI_GetError(&hsai_BlockB2);
//
// //          while(1);
//       }

//  if (HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)sine_wave_table_128_samples, 128) != HAL_OK)
//       {
//           // Handle error (e.g., call Error_Handler())
//           printf("HAL_SAI_Receive_DMA2 failed\r\n");
// //          error = HAL_SAI_GetError(&hsai_BlockB2);
//
// //          while(1);
//       }
//
//  fill_buffer_with_square_wave(audioPlaybackBuffer, AUDIO_BUFFER_SIZE * 2);
//
//  if (HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)audioPlaybackBuffer, 880) != HAL_OK)
//      {
//          // Handle error (e.g., call Error_Handler())
//          printf("HAL_SAI_Receive_DMA2 failed\r\n");
////          error = HAL_SAI_GetError(&hsai_BlockB2);
//
////          while(1);
//      }
//
  fill_buffer_with_triangle_wave(triangleBuffer, TRIANGLE_BUFFER_SIZE);

  if (HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)triangleBuffer, TRIANGLE_BUFFER_SIZE * 2) != HAL_OK)
      {
          // Handle error (e.g., call Error_Handler())
          printf("HAL_SAI_Receive_DMA2 failed\r\n");
//          error = HAL_SAI_GetError(&hsai_BlockB2);

//          while(1);
      }

//  single_value_dma_buffer[0] = DC_VALUE_TO_TRANSMIT; // Left Channel
//      single_value_dma_buffer[1] = DC_VALUE_TO_TRANSMIT; // Right Channel
//      if (HAL_SAI_Transmit_DMA(&hsai_BlockB2, (uint8_t*)single_value_dma_buffer, 2) != HAL_OK)
//           {
//               // Handle error (e.g., call Error_Handler())
//               printf("HAL_SAI_Receive_DMA2 failed\r\n");
//               error = HAL_SAI_GetError(&hsai_BlockB2);
//
////               while(1);
//           }
//  while (bufferFull == 0)
//      {
//          // Optionally, you could put the MCU to sleep or toggle an LED here.
//      }
//
//      // Optionally, if using circular DMA, abort reception to stop further transfers:
//      HAL_SAI_DMAPause(&hsai_BlockB1);

      // At this point, audioRxBuffer contains the full set of samples.
      // For example, print out the received values over UART:
//      for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
//      {
//          printf("Sample %d: %u\r\n", i, audioRxBuffer[i]);
//      }

  read_pcm1865_registers();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x307075B1;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA1.FrameInit.FrameLength = 256;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 128;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockA1.SlotInit.SlotNumber = 8;
  hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB1.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockB1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockB1.FrameInit.FrameLength = 256;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 128;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockB1.SlotInit.SlotNumber = 8;
  hsai_BlockB1.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockB2.Init.DataSize = SAI_DATASIZE_24;
  hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB2.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB2.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockB2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockB2.FrameInit.FrameLength = 64;
  hsai_BlockB2.FrameInit.ActiveFrameLength = 32;
  hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
  hsai_BlockB2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB2.SlotInit.FirstBitOffset = 0;
  hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockB2.SlotInit.SlotNumber = 2;
  hsai_BlockB2.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Output_GPIO_Port, LED_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Output_Pin */
  GPIO_InitStruct.Pin = LED_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Output_GPIO_Port, &GPIO_InitStruct);

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
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
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
