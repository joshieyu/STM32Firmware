/*
 * pcm1865.h
 *
 *  Created on: Apr 1, 2025
 *      Author: joshuayu
 */

#ifndef INC_PCM1865_H_
#define INC_PCM1865_H_

#include "stm32h7xx_hal.h"

// I2C addresses
#define PCM1865_I2C_ADDR   (0x4A << 1)   // Adjust based on hardware strap
#define PCM1865_I2C_ADDR_2 (0x4B << 1)   // Adjust based on hardware strap

// Register definitions
#define PCM1865_RESET               (0x00)
#define PCM1865_PGA_VAL_CH1_L       (0x01)
#define PCM1865_PGA_VAL_CH1_R       (0x02)
#define PCM1865_PGA_VAL_CH2_L       (0x03)
#define PCM1865_PGA_VAL_CH2_R       (0x04)
#define PCM1865_ADC2_IP_SEL_L       (0x08)
#define PCM1865_ADC2_IP_SEL_R       (0x09)
#define PCM1865_FMT                 (0x0B)
#define PCM1865_TDM_OSEL            (0x0C)
#define PCM1865_TX_TDM_OFFSET       (0x0D)
#define PCM1865_CLK_CFG0            (0x20)
#define PCM1865_PLL_STATE           (0x28)
#define PCM1865_PWR_STATE           (0x70)

// Function prototypes
HAL_StatusTypeDef PCM1865_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg, uint8_t value);
HAL_StatusTypeDef PCM1865_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg, uint8_t *value);
HAL_StatusTypeDef PCM1865_Reset(I2C_HandleTypeDef *hi2c, uint8_t device_addr);
HAL_StatusTypeDef PCM1865_Configure(I2C_HandleTypeDef *hi2c, uint8_t device_addr);
HAL_StatusTypeDef PCM1865_Configure_TDM(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t is_master, uint8_t tdm_slot);
HAL_StatusTypeDef PCM1865_Init_TDM(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef PCM1865_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef PCM1865_SetGain(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t channel, uint8_t gain);
HAL_StatusTypeDef PCM1865_Sleep(I2C_HandleTypeDef *hi2c, uint8_t device_addr);
HAL_StatusTypeDef PCM1865_WakeUp(I2C_HandleTypeDef *hi2c, uint8_t device_addr);

// New function prototypes
HAL_StatusTypeDef PCM1865_Configure_Simple(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t is_second_device);
HAL_StatusTypeDef PCM1865_Init_Simple(I2C_HandleTypeDef *hi2c);

#endif /* INC_PCM1865_H_ */




