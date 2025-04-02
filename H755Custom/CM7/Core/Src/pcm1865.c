///*
// * pcm1865.c
// *
// *  Created on: Apr 1, 2025
// *      Author: joshuayu
// */
//#include "pcm1865.h"
//#include "main.h"
//
// #define PCM1865_I2C_ADDR   (0x4A << 1)   // Adjust this based on your hardware strap
// #define PCM1865_I2C_ADDR_2   (0x4B << 1)   // Adjust this based on your hardware strap
//
// #define PCM1865_RESET               (0x00)
// #define PCM1865_PGA_VAL_CH1_L       (0x01)
// #define PCM1865_PGA_VAL_CH1_R       (0x02)
// #define PCM1865_PGA_VAL_CH2_L       (0x03)
// #define PCM1865_PGA_VAL_CH2_R       (0x04)
// #define PCM1865_ADC2_IP_SEL_L       (0x08) // Select input to route to ADC2 left input.
// #define PCM1865_ADC2_IP_SEL_R       (0x09) // Select input to route to ADC2 right input.
// #define PCM1865_FMT                 (0x0B) // RX_WLEN, TDM_LRCLK_MODE, TX_WLEN, FMT
// #define PCM1865_TDM_OSEL            (0x0C)
// #define PCM1865_TX_TDM_OFFSET       (0x0D)
// #define PCM1865_CLK_CFG0            (0x20) // Basic clock config.
// #define PCM1865_PLL_STATE			 (0x28)
// #define PCM1865_PWR_STATE           (0x70) // Power down, Sleep, Standby
//
// #define PCM1865_VINSEL1             (0x06) // Analog input selection 1
// #define PCM1865_VINSEL2             (0x07) // Analog input selection 2
// #define PCM1865_ADCOUTSEL1          (0x08) // ADC Output selection 1
// #define PCM1865_ADCOUTSEL2          (0x09) // ADC Output selection 2
//
///**
// * @brief  Write a value to a PCM1865 register
// * @param  hi2c: Pointer to I2C handle
// * @param  device_addr: I2C address of the PCM1865
// * @param  reg: Register address
// * @param  value: Value to be written
// * @retval HAL status
// */
//HAL_StatusTypeDef PCM1865_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg, uint8_t value)
//{
//    uint8_t data[2];
//    data[0] = reg;
//    data[1] = value;
//
//    return HAL_I2C_Mem_Write(hi2c, device_addr, data, 1, HAL_MAX_DELAY);
//}
//
///**
// * @brief  Read a value from a PCM1865 register
// * @param  hi2c: Pointer to I2C handle
// * @param  device_addr: I2C address of the PCM1865
// * @param  reg: Register address
// * @param  value: Pointer to value read from register
// * @retval HAL status
// */
//HAL_StatusTypeDef PCM1865_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg, uint8_t *value)
//{
//    HAL_StatusTypeDef status;
//
//    status = HAL_I2C_Mem_Write(hi2c, device_addr, &reg, 1, HAL_MAX_DELAY);
//    if (status != HAL_OK)
//        return status;
//
//    return HAL_I2C_Master_Receive(hi2c, device_addr, value, 1, HAL_MAX_DELAY);
//}
//
///**
// * @brief  Reset the PCM1865
// * @param  hi2c: Pointer to I2C handle
// * @param  device_addr: I2C address of the PCM1865
// * @retval HAL status
// */
//HAL_StatusTypeDef PCM1865_Reset(I2C_HandleTypeDef *hi2c, uint8_t device_addr)
//{
//    return PCM1865_WriteReg(hi2c, device_addr, PCM1865_RESET, 0x01);
//}
//
///**
// * @brief  Configure PCM1865 with specific settings
// * @param  hi2c: Pointer to I2C handle
// * @param  device_addr: I2C address of the PCM1865
// * @param  is_second_device: Set to 1 for second device, 0 for first device
// * @retval HAL status
// */
//HAL_StatusTypeDef PCM1865_Configure_Simple(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t is_second_device)
//{
//    HAL_StatusTypeDef status;
//
//    // 1. Reset the device (Write 0xFE to Reg 0x00)
//    status = PCM1865_WriteReg(hi2c, device_addr, PCM1865_RESET, 0xFE);
//    if (status != HAL_OK) return status;
//
//    // Small delay after reset
//    HAL_Delay(10);
//
//    // 2. Configure differential inputs (Write 0x50 0x50 0x60 0x60 to Reg 0x06 0x07 0x08 0x09)
//    status = PCM1865_WriteReg(hi2c, device_addr, PCM1865_VINSEL1, 0x50);
//    if (status != HAL_OK) return status;
//
//    status = PCM1865_WriteReg(hi2c, device_addr, PCM1865_VINSEL2, 0x50);
//    if (status != HAL_OK) return status;
//
//    status = PCM1865_WriteReg(hi2c, device_addr, PCM1865_ADCOUTSEL1, 0x60);
//    if (status != HAL_OK) return status;
//
//    status = PCM1865_WriteReg(hi2c, device_addr, PCM1865_ADCOUTSEL2, 0x60);
//    if (status != HAL_OK) return status;
//
//    // 3. Configure TDM based on device order
//    // Write 0x43 to Reg 0x0B (FMT)
//    status = PCM1865_WriteReg(hi2c, device_addr, PCM1865_FMT, 0x43);
//    if (status != HAL_OK) return status;
//
//    // Write 0x01 to Reg 0x0C (TDM_OSEL)
//    status = PCM1865_WriteReg(hi2c, device_addr, PCM1865_TDM_OSEL, 0x01);
//    if (status != HAL_OK) return status;
//
//    // Write 0x00 or 0x80 to Reg 0x0D (TX_TDM_OFFSET) depending on device order
//    uint8_t tdm_offset = is_second_device ? 0x80 : 0x00;
//    status = PCM1865_WriteReg(hi2c, device_addr, PCM1865_TX_TDM_OFFSET, tdm_offset);
//    if (status != HAL_OK) return status;
//
//    return HAL_OK;
//}
//
///**
// * @brief  Initialize PCM1865 ADCs with simplified configuration
// * @param  hi2c: Pointer to I2C handle
// * @retval HAL status
// */
//HAL_StatusTypeDef PCM1865_Init_Simple(I2C_HandleTypeDef *hi2c)
//{
//    HAL_StatusTypeDef status;
//
//    // Configure first device
//    status = PCM1865_Configure_Simple(hi2c, PCM1865_I2C_ADDR, 0);
//    if (status != HAL_OK) return status;
//
//    // Configure second device
//    status = PCM1865_Configure_Simple(hi2c, PCM1865_I2C_ADDR_2, 1);
//    if (status != HAL_OK) return status;
//
//    return HAL_OK;
//}
//
///**
// * @brief  Set PCM1865 gain for specified channel
// * @param  hi2c: Pointer to I2C handle
// * @param  device_addr: I2C address of the PCM1865
// * @param  channel: Channel register (PCM1865_PGA_VAL_CHx_y)
// * @param  gain: Gain value (0-255)
// * @retval HAL status
// */
//HAL_StatusTypeDef PCM1865_SetGain(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t channel, uint8_t gain)
//{
//    return PCM1865_WriteReg(hi2c, device_addr, channel, gain);
//}
//
///**
// * @brief  Enter low power mode
// * @param  hi2c: Pointer to I2C handle
// * @param  device_addr: I2C address of the PCM1865
// * @retval HAL status
// */
//HAL_StatusTypeDef PCM1865_Sleep(I2C_HandleTypeDef *hi2c, uint8_t device_addr)
//{
//    return PCM1865_WriteReg(hi2c, device_addr, PCM1865_PWR_STATE, 0x02); // Sleep mode
//}
//
///**
// * @brief  Exit low power mode
// * @param  hi2c: Pointer to I2C handle
// * @param  device_addr: I2C address of the PCM1865
// * @retval HAL status
// */
//HAL_StatusTypeDef PCM1865_WakeUp(I2C_HandleTypeDef *hi2c, uint8_t device_addr)
//{
//    return PCM1865_WriteReg(hi2c, device_addr, PCM1865_PWR_STATE, 0x00); // Power up
//}
//
//
