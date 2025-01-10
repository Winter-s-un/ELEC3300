#include "HSMC5883L.h"

void HMC5883L_Init(I2C_HandleTypeDef *hi2c) 
{
    uint8_t configA = 0x70;
    HAL_I2C_Mem_Write(hi2c, HMC5883L_I2C_ADDRESS<<1, HMC5883L_REG_CONFIG_A, 1, &configA, 1, HAL_MAX_DELAY);
    uint8_t configB = 0xA0;
    HAL_I2C_Mem_Write(hi2c, HMC5883L_I2C_ADDRESS<<1, HMC5883L_REG_CONFIG_B, 1, &configB, 1, HAL_MAX_DELAY);
    uint8_t mode = 0x01;
    HAL_I2C_Mem_Write(hi2c, HMC5883L_I2C_ADDRESS<<1, HMC5883L_REG_MODE, 1, &mode, 1, HAL_MAX_DELAY);
}

void HMC5883L_Read(I2C_HandleTypeDef *hi2c, int16_t* magData) 
{
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(hi2c, HMC5883L_I2C_ADDRESS<<1, HMC5883L_REG_DATA_X_MSB, 1, &buffer, 6, HAL_MAX_DELAY);
    // HAL_Delay(10);
    // HAL_I2C_Mem_Read(hi2c, HMC5883L_I2C_ADDRESS, HMC5883L_REG_DATA_X_LSB, 1, &buffer[1], 1, HAL_MAX_DELAY);
    // HAL_Delay(10);
    // HAL_I2C_Mem_Read(hi2c, HMC5883L_I2C_ADDRESS, HMC5883L_REG_DATA_Z_MSB, 1, &buffer[2], 1, HAL_MAX_DELAY);
    // HAL_Delay(10);
    // HAL_I2C_Mem_Read(hi2c, HMC5883L_I2C_ADDRESS, HMC5883L_REG_DATA_Z_LSB, 1, &buffer[3], 1, HAL_MAX_DELAY);
    // HAL_Delay(10);
    // HAL_I2C_Mem_Read(hi2c, HMC5883L_I2C_ADDRESS, HMC5883L_REG_DATA_Y_MSB, 1, &buffer[4], 1, HAL_MAX_DELAY);
    // HAL_Delay(10);
    // HAL_I2C_Mem_Read(hi2c, HMC5883L_I2C_ADDRESS, HMC5883L_REG_DATA_Y_LSB, 1, &buffer[5], 1, HAL_MAX_DELAY);
    // HAL_Delay(10);
    Buffer[0] = buffer[0];
    Buffer[1] = buffer[1];
    Buffer[2] = buffer[2];
    Buffer[3] = buffer[3];
    Buffer[4] = buffer[4];
    Buffer[5] = buffer[5];

    magData[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    magData[1] = (int16_t)((buffer[4] << 8) | buffer[5]);
    magData[2] = (int16_t)((buffer[2] << 8) | buffer[3]);
}
