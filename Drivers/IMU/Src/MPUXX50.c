/**
  * @file           MPUxx50.c
  * @brief          source file for MPUxx50 basic operations
  */
#include "MPUXX50.h"
#include <string.h>

/**
 * @brief Set the IMU address, check for connection, reset IMU, and set full range scale.
 * @param hi2c pointer to I2C structure 
 * @param imu  pointer to MPUxx50 structure
 * @param addr Hex address based on AD0 pin - 0x68 low or 0x69 high.
 * @param aScale Set accelerometer full scale range: 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
 * @param gScale Set gyroscope full scale range: 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
*/

uint8_t MPU_begin(I2CHandle_t *hi2c, IMU_t *imu, uint8_t addr, uint8_t aScale, uint8_t gScale)
{
	//set address
	imu->dev_address = addr;

    // Initialize variables
    uint8_t check = 0U;

    // Confirm device
    I2C_Read_Reg(hi2c, REG_WHO_AM_I, &check, 1,100);

    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS))
    {
        // Startup / reset the sensor
    	//exit sleep
	    uint8_t pwr = 0x00U;
    	I2C_Write_Reg(hi2c, REG_PWR_MGMT_1, &pwr, 1,100);

    	// Set the full scale ranges
        MPU_writeAccFullScaleRange(hi2c, imu, aScale);
        MPU_writeGyroFullScaleRange(hi2c, imu, gScale);

        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief Set the accelerometer full scale range.
 * @param hi2c pointer to I2C structure 
 * @param imu  pointer to MPUxx50 structure  
 * @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
*/
void MPU_writeAccFullScaleRange(I2CHandle_t *hi2c, IMU_t *imu, uint8_t aScale)
{
	//clear scale value
	uint8_t data = 0;
	I2C_Read_Reg(hi2c, REG_ACCEL_CONFIG, &data, 1,100);
	data = data & ~(0x3<<3U);
	I2C_Write_Reg(hi2c, REG_ACCEL_CONFIG, &data, 1,100);

	// Set the value
    switch (aScale)
    {
    case AFSR_2G:{
        imu->aScaleFactor = 16384.0;
        }
        break;
    case AFSR_4G:{
        imu->aScaleFactor = 8192.0;
        }
        break;
    case AFSR_8G:{
        imu->aScaleFactor = 4096.0;
        }
        break;
    case AFSR_16G:{
        imu->aScaleFactor = 2048.0;
        }
        break;
    default:{
        imu->aScaleFactor = 8192.0;
        }
        break;
    }

    data = (aScale<<3U);
    I2C_Write_Reg(hi2c, REG_ACCEL_CONFIG, &(data), 1,100);

}

/**
 * @brief Set the gyroscope full scale range.
 * @param hi2c pointer to I2C structure 
 * @param imu  pointer to MPUxx50 structure  
 * @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
*/
void MPU_writeGyroFullScaleRange(I2CHandle_t *hi2c, IMU_t *imu, uint8_t gScale)
{
	//clear scale value
	uint8_t data = 0;
	I2C_Read_Reg(hi2c, REG_GYRO_CONFIG, &data, 1,100);
	data = data & ~(0x3<<3U);
	I2C_Write_Reg(hi2c, REG_GYRO_CONFIG, &data, 1,100);

    // Set the value
    switch (gScale)
    {
    case GFSR_250DPS:{
        imu->gScaleFactor = 131.0;
        }
        break;
    case GFSR_500DPS:{
        imu->gScaleFactor = 65.5;
        }
        break;
    case GFSR_1000DPS:{
        imu->gScaleFactor = 32.8;
        }
        break;
    case GFSR_2000DPS:{
        imu->gScaleFactor = 16.4;
        }
        break;
    default:{
        imu->gScaleFactor = 65.5;
        }
        break;
    }

    data = (gScale<<3U);
    I2C_Write_Reg(hi2c, REG_GYRO_CONFIG, &(data), 1,100);
}

/**
 * @brief Read raw data from IMU.
 * @param hi2c pointer to I2C structure 
 * @param imu  pointer to MPUxx50 structure  
*/
void MPU_readRawData(I2CHandle_t *hi2c, IMU_t *imu)
{
    // Init buffer
    uint8_t gyro_buf[6];
    uint8_t accel_buf[6];

    // read gyro raw data
    get_rawgyro(hi2c, gyro_buf);

    //read accel raw data
    get_rawaccel(hi2c, accel_buf);

    // Bit shift the data
    imu->rawax = (accel_buf[0] << 8) | accel_buf[1];
    imu->raway = (accel_buf[2] << 8) | accel_buf[3];
    imu->rawaz = (accel_buf[4] << 8) | accel_buf[5];

    imu->rawgx = (gyro_buf[0] << 8) | gyro_buf[1];
    imu->rawgy = (gyro_buf[2] << 8) | gyro_buf[3];
    imu->rawgz = (gyro_buf[4] << 8) | gyro_buf[5];
}

/**
 * @brief Read raw gyro data from IMU 
 * @param hi2c pointer to I2C structure 
 * @param gyro_buf buffer to store gyro data from IMU
*/
void get_rawgyro(I2CHandle_t *hi2c, uint8_t* gyro_buf){
    memset(gyro_buf,0,6);
    uint8_t reg = REG_GYRO_XOUT_H;
    for(int index=0;index<6;index++){
    	I2C_Read_Reg(hi2c, reg, &gyro_buf[index], 1,100);
    	reg++;
    }
}

/**
 * @brief Read raw accel data from IMU
 * @param hi2c pointer to I2C structure 
 * @param accel_buf buffer to store accel data from IMU
*/
void get_rawaccel(I2CHandle_t *hi2c, uint8_t* accel_buf){
    memset(accel_buf,0,6);
    uint8_t reg = REG_ACCEL_XOUT_H;
    for(int index=0;index<6;index++){
        I2C_Read_Reg(hi2c,reg , &accel_buf[index], 1,100);
        reg++;
    }
}

/**
 * @brief Find offsets for each axis of gyroscope.
 * @param hi2c pointer to I2C structure 
 * @param imu  Pointer to MPUxx50 structure  
 * @param numCalPoints Number of data points to average.
*/
void MPU_calibrateGyro(I2CHandle_t *hi2c, IMU_t *imu, uint16_t numCalPoints)
{
    // Init
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        MPU_readRawData(hi2c, imu);
        x += imu->rawgx;
        y += imu->rawgy;
        z += imu->rawgz;
        delay_ms(3);
    }

    // Average the saved data points to find the gyroscope offset
    imu->gx_cal = (float)x / (float)numCalPoints;
    imu->gy_cal = (float)y / (float)numCalPoints;
    imu->gz_cal = (float)z / (float)numCalPoints;
}

/**
 * @brief Calculate the real world sensor values.
 * @param hi2c pointer to I2C structure 
 * @param imu  Pointer to MPUxx50 structure  
*/
void MPU_readscaledData(I2CHandle_t *hi2c, IMU_t *imu)
{
    // Get raw values from the IMU
    MPU_readRawData(hi2c, imu);

    // Convert accelerometer values to g's
    imu->ax = (imu->rawax / imu->aScaleFactor);
    imu->ay = (imu->raway / imu->aScaleFactor);
    imu->az = (imu->rawaz / imu->aScaleFactor);

    // Compensate for gyro offset
    imu->gx = (imu->rawgx - imu->gx_cal);
    imu->gy = (imu->rawgy - imu->gy_cal);
    imu->gz = (imu->rawgz - imu->gz_cal);

    // Convert gyro values to deg/s
    imu->gx /= imu->gScaleFactor;
    imu->gy /= imu->gScaleFactor;
    imu->gz /= imu->gScaleFactor;
}
