/**
  * @file           MPUXX50.h
  * @brief          header file for MPUxx50 operations
  */
#ifndef MPUXX50_H_
#define MPUXX50_H_

// Libs
#include <stdint.h>
#include <math.h>
#include "IMU_I2C.h"

// Constants
#define RAD2DEG 57.2957795131

// Register Defines
#define WHO_AM_I_6050_ANS 0x68
#define WHO_AM_I_9250_ANS 0x71
#define REG_WHO_AM_I          0x75
#define REG_GYRO_CONFIG       0x1B
#define REG_ACCEL_CONFIG      0x1C
#define REG_PWR_MGMT_1        0x6B
#define REG_ACCEL_XOUT_H      0x3B
#define REG_GYRO_XOUT_H       0x43
#define I2C_TIMOUT_MS     1000

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

/*structure for handling MPUxx50*/
typedef struct{
	//sensor 7bit address
	uint8_t dev_address;

	//raw IMU data values
	int16_t rawax;
	int16_t raway;
	int16_t	rawaz;
	int16_t rawgx;
	int16_t rawgy;
	int16_t rawgz;

	//raw avg. gyro data for calibration
	float gx_cal;
	float gy_cal;
	float gz_cal;

	//scale selected
	float aScaleFactor;
	float gScaleFactor;

	//scaled IMU data values
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;

}IMU_t;

// Functions
uint8_t MPU_begin(I2CHandle_t *hi2c,IMU_t *imu, uint8_t addr, uint8_t aScale, uint8_t gScale);
void MPU_calibrateGyro(I2CHandle_t *hi2c, IMU_t *imu, uint16_t numCalPoints);
void MPU_readRawData(I2CHandle_t *hi2c, IMU_t *imu);
void MPU_readscaledData(I2CHandle_t *hi2c, IMU_t *imu);
void MPU_writeGyroFullScaleRange(I2CHandle_t *hi2c, IMU_t *imu, uint8_t gScale);
void MPU_writeAccFullScaleRange(I2CHandle_t *hi2c, IMU_t *imu, uint8_t aScale);
void get_rawgyro(I2CHandle_t *hi2c, uint8_t* gyro_buf);
void get_rawaccel(I2CHandle_t *hi2c, uint8_t* accel_buf);

#endif /* MPUXX50_H_ */
