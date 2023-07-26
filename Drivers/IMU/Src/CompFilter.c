/**
  * @file           CompFilter.c
  * @brief          source file for complimentary filter 
  */
#include "CompFilter.h"
#include "MPUXX50.h"

/**
 * @brief initialize filter params
 * @param filter pointer to filter structure
 * @param alpha Set alpha value for the complementary filter (typically 0.98).
 * @param deltaT Set sampling rate in seconds determined by the timer interrupt.
*/
void Filter_Init(Filter_t* filter,float deltaT, float alpha){
	filter->deltaT = deltaT;
	filter->alpha = alpha;
	filter->compAngleX = 0.0f;
	filter->compAngleY = 0.0f;
	filter->accelAngleX = 0.0f;
	filter->accelAngleY = 0.0f;
	filter->gyroAngleX = 0.0f;
	filter->gyroAngleY = 0.0f;
}


/**
 * @brief Calculate the attitude of the sensor in degrees using a complementary filter.
 * @param filter pointer to filter structure
 * @param imu Pointer to MPUxx50 structure
 * 
*/
void Filter_calcAttitude(Filter_t* filter, IMU_t *imu)
{
    filter->accelAngleY = atan2(imu->ay, imu->az) * RAD2DEG; //pitch
    filter->accelAngleX = atan2(imu->ax, imu->az) * RAD2DEG;  //roll

    filter->gyroAngleX = -(imu->gy * filter->deltaT); //roll
    filter->gyroAngleY = (imu->gx * filter->deltaT);  //pitch

    filter->compAngleX = (filter->alpha * (filter->compAngleX + filter->gyroAngleX)) + ((1.0f - filter->alpha) * filter->accelAngleX);  //roll
    filter->compAngleY = (filter->alpha * (filter->compAngleY + filter->gyroAngleY)) + ((1.0f - filter->alpha) * filter->accelAngleY);   //pitch
}
