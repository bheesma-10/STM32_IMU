/**
  * @file           CompFilter.h
  * @brief          header file for complimentary filter
  */
#ifndef _COMPFILTER_H_
#define _COMPFILTER_H_

#include "MPUXX50.h"

/*structure for handling filter data*/
typedef struct{
	float deltaT;
	float alpha;
	float accelAngleX, accelAngleY;
	float gyroAngleX, gyroAngleY;
	float compAngleX, compAngleY;
}Filter_t;

/*functions*/
void Filter_Init(Filter_t* filter,float deltaT, float alpha);
void Filter_calcAttitude(Filter_t* filter, IMU_t *imu);

#endif /* _COMPFILTER_H_ */
