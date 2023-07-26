/**
  * @file           IMU_I2C.h
  * @brief          header file for i2c operations with IMU
  */
#ifndef IMU_INC_IMU_I2C_H_
#define IMU_INC_IMU_I2C_H_

#include "main.h"

/*structure for handling i2c*/
typedef struct{
	I2C_TypeDef *Instance;
	uint8_t* pRxdata;
	uint8_t* pTxdata;
	uint16_t pRxXfercount;
	uint16_t pTxXfercount;
	uint8_t  errorcode;
}I2CHandle_t;

/*functions*/
void IMU_I2C_Init(I2CHandle_t *hi2c);
void I2C_Read_Reg(I2CHandle_t *hi2c,uint8_t reg,uint8_t* reg_data,uint16_t count,uint16_t timeout);
void I2C_Write_Reg(I2CHandle_t *hi2c,uint8_t reg,uint8_t* reg_data,uint16_t timeout);

#endif /* IMU_INC_IMU_I2C_H_ */
