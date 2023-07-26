/**
  * @file           IMU_UartMsg.h
  * @brief          header file for uart operations
  */
#ifndef IMU_IMU_UARTMSG_H_
#define IMU_IMU_UARTMSG_H_

#include "main.h"

enum{
	UART_ERROR_NONE,
};

/*structure for handling uart*/
typedef struct{
	USART_TypeDef *Instance;
	uint8_t *pTxdata;
	uint8_t *pRxdata;
	uint16_t pTxXfersize;
	uint16_t pTxXfercount;
	uint16_t pRxXfersize;
	uint16_t pRxXfercount;
	uint8_t  errorcode;
}UartHandle_t;

/*functions*/
void IMU_Uart_Init(UartHandle_t *huart);
void Uart_TransmitData(UartHandle_t *huart,uint8_t* pTxdata,uint16_t size);

#endif /* IMU_IMU_UARTMSG_H_ */
