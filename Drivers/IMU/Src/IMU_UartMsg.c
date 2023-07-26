/**
  * @file           IMU_UartMsg.c
  * @brief          source file for Uart operations handling
  */
#include "IMU_UartMsg.h"

#define UART_USED USART3

/**
 * @brief initialize uart instance
 * @param huart pointer to Uart structure
*/
void IMU_Uart_Init(UartHandle_t *huart){
	huart->Instance = UART_USED;
}

/**
 * @brief read registers over Uart protocol
 * @param hi2c pointer to Uart structure
 * @param pTxdata pointer to data that has to be transmitted
 * @param size number of bytes to be transmitted
*/
void Uart_TransmitData(UartHandle_t *huart,uint8_t* pTxdata,uint16_t size){

	/*initialize uart members*/
	huart->pTxdata = pTxdata;
	huart->pTxXfercount = size;
	huart->pTxXfersize = size;
	huart->errorcode = UART_ERROR_NONE;

	while((huart->pTxXfercount)>0){
		huart->pTxXfercount--;

		while(!LL_USART_IsActiveFlag_TXE(huart->Instance)){
			//wait for TXE bit to go high
		}

		LL_USART_TransmitData8(huart->Instance, *(huart->pTxdata++)&(uint8_t)0xFF);
	}

	while(!LL_USART_IsActiveFlag_TC(huart->Instance)){
		//wait for TC bit to go high
	}
}


