/**
  * @file           IMU_I2C.c
  * @brief          source file for I2C operation handling
  */
#include "IMU_I2C.h"

#define I2C_USED  I2C1
extern uint64_t counter_millis;

/**
 * @brief initialize i2c instance
 * @param hi2c pointer to I2C structure
*/
void IMU_I2C_Init(I2CHandle_t *hi2c){
	hi2c->Instance = I2C_USED;
}

/**
 * @brief read registers over I2C protocol
 * @param hi2c pointer to I2C structure
 * @param reg register to read from
 * @param reg_data data buffer to store read data
 * @param count number of bytes to be read
 * @param timeout time in ms to timeout if communication stopped
*/
void I2C_Read_Reg(I2CHandle_t *hi2c,uint8_t reg,uint8_t* reg_data,uint16_t count,uint16_t timeout){

    /*some variables*/
	uint8_t reg_addr = reg;
	uint64_t counter_ms = 0;
	hi2c->pTxdata = &reg_addr;
	hi2c->pRxdata = reg_data;
	hi2c->pTxXfercount = 0;
	hi2c->pRxXfercount = count;

	//check if peripheral enabled
	if(!LL_I2C_IsEnabled(hi2c->Instance)){
		LL_I2C_Enable(hi2c->Instance);
	}

	//Enable acknowledge
	SET_BIT((hi2c->Instance)->CR1,I2C_CR1_ACK);

	//send start
	SET_BIT((hi2c->Instance)->CR1,I2C_CR1_START);

	counter_ms = counter_millis;
	while(!READ_BIT((hi2c->Instance)->SR1,I2C_SR1_SB)){
		//wait till start condition generated
		if((counter_millis-counter_ms)>timeout)return;
	}

	//send slave address
	LL_I2C_TransmitData8(hi2c->Instance, (IMU_SLAVE_ADDR<<1U) & ~(0x01U)); //Write operation

	counter_ms = counter_millis;
	while(!LL_I2C_IsActiveFlag_ADDR(hi2c->Instance)){
		//wait until ADDR flag is set
		if((counter_millis-counter_ms)>timeout)return;
	}

	uint32_t tmpreg = (hi2c->Instance)->SR1;
	tmpreg = (hi2c->Instance)->SR2;
	unused(tmpreg);

	if(LL_I2C_IsActiveFlag_AF(hi2c->Instance)){
		//should not be here (ACK failure)
		CLEAR_BIT((hi2c->Instance)->SR1,I2C_SR1_AF);
		return;
	}

	counter_ms = counter_millis;
	while(!LL_I2C_IsActiveFlag_TXE(hi2c->Instance)){
		//wait until TXE flag is set
		if((counter_millis-counter_ms)>timeout)return;
	}

	//send register address
	LL_I2C_TransmitData8(hi2c->Instance, *(hi2c->pTxdata));
	hi2c->pTxdata++;
	hi2c->pTxXfercount--;
	reg_addr = reg + 1;
	hi2c->pTxdata = &reg_addr;

	counter_ms = counter_millis;
	while(!LL_I2C_IsActiveFlag_TXE(hi2c->Instance)){
		//wait until TXE flag is set
		if((counter_millis-counter_ms)>timeout)return;
	}

	if(LL_I2C_IsActiveFlag_AF(hi2c->Instance)){
		//should not be here (ACK failure)
		CLEAR_BIT((hi2c->Instance)->SR1,I2C_SR1_AF);
		return;
	}

	//send repeated start
	SET_BIT((hi2c->Instance)->CR1,I2C_CR1_START);

	counter_ms = counter_millis;
	while(!READ_BIT((hi2c->Instance)->SR1,I2C_SR1_SB)){
			//wait till repeated start condition generated
		if((counter_millis-counter_ms)>timeout)return;
	}

	//send slave address
	LL_I2C_TransmitData8(hi2c->Instance, (IMU_SLAVE_ADDR<<1U) | (0x01U)); //Read operation

	counter_ms = counter_millis;
	while(!LL_I2C_IsActiveFlag_ADDR(hi2c->Instance)){
		//wait until ADDR flag is set
		if((counter_millis-counter_ms)>timeout)return;
	}

	tmpreg = (hi2c->Instance)->SR1;
	tmpreg = (hi2c->Instance)->SR2;

	if(LL_I2C_IsActiveFlag_AF(hi2c->Instance)){
		//should not be here (ACK failure)
		CLEAR_BIT((hi2c->Instance)->SR1,I2C_SR1_AF);
		return;
	}

	//Disable Acknowledge
	CLEAR_BIT((hi2c->Instance)->CR1,I2C_CR1_ACK);


	while(hi2c->pRxXfercount>0U){

		counter_ms = counter_millis;
		while(!LL_I2C_IsActiveFlag_RXNE(hi2c->Instance)){
			//wait until RXNE flag is set
			if((counter_millis-counter_ms)>timeout)return;
		}

		//read data, increment buffer and update count
		*(hi2c->pRxdata) = LL_I2C_ReceiveData8(hi2c->Instance);
		hi2c->pRxdata++;
		hi2c->pRxXfercount--;

		if(READ_BIT(I2C1->SR1,I2C_SR1_BTF)){
			//read data, increment buffer and update count
			*(hi2c->pRxdata) = LL_I2C_ReceiveData8(hi2c->Instance);
			hi2c->pRxdata++;
			hi2c->pRxXfercount--;

		}

	}

	// Generate Stop
	SET_BIT((hi2c->Instance)->CR1, I2C_CR1_STOP);


}

/**
 * @brief write registers over I2C protocol
 * @param hi2c pointer to I2C structure
 * @param reg register to write to
 * @param reg_data data buffer to store data to be written
 * @param timeout time in ms to timeout if communication stopped
*/
void I2C_Write_Reg(I2CHandle_t *hi2c,uint8_t reg,uint8_t* reg_data,uint16_t timeout){

	uint64_t counter_ms = 0;
	hi2c->pTxdata = &reg;
	hi2c->pTxXfercount = count;

	//check if peripheral enabled
	if(!LL_I2C_IsEnabled(hi2c->Instance)){
		LL_I2C_Enable(hi2c->Instance);
	}

	//Enable acknowledge
	SET_BIT((hi2c->Instance)->CR1,I2C_CR1_ACK);

    //send start
	SET_BIT((hi2c->Instance)->CR1,I2C_CR1_START);

	counter_ms = counter_millis;
	while(!READ_BIT((hi2c->Instance)->SR1,I2C_SR1_SB)){
		//wait till start condition generated
		if((counter_millis-counter_ms)>timeout)return;
	}

	//send slave address
	LL_I2C_TransmitData8(hi2c->Instance, (IMU_SLAVE_ADDR<<1U) & ~(0x01U)); //Write operation

	counter_ms = counter_millis;
    while(!LL_I2C_IsActiveFlag_ADDR(hi2c->Instance)){
    	//wait until ADDR flag is set
    	if((counter_millis-counter_ms)>timeout)return;
    }

    uint32_t tmpreg = (hi2c->Instance)->SR1;
    tmpreg = (hi2c->Instance)->SR2;
    unused(tmpreg);

    if(LL_I2C_IsActiveFlag_AF(hi2c->Instance)){
    	//should not be here (ACK failure)
    	CLEAR_BIT((hi2c->Instance)->SR1,I2C_SR1_AF);
    	return;
    }

    counter_ms = counter_millis;
    while(!LL_I2C_IsActiveFlag_TXE(hi2c->Instance)){
		//wait until TXE flag is set
    	if((counter_millis-counter_ms)>timeout)return;
	}

    //send register address
    LL_I2C_TransmitData8(hi2c->Instance, *(hi2c->pTxdata++));

    counter_ms = counter_millis;
    while(!LL_I2C_IsActiveFlag_TXE(hi2c->Instance)){
		//wait until TXE flag is set
    	if((counter_millis-counter_ms)>timeout)return;
	}

    if(LL_I2C_IsActiveFlag_AF(hi2c->Instance)){
		//should not be here (ACK failure)
		CLEAR_BIT((hi2c->Instance)->SR1,I2C_SR1_AF);
		return;
	}

    counter_ms = counter_millis;
    while(!LL_I2C_IsActiveFlag_TXE(hi2c->Instance)){
		//wait until TXE flag is set
    	if((counter_millis-counter_ms)>timeout)return;
	}

	//send register data
	LL_I2C_TransmitData8(hi2c->Instance, reg_data[0]);

	counter_ms = counter_millis;
	while(!LL_I2C_IsActiveFlag_TXE(hi2c->Instance)){
		//wait until TXE flag is set
		if((counter_millis-counter_ms)>timeout)return;
	}


	if(LL_I2C_IsActiveFlag_AF(hi2c->Instance)){
		//should not be here (ACK failure)
		CLEAR_BIT((hi2c->Instance)->SR1,I2C_SR1_AF);
		return;
	}

    // Generate Stop
    SET_BIT((hi2c->Instance)->CR1, I2C_CR1_STOP);
	
}
