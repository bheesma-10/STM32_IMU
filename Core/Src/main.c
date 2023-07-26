/* USER CODE BEGIN Header */
/**
  * @file           main.c
  * @brief          Main program body
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMU_UartMsg.h"
#include "IMU_I2C.h"
#include "MPUXX50.h"
#include "CompFilter.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//PLL 
#define RCC_PLLCFGR_PLLM_ 4
#define RCC_PLLCFGR_PLLN_ 168
#define RCC_PLLCFGR_PLLQ_ 7
#define RCC_PLLCFGR_PLLR_ 2

//Timer 
#define TIM7_PSC_         8400
#define TIM7_ARR_         49

//Bus clock 
#define AHB_Prescaler_    0
#define APB1_Prescaler_   5
#define APB2_Prescaler_   4

//i2c 
#define OWN_I2CADDR       0x30
#define I2C_CLOCK_SPEED   100000
#define IMU_I2CADDR       0x68

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

UartHandle_t huart;
I2CHandle_t hi2c;
IMU_t imu;
Filter_t filter;
/* USER CODE END PV */
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint64_t counter_millis = 0;
const uint8_t uwTickFreq = 1;
uint32_t uwTickPrio   = (1UL << __NVIC_PRIO_BITS); /* Invalid PRIO */
char buffer[10] = {0x00};

uint8_t start = 0;
volatile uint8_t send_data = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/***Setup system clock***/
	SystemClockInit();

  /* USER CODE END 1 */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
	/***Initialize peripherals***/
    Gpio_Init();
    Timer_Init();
    I2C_Init();
    Usart_Init();
  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */

    /***Initialize usart & i2c instance***/
    IMU_Uart_Init(&huart);
    IMU_I2C_Init(&hi2c);

    //check if device found
    uint8_t dev_find = MPU_begin(&hi2c, &imu, IMU_I2CADDR, 1, 1);
    if(dev_find){
    	SET_BIT(GPIOB->BSRR,GPIO_BSRR_BS14);  //LD3 turned on 
    }

    //calibrate gyro
    MPU_calibrateGyro(&hi2c, &imu, 2000);
    SET_BIT(GPIOB->BSRR,GPIO_BSRR_BS7);     //LD2 turned on once completed

    //Initialize filter
    Filter_Init(&filter, 0.005, 0.98);  //deltaT - 5ms, alpha - 0.98

    start = 1;
    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(send_data){
		//send roll angle over uart
		memset(buffer,0,sizeof(buffer));
		sprintf(buffer,"%.2f\n",filter.compAngleX);
		Uart_TransmitData(&huart, (uint8_t*)buffer, strlen(buffer));
	    Uart_TransmitData(&huart, (uint8_t*)"\n", strlen("\n"));
		delay_ms(1);
		//send pitch angle over uart
		memset(buffer,0,sizeof(buffer));
		sprintf(buffer,"%.2f\n",filter.compAngleY);
		Uart_TransmitData(&huart, (uint8_t*)buffer, strlen(buffer));
	    Uart_TransmitData(&huart, (uint8_t*)"\n", strlen("\n"));
		delay_ms(1);
		send_data = 0;
	}
  }
  /* USER CODE END 3 */
}


/* USER CODE BEGIN 4 */
/**
  * @brief  System Clock Configuration and Systick Interrupt generation
  * @param  None
  * @retval None
  */
void SystemClockInit(void){
	/*****************Flash control section*****************/
	  SET_BIT(FLASH->ACR,FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_DCEN);

	  //init low level hardware
	  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
	  CLEAR_BIT(PWR->CR,PWR_CR_VOS);
	  SET_BIT(PWR->CR,PWR_CR_VOS);

	  /**********************HSE section***********************/
	  //clear HSE ready interrupt
	  SET_BIT(RCC->CIR,RCC_CIR_HSERDYC);

	  //enable HSE ready interrupt
	  SET_BIT(RCC->CIR,RCC_CIR_HSERDYIE);

	  //HSE on
	  SET_BIT(RCC->CR,RCC_CR_HSEBYP);
	  SET_BIT(RCC->CR,RCC_CR_HSEON);

	  //wait until HSE is ready

	  while((READ_BIT(RCC->CR,RCC_CR_HSERDY)==0U) | (READ_BIT(RCC->CIR,RCC_CIR_HSERDYF)==0U) ){
		  //wait until HSE ready interrupt is set
	  }

	  //clear HSE ready interrupt
	  SET_BIT(RCC->CIR,RCC_CIR_HSERDYC);

	  /**********************PLL section**********************/
	  //clear PLL ready interrupt
	  SET_BIT(RCC->CIR,RCC_CIR_PLLRDYC);

	  //enable PLL ready interrupt
	  SET_BIT(RCC->CIR,RCC_CIR_PLLRDYIE);

	  //PLL disable
	  CLEAR_BIT(RCC->CR,RCC_CR_PLLON);

	  CLEAR_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLM);
	  CLEAR_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLN);
	  CLEAR_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLQ);
	  CLEAR_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLP);

	  //PLL config
	  SET_BIT(RCC->PLLCFGR,(RCC_PLLCFGR_PLLM_<<0U) | (RCC_PLLCFGR_PLLN_<<6U) |
			  (RCC_PLLCFGR_PLLQ_<<24U) | RCC_PLLCFGR_PLLSRC_HSE);


	  //PLL enable
	  SET_BIT(RCC->CR,RCC_CR_PLLON);

	  //wait until PLL is ready
	  while((READ_BIT(RCC->CR,RCC_CR_PLLRDY)==0U) | (READ_BIT(RCC->CIR,RCC_CIR_PLLRDYF)==0U) ){
	 	  //nop
	  }

	  //clear PLL ready interrupt
	  SET_BIT(RCC->CIR,RCC_CIR_PLLRDYC);

	  /***************************Flash Latency************************/
	  if(FLASH_ACR_LATENCY_5WS>(READ_REG(FLASH->ACR) & (0xFU << FLASH_ACR_LATENCY_Pos))){
		  SET_BIT(FLASH->ACR,FLASH_ACR_LATENCY_5WS);
		  while((READ_REG(FLASH->ACR) & (0xFU << 0U))!=FLASH_ACR_LATENCY_5WS){
			  //wait until changes are reflected
		  }

	  }

	  /**********************APB1 CLK**************************/
	  CLEAR_BIT(RCC->CFGR,RCC_CFGR_PPRE1);
	  SET_BIT(RCC->CFGR,RCC_CFGR_PPRE1_DIV16);

	  /**********************APB2 CLK**************************/
	  CLEAR_BIT(RCC->CFGR,RCC_CFGR_PPRE2);
	  SET_BIT(RCC->CFGR,RCC_CFGR_PPRE2_DIV16);

	  /***********************AHB CLK**************************/
	  CLEAR_BIT(RCC->CFGR,RCC_CFGR_HPRE);

	  /*********************System Clock section***************/
	  //system clock setting
	  CLEAR_BIT(RCC->CFGR,RCC_CFGR_SW);
	  SET_BIT(RCC->CFGR,RCC_CFGR_SW_PLL);

	  while((READ_REG(RCC->CFGR) & (0x3U<<2U))!=RCC_CFGR_SWS_PLL){
		  //check if system clock is switched correctly
	  }

	  /**********************APB1 CLK**************************/
	  CLEAR_BIT(RCC->CFGR,RCC_CFGR_PPRE1);
	  SET_BIT(RCC->CFGR,(APB1_Prescaler_<<RCC_CFGR_PPRE1_Pos));

	  /**********************APB2 CLK**************************/
	  CLEAR_BIT(RCC->CFGR,RCC_CFGR_PPRE2);
	  SET_BIT(RCC->CFGR,(APB2_Prescaler_<<RCC_CFGR_PPRE2_Pos));

	  /**********************Systick config**********************/
	  //Get System & Bus clocks
	  LL_RCC_ClocksTypeDef rcc_clocks;
	  RCC_GetSysClocksFreq(&rcc_clocks);
	  //set load register
	  CLEAR_BIT(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk);
	  SET_BIT(SysTick->LOAD, (rcc_clocks.HCLK_Frequency/1000U)<<0U);
	  //clear current value
	  CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
	  //clocksource - AHB
	  SET_BIT(SysTick->CTRL,SysTick_CTRL_CLKSOURCE_Msk);
	  //tickint
	  SET_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
	  //enable counter
	  SET_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);

	  NVIC_SetPriority(SysTick_IRQn,0x03);
	  NVIC_EnableIRQ(SysTick_IRQn);





}

/**
  * @brief  Initialize GPIO for Led 
  * @param  None
  * @retval None
  */
void Gpio_Init(void){
	 /***************************GPIOB section****************************/
	  //GPIOB Reset
	  SET_BIT(RCC->AHB1RSTR,RCC_AHB1RSTR_GPIOBRST);
	  CLEAR_BIT(RCC->AHB1RSTR,RCC_AHB1RSTR_GPIOBRST);

	  //GPIOB clock enable
	  SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN);

	  //GPIOB Moder
	  SET_BIT(GPIOB->MODER,GPIO_MODER_MODE14_0);
	  SET_BIT(GPIOB->MODER,GPIO_MODER_MODE7_0);

	  //GPIOB OSPEEDR
	  SET_BIT(GPIOB->OSPEEDR,GPIO_OSPEEDR_OSPEED14_0);
	  SET_BIT(GPIOB->OSPEEDR,GPIO_OSPEEDR_OSPEED7_0);


}

/**
  * @brief  Initialize Timer for IMU data sampling
  * @param  None
  * @retval None
  */
void Timer_Init(void){
	/*************************Timer setup**********************/
	//Enable Timer
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_TIM7EN);
	NVIC_SetPriority(TIM7_IRQn,0x13);
	NVIC_EnableIRQ(TIM7_IRQn);

	// Start by making sure the timer's 'counter' is off.
	CLEAR_BIT(TIM7->CR1,TIM_CR1_CEN);

	//Timer Reset
	SET_BIT(RCC->APB1RSTR,RCC_APB1RSTR_TIM7RST);
	CLEAR_BIT(RCC->APB1RSTR,RCC_APB1RSTR_TIM7RST);

	//Set Prescaler (APB1 CLK - 84MHz)
	SET_BIT(TIM7->PSC,(TIM7_PSC_<<0U));

	//Set ARR
	CLEAR_BIT(TIM7->ARR,TIM_ARR_ARR);
	SET_BIT(TIM7->ARR,(TIM7_ARR_<<0U));

	//Auto-Reload Preload
  //  SET_BIT(TIM7->CR1,TIM_CR1_ARPE);
	CLEAR_BIT(TIM7->CR1,TIM_CR1_OPM);      //Continuous timer

	//Send an update event to reset the timer and apply settings.
	SET_BIT(TIM7->EGR,TIM_EGR_UG);

	//Enable Interrupt
	SET_BIT(TIM7->DIER,TIM_DIER_UIE);

	//Enable Counter
	SET_BIT(TIM7->CR1,TIM_CR1_CEN);
}

/**
  * @brief  Initialize I2C for communication with IMU 
  * @param  None
  * @retval None
  */
void I2C_Init(void){
	/**I2C1 GPIO Configuration
	  PB6   ------> I2C1_SCL
	  PB9   ------> I2C1_SDA
	  */

	//GPIOB MODER
    SET_BIT(GPIOB->MODER,GPIO_MODER_MODE6_1);
    SET_BIT(GPIOB->MODER,GPIO_MODER_MODE9_1);

    //GPIOB OTYPER
    SET_BIT(GPIOB->OTYPER,GPIO_OTYPER_OT6);
	SET_BIT(GPIOB->OTYPER,GPIO_OTYPER_OT9);

	//GPIOB OSPEEDR
	SET_BIT(GPIOB->OSPEEDR,GPIO_OSPEEDR_OSPEED6);
	SET_BIT(GPIOB->OSPEEDR,GPIO_OSPEEDR_OSPEED9);

	//GPIOB AFRL & AFRH
	SET_BIT(GPIOB->AFR[0],GPIO_AFRL_AFSEL6_2);
	SET_BIT(GPIOB->AFR[1],GPIO_AFRH_AFSEL9_2);

	//Reset peripheral
	SET_BIT(RCC->APB1RSTR,RCC_APB1RSTR_I2C1RST);
	CLEAR_BIT(RCC->APB1RSTR,RCC_APB1RSTR_I2C1RST);

	//Enable clock
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_I2C1EN);

	//Disable Peripheral
	CLEAR_BIT(I2C1->CR1,I2C_CR1_PE);

	//Disable OwnAddress2
	CLEAR_BIT(I2C1->OAR2, I2C_OAR2_ENDUAL);

	//Enable OwnAddress1
	CLEAR_BIT(I2C1->OAR1, I2C_OAR1_ADDMODE);
	SET_BIT(I2C1->OAR1, (OWN_I2CADDR<<1U) | (1<<14U));

	//Get System & Bus clocks
	LL_RCC_ClocksTypeDef rcc_clocks;
	RCC_GetSysClocksFreq(&rcc_clocks);

	/*----------------- I2C SCL Clock Speed Configuration ------------
     * Configure the SCL speed :
     * - ClockSpeed: I2C_CR2_FREQ[5:0], I2C_TRISE_TRISE[5:0], I2C_CCR_FS,
     *           and I2C_CCR_CCR[11:0] bits
     * - DutyCycle: I2C_CCR_DUTY[7:0] bits
    */

	//compute frequency range
    uint32_t freqrange = rcc_clocks.PCLK1_Frequency/1000000;
    uint32_t ClockSpeed = I2C_CLOCK_SPEED;
    uint32_t clockconfig = 0x0U;
    uint32_t DutyCycle = 0x0U;
    CLEAR_BIT(I2C1->CR2, I2C_CR2_FREQ);
    SET_BIT(I2C1->CR2, (freqrange<<0U));

    //configure rise time
    CLEAR_BIT(I2C1->TRISE, I2C_TRISE_TRISE);
    SET_BIT(I2C1->TRISE, __LL_I2C_RISE_TIME(freqrange, ClockSpeed));

    /* Configure Speed mode, Duty Cycle and Clock control register value */
    if (ClockSpeed > LL_I2C_MAX_SPEED_STANDARD)
    {
 	  /* Set Speed mode at fast and duty cycle for Clock Speed request in fast clock range */
	  clockconfig = LL_I2C_CLOCK_SPEED_FAST_MODE                                          | \
				  __LL_I2C_SPEED_FAST_TO_CCR(rcc_clocks.PCLK1_Frequency, ClockSpeed, DutyCycle)        | \
				  DutyCycle;
    }
    else
    {
	  /* Set Speed mode at standard for Clock Speed request in standard clock range */
	  clockconfig = __LL_I2C_SPEED_STANDARD_TO_CCR(rcc_clocks.PCLK1_Frequency, ClockSpeed);
    }

    //configure clock control
    CLEAR_BIT(I2C1->CCR, I2C_CCR_FS);
	CLEAR_BIT(I2C1->CCR, I2C_CCR_DUTY);
	SET_BIT(I2C1->CCR, (clockconfig<<0U));

	//setup I2C mode
	CLEAR_BIT(I2C1->CR1, I2C_CR1_SMBUS);

	//Enable Peripheral
	SET_BIT(I2C1->CR1, I2C_CR1_PE);

}

/**
  * @brief  Initialize Uart for logging messages
  * @param  None
  * @retval None
  */
void Usart_Init(void){
	/**USART3 GPIO Configuration
	  PD8   ------> USART3_TX
	  PD9   ------> USART3_RX
	  */
	//GPIOD Reset
    SET_BIT(RCC->AHB1RSTR,RCC_AHB1RSTR_GPIODRST);
    CLEAR_BIT(RCC->AHB1RSTR,RCC_AHB1RSTR_GPIODRST);

    //GPIOD clock enable
    SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);

	//GPIOD Moder
	SET_BIT(GPIOD->MODER,GPIO_MODER_MODE8_1);
	SET_BIT(GPIOD->MODER,GPIO_MODER_MODE9_1);

	//GPIOD OSPEEDR
	SET_BIT(GPIOD->OSPEEDR,GPIO_OSPEEDR_OSPEED8);
	SET_BIT(GPIOD->OSPEEDR,GPIO_OSPEEDR_OSPEED9);

	//GPIOD AFRH
	SET_BIT(GPIOD->AFR[1],GPIO_AFRH_AFSEL8_0 | GPIO_AFRH_AFSEL8_1 | GPIO_AFRH_AFSEL8_2);
	SET_BIT(GPIOD->AFR[1],GPIO_AFRH_AFSEL9_0 | GPIO_AFRH_AFSEL9_1 | GPIO_AFRH_AFSEL9_2);

	//Reset peripheral
	SET_BIT(RCC->APB1RSTR,RCC_APB1RSTR_USART3RST);
	CLEAR_BIT(RCC->APB1RSTR,RCC_APB1RSTR_USART3RST);

	//Enable clock
	SET_BIT(RCC->APB1ENR,RCC_APB1ENR_USART3EN);

	//Disable Peripheral
	CLEAR_BIT(USART3->CR1, USART_CR1_UE);

	//configure parity,oversampling, data direction and word length
    CLEAR_BIT(USART3->CR1, (USART_CR1_M | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8));
    SET_BIT(USART3->CR1, USART_CR1_TE | USART_CR1_RE);

    //configure stop bits
    CLEAR_BIT(USART3->CR2, USART_CR2_STOP);

    //configure hardware flow
    CLEAR_BIT(USART3->CR3, USART_CR3_CTSE | USART_CR3_RTSE);

    //Get System & Bus clocks
	LL_RCC_ClocksTypeDef rcc_clocks;
	RCC_GetSysClocksFreq(&rcc_clocks);

	uint32_t periphclock = rcc_clocks.PCLK1_Frequency;

	//configure baud rate
	uint32_t baudrate = 115200;
	if(READ_BIT(USART3->CR2,USART_CR1_OVER8)){
		SET_BIT(USART3->BRR, (uint16_t)__LL_USART_DIV_SAMPLING8(periphclock, baudrate));
	}else{
		SET_BIT(USART3->BRR, (uint16_t)__LL_USART_DIV_SAMPLING16(periphclock, baudrate));
	}

	 /* In Asynchronous mode, following bits must be kept cleared:
	  - LINEN, CLKEN bits in the USART_CR2 register,
	  - SCEN, IREN and HDSEL bits in the USART_CR3 register.*/
	 CLEAR_BIT(USART3->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
	 CLEAR_BIT(USART3->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));

	 //Enable Peripheral
	 SET_BIT(USART3->CR1, USART_CR1_UE);
}

/**
  * @brief  Return the frequencies of different on chip clocks;  System, AHB, APB1 and APB2 buses clocks
  * @note   Each time SYSCLK, HCLK, PCLK1 and/or PCLK2 clock changes, this function
  *         must be called to update structure fields. Otherwise, any
  *         configuration based on this function will be incorrect.
  * @param  RCC_Clocks pointer to a @ref LL_RCC_ClocksTypeDef structure which will hold the clocks frequencies
  * @retval None
  */
void RCC_GetSysClocksFreq(LL_RCC_ClocksTypeDef *RCC_Clocks)
{
  /* Get SYSCLK frequency */
  RCC_Clocks->SYSCLK_Frequency = RCC_GetSysClockFreq();

  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency   = __LL_RCC_CALC_HCLK_FREQ(RCC_Clocks->SYSCLK_Frequency, LL_RCC_GetAHBPrescaler());

  /* PCLK1 clock frequency */
  RCC_Clocks->PCLK1_Frequency  = __LL_RCC_CALC_PCLK1_FREQ(RCC_Clocks->HCLK_Frequency, LL_RCC_GetAPB1Prescaler());

  /* PCLK2 clock frequency */
  RCC_Clocks->PCLK2_Frequency  = __LL_RCC_CALC_PCLK2_FREQ(RCC_Clocks->HCLK_Frequency , LL_RCC_GetAPB2Prescaler());
}

/**
 * @brief  Function to provide delay in milliseconds specified by user
 * @param  ms milliseconds to wait
 * @retval None
 */
void delay_ms(uint32_t ms){
	uint64_t counter_now = counter_millis;
	while((counter_millis-counter_now)<ms){
		//wait for it
	}


}

/**
 * @brief  ISR routine for TIM7
 * @param  None
 * @retval None
 * @note   called at every 5ms. data is read from the IMU and filtered later
 */
void TIM7_IRQHandler(void)
{
	if(READ_BIT(TIM7->SR,TIM_SR_UIF)){
		CLEAR_BIT(TIM7->SR,TIM_SR_UIF); //clear the interrupt flag
		if(start){
			//read scaled data, then get filter data
			MPU_readscaledData(&hi2c, &imu);
			Filter_calcAttitude(&filter, &imu);
			send_data = 1;
		}
	}
}

/**
  * @brief  ISR routine for Systick timer.
  * @param  None 
  * @retval None
  * @note   called every 1ms. check for the interrupt flag and increase counter.
  */
void SysTick_Handler(void)
{
	counter_millis++;
}

/**
 * @brief  Function to check for the current system core clock
 * @param  None
 * @retval System clock frequency
 * @note   Gets system core frequency in MHz based on the clock source set
 */
uint32_t RCC_GetSysClockFreq(void)
{
  uint32_t pllm = 0U;
  uint32_t pllvco = 0U;
  uint32_t pllp = 0U;
  uint32_t pllr = 0U;
  uint32_t sysclockfreq = 0U;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (RCC->CFGR & RCC_CFGR_SWS)
  {
    case RCC_CFGR_SWS_HSI:  /* HSI used as system clock source */
    {
      sysclockfreq = HSI_VALUE;
       break;
    }
    case RCC_CFGR_SWS_HSE:  /* HSE used as system clock  source */
    {
      sysclockfreq = HSE_VALUE;
      break;
    }
    case RCC_CFGR_SWS_PLL:  /* PLL/PLLP used as system clock  source */
    {
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
      SYSCLK = PLL_VCO / PLLP */
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) != RCC_PLLCFGR_PLLSRC_HSI)
      {
        /* HSE used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) HSE_VALUE * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) HSI_VALUE * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
      }
      pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) *2U);

      sysclockfreq = pllvco/pllp;
      break;
    }
    case RCC_CFGR_SWS_PLLR:  /* PLL/PLLR used as system clock  source */
    {
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
      SYSCLK = PLL_VCO / PLLR */
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) != RCC_PLLCFGR_PLLSRC_HSI)
      {
        /* HSE used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) HSE_VALUE * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) HSI_VALUE * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
      }
      pllr = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos);

      sysclockfreq = pllvco/pllr;
      break;
    }
    default:
    {
      sysclockfreq = HSI_VALUE;
      break;
    }
  }
  return sysclockfreq;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
