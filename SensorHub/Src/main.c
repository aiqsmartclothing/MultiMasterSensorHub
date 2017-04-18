/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
FMPI2C_HandleTypeDef hfmpi2c1;
DMA_HandleTypeDef hdma_fmpi2c1_rx;
DMA_HandleTypeDef hdma_fmpi2c1_tx;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SensorA 0x6A
#define SensorB 0x6B 

#define WHO_AM_I 0x0F
//Gyroscope
#define OUTX_L_G 0x22
//Accelerometer
#define OUTX_L_XL 0x28

#define FIFO_CTRL5 0x0A
#define CTRL1_XL  0x10
#define CTRL2_G   0x11
#define CTRL3_C   0x12
#define CTRL6_C   0x15
#define CTRL7_G   0x16
int Flag=0;
int timestamp=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void GetClothInfo(void);
void FirstI2C_Init(void);
void SecondI2C_Init(void);
void ThirdI2C_Init(void);
void ForthI2C_Init(void);
void Who_am_I(void);
void Read_Value(void);
void Timestamp(void);
void DataFusion(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
struct raw_data{
		char LSB;
		char MSB;
};
struct timestamp{
	unsigned char stamp1;
	unsigned char stamp2;
	unsigned char stamp3;
	unsigned char stamp4;
};
typedef struct LSM6DS33{
	struct raw_data Gyro_X;
	struct raw_data Gyro_Y;
	struct raw_data Gyro_Z;
	struct raw_data Acc_X;
	struct raw_data Acc_Y;
	struct raw_data Acc_Z;
}sensor_data;
struct LSM6DS33_packet{
		char I2C_bus_ID;
		struct timestamp data;
		char FisrtI2C_bus_ID;
	  char FisrtI2C_sensorA_type;
	sensor_data FirstI2CsensorA;
	  char FisrtI2C_sensorB_type;
	sensor_data FirstI2CsensorB;
		char SecondI2C_bus_ID;
	  char SecondI2C_sensorA_type;
	sensor_data SecondI2CsensorA;
	  char SecondI2C_sensorB_type;
	sensor_data SecondI2CsensorB;
		char ThirdI2C_bus_ID;
	  char ThirdI2C_sensorA_type;
	sensor_data ThirdI2CsensorA;
	  char ThirdI2C_sensorB_type;
	sensor_data ThirdI2CsensorB;
		char ForthI2C_bus_ID;
	  char ForthI2C_sensorA_type;
	sensor_data ForthI2CsensorA;
	  char ForthI2C_sensorB_type;
	sensor_data ForthI2CsensorB;
}Packet;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_FMPI2C1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
	{
		do{
		  CDC_Transmit_FS((unsigned char *)&"Waiting Cloth connect......\r",28);
		  HAL_Delay(2000);
		}while(Flag==0);
		
		CDC_Transmit_FS((unsigned char *)&"Cloth detect! Start to initialize!\r",35);
		HAL_Delay(100);
		
		GetClothInfo();
		Who_am_I();
		FirstI2C_Init();
		SecondI2C_Init();
		ThirdI2C_Init();
		ForthI2C_Init();

		CDC_Transmit_FS((unsigned char *)&"Initialization Complete, Start!\r",32);
		HAL_Delay(50);
		
		do{
			Read_Value();
			Timestamp();
			DataFusion();
		}while(Flag==1);

		
		do{
			CDC_Transmit_FS((unsigned char *)&"Cloth lost!\r",12);
      HAL_Delay(2000);
		}while(Flag==0);
		
		continue;
					
	}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
   
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48|RCC_PERIPHCLK_FMPI2C1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 4;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLI2SQ;
  PeriphClkInitStruct.Fmpi2c1ClockSelection = RCC_FMPI2C1CLKSOURCE_APB;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* FMPI2C1 init function */
static void MX_FMPI2C1_Init(void)
{

  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0x004019D6;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 200000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 200000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 200000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PD8   ------> USART3_TX
     PD9   ------> USART3_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : ClothDetectPin_Pin */
  GPIO_InitStruct.Pin = ClothDetectPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ClothDetectPin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Who_am_I(void)
{
	unsigned char buf[1]={0};
	HAL_I2C_Mem_Read_DMA(&hi2c1,(uint16_t)SensorA << 1, (uint16_t) WHO_AM_I, 1, (uint8_t*) buf, 1);
	HAL_Delay(2);	
	HAL_I2C_Mem_Read_DMA(&hi2c2,(uint16_t)SensorA << 1, (uint16_t) WHO_AM_I, 1, (uint8_t*) buf, 1);
	HAL_Delay(2);	
	HAL_I2C_Mem_Read_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) WHO_AM_I, 1, (uint8_t*) buf, 1);
	HAL_Delay(2);	
	HAL_FMPI2C_Mem_Read_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) WHO_AM_I, 1, (uint8_t*) buf, 1);
	HAL_Delay(2);	
}
void FirstI2C_Init(void)
{
	unsigned char nuf[1]={0};
	//----------Sensor A part----------
	nuf[0]=0x2E;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorA << 1, (uint16_t) FIFO_CTRL5, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL1_XL, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL2_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x04;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL3_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x10;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL6_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x80;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL7_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);	
	//----------Sensor B part----------
	nuf[0]=0x2E;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorB << 1, (uint16_t) FIFO_CTRL5, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL1_XL, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL2_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x04;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL3_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x10;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL6_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x80;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL7_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);	
	
}
void SecondI2C_Init(void)
{
		unsigned char nuf[1]={0};
	//----------Sensor A part----------
	nuf[0]=0x2E;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorA << 1, (uint16_t) FIFO_CTRL5, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x5c;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorA << 1, (uint16_t) CTRL1_XL, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorA << 1, (uint16_t) CTRL2_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x04;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorA << 1, (uint16_t) CTRL3_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x10;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorA << 1, (uint16_t) CTRL6_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x80;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorA << 1, (uint16_t) CTRL7_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);	
	//----------Sensor B part----------
	nuf[0]=0x2E;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorB << 1, (uint16_t) FIFO_CTRL5, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5c;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorB << 1, (uint16_t) CTRL1_XL, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorB << 1, (uint16_t) CTRL2_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x04;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorB << 1, (uint16_t) CTRL3_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x10;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorB << 1, (uint16_t) CTRL6_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x80;
	HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorB << 1, (uint16_t) CTRL7_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);	
	
}
void ThirdI2C_Init(void)
{
		unsigned char nuf[1]={0};
	//----------Sensor A part----------
	nuf[0]=0x2E;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) FIFO_CTRL5, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x5c;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) CTRL1_XL, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) CTRL2_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x04;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) CTRL3_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x10;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) CTRL6_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x80;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) CTRL7_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);	
	//----------Sensor B part----------
	nuf[0]=0x2E;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorB << 1, (uint16_t) FIFO_CTRL5, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5c;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorB << 1, (uint16_t) CTRL1_XL, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorB << 1, (uint16_t) CTRL2_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x04;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorB << 1, (uint16_t) CTRL3_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x10;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorB << 1, (uint16_t) CTRL6_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x80;
	HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorB << 1, (uint16_t) CTRL7_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);	
	
}
void ForthI2C_Init(void)
{
		unsigned char nuf[1]={0};
	//----------Sensor A part----------
	nuf[0]=0x2E;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) FIFO_CTRL5, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x5c;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL1_XL, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL2_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x04;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL3_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x10;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL6_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x80;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) CTRL7_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);	
	//----------Sensor B part----------
	nuf[0]=0x2E;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorB << 1, (uint16_t) FIFO_CTRL5, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5c;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL1_XL, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x5C;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL2_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
		
	nuf[0]=0x04;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL3_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x10;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL6_C, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);
	
	nuf[0]=0x80;
	HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorB << 1, (uint16_t) CTRL7_G, 1, (uint8_t*)nuf, 1);
	HAL_Delay(2);	
	
}
void GetClothInfo(void)
{
  Packet.I2C_bus_ID = 0xFF;
	Packet.FisrtI2C_bus_ID = 0x12;
	Packet.FisrtI2C_sensorA_type = 0x01;
	Packet.FisrtI2C_sensorB_type = 0x01;
	Packet.SecondI2C_bus_ID = 0x22;
	Packet.SecondI2C_sensorA_type = 0x01;
	Packet.SecondI2C_sensorB_type = 0x01;
	Packet.ThirdI2C_bus_ID = 0x32;
	Packet.ThirdI2C_sensorA_type = 0x01;
	Packet.ThirdI2C_sensorB_type = 0x01;
	Packet.ForthI2C_bus_ID = 0x42;
	Packet.ForthI2C_sensorA_type = 0x01;
	Packet.ForthI2C_sensorB_type = 0x01;
}	
void Read_Value(void)
{
	HAL_I2C_Mem_Read_DMA(&hi2c1,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, (uint8_t*) &Packet.FirstI2CsensorA,12);

	HAL_I2C_Mem_Read_DMA(&hi2c2,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, (uint8_t*) &Packet.SecondI2CsensorA,12);
	
	HAL_I2C_Mem_Read_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, (uint8_t*) &Packet.ThirdI2CsensorA,12);
	
	HAL_FMPI2C_Mem_Read_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, (uint8_t*) &Packet.ForthI2CsensorA,12);
	
	HAL_Delay(1);
	
	HAL_I2C_Mem_Read_DMA(&hi2c1,(uint16_t)SensorB << 1, (uint16_t) OUTX_L_G, 1, (uint8_t*) &Packet.FirstI2CsensorB,12);

	HAL_I2C_Mem_Read_DMA(&hi2c2,(uint16_t)SensorB << 1, (uint16_t) OUTX_L_G, 1, (uint8_t*) &Packet.SecondI2CsensorB,12);

  HAL_I2C_Mem_Read_DMA(&hi2c3,(uint16_t)SensorB << 1, (uint16_t) OUTX_L_G, 1, (uint8_t*) &Packet.ThirdI2CsensorB,12);
	
	HAL_FMPI2C_Mem_Read_DMA(&hfmpi2c1,(uint16_t)SensorB << 1, (uint16_t) OUTX_L_G, 1, (uint8_t*) &Packet.ForthI2CsensorB,12);

	HAL_Delay(1);
}
void Timestamp(void)
{
	Packet.data.stamp1=timestamp >> 24;
  Packet.data.stamp2=timestamp >> 16;
	Packet.data.stamp3=timestamp >> 8;
	Packet.data.stamp4=timestamp;
}
void DataFusion(void)
{
  CDC_Transmit_FS((uint8_t*)&Packet,sizeof(Packet));
}
void HAL_GPIO_EXTI_Callback_CD(uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  if(GPIO_Pin == GPIO_PIN_13 && Flag==0)
  {  	
		Flag=1;
	  GPIO_InitStruct.Pin = ClothDetectPin_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ClothDetectPin_GPIO_Port, &GPIO_InitStruct);
  }
  else
	{
		Flag=0;		
	  GPIO_InitStruct.Pin = ClothDetectPin_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ClothDetectPin_GPIO_Port, &GPIO_InitStruct);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  timestamp++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
