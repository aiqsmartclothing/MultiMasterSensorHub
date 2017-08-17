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

int Flag=1;
int timestamp=0;
int Num,Q;
int a=0,i=0;
uint16_t Register[20];
uint8_t Value[20];
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
void DefaultSetting(void);
void FirstI2C_Init(void);
void SecondI2C_Init(void);
void ThirdI2C_Init(void);
void ForthI2C_Init(void);
void Read_Value(void);
void DataConvertA(void);
void DataConvertB(void);
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
	struct raw_data Acc_X;
	struct raw_data Acc_Y;
	struct raw_data Acc_Z;
	struct raw_data Gyro_X;
	struct raw_data Gyro_Y;
	struct raw_data Gyro_Z;
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
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		DefaultSetting();
		GetClothInfo();
		FirstI2C_Init();
		SecondI2C_Init();
		ThirdI2C_Init();
		ForthI2C_Init();
		
		do{
			Read_Value();
			DataFusion();
			DataFusion();
		}while(Flag==1);
  }
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
  hfmpi2c1.Init.Timing = 0x20100D5D;
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
  hi2c1.Init.ClockSpeed = 150000;
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
  hi2c2.Init.ClockSpeed = 150000;
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
  hi2c3.Init.ClockSpeed = 150000;
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

  /*Configure GPIO pin : WakeUp_Pin */
  GPIO_InitStruct.Pin = WakeUp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WakeUp_GPIO_Port, &GPIO_InitStruct);

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
void FirstI2C_Init(void)
{
	for(Num=1;Num<=Q;++Num)
	{
		//----------Sensor A part----------
		HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorA << 1, Register[Num], 1, &Value[Num], 1);
	  HAL_Delay(2);
		//----------Sensor B part----------
		HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)SensorB << 1, Register[Num], 1, &Value[Num], 1);
	  HAL_Delay(2);
	}
}
void SecondI2C_Init(void)
{
	for(Num=1;Num<=Q;++Num)
	{
		//----------Sensor A part----------
		HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorA << 1, Register[Num], 1, &Value[Num], 1);
	  HAL_Delay(2);
		//----------Sensor B part----------
		HAL_I2C_Mem_Write_DMA(&hi2c2,(uint16_t)SensorB << 1, Register[Num], 1, &Value[Num], 1);
	  HAL_Delay(2);
	}
}
void ThirdI2C_Init(void)
{
	for(Num=1;Num<=Q;++Num)
	{
		//----------Sensor A part----------
		HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorA << 1, Register[Num], 1, &Value[Num], 1);
	  HAL_Delay(2);
		//----------Sensor B part----------
		HAL_I2C_Mem_Write_DMA(&hi2c3,(uint16_t)SensorB << 1, Register[Num], 1, &Value[Num], 1);
	  HAL_Delay(2);
	}
}
void ForthI2C_Init(void)
{
	for(Num=1;Num<=Q;++Num)
	{
		//----------Sensor A part----------
		HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, Register[Num], 1, &Value[Num], 1);
	  HAL_Delay(2);
		//----------Sensor B part----------
		HAL_FMPI2C_Mem_Write_DMA(&hfmpi2c1,(uint16_t)SensorB << 1, Register[Num], 1, &Value[Num], 1);
	  HAL_Delay(2);
	}
}
void DefaultSetting(void)
{
		/*Register name*/
		Register[1]=0x0A;
		Register[2]=0x10;
		Register[3]=0x11;
	  Register[4]=0x12;
	  Register[5]=0x13;
	  Register[6]=0x14;
		Register[7]=0x15;
		Register[8]=0x16;
	  Register[9]=0x17;
	  Register[10]=0x18;
	  Register[11]=0x19;
	  Register[12]=0x0B;
		/*Feed I2C parameters*/ 
		Value[1]=0x2e;
	  Value[2]=0x5c;
	  Value[3]=0x5c;
	  Value[4]=0x44;
	  Value[5]=0x00;
	  Value[6]=0x00;
	  Value[7]=0x00;
		Value[8]=0x80;
		Value[9]=0x00;
		Value[10]=0x38;
		Value[11]=0x38;
		Value[12]=0x00;
	  Q=12;
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
uint8_t buf1[12]={0},buf2[12]={0},buf3[12]={0},buf4[12]={0},buf5[12]={0},buf6[12]={0},buf7[12]={0},buf8[12]={0};
void Read_Value(void)
{
	HAL_I2C_Mem_Read_DMA(&hi2c1,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, buf1,12);

	HAL_I2C_Mem_Read_DMA(&hi2c2,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, buf2,12);
	
	HAL_I2C_Mem_Read_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, buf3,12);
	
	HAL_FMPI2C_Mem_Read_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, buf4,12);
	
	DataConvertA();
	
	HAL_Delay(1);
	
	HAL_I2C_Mem_Read_DMA(&hi2c1,(uint16_t)SensorB << 1, (uint16_t) OUTX_L_G, 1, buf5,12);

	HAL_I2C_Mem_Read_DMA(&hi2c2,(uint16_t)SensorB << 1, (uint16_t) OUTX_L_G, 1, buf6,12);

	HAL_I2C_Mem_Read_DMA(&hi2c3,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, buf7,12);
	
	HAL_FMPI2C_Mem_Read_DMA(&hfmpi2c1,(uint16_t)SensorA << 1, (uint16_t) OUTX_L_G, 1, buf8,12);
	
	DataConvertB();
	
	HAL_Delay(1);
}
void DataConvertA(void)
{
	Packet.FirstI2CsensorA.Acc_X.LSB=buf1[6];
	Packet.FirstI2CsensorA.Acc_X.MSB=buf1[7];
	Packet.FirstI2CsensorA.Acc_Y.LSB=buf1[8];
	Packet.FirstI2CsensorA.Acc_Y.MSB=buf1[9];
	Packet.FirstI2CsensorA.Acc_Z.LSB=buf1[10];
	Packet.FirstI2CsensorA.Acc_Z.MSB=buf1[11];
	
	Packet.FirstI2CsensorA.Gyro_X.LSB=buf1[0];
  Packet.FirstI2CsensorA.Gyro_X.MSB=buf1[1];
	Packet.FirstI2CsensorA.Gyro_Y.LSB=buf1[2];
	Packet.FirstI2CsensorA.Gyro_Y.MSB=buf1[3];
	Packet.FirstI2CsensorA.Gyro_Z.LSB=buf1[4];
	Packet.FirstI2CsensorA.Gyro_Z.MSB=buf1[5];
	
	Packet.SecondI2CsensorA.Acc_X.LSB=buf2[6];
	Packet.SecondI2CsensorA.Acc_X.MSB=buf2[7];
	Packet.SecondI2CsensorA.Acc_Y.LSB=buf2[8];
	Packet.SecondI2CsensorA.Acc_Y.MSB=buf2[9]; 
	Packet.SecondI2CsensorA.Acc_Z.LSB=buf2[10];
	Packet.SecondI2CsensorA.Acc_Z.MSB=buf2[11];
	
	Packet.SecondI2CsensorA.Gyro_X.LSB=buf2[0];
  Packet.SecondI2CsensorA.Gyro_X.MSB=buf2[1];
	Packet.SecondI2CsensorA.Gyro_Y.LSB=buf2[2];
	Packet.SecondI2CsensorA.Gyro_Y.MSB=buf2[3];
	Packet.SecondI2CsensorA.Gyro_Z.LSB=buf2[4];
	Packet.SecondI2CsensorA.Gyro_Z.MSB=buf2[5];
	
	Packet.ThirdI2CsensorA.Acc_X.LSB=buf3[6];
	Packet.ThirdI2CsensorA.Acc_X.MSB=buf3[7];
	Packet.ThirdI2CsensorA.Acc_Y.LSB=buf3[8];
	Packet.ThirdI2CsensorA.Acc_Y.MSB=buf3[9];
	Packet.ThirdI2CsensorA.Acc_Z.LSB=buf3[10];
	Packet.ThirdI2CsensorA.Acc_Z.MSB=buf3[11];
	
	Packet.ThirdI2CsensorA.Gyro_X.LSB=buf3[0];
  Packet.ThirdI2CsensorA.Gyro_X.MSB=buf3[1];
	Packet.ThirdI2CsensorA.Gyro_Y.LSB=buf3[2];
	Packet.ThirdI2CsensorA.Gyro_Y.MSB=buf3[3];
	Packet.ThirdI2CsensorA.Gyro_Z.LSB=buf3[4];
	Packet.ThirdI2CsensorA.Gyro_Z.MSB=buf3[5];
	
	Packet.ForthI2CsensorA.Acc_X.LSB=buf4[6];
	Packet.ForthI2CsensorA.Acc_X.MSB=buf4[7];
	Packet.ForthI2CsensorA.Acc_Y.LSB=buf4[8];
	Packet.ForthI2CsensorA.Acc_Y.MSB=buf4[9]; 
	Packet.ForthI2CsensorA.Acc_Z.LSB=buf4[10];
	Packet.ForthI2CsensorA.Acc_Z.MSB=buf4[11];
	
	Packet.ForthI2CsensorA.Gyro_X.LSB=buf4[0];
  Packet.ForthI2CsensorA.Gyro_X.MSB=buf4[1];
	Packet.ForthI2CsensorA.Gyro_Y.LSB=buf4[2];
	Packet.ForthI2CsensorA.Gyro_Y.MSB=buf4[3];
	Packet.ForthI2CsensorA.Gyro_Z.LSB=buf4[4];
	Packet.ForthI2CsensorA.Gyro_Z.MSB=buf4[5];
}
void DataConvertB(void)
{
	Packet.FirstI2CsensorB.Acc_X.LSB=buf5[6];
	Packet.FirstI2CsensorB.Acc_X.MSB=buf5[7];
	Packet.FirstI2CsensorB.Acc_Y.LSB=buf5[8];
	Packet.FirstI2CsensorB.Acc_Y.MSB=buf5[9];
	Packet.FirstI2CsensorB.Acc_Z.LSB=buf5[10];
	Packet.FirstI2CsensorB.Acc_Z.MSB=buf5[11];
	
	Packet.FirstI2CsensorB.Gyro_X.LSB=buf5[0];
  Packet.FirstI2CsensorB.Gyro_X.MSB=buf5[1];
	Packet.FirstI2CsensorB.Gyro_Y.LSB=buf5[2];
	Packet.FirstI2CsensorB.Gyro_Y.MSB=buf5[3];
	Packet.FirstI2CsensorB.Gyro_Z.LSB=buf5[4];
	Packet.FirstI2CsensorB.Gyro_Z.MSB=buf5[5];
	
	Packet.SecondI2CsensorB.Acc_X.LSB=buf6[6];
	Packet.SecondI2CsensorB.Acc_X.MSB=buf6[7];
	Packet.SecondI2CsensorB.Acc_Y.LSB=buf6[8];
	Packet.SecondI2CsensorB.Acc_Y.MSB=buf6[9]; 
	Packet.SecondI2CsensorB.Acc_Z.LSB=buf6[10];
	Packet.SecondI2CsensorB.Acc_Z.MSB=buf6[11];
	
	Packet.SecondI2CsensorB.Gyro_X.LSB=buf6[0];
  Packet.SecondI2CsensorB.Gyro_X.MSB=buf6[1];
	Packet.SecondI2CsensorB.Gyro_Y.LSB=buf6[2];
	Packet.SecondI2CsensorB.Gyro_Y.MSB=buf6[3];
	Packet.SecondI2CsensorB.Gyro_Z.LSB=buf6[4];
	Packet.SecondI2CsensorB.Gyro_Z.MSB=buf6[5];
	
	Packet.ThirdI2CsensorB.Acc_X.LSB=buf7[6];
	Packet.ThirdI2CsensorB.Acc_X.MSB=buf7[7];
	Packet.ThirdI2CsensorB.Acc_Y.LSB=buf7[8];
	Packet.ThirdI2CsensorB.Acc_Y.MSB=buf7[9];
	Packet.ThirdI2CsensorB.Acc_Z.LSB=buf7[10];
	Packet.ThirdI2CsensorB.Acc_Z.MSB=buf7[11];
	
	Packet.ThirdI2CsensorB.Gyro_X.LSB=buf7[0];
  Packet.ThirdI2CsensorB.Gyro_X.MSB=buf7[1];
	Packet.ThirdI2CsensorB.Gyro_Y.LSB=buf7[2];
	Packet.ThirdI2CsensorB.Gyro_Y.MSB=buf7[3];
	Packet.ThirdI2CsensorB.Gyro_Z.LSB=buf7[4];
	Packet.ThirdI2CsensorB.Gyro_Z.MSB=buf7[5];
	
	Packet.ForthI2CsensorB.Acc_X.LSB=buf8[6];
	Packet.ForthI2CsensorB.Acc_X.MSB=buf8[7];
	Packet.ForthI2CsensorB.Acc_Y.LSB=buf8[8];
	Packet.ForthI2CsensorB.Acc_Y.MSB=buf8[9]; 
	Packet.ForthI2CsensorB.Acc_Z.LSB=buf8[10];
	Packet.ForthI2CsensorB.Acc_Z.MSB=buf8[11];
	
	Packet.ForthI2CsensorB.Gyro_X.LSB=buf8[0];
  Packet.ForthI2CsensorB.Gyro_X.MSB=buf8[1];
	Packet.ForthI2CsensorB.Gyro_Y.LSB=buf8[2];
	Packet.ForthI2CsensorB.Gyro_Y.MSB=buf8[3];
	Packet.ForthI2CsensorB.Gyro_Z.LSB=buf8[4];
	Packet.ForthI2CsensorB.Gyro_Z.MSB=buf8[5];
	
}
void Timestamp(void)
{
	Packet.data.stamp1=timestamp ;
  Packet.data.stamp2=timestamp >> 8;
	Packet.data.stamp3=timestamp >> 16;
	Packet.data.stamp4=timestamp >> 24;
}
void DataFusion(void)
{
  CDC_Transmit_FS((uint8_t*)&Packet,sizeof(Packet));
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
