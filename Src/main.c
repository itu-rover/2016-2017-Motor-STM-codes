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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int i=0;
char buffer[100];
int len;
char Rx_indx;
unsigned char Rx_data[2], Rx_Buffer[100], Transfer_cplt, flag[6];

int count=0,axis_index;
unsigned char axis_enable[6]={0,0,0,0,0,0};
unsigned char axis_on[6]={0,0,0,0,0,0};
unsigned char axis_stop[6]={0,0,0,0,0,0};
unsigned char axis_direction[6]={0,0,0,0,0,0};
unsigned char axis_velocity[6]={0,0,0,0,0,0};
uint32_t axis_destination[6]={0,0,0,0,0,0};
uint32_t axis_location[6]={0,0,0,0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t i;
	if (1)	//uart kesmesi kime ait
		{
		if (Rx_indx==0) {for (i=0;i<100;i++) Rx_Buffer[i]=0;}	//Rx_Buffer temizle	
		if(Rx_data[0]=='B')Rx_indx=0;
		if (Rx_data[0]!='E')	//if received data different from ascii 13 (enter)
			{
			Rx_Buffer[Rx_indx++]=Rx_data[0];	//add data to Rx_Buffer
			}
		else			//end bit arrived
			{
			Rx_indx=0;
			Transfer_cplt=1;//flag enable
				// x dir x val y dir y val
			for(int a=0;a<3;a++){				
				axis_index=a; //Rx_Buffer[0]-'0';
				axis_enable[axis_index]=1;	// axis enabled
				axis_direction[axis_index]=(Rx_Buffer[1]-'0'); //axis direction is set
				axis_velocity[axis_index]=((Rx_Buffer[2]-'0')*5); //axis velocity is set
				if(Rx_Buffer[2]<Rx_Buffer[4]){
				if(Rx_Buffer[3]-'0'){axis_velocity[axis_index]=(Rx_Buffer[4]-'0')*5;axis_direction[axis_index]=0;}
				else {axis_velocity[axis_index]=(Rx_Buffer[4]-'0')*5;axis_direction[axis_index]=1;};
				}
					axis_destination[axis_index]= 1; //(Rx_Buffer[3]-'0')*10000; // axis destination is set
				axis_destination[axis_index]+=1; //(Rx_Buffer[4]-'0')*1000; // axis destination is set
			}
			for(int a=3;a<6;a++){				
				axis_index=a; //Rx_Buffer[0]-'0';
				axis_enable[axis_index]=1;	// axis enabled
				axis_direction[axis_index]=(Rx_Buffer[1]-'0'); //axis direction is set
				axis_velocity[axis_index]=((Rx_Buffer[2]-'0')*5); //axis velocity is set
				if(Rx_Buffer[2]<Rx_Buffer[4]){
				if(Rx_Buffer[3]-'0'){axis_velocity[axis_index]=(Rx_Buffer[4]-'0')*5;axis_direction[axis_index]=1;}
				else{axis_velocity[axis_index]=(Rx_Buffer[4]-'0')*5;axis_direction[axis_index]=0;}
				}
				axis_destination[axis_index]= 1; //(Rx_Buffer[3]-'0')*10000; // axis destination is set
				axis_destination[axis_index]+=1; //(Rx_Buffer[4]-'0')*1000; // axis destination is set
			}
					for(i=0;i<6;i++){
					if(axis_velocity[i]>44)axis_velocity[i]=48;
					}	
				}
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		
		HAL_UART_Receive_IT(&huart1, Rx_data, 1);	//uart tekrar aktif
		}

}

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1)
    {	
        if (TIM_GET_ITSTATUS(htim1, TIM_IT_UPDATE) != RESET) 
        {
             
             }
    }	
		
//void EXTI15_10_IRQHandler(void)
//{//motor number0
//		//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
//	
//  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET) 
//  { 
//    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//		axis_location[0]++;				
//	}
//}
//void EXTI0_IRQHandler(void)
//	{
//  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) 
//  { 
//    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//		axis_location[2]++;				
//	}
//}
//void EXTI4_IRQHandler(void)
//	{
//  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) 
//  { 
//    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//		axis_location[1]++;				
//	}
//}
		
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
 	HAL_TIM_Base_Start_IT(&htim1);	//timer1 aktif
	HAL_UART_Receive_IT(&huart1, Rx_data, 1);	//uart1 aktif
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
  /* USER CODE END WHILE */

//  /* USER CODE BEGIN 3 */
//		if(Transfer_cplt){
//		sprintf(buffer,"%s\r\n",Rx_Buffer);
//		int len=strlen(buffer);
//		HAL_UART_Transmit(&huart1,(uint8_t *)buffer, len, 1000);
//			HAL_UART_Transmit(&huart1,"\r\n sol yon", 10, 1000);
//			char enco= axis_direction[0] + '0';
//		HAL_UART_Transmit(&huart1,(uint8_t *)&enco, 1, 1000);
//		HAL_UART_Transmit(&huart1,"\r\n sol hiz", 10, 1000);		
//			char hedef= axis_velocity[0]/5 + '0';			
//		HAL_UART_Transmit(&huart1,(uint8_t *)&hedef, 1, 1000);
//			HAL_UART_Transmit(&huart1,"\r\n sag yon", 10, 1000);
//			char enco1= axis_direction[3] + '0';
//		HAL_UART_Transmit(&huart1,(uint8_t *)&enco1, 1, 1000);
//		HAL_UART_Transmit(&huart1,"\r\n sag hiz", 10, 1000);		
//			char hedef1= axis_velocity[3]/5 + '0';			
//		HAL_UART_Transmit(&huart1,(uint8_t *)&hedef1, 1, 1000);
//				//axis_location[axis_index]=0;
//				
//			Transfer_cplt=0;
//	}
					if(axis_enable[0]){
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
//			HAL_UART_Transmit(&huart1,"\r\nA1-on", 7, 1000);
						axis_enable[0]=0;
						axis_on[0]=1;
					}
					if(axis_stop[0]){						
//						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
						//HAL_Delay(100);			// delay for brake
//			HAL_UART_Transmit(&huart1,"\r\nA1-off", 8, 1000);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
						axis_stop[0]=0;
						axis_on[0]=0;
					}
					///set a11 -a12 a8
					if(axis_enable[1]){
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
//			HAL_UART_Transmit(&huart1,"\r\nA2-on", 7, 1000);
					axis_enable[1]=0;
					axis_on[1]=1;
					}
					if(axis_stop[1]){						
//						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
						//HAL_Delay(100);			// delay for brake
//			HAL_UART_Transmit(&huart1,"\r\nA2-off", 8, 1000);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
						axis_stop[1]=0;
						axis_on[1]=0;
					}
					
					if(axis_enable[2]){
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
//			HAL_UART_Transmit(&huart1,"\r\nA3-on", 7, 1000);
					axis_enable[2]=0;
						axis_on[2]=1;
					}
					if(axis_stop[2]){						
//						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
						//HAL_Delay(100);			// delay for brake
			HAL_UART_Transmit(&huart1,"\r\nA3-off", 8, 1000);
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
						axis_stop[2]=0;
						axis_on[2]=0;
					}
					if(axis_enable[3]){
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
//			HAL_UART_Transmit(&huart1,"\r\nA4-on", 7, 1000);
					axis_enable[3]=0;
						axis_on[3]=1;
					}
					if(axis_stop[3]){						
//						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
						//HAL_Delay(100);			// delay for brake
//			HAL_UART_Transmit(&huart1,"\r\nA4-off", 8, 1000);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
						axis_stop[3]=0;
						axis_on[3]=0;
					}
					if(axis_enable[4]){
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
//			HAL_UART_Transmit(&huart1,"\r\nA5-on", 7, 1000);
					axis_enable[4]=0;
						axis_on[4]=1;
					}
					if(axis_stop[4]){						
//						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
						//HAL_Delay(100);			// delay for brake
//			HAL_UART_Transmit(&huart1,"\r\nA5-off", 8, 1000);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
						axis_stop[4]=0;
						axis_on[4]=0;
					}
					if(axis_enable[5]){
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
//			HAL_UART_Transmit(&huart1,"\r\nA6-on", 7, 1000);
					axis_enable[5]=0;
						axis_on[5]=1;
					}
					if(axis_stop[5]){						
//						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
//						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
						//HAL_Delay(100);			// delay for brake
//			HAL_UART_Transmit(&huart1,"\r\nA6-off", 8, 1000);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
						axis_stop[5]=0;
						axis_on[5]=0;
					}
	//3-c13 b8  b7 b9
	//- a1  c15 a0 c14
	//5 a7  a5  a6
	//2-b11 b12 b10
	//4-a8  a12 a11
	//1-b12 b14 b13
			count=__HAL_TIM_GetCounter(&htim1);
if(axis_on[0])
	{
	if(axis_direction[0])
		{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		if(count<axis_velocity[0])
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
		}
	else
		{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
		if(count<axis_velocity[0])
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		}
	}
if(axis_on[1])
	{
	if(axis_direction[1])
		{					
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		if(count<axis_velocity[1])
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
		}
	else 
		{					
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
		if(count<axis_velocity[1])
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
		else
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		}
	}
if(axis_on[2])
	{
	if(axis_direction[2])
		{					
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		if(count<axis_velocity[2])
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
		}
	else 
		{					
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
		if(count<axis_velocity[2])
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
		else
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		}
	}
if(axis_on[3])
	{
	if(axis_direction[3])
		{					
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		if(count<axis_velocity[3])
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
		}
	else 
		{					
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
		if(count<axis_velocity[3])
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
		else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		}
	}
if(axis_on[4])
	{
	if(axis_direction[4])
		{					
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
		if(count<axis_velocity[4])
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
		}
	else 
		{					
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
		if(count<axis_velocity[4])
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
		else
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
		}
	}
if(axis_on[5])
	{
	if(axis_direction[5])
		{					
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		if(count<axis_velocity[5])
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		}
	else 
		{					
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		if(count<axis_velocity[5])
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		}
	}
		for(i=0;i<6;i++) 
	{
		if((axis_destination[i]<axis_location[i] | axis_velocity[i]==0) & axis_on[i]){axis_enable[i]=0;axis_location[i]=0;axis_stop[i]=1;}					
	}
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 255;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN_S_3_Pin|IN_S1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_S_Pin|IN_S_Pin|IN_S1_5_Pin|SD_S_5_Pin 
                          |IN_S_5_Pin|IN_S_4_Pin|SD_S_4_Pin|IN_S1_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN_S1_2_Pin|SD_S_2_Pin|IN_S_2_Pin|IN_S_1_Pin 
                          |SD_S_1_Pin|IN_S1_1_Pin|SD_S_3_Pin|IN_S1_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN_S_3_Pin IN_S1_Pin */
  GPIO_InitStruct.Pin = IN_S_3_Pin|IN_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : C_SENSE_Pin */
  GPIO_InitStruct.Pin = C_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(C_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_S_Pin IN_S_Pin IN_S1_5_Pin SD_S_5_Pin 
                           IN_S_5_Pin IN_S_4_Pin SD_S_4_Pin IN_S1_4_Pin */
  GPIO_InitStruct.Pin = SD_S_Pin|IN_S_Pin|IN_S1_5_Pin|SD_S_5_Pin 
                          |IN_S_5_Pin|IN_S_4_Pin|SD_S_4_Pin|IN_S1_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_S1_2_Pin SD_S_2_Pin IN_S_2_Pin IN_S_1_Pin 
                           SD_S_1_Pin IN_S1_1_Pin SD_S_3_Pin IN_S1_3_Pin */
  GPIO_InitStruct.Pin = IN_S1_2_Pin|SD_S_2_Pin|IN_S_2_Pin|IN_S_1_Pin 
                          |SD_S_1_Pin|IN_S1_1_Pin|SD_S_3_Pin|IN_S1_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 C_SENSE_3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|C_SENSE_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
