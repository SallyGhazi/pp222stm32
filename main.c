/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "stdio.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
int sensor[5]={};
int error;
double lastError=0;
int P , I , D;
float Kp = 0.001;
float ki = 0;
float Kd = 0.1;
const uint8_t maxspeedr = 100;
const uint8_t maxspeedl = 100;
const uint8_t basespeedr = 75;
const uint8_t basespeedl = 75;
char path[100];
int pathLength=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Forward(double PWM_L,double PWM_R);
void Backward(double PWM_L,double PWM_R);
void stop();
void Turn_Left(double PWM_L,double PWM_R);
void Turn_Right(double PWM_L,double PWM_R);
int read_sensor();
void PID_control();
void save_path(char dir);
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Forward(75,75);
	  HAL_Delay(1000);

	  stop();
	  HAL_Delay(1000);

	  Forward(75.11,74.89);
	  HAL_Delay(1000);

	  stop();
	  HAL_Delay(1000);

	  Forward(74.89,75.11);
	  HAL_Delay(1000);

	  stop();
	  HAL_Delay(1000);

	  Forward(75.22,74.78);
	  HAL_Delay(1000);

	  stop();
	  HAL_Delay(1000);

	  Forward(74.78,75.22);
	  HAL_Delay(1000);

	  stop();
	  HAL_Delay(1000);



/*
    int Error= read_sensor();
	  if((Error == 0) || (Error == 1) || (Error == -1) || (Error == 2) || (Error == -2))
	  {
		  PID_control();
		 // Forward(75,75);
		  save_path('S'); //value ‘S’ saved for straight
	  }
	  else if((Error == 3) || (Error == 4) || (Error == 6))
	  {
		 // Backward(60,60);
		  stop();
		  HAL_Delay(200);
		  Turn_Right(75,35);
		 // HAL_Delay(1000);
		  save_path('R'); //value ‘R’ saved for right turn
	  }
	  else if((Error == -3) || (Error == -4) || (Error == -6))
	  {
		  Turn_Left(35,75);
		  HAL_Delay(500);
		  stop();
		  HAL_Delay(800);
		  save_path('L'); //value ‘L’ saved for left turn
	  }
	  else
	  {
		  HAL_Delay(1000);
		  stop();
	  } */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, B_IN3_Pin|B_IN4_Pin|LCD_EN_Pin|LCD_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A_IN1_Pin|A_IN2_Pin|LCD_D7_Pin|LCD_D6_Pin
                          |LCD_D4_Pin|LCD_RS_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L2_Pin L1_Pin M_Pin R1_Pin
                           R2_Pin */
  GPIO_InitStruct.Pin = L2_Pin|L1_Pin|M_Pin|R1_Pin
                          |R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : A5_EXIT15_Pin */
  GPIO_InitStruct.Pin = A5_EXIT15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(A5_EXIT15_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_IN3_Pin B_IN4_Pin LCD_EN_Pin LCD_RW_Pin */
  GPIO_InitStruct.Pin = B_IN3_Pin|B_IN4_Pin|LCD_EN_Pin|LCD_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A_IN1_Pin A_IN2_Pin LCD_D7_Pin LCD_D6_Pin
                           LCD_D4_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = A_IN1_Pin|A_IN2_Pin|LCD_D7_Pin|LCD_D6_Pin
                          |LCD_D4_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_D5_Pin */
  GPIO_InitStruct.Pin = LCD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*-----------------------------------------------------------Forward-------------------------------------------------------*/
void Forward(double PWM_L,double PWM_R)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_L);      //Left  A_EN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_R);      //Right B_EN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}
/*-----------------------------------------------------------Backward-------------------------------------------------------*/
void Backward(double PWM_L,double PWM_R)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_L);     //Left
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_R);     //Right
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	/*
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_L);      //Left  A_EN
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_R);      //Right B_EN
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			*/
}
/*---------------------------------------------------------------stop--------------------------------------------------------------*/
void stop()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); // A_EN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_0, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // B_EN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
}
/*-------------------------------------------------------------------Turn_Left------------------------------------------------------------*/
void Turn_Left(double PWM_L,double PWM_R)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_L);     //Left
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_R);     //Right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

}
/*-------------------------------------------------------------------Turn_Right----------------------------------------------------------*/
void Turn_Right(double PWM_L,double PWM_R)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_L);     //Left
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_R);     //Right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

}
/*-------------------------------------------------------------Sensor_read----------------------------------------------------------*/
int read_sensor()
{
sensor[0]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
sensor[1]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
sensor[2]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
sensor[3]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
sensor[4]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);

int error ;
if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)   //11111
{
    error = 5;
}
if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0)   //00000
{
    error = -5;
}
if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 0)   //11110
{
    error = 4;
}
if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0)   //11100
{
    error = 3;
}
if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 1)   //11101
{
    error = 2;
}
if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1)   //11001
{
     error= 1;
   // last_end = 1;
}
if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)   //11011
{
    error = 0;
   // last_end = 1;
}
if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1)   //10011
{
    error = -1;
}
if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)   //10111
{
    error = -2;
}
if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)   //00111
{
    error = -3;
}
if(sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)   //01111
{
    error = -4;
 //   last_end = 0;
}
if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0)   //10000
{
    error = 6;
}
if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1)   //00001
{
    error = -6;
}
return error;
}
/*-----------------------------------------------------------PID_control---------------------------------------------------------*/
void PID_control()
{
	int Error = read_sensor();
	P = Error;
	I += Error;
	D = Error - lastError ;
	lastError = Error;
  int motorspeed = P*Kp + ki*I +Kd*D;
  int motorspeedl = basespeedl + motorspeed ;
  int motorspeedr = basespeedr - motorspeed ;

	if (motorspeedl > maxspeedl)  motorspeedl = maxspeedl;
    if (motorspeedr > maxspeedr)  motorspeedr = maxspeedr;
    Forward(motorspeedl,motorspeedr);

}
/*------------------------------------------------------------------------------------------------------------------*/
void save_path(char dir) //saves dry run data in array path[]
{
path[pathLength] = dir;
pathLength++;
HAL_Delay(1);   //_delay_us(1);
}
/*------------------------------------------------------------------------------------------------------------------*/
/*void Make_Decession(char Incoming_Command) {
  switch (Incoming_Command) {
    case 'S':
      Go_one_step();
      break;

    case 'R':
      GO_RIGHT();
      break;

    case 'L':
      GO_LEFT();
      break;

    case 'O':
      Finish_Point();
      break;
  }
}*/
/*--------------------------------------------------A7_EXIT Button ++ LED ON / OFF -------------------------------------------------*/
/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == A5_EXIT15_Pin)
	//if( A5_EXIT15_Pin==0)
	{
	    //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	}
}*/

/*--------------------------------------------------------------------------------------------------*/
/*void leftHandWall()
{
	if(sensor[0]==0 && sensor[4]==0){        //indicates either 3way intersection or end of maze
		//GO_One_Step();
		read_sensor();
		if(sensor[0]==0 || sensor[4]==0)    //if it moves forward and still sees all black, maze is done
		{
			//Finish();
		}
		if(sensor[0]==1 && sensor[4]==1)   //otherwise it turns left
		{
			Turn_Left(35,75);
		}
	}

}*/
/*--------------------------------------------------------------------------------------------*/
void GO_One_Step()
{
	Forward(75,75);
	HAL_Delay(100);
	stop();
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
