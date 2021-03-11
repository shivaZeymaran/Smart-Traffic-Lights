/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRAFFIC_LIGHT_1 0
#define TRAFFIC_LIGHT_2 1

#define LIGHT_1_PORT GPIOA
#define LIGHT_1_RED GPIO_PIN_10
#define LIGHT_1_YELLOW GPIO_PIN_11
#define LIGHT_1_GREEN GPIO_PIN_12

#define LIGHT_2_PORT GPIOC
#define LIGHT_2_RED GPIO_PIN_2
#define LIGHT_2_YELLOW GPIO_PIN_3
#define LIGHT_2_GREEN GPIO_PIN_4

// 3 states for line
#define NORMAL 0
#define TRAFFIC 1
#define CLOSED 2

#define START_TIME 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint16_t num_of_cars[] = {0, 0}; // line1, line2
volatile uint16_t total_time = START_TIME;
uint16_t line_1_density;
uint16_t line_2_density;
// pot value is between 0-A2
uint16_t A_th = 33;  // minimum threshold for potentiometer
uint16_t B_th = 66;
uint8_t reset_lights = 0; // to change state of traffic lights
uint8_t last_red = TRAFFIC_LIGHT_1;
uint16_t t_n = 2;
uint16_t t_h = 3;
uint16_t counter = 0;
uint16_t temp_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0){
		num_of_cars[0]++;
	}
	else if(GPIO_Pin == GPIO_PIN_1){
		num_of_cars[1]++;
	}
}

void set_traffic_light(uint16_t light_number, uint16_t color_pin)
{
	if(light_number == TRAFFIC_LIGHT_1){
	    HAL_GPIO_WritePin(LIGHT_1_PORT, LIGHT_1_RED, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(LIGHT_1_PORT, LIGHT_1_YELLOW, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(LIGHT_1_PORT, LIGHT_1_GREEN, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(LIGHT_1_PORT, color_pin, GPIO_PIN_SET);
	}
	else if(light_number == TRAFFIC_LIGHT_2){
		HAL_GPIO_WritePin(LIGHT_2_PORT, LIGHT_2_RED, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(LIGHT_2_PORT, LIGHT_2_YELLOW, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(LIGHT_2_PORT, LIGHT_2_GREEN, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(LIGHT_2_PORT, color_pin, GPIO_PIN_SET);
	}
}

void set_seven_segment(GPIO_TypeDef *GPIOx, int num, uint16_t GPIO_Pin1, uint16_t GPIO_Pin2, 
                        uint16_t GPIO_Pin3, uint16_t GPIO_Pin4) {
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin1, num%2);
  num /= 2;
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin2, num%2);
  num /= 2;
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin3, num%2);
  num /= 2;
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin4, num%2);
}

void set_7segment_pairs(uint16_t time_left)
{
  // First pair
  set_seven_segment(GPIOA, time_left/10, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9); // left one
  set_seven_segment(GPIOA, time_left%10, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5);

  // Second pair
  set_seven_segment(GPIOB, time_left/10, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7); // left one
  set_seven_segment(GPIOB, time_left%10, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3);
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if(temp_t != 2000){ // :)
	  counter++;
	}
	if(counter == temp_t && num_of_cars[!last_red] > 0){
		num_of_cars[!last_red]--;
		counter = 0;
	}
	if(total_time > 1){
		total_time--;	
	}
	else{
		reset_lights = 1;
	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

uint8_t get_line_state(uint16_t light_number)
{
	if(light_number == TRAFFIC_LIGHT_1){
		if(line_1_density < A_th)
			return NORMAL;
		else if(A_th <= line_1_density && line_1_density <= B_th)
			return TRAFFIC;
		else  // density > B_th
			return CLOSED;
	}
	else{  // traffic_light_2
		if(line_2_density < A_th)
			return NORMAL;
		else if(A_th <= line_2_density && line_2_density <= B_th)
			return TRAFFIC;
		else  // density > B_th
			return CLOSED;
	}
}

// main logic
void handle_lights(void)
{
	counter = 0;
	uint16_t last_red_state = get_line_state(last_red);
	__disable_irq();
	if(last_red_state == NORMAL){
		if(num_of_cars[last_red] == 0 && num_of_cars[!last_red] > 0){  // there is no car in the line that wants to be green
			total_time = 1;
		}
		else{
			total_time = num_of_cars[last_red] * t_n;
	    	total_time = (total_time == 0)?(START_TIME):(total_time);  // there are no cars in both lines
		}
	}
	else if(last_red_state == TRAFFIC){
		if(num_of_cars[last_red] == 0 && num_of_cars[!last_red] > 0){
			total_time = 1;
		}
		else{
			total_time = num_of_cars[last_red] * t_h;
	    	total_time = (total_time == 0)?(START_TIME):(total_time);
		}
	}
	else{  // closed
		if(num_of_cars[last_red] == 0 && num_of_cars[!last_red] > 0){
			total_time = START_TIME;
			__enable_irq();
			return;
		}
		total_time = 2 * START_TIME;
	}
	__enable_irq();

	uint16_t green_pin = (last_red == TRAFFIC_LIGHT_1)?(LIGHT_1_GREEN):(LIGHT_2_GREEN);
    set_traffic_light(last_red, green_pin);

    uint16_t red_pin = (!last_red == TRAFFIC_LIGHT_1)?(LIGHT_1_RED):(LIGHT_2_RED);
    set_traffic_light(!last_red, red_pin);

    last_red = !last_red;
}

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  set_traffic_light(TRAFFIC_LIGHT_1, LIGHT_1_RED);
  set_traffic_light(TRAFFIC_LIGHT_2, LIGHT_2_GREEN);
  HAL_TIM_Base_Start_IT(&htim2);

  	//SETUP ADC
	RCC->AHB1ENR |= 7;
	GPIOA->MODER |= 0xC;
	GPIOC->MODER |= 0x3;
	RCC->APB2ENR |= 0x00000100;
	ADC1->CR1 = 0x00000100;   // scan
	ADC1->CR2 = 0;
	ADC1->SQR3 = 0x00000141;  // for in1 and in10
	ADC1->SQR1 = 0x00100000;  // for 2 conversions
	ADC1->CR2 |= 1;
	
	//END SETUP ADC

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	set_7segment_pairs(total_time);
  	ADC1->CR2 |= 0x40000000; /* start a conversion */

	while(!(ADC1->SR & 2)) {} /* wait for conv complete */
	line_1_density = ADC1->DR / 40; /* read conversion result */

	HAL_Delay(500);

	while(!(ADC1->SR & 2)) {} /* wait for conv complete */
	line_2_density = ADC1->DR / 40; /* read conversion result */

	if(reset_lights == 1){
		reset_lights = 0;
		handle_lights();
	}

	uint8_t temp_state = get_line_state(!last_red);
	if(temp_state == NORMAL){
	    temp_t = t_n;
	}
	else if(temp_state == TRAFFIC){
		temp_t = t_h;
	}
	else{
		temp_t = 2000; // not important number:)
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_6B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000-1;  // 16000-1
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;  // 1000-1
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA6 PA7 PA8 PA9
                           PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
