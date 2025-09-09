/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void full_speed(void) {

	TIM1->CCR1 = 15999;
	TIM1->CCR2 = 15999;

}

void half_speed(void) {

	TIM1->CCR1 = 8000;
	TIM1->CCR2 = 8000;

}

void turn_left_timed(int ms) {
	TIM1->CCR1 = 1600;
	TIM1->CCR2 = 9599;
	HAL_Delay(ms); //delay timing subject to further testing
}

void turn_right_timed(int ms) {
	TIM1->CCR1 = 9599;
	TIM1->CCR2 = 1600;
	HAL_Delay(ms); //delay timing subject to further testing
}

void turn_left_sensor(void) {

}

void turn_right_sensor(void) {

}

void straight_ahead(void) {

}
void stop(void) {

}

int error_calculation(void) {

	//M is middle L is left LL is far left same for right sensors R for Right RR for far right, T for Top and B for bottom

	int M = 0;
	int L = 0;
	int R = 0;
	int LL = 0;
	int RR = 0;
	int error = 0;

	M = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	L = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	R = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	LL = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
	RR = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);

	error = (LL * (-2)) + (L * (-1)) + (M * 0) + (R * 1) + (RR * 2);

	return error;

}

void set_motor_speed(int left_speed, int right_speed) {
	TIM1->CCR1 = left_speed;
	TIM1->CCR2 = right_speed;
}

void special_turns(void) {
	//M is middle L is left LL is far left same for right sensors R for Right RR for far right, T for Top and B for bottom
	int M = 0;
	int L = 0;
	int R = 0;
	int LL = 0;
	int RR = 0;
	int T = 0;
	int B = 0;

	M = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	L = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	R = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	LL = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
	RR = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	T = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
	B = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

	if ((M == 1) && (L == 1) && (LL == 1) && (R == 0) && (RR == 0)) //left right angle turn
			{
		turn_left_timed(300); //delay subject to further testing
	} else if ((M == 1) && (L == 0) && (LL == 0) && (R == 1) && (RR == 1)) //right right angle turn
			{
		turn_right_timed(300); //delay subject to further testing
	} else if ((M == 1) && (L == 1) && (T == 1) && (B == 1) && (R == 1)) //+ sign junction
			{
		straight_ahead();
		HAL_Delay(100);
	} else if ((M == 1) && (L == 0) && (LL == 0) && (R == 0) && (RR == 1)) //obtuse angle turn Right
			{
		turn_right_sensor();
	} else if ((M == 1) && (L == 0) && (LL == 1) && (R == 0) && (RR == 0)) //obtuse angle turn Left
			{
		turn_left_sensor();
	}

}

int check_inverted(void) {

	//IL is left invert check sensor and IR is right invert check sensor

	int IL = 0;
	int IR = 0;

	IL = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	IR = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

	if ((IL == 0) && (IR == 0)) {
		return 1;
	} else {
		return 0;
	}

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	float Kp = 5;
	float Ki = 5;
	float Kd = 5;
	float p = 0;
	float i = 0;
	float d = 0;
	float correction = 0;
	float base_speed = 8000; //50 percent duty cycle range of pwm is 0 to 15999
	float max_speed = 15999;
	float zero_speed = 0;
	int left_speed = 0;
	int right_speed = 0;
	int error = 0;
	int prev_error = 0;
	int invert_check = 0;

	HAL_TIMEx_PWMN_Start(&htim1, 1);
	HAL_TIMEx_PWMN_Start(&htim1, 2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		invert_check = check_inverted();
		if (invert_check == 1) {
			//inverted motor handling in a while loop until inverted section over
		}

		error = error_calculation();
		p = error;
		i = i + error;
		d = error - prev_error;
		prev_error = error;

		//(ccr/arr) * 100

		correction = (Kp * p) + (Ki * i) + (Kd * d);

		left_speed = base_speed + correction;
		right_speed = base_speed - correction;

		if (left_speed > max_speed) {
			left_speed = max_speed;
		}
		if (right_speed > max_speed) {
			right_speed = max_speed;
		}
		if (left_speed < zero_speed) {
			 left_speed = 0;
		}
		if (right_speed < 0) {
			 right_speed= 0;
		}

		set_motor_speed(left_speed, right_speed);

		special_turns();

		invert_check = 0;

	}
}
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 4;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 63999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pins : Inv_Det_L_Pin Inv_Det_R_Pin */
	GPIO_InitStruct.Pin = Inv_Det_L_Pin | Inv_Det_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : RR_Pin R_Pin Bottom_Pin Middle_Pin
	 Top_Pin L_Pin LL_Pin */
	GPIO_InitStruct.Pin = RR_Pin | R_Pin | Bottom_Pin | Middle_Pin | Top_Pin
			| L_Pin | LL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

