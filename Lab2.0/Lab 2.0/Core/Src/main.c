/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "oled.h"
#include <string.h>
#include <MPU6050.h>  // Include the MPU6050 driver
#include <math.h>     // Include math library for atan2 function

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
 I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
IMU_Data imu;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint16_t tc1 = 0;   // Capture values
volatile uint16_t tc2 = 0;
volatile uint16_t echo = 0;  // Pulse width
uint32_t millisOld = 0, millisNow = 0;
float dt = 0;
float yaw = 0, pitch = 0, roll = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint16_t us);
uint8_t IMU_ADDR = 0x68 << 1;  // Address with AD0 low

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
//	uint8_t sbuf[15] = "Hello World! \n\r";
//	uint8_t *OLED_buf;
//	char buf[15];
	//----Lab 2-----//
	  char sbuf[50];

//	//Lab 2//
    IMU_Data imu;
    char msg[50];

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
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_I2C2_Init();

  uint8_t status = IMU_Initialise(&imu, &hi2c2, &huart3);
  /* USER CODE BEGIN 2 */
//  OLED_Init();
//  OLED_ShowString(10,5, "hello");
//  OLED_Refresh_Gram();
//

  //Lab 2//
//  OLED_Init();
//  OLED_ShowString(10, 5, "Initializing...");
//  OLED_Refresh_Gram();

  //lab try 2//
  uint32_t millisOld = 0, millisNow = 0;
  float dt, pitch_acc = 0, roll_acc = 0;

  float pitch_gyro = 0, roll_gyro = 0,yaw_gyro = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //-----LED------
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	   HAL_Delay(50);
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	   HAL_Delay(50);

	  //-----Buzzer-----
//	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
//	   HAL_Delay(1000);
//	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//	   HAL_Delay(1000);

	  //-----UART-----
//	   uint8_t sbuf[] = "Hello World.\n\r";
//	   HAL_UART_Transmit(&huart3,sbuf,sizeof(sbuf),HAL_MAX_DELAY);

//	  //---------ULTRA Sound---------
//	  //Set TRIG to low for awhile
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
//	  HAL_Delay(50);
//
//	  //send sound
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
//	  delay_us(10);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
//	  HAL_Delay(50);
//
//	  //read echo
//	  // HAL_TIM_Base_Start(&htim6); //seems not needed as well. I dont see the purpose
//	  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
//	  // HAL_TIM_IC_CaptureCallback(&htim8); //Not needed as called automatically as callback
//
//	  //measure
//	  char buf1[15];
//	  char buf2[15];
//
//	  sprintf(buf1, "Echo = %5d us ", echo);
//	  OLED_ShowString(10,40, &buf1[0]);
//
//	  float speed_of_sound = 0.0343;
//	  int tester = echo * (speed_of_sound /2) ;
//	  printf("%d",tester);
//	  sprintf(buf2,"Dis = %4d cm ", tester);
//	  OLED_ShowString(10,50, &buf2[0]);
//	  OLED_Refresh_Gram();


	   // LAb 2 lowkey work pundai//
      /*IMU_AccelRead(&imu);
      IMU_GyroRead(&imu);

       // Format and transmit the accelerometer data
       for (int i = 0; i < 3; i++) {
    	   sprintf(sbuf, "Accel %d: %5.2f\r\n", i, imu.acc[i]);
           HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);
       }

       // Format and transmit the gyroscope data
      *for (int i = 0; i < 3; i++) {
    	   sprintf(sbuf, "Gyro %d: %5.2f\r\n", i, imu.gyro[i]);
          HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);
      }
       HAL_Delay(500);*/

	   //lab 2 sending raw data works//

	   /* millisNow = HAL_GetTick();
	    dt = (millisNow - millisOld) * 0.001;  // Calculate time elapsed in seconds
	    millisOld = millisNow;

	    IMU_AccelRead(&imu);  // Read accelerometer data
	    IMU_GyroRead(&imu);   // Read gyroscope data

	    // Print raw sensor data
	    sprintf(sbuf, "%5.2f, %5.2f, %5.2f, %5.2f, %5.2f, %5.2f\r\n",
	            imu.acc[0], imu.acc[1], imu.acc[2], imu.gyro[0], imu.gyro[1], imu.gyro[2]);
	    HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);

	    // Calculate angles
	    roll = atan2(imu.acc[1], imu.acc[2]) * 180.0 / M_PI;
	    pitch = atan2(-imu.acc[0], sqrt(imu.acc[1]*imu.acc[1] + imu.acc[2]*imu.acc[2])) * 180.0 / M_PI;

	    yaw += imu.gyro[0] * dt;
	    pitch += imu.gyro[1] * dt;
	    roll += imu.gyro[2] * dt;

	    // Print calculated angles
	    sprintf(sbuf, "%5.2f, %5.2f, %5.2f\r\n", pitch, roll, yaw);
       HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);

	    HAL_Delay(100);  // Adjust delay as needed*/



	    //lab 2 only angles WORKING!!!!//

	    // Get current time
	    uint32_t millisNow = HAL_GetTick();
	    float dt = (millisNow - millisOld) * 0.001f;  // Calculate time elapsed in seconds
	    millisOld = millisNow;
       // Read sensor data
	    IMU_AccelRead(&imu);
	    IMU_GyroRead(&imu);

	    // Calculate angles from accelerometer data
	    roll_acc = atan2 (imu.acc[0]/9.8, imu.acc[2]/9.8) *(180/M_PI);
	    pitch_acc = atan2(imu.acc[1]/9.8, imu.acc[2]/9.8)*(180/M_PI);

	    sprintf(sbuf, "%.2f, %.2f\r\n", pitch_acc, roll_acc);
	    HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);

	    //Gyroscope
	    //roll_gyro = roll_gyro-imu.gyro[1]*dt;
	    //pitch_gyro = pitch_gyro-imu.gyro[0]*dt;
	    //yaw_gyro = yaw_gyro-imu.gyro[2]*dt;

	    //sprintf(sbuf, "%.2f, %.2f, %.2f\r\n", pitch_gyro, roll_gyro,yaw_gyro);
	    //HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);




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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|US_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 US_Trig_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|US_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RESET__Pin SDIN_Pin SCLK_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin LED_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void delay_us(uint16_t us){
	HAL_TIM_Base_Start(&htim6);
	__HAL_TIM_SET_COUNTER(&htim6, 0);

	while(__HAL_TIM_GET_COUNTER(&htim6) < us);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM8) {
        GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);

        if (pinState == GPIO_PIN_SET) {  // Positive edge
            tc1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // Capture the value
        } else if (pinState == GPIO_PIN_RESET) {  // Negative edge
            tc2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // Capture the value

            // Calculate the pulse width
            if (tc2 >= tc1) {
                echo = tc2 - tc1;
            } else {
                echo = (65536 - tc1) + tc2;  // Handle overflow
            }
        }
    }
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
