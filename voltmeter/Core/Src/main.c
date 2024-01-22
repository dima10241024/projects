/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "MAX30102.h"
#include "printf.h"
#include "algorithm_by_RF.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_BRIGHTNESS 		255U
#define MY_SNPRINTF 		snprintf	// newlib snprintf with float support is too heavy and not threadsafe so using local snprintf from printf.c
										// but this causes a compile warning that float support is off so use this macro to prevent it
#define FAKE_MAX30102_CLONE				// if you have a fake Chinese cloned MAX30102 define this

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
static uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static void Max30102Setup(void);
static void Max30102Loop(void);
static void DebugPrint(const char *text);
void SetPwmDutyCycle(uint8_t percent);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DebugPrint(const char *text)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)text, strlen(text), 100U);
}



void Max30102Setup()
{
  uint8_t uch_dummy;

  maxim_max30102_reset();
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy);  // reads/clears the interrupt status register
  maxim_max30102_init();
}

// Takes samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void Max30102Loop()
{
  char buf[64];
  float n_spo2;
  float ratio;
  float correl;
  int8_t ch_spo2_valid;  				// indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; 				// heart rate value
  int8_t  ch_hr_valid;  				// indicator to show if the heart rate calculation is valid
  uint8_t i;
  static uint32_t un_min = 0x3FFFFUL;
  static uint32_t un_max = 0UL;
  static uint32_t un_prev_data = 0UL;  	// variables to calculate the on-board LED brightness that reflects the heartbeats
  static float f_brightness = 0UL;
  float f_temp;

  // buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  for (i = 0U; i < BUFFER_SIZE; i++)
  {
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET);	// wait until the interrupt pin asserts

#ifdef FAKE_MAX30102_CLONE
    maxim_max30102_read_fifo((aun_ir_buffer + i), (aun_red_buffer + i));  // read from MAX30102 FIFO
#else
    maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));  // read from MAX30102 FIFO
#endif
    // calculate LED brightness
    if (aun_red_buffer[i] > un_prev_data)
    {
      f_temp = aun_red_buffer[i] - un_prev_data;
      f_temp /= (un_max - un_min);
      f_temp *= MAX_BRIGHTNESS;
      f_brightness -= 2.0f * f_temp;
      if (f_brightness < 0.0f)
      {
        f_brightness = 0.0f;
      }
    }
    else
    {
	  f_temp = un_prev_data - aun_red_buffer[i];
	  f_temp /= (un_max - un_min);
	  f_temp *= MAX_BRIGHTNESS;
	  f_brightness += 2.0f * f_temp;
	  if (f_brightness > (float)MAX_BRIGHTNESS)
	  {
		f_brightness = (float)MAX_BRIGHTNESS;
	  }
    }


    un_prev_data = aun_red_buffer[i];
  }

  un_min = 0x3FFFFUL;
  un_max = 0UL;
  for (i = 0U; i < BUFFER_SIZE; i++)
  {
    if (un_min > aun_red_buffer[i])
    {
      un_min = aun_red_buffer[i];    //update signal min for next cycle using values from this cycle
    }
    if (un_max < aun_red_buffer[i])
    {
      un_max = aun_red_buffer[i];    //update signal max for next cycle using values from this cycle
    }
  }

  // calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);

  // display results
  if (ch_hr_valid && ch_spo2_valid)
  {
    MY_SNPRINTF(buf, (size_t)64, "SpO2: %3.1f\t", n_spo2);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buf, Font_7x10, White);
    DebugPrint(buf);
    snprintf(buf, (size_t)64, "heart rate: %d\r\n", n_heart_rate);
	ssd1306_SetCursor(0, 15);
	ssd1306_WriteString(buf, Font_7x10, White);
	ssd1306_UpdateScreen();
    DebugPrint(buf);

	  HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);				//переключаем состояние светодиода ЗУММЕРААААААА
	  HAL_Delay(300);												//задержка 1000мс=1с
	  HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);				//переключаем состояние светодиода ЗУММЕРААААААА
  }
  else
  {
	ssd1306_Fill(Black);
    DebugPrint("I can' t find a pulse\r\n");
    snprintf(buf, (size_t)64, "Not valid.");
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buf, Font_7x10, White);
    snprintf(buf, (size_t)64, "I can't find pulse");
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString(buf, Font_7x10, White);
    ssd1306_UpdateScreen();
  }
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  Max30102Setup();
  ssd1306_Init();

  int f=0; 														//флаг вывода значения ацп
  int but=1; 													//текущее состояние кнопки
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  but = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);		//считываем состояние кнопки
	  if (but == 0){
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }
	  /*
	  if (but == 0){											//если кнопка нажата (замкнули на землю), в ioc  настраивали pull up для пина
		  if (f == 0)											//если значение ацп не выводится
		  {
			  f=1;												//меняем флаг чтоб значение ацп выводилось
		  }
		  else if (f == 1)										//если значение ацп выводится
		  {
			  f=0;												//меняем флаг чтоб значение ацп не выводилось
		  }
	  }
	  if(f == 1)												//если флаг настроен для вывода значение ацп
	  {
		  printf("adc value = %d\r\n", adc);					//значение ацп от 0 до 4096
	  }

	  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);				//переключаем состояние светодиода
	  //HAL_Delay(1000);											//задержка 1000мс=1с
	  */


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Max30102Loop();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
