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
#include "dma.h"
#include "i2s.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef volatile int16_t sample_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define PC_UART     huart2
#define MIC_SAMPLE_PER_FRAME	16

#define DMA_MIC_BUFFER_IN_SIZE (MIC_SAMPLE_PER_FRAME * 4)

/*static*/ sample_t dma_mic_buffer[DMA_MIC_BUFFER_IN_SIZE];//буфер дма получения с микрофона
static sample_t out_buffer[MIC_SAMPLE_PER_FRAME];//буфер, на отправку в уарт
static volatile int dma_i2s_recive = 0;//флаг дма микрофона
static volatile int dma_uart_transmit = 1; //флаг

bool _running;// флаг
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void processing_ready_buffer(sample_t* buff, size_t sz);
void transmit_frame_to_uart(sample_t* buff, size_t sz);
int8_t microphone_start_stop (bool);
int8_t button;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2S1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  microphone_start_stop (true);
  while (1)
  {
	button = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	if (button == 0)
	{
		if(dma_i2s_recive == 1)
		{
			dma_i2s_recive = 0;
			processing_ready_buffer(&dma_mic_buffer[0], DMA_MIC_BUFFER_IN_SIZE/2);
		}
		else if (dma_i2s_recive == 2)
		{
			dma_i2s_recive = 0;
			processing_ready_buffer(&dma_mic_buffer[DMA_MIC_BUFFER_IN_SIZE/2], DMA_MIC_BUFFER_IN_SIZE/2);

		}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





/**
 * Processing received I2S buffer
 */
sample_t max_sample;
void processing_ready_buffer(sample_t* buff, size_t sz){

    /* Auto select left/right */
    if(*buff == 0 && *(buff+2) == 0)
    {
        buff++;
    }
    /* */

	for (size_t i=0; i<sizeof(out_buffer)/sizeof(sample_t); i++)
	{
		out_buffer [i] = buff [i*2];
	}
    transmit_frame_to_uart(out_buffer, MIC_SAMPLE_PER_FRAME);

}

/**
 * Processing received I2S buffer
 */
void transmit_frame_to_uart(sample_t* buff, size_t sz){
    while(dma_uart_transmit == 0){}//;ждем пока все посылки переданы по DMA

    dma_uart_transmit = 0;
    HAL_UART_Transmit_DMA(&PC_UART, (uint8_t*)buff, sz*sizeof(sample_t));
}


/**
 * Start/Stop the I2S DMA transfer
 */
int8_t microphone_start_stop(bool enable) {

	HAL_StatusTypeDef status;
	if (enable == true)
	{
		status = HAL_I2S_Receive_DMA(&hi2s1, (uint16_t *)dma_mic_buffer, DMA_MIC_BUFFER_IN_SIZE);
		if (status == HAL_OK)
		{
			_running = true;
		}
	}
	else
	{
		  status = HAL_I2S_DMAStop(&hi2s1);
		  if (status == HAL_OK)
		 {
			_running = false;
		 }
	}
	return status;
}


void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	dma_i2s_recive = 1;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
    dma_i2s_recive = 2;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart == &PC_UART)
    {
        dma_uart_transmit = 1;
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
