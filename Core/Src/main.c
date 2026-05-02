/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include <stdint.h>
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

/* USER CODE BEGIN PV */
#define ADC_BUF_LEN 400 // DO DAI CUA MANG ADC
uint16_t emg_buffer[ADC_BUF_LEN]; // MANG CHUA TONG HOP DATA TU DMA

uint16_t emg_filtered_1[200]; // LUU GIA TRI SAU KHI LOC PA0
uint16_t emg_filtered_2[200]; // LUU GIA TRI SAU KHI LOC PA1

volatile uint8_t flag_half_ready = 0; // KHI CO 100 PHAN TU DAU TIEN MOI KENH
volatile uint8_t flag_full_ready = 0; // KHI CO 100 PHAN TU CUOI CUNG MOI KENH

#define WINDOW_SIZE 16 // KICH THUOC CUA SO THANH 16 (LUY THUA BANG 2) THAY VI 10
char msg_buffer[80]; // BUFFER CHUA CHUOI GUI UART 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void EMG_Filter_Channel(uint16_t *input_dma, uint16_t *output_filtered, int channel_offset, int start_idx, int length);
void UART_Send_EMG(uint16_t *data1, uint16_t *data2, int start_idx, int length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void EMG_Filter_Channel(uint16_t *input_dma, uint16_t *output_filtered, int channel_offset, int start_idx, int length)
{
  for (int i = start_idx; i < start_idx + length; i++)
  {
    uint32_t sum = 0;
    for (int j = 0; j < WINDOW_SIZE; j++) 
    {
      // Tính chỉ số lịch sử vòng tròn. 
      // 'i' bị trừ lùi đi để quét lịch sử, cộng 200 tránh số âm trước khi chia % 200
      int history_idx = (i - j + 200) % 200;
      
      // Trích xuất giá trị trực tiếp từ luồng gộp của DMA (chẵn/lẻ)
      sum += input_dma[2 * history_idx + channel_offset];
    }
    // Dịch logic qua phải 4 byte tương đương thao tác chia `(sum / 16)`
    output_filtered[i] = sum >> 4;
  }
}

void UART_Send_EMG(uint16_t *data1, uint16_t *data2, int start_idx, int length)
{
  for (int i = start_idx; i < start_idx + length; i+=5)
  {
    sprintf(msg_buffer, "%u,%u\n", data1[i], data2[i]);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg_buffer, strlen(msg_buffer), 5);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)emg_buffer, ADC_BUF_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // KIEM TRA NEU NUA DAU DA SAN SANG
    if (flag_half_ready == 1)
    {
      // 1. Máy chạy lọc trung bình trực tiếp truy xuất luồng DMA 
      // Channel 0 (Chẵn), Channel 1 (Lẻ)
      EMG_Filter_Channel(emg_buffer, emg_filtered_1, 0, 0, 100);
      EMG_Filter_Channel(emg_buffer, emg_filtered_2, 1, 0, 100);
      
      // 2. Gửi dữ liệu qua UART
      UART_Send_EMG(emg_filtered_1, emg_filtered_2, 0, 100);
      
      flag_half_ready = 0;
    }

    // KIEM TRA NEU NUA SAU DA SAN SANG
    if (flag_full_ready == 1)
    {
      // 1. Tương tự lọc cho nửa sau của luồng dữ liệu DMA
      EMG_Filter_Channel(emg_buffer, emg_filtered_1, 0, 100, 100);
      EMG_Filter_Channel(emg_buffer, emg_filtered_2, 1, 100, 100);
      
      // 2. Gửi dữ liệu qua UART
      UART_Send_EMG(emg_filtered_1, emg_filtered_2, 100, 100);
      
      flag_full_ready = 0;
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// KIEM TRA XEM DU 100 PHAN TU DAU TIEN (DMA NGẮT NỬA)
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) 
  {
    flag_half_ready = 1; // Chỉ dùng cờ để giải phóng CPU khỏi vòng FOR
  }
}
// KIEM TRA XEM DU 100 PHAN TU CUOI CUNG (DMA NGẮT ĐẦY)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) 
  {
    flag_full_ready = 1; // Tương tự nửa sau
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
#ifdef USE_FULL_ASSERT
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
