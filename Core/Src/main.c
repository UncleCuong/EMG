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
#include "features.h"
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
char msg_buffer[200]; // BUFFER CHUA CHUOI GUI UART (increased for JSON)

// Sliding window buffers for feature extraction
#define WINDOW_LEN 200
#define WINDOW_OVERLAP 100
uint16_t window_ch1[WINDOW_LEN];  // Sliding window for channel 1
uint16_t window_ch2[WINDOW_LEN];  // Sliding window for channel 2
volatile int window_index = 0;    // Current position in window
volatile int samples_in_window = 0;  // Number of samples in window
volatile uint8_t window_ready_flag = 0;  // Flag when window is full (200 samples)

// Features storage
EMGFeatures_t emg_features;  // Struct to store calculated features
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
    // Check if sliding window is ready (200 samples collected)
    if (window_ready_flag == 1)
    {
      // Calculate features for channel 1 (PA0)
      emg_features.ch1.rms = calculate_RMS(window_ch1, WINDOW_LEN);
      emg_features.ch1.mav = calculate_MAV(window_ch1, WINDOW_LEN);
      emg_features.ch1.mf = calculate_MF(window_ch1, WINDOW_LEN, 1000);  // 1kHz sampling

      // Calculate features for channel 2 (PA1)
      emg_features.ch2.rms = calculate_RMS(window_ch2, WINDOW_LEN);
      emg_features.ch2.mav = calculate_MAV(window_ch2, WINDOW_LEN);
      emg_features.ch2.mf = calculate_MF(window_ch2, WINDOW_LEN, 1000);  // 1kHz sampling

      // Format features to JSON string (values as integers × 100 to preserve 2 decimal places)
      // Example: rms=512.45 becomes "rms":51245 in JSON
      int json_len = format_features_json(&emg_features, msg_buffer);

      // Transmit over UART (newline already included in JSON)
      if (json_len > 0)
      {
        HAL_UART_Transmit(&huart1, (uint8_t*)msg_buffer, json_len, 100);
      }

      // Reset flag
      window_ready_flag = 0;
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

/**
 * @brief Process ADC samples and update sliding windows
 * Extracts samples from DMA buffer and feeds into sliding windows
 * @param buffer: pointer to ADC DMA buffer (interleaved ch0, ch1, ch0, ch1, ...)
 * @param start_idx: starting index in buffer
 * @param num_pairs: number of interleaved pairs (e.g., 100 pairs = 200 elements)
 */
static void ADC_UpdateSlidingWindows(uint16_t *buffer, int start_idx, int num_pairs)
{
  for (int i = 0; i < num_pairs; i++)
  {
    // Extract channel 0 (even indices) and channel 1 (odd indices)
    uint16_t ch0_sample = buffer[start_idx + 2 * i];
    uint16_t ch1_sample = buffer[start_idx + 2 * i + 1];

    // Add to sliding windows
    window_ch1[window_index] = ch0_sample;
    window_ch2[window_index] = ch1_sample;

    window_index++;
    samples_in_window++;

    // Check if window is full (200 samples)
    if (samples_in_window >= 200)
    {
      window_ready_flag = 1;
      samples_in_window = 0;
      window_index = 0;
      // Shift buffer by copying last 100 to first 100 (overlap = 100)
      for (int j = 0; j < 100; j++)
      {
        window_ch1[j] = window_ch1[100 + j];
        window_ch2[j] = window_ch2[100 + j];
      }
      window_index = 100;
      samples_in_window = 100;
    }
  }
}

/**
 * @brief ADC conversion half-complete callback
 * Called when DMA reaches the middle of the buffer (100 pairs = 200 samples)
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    // Process first half of ADC buffer (samples 0-199, which is 100 pairs of channels)
    ADC_UpdateSlidingWindows(emg_buffer, 0, 100);
  }
}

/**
 * @brief ADC conversion complete callback
 * Called when DMA completes the buffer (100 pairs = 200 samples)
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    // Process second half of ADC buffer (samples 200-399, which is 100 pairs of channels)
    ADC_UpdateSlidingWindows(emg_buffer, 200, 100);
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
