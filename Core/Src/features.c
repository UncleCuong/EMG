/**
  ******************************************************************************
  * @file           : features.c
  * @brief          : EMG feature extraction implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "features.h"
#include <stdio.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
// Frequency bins for Goertzel analysis (50, 100, 150, 200, 250, 300, 350, 400, 450, 500 Hz)
#define NUM_FREQ_BINS 10
// Các tần số (Hz) dùng trong bộ lọc Goertzel để tính năng lượng tại các dải tần
// Dùng để tính Mean Frequency (MF) như một trung bình có trọng số theo năng lượng
static const float freq_bins[NUM_FREQ_BINS] = {50, 100, 150, 200, 250, 300, 350, 400, 450, 500};

/* Function implementations --------------------------------------------------*/

/**
 * @brief Calculate RMS (Root Mean Square) of signal
 * Formula: sqrt(mean(x^2))
 */
float calculate_RMS(uint16_t *data, int length)
{
  if (data == NULL || length <= 0)
    return 0.0f;

  float sum = 0.0f;
  for (int i = 0; i < length; i++)
  {
    float val = (float)data[i];
    sum += val * val;
  }

  float mean_square = sum / length;
  return sqrtf(mean_square);
}

/*
 * Chú thích (tiếng Việt):
 * - Hàm `calculate_RMS` tính giá trị RMS của một dãy mẫu ADC.
 * - RMS phản ánh năng lượng/tính mạnh của tín hiệu (có ích để so sánh biên độ).
 * - Trả về `0.0f` nếu dữ liệu rỗng hoặc độ dài không hợp lệ.
 */

/**
 * @brief Calculate MAV (Mean Absolute Value) of signal
 * Formula: mean(|x|)
 */
float calculate_MAV(uint16_t *data, int length)
{
  if (data == NULL || length <= 0)
    return 0.0f;

  float sum = 0.0f;
  for (int i = 0; i < length; i++)
  {
    sum += (float)data[i];  // uint16 is always positive
  }

  return sum / length;
}

/*
 * Chú thích (tiếng Việt):
 * - `calculate_MAV` tính giá trị trung bình của mẫu (Mean Absolute Value).
 * - Do dữ liệu ở dạng `uint16_t` (không âm), MAV là trung bình mẫu trực tiếp.
 * - MAV hữu dụng để đánh giá biên độ trung bình của tín hiệu EMG.
 */

/**
 * @brief Goertzel algorithm - compute power at specific frequency
 * Efficiently computes DFT at a single frequency bin
 */
static float Goertzel(uint16_t *data, int length, float frequency, float sampling_freq)
{
  if (length <= 0 || sampling_freq <= 0)
    return 0.0f;

  // Normalized frequency (0 to 1, where 1 = sampling_freq)
  float norm_freq = frequency / sampling_freq;

  // Goertzel coefficients
  float w = 2.0f * M_PI * norm_freq;
  float coeff = 2.0f * cosf(w);

  // Initialize accumulators
  float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;

  // Goertzel filter iterations
  for (int i = 0; i < length; i++)
  {
    float sample = (float)data[i];
    s0 = sample + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }

  // Calculate power at this frequency
  float real_part = s1 - s2 * cosf(w);
  float imag_part = s2 * sinf(w);
  float power = real_part * real_part + imag_part * imag_part;

  return power;
}

/*
 * Chú thích (tiếng Việt):
 * - Hàm `Goertzel` tính công suất (power) tín hiệu tại một tần số cụ thể bằng
 *   thuật toán Goertzel (thuận tiện khi chỉ cần vài tần số mà không cần FFT đầy đủ).
 * - Tham số `frequency` là tần số mục tiêu (Hz), `sampling_freq` là tần số lấy mẫu (Hz).
 * - Kết quả trả về là công suất tương đối tại tần số đó (không phải biên độ).
 * - Hàm bỏ qua nếu `length` hoặc `sampling_freq` không hợp lệ.
 */

/**
 * @brief Calculate MF (Mean Frequency) using Goertzel filter bank
 * Computes power at multiple frequency bins and calculates weighted mean frequency
 */
float calculate_MF(uint16_t *data, int length, uint16_t sampling_freq)
{
  if (data == NULL || length <= 0 || sampling_freq == 0)
    return 0.0f;

  float sum_power = 0.0f;
  float sum_freq_power = 0.0f;

  // Compute power at each frequency bin
  for (int i = 0; i < NUM_FREQ_BINS; i++)
  {
    float power = Goertzel(data, length, freq_bins[i], (float)sampling_freq);
    sum_power += power;
    sum_freq_power += freq_bins[i] * power;
  }

  // Calculate mean frequency (weighted average)
  float mean_freq = 0.0f;
  if (sum_power > 0.0f)
  {
    mean_freq = sum_freq_power / sum_power;
  }
  else
  {
    // Fallback to simple heuristic if no power detected
    mean_freq = 100.0f;
  }

  return mean_freq;
}

/*
 * Chú thích (tiếng Việt):
 * - `calculate_MF` dùng một bộ lọc Goertzel trên các tần số đã định để ước lượng
 *   Mean Frequency (MF) của tín hiệu: MF = sum(f_i * P_i) / sum(P_i).
 * - `P_i` là công suất tại tần số `f_i` do Goertzel tính được.
 * - Nếu không có công suất (sum_power == 0), trả về giá trị mặc định 100 Hz.
 */

/**
 * @brief Format EMG features to JSON string
 * Output: {"ch1": {"rms": 51245, "mav": 32432, "mf": 12550}, "ch2": {...}}
 * Note: Values are multiplied by 100 to preserve 2 decimal places as integers
 * Example: 512.45 → 51245
 */
int format_features_json(EMGFeatures_t *features, char *buffer)
{
  if (features == NULL || buffer == NULL)
    return 0;

  // Convert float values to integers by multiplying by 100
  // This preserves 2 decimal places
  int ch1_rms = (int)(features->ch1.rms * 100);
  int ch1_mav = (int)(features->ch1.mav * 100);
  int ch1_mf = (int)(features->ch1.mf * 100);
  int ch2_rms = (int)(features->ch2.rms * 100);
  int ch2_mav = (int)(features->ch2.mav * 100);
  int ch2_mf = (int)(features->ch2.mf * 100);

  int len = sprintf(buffer,
    "{\"ch1\":{\"rms\":%d,\"mav\":%d,\"mf\":%d},\"ch2\":{\"rms\":%d,\"mav\":%d,\"mf\":%d}}\n",
    ch1_rms, ch1_mav, ch1_mf,
    ch2_rms, ch2_mav, ch2_mf);

  return len;
}

/*
 * Chú thích (tiếng Việt):
 * - `format_features_json` chuyển cấu trúc `EMGFeatures_t` thành chuỗi JSON.
 * - Các giá trị float được nhân 100 và ghi dưới dạng integer để giữ 2 chữ số thập phân.
 * - Ví dụ: 512.45 -> 51245
 */

