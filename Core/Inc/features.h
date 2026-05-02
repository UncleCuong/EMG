/**
  ******************************************************************************
  * @file           : features.h
  * @brief          : EMG feature extraction header
  ******************************************************************************
  */

#ifndef __FEATURES_H
#define __FEATURES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Channel features structure (RMS, MAV, MF for one channel)
 */
typedef struct {
  float rms;  // Root Mean Square
  float mav;  // Mean Absolute Value
  float mf;   // Mean Frequency (Hz)
} ChannelFeatures_t;

/**
 * @brief Complete EMG features structure for both channels
 */
typedef struct {
  ChannelFeatures_t ch1;
  ChannelFeatures_t ch2;
} EMGFeatures_t;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Calculate RMS (Root Mean Square) of signal
 * @param data: pointer to uint16_t data array
 * @param length: number of samples
 * @return RMS value as float
 */
float calculate_RMS(uint16_t *data, int length);

/**
 * @brief Calculate MAV (Mean Absolute Value) of signal
 * @param data: pointer to uint16_t data array
 * @param length: number of samples
 * @return MAV value as float
 */
float calculate_MAV(uint16_t *data, int length);

/**
 * @brief Calculate MF (Mean Frequency) using spectral estimation
 * @param data: pointer to uint16_t data array
 * @param length: number of samples
 * @param sampling_freq: sampling frequency in Hz
 * @return Mean Frequency in Hz as float
 */
float calculate_MF(uint16_t *data, int length, uint16_t sampling_freq);

/**
 * @brief Format EMG features to JSON string
 * @param features: pointer to EMGFeatures_t structure
 * @param buffer: output string buffer (should be at least 200 chars)
 * @return length of formatted string
 */
int format_features_json(EMGFeatures_t *features, char *buffer);

#ifdef __cplusplus
}
#endif

#endif /* __FEATURES_H */
