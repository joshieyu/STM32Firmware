#ifndef INC_EFFECT_GAIN_H_
#define INC_EFFECT_GAIN_H_

#include <stdint.h>

void EffectGain_Init(void); // Optional: If gain needs state init

/**
 * @brief Applies linear gain to a buffer of samples (in-place).
 * @param buffer Buffer of 32-bit samples (left-aligned audio). Modified in-place.
 * @param num_samples Number of samples in the buffer.
 * @param linear_gain Gain factor (e.g., 1.0 = 0dB, 0.5 = -6dB).
 */
void EffectGain_Process(int32_t* buffer, uint32_t num_samples, float linear_gain);

// You might add a stereo version if master bus processing is different
// void EffectGain_ProcessStereo(int32_t* buffer_l, int32_t* buffer_r, uint32_t num_samples, float linear_gain);

// Helper to convert dB to linear gain
float db_to_linear(float db);

#endif /* INC_EFFECT_GAIN_H_ */