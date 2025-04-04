#include "effect_gain.h"
#include <math.h> // For powf in db_to_linear

#define MAX_AMPLITUDE_24BIT (8388607)
#define MIN_AMPLITUDE_24BIT (-8388608)

// Clip helper (could be moved to a shared utility header)
static inline int32_t ClipSample24Bit(int64_t sample) {
    if (sample > MAX_AMPLITUDE_24BIT) return MAX_AMPLITUDE_24BIT;
    if (sample < MIN_AMPLITUDE_24BIT) return MIN_AMPLITUDE_24BIT;
    return (int32_t)sample;
}

void EffectGain_Init(void) {
    // Initialize gain state if needed (e.g., smoothing parameters)
}

void EffectGain_Process(int32_t* buffer, uint32_t num_samples, float linear_gain) {
    for (uint32_t i = 0; i < num_samples; ++i) {
        // Operate on the 24-bit value
        int32_t sample24 = buffer[i] >> 8;
        int64_t processed_sample = (int64_t)((float)sample24 * linear_gain);

        // Write back clipped and left-shifted
        buffer[i] = ClipSample24Bit(processed_sample) << 8;
    }
}

float db_to_linear(float db) {
    return powf(10.0f, db / 20.0f);
}