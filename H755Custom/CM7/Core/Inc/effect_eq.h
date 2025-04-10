#ifndef EFFECT_EQ_H
#define EFFECT_EQ_H

#include <stdint.h>
#include <stdbool.h>
#include "mixer_state.h" // Needs EqualizerParameters definition

#define EQ_MAX_BANDS 6 // lowShelf, highShelf, band0..3

// State for ONE Biquad filter section
typedef struct {
    float b0, b1, b2, a1, a2; // Coefficients
    float x1, x2, y1, y2;     // State variables (previous inputs/outputs)
} BiquadState;

// State for the multi-band EQ effect
typedef struct {
    float sample_rate;
    BiquadState band_states[EQ_MAX_BANDS];
    EqualizerParameters last_params; // To detect parameter changes
    bool coeffs_dirty;               // Flag to recalculate coeffs
} EQState;

// Public Functions
void EQ_Init(EQState *state, float sample_rate);
void EQ_Process(EQState *state, float* buffer, uint32_t num_samples, const EqualizerParameters *params);
// Optional: Function to force coefficient update if params are changed externally
// void EQ_SetCoeffsDirty(EQState *state);

#endif // EFFECT_EQ_H