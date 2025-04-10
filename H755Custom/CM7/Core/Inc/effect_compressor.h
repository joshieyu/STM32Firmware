// effect_compressor.h
#ifndef EFFECT_COMPRESSOR_H
#define EFFECT_COMPRESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "mixer_state.h" // Needs CompressorParameters definition

// State for the compressor effect
typedef struct {
    float sample_rate;
    float envelope_db;      // Current detected level in dB (smoothed)
    float attack_coeff;     // Smoothing coefficient for attack phase
    float release_coeff;    // Smoothing coefficient for release phase

    // Store last params to detect changes for coefficient recalculation
    CompressorParameters last_params;
    bool coeffs_dirty;

} CompressorState;

// --- Public Functions ---
void Compressor_Init(CompressorState *state, float sample_rate);

/**
 * @brief Processes a block of audio samples through the compressor.
 *        Modifies the buffer in-place.
 * @param state Pointer to the initialized CompressorState structure.
 * @param buffer Pointer to the audio buffer (float, nominally +/- 1.0). Modified in-place.
 * @param num_samples Number of samples in the buffer.
 * @param params Pointer to the CompressorParameters struct containing current settings.
 */
void Compressor_Process(CompressorState *state, float* buffer, uint32_t num_samples, const CompressorParameters *params);

// Optional: Function to force coefficient update if params are changed externally
// void Compressor_SetCoeffsDirty(CompressorState *state);

#endif // EFFECT_COMPRESSOR_H