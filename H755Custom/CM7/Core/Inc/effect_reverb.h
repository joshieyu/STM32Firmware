// effect_reverb.h
#ifndef EFFECT_REVERB_H
#define EFFECT_REVERB_H

#include <stdint.h>
#include <stdbool.h>
#include "mixer_state.h" // Needs ReverbParameters definition

// --- Configuration Defines ---
// Adjust these based on desired quality, CPU, and RAM limits

#define REVERB_NUM_COMBS 8      // Number of parallel comb filters
#define REVERB_NUM_ALLPASS 4    // Number of series all-pass filters

// Maximum delay line sizes in SAMPLES. These determine RAM usage!
// Choose based on max expected sample rate and desired max decay time.
// Primes are good, but sizes relative to sample rate matter more for time.
// Example target: ~50-100ms range max delays for individual components at 48kHz
#define REVERB_COMB1_MAX_SAMPLES  (2400) // ~50ms @ 48kHz
#define REVERB_COMB2_MAX_SAMPLES  (2544) // ~53ms
#define REVERB_COMB3_MAX_SAMPLES  (2736) // ~57ms
#define REVERB_COMB4_MAX_SAMPLES  (2880) // ~60ms
#define REVERB_COMB5_MAX_SAMPLES  (3072) // ~64ms
#define REVERB_COMB6_MAX_SAMPLES  (3216) // ~67ms
#define REVERB_COMB7_MAX_SAMPLES  (3360) // ~70ms
#define REVERB_COMB8_MAX_SAMPLES  (3504) // ~73ms
#define REVERB_ALLPASS1_MAX_SAMPLES (240) // ~5ms
#define REVERB_ALLPASS2_MAX_SAMPLES (336) // ~7ms
#define REVERB_ALLPASS3_MAX_SAMPLES (816) // ~17ms
#define REVERB_ALLPASS4_MAX_SAMPLES (1008)// ~21ms


// --- State Structures ---

// State for one Comb filter (with low-pass feedback damping)
typedef struct {
    float buffer[REVERB_COMB1_MAX_SAMPLES]; // ** Allocate largest needed size **
    uint32_t buffer_size; // Actual delay length used
    uint32_t index;       // Current write/read index
    float feedback;    // Calculated feedback gain (0 to <1)
    float damp_coeff;  // Damping LPF coefficient (0 to <1)
    float damp_z1;     // LPF state variable
} CombState;

// State for one All-pass filter (1st order)
typedef struct {
    float buffer[REVERB_ALLPASS1_MAX_SAMPLES]; // ** Allocate largest needed size **
    uint32_t buffer_size; // Actual delay length used
    uint32_t index;       // Current write/read index
    float feedback;    // Fixed feedback/coefficient (e.g., 0.5f)
    // No filter state needed for simple 1st order allpass with this structure
} AllpassState_Reverb; // Renamed to avoid conflict with Phaser if needed

// Overall Reverb State
typedef struct {
    float sample_rate;
    CombState combs[REVERB_NUM_COMBS];
    AllpassState_Reverb allpasses[REVERB_NUM_ALLPASS];

    // Store calculated parameters based on shared struct
    float calculated_wet_level;
    float calculated_dry_level;
    bool params_dirty; // Flag to recalculate internal gains/coeffs
    ReverbParameters last_params; // To detect changes

} ReverbState;

// --- Public Functions ---
void Reverb_Init(ReverbState *state, float sample_rate);

/**
 * @brief Processes a block of stereo audio through the reverb effect.
 *        Reads L/R, processes mono sum, outputs mono wet to both L/R.
 * @param state Pointer to the initialized ReverbState structure.
 * @param buffer_l Pointer to the LEFT audio buffer (float, nominally +/- 1.0). Modified in-place.
 * @param buffer_r Pointer to the RIGHT audio buffer (float, nominally +/- 1.0). Modified in-place.
 * @param num_samples Number of samples in each buffer.
 * @param params Pointer to the ReverbParameters struct containing current settings.
 */
void Reverb_ProcessStereo(ReverbState *state, float* buffer_l, float* buffer_r, uint32_t num_samples, const ReverbParameters *params);

#endif // EFFECT_REVERB_H