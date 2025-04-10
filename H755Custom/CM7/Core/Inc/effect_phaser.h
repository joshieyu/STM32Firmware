// effect_phaser.h
#ifndef EFFECT_PHASER_H
#define EFFECT_PHASER_H

#include <stdint.h>
#include <stdbool.h>
#include "mixer_state.h" // Needs PhaserParameters definition

#define PHASER_MAX_STAGES 6 // Number of all-pass filter stages (adjust as needed)

// State for ONE 1st-order All-pass filter (Direct Form II Transposed)
typedef struct {
    float z1; // State variable (delay element)
} AllpassState;

// State for the Phaser effect
typedef struct {
    float sample_rate;
    float lfo_phase;     // Current phase of the Low-Frequency Oscillator (0 to 2*PI)
    AllpassState stages[PHASER_MAX_STAGES]; // State for each filter stage
    // No need to store last_params/dirty flag if we read params every block
} PhaserState;

// Public Functions
void Phaser_Init(PhaserState *state, float sample_rate);

/**
 * @brief Processes a block of audio samples through the phaser effect.
 *        Modifies the buffer in-place. Uses parameters directly.
 * @param state Pointer to the initialized PhaserState structure.
 * @param buffer Pointer to the audio buffer (float, nominally +/- 1.0). Will be modified.
 * @param num_samples Number of samples in the buffer.
 * @param params Pointer to the PhaserParameters struct containing current settings.
 */
void Phaser_Process(PhaserState *state, float* buffer, uint32_t num_samples, const PhaserParameters *params);

#endif // EFFECT_PHASER_H