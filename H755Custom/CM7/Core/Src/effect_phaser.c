// effect_phaser.c (Revised with Fc Modulation)
#include "effect_phaser.h"
#include <math.h>
#include <string.h> // For memset
#include <stdio.h>  // For potential debugging prints

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif
#define TWO_PI_F (2.0f * PI_F)

// --- Private Helper Functions ---

// Clamp helper
static inline float Clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// --- Public Function Implementations ---

void Phaser_Init(PhaserState *state, float sample_rate) {
    if (!state) return;
    memset(state, 0, sizeof(PhaserState)); // Clear all states
    state->sample_rate = sample_rate;
    state->lfo_phase = 0.0f;
}


void Phaser_Process(PhaserState *state, float* buffer, uint32_t num_samples, const PhaserParameters *params) {
    if (!state || !buffer || !params || !params->enabled) {
        return; // Bypass
    }

    // --- Get Parameters ---
    float rate_hz = Clamp(params->rate, 0.1f, 10.0f);
    float depth_0_to_1 = Clamp(params->depth, 0.0f, 1.0f); // Convert % to 0-1
    const float mix = 0.5f; // Still assuming 50/50 mix

    // --- LFO ---
    float lfo_phase = state->lfo_phase;
    const float lfo_inc = TWO_PI_F * rate_hz / state->sample_rate;

    // --- All-pass Modulation Parameters ---
    // Define the FREQUENCY range the LFO will sweep
    const float min_center_freq_hz = 250.0f;  // Example: Minimum frequency for notches
    const float max_center_freq_hz = 3000.0f; // Example: Maximum frequency for notches
    const float sweep_range_hz = max_center_freq_hz - min_center_freq_hz;
    const float sweep_center_hz = (max_center_freq_hz + min_center_freq_hz) * 0.5f;
    const float sweep_amount_hz = sweep_range_hz * 0.5f * depth_0_to_1; // Control range with depth

    // Debug counter (optional)
    // static uint32_t debug_counter = 0;


    for (uint32_t i = 0; i < num_samples; ++i) {
        // --- Calculate LFO and Modulated Center Frequency Fc ---
        float lfo_val = cosf(lfo_phase); // LFO output (-1 to +1)
        float modulated_fc_hz = sweep_center_hz + lfo_val * sweep_amount_hz;
        // Ensure Fc stays within valid Nyquist limits (Fs/2) and above 0
        modulated_fc_hz = Clamp(modulated_fc_hz, 20.0f, state->sample_rate * 0.49f);


        // --- Calculate All-pass Coefficient 'a1' from Modulated Fc ---
        // Use the relationship: a1 = (tan(omega/2) - 1) / (tan(omega/2) + 1)
        // where omega = 2*pi*Fc/Fs
        float omega = TWO_PI_F * modulated_fc_hz / state->sample_rate;
        float tan_omega_half = tanf(omega * 0.5f);

        // Check for potential invalid tan result (near pi/2 for omega/2)
        if (isnan(tan_omega_half) || isinf(tan_omega_half) || fabsf(tan_omega_half + 1.0f) < 1e-7f) {
             // If tan is invalid or denominator is near zero, use a safe coefficient (e.g., 0)
             // printf("WARN: Phaser tan calculation issue! Fc=%.1f\n", modulated_fc_hz);
             tan_omega_half = 0.0f; // Results in a1 = -1, maybe not ideal, perhaps 0.5? Let's try 0.0 for a1= -1
        }

        float a1 = (tan_omega_half - 1.0f) / (tan_omega_half + 1.0f);
        // Coefficient 'a1' MUST be strictly between -1 and 1 for stability
        a1 = Clamp(a1, -0.9999f, 0.9999f);


        // --- Update LFO Phase ---
        lfo_phase += lfo_inc;
        if (lfo_phase >= TWO_PI_F) {
            lfo_phase -= TWO_PI_F;
        }

        // --- Optional Debug Print ---
        /*
        if (++debug_counter > 44100) {
             printf("DBG: Fc=%.1f Hz, tan=%.3f, a1=%.4f\n", modulated_fc_hz, tan_omega_half, a1);
             debug_counter = 0;
        }
        */

        // --- Process All-Pass Stages ---
        float dry_sample = buffer[i];
        float stage_input = dry_sample;

        // Apply the filter stages using Direct Form II Transposed
        // y[n] = a1*x[n] + z1[n-1]
        // z1[n] = x[n] - a1*y[n]
        for (int stage = 0; stage < PHASER_MAX_STAGES; ++stage) {
            AllpassState* aps = &state->stages[stage];
            if (isnan(aps->z1) || isinf(aps->z1)) aps->z1 = 0.0f; // Check state

            float stage_output = a1 * stage_input + aps->z1;

            if (isnan(stage_output) || isinf(stage_output)) {
                stage_output = 0.0f;
                aps->z1 = 0.0f;
            } else {
                aps->z1 = stage_input - a1 * stage_output;
            }
            stage_input = stage_output; // Output becomes input for next stage
        }
        // stage_input now holds the final wet signal

        // --- Mix ---
        buffer[i] = (dry_sample + stage_input) * mix; // 50/50 Mix

        // --- OR Output only WET signal for testing ---
        //  buffer[i] = stage_input;

    } // End loop over samples

    // --- Store updated LFO phase ---
    state->lfo_phase = lfo_phase;
}