// effect_reverb.c
#include "effect_reverb.h"
#include <math.h>
#include <string.h> // For memset, memcmp
#include <stdio.h>  // For printf debugging
#include <stdlib.h> // For abs // Consider using fabsf from math.h instead for float

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif
#define TWO_PI_F (2.0f * PI_F)

// --- Fixed Internal Parameters (Tune these for desired sound) ---

// Delay lengths in SAMPLES for each filter component AT a reference sample rate (e.g., 44.1kHz)
// These will be scaled based on the actual sample rate in Init.
// Using prime-ish numbers helps diffusion. Ensure these are <= MAX defines in .h!
const uint32_t comb_delays_ref[REVERB_NUM_COMBS] = {
    1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617
};
const uint32_t allpass_delays_ref[REVERB_NUM_ALLPASS] = {
    225, 556, 441, 341 // Different order/values from Freeverb common examples
};

// Fixed All-pass feedback coefficient (common value)
const float allpass_feedback = 0.5f;

// Fixed Comb filter damping factor (controls HF decay, 0=no damp, <1=more damp)
// This translates to the LPF coefficient within the feedback loop.
const float default_comb_damping = 0.25f; // Higher value = more damping = darker reverb
const float comb_damping_hf_limit_hz = 5000.0f; // Max freq affected by damping LPF

// Room size scaling factor for feedback (subtle adjustment based on a size param if added)
const float room_size_scale = 0.28f;
const float room_size_offset = 0.7f;

// Fixed stereo spread (if stereo output was implemented differently)
// const float stereo_spread = 0.3f;

// --- Private Helper Functions ---

// Clamp helper
static inline float Clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// Function to update internal gains/coeffs based on parameters
static void Reverb_UpdateParams(ReverbState *state, const ReverbParameters *params) {
    if (!state || !params) return;

    printf("DEBUG: Reverb Updating Parameters...\n");

    // --- Wet/Dry ---
    float wet = Clamp(params->wet_level, 0.0f, 1.0f); // % to 0-1
    state->calculated_wet_level = wet;
    // Could use square root scaling for constant power, or simple linear:
    state->calculated_dry_level = 1.0f - wet;

    // --- Decay Time -> Comb Feedback & Damping ---
    // Map decay_time (0.3 to 3s) to feedback gain
    // This is a simplified mapping; real decay involves damping too.
    // We use an overall 'room scale' concept influencing feedback.
    float room_scale_factor = room_size_scale; // Could be adjusted by a room size param later
    float feedback_base = room_size_offset;
    // Longer decay = higher feedback gain towards 1.0
    float decay_time_s = Clamp(params->decay_time, 0.1f, 8.0f); // Allow slightly wider range internally
    // Simple linear map for feedback base adjustment (needs tuning)
    float feedback_adjust = (decay_time_s / 8.0f) * 0.3f; // Longer time = closer to 1.0

    // Calculate damping LPF coeff based on fixed freq & sample rate
    // coeff = exp(-2*pi*Fc/Fs) for 1-pole LPF
    float damp_lp_coeff = expf(-TWO_PI_F * comb_damping_hf_limit_hz / state->sample_rate);
    // Adjust damping amount based on decay time? (Longer decay = less damping?) - Simple version uses fixed default_comb_damping
    float damp_amount = default_comb_damping;


    for (int i = 0; i < REVERB_NUM_COMBS; ++i) {
        // Calculate feedback gain based on decay time concept
        // This is highly empirical - needs heavy tuning!
        // Trying a simpler approach: decay affects feedback directly
        // RT60 approx -60dB. If gain is 'g', after N reflections (N = delay*Fs)
        // g^N = 10^(-60/20) = 0.001
        // N * log(g) = log(0.001)
        // delay * Fs * log(g) = log(0.001)
        // log(g) = log(0.001) / (delay * Fs)
        // g = exp(log(0.001) / (delay * Fs)) --- This assumes RT60 = delay time! Not quite right.

        // Let's use a more direct empirical mapping for feedback
        // Base feedback + adjustment based on decay time
        float target_feedback = feedback_base + feedback_adjust * (1.0f - feedback_base);
        state->combs[i].feedback = Clamp(target_feedback, 0.0f, 0.995f); // Clamp below 1.0

        // Set damping coefficient (how much HF is cut in feedback)
        state->combs[i].damp_coeff = damp_amount * damp_lp_coeff + (1.0f - damp_amount); // Mix between filtered and direct feedback
    }

    // Allpass feedback is fixed
    for (int i = 0; i < REVERB_NUM_ALLPASS; ++i) {
        state->allpasses[i].feedback = allpass_feedback;
    }

    // Store params and clear flag
    memcpy(&state->last_params, params, sizeof(ReverbParameters));
    state->params_dirty = false;
}


// --- Public Function Implementations ---

void Reverb_Init(ReverbState *state, float sample_rate) {
    if (!state) return;
    memset(state, 0, sizeof(ReverbState));
    state->sample_rate = sample_rate;
    state->params_dirty = true;
    memset(&state->last_params, 0xFF, sizeof(ReverbParameters)); // Force first update

    printf("Reverb_Init: Sample Rate = %.1f Hz\n", sample_rate);

    // --- Initialize Comb Filters ---
    // Scale delay times based on sample rate
    float scale = sample_rate / 44100.0f; // Scale relative to reference rate
    uint32_t max_sizes_comb[REVERB_NUM_COMBS] = {
        REVERB_COMB1_MAX_SAMPLES, REVERB_COMB2_MAX_SAMPLES, REVERB_COMB3_MAX_SAMPLES, REVERB_COMB4_MAX_SAMPLES,
        REVERB_COMB5_MAX_SAMPLES, REVERB_COMB6_MAX_SAMPLES, REVERB_COMB7_MAX_SAMPLES, REVERB_COMB8_MAX_SAMPLES
    };
    for (int i = 0; i < REVERB_NUM_COMBS; ++i) {
        state->combs[i].buffer_size = (uint32_t)(comb_delays_ref[i] * scale);
        // Clamp size to allocated buffer max
        if (state->combs[i].buffer_size >= max_sizes_comb[i]) {
            state->combs[i].buffer_size = max_sizes_comb[i] - 1;
        }
        if (state->combs[i].buffer_size < 5) state->combs[i].buffer_size = 5; // Minimum delay
        state->combs[i].index = 0;
        state->combs[i].damp_z1 = 0.0f;
        state->combs[i].feedback = 0.0f; // Will be set by UpdateParams
        memset(state->combs[i].buffer, 0, sizeof(state->combs[i].buffer)); // Clear specific buffer
         printf("  Comb %d: Delay = %lu samples\n", i, state->combs[i].buffer_size);
    }

    // --- Initialize All-Pass Filters ---
    uint32_t max_sizes_ap[REVERB_NUM_ALLPASS] = {
        REVERB_ALLPASS1_MAX_SAMPLES, REVERB_ALLPASS2_MAX_SAMPLES,
        REVERB_ALLPASS3_MAX_SAMPLES, REVERB_ALLPASS4_MAX_SAMPLES
    };
     for (int i = 0; i < REVERB_NUM_ALLPASS; ++i) {
        state->allpasses[i].buffer_size = (uint32_t)(allpass_delays_ref[i] * scale);
        if (state->allpasses[i].buffer_size >= max_sizes_ap[i]) {
            state->allpasses[i].buffer_size = max_sizes_ap[i] - 1;
        }
         if (state->allpasses[i].buffer_size < 5) state->allpasses[i].buffer_size = 5;
        state->allpasses[i].index = 0;
        state->allpasses[i].feedback = allpass_feedback; // Fixed value
        memset(state->allpasses[i].buffer, 0, sizeof(state->allpasses[i].buffer));
        printf("  Allpass %d: Delay = %lu samples\n", i, state->allpasses[i].buffer_size);
    }
}


void Reverb_ProcessStereo(ReverbState *state, float* buffer_l, float* buffer_r, uint32_t num_samples, const ReverbParameters *params) {
    if (!state || !buffer_l || !buffer_r || !params || !params->enabled) {
        // printf("Reverb_Process: Bypass or invalid state\n");
        return; // Bypass
    }

    // printf("Reverb_Process: Processing %u samples\n", num_samples);
    // Check if parameters changed
    if (state->params_dirty || memcmp(&state->last_params, params, sizeof(ReverbParameters)) != 0) {
        Reverb_UpdateParams(state, params);
    }

    // Get wet/dry levels
    const float wet_level = state->calculated_wet_level;
    const float dry_level = state->calculated_dry_level;

    for (uint32_t i = 0; i < num_samples; ++i) {
        float dry_l = buffer_l[i];
        float dry_r = buffer_r[i];

        // Create mono input for reverb network (simple average)
        float input_mono = (dry_l + dry_r) * 0.5f;

        float comb_output_sum = 0.0f;

        // --- Process Parallel Comb Filters ---
        for (int c = 0; c < REVERB_NUM_COMBS; ++c) {
            CombState* comb = &state->combs[c];

            // Read output from delay line
            float delayed_sample = comb->buffer[comb->index];

            // Apply damping (1-pole LPF on feedback signal)
            // y[n] = y[n-1] + C * (x[n] - y[n-1]) where C = (1-coeff) from Reverb_UpdateParams
            // Simplified: y[n] = (1-damp_coeff)*delayed + damp_coeff*damp_z1
            comb->damp_z1 = delayed_sample * (1.0f - comb->damp_coeff) + comb->damp_z1 * comb->damp_coeff;

            // Calculate sample to write back (input + damped feedback)
            float sample_to_write = input_mono + comb->damp_z1 * comb->feedback;

            // Write to delay line (no clipping here, rely on overall output clip)
            comb->buffer[comb->index] = sample_to_write;

            // Add this comb's direct output (the delayed sample) to the sum
            comb_output_sum += delayed_sample;

            // Increment index
            comb->index++;
            if (comb->index >= comb->buffer_size) {
                comb->index = 0;
            }
        }

        // --- Process Series All-Pass Filters ---
        float allpass_input = comb_output_sum; // Output of combs feeds first allpass

        for (int a = 0; a < REVERB_NUM_ALLPASS; ++a) {
             AllpassState_Reverb* ap = &state->allpasses[a];

             // Read delayed sample
             float delayed_sample = ap->buffer[ap->index];

             // All-pass output: y[n] = -g*x[n] + x[n-M] + g*y[n-M] (where g=feedback)
             float allpass_output = -ap->feedback * allpass_input + delayed_sample; // Part 1: -g*x + delayed

             // Write to buffer: x[n] + g*y[n] (using the just calculated output)
             ap->buffer[ap->index] = allpass_input + ap->feedback * allpass_output;

             // Increment index
            ap->index++;
            if (ap->index >= ap->buffer_size) {
                ap->index = 0;
            }

             // Output of this stage is input to next
             allpass_input = allpass_output;
        }

        // --- Mix Output ---
        // allpass_input now holds the final mono wet signal
        float wet_mono = allpass_input;

        // Simple mono reverb: apply same wet signal to L/R
        buffer_l[i] = dry_l * dry_level + wet_mono * wet_level;
        buffer_r[i] = dry_r * dry_level + wet_mono * wet_level;

        // --- Optional Final Clipping (if needed, depends on gain staging) ---
        // buffer_l[i] = Clamp(buffer_l[i], -1.0f, 1.0f);
        // buffer_r[i] = Clamp(buffer_r[i], -1.0f, 1.0f);
    }
}