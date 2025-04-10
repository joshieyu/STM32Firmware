// effect_compressor.c
#include "effect_compressor.h"
#include <math.h>
#include <string.h> // For memcmp, memcpy
#include <stdio.h>  // For debugging prints
#include <float.h>  // For FLT_MIN for log safety

#define TINY_FLOAT_LOG FLT_MIN // Use smallest positive float for log safety instead of 1e-9f

// --- Private Helper Functions ---

// Clamp helper
static inline float Clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// dB to Linear conversion (consistent with others)
static inline float DB_to_Linear(float db) {
    // Handle silence explicitly
    if (db <= -100.0f) return 0.0f; // Or very small value like 1e-5f
    return powf(10.0f, db / 20.0f);
}

// Update attack/release coefficients based on parameters
static void Compressor_UpdateCoefficients(CompressorState *state, const CompressorParameters *params) {
    if (!state || !params) return;

    printf("DEBUG: Compressor Updating Coefficients...\n");

    // Calculate time constants in seconds, ensuring minimum positive values
    float attack_s = fmaxf(0.0001f, params->attack_ms * 0.001f); // Min 0.1ms
    float release_s = fmaxf(0.001f, params->release_ms * 0.001f); // Min 1ms

    // Calculate coefficients using standard exponential decay formula
    // coeff = exp(-1 / (time_seconds * sample_rate))
    // (Coefficient represents how much of the previous value remains after one sample)
    state->attack_coeff = expf(-1.0f / (attack_s * state->sample_rate));
    state->release_coeff = expf(-1.0f / (release_s * state->sample_rate));

    // Store current params and clear dirty flag
    memcpy(&state->last_params, params, sizeof(CompressorParameters));
    state->coeffs_dirty = false;
}

// --- Public Function Implementations ---

void Compressor_Init(CompressorState *state, float sample_rate) {
    if (!state) return;
    memset(state, 0, sizeof(CompressorState));
    state->sample_rate = sample_rate;
    state->envelope_db = -120.0f; // Initialize envelope very low (approx silence)
    state->coeffs_dirty = true;   // Force coefficient calculation on first Process call
    // Ensure memcmp fails first time
    memset(&state->last_params, 0xFF, sizeof(CompressorParameters));
    state->last_params.threshold_db = 999.0f; // Guarantee difference
}


void Compressor_Process(CompressorState *state, float* buffer, uint32_t num_samples, const CompressorParameters *params) {
    if (!state || !buffer || !params || !params->enabled) {
        printf("Compressor_Process: Bypass or invalid state\n");
        return; // Bypass
    }

    // printf("DEBUG: Compressor Processing %u samples\n", num_samples);

    // --- Update Coefficients if Parameters Changed ---
    // Use memcmp for efficient check (assumes no padding issues in struct)
    if (state->coeffs_dirty || memcmp(&state->last_params, params, sizeof(CompressorParameters)) != 0) {
        Compressor_UpdateCoefficients(state, params);
    }

    // --- Get Parameters (Local copies for loop efficiency) ---
    const float threshold_db = params->threshold_db;
    const float knee_db = fmaxf(0.0f, params->knee_db); // Ensure non-negative knee
    const float ratio = fmaxf(1.0f, params->ratio);     // Ensure ratio >= 1:1
    const float attack_coeff = state->attack_coeff;
    const float release_coeff = state->release_coeff;
    const float makeup_gain_db = params->makeup_gain_db;
    // Pre-calculate for gain computation
    const float slope = 1.0f - (1.0f / ratio); // Compression slope above threshold (0 to 1)
    const float knee_lower_bound_db = threshold_db - knee_db * 0.5f;
    const float knee_upper_bound_db = threshold_db + knee_db * 0.5f;

    // Get envelope state (local copy for modification within loop)
    float envelope_db = state->envelope_db;

    // --- Process Samples ---
    for (uint32_t i = 0; i < num_samples; ++i) {
        float input_sample = buffer[i]; // Keep original sample for gain application later

        // printf("input_sample: %f\n", input_sample);

        // --- 1. Level Detection (Peak dB) ---
        // Use fabsf for absolute value, add tiny float to avoid log10(0)
        float input_level_db = 20.0f * log10f(fabsf(input_sample) + TINY_FLOAT_LOG);

        // --- 2. Envelope Follower ---
        float coeff;
        if (input_level_db > envelope_db) { // Signal increasing or above envelope -> Attack
            coeff = attack_coeff;
        } else { // Signal decreasing or below envelope -> Release
            coeff = release_coeff;
        }
        // Apply smoothing: envelope approaches input_level_db exponentially
        // envelope = target + coeff * (current - target)
        envelope_db = input_level_db + coeff * (envelope_db - input_level_db);
        // Clamp envelope just in case to prevent extreme values if coeffs are near 1
        envelope_db = Clamp(envelope_db, -120.0f, 20.0f);


        // --- 3. Gain Computation ---
        float gain_reduction_db = 0.0f; // Default: No gain reduction

        if (knee_db > 0.0f && envelope_db > knee_lower_bound_db && envelope_db < knee_upper_bound_db) {
            // --- Soft Knee Region ---
            // Calculate how far into the knee the envelope is (0 to 1)
            float knee_pos = (envelope_db - knee_lower_bound_db) / knee_db;
            // The slope increases linearly across the knee from 0 to the full 'slope'
            float slope_at_knee = knee_pos * slope;
            // Calculate gain reduction based on quadratic curve within the knee
            // Reduction = slope_at_point * (overshoot_from_knee_start)
            // Simplified quadratic approximation: reduction = slope * (overshoot^2) / (2 * knee)
            // Let's use a simpler linear interpolation of gain reduction across the knee:
            // Calculate reduction as if hard knee at upper bound: slope * (env - thresh)
            // Calculate reduction as if hard knee at lower bound: 0
            // Overshoot relative to threshold at upper knee: knee_db / 2
            // Full reduction at upper knee: slope * (knee_db / 2)
            // Linearly interpolate this reduction based on knee_pos
            gain_reduction_db = knee_pos * slope * (envelope_db - knee_lower_bound_db); // Simpler linear interpolation


        } else if (envelope_db >= knee_upper_bound_db) { // Also handles hard knee case (knee_db = 0)
             // --- Above Knee or Hard Knee ---
             float overshoot = envelope_db - threshold_db;
             gain_reduction_db = slope * overshoot;
        }
        // gain_reduction_db will be <= 0.0f


        // --- 4. Apply Gain ---
        // Calculate total gain change in dB
        float target_gain_db = gain_reduction_db + makeup_gain_db;

        // Convert total dB gain to linear scale
        float target_gain_linear = DB_to_Linear(target_gain_db);

        // Apply gain to the original input sample
        buffer[i] = input_sample * target_gain_linear;

        // printf("output_sample: %f\n", buffer[i]);

        // --- Optional: Final output clipping (usually done after master gain anyway) ---
        // buffer[i] = Clamp(buffer[i], -1.0f, 1.0f);

    } // End loop over samples

    // --- Store final envelope value back to state ---
    state->envelope_db = envelope_db;
}


// Optional: Function to force coefficient update if params are changed externally
// void Compressor_SetCoeffsDirty(CompressorState *state) {
//     if(state) state->coeffs_dirty = true;
// }