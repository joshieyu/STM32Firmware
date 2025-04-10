#include "effect_eq.h"
#include <math.h>
#include <string.h> // for memcmp/memcpy
#include <stdio.h>  // For printf debugging
#include <float.h>  // For FLT_EPSILON

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

// --- Biquad Coefficient Calculation Helpers (Based on Audio EQ Cookbook) ---

// Forward declaration for NaN check function
static bool check_coeffs_valid(const BiquadState* target, int band_index);

// --- Peaking EQ ---
static bool calc_peak_coeffs(float Fs, float Fc, float Q, float gain_db, BiquadState* target, int band_index) {
    if (Fc <= 0.0f || Fc >= Fs * 0.5f || Q <= 0.0f) {
        // Invalid parameters - Set to bypass (flat response)
        printf("WARN: EQ Band %d - Invalid Peak params (Fs=%.1f, Fc=%.1f, Q=%.1f). Setting flat.\n", band_index, Fs, Fc, Q);
        target->b0 = 1.0f; target->b1 = 0.0f; target->b2 = 0.0f;
        target->a1 = 0.0f; target->a2 = 0.0f;
        return false; // Indicate parameters were invalid
    }

    float A = powf(10.0f, gain_db / 40.0f); // Linear gain derived from dB/2
    float omega = 2.0f * PI_F * Fc / Fs;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * Q);

    float b0 = 1.0f + alpha * A;
    float b1 = -2.0f * cs;
    float b2 = 1.0f - alpha * A;
    float a0 = 1.0f + alpha / A;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha / A;

    // Normalize coefficients by a0
    // Check for a0 being close to zero
    if (fabsf(a0) < FLT_EPSILON) {
        printf("ERROR: EQ Band %d - Peak filter a0 is near zero. Setting flat.\n", band_index);
        target->b0 = 1.0f; target->b1 = 0.0f; target->b2 = 0.0f;
        target->a1 = 0.0f; target->a2 = 0.0f;
        return false;
    }
    float a0_inv = 1.0f / a0;

    target->b0 = b0 * a0_inv;
    target->b1 = b1 * a0_inv;
    target->b2 = b2 * a0_inv;
    // Store a1 and a2 negated for DFII-T difference equation: y = b0*x + s1; s1 = b1*x - a1*y + s2; s2 = b2*x - a2*y
    target->a1 = -(a1 * a0_inv);
    target->a2 = -(a2 * a0_inv);

    return check_coeffs_valid(target, band_index);
}

// --- Low Shelf Filter ---
static bool calc_low_shelf_coeffs(float Fs, float Fc, float Q, float gain_db, BiquadState* target, int band_index) {
     if (Fc <= 0.0f || Fc >= Fs * 0.5f || Q <= 0.0f) {
        printf("WARN: EQ Band %d - Invalid LShelf params (Fs=%.1f, Fc=%.1f, Q=%.1f). Setting flat.\n", band_index, Fs, Fc, Q);
        target->b0 = 1.0f; target->b1 = 0.0f; target->b2 = 0.0f;
        target->a1 = 0.0f; target->a2 = 0.0f;
        return false;
    }

    float A = powf(10.0f, gain_db / 40.0f);
    float omega = 2.0f * PI_F * Fc / Fs;
    float sn = sinf(omega);
    float cs = cosf(omega);
    // Use sqrt(Q) related term for shelf slope control if desired, or keep alpha simple based on Q=1/sqrt(2) typically
    // float alpha = sn / (2.0f * Q); // Simple alpha based on Q param
    // Alternative shelf calculation: S=1 assumed here for simplicity (fixed slope)
    float alpha = sn / 2.0f * sqrtf( (A + 1.0f/A)*(1.0f/1.0f - 1.0f) + 2.0f ); // S=1.0 slope approx

    float two_sqrtA_alpha = 2.0f * sqrtf(A) * alpha;

    float b0 =    A*( (A+1.0f) - (A-1.0f)*cs + two_sqrtA_alpha );
    float b1 =  2.0f*A*( (A-1.0f) - (A+1.0f)*cs                   );
    float b2 =    A*( (A+1.0f) - (A-1.0f)*cs - two_sqrtA_alpha );
    float a0 =        (A+1.0f) + (A-1.0f)*cs + two_sqrtA_alpha;
    float a1 = -2.0f*( (A-1.0f) + (A+1.0f)*cs                   );
    float a2 =        (A+1.0f) + (A-1.0f)*cs - two_sqrtA_alpha;

    if (fabsf(a0) < FLT_EPSILON) {
        printf("ERROR: EQ Band %d - LShelf filter a0 is near zero. Setting flat.\n", band_index);
        target->b0 = 1.0f; target->b1 = 0.0f; target->b2 = 0.0f;
        target->a1 = 0.0f; target->a2 = 0.0f;
        return false;
    }
    float a0_inv = 1.0f / a0;

    target->b0 = b0 * a0_inv;
    target->b1 = b1 * a0_inv;
    target->b2 = b2 * a0_inv;
    target->a1 = -(a1 * a0_inv);
    target->a2 = -(a2 * a0_inv);

    return check_coeffs_valid(target, band_index);
}

// --- High Shelf Filter ---
static bool calc_high_shelf_coeffs(float Fs, float Fc, float Q, float gain_db, BiquadState* target, int band_index) {
    if (Fc <= 0.0f || Fc >= Fs * 0.5f || Q <= 0.0f) {
        printf("WARN: EQ Band %d - Invalid HShelf params (Fs=%.1f, Fc=%.1f, Q=%.1f). Setting flat.\n", band_index, Fs, Fc, Q);
        target->b0 = 1.0f; target->b1 = 0.0f; target->b2 = 0.0f;
        target->a1 = 0.0f; target->a2 = 0.0f;
        return false;
    }

    float A = powf(10.0f, gain_db / 40.0f);
    float omega = 2.0f * PI_F * Fc / Fs;
    float sn = sinf(omega);
    float cs = cosf(omega);
    // float alpha = sn / (2.0f * Q); // Simple alpha
    // Alternative shelf calculation: S=1 assumed here
    float alpha = sn / 2.0f * sqrtf( (A + 1.0f/A)*(1.0f/1.0f - 1.0f) + 2.0f );

    float two_sqrtA_alpha = 2.0f * sqrtf(A) * alpha;

    float b0 =    A*( (A+1.0f) + (A-1.0f)*cs + two_sqrtA_alpha );
    float b1 = -2.0f*A*( (A-1.0f) + (A+1.0f)*cs                   );
    float b2 =    A*( (A+1.0f) + (A-1.0f)*cs - two_sqrtA_alpha );
    float a0 =        (A+1.0f) - (A-1.0f)*cs + two_sqrtA_alpha;
    float a1 =  2.0f*( (A-1.0f) - (A+1.0f)*cs                   );
    float a2 =        (A+1.0f) - (A-1.0f)*cs - two_sqrtA_alpha;

    if (fabsf(a0) < FLT_EPSILON) {
        printf("ERROR: EQ Band %d - HShelf filter a0 is near zero. Setting flat.\n", band_index);
        target->b0 = 1.0f; target->b1 = 0.0f; target->b2 = 0.0f;
        target->a1 = 0.0f; target->a2 = 0.0f;
        return false;
    }
    float a0_inv = 1.0f / a0;

    target->b0 = b0 * a0_inv;
    target->b1 = b1 * a0_inv;
    target->b2 = b2 * a0_inv;
    target->a1 = -(a1 * a0_inv);
    target->a2 = -(a2 * a0_inv);

    return check_coeffs_valid(target, band_index);
}

// --- Check for NaN/Inf in calculated coefficients ---
static bool check_coeffs_valid(const BiquadState* target, int band_index) {
    bool valid = true;
    if (isnan(target->b0) || isinf(target->b0)) valid = false;
    if (isnan(target->b1) || isinf(target->b1)) valid = false;
    if (isnan(target->b2) || isinf(target->b2)) valid = false;
    if (isnan(target->a1) || isinf(target->a1)) valid = false;
    if (isnan(target->a2) || isinf(target->a2)) valid = false;

    if (!valid) {
        printf("ERROR: EQ Band %d - NaN/Inf detected in calculated coefficients! Setting flat.\n", band_index);
        // Cast needed because target is const in this function, but we need a non-const pointer to modify
        BiquadState* mutable_target = (BiquadState*)target;
        mutable_target->b0 = 1.0f; mutable_target->b1 = 0.0f; mutable_target->b2 = 0.0f;
        mutable_target->a1 = 0.0f; mutable_target->a2 = 0.0f;
    }
    return valid;
}
// --- End Biquad Helpers ---


// Update coefficients based on parameters
static void EQ_UpdateCoefficients(EQState *state, const EqualizerParameters *params) {
    if (!state || !params) return;

    printf("DEBUG: EQ Updating Coefficients...\n"); // Add print for debugging

    // Calculate coeffs for each band using helper functions
    calc_low_shelf_coeffs(state->sample_rate, params->lowShelf.cutoff_freq, params->lowShelf.q_factor, params->lowShelf.gain_db, &state->band_states[0], 0);
    calc_high_shelf_coeffs(state->sample_rate, params->highShelf.cutoff_freq, params->highShelf.q_factor, params->highShelf.gain_db, &state->band_states[1], 1);
    calc_peak_coeffs(state->sample_rate, params->band0.cutoff_freq, params->band0.q_factor, params->band0.gain_db, &state->band_states[2], 2);
    calc_peak_coeffs(state->sample_rate, params->band1.cutoff_freq, params->band1.q_factor, params->band1.gain_db, &state->band_states[3], 3);
    calc_peak_coeffs(state->sample_rate, params->band2.cutoff_freq, params->band2.q_factor, params->band2.gain_db, &state->band_states[4], 4);
    calc_peak_coeffs(state->sample_rate, params->band3.cutoff_freq, params->band3.q_factor, params->band3.gain_db, &state->band_states[5], 5);

    // Store current params and clear dirty flag
    memcpy(&state->last_params, params, sizeof(EqualizerParameters));
    state->coeffs_dirty = false;
}


void EQ_Init(EQState *state, float sample_rate) {

    printf("DEBUG: EQ_Init called with sample_rate: %f\n", sample_rate); // Debug print
    if (!state) 
    {
        printf("ERROR: EQ_Init called with NULL state pointer!\n");
        return;
    }
    memset(state, 0, sizeof(EQState)); // Clear state (x1, x2) and coefficients
    state->sample_rate = sample_rate;
    state->coeffs_dirty = true; // Force initial coefficient calculation
    // Initialize last_params with values guaranteed to be different
    memset(&state->last_params, 0xFF, sizeof(EqualizerParameters));
    // Set initial gain to something non-zero to ensure memcmp fails first time
    state->last_params.band0.gain_db = -999.0f;
}


void EQ_Process(EQState *state, float* buffer, uint32_t num_samples, const EqualizerParameters *params) {
    if (!state || !buffer || !params || !params->enabled) {
        return; // Bypass
    }

    // Check if parameters have changed, update coefficients if necessary
    if (state->coeffs_dirty || memcmp(&state->last_params, params, sizeof(EqualizerParameters)) != 0) {
        EQ_UpdateCoefficients(state, params);
    }

    // print out the coefficients for debugging
    for (int band = 0; band < EQ_MAX_BANDS; ++band) {
        BiquadState* bs = &state->band_states[band];
        printf("DEBUG: EQ Band %d Coeffs: b0=%.2e b1=%.2e b2=%.2e a1=%.2e a2=%.2e\n", band, bs->b0, bs->b1, bs->b2, bs->a1, bs->a2);
    }

    // Process samples through each band sequentially
    for (int band = 0; band < EQ_MAX_BANDS; ++band) {
        BiquadState* bs = &state->band_states[band];

        // Check if this specific band's coefficients are valid (might have been set flat due to bad params)
        // A flat band would have b0=1, b1=0, b2=0, a1=0, a2=0
        bool is_flat = (fabsf(bs->b0 - 1.0f) < FLT_EPSILON) && (fabsf(bs->b1) < FLT_EPSILON) &&
                       (fabsf(bs->b2) < FLT_EPSILON) && (fabsf(bs->a1) < FLT_EPSILON) &&
                       (fabsf(bs->a2) < FLT_EPSILON);

        if (is_flat) {
            // printf("DEBUG: EQ Band %d - Coefficients flat, skipping processing.\n", band);
            continue; // Skip processing for this band if it's set to flat/bypass
        }

        // Apply the difference equation for this band (DFII-T)
        for (uint32_t i = 0; i < num_samples; ++i) {
            float x0 = buffer[i]; // Current input sample for this stage

            // --- Check for NaN/Inf in state before calculation ---
             if (isnan(bs->x1) || isinf(bs->x1) || isnan(bs->x2) || isinf(bs->x2)) {
                 printf("ERROR: EQ Band %d, Sample %lu: Input state NaN/Inf! Resetting state.\n", band, i);
                 bs->x1 = 0.0f;
                 bs->x2 = 0.0f;
                 // Possibly zero output to prevent propagating NaN further?
                 // buffer[i] = 0.0f;
                 // continue; // Or just process with zeroed state
             }

            // Difference Equation: y[n] = b0*x[n] + s1[n-1]
            //                  s1[n] = b1*x[n] - a1*y[n] + s2[n-1]
            //                  s2[n] = b2*x[n] - a2*y[n]
            float y0 = bs->b0 * x0 + bs->x1; // Calculate output

            // --- Check for NaN/Inf in output ---
            if (isnan(y0) || isinf(y0)) {
                printf("ERROR: EQ Band %d, Sample %lu: Output NaN/Inf! Coeffs: b0=%.2e a1=%.2e a2=%.2e\n", band, i, bs->b0, bs->a1, bs->a2);
                y0 = 0.0f;       // Output silence for this sample
                bs->x1 = 0.0f;   // Reset state
                bs->x2 = 0.0f;
            } else {
                 // Update state variables ONLY if output was valid
                 bs->x1 = bs->b1 * x0 + bs->a1 * y0 + bs->x2; // Note: signs of a1/a2 are already negated
                 bs->x2 = bs->b2 * x0 + bs->a2 * y0;

                 // --- Check for NaN/Inf in state after update ---
                 if (isnan(bs->x1) || isinf(bs->x1) || isnan(bs->x2) || isinf(bs->x2)) {
                     printf("ERROR: EQ Band %d, Sample %lu: Output state NaN/Inf! Resetting state.\n", band, i);
                     bs->x1 = 0.0f;
                     bs->x2 = 0.0f;
                 }
            }
            printf("DEBUG: EQ Band %d, Sample %lu: x0=%.2f, y0=%.2f, x1=%.2f, x2=%.2f\n", band, i, x0, y0, bs->x1, bs->x2); // Debug print
            buffer[i] = y0; // Output of this band becomes input for the next band
        }
    }
}