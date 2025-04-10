#include "effect_distortion.h"
#include <math.h>

// Helper to convert dB to linear
static inline float DB_to_Linear_Dist(float db) {
    return powf(10.0f, db / 20.0f);
}

// Helper to clip float
static inline float ClipFloatPM1(float sample) {
    if (sample > 1.0f) return 1.0f;
    if (sample < -1.0f) return -1.0f;
    return sample;
}

void Distortion_Init(DistortionState *state) {
    // Initialize any state if added (e.g., filter state)
    (void)state; // Mark as unused if stateless
}

void Distortion_Process(DistortionState *state, float* buffer, uint32_t num_samples, const DistortionParameters *params) {
    if (!state || !buffer || !params || !params->enabled) {
        
        return; // Bypass
    }
    (void)state; // Mark as unused if stateless

    // Convert gains from dB to linear
    float drive_lin = DB_to_Linear_Dist(fmaxf(0.0f, params->drive)); // Ensure drive >= 0dB
    float output_gain_lin = DB_to_Linear_Dist(params->output_gain_db);

    for (uint32_t i = 0; i < num_samples; ++i) {
        float sample = buffer[i];

        // 1. Apply Drive Gain
        sample *= drive_lin;

        // 2. Apply Waveshaping Function (tanh is a common soft clipper)
        // Other options: atanf(sample), sample / (1 + fabsf(sample)), hard clipping...
        sample = tanhf(sample);

        // 3. Apply Output Gain
        sample *= output_gain_lin;

        // 4. Optional: Clip output to +/- 1.0 before returning, although
        //    the main DSP loop also has clipping. Depends if you want
        //    distortion character or just level control here.
        // sample = ClipFloatPM1(sample);

        buffer[i] = sample;
    }
}