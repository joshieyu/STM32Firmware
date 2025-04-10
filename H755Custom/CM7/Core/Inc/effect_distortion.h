#ifndef EFFECT_DISTORTION_H
#define EFFECT_DISTORTION_H

#include <stdint.h>
#include <stdbool.h>
#include "mixer_state.h" // Needs DistortionParameters

// Can be stateless for simple waveshaping
typedef struct {
   // Add state here if pre/post filtering is included
   int placeholder; // Struct cannot be empty if used with sizeof() etc.
} DistortionState;

// Public Functions
void Distortion_Init(DistortionState *state); // May do nothing
void Distortion_Process(DistortionState *state, float* buffer, uint32_t num_samples, const DistortionParameters *params);

#endif // EFFECT_DISTORTION_H