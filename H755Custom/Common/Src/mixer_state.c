/*
 * mixer_state.c
 *
 *  Created on: Apr 9, 2025
 *      Author: joshuayu
 */


// Common/Src/mixer_state.c

#include "mixer_state.h" // Include the header with the 'extern' declarations

// --- Definitions of Global Variables ---
// This is where the memory is actually associated with the pointers.
volatile MixerParameters * const shared_buffer_0 = (MixerParameters *)SHARED_MEM_BASE;

// --- If you ARE using double buffering, uncomment these ---
// volatile MixerParameters * const shared_buffer_1 = (MixerParameters *)(SHARED_MEM_BASE + sizeof(MixerParameters));
// volatile uint32_t * const shared_active_idx_ptr = (uint32_t *)(SHARED_MEM_BASE + 2 * sizeof(MixerParameters));

// --- If you are NOT using double buffering, remove or comment out the lines above for buffer_1 and idx_ptr ---