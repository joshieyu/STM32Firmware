// audio_dsp.h (For CM7 Core)
#ifndef AUDIO_DSP_H
#define AUDIO_DSP_H

#include <stdint.h>
#include "mixer_state.h" // Include YOUR shared structure definition

// --- Defines ---
// Maximum samples per channel in one processing block (DMA half buffer size / num channels)
// Example: If TDM_RX_HALF_SIZE=4096, TDM_SLOTS=8 => 512
#define DSP_MAX_SAMPLES_PER_CHUNK 512 // *** ADJUST THIS TO MATCH YOUR DMA SETUP! ***

#define DSP_INPUT_CHANNELS 8   // Number of physical input channels (1-8 in MixerParameters)
#define DSP_OUTPUT_CHANNELS 2  // Stereo Output

// --- Public Function Prototypes ---

/**
 * @brief Initializes the DSP engine, internal states, and effects.
 * @param sample_rate The system audio sample rate (e.g., 48000.0f).
 */
void AudioDSP_Init();

/**
 * @brief Processes one block of audio data according to parameters in shared memory.
 *        Reads directly from shared_buffer_0 (or your designated single buffer).
 *        Handles demux, channel DSP, mixdown, master DSP, and output formatting.
 * @param rx_chunk_start Pointer to the beginning of the input (RX) interleaved audio chunk.
 * @param rx_chunk_num_samples Total number of *individual samples* in the input chunk (e.g., 4096).
 * @param tx_chunk_start Pointer to the beginning of the output (TX) stereo audio chunk.
 * @param tx_chunk_num_stereo_samples Total number of *individual samples* in the output chunk (e.g., 1024 for 512 L/R pairs).
 */
void AudioDSP_Process(int32_t* rx_chunk_start, uint32_t rx_chunk_num_samples,
                      int32_t* tx_chunk_start, uint32_t tx_chunk_num_stereo_samples);

#endif // AUDIO_DSP_H