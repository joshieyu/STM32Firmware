// Src/audio_dsp.c
#include "audio_dsp.h"
#include <stddef.h> // For NULL
#include <string.h> // For memset

// --- Include Headers for specific effect implementations ---
#include "effect_gain.h" // Assuming you create this for gain control
// #include "effect_filter.h" // Assuming you create this

// --- Private Defines ---
#define MAX_AMPLITUDE_24BIT (8388607)
#define MIN_AMPLITUDE_24BIT (-8388608)

// --- Private Variables ---

// Buffers for demultiplexed channel data
static int32_t channel_proc_buffers[DSP_NUM_TDM_CHANNELS][DSP_MAX_SAMPLES_PER_CHANNEL_CHUNK];

// Buffers for the stereo master bus (before output formatting)
// Stored as 24-bit values (shifted down) for processing convenience
static int32_t master_bus_buffers[DSP_NUM_OUTPUT_CHANNELS][DSP_MAX_SAMPLES_PER_CHANNEL_CHUNK];

// Configuration: Which effect is active on each input channel?
static DSPEffectType_t channel_effects[DSP_NUM_TDM_CHANNELS];

// Configuration: Which effect is active on the master bus?
static DSPEffectType_t master_bus_effect;

// --- Effect State Variables (Example) ---
// If effects have state (like filter coefficients, gain levels), declare them here.
// static float channel_gains[DSP_NUM_TDM_CHANNELS];
// static float master_gain;
// static BiquadCoeffs channel_filters[DSP_NUM_TDM_CHANNELS];
// etc.


// --- Private Helper Functions ---

/**
 * @brief Clips a 64-bit value to the range of a 24-bit signed integer.
 */
static inline int32_t ClipSample24Bit(int64_t sample) {
    if (sample > MAX_AMPLITUDE_24BIT) return MAX_AMPLITUDE_24BIT;
    if (sample < MIN_AMPLITUDE_24BIT) return MIN_AMPLITUDE_24BIT;
    return (int32_t)sample;
}


/**
 * @brief Applies the configured DSP effect to a single channel's buffer (in-place).
 * @param channel_index The index of the channel (0 to TDM_SLOTS-1).
 * @param channel_buffer Pointer to the buffer (contains 32-bit left-aligned samples).
 * @param num_samples Number of samples in the buffer.
 */
static void ProcessChannelDSP(uint8_t channel_index, int32_t* channel_buffer, uint32_t num_samples) {
    DSPEffectType_t effect_type = channel_effects[channel_index];

    switch (effect_type) {
        case DSP_EFFECT_GAIN:
            // Example: Assumes an EffectGain_Process function exists
            // It would likely read a gain value from channel_gains[channel_index]
            // EffectGain_Process(channel_buffer, num_samples, channel_gains[channel_index]);
            break;

        case DSP_EFFECT_FILTER:
            // Example: Assumes an EffectFilter_Process function exists
            // It would likely use state/coefficients stored for this channel
            // EffectFilter_Process(channel_buffer, num_samples, &channel_filters[channel_index]);
            break;

        case DSP_EFFECT_BYPASS:
        default:
            // Do nothing for bypass or unknown
            break;
    }
    // Note: Effects should modify the channel_buffer in-place.
}

/**
 * @brief Mixes processed channel data into the stereo master bus buffers.
 *        THIS FUNCTION DEFINES YOUR FIXED MIXING ROUTING.
 * @param num_samples Number of samples per channel.
 */
static void MixChannelsToMasterBus(uint32_t num_samples) {
    // Clear master bus buffers before summing
    memset(master_bus_buffers, 0, sizeof(master_bus_buffers));

    for (uint32_t frame = 0; frame < num_samples; ++frame) {
        int64_t sum_l = 0;
        int64_t sum_r = 0;

        // --- DEFINE YOUR MIXING RULES HERE ---
        // Example: Ch0->L, Ch1->R, Ch2->L(quiet), Ch3->R(quiet), Others ignored
        if (DSP_NUM_TDM_CHANNELS >= 1) sum_l += (int64_t)(channel_proc_buffers[0][frame] >> 8); // Ch0 to Left
        if (DSP_NUM_TDM_CHANNELS >= 2) sum_r += (int64_t)(channel_proc_buffers[1][frame] >> 8); // Ch1 to Right
        if (DSP_NUM_TDM_CHANNELS >= 3) sum_l += (int64_t)(channel_proc_buffers[2][frame] >> 8) / 4; // Ch2 quieter to Left
        if (DSP_NUM_TDM_CHANNELS >= 4) sum_r += (int64_t)(channel_proc_buffers[3][frame] >> 8) / 4; // Ch3 quieter to Right
        // ... add rules for channels 4, 5, 6, 7 if needed ...

        // --- END MIXING RULES ---

        // Clip the sum for each master bus channel
        // Store as 24-bit value (no left-shift yet)
        master_bus_buffers[0][frame] = ClipSample24Bit(sum_l); // Left Bus
        master_bus_buffers[1][frame] = ClipSample24Bit(sum_r); // Right Bus
    }
}


/**
 * @brief Applies the configured DSP effect to the stereo master bus buffers (in-place).
 * @param num_samples Number of samples per channel.
 */
static void ApplyDSP_MasterBus(uint32_t num_samples) {
    switch (master_bus_effect) {
        case DSP_EFFECT_GAIN:
            // Example: Apply gain to both L and R master bus buffers
            // Assumes EffectGain_Process can handle this or call it twice
            // EffectGain_Process(master_bus_buffers[0], num_samples, master_gain); // Process Left
            // EffectGain_Process(master_bus_buffers[1], num_samples, master_gain); // Process Right
            break;

        case DSP_EFFECT_FILTER:
            // Example: Apply filter to L/R
            // EffectFilter_ProcessStereo(master_bus_buffers[0], master_bus_buffers[1], num_samples, &master_bus_filter_coeffs);
            break;

        case DSP_EFFECT_BYPASS:
        default:
            // Do nothing
            break;
    }
    // Note: Master bus effects modify master_bus_buffers in-place.
    // They operate on the 24-bit data (not left-shifted yet).
}


// --- Public Function Implementations ---

void AudioDSP_Init(void) {
    memset(channel_proc_buffers, 0, sizeof(channel_proc_buffers));
    memset(master_bus_buffers, 0, sizeof(master_bus_buffers));

    // Set default effects to bypass
    for (int i = 0; i < DSP_NUM_TDM_CHANNELS; ++i) {
        channel_effects[i] = DSP_EFFECT_BYPASS;
        // Initialize default parameters if needed (e.g., channel_gains[i] = 1.0f;)
    }
    master_bus_effect = DSP_EFFECT_BYPASS;
    // Initialize default master parameters (e.g., master_gain = 1.0f;)

    // Call Init functions for specific effect modules if they have state
    // EffectGain_Init();
    // EffectFilter_Init();
}

int AudioDSP_SetChannelEffect(uint8_t channel_index, DSPEffectType_t effect) {
    if (channel_index >= DSP_NUM_TDM_CHANNELS || effect >= DSP_EFFECT_TYPE_COUNT) {
        return -1; // Invalid input
    }
    channel_effects[channel_index] = effect;
    // Optional: Reset parameters for this channel when changing effect type
    return 0;
}

int AudioDSP_SetMasterBusEffect(DSPEffectType_t effect) {
    if (effect >= DSP_EFFECT_TYPE_COUNT) {
        return -1; // Invalid input
    }
    master_bus_effect = effect;
    // Optional: Reset master parameters when changing effect type
    return 0;
}


/**
 * @brief Processes audio chunk through the full pipeline.
 */
void AudioDSP_Process(int32_t* rx_chunk_start, uint32_t rx_chunk_num_samples,
                      int32_t* tx_chunk_start, uint32_t tx_chunk_num_stereo_samples)
{
    // --- Input Validation ---
    if (rx_chunk_start == NULL || tx_chunk_start == NULL) return;

    uint32_t samples_per_channel = rx_chunk_num_samples / DSP_NUM_TDM_CHANNELS;
    uint32_t num_tx_pairs = tx_chunk_num_stereo_samples / DSP_NUM_OUTPUT_CHANNELS;

    if (samples_per_channel == 0 || samples_per_channel > DSP_MAX_SAMPLES_PER_CHANNEL_CHUNK ||
        samples_per_channel != num_tx_pairs ||
        rx_chunk_num_samples % DSP_NUM_TDM_CHANNELS != 0 ||
        tx_chunk_num_stereo_samples % DSP_NUM_OUTPUT_CHANNELS != 0)
    {
        memset(tx_chunk_start, 0, tx_chunk_num_stereo_samples * sizeof(int32_t));
        return;
    }

    // --- Stage 1: Demultiplex ---
    for (uint32_t frame = 0; frame < samples_per_channel; ++frame) {
        uint32_t rx_frame_start_index = frame * DSP_NUM_TDM_CHANNELS;
        for (int ch = 0; ch < DSP_NUM_TDM_CHANNELS; ++ch) {
            channel_proc_buffers[ch][frame] = rx_chunk_start[rx_frame_start_index + ch];
        }
    }

    // --- Stage 2: Per-Channel DSP ---
    for (int ch = 0; ch < DSP_NUM_TDM_CHANNELS; ++ch) {
        ProcessChannelDSP(ch, channel_proc_buffers[ch], samples_per_channel);
    }

    // --- Stage 3: Mixdown to Master Bus ---
    MixChannelsToMasterBus(samples_per_channel);

    // --- Stage 4: Master Bus DSP ---
    ApplyDSP_MasterBus(samples_per_channel);

    // --- Stage 5: Output Formatting ---
    // Copy processed & clipped master bus data to the final TX buffer, left-shifting.
    for (uint32_t frame = 0; frame < samples_per_channel; ++frame) {
        uint32_t tx_pair_start_index = frame * DSP_NUM_OUTPUT_CHANNELS;
        tx_chunk_start[tx_pair_start_index + 0] = master_bus_buffers[0][frame]; // Left Output
        tx_chunk_start[tx_pair_start_index + 1] = master_bus_buffers[1][frame]; // Right Output
    }
}

// --- Implement parameter setting functions if needed ---
// int AudioDSP_SetChannelGain(uint8_t channel_index, float gain_db) { ... }
// int AudioDSP_SetMasterBusGain(float gain_db) { ... }