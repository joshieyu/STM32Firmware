// Inc/audio_dsp.h
#ifndef INC_AUDIO_DSP_H_
#define INC_AUDIO_DSP_H_

#include <stdint.h>

// --- Defines ---
#define DSP_NUM_TDM_CHANNELS    8
#define DSP_NUM_OUTPUT_CHANNELS 2 // Stereo Master Bus & Output

#define DSP_MAX_SAMPLES_PER_CHANNEL_CHUNK 256 // ADJUST THIS! (TDM_RX_HALF_SIZE / DSP_NUM_TDM_CHANNELS)

// --- Typedefs ---

/**
 * @brief Enum defining available DSP effect types.
 *        These can be applied per-channel or on the master bus.
 */
typedef enum {
    DSP_EFFECT_BYPASS,      // No processing
    DSP_EFFECT_GAIN,        // Simple gain adjustment
    DSP_EFFECT_FILTER,      // Example: Biquad filter
    // Add other specific effect types here...
    DSP_EFFECT_TYPE_COUNT // Helper
} DSPEffectType_t;

// --- Public Function Prototypes ---

/**
 * @brief Initializes the DSP module, effects, and routing.
 */
void AudioDSP_Init(void);

/**
 * @brief Assigns a DSP effect to a specific input channel.
 * @param channel_index Input channel (0 to DSP_NUM_TDM_CHANNELS - 1).
 * @param effect The effect type to apply (DSP_EFFECT_BYPASS for none).
 * @retval 0 on success, -1 on error (invalid channel index or effect).
 */
int AudioDSP_SetChannelEffect(uint8_t channel_index, DSPEffectType_t effect);

/**
 * @brief Assigns a DSP effect to the stereo master bus.
 * @param effect The effect type to apply (DSP_EFFECT_BYPASS for none).
 * @retval 0 on success, -1 on error (invalid effect).
 */
int AudioDSP_SetMasterBusEffect(DSPEffectType_t effect);

// Optional: Functions to set parameters for specific effects
// int AudioDSP_SetChannelGain(uint8_t channel_index, float gain_db);
// int AudioDSP_SetMasterBusGain(float gain_db);


/**
 * @brief Processes a chunk of audio data using the configured channel/master effects and mixing.
 *        (Signature remains the same)
 */
void AudioDSP_Process(int32_t* rx_chunk_start, uint32_t rx_chunk_num_samples,
                      int32_t* tx_chunk_start, uint32_t tx_chunk_num_stereo_samples);


#endif /* INC_AUDIO_DSP_H_ */