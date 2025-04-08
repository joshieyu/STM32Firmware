#include <stdint.h>
#include <stdbool.h>

#define SHARED_MEM_BASE 0x38000000

volatile MixerParameters * const shared_buffer_0 = (MixerParameters *)SHARED_MEM_BASE;
volatile MixerParameters * const shared_buffer_1 = (MixerParameters *)(SHARED_MEM_BASE + sizeof(MixerParameters));
volatile uint32_t * const shared_active_idx_ptr = (uint32_t *)(SHARED_MEM_BASE + 2 * sizeof(MixerParameters));

// Defines typedefs for effects
typedef struct EqualizerParameters EqualizerParameters;
typedef struct CompressorParameters CompressorParameters;
typedef struct DistortionParameters DistortionParameters;
typedef struct PhaserParameters PhaserParameters;
typedef struct ReverbParameters ReverbParameters;

// Define typedefs for channel and overall mixer state
typedef struct ChannelParameters ChannelParameters;
typedef struct MixerParameters MixerParameters;


// Each of the following 5 effect parameter structs has an "enabled" bool.
// If !enabled, skip processing that effect

// Helper substruct type for Equalizer shelves/bands
typedef struct EqualizerBandParameters {
    float gain_db;
    float cutoff_freq;
    float q_factor;
} EqualizerBandParameters;

typedef struct EqualizerParameters {
    bool enabled;
    EqualizerBandParameters lowShelf;
    EqualizerBandParameters highShelf;
    EqualizerBandParameters band0;
    EqualizerBandParameters band1;
    EqualizerBandParameters band2;
    EqualizerBandParameters band3;
} EqualizerParameters;

typedef struct CompressorParameters {
    bool enabled;
    float threshold_db;
    float ratio;
    float attack_ms;
    float release_ms;
    float knee_db;
    float makeup_gain_db;
} CompressorParameters;

typedef struct DistortionParameters {
    bool enabled;
    float drive; // 0 db to 20 dB
    float output_gain_db; // -20 dB to 0 dB
} DistortionParameters;

typedef struct PhaserParameters {
    bool enabled;
    float rate; // 0.1 Hz to 10 Hz (logarithmic scaling)
    float depth; // from 0% to 100%
} PhaserParameters;

// Reverb is only found on the main bus
typedef struct ReverbParameters {
    bool enabled;
    float decay_time; // 0.3 seconds to 3 seconds
    float wet_level; // 0% to 100%
} ReverbParameters;

// Channel Parameters
typedef struct ChannelParameters {
    // Following 5 values are parameters for the channel in and of itself
    bool muted; // 0 = not muted, 1 = muted (or use bool if supported)
    bool soloed; // 0 = not soloed, 1 = soloed
    float panning; // [0.0 .. 1.0]; 0.5 is center, 0.0 is left, 1.0 is right
    float digital_gain // 0 db = signal is unmodified. Range is [-60 dB (essentially -inf dB) ... +6 dB]

    bool stereo; // only relevant for the main channel. 0 = mono, 1 = stereo
    // analog gain is not stored in shared memory, ... CM4 handles that by itself

    // Following 5 values are structs containing parameters for each of our 5 effects
    EqualizerParameters equalizer;
    CompressorParameters compressor;
    DistortionParameters distortion;
    PhaserParameters phaser;
    ReverbParameters reverb;
} ChannelParameters;

// Parameters corresponding to overall mixer state
typedef struct MixerParameters {
    ChannelParameters channels[9]; // 0 = main, 1-8 correspond to input channels 1 to 8
    bool soloing_active; // true if one or more channels are soloed
    bool inferencing_active; // true if inferencing is underway (i.e. audio data is being streamed from STM to Pi)
    bool hw_init_ready; // true is ADCs/DACs are initialized, false if not (and the CM7 should not perform DSP)
} MixerParameters;