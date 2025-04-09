// Common/Inc/mixer_state.h

#ifndef MIXER_STATE_H // <-- Add Include Guard
#define MIXER_STATE_H // <-- Add Include Guard

#include <stdint.h>
#include <stdbool.h>

#define SHARED_MEM_BASE 0x38000000

// --- Remove Forward Typedefs (they are redundant with the full definitions below) ---
// typedef struct EqualizerParameters EqualizerParameters;
// ... and so on for all structs ...

// --- Struct Definitions (Keep these as they define the types) ---

// Helper substruct type for Equalizer shelves/bands
typedef struct { // No forward typedef needed here
    float gain_db;
    float cutoff_freq;
    float q_factor;
} EqualizerBandParameters; // Define the type name *after* the struct body

typedef struct { // No forward typedef needed here
    bool enabled;
    EqualizerBandParameters lowShelf;
    EqualizerBandParameters highShelf;
    EqualizerBandParameters band0;
    EqualizerBandParameters band1;
    EqualizerBandParameters band2;
    EqualizerBandParameters band3;
} EqualizerParameters; // Define the type name *after* the struct body

typedef struct { // No forward typedef needed here
    bool enabled;
    float threshold_db;
    float ratio;
    float attack_ms;
    float release_ms;
    float knee_db;
    float makeup_gain_db;
} CompressorParameters; // Define the type name *after* the struct body

typedef struct { // No forward typedef needed here
    bool enabled;
    float drive; // 0 db to 20 dB
    float output_gain_db; // -20 dB to 0 dB
} DistortionParameters; // Define the type name *after* the struct body

typedef struct { // No forward typedef needed here
    bool enabled;
    float rate; // 0.1 Hz to 10 Hz (logarithmic scaling)
    float depth; // from 0% to 100%
} PhaserParameters; // Define the type name *after* the struct body

typedef struct { // No forward typedef needed here
    bool enabled;
    float decay_time; // 0.3 seconds to 3 seconds
    float wet_level; // 0% to 100%
} ReverbParameters; // Define the type name *after* the struct body

// Define ChannelParameters *after* all the effect structs it uses
typedef struct { // No forward typedef needed here
    bool muted;
    bool soloed;
    float panning; // [0.0 .. 1.0]
    float digital_gain; // [-60 dB ... +6 dB]
    bool stereo; // only relevant for the main channel.

    EqualizerParameters equalizer;
    CompressorParameters compressor;
    DistortionParameters distortion;
    PhaserParameters phaser;
    ReverbParameters reverb;
} ChannelParameters; // Define the type name *after* the struct body

// Define MixerParameters *after* ChannelParameters
typedef struct { // No forward typedef needed here
    ChannelParameters channels[9]; // 0 = main, 1-8 correspond to input channels 1 to 8
    bool soloing_active;
    bool inferencing_active;
    bool hw_init_ready;
} MixerParameters; // Define the type name *after* the struct body


// --- Declarations of Global Variables (use extern) ---
// This tells the compiler these variables exist elsewhere.
extern volatile MixerParameters * const shared_buffer_0;
extern volatile MixerParameters * const shared_buffer_1; // Keep if using double buffering
extern volatile uint32_t * const shared_active_idx_ptr; // Keep if using double buffering

#endif // MIXER_STATE_H <-- Add Include Guard End -->