// Common/Inc/mixer_state.h

#ifndef MIXER_STATE_H
#define MIXER_STATE_H

#include <stdint.h>
#include <stdbool.h> // Use standard bool

#define SHARED_MEM_BASE 0x38000000

// --- Alignment Attribute Macro (Compiler dependent, GCC/Clang example) ---
#define ALIGN(N) __attribute__((aligned(N)))

// --- Struct Definitions with Alignment ---

// Helper substruct type for Equalizer shelves/bands
// Align to 4 bytes because it contains floats
typedef struct ALIGN(4) {
    float gain_db;
    float cutoff_freq;
    float q_factor;
} EqualizerBandParameters;

// Align to 4 bytes
typedef struct ALIGN(4) {
    bool enabled;
    // bool padding[3]; // Optional explicit padding if needed after bool
    EqualizerBandParameters lowShelf;
    EqualizerBandParameters highShelf;
    EqualizerBandParameters band0;
    EqualizerBandParameters band1;
    EqualizerBandParameters band2;
    EqualizerBandParameters band3;
} EqualizerParameters;

// Align to 4 bytes
typedef struct ALIGN(4) {
    bool enabled;
    // bool padding[3]; // Optional explicit padding
    float threshold_db;
    float ratio;
    float attack_ms;
    float release_ms;
    float knee_db;
    float makeup_gain_db;
} CompressorParameters;

// Align to 4 bytes
typedef struct ALIGN(4) {
    bool enabled;
    // bool padding[3];
    float drive;
    float output_gain_db;
} DistortionParameters;

// Align to 4 bytes
typedef struct ALIGN(4) {
    bool enabled;
    // bool padding[3];
    float rate;
    float depth;
} PhaserParameters;

// Align to 4 bytes
typedef struct ALIGN(4) {
    bool enabled;
    // bool padding[3];
    float decay_time;
    float wet_level;
} ReverbParameters;

// Align ChannelParameters to 4 bytes (contains floats and aligned structs)
typedef struct ALIGN(4) {
    // Group bools together - compiler might pack them better
    bool muted;
    bool soloed;
    bool stereo;
    // bool padding1; // Optional explicit padding
    float analog_gain;
    float panning;
    float digital_gain;
    // Analog gain is handled separately, not stored here

    // Nested structs are already aligned to 4 bytes
    EqualizerParameters equalizer;
    CompressorParameters compressor;
    DistortionParameters distortion;
    PhaserParameters phaser;
    ReverbParameters reverb;
} ChannelParameters;

// Align MixerParameters to 8 bytes (or 16 if very paranoid/using specific SIMD)
// Contains an array of aligned structs and bools at the end.
typedef struct ALIGN(8) { // Align main struct to 8 bytes
    ChannelParameters channels[9]; // Array of aligned structs

    // Bools at the end
    bool soloing_active;
    bool inferencing_active;
    bool hw_init_ready;
    // bool padding_end[1]; // Optional padding to make total size multiple of 8? Check sizeof.
} MixerParameters;


// --- Declarations of Global Variables ---
// Using volatile and const correctly for shared memory pointers
extern volatile MixerParameters * const shared_buffer_0;
// extern volatile MixerParameters * const shared_buffer_1; // Uncomment if using double buffer
// extern volatile uint32_t * const shared_active_idx_ptr; // Uncomment if using double buffer

#endif // MIXER_STATE_H