// audio_dsp.c (For CM7 Core)
#include "audio_dsp.h"
#include <string.h> // For memset
#include <math.h>   // For powf, sinf, cosf etc.
#include <stdbool.h>

// --- Include Headers for your actual effect implementations ---
// Ensure these define State structs, _Init, and _Process functions
#include "effect_eq.h"
#include "effect_compressor.h"
#include "effect_distortion.h"
#include "effect_phaser.h"
#include "effect_reverb.h" // Needed for Master Reverb

// --- Private Defines ---
#define PI_F 3.14159265358979323846f

// Scaling/Clipping values
#define FLOAT_SCALE_FACTOR (1.0f / 8388607.0f) // Scale int24 range to approx +/- 1.0f
#define MAX_AMPLITUDE_24BIT_I (8388607)
#define MIN_AMPLITUDE_24BIT_I (-8388608)


#define TEST_EFFECT_TARGET_CHANNEL 4

// --- Static Variables ---

// Pointer to the single shared parameter buffer (using the definition from your header)
// Note: We read directly from shared_buffer_0 as per your updated requirement.
// Make sure shared_buffer_0 itself is correctly defined and linked to the shared memory address.
static volatile const MixerParameters* g_params = NULL; // Initialize in AudioDSP_Init
static volatile MixerParameters g_local_params; // Local copy for processing

// Current sample rate
static float g_sample_rate = 44100.0f;

// Internal processing buffers (using float)
static float channel_proc_buffers[DSP_INPUT_CHANNELS][DSP_MAX_SAMPLES_PER_CHUNK]; // Indices 0-7
static float master_bus_buffer_L[DSP_MAX_SAMPLES_PER_CHUNK];
static float master_bus_buffer_R[DSP_MAX_SAMPLES_PER_CHUNK];

// --- Effect State Structures ---
// These hold the *runtime state* (filter history, LFO phase, etc.)

// // Channel States (Arrays match INPUT channels 0-7)
static EQState         channel_eq_states[DSP_INPUT_CHANNELS];
static CompressorState channel_comp_states[DSP_INPUT_CHANNELS];
// // Distortion might be stateless or need state struct
static DistortionState channel_dist_states[DSP_INPUT_CHANNELS]; // Assuming DistortionState exists
static PhaserState     channel_phaser_states[DSP_INPUT_CHANNELS];
// // Reverb state only needed for Master

// // Master Bus States (Master corresponds to channels[0] in params)
static EQState         master_eq_state; // Assuming stereo EQ process or separate L/R state if needed
static CompressorState master_comp_state; // Assuming stereo Comp process
static ReverbState     master_reverb_state;


// --- Private Helper Functions ---

// Clipping
static inline float ClipFloat(float sample, float min_val, float max_val) {
    if (sample > max_val) return max_val;
    if (sample < min_val) return min_val;
    return sample;
}

// dB to Linear conversion
static inline float DB_to_Linear(float db) {
    // Handle very low dB values to prevent large negative gains becoming huge positive numbers
    if (db < -60.0f) return 0.0f;
    return powf(10.0f, db / 20.0f);
}

// Panning Gains (0=Left, 0.5=Center, 1.0=Right)
static void CalculatePanFactors(float pan_0_to_1, float* pan_l, float* pan_r) {
    // Ensure pan is within 0.0 to 1.0 range
    pan_0_to_1 = ClipFloat(pan_0_to_1, 0.0f, 1.0f);
    // Use square root law for constant power panning
    float angle = pan_0_to_1 * PI_F * 0.5f; // Map 0..1 to 0..pi/2
    *pan_l = cosf(angle);
    *pan_r = sinf(angle);
}


// --- Public Function Implementations ---

// void AudioDSP_Init(float sample_rate) {
//     g_sample_rate = sample_rate;
//     g_params = shared_buffer_0; // Point to the designated shared buffer

//     printf("AudioDSP: Initializing...\r\n");
//     printf("AudioDSP: Sample Rate: %.1f Hz\r\n", g_sample_rate);

//     if (!g_params) {
//         printf("AudioDSP: Error! Shared parameter buffer pointer is NULL (shared_buffer_0 invalid?)\r\n");
//         // Handle fatal error - perhaps hang here if essential
//         while(1);
//     }

//     // Initialize internal processing buffers
//     memset(channel_proc_buffers, 0, sizeof(channel_proc_buffers));
//     memset(master_bus_buffer_L, 0, sizeof(master_bus_buffer_L));
//     memset(master_bus_buffer_R, 0, sizeof(master_bus_buffer_R));

//     // Initialize Effect States
//     printf("AudioDSP: Initializing effect states...\r\n");
//     for (int i = 0; i < DSP_INPUT_CHANNELS; ++i) {
//         EQ_Init(&channel_eq_states[i], g_sample_rate);
//         Compressor_Init(&channel_comp_states[i], g_sample_rate);
//         Distortion_Init(&channel_dist_states[i]); // Pass state if needed
//         Phaser_Init(&channel_phaser_states[i], g_sample_rate);
//         // No reverb state per channel
//     }
//     // Initialize Master States
//     EQ_Init(&master_eq_state, g_sample_rate);
//     Compressor_Init(&master_comp_state, g_sample_rate);
//     Reverb_Init(&master_reverb_state, g_sample_rate);

//     printf("AudioDSP: Initialization Complete.\r\n");
// }

// void AudioDSP_Init() {

//     printf("AudioDSP (Test Mode): Initializing...\r\n");
//     printf("AudioDSP: Sample Rate: %.1f Hz\r\n", g_sample_rate);

//     // --- Initialize LOCAL Parameters with Defaults ---
//     printf("AudioDSP: Setting default local parameters...\r\n");
//     memset(&g_local_params, 0, sizeof(MixerParameters)); // Clear everything first

//     // g_params = shared_buffer_0; // Point to the designated shared buffer

//     // Global defaults
//     g_local_params.soloing_active = false;
//     g_local_params.inferencing_active = false;
//     g_local_params.hw_init_ready = true; // Assume ready for testing

//     // Master Channel (Index 0) Defaults
//     g_local_params.channels[0].muted = false;
//     g_local_params.channels[0].soloed = false; // Master usually isn't soloed
//     g_local_params.channels[0].panning = 0.5f; // Center
//     g_local_params.channels[0].digital_gain = 0.0f; // Unity gain
//     g_local_params.channels[0].stereo = true; // Default to stereo output
//     // Master Effects Defaults (example: disabled)
//     g_local_params.channels[0].equalizer.enabled = false;
//     // Set some default EQ band params if needed, e.g., flat
//     g_local_params.channels[0].compressor.enabled = false;
//     g_local_params.channels[0].reverb.enabled = false;

//     // Input Channels (Indices 1-8) Defaults
//     for (int i = 1; i <= DSP_INPUT_CHANNELS; ++i) {
//         g_local_params.channels[i].muted = false;
//         g_local_params.channels[i].soloed = false;
//         g_local_params.channels[i].panning = 0.5f; // Center pan
//         g_local_params.channels[i].digital_gain = 0.0f; // Unity gain
//         // Input Channel Effects Defaults (example: all disabled)
//         g_local_params.channels[i].equalizer.enabled = false;
//         g_local_params.channels[i].compressor.enabled = false;
//         g_local_params.channels[i].distortion.enabled = false;
//         g_local_params.channels[i].phaser.enabled = false;
//         // Reverb not applicable per input channel in this struct design
//     }

//     // --- Initialize internal processing buffers ---
//     memset(channel_proc_buffers, 0, sizeof(channel_proc_buffers));
//     memset(master_bus_buffer_L, 0, sizeof(master_bus_buffer_L));
//     memset(master_bus_buffer_R, 0, sizeof(master_bus_buffer_R));

//     // // --- Initialize Effect States ---
//     // printf("AudioDSP: Initializing effect states...\r\n");
//     for (int i = 0; i < DSP_INPUT_CHANNELS; ++i) {
//         EQ_Init(&channel_eq_states[i], g_sample_rate);
//     //     Compressor_Init(&channel_comp_states[i], g_sample_rate);
//         Distortion_Init(&channel_dist_states[i]);
//     //     Phaser_Init(&channel_phaser_states[i], g_sample_rate);
//     }
//     EQ_Init(&master_eq_state, g_sample_rate);
//     // Compressor_Init(&master_comp_state, g_sample_rate);
//     // Reverb_Init(&master_reverb_state, g_sample_rate);

//     printf("AudioDSP (Test Mode): Initialization Complete.\r\n");
// }

// --- Public Function Implementations ---

void AudioDSP_Init() {

    printf("AudioDSP (Test Mode): Initializing...\r\n");
    printf("AudioDSP: Sample Rate: %.1f Hz\r\n", g_sample_rate);

    // --- Initialize LOCAL Parameters with Defaults ---
    printf("AudioDSP: Setting default local parameters...\r\n");
    memset(&g_local_params, 0, sizeof(MixerParameters)); // Clear everything first

    // Global defaults
    g_local_params.soloing_active = false;
    g_local_params.inferencing_active = false;
    g_local_params.hw_init_ready = true; // Assume ready for testing

    // Master Channel (Index 0) Defaults
    g_local_params.channels[0].muted = false;
    g_local_params.channels[0].soloed = false;
    g_local_params.channels[0].panning = 0.5f;
    g_local_params.channels[0].digital_gain = 0.0f;
    g_local_params.channels[0].stereo = true;
    g_local_params.channels[0].equalizer.enabled = false;
    g_local_params.channels[0].compressor.enabled = false;
    // g_local_params.channels[0].reverb.enabled = false;

    // Inside AudioDSP_Init, after other defaults
    g_local_params.channels[0].reverb.enabled = false;
    g_local_params.channels[0].reverb.decay_time = 3.0f; // 1.5 seconds
    g_local_params.channels[0].reverb.wet_level = 0.2f; // 35% wet

    // Input Channels (Indices 1-8) Defaults
    for (int i = 1; i <= DSP_INPUT_CHANNELS; ++i) {
        g_local_params.channels[i].muted = false;
        g_local_params.channels[i].soloed = false;
        g_local_params.channels[i].panning = 0.5f;
        g_local_params.channels[i].digital_gain = 4.0f;
        g_local_params.channels[i].equalizer.enabled = false;
        g_local_params.channels[i].compressor.enabled = false;
        g_local_params.channels[i].distortion.enabled = false;
        g_local_params.channels[i].phaser.enabled = false;
    }

    // --- *** APPLY AUDIBLE TEST DEFAULTS TO THE SELECTED CHANNEL *** ---
    // Validate the target channel index
    if (TEST_EFFECT_TARGET_CHANNEL >= 1 && TEST_EFFECT_TARGET_CHANNEL <= DSP_INPUT_CHANNELS) {
        printf("AudioDSP: Applying audible test defaults to Channel %d...\r\n", TEST_EFFECT_TARGET_CHANNEL);

        // --- EQ Settings for Target Channel ---
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.enabled = false;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.lowShelf.gain_db = -0.0f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.lowShelf.cutoff_freq = 2000.0f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.lowShelf.q_factor = 0.707f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.highShelf.gain_db = -0.0f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.highShelf.cutoff_freq = 3000.0f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.highShelf.q_factor = 0.707f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.band0.gain_db = 12.0f; // Strong mid boost
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.band0.cutoff_freq = 3000.0f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.band0.q_factor = 1.5f;
        // Ensure other bands are flat (should be due to memset, but explicit is safe)
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.band1.gain_db = 0.0f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.band2.gain_db = 0.0f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].equalizer.band3.gain_db = 0.0f;

        // --- Distortion Settings for Target Channel ---
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].distortion.enabled = false;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].distortion.drive = 20.0f;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].distortion.output_gain_db = -10.0f;

        // --- Phaser Settings for Target Channel ---
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].phaser.enabled = true;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].phaser.rate = 0.5f; // 0.5 Hz
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].phaser.depth = 0.9f; // 50% depth

        // Inside AudioDSP_Init, after setting other defaults for channel 1
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].compressor.enabled = true;
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].compressor.threshold_db = -30.0f; // Low threshold
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].compressor.ratio = 6.0f;      // High ratio (6:1)
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].compressor.attack_ms = 100.0f;     // Fast attack
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].compressor.release_ms = 700.0f;  // Moderate release
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].compressor.knee_db = 0.0f;       // Soft knee of 6dB
        g_local_params.channels[TEST_EFFECT_TARGET_CHANNEL].compressor.makeup_gain_db = 6.0f; // Add some gain back

    } else {
        printf("AudioDSP: Warning - TEST_EFFECT_TARGET_CHANNEL (%d) is invalid. No test effects applied.\r\n", TEST_EFFECT_TARGET_CHANNEL);
    }
    // --- *** END OF TEST DEFAULTS APPLICATION *** ---


    // --- Initialize internal processing buffers ---
    memset(channel_proc_buffers, 0, sizeof(channel_proc_buffers));
    memset(master_bus_buffer_L, 0, sizeof(master_bus_buffer_L));
    memset(master_bus_buffer_R, 0, sizeof(master_bus_buffer_R));

    // --- Initialize ALL Effect States --- (Keep uncommented)
    printf("AudioDSP: Initializing effect states...\r\n");
    for (int i = 0; i < DSP_INPUT_CHANNELS; ++i) {
        EQ_Init(&channel_eq_states[i], g_sample_rate);
        Compressor_Init(&channel_comp_states[i], g_sample_rate);
        Distortion_Init(&channel_dist_states[i]);
        Phaser_Init(&channel_phaser_states[i], g_sample_rate);
    }
    EQ_Init(&master_eq_state, g_sample_rate);
    Compressor_Init(&master_comp_state, g_sample_rate);
    Reverb_Init(&master_reverb_state, g_sample_rate);

    printf("AudioDSP (Test Mode): Initialization Complete.\r\n");
}


void AudioDSP_Process(int32_t* rx_chunk_start, uint32_t rx_chunk_num_samples,
                      int32_t* tx_chunk_start, uint32_t tx_chunk_num_stereo_samples)
{

    // --- Pre-Checks ---
    if (!rx_chunk_start || !tx_chunk_start) 
    {
        printf("init return 1\r\n");
        return;
    }

    // Check if hardware is ready (flag set by CM4)
    if (!g_local_params.hw_init_ready) {
        printf("init return\r\n");
        memset(tx_chunk_start, 0, tx_chunk_num_stereo_samples * sizeof(int32_t)); // Output silence
        return;
    }

    uint32_t samples_per_channel = rx_chunk_num_samples / DSP_INPUT_CHANNELS; // Use DSP_INPUT_CHANNELS (8)
    uint32_t num_tx_pairs = tx_chunk_num_stereo_samples / DSP_OUTPUT_CHANNELS;

    // Validate buffer sizes
    if (samples_per_channel == 0 || samples_per_channel > DSP_MAX_SAMPLES_PER_CHUNK ||
        samples_per_channel != num_tx_pairs ||
        rx_chunk_num_samples % DSP_INPUT_CHANNELS != 0 ||
        tx_chunk_num_stereo_samples % DSP_OUTPUT_CHANNELS != 0)
    {
        memset(tx_chunk_start, 0, tx_chunk_num_stereo_samples * sizeof(int32_t));
        return;
    }

    // --- Stage 1: Demultiplex and Convert to Float ---
    for (uint32_t frame = 0; frame < samples_per_channel; ++frame) {
        uint32_t rx_frame_start_index = frame * DSP_INPUT_CHANNELS;
        for (int ch = 0; ch < DSP_INPUT_CHANNELS; ++ch) { // Loop 0-7
            // Convert int24 (in int32, left-aligned) to float approx +/- 1.0
            channel_proc_buffers[ch][frame] = (float)(rx_chunk_start[rx_frame_start_index + ch] >> 8) * FLOAT_SCALE_FACTOR;
        }
    }


    // --- Stage 2: Per-Channel DSP ---
    bool solo_mode_active = g_local_params.soloing_active;

    for (int i = 0; i < DSP_INPUT_CHANNELS; ++i) { // Loop 0-7 for buffers
        int param_idx = i + 1; // Corresponding index in params->channels[1..8]

        // Determine if channel should pass based on Mute/Solo
        bool channel_active = true;
        if (g_local_params.channels[param_idx].muted) {
            channel_active = false;
        } else if (solo_mode_active && !g_local_params.channels[param_idx].soloed) {
            channel_active = false;
        }

        if (channel_active) {
            // Apply effects sequentially if enabled
            // Pass pointer to relevant parameter struct from local memory
            const ChannelParameters* chan_p = &g_local_params.channels[param_idx];

            if (chan_p->equalizer.enabled) {
                EQ_Process(&channel_eq_states[i], channel_proc_buffers[i], samples_per_channel, &chan_p->equalizer);
                // printf("eq here\r\n");
            }
            if (chan_p->compressor.enabled) {
                Compressor_Process(&channel_comp_states[i], channel_proc_buffers[i], samples_per_channel, &chan_p->compressor);
            }
            if (chan_p->distortion.enabled) {
                Distortion_Process(&channel_dist_states[i], channel_proc_buffers[i], samples_per_channel, &chan_p->distortion);
                // printf("distortion here\r\n");
            }
            if (chan_p->phaser.enabled) {
                Phaser_Process(&channel_phaser_states[i], channel_proc_buffers[i], samples_per_channel, &chan_p->phaser);
            }
            // Reverb processing happens only on master according to struct design
        } else {
            // If channel is inactive (muted/not soloed), clear its processing buffer
            memset(channel_proc_buffers[i], 0, samples_per_channel * sizeof(float));
        }
    }


    // --- Stage 3: Mixdown (Apply Gain, Pan, Sum to Master) ---
    memset(master_bus_buffer_L, 0, samples_per_channel * sizeof(float));
    memset(master_bus_buffer_R, 0, samples_per_channel * sizeof(float));

    for (int i = 0; i < DSP_INPUT_CHANNELS; ++i) { // Loop 0-7 for buffers
        int param_idx = i + 1; // Corresponding index in params->channels[1..8]

        // Check active state again (don't sum inactive channels)
        bool channel_active = true;
        if (g_local_params.channels[param_idx].muted) channel_active = false;
        else if (solo_mode_active && !g_local_params.channels[param_idx].soloed) channel_active = false;

        if (channel_active) {
            float gain_linear = DB_to_Linear(g_local_params.channels[param_idx].digital_gain);
            float pan_l, pan_r;
            CalculatePanFactors(g_local_params.channels[param_idx].panning, &pan_l, &pan_r);

            // Apply gain and panning, then sum to master buses
            for (uint32_t frame = 0; frame < samples_per_channel; ++frame) {
                float sample = channel_proc_buffers[i][frame] * gain_linear;
                master_bus_buffer_L[frame] += sample * pan_l;
                master_bus_buffer_R[frame] += sample * pan_r;
                // master_bus_buffer_L[frame] += sample;
                // master_bus_buffer_R[frame] += sample;
            }
        }
    }


    // --- Stage 4: Master Bus Processing ---
    // Access master parameters via index 0
    const ChannelParameters* master_p = &g_local_params.channels[0];

    // // Apply Master Effects (if enabled)
    if (master_p->equalizer.enabled) {
        // Assuming EQ_Process handles stereo or call twice if needed
        EQ_Process(&master_eq_state, master_bus_buffer_L, samples_per_channel, &master_p->equalizer); // Process L
        EQ_Process(&master_eq_state, master_bus_buffer_R, samples_per_channel, &master_p->equalizer); // Process R (using same state/params?) - Adjust if EQ needs separate L/R state
    }
    // if (master_p->compressor.enabled) {
    //     // Assuming Compressor_Process handles stereo or call twice
    //     Compressor_Process(&master_comp_state, master_bus_buffer_L, samples_per_channel, &master_p->compressor); // Process L
    //     Compressor_Process(&master_comp_state, master_bus_buffer_R, samples_per_channel, &master_p->compressor); // Process R (using same state/params?) - Adjust if needed
    // }

     if (master_p->reverb.enabled) {
        // Reverb usually creates stereo output even from mono input, or processes L/R separately
        // Needs a Reverb_ProcessStereo or similar function signature
        Reverb_ProcessStereo(&master_reverb_state, master_bus_buffer_L, master_bus_buffer_R, samples_per_channel, &master_p->reverb);
    }


    // Apply Master Gain
    float master_gain_linear = DB_to_Linear(master_p->digital_gain);
    for (uint32_t frame = 0; frame < samples_per_channel; ++frame) {
        master_bus_buffer_L[frame] *= master_gain_linear;
        master_bus_buffer_R[frame] *= master_gain_linear;
    }

    // Apply Mono/Stereo Toggle
    if (!master_p->stereo) { // If stereo bool is FALSE, make mono
        for (uint32_t frame = 0; frame < samples_per_channel; ++frame) {
            float mono_sample = (master_bus_buffer_L[frame] + master_bus_buffer_R[frame]) * 0.5f;
            master_bus_buffer_L[frame] = mono_sample;
            master_bus_buffer_R[frame] = mono_sample;
        }
    }


    // --- Stage 5: Output Formatting (Float -> Int24 -> Int32 Left-Aligned) ---
    for (uint32_t frame = 0; frame < samples_per_channel; ++frame) {
        // Clip final float values (nominally +/- 1.0) before converting
        float clipped_l = ClipFloat(master_bus_buffer_L[frame], -1.0f, 1.0f);
        float clipped_r = ClipFloat(master_bus_buffer_R[frame], -1.0f, 1.0f);

        // Scale float to 24-bit integer range
        int32_t output_l_24bit = (int32_t)(clipped_l * (float)MAX_AMPLITUDE_24BIT_I);
        int32_t output_r_24bit = (int32_t)(clipped_r * (float)MAX_AMPLITUDE_24BIT_I);

        // Write to stereo TX buffer, left-shifting
        uint32_t tx_pair_start_index = frame * DSP_OUTPUT_CHANNELS;
        tx_chunk_start[tx_pair_start_index + 0] = output_l_24bit; // Left
        tx_chunk_start[tx_pair_start_index + 1] = output_r_24bit; // Right
    }

    // print out buffer for testing
    // printf("AudioDSP: Processed %d samples\r\n", samples_per_channel);
    // for (int i = 0; i < samples_per_channel; i++) {
    //     if (i % 8 == 0) {
    //         printf("Sample %d: L=%d, R=%d\r\n", i, tx_chunk_start[i * 2], tx_chunk_start[i * 2 + 1]);
    //     }
    // }
    // --- End of Processing ---
}
