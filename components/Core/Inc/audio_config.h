#ifndef AUDIO_CONFIG_H
#define AUDIO_CONFIG_H

/*
 * Every active audio source and the PCM5102 output run at this rate.
 * Keep the matching CubeMX I2S2, I2S3 and TIM2 settings at 32 kHz.
 */
#define AUDIO_STREAM_SAMPLE_RATE_HZ 32000U

/* Exposed in audio_debug_config so a screenshot identifies the flashed build. */
#define AUDIO_FIRMWARE_BUILD_ID 2026072303U

#if defined(__OPTIMIZE__)
#define AUDIO_COMPILER_OPTIMIZED 1U
#else
#define AUDIO_COMPILER_OPTIMIZED 0U
#endif

#if (AUDIO_STREAM_SAMPLE_RATE_HZ != 32000U)
#error "Current FFT labels and HP/LP coefficients require a 32 kHz stream"
#endif

#endif /* AUDIO_CONFIG_H */
