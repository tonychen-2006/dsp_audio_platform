#ifndef AUDIO_SAMPLES_H
#define AUDIO_SAMPLES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "audio_capture.h"

#define AUDIO_SAMPLE_BUF_LEN (I2S_BUF_LEN / 4U)

extern int32_t audio_sample_buf[AUDIO_SAMPLE_BUF_LEN];
extern volatile uint32_t audio_sample_count;
extern volatile int32_t audio_sample_avg;
extern volatile int32_t audio_sample_min;
extern volatile int32_t audio_sample_max;
extern volatile uint32_t audio_sample_abs_avg;
extern volatile uint32_t audio_sample_peak;
extern volatile uint32_t audio_sample_level_smooth;
extern volatile uint32_t audio_sample_noise_floor;
extern volatile uint32_t audio_sample_signal_level;
extern volatile uint32_t audio_sample_signal_smooth;
extern volatile uint32_t audio_sample_left_abs_avg;
extern volatile uint32_t audio_sample_right_abs_avg;
extern volatile uint8_t audio_sample_active_slot;
extern volatile uint32_t audio_sample_process_count;
extern volatile uint32_t audio_sample_debug_raw_left;
extern volatile uint32_t audio_sample_debug_raw_right;

uint32_t AudioSamples_UnpackI2s32(const uint16_t *raw_buf, uint32_t raw_len);
void AudioSamples_UpdateFromS32(const int32_t *samples, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_SAMPLES_H */
