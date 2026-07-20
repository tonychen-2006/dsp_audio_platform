#ifndef AUDIO_FFT_H
#define AUDIO_FFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define AUDIO_FFT_SIZE         1024U
#define AUDIO_FFT_DISPLAY_BINS 16U
/* At 32 ksample/s this spans 100 Hz to the 16 kHz Nyquist boundary. */
#define AUDIO_FFT_MIN_FREQ_HZ  100U
#define AUDIO_FFT_MAX_FREQ_HZ  16000U
/* Non-overlap retains 31.25 Hz resolution and is already faster than the UI. */
#define AUDIO_FFT_HOP_SIZE     AUDIO_FFT_SIZE

#define AUDIO_FFT_VIEW_MODE_FULL     0U
#define AUDIO_FFT_VIEW_MODE_KHZ_ZOOM 1U
#define AUDIO_FFT_AXIS_TICK_COUNT    5U

extern volatile uint32_t audio_fft_init_status;
extern volatile uint32_t audio_fft_frame_count;
extern volatile uint32_t audio_fft_process_count;
extern volatile uint32_t audio_fft_drop_count;
extern volatile uint32_t audio_fft_stream_reset_count;
extern volatile uint32_t audio_fft_stream_reset_discarded_ready_count;
extern volatile uint32_t audio_fft_stream_reset_last_collect_index;
extern volatile uint32_t audio_fft_ready;
extern volatile uint32_t audio_fft_collect_index;
extern volatile uint32_t audio_fft_peak_bin;
extern volatile uint32_t audio_fft_peak_fft_bin;
extern volatile uint32_t audio_fft_peak_freq_hz;
extern volatile uint32_t audio_fft_peak_value;
extern volatile int32_t audio_fft_peak_db_x10;
extern volatile uint32_t audio_fft_peak_prominence_db_x10;
extern volatile int32_t audio_fft_floor_db;
extern volatile uint32_t audio_fft_bin_values[AUDIO_FFT_DISPLAY_BINS];
extern volatile uint32_t audio_fft_adaptive_enable;
extern volatile int32_t audio_fft_adaptive_min_peak_db_x10;
extern volatile uint32_t audio_fft_adaptive_min_prominence_db_x10;
extern volatile uint32_t audio_fft_adaptive_confirm_frames;
extern volatile uint32_t audio_fft_view_mode;
extern volatile uint32_t audio_fft_view_bucket_khz;
extern volatile uint32_t audio_fft_view_min_hz;
extern volatile uint32_t audio_fft_view_max_hz;
extern volatile uint32_t audio_fft_view_bin_width_hz;
extern volatile uint32_t audio_fft_view_change_count;
extern volatile uint32_t audio_fft_view_candidate_bucket_khz;
extern volatile uint32_t audio_fft_view_candidate_count;
extern volatile uint32_t audio_fft_view_peak_qualified;
extern volatile uint32_t audio_fft_global_peak_in_view;
extern volatile uint32_t audio_fft_band_edge_hz[AUDIO_FFT_DISPLAY_BINS + 1U];
extern volatile uint32_t audio_fft_axis_tick_count;
extern volatile uint32_t audio_fft_axis_tick_hz[AUDIO_FFT_AXIS_TICK_COUNT];
extern volatile uint32_t audio_fft_process_cycles_last;
extern volatile uint32_t audio_fft_process_cycles_max;
extern volatile uint32_t audio_fft_process_cycle_budget;
extern volatile uint32_t audio_fft_process_deadline_miss_count;

HAL_StatusTypeDef AudioFFT_Init(void);
void AudioFFT_ResetStream(void);
void AudioFFT_PushSamplesS32(const int32_t *samples, uint32_t len);
uint8_t AudioFFT_ProcessIfReady(void);
uint8_t AudioFFT_GetDisplayBins(uint16_t *bins, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_FFT_H */
