#ifndef AUDIO_FFT_H
#define AUDIO_FFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define AUDIO_FFT_SIZE         1024U
#define AUDIO_FFT_DISPLAY_BINS 16U

extern volatile uint32_t audio_fft_init_status;
extern volatile uint32_t audio_fft_frame_count;
extern volatile uint32_t audio_fft_process_count;
extern volatile uint32_t audio_fft_drop_count;
extern volatile uint32_t audio_fft_ready;
extern volatile uint32_t audio_fft_collect_index;
extern volatile uint32_t audio_fft_peak_bin;
extern volatile uint32_t audio_fft_peak_fft_bin;
extern volatile uint32_t audio_fft_peak_freq_hz;
extern volatile uint32_t audio_fft_peak_value;
extern volatile uint32_t audio_fft_bin_values[AUDIO_FFT_DISPLAY_BINS];

HAL_StatusTypeDef AudioFFT_Init(void);
void AudioFFT_PushSamplesS32(const int32_t *samples, uint32_t len);
uint8_t AudioFFT_ProcessIfReady(void);
uint8_t AudioFFT_GetDisplayBins(uint16_t *bins, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_FFT_H */
