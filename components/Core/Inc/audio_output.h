#ifndef AUDIO_OUTPUT_H
#define AUDIO_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define AUDIO_OUT_BUF_LEN 512U

typedef enum
{
  AUDIO_OUTPUT_MODE_SILENCE = 0,
  AUDIO_OUTPUT_MODE_TEST_TONE,
  AUDIO_OUTPUT_MODE_MIC_MONITOR
} AudioOutput_Mode_t;

extern uint16_t audio_tx_buf[AUDIO_OUT_BUF_LEN];
extern volatile uint32_t audio_out_start_status;
extern volatile uint32_t audio_out_i2s_state_before_start;
extern volatile uint32_t audio_out_i2s_state_after_start;
extern volatile uint32_t audio_out_i2s_error_code;
extern volatile uint32_t audio_out_dma_state_before_start;
extern volatile uint32_t audio_out_dma_state_after_start;
extern volatile uint32_t audio_out_dma_error_code;
extern volatile uint32_t audio_out_i2s_runtime_error_count;
extern volatile uint32_t audio_out_hdmatx_is_null;
extern volatile uint32_t audio_out_half_count;
extern volatile uint32_t audio_out_full_count;
extern volatile uint32_t audio_out_fill_count;
extern volatile uint32_t audio_out_last_callback_tick_ms;
extern volatile uint32_t audio_out_last_sample;
extern volatile int16_t audio_out_last_sample_s16;
extern volatile uint32_t audio_out_tx_peak;
extern volatile uint32_t audio_out_tx_abs_avg;
extern volatile uint32_t audio_out_tx_level_smooth;
extern volatile uint32_t audio_out_tx_debug_update_count;
extern volatile uint32_t audio_out_tx_zero_block_count;
extern volatile uint32_t audio_out_tx_nonzero_block_count;
extern volatile int16_t audio_out_tx_debug_samples[16];
extern volatile uint8_t audio_out_force_test_tone;
extern volatile uint32_t audio_out_mode_debug;
extern volatile uint32_t audio_out_test_tone_divider;
extern volatile uint32_t audio_out_test_tone_frequency_hz;
extern volatile uint32_t audio_out_test_tone_frequency_applied_hz;
extern volatile uint32_t audio_out_test_tone_phase_step_q16;
extern volatile uint32_t audio_out_block_len_debug;
extern volatile uint32_t audio_out_dma_size_debug;
extern volatile uint32_t audio_out_mic_push_count;
extern volatile uint32_t audio_out_mic_pop_count;
extern volatile uint32_t audio_out_mic_underrun_count;
extern volatile uint32_t audio_out_mic_overrun_count;
extern volatile uint32_t audio_out_mic_available;
extern volatile uint32_t audio_out_mic_peak;
extern volatile uint32_t audio_out_mic_prefill_threshold;
extern volatile uint32_t audio_out_mic_wait_count;
extern volatile uint8_t audio_out_mic_streaming;
extern volatile uint32_t audio_out_mic_available_min;
extern volatile uint32_t audio_out_mic_available_max;
extern volatile uint32_t audio_out_mic_gain_q8;
extern volatile uint32_t audio_out_mic_repeat_factor;
extern volatile uint32_t audio_out_mic_hold_count;
extern volatile int16_t audio_out_mic_last_sample_s16;
extern volatile uint32_t audio_out_limiter_count;
extern volatile int32_t audio_out_limiter_last_input;
extern volatile int16_t audio_out_limiter_last_output;
extern volatile uint8_t audio_out_ready;
extern volatile uint32_t audio_pwm_out_start_status;
extern volatile uint32_t audio_pwm_out_pwm_start_status;
extern volatile uint32_t audio_pwm_out_timer_start_status;
extern volatile uint32_t audio_pwm_out_sample_count;
extern volatile uint32_t audio_pwm_out_samples_per_sec;
extern volatile uint32_t audio_pwm_out_last_duty;
extern volatile uint32_t audio_pwm_out_peak;
extern volatile uint32_t audio_pwm_out_last_peak;
extern volatile uint32_t audio_pwm_out_level_smooth;
extern volatile uint32_t audio_pwm_out_clip_count;
extern volatile int16_t audio_pwm_out_last_sample_s16;
extern volatile uint32_t audio_pwm_out_noise_shape_enable;
extern volatile int32_t audio_pwm_out_quant_error_q16;
extern volatile uint8_t audio_pwm_out_ready;

HAL_StatusTypeDef AudioOutput_Start(I2S_HandleTypeDef *hi2s);
void AudioOutput_SetMode(AudioOutput_Mode_t mode);
void AudioOutput_FillBlock(uint16_t *buf, uint32_t len);
void AudioOutput_PushSamplesS32(const int32_t *samples, uint32_t count);
void AudioOutput_PushSamplesS16(const int16_t *samples, uint32_t count);
void AudioOutput_HandleI2sError(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef AudioPwmOutput_Start(TIM_HandleTypeDef *pwm_htim,
                                       uint32_t pwm_channel,
                                       TIM_HandleTypeDef *sample_htim);
void AudioPwmOutput_HandleSampleTimer(TIM_HandleTypeDef *htim);
uint8_t AudioPwmOutput_HandleSampleTimerIrq(TIM_HandleTypeDef *htim);
void AudioPwmOutput_UpdateDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_OUTPUT_H */
