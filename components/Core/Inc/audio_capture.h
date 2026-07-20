#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define I2S_BUF_LEN 512U
#define AUX_CAPTURE_BUF_LEN 128U
#define AUX_ADC_DMA_BUF_LEN (AUX_CAPTURE_BUF_LEN * 2U)

#define AUDIO_INPUT_SOURCE_I2S 0U
#define AUDIO_INPUT_SOURCE_AUX 1U

/* Select whether AUX analysis sees ADC codes or the speaker DSP stream. */
#define AUX_ANALYSIS_TAP_RAW_ADC  0U
#define AUX_ANALYSIS_TAP_POST_DSP 1U

extern uint16_t i2s_rx_buf[I2S_BUF_LEN];
extern volatile uint32_t audio_avg;
extern volatile uint32_t audio_min;
extern volatile uint32_t audio_max;
extern volatile uint32_t audio_nonzero_count;
extern volatile uint32_t audio_zero_count;
extern volatile uint32_t audio_changed_count;
extern volatile uint32_t audio_last_sample;
extern volatile uint16_t audio_debug_samples[16];
extern volatile uint32_t audio_process_count;
extern volatile uint32_t audio_start_status;
extern volatile uint32_t audio_half_count;
extern volatile uint32_t audio_full_count;
extern volatile uint8_t audio_ready;
extern volatile uint8_t audio_input_source;
extern volatile uint32_t audio_i2s_error_count;
extern volatile uint32_t audio_i2s_dma_error_code;

extern volatile uint32_t i2s_mic_gain_q8;
extern volatile int32_t i2s_mic_dc_s24;
extern volatile uint32_t i2s_mic_left_abs_avg;
extern volatile uint32_t i2s_mic_right_abs_avg;
extern volatile uint32_t i2s_mic_selected_slot;
extern volatile uint32_t i2s_mic_highpass_enable;
extern volatile uint32_t i2s_mic_highpass_peak_s16;
extern volatile uint32_t i2s_mic_lowpass_enable;
extern volatile uint32_t i2s_mic_lowpass_peak_s16;
extern volatile uint32_t i2s_mic_gate_enable;
extern volatile uint32_t i2s_mic_gate_open;
extern volatile uint32_t i2s_mic_gate_gain_q8;
extern volatile uint32_t i2s_mic_gate_detector_avg_s16;
extern volatile uint32_t i2s_mic_gate_open_peak_threshold_s16;
extern volatile uint32_t i2s_mic_gate_peak_min_avg_s16;
extern volatile uint32_t i2s_mic_gate_open_avg_threshold_s16;
extern volatile uint32_t i2s_mic_gate_close_avg_threshold_s16;
extern volatile uint32_t i2s_mic_gate_floor_gain_q16;
extern volatile uint32_t i2s_mic_gate_hold_remaining_samples;
extern volatile uint32_t i2s_mic_output_peak_s16;
extern volatile uint32_t i2s_mic_output_abs_avg_s16;
extern volatile uint32_t i2s_mic_limiter_count;
extern volatile int32_t i2s_mic_limiter_last_input;
extern volatile int16_t i2s_mic_limiter_last_output;
extern volatile uint32_t i2s_mic_processed_sample_count;
extern volatile uint32_t i2s_mic_process_cycles_last;
extern volatile uint32_t i2s_mic_process_cycles_max;
extern volatile uint32_t i2s_mic_process_cycle_budget;
extern volatile uint32_t i2s_mic_process_deadline_miss_count;
extern volatile uint32_t i2s_mic_analysis_queue_count;
extern volatile uint32_t i2s_mic_analysis_queue_max;
extern volatile uint32_t i2s_mic_analysis_drop_count;
extern volatile uint32_t i2s_mic_analysis_service_count;
extern const uint32_t i2s_mic_analysis_queue_capacity;
extern volatile uint32_t i2s_mic_analysis_source_sequence;
extern volatile uint32_t i2s_mic_analysis_last_serviced_sequence;
extern volatile uint32_t i2s_mic_analysis_discontinuity_count;

extern int32_t aux_sample_buf[AUX_CAPTURE_BUF_LEN];
extern volatile uint32_t aux_adc_raw;
extern volatile uint32_t aux_adc_avg;
extern volatile uint32_t aux_adc_min;
extern volatile uint32_t aux_adc_max;
extern volatile uint32_t aux_adc_peak;
extern volatile uint32_t aux_adc_clip_active;
extern volatile uint32_t aux_adc_clip_sample_count;
extern volatile uint32_t aux_adc_clip_block_count;
extern volatile uint32_t aux_adc_headroom_codes;
extern volatile uint32_t aux_adc_abs_avg;
extern volatile uint32_t aux_adc_signal_smooth;
extern volatile uint32_t aux_adc_bias_mv;
extern volatile uint32_t aux_adc_vref_mv;
extern volatile uint32_t aux_adc_sample_count;
extern volatile uint32_t aux_adc_block_count;
extern volatile uint32_t aux_adc_error_count;
extern volatile uint32_t aux_adc_start_status;
extern volatile uint32_t aux_adc_timer_start_status;
extern volatile uint32_t aux_adc_poll_status;
extern volatile uint32_t aux_adc_dma_mode;
extern volatile uint8_t aux_adc_dma_is_circular;
extern volatile uint32_t aux_adc_hal_state;
extern volatile uint32_t aux_adc_error_code;
extern volatile uint32_t aux_adc_dma_state;
extern volatile uint32_t aux_adc_dma_error_code;
extern volatile uint32_t aux_adc_dma_ndtr;
extern volatile uint32_t aux_adc_timer_counter;
extern volatile uint32_t aux_adc_timer_cr1;
extern volatile uint32_t aux_adc_timer_sr;
extern volatile uint32_t aux_adc_half_count;
extern volatile uint32_t aux_adc_full_count;
extern volatile uint32_t aux_adc_pending_count;
extern volatile uint32_t aux_adc_service_count;
extern volatile uint32_t aux_adc_overrun_count;
extern volatile uint32_t aux_analysis_queue_count;
extern volatile uint32_t aux_analysis_queue_max;
extern volatile uint32_t aux_analysis_drop_count;
extern volatile uint32_t aux_analysis_service_count;
extern const uint32_t aux_analysis_queue_capacity;
extern volatile uint32_t aux_analysis_source_sequence;
extern volatile uint32_t aux_analysis_last_serviced_sequence;
extern volatile uint32_t aux_analysis_discontinuity_count;
extern volatile uint32_t aux_analysis_tap;
extern volatile uint32_t aux_analysis_last_serviced_tap;
extern volatile uint32_t aux_analysis_raw_origin_adc_q8;
extern volatile uint32_t aux_analysis_raw_origin_valid;
extern volatile uint32_t aux_output_realtime_enable;
extern volatile uint32_t aux_output_realtime_push_count;
extern volatile uint32_t aux_output_realtime_peak;
extern volatile uint32_t aux_output_realtime_avg;
extern volatile uint32_t aux_output_realtime_min;
extern volatile uint32_t aux_output_realtime_max;
extern volatile uint32_t aux_output_realtime_abs_avg;
extern volatile uint32_t aux_output_realtime_bias_mv;
extern volatile uint32_t aux_output_process_cycles_last;
extern volatile uint32_t aux_output_process_cycles_max;
extern volatile uint32_t aux_output_process_cycle_budget;
extern volatile uint32_t aux_output_process_deadline_miss_count;
extern volatile uint32_t aux_output_gain_q8;
extern volatile uint32_t aux_output_bias_adc;
extern volatile uint32_t aux_output_peak_s16;
extern volatile uint32_t aux_output_abs_avg_s16;
extern volatile uint32_t aux_output_limiter_count;
extern volatile int32_t aux_output_limiter_last_input;
extern volatile int16_t aux_output_limiter_last_output;
extern volatile int16_t aux_output_last_sample_s16;
extern volatile uint32_t aux_output_gate_enable;
extern volatile uint32_t aux_output_gate_open;
extern volatile uint32_t aux_output_gate_gain_q8;
extern volatile uint32_t aux_output_gate_detector_avg_s16;
extern volatile uint32_t aux_output_gate_open_peak_threshold_s16;
extern volatile uint32_t aux_output_gate_peak_min_avg_s16;
extern volatile uint32_t aux_output_gate_open_avg_threshold_s16;
extern volatile uint32_t aux_output_gate_close_avg_threshold_s16;
extern volatile uint32_t aux_output_gate_hold_remaining_samples;
extern volatile uint32_t aux_output_gate_peak_qualified;
extern volatile uint32_t aux_output_gate_rejected_peak_count;
extern volatile uint32_t aux_output_gate_open_count;
extern volatile uint32_t aux_output_gate_close_count;
extern volatile uint32_t aux_output_highpass_enable;
extern const uint32_t aux_output_highpass_cutoff_hz;
extern volatile uint32_t aux_output_highpass_peak_s16;
extern volatile uint32_t aux_output_lowpass_enable;
extern const uint32_t aux_output_lowpass_cutoff_hz;
extern volatile uint32_t aux_output_lowpass_peak_s16;
extern volatile uint16_t aux_adc_debug_samples[16];
extern volatile uint8_t aux_ready;
extern uint16_t aux_adc_dma_buf[AUX_ADC_DMA_BUF_LEN];

HAL_StatusTypeDef AudioCapture_Start(I2S_HandleTypeDef *hi2s);
void AudioCapture_ProcessBlock(uint16_t *buf, uint32_t len);
void AudioCapture_Service(void);
HAL_StatusTypeDef AuxCapture_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef AuxCapture_StartDma(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);
void AuxCapture_Poll(void);
void AuxCapture_Service(void);
void AuxCapture_UpdateDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_CAPTURE_H */
