/**
 * @file audio_capture.h
 * @brief Audio-input capture, AUX reconstruction, and analysis interfaces.
 */

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

/** @brief Select raw ADC codes as the AUX analysis source. */
#define AUX_ANALYSIS_TAP_RAW_ADC  0U

/** @brief Select reconstructed post-DSP samples as the AUX analysis source. */
#define AUX_ANALYSIS_TAP_POST_DSP 1U

/** @name Optional I2S microphone buffers, controls, and diagnostics. */
/** @{ */
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
/** @} */

/**
 * @brief AUX ADC measurements, capture counters, and peripheral health.
 */
typedef struct
{
  /* Latest polled/legacy block; realtime DMA levels live in aux_output_diag. */
  uint32_t raw;          /**< Latest ADC code from the polled path. */
  uint32_t avg;          /**< Mean ADC code in the latest legacy block. */
  uint32_t min;          /**< Minimum ADC code in the latest legacy block. */
  uint32_t max;          /**< Maximum ADC code in the latest legacy block. */
  uint32_t peak;         /**< Peak centered magnitude in ADC codes. */
  uint32_t abs_avg;      /**< Mean centered magnitude in ADC codes. */
  uint32_t sample_count; /**< Total samples processed by the legacy path. */
  uint32_t block_count;  /**< Total blocks processed by the legacy path. */

  /* Realtime ADC rail-clipping and analog-headroom measurements. */
  uint32_t clip_active;       /**< Nonzero when the latest block neared a rail. */
  uint32_t clip_sample_count; /**< Total near-rail ADC samples. */
  uint32_t clip_block_count;  /**< Total blocks containing near-rail samples. */
  uint32_t headroom_codes;    /**< Latest minimum distance from either ADC rail. */

  /* ADC, DMA, and sample-timer health. */
  uint32_t error_count;        /**< Total AUX ADC or timer start/poll errors. */
  uint32_t start_status;       /**< HAL status returned when ADC capture started. */
  uint32_t timer_start_status; /**< HAL status returned when the sample timer started. */
  uint32_t poll_status;        /**< Most recent polled-conversion HAL status. */
  uint32_t dma_mode;           /**< Configured HAL DMA mode. */
  uint32_t hal_state;          /**< Current HAL ADC state bitmask. */
  uint32_t error_code;         /**< Current HAL ADC error code. */
  uint32_t dma_state;          /**< Current HAL DMA state. */
  uint32_t dma_error_code;     /**< Current HAL DMA error code. */
  uint32_t dma_ndtr;           /**< Remaining DMA transfers. */
  uint32_t timer_counter;      /**< Current sample-timer counter value. */
  uint32_t timer_cr1;          /**< Snapshot of the sample-timer CR1 register. */
  uint32_t timer_sr;           /**< Snapshot of the sample-timer status register. */

  /* DMA callback, foreground-service, and throughput counters. */
  uint32_t half_count;      /**< Total DMA half-complete callbacks. */
  uint32_t full_count;      /**< Total DMA complete callbacks. */
  uint32_t pending_count;   /**< Legacy staging blocks awaiting service. */
  uint32_t service_count;   /**< Legacy staging blocks serviced. */
  uint32_t overrun_count;   /**< Legacy staging blocks overwritten or invalidated. */
  uint32_t blocks_per_sec;  /**< Measured DMA block rate. */
  uint32_t samples_per_sec; /**< Measured ADC sample rate. */
  uint8_t dma_is_circular;  /**< Nonzero when DMA is configured as circular. */
  uint8_t ready;            /**< Nonzero after a complete legacy block is processed. */
  uint8_t start_failed;     /**< Nonzero when application-level capture startup failed. */
} AuxAdcDiagnostics;

/**
 * @brief Deferred AUX analysis queue and stream-continuity diagnostics.
 */
typedef struct
{
  /* Queue occupancy and loss. */
  uint32_t queue_count;      /**< Analysis blocks currently queued. */
  uint32_t queue_max;        /**< Maximum observed queue occupancy. */
  uint32_t queue_capacity;   /**< Usable analysis queue capacity. */
  uint32_t drop_count;       /**< Blocks dropped because the queue was full. */
  uint32_t service_count;    /**< Blocks delivered to analysis consumers. */
  uint32_t source_sequence;  /**< Sequence number assigned to the newest source block. */
  uint32_t last_serviced_sequence; /**< Sequence number most recently consumed. */
  uint32_t discontinuity_count;    /**< Detected source-sequence discontinuities. */

  /* Selected source tap and the last block delivered to analysis. */
  uint32_t tap;               /**< Requested AUX_ANALYSIS_TAP_* source. */
  uint32_t last_serviced_tap; /**< Source tap of the last consumed block. */
  uint32_t raw_origin_adc_q8; /**< Continuous raw-ADC centering origin in Q8. */
  uint32_t raw_origin_valid;  /**< Nonzero when the raw centering origin is valid. */
  uint32_t blocks_per_sec;    /**< Measured analysis service rate. */
} AuxAnalysisDiagnostics;

/**
 * @brief Runtime controls for the realtime AUX-to-speaker path.
 * @note gain_q8 uses 256 = 1.0x; enable fields use 0 = off and 1 = on.
 */
typedef struct
{
  uint32_t realtime_enable; /**< Enables DMA-to-speaker reconstruction. */
  uint32_t gain_q8;         /**< Linear reconstruction gain in Q8. */
  uint32_t gate_enable;     /**< Enables the optional noise gate. */
  uint32_t gate_open_peak_threshold_s16; /**< Peak threshold that can open the gate. */
  uint32_t gate_peak_min_avg_s16; /**< Minimum block average qualifying a peak. */
  uint32_t gate_open_avg_threshold_s16; /**< Average-level gate-open threshold. */
  uint32_t gate_close_avg_threshold_s16; /**< Average-level gate-close threshold. */
  uint32_t highpass_enable; /**< Enables the fixed high-pass filter. */
  uint32_t lowpass_enable;  /**< Enables the fixed low-pass filter. */
} AuxOutputControls;

/**
 * @brief Realtime AUX signal, gate, filter, and deadline diagnostics.
 */
typedef struct
{
  uint32_t realtime_push_count; /**< Samples pushed into the speaker ring. */
  uint32_t realtime_peak;       /**< Latest centered ADC peak in codes. */
  uint32_t realtime_avg;        /**< Latest mean raw ADC code. */
  uint32_t realtime_min;        /**< Latest minimum raw ADC code. */
  uint32_t realtime_max;        /**< Latest maximum raw ADC code. */
  uint32_t realtime_abs_avg;    /**< Latest mean centered magnitude in codes. */
  uint32_t process_cycles_max;  /**< Maximum measured realtime processing cycles. */
  uint32_t process_cycle_budget; /**< Cycle budget for one DMA block. */
  uint32_t process_deadline_miss_count; /**< Blocks exceeding the cycle budget. */
  uint32_t bias_adc;             /**< Tracked ADC DC bias in codes. */
  uint32_t peak_s16;             /**< Latest post-gate S16 peak. */
  uint32_t abs_avg_s16;          /**< Latest post-gate S16 mean magnitude. */
  uint32_t limiter_count;        /**< Samples processed by the soft limiter. */
  uint32_t gate_open;            /**< Nonzero while the optional gate is open. */
  uint32_t gate_gain_q8;         /**< Current gate gain in Q8. */
  uint32_t gate_detector_avg_s16; /**< Latest pre-gate mean magnitude. */
  uint32_t gate_hold_remaining_samples; /**< Samples remaining in gate hold time. */
  uint32_t gate_peak_qualified;  /**< Nonzero when a peak passed qualification. */
  uint32_t gate_rejected_peak_count; /**< Isolated peaks rejected by qualification. */
  uint32_t gate_open_count;      /**< Closed-to-open gate transitions. */
  uint32_t gate_close_count;     /**< Open-to-closed gate transitions. */
  uint32_t highpass_cutoff_hz;   /**< Applied high-pass cutoff frequency. */
  uint32_t highpass_peak_s16;    /**< Latest post-high-pass peak. */
  uint32_t lowpass_cutoff_hz;    /**< Applied low-pass cutoff frequency. */
  uint32_t lowpass_peak_s16;     /**< Latest post-low-pass peak. */
  uint32_t samples_per_sec;      /**< Measured reconstructed sample rate. */
} AuxOutputDiagnostics;

/** @brief AUX ADC and peripheral diagnostic state. */
extern volatile AuxAdcDiagnostics aux_adc_diag;

/** @brief AUX deferred-analysis queue state. */
extern volatile AuxAnalysisDiagnostics aux_analysis_diag;

/** @brief Live controls for realtime AUX reconstruction. */
extern volatile AuxOutputControls aux_output_controls;

/** @brief AUX reconstruction and output diagnostic state. */
extern volatile AuxOutputDiagnostics aux_output_diag;

/**
 * @brief Start optional I2S microphone capture using circular DMA.
 * @param hi2s Configured I2S receive handle.
 * @return HAL_OK on success; otherwise the HAL startup error.
 */
HAL_StatusTypeDef AudioCapture_Start(I2S_HandleTypeDef *hi2s);

/**
 * @brief Process one completed I2S receive block.
 * @param buf Pointer to received I2S DMA half-words.
 * @param len Number of half-words available in @p buf.
 */
void AudioCapture_ProcessBlock(uint16_t *buf, uint32_t len);

/** @brief Service deferred I2S microphone analysis work. */
void AudioCapture_Service(void);

/**
 * @brief Initialize the polled AUX capture fallback.
 * @param hadc Configured ADC handle.
 * @return HAL_OK when the handle is accepted; otherwise HAL_ERROR.
 */
HAL_StatusTypeDef AuxCapture_Start(ADC_HandleTypeDef *hadc);

/**
 * @brief Start realtime AUX capture using timer-triggered circular ADC DMA.
 *
 * DMA callbacks reconstruct the speaker stream and enqueue a selected analysis
 * tap. AuxCapture_Service() performs the deferred FFT/display preparation.
 *
 * @param hadc Configured ADC handle with an attached DMA handle.
 * @param htim Timer providing the ADC sample clock through TRGO.
 * @return HAL_OK on success; otherwise the ADC or timer HAL startup error.
 */
HAL_StatusTypeDef AuxCapture_StartDma(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);

/** @brief Acquire one sample for the polled AUX fallback. */
void AuxCapture_Poll(void);

/**
 * @brief Service deferred AUX analysis and any pending legacy capture block.
 */
void AuxCapture_Service(void);

/** @brief Refresh ADC, DMA, and sample-timer register diagnostics. */
void AuxCapture_UpdateDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_CAPTURE_H */
