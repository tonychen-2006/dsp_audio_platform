/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audio_config.h"
#include "audio_capture.h"
#include "audio_fft.h"
#include "audio_output.h"
#include "audio_samples.h"
#include "audio_visualizer.h"
#include "display_st7789.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint32_t snapshot_sequence_begin;
  uint32_t tick_ms;
  uint32_t firmware_build_id;
  uint32_t compiler_optimized;
  uint32_t input_source;
  uint32_t output_mode;
  uint32_t force_test_tone;
  uint32_t aux_realtime_enabled;
  uint32_t highpass_enabled;
  uint32_t lowpass_enabled;
  uint32_t gate_enabled;
  uint32_t aux_samples_per_sec;
  uint32_t analysis_block_samples;
  uint32_t analysis_expected_blocks_per_sec;
  uint32_t analysis_blocks_per_sec;
  uint32_t analysis_queue_blocks;
  uint32_t analysis_queue_capacity_blocks;
  uint32_t analysis_queue_lag_blocks;
  uint32_t analysis_latency_ms;
  uint32_t analysis_queue_before_service_blocks;
  uint32_t analysis_queue_after_service_blocks;
  uint32_t analysis_catchup_active;
  uint32_t analysis_catchup_events_total;
  uint32_t analysis_service_passes_last;
  uint32_t analysis_service_passes_max;
  uint32_t analysis_service_passes_total;
  uint32_t analysis_service_cycles_last;
  uint32_t analysis_service_cycles_max;
  uint32_t visualizer_updates_per_sec;
  uint32_t analysis_low_water_blocks;
  uint32_t analysis_catchup_start_blocks;
  uint32_t analysis_catchup_max_passes;
  uint32_t fft_frames_per_sec;
  uint32_t tx_updates_per_sec;
  uint32_t display_draws_per_sec;
  uint32_t display_period_ms;
  uint32_t display_ready;
  uint32_t display_waveform_enabled;
  uint32_t display_frame_ready;
  uint32_t display_last_draw_age_ms;
  uint32_t display_draw_duration_last_ms;
  uint32_t display_draw_duration_max_ms;
  uint32_t display_defer_events_total;
  uint32_t display_queue_before_draw_blocks;
  uint32_t display_queue_after_draw_blocks;
  uint32_t display_spi_blocking_tx_total;
  uint32_t scope_configured_mode;
  uint32_t scope_effective_mode;
  uint32_t scope_sample_rate_hz;
  uint32_t scope_sample_count;
  uint32_t scope_sample_period_ns;
  uint32_t scope_window_us;
  uint32_t scope_pretrigger_samples;
  uint32_t scope_trigger_hysteresis_s24;
  uint32_t scope_trigger_hysteresis_adc_codes;
  uint32_t scope_trigger_count;
  uint32_t scope_auto_trigger_count;
  uint32_t scope_stream_reset_count;
  uint32_t scope_snapshot_sequence;
  uint32_t scope_drawn_sequence;
  uint32_t scope_snapshot_ready;
  uint32_t scope_capture_active;
  uint32_t scope_trigger_armed;
  int32_t scope_snapshot_min_s24;
  int32_t scope_snapshot_max_s24;
  int32_t scope_snapshot_mean_s24;
  uint32_t scope_snapshot_peak_s24;
  uint32_t scope_snapshot_scale_s24;
  uint32_t scope_snapshot_vpp_s24;
  uint32_t scope_snapshot_vpp_adc_codes;
  uint32_t snapshot_sequence_end;
} AudioDebugConfigView;

typedef struct
{
  uint32_t snapshot_sequence_begin;
  uint32_t tick_ms;
  uint32_t source_samples_total;
  uint32_t analysis_process_count;
  uint32_t raw_avg_adc_codes;
  uint32_t raw_min_adc_codes;
  uint32_t raw_max_adc_codes;
  uint32_t raw_peak_adc_codes;
  uint32_t raw_abs_avg_adc_codes;
  uint32_t raw_bias_mv;
  uint32_t tracked_bias_adc_codes;
  uint32_t headroom_adc_codes;
  uint32_t adc_clip_active;
  uint32_t output_gain_q8;
  uint32_t highpass_cutoff_hz;
  uint32_t highpass_peak_s16;
  uint32_t lowpass_cutoff_hz;
  uint32_t lowpass_peak_s16;
  uint32_t gate_detector_avg_s16;
  uint32_t gate_peak_qualified;
  uint32_t gate_open;
  uint32_t gate_gain_q8;
  uint32_t gate_hold_samples;
  uint32_t gate_open_peak_threshold_s16;
  uint32_t gate_peak_min_avg_s16;
  uint32_t gate_open_avg_threshold_s16;
  uint32_t gate_close_avg_threshold_s16;
  uint32_t post_gate_peak_s16;
  uint32_t post_gate_abs_avg_s16;
  uint32_t analysis_peak_s16;
  uint32_t analysis_abs_avg_s16;
  uint32_t analysis_noise_floor_s16;
  uint32_t analysis_signal_s16;
  uint32_t analysis_noise_floor_frozen;
  uint32_t analysis_noise_floor_freeze_count;
  uint32_t analysis_selected_tap;
  uint32_t analysis_last_serviced_tap;
  uint32_t analysis_raw_origin_adc_q8;
  uint32_t analysis_raw_origin_adc_codes;
  uint32_t analysis_raw_origin_valid;
  uint32_t analysis_queue_blocks;
  uint32_t analysis_queue_max_blocks;
  uint32_t analysis_queue_capacity_blocks;
  uint32_t analysis_blocks_per_sec;
  uint32_t analysis_source_sequence;
  uint32_t analysis_last_serviced_sequence;
  uint32_t counter_window_ms;
  uint32_t adc_clip_samples_total;
  uint32_t adc_clip_samples_delta;
  uint32_t adc_clip_blocks_total;
  uint32_t adc_clip_blocks_delta;
  uint32_t limiter_samples_total;
  uint32_t limiter_samples_delta;
  uint32_t adc_overrun_blocks_total;
  uint32_t adc_overrun_blocks_delta;
  uint32_t analysis_drop_blocks_total;
  uint32_t analysis_drop_blocks_delta;
  uint32_t analysis_discontinuities_total;
  uint32_t analysis_discontinuities_delta;
  uint32_t gate_rejected_peaks_total;
  uint32_t gate_rejected_peaks_delta;
  uint32_t gate_open_events_total;
  uint32_t gate_open_events_delta;
  uint32_t gate_close_events_total;
  uint32_t gate_close_events_delta;
  uint32_t output_process_cycles_last;
  uint32_t output_process_cycles_max;
  uint32_t output_process_cycle_budget;
  uint32_t deadline_miss_blocks_total;
  uint32_t deadline_miss_blocks_delta;
  uint32_t snapshot_sequence_end;
} AudioDebugAuxView;

typedef struct
{
  uint32_t snapshot_sequence_begin;
  uint32_t tick_ms;
  uint32_t analysis_process_count;
  uint32_t analysis_peak_s16;
  uint32_t analysis_abs_avg_s16;
  uint32_t analysis_noise_floor_s16;
  uint32_t analysis_signal_s16;
  uint32_t analysis_noise_floor_frozen;
  uint32_t analysis_noise_floor_freeze_count;
  uint32_t sample_rate_hz;
  uint32_t nyquist_boundary_hz;
  uint32_t analysis_min_hz;
  uint32_t analysis_max_boundary_hz;
  uint32_t highest_analyzed_bin_hz;
  uint32_t frame_count;
  uint32_t frame_hop_samples;
  uint32_t expected_frames_per_sec_x10;
  uint32_t process_count;
  uint32_t ready;
  uint32_t collect_index;
  uint32_t peak_fft_bin;
  uint32_t peak_frequency_hz;
  uint32_t peak_display_band;
  uint32_t peak_level_ppm_fs;
  int32_t peak_dbfs_x10;
  uint32_t peak_prominence_db_x10;
  int32_t floor_dbfs;
  uint32_t adaptive_enabled;
  int32_t adaptive_min_peak_dbfs_x10;
  uint32_t adaptive_min_prominence_db_x10;
  uint32_t adaptive_confirm_frames;
  uint32_t adaptive_peak_qualified;
  uint32_t global_peak_in_view;
  uint32_t view_mode;
  uint32_t view_bucket_khz;
  uint32_t view_min_hz;
  uint32_t view_max_hz;
  uint32_t view_bin_width_hz;
  uint32_t view_change_count;
  uint32_t view_candidate_bucket_khz;
  uint32_t view_candidate_count;
  uint32_t display_axis_change_drawn;
  uint32_t axis_tick_count;
  uint32_t axis_tick_hz[AUDIO_FFT_AXIS_TICK_COUNT];
  uint32_t band_edge_hz[AUDIO_FFT_DISPLAY_BINS + 1U];
  uint32_t band_level_permille[AUDIO_FFT_DISPLAY_BINS];
  uint16_t band_height_pixels[AUDIO_FFT_DISPLAY_BINS];
  uint32_t fft_draw_count;
  uint32_t fft_empty_count;
  uint32_t display_draw_errors_total;
  uint32_t display_draw_error_stage;
  uint32_t counter_window_ms;
  uint32_t dropped_frames_total;
  uint32_t dropped_frames_delta;
  uint32_t source_discontinuities_total;
  uint32_t source_discontinuities_delta;
  uint32_t stream_resets_total;
  uint32_t stream_resets_delta;
  uint32_t stream_reset_discarded_ready_total;
  uint32_t stream_reset_last_collect_index;
  uint32_t process_cycles_last;
  uint32_t process_cycles_max;
  uint32_t process_cycle_budget;
  uint32_t deadline_miss_frames_total;
  uint32_t deadline_miss_frames_delta;
  uint32_t snapshot_sequence_end;
} AudioDebugFftView;

typedef struct
{
  uint32_t snapshot_sequence_begin;
  uint32_t tick_ms;
  uint32_t input_source;
  uint32_t output_mode;
  uint32_t force_test_tone;
  uint32_t streaming;
  uint32_t tx_peak_s16;
  uint32_t tx_abs_avg_s16;
  uint32_t tx_level_smooth_s16;
  uint32_t tx_debug_updates_total;
  uint32_t tx_half_callbacks_total;
  uint32_t tx_full_callbacks_total;
  uint32_t tx_fill_count_total;
  uint32_t ring_available_samples;
  uint32_t ring_available_min_samples;
  uint32_t ring_available_max_samples;
  uint32_t ring_push_samples_total;
  uint32_t ring_pop_samples_total;
  uint32_t analysis_queue_source;
  uint32_t analysis_queue_blocks;
  uint32_t analysis_queue_max_blocks;
  uint32_t analysis_queue_capacity_blocks;
  uint32_t analysis_source_sequence;
  uint32_t analysis_last_serviced_sequence;
  uint32_t analysis_drop_blocks_total;
  uint32_t analysis_drop_blocks_delta;
  uint32_t analysis_discontinuities_total;
  uint32_t analysis_discontinuities_delta;
  uint32_t mic_processed_samples_total;
  uint32_t mic_output_peak_s16;
  uint32_t mic_output_abs_avg_s16;
  uint32_t mic_gate_open;
  uint32_t mic_selected_slot;
  uint32_t mic_process_cycles_last;
  uint32_t mic_process_cycles_max;
  uint32_t mic_process_cycle_budget;
  uint32_t mic_deadline_miss_blocks_total;
  uint32_t mic_deadline_miss_blocks_delta;
  uint32_t start_status;
  uint32_t i2s_error_code;
  uint32_t dma_error_code;
  uint32_t counter_window_ms;
  uint32_t underrun_samples_total;
  uint32_t underrun_samples_delta;
  uint32_t overrun_samples_total;
  uint32_t overrun_samples_delta;
  uint32_t runtime_error_events_total;
  uint32_t runtime_error_events_delta;
  uint32_t output_limiter_samples_total;
  uint32_t output_limiter_samples_delta;
  uint32_t snapshot_sequence_end;
} AudioDebugI2sView;

typedef struct
{
  uint32_t aux_clip_samples;
  uint32_t aux_clip_blocks;
  uint32_t aux_limiter_samples;
  uint32_t aux_adc_overrun_blocks;
  uint32_t aux_analysis_drop_blocks;
  uint32_t aux_analysis_discontinuities;
  uint32_t aux_gate_rejected_peaks;
  uint32_t aux_gate_open_events;
  uint32_t aux_gate_close_events;
  uint32_t aux_deadline_miss_blocks;
  uint32_t fft_dropped_frames;
  uint32_t fft_stream_resets;
  uint32_t fft_deadline_miss_frames;
  uint32_t i2s_analysis_drop_blocks;
  uint32_t i2s_analysis_discontinuities;
  uint32_t i2s_mic_deadline_miss_blocks;
  uint32_t tx_underrun_samples;
  uint32_t tx_overrun_samples;
  uint32_t tx_runtime_error_events;
  uint32_t tx_limiter_samples;
} AudioDebugCounterState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_ENABLE 1U
#define DISPLAY_BOOT_PATTERN_ENABLE 0U
#define DISPLAY_DIRECT_SWEEP_TEST 0U
#define DISPLAY_DRAW_PERIOD_MS 100U
#define AUDIO_FFT_PROCESS_ENABLE 1U
#define AUDIO_DEBUG_REFRESH_PERIOD_MS 50U
#define AUDIO_ANALYSIS_QUEUE_LOW_WATER_BLOCKS 8U
#define AUDIO_ANALYSIS_CATCHUP_START_BLOCKS 32U
#define AUDIO_ANALYSIS_CATCHUP_MAX_PASSES 4U
/* Change only this source selector; the matching capture path is derived. */
#define AUDIO_INPUT_DEFAULT_SOURCE AUDIO_INPUT_SOURCE_AUX
/* AUTO locks once at startup; use LEFT/RIGHT once the mic L/R strap is known. */
#define AUDIO_I2S_MIC_SLOT_MODE AUDIO_I2S_SLOT_AUTO
#if (AUDIO_INPUT_DEFAULT_SOURCE == AUDIO_INPUT_SOURCE_I2S)
#define AUDIO_CAPTURE_ENABLE 1U
#define AUX_CAPTURE_ENABLE 0U
#elif (AUDIO_INPUT_DEFAULT_SOURCE == AUDIO_INPUT_SOURCE_AUX)
#define AUDIO_CAPTURE_ENABLE 0U
#define AUX_CAPTURE_ENABLE 1U
#else
#error "AUDIO_INPUT_DEFAULT_SOURCE must be I2S or AUX"
#endif
#define AUX_CAPTURE_USE_DMA 1U
#define AUDIO_OUTPUT_ENABLE 1U
/* 0 = I2S3/PCM5102 output, 1 = legacy PA8 PWM output. */
#define AUDIO_OUTPUT_USE_PWM 0U
#define AUDIO_OUTPUT_DEFAULT_MODE AUDIO_OUTPUT_MODE_MIC_MONITOR
#define AUDIO_OUTPUT_BOOT_TEST_TONE_MS 0U

#if ((AUDIO_OUTPUT_ENABLE != 0U) && (AUDIO_OUTPUT_USE_PWM == 0U))
/* I2S3 carries the selected, processed mono stream to both PCM5102 channels. */
#define AUDIO_OUTPUT_I2S_HANDLE hi2s3
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
ST7789_HandleTypeDef hdisplay;
volatile uint8_t display_ready = 0U;
volatile uint8_t display_waveform_enable = 1U;
volatile uint32_t display_draw_error_count = 0U;
volatile uint32_t display_waveform_draw_count = 0U;
volatile uint32_t display_waveform_wait_count = 0U;
volatile uint32_t display_analysis_defer_count = 0U;
volatile uint32_t display_draw_duration_last_ms = 0U;
volatile uint32_t display_draw_duration_max_ms = 0U;
volatile uint32_t display_startup_draw_status = HAL_ERROR;
volatile uint32_t display_direct_test_count = 0U;
volatile uint32_t display_direct_test_x = 0U;
volatile uint32_t display_direct_test_status = HAL_ERROR;
volatile uint8_t aux_capture_start_failed = 0U;
volatile uint8_t audio_output_start_failed = 0U;
volatile uint32_t diag_aux_blocks_per_sec = 0U;
volatile uint32_t diag_aux_samples_per_sec = 0U;
volatile uint32_t diag_aux_realtime_samples_per_sec = 0U;
volatile uint32_t diag_aux_analysis_blocks_per_sec = 0U;
volatile uint32_t diag_i2s_blocks_per_sec = 0U;
volatile uint32_t diag_i2s_samples_per_sec = 0U;
volatile uint32_t diag_i2s_analysis_blocks_per_sec = 0U;
volatile uint32_t diag_output_blocks_per_sec = 0U;
volatile uint32_t diag_output_debug_updates_per_sec = 0U;
volatile uint32_t diag_fft_process_per_sec = 0U;
volatile uint32_t diag_display_draws_per_sec = 0U;
volatile uint32_t diag_visualizer_updates_per_sec = 0U;
volatile uint32_t analysis_queue_before_service_blocks = 0U;
volatile uint32_t analysis_queue_after_service_blocks = 0U;
volatile uint32_t analysis_service_passes_last = 0U;
volatile uint32_t analysis_service_passes_max = 0U;
volatile uint32_t analysis_service_passes_total = 0U;
volatile uint32_t analysis_service_cycles_last = 0U;
volatile uint32_t analysis_service_cycles_max = 0U;
volatile uint32_t analysis_catchup_active = 0U;
volatile uint32_t analysis_catchup_events_total = 0U;
volatile uint32_t display_queue_before_draw_blocks = 0U;
volatile uint32_t display_queue_after_draw_blocks = 0U;
volatile uint8_t audio_output_boot_test_active = 0U;
volatile uint32_t audio_output_boot_test_start_tick = 0U;
static uint32_t display_last_draw_tick = 0U;
static uint8_t display_waiting_for_analysis = 0U;
static uint32_t diag_last_tick = 0U;
static uint32_t diag_last_aux_block_count = 0U;
static uint32_t diag_last_aux_realtime_push_count = 0U;
static uint32_t diag_last_aux_analysis_service_count = 0U;
static uint32_t diag_last_i2s_block_count = 0U;
static uint32_t diag_last_i2s_sample_count = 0U;
static uint32_t diag_last_i2s_analysis_service_count = 0U;
static uint32_t diag_last_output_block_count = 0U;
static uint32_t diag_last_output_debug_update_count = 0U;
static uint32_t diag_last_fft_process_count = 0U;
static uint32_t diag_last_display_draw_count = 0U;
static uint32_t diag_last_visualizer_update_count = 0U;
volatile AudioDebugConfigView audio_debug_config;
volatile AudioDebugAuxView audio_debug_aux;
volatile AudioDebugFftView audio_debug_fft;
volatile AudioDebugI2sView audio_debug_i2s;
static uint32_t audio_debug_last_refresh_tick = 0U;
static uint32_t audio_debug_counter_window_ms = 0U;
static AudioDebugCounterState audio_debug_counter_baseline;
static AudioDebugCounterState audio_debug_counter_delta;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2S3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
#if (DISPLAY_ENABLE != 0U)
static void Display_Init(void);
#if (DISPLAY_BOOT_PATTERN_ENABLE != 0U)
static void Display_DrawBootPattern(void);
#endif
#if (DISPLAY_DIRECT_SWEEP_TEST != 0U)
static HAL_StatusTypeDef Display_DirectSweepTest(void);
#endif
#endif
static uint32_t DiagnosticRatePerSecond(uint32_t delta,
                                        uint32_t scale,
                                        uint32_t elapsed_ms);
static void AudioDebug_ResetCounterBaseline(void);
static void AudioDebug_UpdateCounterDeltas(uint32_t elapsed_ms);
static void AudioDebug_Refresh(uint32_t tick_ms);
static uint32_t AudioAnalysis_GetQueueBlocks(void);
static uint32_t AudioAnalysis_GetBlockSamples(void);
static void AudioAnalysis_ServiceOnce(void);
static void AudioAnalysis_ServiceScheduled(void);

/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t DiagnosticRatePerSecond(uint32_t delta,
                                        uint32_t scale,
                                        uint32_t elapsed_ms)
{
  if (elapsed_ms == 0U)
  {
    return 0U;
  }

  return (uint32_t)((((uint64_t)delta * scale * 1000U) +
                     (elapsed_ms / 2U)) /
                    elapsed_ms);
}

static uint32_t AudioAnalysis_GetQueueBlocks(void)
{
  return (audio_input_source == AUDIO_INPUT_SOURCE_AUX) ?
         aux_analysis_queue_count : i2s_mic_analysis_queue_count;
}

static uint32_t AudioAnalysis_GetBlockSamples(void)
{
  /* Each I2S DMA half contains 64 stereo frames; AUX halves contain 128. */
  return (audio_input_source == AUDIO_INPUT_SOURCE_AUX) ?
         AUX_CAPTURE_BUF_LEN : (I2S_BUF_LEN / 8U);
}

static void AudioAnalysis_ServiceOnce(void)
{
#if ((AUX_CAPTURE_ENABLE != 0U) && (AUX_CAPTURE_USE_DMA == 0U) && (DISPLAY_DIRECT_SWEEP_TEST == 0U))
  AuxCapture_Poll();
#elif ((AUX_CAPTURE_ENABLE != 0U) && (AUX_CAPTURE_USE_DMA != 0U) && (DISPLAY_DIRECT_SWEEP_TEST == 0U))
  AuxCapture_Service();
#endif
#if ((AUDIO_CAPTURE_ENABLE != 0U) && (DISPLAY_DIRECT_SWEEP_TEST == 0U))
  AudioCapture_Service();
#endif
#if ((DISPLAY_DIRECT_SWEEP_TEST == 0U) && (AUDIO_FFT_PROCESS_ENABLE != 0U))
  (void)AudioFFT_ProcessIfReady();
#endif
}

static void AudioAnalysis_ServiceScheduled(void)
{
  uint32_t cycle_start = DWT->CYCCNT;
  uint32_t queue_blocks = AudioAnalysis_GetQueueBlocks();
  uint32_t pass_limit;
  uint32_t passes = 0U;

  analysis_queue_before_service_blocks = queue_blocks;
  if ((analysis_catchup_active == 0U) &&
      (queue_blocks >= AUDIO_ANALYSIS_CATCHUP_START_BLOCKS))
  {
    analysis_catchup_active = 1U;
    analysis_catchup_events_total++;
  }
  else if ((analysis_catchup_active != 0U) &&
           (queue_blocks <= AUDIO_ANALYSIS_QUEUE_LOW_WATER_BLOCKS))
  {
    analysis_catchup_active = 0U;
  }

  if ((queue_blocks == 0U) && (audio_fft_ready == 0U))
  {
    analysis_queue_after_service_blocks = 0U;
    analysis_service_passes_last = 0U;
    return;
  }

  pass_limit = (analysis_catchup_active != 0U) ?
               AUDIO_ANALYSIS_CATCHUP_MAX_PASSES : 1U;

  for (passes = 0U; passes < pass_limit; passes++)
  {
    AudioAnalysis_ServiceOnce();
    queue_blocks = AudioAnalysis_GetQueueBlocks();
    if (queue_blocks <= AUDIO_ANALYSIS_QUEUE_LOW_WATER_BLOCKS)
    {
      analysis_catchup_active = 0U;
      passes++;
      break;
    }
  }

  analysis_queue_after_service_blocks = queue_blocks;
  analysis_service_passes_last = passes;
  analysis_service_passes_total += passes;
  if (passes > analysis_service_passes_max)
  {
    analysis_service_passes_max = passes;
  }

  analysis_service_cycles_last = DWT->CYCCNT - cycle_start;
  if (analysis_service_cycles_last > analysis_service_cycles_max)
  {
    analysis_service_cycles_max = analysis_service_cycles_last;
  }
}

static AudioDebugCounterState AudioDebug_ReadCounterState(void)
{
  AudioDebugCounterState state;

  state.aux_clip_samples = aux_adc_clip_sample_count;
  state.aux_clip_blocks = aux_adc_clip_block_count;
  state.aux_limiter_samples = aux_output_limiter_count;
  state.aux_adc_overrun_blocks = aux_adc_overrun_count;
  state.aux_analysis_drop_blocks = aux_analysis_drop_count;
  state.aux_analysis_discontinuities = aux_analysis_discontinuity_count;
  state.aux_gate_rejected_peaks = aux_output_gate_rejected_peak_count;
  state.aux_gate_open_events = aux_output_gate_open_count;
  state.aux_gate_close_events = aux_output_gate_close_count;
  state.aux_deadline_miss_blocks = aux_output_process_deadline_miss_count;
  state.fft_dropped_frames = audio_fft_drop_count;
  state.fft_stream_resets = audio_fft_stream_reset_count;
  state.fft_deadline_miss_frames =
      audio_fft_process_deadline_miss_count;
  state.i2s_analysis_drop_blocks = i2s_mic_analysis_drop_count;
  state.i2s_analysis_discontinuities =
      i2s_mic_analysis_discontinuity_count;
  state.i2s_mic_deadline_miss_blocks =
      i2s_mic_process_deadline_miss_count;
  state.tx_underrun_samples = audio_out_mic_underrun_count;
  state.tx_overrun_samples = audio_out_mic_overrun_count;
  state.tx_runtime_error_events = audio_out_i2s_runtime_error_count;
  state.tx_limiter_samples = audio_out_limiter_count;

  return state;
}

static void AudioDebug_ResetCounterBaseline(void)
{
  AudioDebugCounterState zero = {0};

  audio_debug_counter_baseline = AudioDebug_ReadCounterState();
  audio_debug_counter_delta = zero;
  audio_debug_counter_window_ms = 0U;
}

static void AudioDebug_UpdateCounterDeltas(uint32_t elapsed_ms)
{
  AudioDebugCounterState current = AudioDebug_ReadCounterState();

  audio_debug_counter_delta.aux_clip_samples =
      current.aux_clip_samples - audio_debug_counter_baseline.aux_clip_samples;
  audio_debug_counter_delta.aux_clip_blocks =
      current.aux_clip_blocks - audio_debug_counter_baseline.aux_clip_blocks;
  audio_debug_counter_delta.aux_limiter_samples =
      current.aux_limiter_samples - audio_debug_counter_baseline.aux_limiter_samples;
  audio_debug_counter_delta.aux_adc_overrun_blocks =
      current.aux_adc_overrun_blocks -
      audio_debug_counter_baseline.aux_adc_overrun_blocks;
  audio_debug_counter_delta.aux_analysis_drop_blocks =
      current.aux_analysis_drop_blocks -
      audio_debug_counter_baseline.aux_analysis_drop_blocks;
  audio_debug_counter_delta.aux_analysis_discontinuities =
      current.aux_analysis_discontinuities -
      audio_debug_counter_baseline.aux_analysis_discontinuities;
  audio_debug_counter_delta.aux_gate_rejected_peaks =
      current.aux_gate_rejected_peaks -
      audio_debug_counter_baseline.aux_gate_rejected_peaks;
  audio_debug_counter_delta.aux_gate_open_events =
      current.aux_gate_open_events -
      audio_debug_counter_baseline.aux_gate_open_events;
  audio_debug_counter_delta.aux_gate_close_events =
      current.aux_gate_close_events -
      audio_debug_counter_baseline.aux_gate_close_events;
  audio_debug_counter_delta.aux_deadline_miss_blocks =
      current.aux_deadline_miss_blocks -
      audio_debug_counter_baseline.aux_deadline_miss_blocks;
  audio_debug_counter_delta.fft_dropped_frames =
      current.fft_dropped_frames -
      audio_debug_counter_baseline.fft_dropped_frames;
  audio_debug_counter_delta.fft_stream_resets =
      current.fft_stream_resets -
      audio_debug_counter_baseline.fft_stream_resets;
  audio_debug_counter_delta.fft_deadline_miss_frames =
      current.fft_deadline_miss_frames -
      audio_debug_counter_baseline.fft_deadline_miss_frames;
  audio_debug_counter_delta.i2s_analysis_drop_blocks =
      current.i2s_analysis_drop_blocks -
      audio_debug_counter_baseline.i2s_analysis_drop_blocks;
  audio_debug_counter_delta.i2s_analysis_discontinuities =
      current.i2s_analysis_discontinuities -
      audio_debug_counter_baseline.i2s_analysis_discontinuities;
  audio_debug_counter_delta.i2s_mic_deadline_miss_blocks =
      current.i2s_mic_deadline_miss_blocks -
      audio_debug_counter_baseline.i2s_mic_deadline_miss_blocks;
  audio_debug_counter_delta.tx_underrun_samples =
      current.tx_underrun_samples -
      audio_debug_counter_baseline.tx_underrun_samples;
  audio_debug_counter_delta.tx_overrun_samples =
      current.tx_overrun_samples -
      audio_debug_counter_baseline.tx_overrun_samples;
  audio_debug_counter_delta.tx_runtime_error_events =
      current.tx_runtime_error_events -
      audio_debug_counter_baseline.tx_runtime_error_events;
  audio_debug_counter_delta.tx_limiter_samples =
      current.tx_limiter_samples -
      audio_debug_counter_baseline.tx_limiter_samples;

  audio_debug_counter_baseline = current;
  audio_debug_counter_window_ms = elapsed_ms;
}

static uint32_t AudioDebug_NextSequence(uint32_t current)
{
  current++;
  return (current == 0U) ? 1U : current;
}

static void AudioDebug_Refresh(uint32_t tick_ms)
{
  uint32_t sequence;
  AudioVisualizerScopeDiagnostics scope_diagnostics;
  uint32_t selected_analysis_block_samples;
  uint32_t selected_analysis_queue_blocks;
  uint32_t selected_analysis_queue_max_blocks;
  uint32_t selected_analysis_queue_capacity_blocks;
  uint32_t selected_analysis_source_sequence;
  uint32_t selected_analysis_last_serviced_sequence;
  uint32_t selected_analysis_drop_blocks;
  uint32_t selected_analysis_drop_blocks_delta;
  uint32_t selected_analysis_discontinuities;
  uint32_t selected_analysis_discontinuities_delta;
  uint32_t selected_analysis_blocks_per_sec;
  uint32_t selected_analysis_queue_lag_blocks;

  if (audio_input_source == AUDIO_INPUT_SOURCE_AUX)
  {
    selected_analysis_queue_blocks = aux_analysis_queue_count;
    selected_analysis_queue_max_blocks = aux_analysis_queue_max;
    selected_analysis_queue_capacity_blocks = aux_analysis_queue_capacity;
    selected_analysis_source_sequence = aux_analysis_source_sequence;
    selected_analysis_last_serviced_sequence =
        aux_analysis_last_serviced_sequence;
    selected_analysis_drop_blocks = aux_analysis_drop_count;
    selected_analysis_drop_blocks_delta =
        audio_debug_counter_delta.aux_analysis_drop_blocks;
    selected_analysis_discontinuities = aux_analysis_discontinuity_count;
    selected_analysis_discontinuities_delta =
        audio_debug_counter_delta.aux_analysis_discontinuities;
    selected_analysis_blocks_per_sec = diag_aux_analysis_blocks_per_sec;
  }
  else
  {
    selected_analysis_queue_blocks = i2s_mic_analysis_queue_count;
    selected_analysis_queue_max_blocks = i2s_mic_analysis_queue_max;
    selected_analysis_queue_capacity_blocks =
        i2s_mic_analysis_queue_capacity;
    selected_analysis_source_sequence = i2s_mic_analysis_source_sequence;
    selected_analysis_last_serviced_sequence =
        i2s_mic_analysis_last_serviced_sequence;
    selected_analysis_drop_blocks = i2s_mic_analysis_drop_count;
    selected_analysis_drop_blocks_delta =
        audio_debug_counter_delta.i2s_analysis_drop_blocks;
    selected_analysis_discontinuities =
        i2s_mic_analysis_discontinuity_count;
    selected_analysis_discontinuities_delta =
        audio_debug_counter_delta.i2s_analysis_discontinuities;
    selected_analysis_blocks_per_sec = diag_i2s_analysis_blocks_per_sec;
  }

  selected_analysis_queue_lag_blocks =
      selected_analysis_source_sequence -
      selected_analysis_last_serviced_sequence;
  selected_analysis_block_samples = AudioAnalysis_GetBlockSamples();
  AudioVisualizer_GetScopeDiagnostics(&scope_diagnostics);

  sequence = AudioDebug_NextSequence(audio_debug_config.snapshot_sequence_end);
  audio_debug_config.snapshot_sequence_begin = sequence;
  __DMB();
  audio_debug_config.tick_ms = tick_ms;
  audio_debug_config.firmware_build_id = AUDIO_FIRMWARE_BUILD_ID;
  audio_debug_config.compiler_optimized = AUDIO_COMPILER_OPTIMIZED;
  audio_debug_config.input_source = audio_input_source;
  audio_debug_config.output_mode = audio_out_mode_debug;
  audio_debug_config.force_test_tone = audio_out_force_test_tone;
  audio_debug_config.aux_realtime_enabled = aux_output_realtime_enable;
  audio_debug_config.highpass_enabled = aux_output_highpass_enable;
  audio_debug_config.lowpass_enabled = aux_output_lowpass_enable;
  audio_debug_config.gate_enabled = aux_output_gate_enable;
  audio_debug_config.aux_samples_per_sec = diag_aux_realtime_samples_per_sec;
  audio_debug_config.analysis_block_samples =
      selected_analysis_block_samples;
  audio_debug_config.analysis_expected_blocks_per_sec =
      AUDIO_STREAM_SAMPLE_RATE_HZ / selected_analysis_block_samples;
  audio_debug_config.analysis_blocks_per_sec =
      selected_analysis_blocks_per_sec;
  audio_debug_config.analysis_queue_blocks =
      selected_analysis_queue_blocks;
  audio_debug_config.analysis_queue_capacity_blocks =
      selected_analysis_queue_capacity_blocks;
  audio_debug_config.analysis_queue_lag_blocks =
      selected_analysis_queue_lag_blocks;
  audio_debug_config.analysis_latency_ms = (uint32_t)(
      ((uint64_t)selected_analysis_queue_blocks *
       selected_analysis_block_samples *
       1000U) / AUDIO_STREAM_SAMPLE_RATE_HZ);
  audio_debug_config.analysis_queue_before_service_blocks =
      analysis_queue_before_service_blocks;
  audio_debug_config.analysis_queue_after_service_blocks =
      analysis_queue_after_service_blocks;
  audio_debug_config.analysis_catchup_active = analysis_catchup_active;
  audio_debug_config.analysis_catchup_events_total =
      analysis_catchup_events_total;
  audio_debug_config.analysis_service_passes_last =
      analysis_service_passes_last;
  audio_debug_config.analysis_service_passes_max =
      analysis_service_passes_max;
  audio_debug_config.analysis_service_passes_total =
      analysis_service_passes_total;
  audio_debug_config.analysis_service_cycles_last =
      analysis_service_cycles_last;
  audio_debug_config.analysis_service_cycles_max =
      analysis_service_cycles_max;
  audio_debug_config.visualizer_updates_per_sec =
      diag_visualizer_updates_per_sec;
  audio_debug_config.analysis_low_water_blocks =
      AUDIO_ANALYSIS_QUEUE_LOW_WATER_BLOCKS;
  audio_debug_config.analysis_catchup_start_blocks =
      AUDIO_ANALYSIS_CATCHUP_START_BLOCKS;
  audio_debug_config.analysis_catchup_max_passes =
      AUDIO_ANALYSIS_CATCHUP_MAX_PASSES;
  audio_debug_config.fft_frames_per_sec = diag_fft_process_per_sec;
  audio_debug_config.tx_updates_per_sec = diag_output_debug_updates_per_sec;
  audio_debug_config.display_draws_per_sec = diag_display_draws_per_sec;
  audio_debug_config.display_period_ms = DISPLAY_DRAW_PERIOD_MS;
  audio_debug_config.display_ready = display_ready;
  audio_debug_config.display_waveform_enabled = display_waveform_enable;
  audio_debug_config.display_frame_ready = AudioVisualizer_IsFrameReady();
  audio_debug_config.display_last_draw_age_ms =
      tick_ms - display_last_draw_tick;
  audio_debug_config.display_draw_duration_last_ms =
      display_draw_duration_last_ms;
  audio_debug_config.display_draw_duration_max_ms =
      display_draw_duration_max_ms;
  audio_debug_config.display_defer_events_total =
      display_analysis_defer_count;
  audio_debug_config.display_queue_before_draw_blocks =
      display_queue_before_draw_blocks;
  audio_debug_config.display_queue_after_draw_blocks =
      display_queue_after_draw_blocks;
  audio_debug_config.display_spi_blocking_tx_total =
      st7789_blocking_tx_count;
  audio_debug_config.scope_configured_mode =
      scope_diagnostics.configured_mode;
  audio_debug_config.scope_effective_mode =
      scope_diagnostics.effective_mode;
  audio_debug_config.scope_sample_rate_hz =
      scope_diagnostics.sample_rate_hz;
  audio_debug_config.scope_sample_count = scope_diagnostics.sample_count;
  audio_debug_config.scope_sample_period_ns =
      scope_diagnostics.sample_period_ns;
  audio_debug_config.scope_window_us = scope_diagnostics.window_us;
  audio_debug_config.scope_pretrigger_samples =
      scope_diagnostics.pretrigger_samples;
  audio_debug_config.scope_trigger_hysteresis_s24 =
      scope_diagnostics.trigger_hysteresis;
  audio_debug_config.scope_trigger_hysteresis_adc_codes =
      (scope_diagnostics.trigger_hysteresis + 2048U) / 4096U;
  audio_debug_config.scope_trigger_count = scope_diagnostics.trigger_count;
  audio_debug_config.scope_auto_trigger_count =
      scope_diagnostics.auto_trigger_count;
  audio_debug_config.scope_stream_reset_count =
      scope_diagnostics.stream_reset_count;
  audio_debug_config.scope_snapshot_sequence =
      scope_diagnostics.snapshot_sequence;
  audio_debug_config.scope_drawn_sequence =
      scope_diagnostics.drawn_sequence;
  audio_debug_config.scope_snapshot_ready =
      scope_diagnostics.snapshot_ready;
  audio_debug_config.scope_capture_active =
      scope_diagnostics.capture_active;
  audio_debug_config.scope_trigger_armed =
      scope_diagnostics.trigger_armed;
  audio_debug_config.scope_snapshot_min_s24 =
      scope_diagnostics.snapshot_min;
  audio_debug_config.scope_snapshot_max_s24 =
      scope_diagnostics.snapshot_max;
  audio_debug_config.scope_snapshot_mean_s24 =
      scope_diagnostics.snapshot_mean;
  audio_debug_config.scope_snapshot_peak_s24 =
      scope_diagnostics.snapshot_peak;
  audio_debug_config.scope_snapshot_scale_s24 =
      scope_diagnostics.snapshot_scale;
  audio_debug_config.scope_snapshot_vpp_s24 = (uint32_t)(
      (int64_t)scope_diagnostics.snapshot_max -
      (int64_t)scope_diagnostics.snapshot_min);
  audio_debug_config.scope_snapshot_vpp_adc_codes =
      (audio_debug_config.scope_snapshot_vpp_s24 + 2048U) / 4096U;
  __DMB();
  audio_debug_config.snapshot_sequence_end = sequence;

  sequence = AudioDebug_NextSequence(audio_debug_aux.snapshot_sequence_end);
  audio_debug_aux.snapshot_sequence_begin = sequence;
  __DMB();
  audio_debug_aux.tick_ms = tick_ms;
  audio_debug_aux.source_samples_total = aux_output_realtime_push_count;
  audio_debug_aux.analysis_process_count = audio_sample_process_count;
  audio_debug_aux.raw_avg_adc_codes = aux_output_realtime_avg;
  audio_debug_aux.raw_min_adc_codes = aux_output_realtime_min;
  audio_debug_aux.raw_max_adc_codes = aux_output_realtime_max;
  audio_debug_aux.raw_peak_adc_codes = aux_output_realtime_peak;
  audio_debug_aux.raw_abs_avg_adc_codes = aux_output_realtime_abs_avg;
  audio_debug_aux.raw_bias_mv = aux_output_realtime_bias_mv;
  audio_debug_aux.tracked_bias_adc_codes = aux_output_bias_adc;
  audio_debug_aux.headroom_adc_codes = aux_adc_headroom_codes;
  audio_debug_aux.adc_clip_active = aux_adc_clip_active;
  audio_debug_aux.output_gain_q8 = aux_output_gain_q8;
  audio_debug_aux.highpass_cutoff_hz = aux_output_highpass_cutoff_hz;
  audio_debug_aux.highpass_peak_s16 = aux_output_highpass_peak_s16;
  audio_debug_aux.lowpass_cutoff_hz = aux_output_lowpass_cutoff_hz;
  audio_debug_aux.lowpass_peak_s16 = aux_output_lowpass_peak_s16;
  audio_debug_aux.gate_detector_avg_s16 = aux_output_gate_detector_avg_s16;
  audio_debug_aux.gate_peak_qualified = aux_output_gate_peak_qualified;
  audio_debug_aux.gate_open = aux_output_gate_open;
  audio_debug_aux.gate_gain_q8 = aux_output_gate_gain_q8;
  audio_debug_aux.gate_hold_samples = aux_output_gate_hold_remaining_samples;
  audio_debug_aux.gate_open_peak_threshold_s16 =
      aux_output_gate_open_peak_threshold_s16;
  audio_debug_aux.gate_peak_min_avg_s16 = aux_output_gate_peak_min_avg_s16;
  audio_debug_aux.gate_open_avg_threshold_s16 =
      aux_output_gate_open_avg_threshold_s16;
  audio_debug_aux.gate_close_avg_threshold_s16 =
      aux_output_gate_close_avg_threshold_s16;
  audio_debug_aux.post_gate_peak_s16 = aux_output_peak_s16;
  audio_debug_aux.post_gate_abs_avg_s16 = aux_output_abs_avg_s16;
  if (audio_input_source == AUDIO_INPUT_SOURCE_AUX)
  {
    audio_debug_aux.analysis_peak_s16 = audio_sample_peak / 256U;
    audio_debug_aux.analysis_abs_avg_s16 = audio_sample_abs_avg / 256U;
    audio_debug_aux.analysis_noise_floor_s16 = audio_sample_noise_floor / 256U;
    audio_debug_aux.analysis_signal_s16 = audio_sample_signal_level / 256U;
  }
  else
  {
    audio_debug_aux.analysis_peak_s16 = 0U;
    audio_debug_aux.analysis_abs_avg_s16 = 0U;
    audio_debug_aux.analysis_noise_floor_s16 = 0U;
    audio_debug_aux.analysis_signal_s16 = 0U;
  }
  audio_debug_aux.analysis_noise_floor_frozen =
      (audio_input_source == AUDIO_INPUT_SOURCE_AUX) ?
      (uint32_t)audio_sample_noise_floor_frozen : 0U;
  audio_debug_aux.analysis_noise_floor_freeze_count =
      (audio_input_source == AUDIO_INPUT_SOURCE_AUX) ?
      audio_sample_noise_floor_freeze_count : 0U;
  audio_debug_aux.analysis_selected_tap = aux_analysis_tap;
  audio_debug_aux.analysis_last_serviced_tap =
      aux_analysis_last_serviced_tap;
  audio_debug_aux.analysis_raw_origin_adc_q8 =
      aux_analysis_raw_origin_adc_q8;
  audio_debug_aux.analysis_raw_origin_adc_codes =
      (aux_analysis_raw_origin_adc_q8 + 128U) / 256U;
  audio_debug_aux.analysis_raw_origin_valid =
      aux_analysis_raw_origin_valid;
  audio_debug_aux.analysis_queue_blocks = aux_analysis_queue_count;
  audio_debug_aux.analysis_queue_max_blocks = aux_analysis_queue_max;
  audio_debug_aux.analysis_queue_capacity_blocks =
      aux_analysis_queue_capacity;
  audio_debug_aux.analysis_blocks_per_sec =
      diag_aux_analysis_blocks_per_sec;
  audio_debug_aux.analysis_source_sequence =
      aux_analysis_source_sequence;
  audio_debug_aux.analysis_last_serviced_sequence =
      aux_analysis_last_serviced_sequence;
  audio_debug_aux.counter_window_ms = audio_debug_counter_window_ms;
  audio_debug_aux.adc_clip_samples_total = aux_adc_clip_sample_count;
  audio_debug_aux.adc_clip_samples_delta =
      audio_debug_counter_delta.aux_clip_samples;
  audio_debug_aux.adc_clip_blocks_total = aux_adc_clip_block_count;
  audio_debug_aux.adc_clip_blocks_delta =
      audio_debug_counter_delta.aux_clip_blocks;
  audio_debug_aux.limiter_samples_total = aux_output_limiter_count;
  audio_debug_aux.limiter_samples_delta =
      audio_debug_counter_delta.aux_limiter_samples;
  audio_debug_aux.adc_overrun_blocks_total = aux_adc_overrun_count;
  audio_debug_aux.adc_overrun_blocks_delta =
      audio_debug_counter_delta.aux_adc_overrun_blocks;
  audio_debug_aux.analysis_drop_blocks_total = aux_analysis_drop_count;
  audio_debug_aux.analysis_drop_blocks_delta =
      audio_debug_counter_delta.aux_analysis_drop_blocks;
  audio_debug_aux.analysis_discontinuities_total =
      aux_analysis_discontinuity_count;
  audio_debug_aux.analysis_discontinuities_delta =
      audio_debug_counter_delta.aux_analysis_discontinuities;
  audio_debug_aux.gate_rejected_peaks_total =
      aux_output_gate_rejected_peak_count;
  audio_debug_aux.gate_rejected_peaks_delta =
      audio_debug_counter_delta.aux_gate_rejected_peaks;
  audio_debug_aux.gate_open_events_total = aux_output_gate_open_count;
  audio_debug_aux.gate_open_events_delta =
      audio_debug_counter_delta.aux_gate_open_events;
  audio_debug_aux.gate_close_events_total = aux_output_gate_close_count;
  audio_debug_aux.gate_close_events_delta =
      audio_debug_counter_delta.aux_gate_close_events;
  audio_debug_aux.output_process_cycles_last =
      aux_output_process_cycles_last;
  audio_debug_aux.output_process_cycles_max =
      aux_output_process_cycles_max;
  audio_debug_aux.output_process_cycle_budget =
      aux_output_process_cycle_budget;
  audio_debug_aux.deadline_miss_blocks_total =
      aux_output_process_deadline_miss_count;
  audio_debug_aux.deadline_miss_blocks_delta =
      audio_debug_counter_delta.aux_deadline_miss_blocks;
  __DMB();
  audio_debug_aux.snapshot_sequence_end = sequence;

  sequence = AudioDebug_NextSequence(audio_debug_fft.snapshot_sequence_end);
  audio_debug_fft.snapshot_sequence_begin = sequence;
  __DMB();
  audio_debug_fft.tick_ms = tick_ms;
  audio_debug_fft.analysis_process_count = audio_sample_process_count;
  audio_debug_fft.analysis_peak_s16 = audio_sample_peak / 256U;
  audio_debug_fft.analysis_abs_avg_s16 = audio_sample_abs_avg / 256U;
  audio_debug_fft.analysis_noise_floor_s16 =
      audio_sample_noise_floor / 256U;
  audio_debug_fft.analysis_signal_s16 = audio_sample_signal_level / 256U;
  audio_debug_fft.analysis_noise_floor_frozen =
      (uint32_t)audio_sample_noise_floor_frozen;
  audio_debug_fft.analysis_noise_floor_freeze_count =
      audio_sample_noise_floor_freeze_count;
  audio_debug_fft.sample_rate_hz = AUDIO_STREAM_SAMPLE_RATE_HZ;
  audio_debug_fft.nyquist_boundary_hz =
      AUDIO_STREAM_SAMPLE_RATE_HZ / 2U;
  audio_debug_fft.analysis_min_hz = AUDIO_FFT_MIN_FREQ_HZ;
  audio_debug_fft.analysis_max_boundary_hz = AUDIO_FFT_MAX_FREQ_HZ;
  audio_debug_fft.highest_analyzed_bin_hz =
      (((AUDIO_FFT_SIZE / 2U) - 1U) * AUDIO_STREAM_SAMPLE_RATE_HZ) /
      AUDIO_FFT_SIZE;
  audio_debug_fft.frame_count = audio_fft_frame_count;
  audio_debug_fft.frame_hop_samples = AUDIO_FFT_HOP_SIZE;
  audio_debug_fft.expected_frames_per_sec_x10 = (uint32_t)(
      (((uint64_t)AUDIO_STREAM_SAMPLE_RATE_HZ * 10U) +
       (AUDIO_FFT_HOP_SIZE / 2U)) / AUDIO_FFT_HOP_SIZE);
  audio_debug_fft.process_count = audio_fft_process_count;
  audio_debug_fft.ready = audio_fft_ready;
  audio_debug_fft.collect_index = audio_fft_collect_index;
  audio_debug_fft.peak_fft_bin = audio_fft_peak_fft_bin;
  audio_debug_fft.peak_frequency_hz = audio_fft_peak_freq_hz;
  audio_debug_fft.peak_display_band = audio_fft_peak_bin;
  audio_debug_fft.peak_level_ppm_fs = audio_fft_peak_value;
  audio_debug_fft.peak_dbfs_x10 = audio_fft_peak_db_x10;
  audio_debug_fft.peak_prominence_db_x10 =
      audio_fft_peak_prominence_db_x10;
  audio_debug_fft.floor_dbfs = audio_fft_floor_db;
  audio_debug_fft.adaptive_enabled = audio_fft_adaptive_enable;
  audio_debug_fft.adaptive_min_peak_dbfs_x10 =
      audio_fft_adaptive_min_peak_db_x10;
  audio_debug_fft.adaptive_min_prominence_db_x10 =
      audio_fft_adaptive_min_prominence_db_x10;
  audio_debug_fft.adaptive_confirm_frames =
      audio_fft_adaptive_confirm_frames;
  audio_debug_fft.adaptive_peak_qualified =
      audio_fft_view_peak_qualified;
  audio_debug_fft.global_peak_in_view = audio_fft_global_peak_in_view;
  audio_debug_fft.view_mode = audio_fft_view_mode;
  audio_debug_fft.view_bucket_khz = audio_fft_view_bucket_khz;
  audio_debug_fft.view_min_hz = audio_fft_view_min_hz;
  audio_debug_fft.view_max_hz = audio_fft_view_max_hz;
  audio_debug_fft.view_bin_width_hz = audio_fft_view_bin_width_hz;
  audio_debug_fft.view_change_count = audio_fft_view_change_count;
  audio_debug_fft.view_candidate_bucket_khz =
      audio_fft_view_candidate_bucket_khz;
  audio_debug_fft.view_candidate_count =
      audio_fft_view_candidate_count;
  audio_debug_fft.display_axis_change_drawn =
      audio_vis_fft_axis_change_drawn;
  audio_debug_fft.axis_tick_count = audio_fft_axis_tick_count;
  for (uint32_t i = 0U; i < AUDIO_FFT_AXIS_TICK_COUNT; i++)
  {
    audio_debug_fft.axis_tick_hz[i] = audio_fft_axis_tick_hz[i];
  }
  for (uint32_t i = 0U; i <= AUDIO_FFT_DISPLAY_BINS; i++)
  {
    audio_debug_fft.band_edge_hz[i] = audio_fft_band_edge_hz[i];
  }
  for (uint32_t i = 0U; i < AUDIO_FFT_DISPLAY_BINS; i++)
  {
    audio_debug_fft.band_level_permille[i] = audio_fft_bin_values[i];
    audio_debug_fft.band_height_pixels[i] = audio_vis_fft_debug_bins[i];
  }
  audio_debug_fft.fft_draw_count = audio_vis_fft_draw_count;
  audio_debug_fft.fft_empty_count = audio_vis_fft_empty_count;
  audio_debug_fft.display_draw_errors_total = display_draw_error_count;
  audio_debug_fft.display_draw_error_stage = audio_vis_draw_error_stage;
  audio_debug_fft.counter_window_ms = audio_debug_counter_window_ms;
  audio_debug_fft.dropped_frames_total = audio_fft_drop_count;
  audio_debug_fft.dropped_frames_delta =
      audio_debug_counter_delta.fft_dropped_frames;
  audio_debug_fft.source_discontinuities_total =
      selected_analysis_discontinuities;
  audio_debug_fft.source_discontinuities_delta =
      selected_analysis_discontinuities_delta;
  audio_debug_fft.stream_resets_total = audio_fft_stream_reset_count;
  audio_debug_fft.stream_resets_delta =
      audio_debug_counter_delta.fft_stream_resets;
  audio_debug_fft.stream_reset_discarded_ready_total =
      audio_fft_stream_reset_discarded_ready_count;
  audio_debug_fft.stream_reset_last_collect_index =
      audio_fft_stream_reset_last_collect_index;
  audio_debug_fft.process_cycles_last = audio_fft_process_cycles_last;
  audio_debug_fft.process_cycles_max = audio_fft_process_cycles_max;
  audio_debug_fft.process_cycle_budget = audio_fft_process_cycle_budget;
  audio_debug_fft.deadline_miss_frames_total =
      audio_fft_process_deadline_miss_count;
  audio_debug_fft.deadline_miss_frames_delta =
      audio_debug_counter_delta.fft_deadline_miss_frames;
  __DMB();
  audio_debug_fft.snapshot_sequence_end = sequence;

  sequence = AudioDebug_NextSequence(audio_debug_i2s.snapshot_sequence_end);
  audio_debug_i2s.snapshot_sequence_begin = sequence;
  __DMB();
  audio_debug_i2s.tick_ms = tick_ms;
  audio_debug_i2s.input_source = audio_input_source;
  audio_debug_i2s.output_mode = audio_out_mode_debug;
  audio_debug_i2s.force_test_tone = audio_out_force_test_tone;
  audio_debug_i2s.streaming = audio_out_mic_streaming;
  audio_debug_i2s.tx_peak_s16 = audio_out_tx_peak;
  audio_debug_i2s.tx_abs_avg_s16 = audio_out_tx_abs_avg;
  audio_debug_i2s.tx_level_smooth_s16 = audio_out_tx_level_smooth;
  audio_debug_i2s.tx_debug_updates_total = audio_out_tx_debug_update_count;
  audio_debug_i2s.tx_half_callbacks_total = audio_out_half_count;
  audio_debug_i2s.tx_full_callbacks_total = audio_out_full_count;
  audio_debug_i2s.tx_fill_count_total = audio_out_fill_count;
  audio_debug_i2s.ring_available_samples = audio_out_mic_available;
  audio_debug_i2s.ring_available_min_samples = audio_out_mic_available_min;
  audio_debug_i2s.ring_available_max_samples = audio_out_mic_available_max;
  audio_debug_i2s.ring_push_samples_total = audio_out_mic_push_count;
  audio_debug_i2s.ring_pop_samples_total = audio_out_mic_pop_count;
  audio_debug_i2s.analysis_queue_source = audio_input_source;
  audio_debug_i2s.analysis_queue_blocks = selected_analysis_queue_blocks;
  audio_debug_i2s.analysis_queue_max_blocks =
      selected_analysis_queue_max_blocks;
  audio_debug_i2s.analysis_queue_capacity_blocks =
      selected_analysis_queue_capacity_blocks;
  audio_debug_i2s.analysis_source_sequence =
      selected_analysis_source_sequence;
  audio_debug_i2s.analysis_last_serviced_sequence =
      selected_analysis_last_serviced_sequence;
  audio_debug_i2s.analysis_drop_blocks_total =
      selected_analysis_drop_blocks;
  audio_debug_i2s.analysis_drop_blocks_delta =
      selected_analysis_drop_blocks_delta;
  audio_debug_i2s.analysis_discontinuities_total =
      selected_analysis_discontinuities;
  audio_debug_i2s.analysis_discontinuities_delta =
      selected_analysis_discontinuities_delta;
  audio_debug_i2s.mic_processed_samples_total =
      i2s_mic_processed_sample_count;
  audio_debug_i2s.mic_output_peak_s16 = i2s_mic_output_peak_s16;
  audio_debug_i2s.mic_output_abs_avg_s16 = i2s_mic_output_abs_avg_s16;
  audio_debug_i2s.mic_gate_open = i2s_mic_gate_open;
  audio_debug_i2s.mic_selected_slot = i2s_mic_selected_slot;
  audio_debug_i2s.mic_process_cycles_last = i2s_mic_process_cycles_last;
  audio_debug_i2s.mic_process_cycles_max = i2s_mic_process_cycles_max;
  audio_debug_i2s.mic_process_cycle_budget = i2s_mic_process_cycle_budget;
  audio_debug_i2s.mic_deadline_miss_blocks_total =
      i2s_mic_process_deadline_miss_count;
  audio_debug_i2s.mic_deadline_miss_blocks_delta =
      audio_debug_counter_delta.i2s_mic_deadline_miss_blocks;
  audio_debug_i2s.start_status = audio_out_start_status;
  audio_debug_i2s.i2s_error_code = audio_out_i2s_error_code;
  audio_debug_i2s.dma_error_code = audio_out_dma_error_code;
  audio_debug_i2s.counter_window_ms = audio_debug_counter_window_ms;
  audio_debug_i2s.underrun_samples_total = audio_out_mic_underrun_count;
  audio_debug_i2s.underrun_samples_delta =
      audio_debug_counter_delta.tx_underrun_samples;
  audio_debug_i2s.overrun_samples_total = audio_out_mic_overrun_count;
  audio_debug_i2s.overrun_samples_delta =
      audio_debug_counter_delta.tx_overrun_samples;
  audio_debug_i2s.runtime_error_events_total =
      audio_out_i2s_runtime_error_count;
  audio_debug_i2s.runtime_error_events_delta =
      audio_debug_counter_delta.tx_runtime_error_events;
  audio_debug_i2s.output_limiter_samples_total = audio_out_limiter_count;
  audio_debug_i2s.output_limiter_samples_delta =
      audio_debug_counter_delta.tx_limiter_samples;
  __DMB();
  audio_debug_i2s.snapshot_sequence_end = sequence;
}

#if (DISPLAY_ENABLE != 0U)
static void Display_Init(void)
{
  hdisplay.hspi = &hspi1;
  hdisplay.dc_port = DC_GPIO_Port;
  hdisplay.dc_pin = DC_Pin;

  hdisplay.rst_port = RST_GPIO_Port;
  hdisplay.rst_pin = RST_Pin;

  hdisplay.cs_port = CS_GPIO_Port;
  hdisplay.cs_pin = CS_Pin;

  hdisplay.bl_port = SPI_MISO_GPIO_Port;
  hdisplay.bl_pin = SPI_MISO_Pin;

  hdisplay.x_offset = 0U;
  hdisplay.y_offset = 0U;

  if (ST7789_Init(&hdisplay) == HAL_OK)
  {
    display_ready = 1U;
  }
  else
  {
    display_ready = 2U;
  }
}

#if (DISPLAY_BOOT_PATTERN_ENABLE != 0U)
static void Display_DrawBootPattern(void)
{
  if (display_ready != 1U)
  {
    return;
  }

  if (ST7789_FillScreen(&hdisplay, ST7789_BLACK) != HAL_OK)
  {
    display_draw_error_count++;
    return;
  }

  if (ST7789_FillRect(&hdisplay, 0U, 0U, 80U, 240U, ST7789_RED) != HAL_OK)
  {
    display_draw_error_count++;
    return;
  }

  if (ST7789_FillRect(&hdisplay, 80U, 0U, 80U, 240U, ST7789_GREEN) != HAL_OK)
  {
    display_draw_error_count++;
    return;
  }

  if (ST7789_FillRect(&hdisplay, 160U, 0U, 80U, 240U, ST7789_BLUE) != HAL_OK)
  {
    display_draw_error_count++;
    return;
  }
}
#endif

#if (DISPLAY_DIRECT_SWEEP_TEST != 0U)
static HAL_StatusTypeDef Display_DirectSweepTest(void)
{
  static uint8_t initialized = 0U;
  static uint16_t x = 0U;
  static uint16_t last_x = 0U;
  HAL_StatusTypeDef status;

  if (display_ready != 1U)
  {
    display_direct_test_status = HAL_ERROR;
    return HAL_ERROR;
  }

  if (initialized == 0U)
  {
    status = ST7789_FillScreen(&hdisplay, ST7789_BLACK);
    if (status != HAL_OK)
    {
      display_direct_test_status = status;
      return status;
    }

    status = ST7789_FillRect(&hdisplay, 0U, 0U, ST7789_WIDTH, 8U, ST7789_BLUE);
    if (status != HAL_OK)
    {
      display_direct_test_status = status;
      return status;
    }

    status = ST7789_DrawHLine(&hdisplay, 0U, 120U, ST7789_WIDTH, ST7789_CYAN);
    if (status != HAL_OK)
    {
      display_direct_test_status = status;
      return status;
    }

    initialized = 1U;
  }

  status = ST7789_DrawVLine(&hdisplay, last_x, 8U, (uint16_t)(ST7789_HEIGHT - 8U), ST7789_BLACK);
  if (status != HAL_OK)
  {
    display_direct_test_status = status;
    return status;
  }

  if (last_x == 0U)
  {
    (void)ST7789_DrawVLine(&hdisplay, 0U, 8U, (uint16_t)(ST7789_HEIGHT - 8U), ST7789_CYAN);
  }

  status = ST7789_DrawVLine(&hdisplay, x, 8U, (uint16_t)(ST7789_HEIGHT - 8U), ST7789_YELLOW);
  if (status != HAL_OK)
  {
    display_direct_test_status = status;
    return status;
  }

  last_x = x;
  x = (uint16_t)(x + 4U);
  if (x >= ST7789_WIDTH)
  {
    x = 0U;
  }

  display_direct_test_x = x;
  display_direct_test_count++;
  display_direct_test_status = HAL_OK;

  return HAL_OK;
}
#endif
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_I2S3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

#if (DISPLAY_ENABLE != 0U)
  Display_Init();
#if (DISPLAY_BOOT_PATTERN_ENABLE != 0U)
  Display_DrawBootPattern();
  display_waveform_enable = 0U;
#elif (DISPLAY_DIRECT_SWEEP_TEST != 0U)
  display_waveform_enable = 0U;
  display_last_draw_tick = 0U;
#else
  display_waveform_enable = 1U;
  if (display_ready == 1U)
  {
    display_startup_draw_status = (uint32_t)AudioVisualizer_DrawWaveform(&hdisplay);
    if (display_startup_draw_status != HAL_OK)
    {
      display_draw_error_count++;
    }
    else
    {
      display_waveform_draw_count++;
    }
  }
  display_last_draw_tick = HAL_GetTick();
#endif
#else
  display_ready = 0U;
  display_waveform_enable = 0U;
#endif

#if (DISPLAY_DIRECT_SWEEP_TEST == 0U)
  if (AudioFFT_Init() != HAL_OK)
  {
    Error_Handler();
  }
#endif

  audio_input_source = AUDIO_INPUT_DEFAULT_SOURCE;
  audio_sample_slot_mode = AUDIO_I2S_MIC_SLOT_MODE;
  audio_out_mic_repeat_factor = 1U;

#if (AUDIO_OUTPUT_ENABLE != 0U)
#if (AUDIO_OUTPUT_BOOT_TEST_TONE_MS != 0U)
  AudioOutput_SetMode(AUDIO_OUTPUT_MODE_TEST_TONE);
  audio_output_boot_test_active = 1U;
  audio_output_boot_test_start_tick = HAL_GetTick();
#else
  AudioOutput_SetMode(AUDIO_OUTPUT_DEFAULT_MODE);
#endif
#endif

#if ((AUDIO_CAPTURE_ENABLE != 0U) && (DISPLAY_DIRECT_SWEEP_TEST == 0U))
  if (AudioCapture_Start(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
#endif

#if ((AUX_CAPTURE_ENABLE != 0U) && (DISPLAY_DIRECT_SWEEP_TEST == 0U))
#if (AUX_CAPTURE_USE_DMA != 0U)
  if (AuxCapture_StartDma(&hadc1, &htim2) != HAL_OK)
  {
    aux_capture_start_failed = 1U;
  }
#else
  if (AuxCapture_Start(&hadc1) != HAL_OK)
  {
    aux_capture_start_failed = 1U;
  }
#endif
#endif

#if (AUDIO_OUTPUT_ENABLE != 0U)
  /*
   * Start capture before reconstruction. I2S RX fills from its DMA callback;
   * AUX needs its queued blocks serviced during this short prefill window.
   */
  uint32_t output_prefill_start_tick = HAL_GetTick();
  while ((HAL_GetTick() - output_prefill_start_tick) < 12U)
  {
#if ((AUX_CAPTURE_ENABLE != 0U) && (AUX_CAPTURE_USE_DMA != 0U) && (DISPLAY_DIRECT_SWEEP_TEST == 0U))
    AuxCapture_Service();
#elif ((AUX_CAPTURE_ENABLE != 0U) && (AUX_CAPTURE_USE_DMA == 0U) && (DISPLAY_DIRECT_SWEEP_TEST == 0U))
    AuxCapture_Poll();
#endif
  }
#if (AUDIO_OUTPUT_USE_PWM != 0U)
  if (AudioPwmOutput_Start(&htim1, TIM_CHANNEL_1, &htim3) != HAL_OK)
  {
    audio_output_start_failed = 1U;
  }
#else
  if (AudioOutput_Start(&AUDIO_OUTPUT_I2S_HANDLE) != HAL_OK)
  {
    audio_output_start_failed = 1U;
  }
#endif
#endif

  diag_last_tick = HAL_GetTick();
  diag_last_aux_block_count = aux_adc_half_count + aux_adc_full_count;
  diag_last_aux_realtime_push_count = aux_output_realtime_push_count;
  diag_last_aux_analysis_service_count = aux_analysis_service_count;
  diag_last_i2s_block_count = audio_half_count + audio_full_count;
  diag_last_i2s_sample_count = i2s_mic_processed_sample_count;
  diag_last_i2s_analysis_service_count = i2s_mic_analysis_service_count;
  diag_last_output_block_count = audio_out_half_count + audio_out_full_count;
  diag_last_output_debug_update_count = audio_out_tx_debug_update_count;
  diag_last_fft_process_count = audio_fft_process_count;
  diag_last_display_draw_count = display_waveform_draw_count;
  diag_last_visualizer_update_count = audio_vis_update_count;
  AudioDebug_ResetCounterBaseline();
  audio_debug_last_refresh_tick = HAL_GetTick();
  AudioDebug_Refresh(audio_debug_last_refresh_tick);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if (DISPLAY_DIRECT_SWEEP_TEST == 0U)
    AudioAnalysis_ServiceScheduled();
#endif

#if ((AUDIO_OUTPUT_ENABLE != 0U) && (AUDIO_OUTPUT_BOOT_TEST_TONE_MS != 0U))
    if ((audio_output_boot_test_active != 0U) &&
        ((HAL_GetTick() - audio_output_boot_test_start_tick) >= AUDIO_OUTPUT_BOOT_TEST_TONE_MS))
    {
      AudioOutput_SetMode(AUDIO_OUTPUT_DEFAULT_MODE);
      audio_output_boot_test_active = 0U;
    }
#endif

#if (DISPLAY_DIRECT_SWEEP_TEST != 0U)
    if (display_ready == 1U)
    {
      display_last_draw_tick = HAL_GetTick();
      if (Display_DirectSweepTest() != HAL_OK)
      {
        display_draw_error_count++;
      }
      else
      {
        display_waveform_draw_count++;
      }
    }
    HAL_Delay(30U);
#else
    uint32_t display_now_tick = HAL_GetTick();
    uint32_t analysis_queue_blocks =
        (audio_input_source == AUDIO_INPUT_SOURCE_AUX) ?
        aux_analysis_queue_count : i2s_mic_analysis_queue_count;
    uint8_t display_due =
        ((display_ready == 1U) &&
         (display_waveform_enable != 0U) &&
         ((display_now_tick - display_last_draw_tick) >=
          DISPLAY_DRAW_PERIOD_MS)) ? 1U : 0U;

    if ((display_due != 0U) &&
        (analysis_queue_blocks <= AUDIO_ANALYSIS_QUEUE_LOW_WATER_BLOCKS) &&
        (audio_fft_ready == 0U) &&
        (AudioVisualizer_IsFrameReady() != 0U))
    {
      uint32_t draw_start_tick = HAL_GetTick();
      display_queue_before_draw_blocks = analysis_queue_blocks;
      HAL_StatusTypeDef draw_status = AudioVisualizer_DrawWaveform(&hdisplay);

      display_last_draw_tick = HAL_GetTick();
      display_queue_after_draw_blocks = AudioAnalysis_GetQueueBlocks();
      display_draw_duration_last_ms = display_last_draw_tick - draw_start_tick;
      if (display_draw_duration_last_ms > display_draw_duration_max_ms)
      {
        display_draw_duration_max_ms = display_draw_duration_last_ms;
      }
      display_waiting_for_analysis = 0U;

      if (draw_status != HAL_OK)
      {
        display_draw_error_count++;
      }
      else
      {
        display_waveform_draw_count++;
      }
    }
    else if (display_due != 0U)
    {
      if (display_waiting_for_analysis == 0U)
      {
        display_analysis_defer_count++;
        display_waiting_for_analysis = 1U;
      }
    }
    else if ((display_ready == 1U) &&
             (display_waveform_enable != 0U) &&
             (AudioVisualizer_IsFrameReady() == 0U))
    {
      display_waveform_wait_count++;
    }
#endif

    uint32_t diag_now_tick = HAL_GetTick();
    uint32_t diag_elapsed_ms = diag_now_tick - diag_last_tick;
    if (diag_elapsed_ms >= 1000U)
    {
      uint32_t aux_blocks = aux_adc_half_count + aux_adc_full_count;
      uint32_t aux_realtime_samples = aux_output_realtime_push_count;
      uint32_t aux_analysis_blocks = aux_analysis_service_count;
      uint32_t i2s_blocks = audio_half_count + audio_full_count;
      uint32_t i2s_samples = i2s_mic_processed_sample_count;
      uint32_t i2s_analysis_blocks = i2s_mic_analysis_service_count;
      uint32_t output_blocks = audio_out_half_count + audio_out_full_count;
      uint32_t output_debug_updates = audio_out_tx_debug_update_count;
      uint32_t fft_process_count = audio_fft_process_count;
      uint32_t display_draw_count = display_waveform_draw_count;
      uint32_t visualizer_update_count = audio_vis_update_count;

      AuxCapture_UpdateDiagnostics();
#if (AUDIO_OUTPUT_USE_PWM != 0U)
      AudioPwmOutput_UpdateDiagnostics();
#endif
      diag_last_tick = diag_now_tick;

      diag_aux_blocks_per_sec = DiagnosticRatePerSecond(
          aux_blocks - diag_last_aux_block_count, 1U, diag_elapsed_ms);
      diag_aux_samples_per_sec = DiagnosticRatePerSecond(
          aux_blocks - diag_last_aux_block_count,
          AUX_CAPTURE_BUF_LEN,
          diag_elapsed_ms);
      diag_aux_realtime_samples_per_sec = DiagnosticRatePerSecond(
          aux_realtime_samples - diag_last_aux_realtime_push_count,
          1U,
          diag_elapsed_ms);
      diag_aux_analysis_blocks_per_sec = DiagnosticRatePerSecond(
          aux_analysis_blocks - diag_last_aux_analysis_service_count,
          1U,
          diag_elapsed_ms);
      diag_i2s_blocks_per_sec = DiagnosticRatePerSecond(
          i2s_blocks - diag_last_i2s_block_count, 1U, diag_elapsed_ms);
      diag_i2s_samples_per_sec = DiagnosticRatePerSecond(
          i2s_samples - diag_last_i2s_sample_count, 1U, diag_elapsed_ms);
      diag_i2s_analysis_blocks_per_sec = DiagnosticRatePerSecond(
          i2s_analysis_blocks - diag_last_i2s_analysis_service_count,
          1U,
          diag_elapsed_ms);
      diag_output_blocks_per_sec = DiagnosticRatePerSecond(
          output_blocks - diag_last_output_block_count, 1U, diag_elapsed_ms);
      diag_output_debug_updates_per_sec = DiagnosticRatePerSecond(
          output_debug_updates - diag_last_output_debug_update_count,
          1U,
          diag_elapsed_ms);
      diag_fft_process_per_sec = DiagnosticRatePerSecond(
          fft_process_count - diag_last_fft_process_count,
          1U,
          diag_elapsed_ms);
      diag_display_draws_per_sec = DiagnosticRatePerSecond(
          display_draw_count - diag_last_display_draw_count,
          1U,
          diag_elapsed_ms);
      diag_visualizer_updates_per_sec = DiagnosticRatePerSecond(
          visualizer_update_count - diag_last_visualizer_update_count,
          1U,
          diag_elapsed_ms);
      AudioDebug_UpdateCounterDeltas(diag_elapsed_ms);

      diag_last_aux_block_count = aux_blocks;
      diag_last_aux_realtime_push_count = aux_realtime_samples;
      diag_last_aux_analysis_service_count = aux_analysis_blocks;
      diag_last_i2s_block_count = i2s_blocks;
      diag_last_i2s_sample_count = i2s_samples;
      diag_last_i2s_analysis_service_count = i2s_analysis_blocks;
      diag_last_output_block_count = output_blocks;
      diag_last_output_debug_update_count = output_debug_updates;
      diag_last_fft_process_count = fft_process_count;
      diag_last_display_draw_count = display_draw_count;
      diag_last_visualizer_update_count = visualizer_update_count;
    }

    if ((diag_now_tick - audio_debug_last_refresh_tick) >=
        AUDIO_DEBUG_REFRESH_PERIOD_MS)
    {
      audio_debug_last_refresh_tick = diag_now_tick;
      AudioDebug_Refresh(diag_now_tick);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 256;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 128;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2624;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2624;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|SPI_MISO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin SPI_MISO_Pin */
  GPIO_InitStruct.Pin = CS_Pin|SPI_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RST_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
