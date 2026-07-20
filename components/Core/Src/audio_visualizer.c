#include "audio_visualizer.h"
#include "audio_config.h"
#include "audio_capture.h"
#include "audio_fft.h"
#include "audio_output.h"
#include "audio_samples.h"
#include "arm_math.h"
#include <math.h>

#define AUDIO_VIS_WAVE_LABEL_Y 4U
#define AUDIO_VIS_FREQ_LABEL_Y 108U
#define AUDIO_VIS_TICK_LABEL_Y 220U
#define AUDIO_VIS_FREQ_AXIS_Y  231U
#define AUDIO_VIS_GRID_COLOR   0x2104U
#define AUDIO_VIS_BORDER_COLOR 0x39E7U
#define AUDIO_VIS_DIM_CYAN     0x03EFU
#define AUDIO_VIS_DIM_GREEN    0x03E0U
#define AUDIO_VIS_SAMPLE_RATE_HZ AUDIO_STREAM_SAMPLE_RATE_HZ
#define AUDIO_VIS_SCOPE_Q16     65536U
#define AUDIO_VIS_MAX_DRAW_COLUMNS 12U
#define AUDIO_VIS_MAX_PENDING_COLUMNS 36U
#define AUDIO_VIS_DEFAULT_SWEEP_MS 1000U
#define AUDIO_VIS_DEFAULT_FFT_DRAW_MS 100U
#define AUDIO_VIS_MIN_PEAK_SCALE 32768U
#define AUDIO_VIS_MAX_PEAK_SCALE 8388608U
#define AUDIO_VIS_TRIGGER_HYSTERESIS 16384U
#define AUDIO_VIS_TRIGGER_AUTO_SAMPLES (AUDIO_VIS_SAMPLE_RATE_HZ / 10U)
#define AUDIO_VIS_SCOPE_PLOT_Y (AUDIO_VIS_WAVEFORM_Y + 1U)
#define AUDIO_VIS_SCOPE_PLOT_H (AUDIO_VIS_WAVEFORM_H - 1U)
#define AUDIO_VIS_SCOPE_TILE_W 14U
#define AUDIO_FFT_DEFAULT_FLOOR_DB (-72)
#define AUDIO_FFT_ADAPTIVE_MIN_HZ 1000U
#define AUDIO_FFT_ADAPTIVE_DEFAULT_MIN_DB_X10 (-550)
#define AUDIO_FFT_ADAPTIVE_DEFAULT_PROMINENCE_DB_X10 80U
#define AUDIO_FFT_ADAPTIVE_DEFAULT_CONFIRM_FRAMES 3U

#if (AUDIO_FFT_MAX_FREQ_HZ > (AUDIO_VIS_SAMPLE_RATE_HZ / 2U))
#error "FFT display maximum cannot exceed the Nyquist frequency"
#endif

#if ((AUDIO_VIS_SCOPE_PLOT_X + AUDIO_VIS_SCOPE_SAMPLE_COUNT) != (AUDIO_VIS_WIDTH - 1U))
#error "Triggered scope must fill the complete area inside the graph borders"
#endif

#if ((AUDIO_VIS_SCOPE_SAMPLE_COUNT % AUDIO_VIS_SCOPE_TILE_W) != 0U)
#error "Triggered scope plot width must be an integer number of drawing tiles"
#endif

volatile uint8_t audio_vis_frame_ready = 0U;
volatile uint32_t audio_vis_update_count = 0U;
volatile uint32_t audio_vis_draw_count = 0U;
volatile uint32_t audio_vis_peak = 0U;
volatile uint32_t audio_vis_level_bar_width = 0U;
volatile uint32_t audio_vis_spectrum_peak = 0U;
volatile uint32_t audio_vis_live_draw_count = 0U;
volatile uint32_t audio_vis_live_peak = 0U;
volatile uint32_t audio_vis_live_sample_count = 0U;
volatile uint32_t audio_vis_live_y_span = 0U;
volatile uint32_t audio_vis_draw_stage = 0U;
volatile uint32_t audio_vis_draw_error_stage = 0U;
volatile uint32_t audio_vis_fft_draw_count = 0U;
volatile uint32_t audio_vis_fft_empty_count = 0U;
volatile uint16_t audio_vis_fft_debug_bins[AUDIO_VIS_SPECTRUM_BINS];
volatile uint32_t audio_vis_full_clear_count = 0U;
volatile uint32_t audio_vis_incremental_draw_count = 0U;
volatile uint32_t audio_vis_scope_x = AUDIO_VIS_SCOPE_PLOT_X;
volatile uint32_t audio_vis_scope_wrap_count = 0U;
volatile uint32_t audio_vis_scope_columns_per_draw = 0U;
volatile uint32_t audio_vis_scope_sweep_ms = AUDIO_VIS_DEFAULT_SWEEP_MS;
volatile uint32_t audio_vis_scope_elapsed_ms = 0U;
volatile uint32_t audio_vis_scope_column_accum = 0U;
volatile uint32_t audio_vis_scope_pending_columns = 0U;
volatile uint32_t audio_vis_scope_generated_columns = 0U;
volatile uint32_t audio_vis_scope_dropped_columns = 0U;
volatile uint32_t audio_vis_scope_empty_draw_count = 0U;
volatile uint32_t audio_vis_scope_cursor_x = 0U;
volatile uint32_t audio_vis_scope_cursor_draw_count = 0U;
volatile uint32_t audio_vis_scope_peak_scale = 524288U;
volatile uint32_t audio_vis_spectrum_draw_period_ms = AUDIO_VIS_DEFAULT_FFT_DRAW_MS;
volatile uint32_t audio_vis_spectrum_skip_count = 0U;
volatile uint32_t audio_vis_last_update_tick = 0U;
volatile uint32_t audio_vis_live_timeout_ms = 500U;
volatile uint32_t audio_vis_demo_draw_count = 0U;
volatile uint8_t audio_vis_auto_gain_enable = 1U;
volatile uint8_t audio_vis_demo_until_live = 0U;
volatile uint8_t audio_vis_live_active = 0U;
volatile uint8_t audio_vis_force_demo_waveform = 0U;
volatile uint8_t audio_vis_mode = AUDIO_VIS_SCOPE_MODE_AUTO;

volatile uint32_t audio_fft_init_status = HAL_ERROR;
volatile uint32_t audio_fft_frame_count = 0U;
volatile uint32_t audio_fft_process_count = 0U;
volatile uint32_t audio_fft_drop_count = 0U;
volatile uint32_t audio_fft_stream_reset_count = 0U;
volatile uint32_t audio_fft_stream_reset_discarded_ready_count = 0U;
volatile uint32_t audio_fft_stream_reset_last_collect_index = 0U;
volatile uint32_t audio_fft_ready = 0U;
volatile uint32_t audio_fft_collect_index = 0U;
volatile uint32_t audio_fft_peak_bin = 0U;
volatile uint32_t audio_fft_peak_fft_bin = 0U;
volatile uint32_t audio_fft_peak_freq_hz = 0U;
volatile uint32_t audio_fft_peak_value = 0U;
volatile int32_t audio_fft_peak_db_x10 = AUDIO_FFT_DEFAULT_FLOOR_DB * 10;
volatile uint32_t audio_fft_peak_prominence_db_x10 = 0U;
volatile int32_t audio_fft_floor_db = AUDIO_FFT_DEFAULT_FLOOR_DB;
volatile uint32_t audio_fft_bin_values[AUDIO_FFT_DISPLAY_BINS];
volatile uint32_t audio_fft_adaptive_enable = 1U;
volatile int32_t audio_fft_adaptive_min_peak_db_x10 =
    AUDIO_FFT_ADAPTIVE_DEFAULT_MIN_DB_X10;
volatile uint32_t audio_fft_adaptive_min_prominence_db_x10 =
    AUDIO_FFT_ADAPTIVE_DEFAULT_PROMINENCE_DB_X10;
volatile uint32_t audio_fft_adaptive_confirm_frames =
    AUDIO_FFT_ADAPTIVE_DEFAULT_CONFIRM_FRAMES;
volatile uint32_t audio_fft_view_mode = AUDIO_FFT_VIEW_MODE_FULL;
volatile uint32_t audio_fft_view_bucket_khz = 0U;
volatile uint32_t audio_fft_view_min_hz = AUDIO_FFT_MIN_FREQ_HZ;
volatile uint32_t audio_fft_view_max_hz = AUDIO_FFT_MAX_FREQ_HZ;
volatile uint32_t audio_fft_view_bin_width_hz = 0U;
volatile uint32_t audio_fft_view_change_count = 0U;
volatile uint32_t audio_fft_view_candidate_bucket_khz = 0U;
volatile uint32_t audio_fft_view_candidate_count = 0U;
volatile uint32_t audio_fft_view_peak_qualified = 0U;
volatile uint32_t audio_fft_global_peak_in_view = 1U;
volatile uint32_t audio_fft_band_edge_hz[AUDIO_FFT_DISPLAY_BINS + 1U] =
{
   100U,  1000U,  2000U,  3000U,  4000U,  5000U,
  6000U,  7000U,  8000U,  9000U, 10000U, 11000U,
 12000U, 13000U, 14000U, 15000U, 16000U
};
volatile uint32_t audio_fft_axis_tick_count = 5U;
volatile uint32_t audio_fft_axis_tick_hz[AUDIO_FFT_AXIS_TICK_COUNT] =
{
  100U, 4000U, 8000U, 12000U, 16000U
};
volatile uint32_t audio_fft_process_cycles_last = 0U;
volatile uint32_t audio_fft_process_cycles_max = 0U;
volatile uint32_t audio_fft_process_cycle_budget = 0U;
volatile uint32_t audio_fft_process_deadline_miss_count = 0U;

static uint16_t waveform_y_min[AUDIO_VIS_WIDTH];
static uint16_t waveform_y_max[AUDIO_VIS_WIDTH];
static uint16_t waveform_draw_y_min[AUDIO_VIS_WIDTH];
static uint16_t waveform_draw_y_max[AUDIO_VIS_WIDTH];
static uint16_t spectrum_height[AUDIO_VIS_SPECTRUM_BINS];
static uint16_t spectrum_draw_height[AUDIO_VIS_SPECTRUM_BINS];
static uint16_t spectrum_prev_height[AUDIO_VIS_SPECTRUM_BINS];
static uint8_t audio_vis_has_draw_frame = 0U;
static uint8_t audio_vis_screen_initialized = 0U;
static uint16_t audio_vis_scope_pending_y_min[AUDIO_VIS_WIDTH];
static uint16_t audio_vis_scope_pending_y_max[AUDIO_VIS_WIDTH];
static volatile uint32_t audio_vis_scope_pending_read = 0U;
static volatile uint32_t audio_vis_scope_pending_write = 0U;
static uint16_t audio_vis_scope_draw_y_min[AUDIO_VIS_MAX_DRAW_COLUMNS];
static uint16_t audio_vis_scope_draw_y_max[AUDIO_VIS_MAX_DRAW_COLUMNS];
static int32_t audio_vis_scope_column_min = INT32_MAX;
static int32_t audio_vis_scope_column_max = INT32_MIN;
static uint32_t audio_vis_demo_phase = 0U;
static uint32_t audio_vis_spectrum_last_draw_tick = 0U;
volatile uint32_t audio_vis_fft_axis_change_drawn = UINT32_MAX;

/*
 * Triggered scope state.  A published buffer remains immutable until its
 * sequence has been drawn, so an analysis update can never tear an LCD frame.
 */
static int32_t audio_vis_trigger_prebuffer[AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES];
static int32_t audio_vis_trigger_snapshot[2U][AUDIO_VIS_SCOPE_SAMPLE_COUNT];
static uint16_t audio_vis_trigger_draw_y[AUDIO_VIS_SCOPE_SAMPLE_COUNT];
static uint8_t audio_vis_trigger_tile[AUDIO_VIS_SCOPE_TILE_W *
                                      AUDIO_VIS_SCOPE_PLOT_H * 2U];
static uint32_t audio_vis_trigger_prewrite = 0U;
static uint32_t audio_vis_trigger_prefill = 0U;
static uint32_t audio_vis_trigger_capture_count = 0U;
static uint32_t audio_vis_trigger_write_index = 0U;
static volatile uint32_t audio_vis_trigger_published_index = 0U;
static uint32_t audio_vis_trigger_published_scale[2U] = {1U, 1U};
static uint32_t audio_vis_trigger_wait_samples = 0U;
static int32_t audio_vis_trigger_bias_q8 = 0;
static uint8_t audio_vis_trigger_bias_valid = 0U;
static volatile uint32_t audio_vis_trigger_sequence = 0U;
static volatile uint32_t audio_vis_trigger_drawn_sequence = 0U;
static volatile uint32_t audio_vis_trigger_count = 0U;
static volatile uint32_t audio_vis_trigger_auto_count = 0U;
static volatile uint32_t audio_vis_trigger_reset_count = 0U;
static volatile uint32_t audio_vis_trigger_snapshot_ready = 0U;
static volatile uint32_t audio_vis_trigger_capture_active = 0U;
static volatile uint32_t audio_vis_trigger_armed = 0U;
static volatile uint32_t audio_vis_trigger_draw_active = 0U;
static volatile int32_t audio_vis_trigger_snapshot_min = 0;
static volatile int32_t audio_vis_trigger_snapshot_max = 0;
static volatile int32_t audio_vis_trigger_snapshot_mean = 0;
static volatile uint32_t audio_vis_trigger_snapshot_peak = 0U;
static volatile uint32_t audio_vis_trigger_snapshot_scale = 1U;
static volatile uint8_t audio_vis_trigger_snapshot_valid = 0U;

static arm_rfft_fast_instance_f32 audio_fft_instance;
static int32_t audio_fft_collect_buf[AUDIO_FFT_SIZE];
static int32_t audio_fft_process_buf[AUDIO_FFT_SIZE];
static float32_t audio_fft_window[AUDIO_FFT_SIZE];
static float32_t audio_fft_input[AUDIO_FFT_SIZE];
static float32_t audio_fft_output[AUDIO_FFT_SIZE];
static float32_t audio_fft_window_sum = 1.0f;

/* Full view uses 16 readable bands; adaptive zoom replaces them at run time. */
static const uint16_t audio_fft_full_band_edges_hz[AUDIO_FFT_DISPLAY_BINS + 1U] =
{
   100U,  1000U,  2000U,  3000U,  4000U,  5000U,
  6000U,  7000U,  8000U,  9000U, 10000U, 11000U,
 12000U, 13000U, 14000U, 15000U, 16000U
};

static uint16_t AudioVisualizer_MapToY(int32_t sample, uint32_t peak);
static uint32_t AudioVisualizer_Abs32(int32_t value);
static void AudioVisualizer_UpdateFromS32Internal(const int32_t *samples,
                                                  uint32_t len,
                                                  uint8_t remove_block_dc);
static HAL_StatusTypeDef AudioVisualizer_DrawStatus(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawLabels(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawScopeTimebaseLabel(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawLevelBar(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawSpectrum(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawLiveSampleWaveform(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawTriggeredWaveform(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawDemoWaveform(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawWaveformCenter(ST7789_HandleTypeDef *display);
static void AudioVisualizer_ScopePushSamples(const int32_t *samples,
                                             uint32_t len,
                                             int32_t dc_offset);
static void AudioVisualizer_UpdateAutoScale(uint32_t peak);
static uint8_t AudioVisualizer_EffectiveScopeMode(void);
static void AudioVisualizer_TriggeredPushSamples(const int32_t *samples,
                                                 uint32_t len,
                                                 int32_t dc_offset);
static void AudioVisualizer_TriggeredStartCapture(int32_t trigger_sample,
                                                  uint8_t automatic);
static void AudioVisualizer_TriggeredPublishCapture(void);
static void AudioVisualizer_TriggeredStorePreSample(int32_t sample);
static void AudioVisualizer_TriggeredTileSet(uint32_t x,
                                             uint32_t y,
                                             uint16_t color);
static uint8_t AudioFFT_UpdateAdaptiveView(uint32_t peak_frequency_hz,
                                           int32_t peak_db_x10,
                                           uint32_t prominence_db_x10);
static uint8_t AudioFFT_ConfigureView(uint32_t bucket_khz,
                                      uint8_t count_change);
static void AudioVisualizer_ScopePushColumn(int32_t min_sample,
                                            int32_t max_sample);
static HAL_StatusTypeDef AudioVisualizer_DrawPanel(ST7789_HandleTypeDef *display,
                                                   uint16_t y,
                                                   uint16_t h,
                                                   uint16_t border_color);
static HAL_StatusTypeDef AudioVisualizer_DrawGrid(ST7789_HandleTypeDef *display,
                                                  uint16_t y,
                                                  uint16_t h);
static HAL_StatusTypeDef AudioVisualizer_DrawAxes(ST7789_HandleTypeDef *display,
                                                  uint16_t y,
                                                  uint16_t h,
                                                  uint16_t color);
static HAL_StatusTypeDef AudioVisualizer_DrawFrequencyTicks(ST7789_HandleTypeDef *display,
                                                            uint8_t draw_labels);
static HAL_StatusTypeDef AudioVisualizer_DrawFftReadout(ST7789_HandleTypeDef *display);
static uint16_t AudioVisualizer_FrequencyToSpectrumX(uint32_t frequency_hz);
static uint32_t AudioVisualizer_FormatFrequencyLabel(char *text,
                                                     uint32_t frequency_hz);
static HAL_StatusTypeDef AudioVisualizer_RestoreWaveformColumn(ST7789_HandleTypeDef *display,
                                                               uint16_t x);
static HAL_StatusTypeDef AudioVisualizer_DrawText(ST7789_HandleTypeDef *display,
                                                  uint16_t x,
                                                  uint16_t y,
                                                  const char *text,
                                                  uint16_t color);
static HAL_StatusTypeDef AudioVisualizer_DrawChar(ST7789_HandleTypeDef *display,
                                                  uint16_t x,
                                                  uint16_t y,
                                                  char ch,
                                                  uint16_t color);
static const uint8_t *AudioVisualizer_Font5x7(char ch);
static void AudioVisualizer_LoadFftSpectrum(void);
static void AudioVisualizer_LoadDemoSpectrum(void);
static float32_t AudioFFT_SqrtApprox(float32_t value);

static uint8_t AudioFFT_ConfigureView(uint32_t bucket_khz,
                                      uint8_t count_change)
{
  uint32_t mode = AUDIO_FFT_VIEW_MODE_FULL;
  uint32_t minimum_hz = AUDIO_FFT_MIN_FREQ_HZ;
  uint32_t maximum_hz = AUDIO_FFT_MAX_FREQ_HZ;
  uint8_t changed;

  if (bucket_khz > 15U)
  {
    bucket_khz = 15U;
  }
  if (bucket_khz != 0U)
  {
    mode = AUDIO_FFT_VIEW_MODE_KHZ_ZOOM;
    minimum_hz = (bucket_khz - 1U) * 1000U;
    maximum_hz = (bucket_khz + 1U) * 1000U;
    if (maximum_hz > AUDIO_FFT_MAX_FREQ_HZ)
    {
      maximum_hz = AUDIO_FFT_MAX_FREQ_HZ;
      minimum_hz = maximum_hz - 2000U;
    }
  }

  changed = ((audio_fft_view_mode != mode) ||
             (audio_fft_view_bucket_khz != bucket_khz) ||
             (audio_fft_view_min_hz != minimum_hz) ||
             (audio_fft_view_max_hz != maximum_hz)) ? 1U : 0U;

  if (mode == AUDIO_FFT_VIEW_MODE_FULL)
  {
    for (uint32_t edge = 0U; edge <= AUDIO_FFT_DISPLAY_BINS; edge++)
    {
      audio_fft_band_edge_hz[edge] =
          audio_fft_full_band_edges_hz[edge];
    }
    audio_fft_axis_tick_count = 5U;
    audio_fft_axis_tick_hz[0] = 100U;
    audio_fft_axis_tick_hz[1] = 4000U;
    audio_fft_axis_tick_hz[2] = 8000U;
    audio_fft_axis_tick_hz[3] = 12000U;
    audio_fft_axis_tick_hz[4] = 16000U;
    audio_fft_view_bin_width_hz = 0U;
  }
  else
  {
    uint32_t span_hz = maximum_hz - minimum_hz;

    for (uint32_t edge = 0U; edge <= AUDIO_FFT_DISPLAY_BINS; edge++)
    {
      audio_fft_band_edge_hz[edge] = minimum_hz +
          (uint32_t)(((uint64_t)edge * span_hz) /
                     AUDIO_FFT_DISPLAY_BINS);
    }
    audio_fft_axis_tick_count = 5U;
    for (uint32_t tick = 0U; tick < AUDIO_FFT_AXIS_TICK_COUNT; tick++)
    {
      audio_fft_axis_tick_hz[tick] = minimum_hz +
          (uint32_t)(((uint64_t)tick * span_hz) /
                     (AUDIO_FFT_AXIS_TICK_COUNT - 1U));
    }
    audio_fft_view_bin_width_hz = span_hz / AUDIO_FFT_DISPLAY_BINS;
  }

  audio_fft_view_mode = mode;
  audio_fft_view_bucket_khz = bucket_khz;
  audio_fft_view_min_hz = minimum_hz;
  audio_fft_view_max_hz = maximum_hz;
  if ((changed != 0U) && (count_change != 0U))
  {
    audio_fft_view_change_count++;
  }
  __DMB();
  return changed;
}

static uint8_t AudioFFT_UpdateAdaptiveView(uint32_t peak_frequency_hz,
                                           int32_t peak_db_x10,
                                           uint32_t prominence_db_x10)
{
  uint32_t desired_bucket = 0U;
  uint32_t required_frames = audio_fft_adaptive_confirm_frames;
  uint8_t peak_qualified;

  if (required_frames == 0U)
  {
    required_frames = 1U;
  }
  if (audio_fft_adaptive_enable == 0U)
  {
    audio_fft_view_peak_qualified = 0U;
    audio_fft_view_candidate_bucket_khz = 0U;
    audio_fft_view_candidate_count = 0U;
    return AudioFFT_ConfigureView(0U, 1U);
  }

  peak_qualified =
      ((peak_frequency_hz >= AUDIO_FFT_MIN_FREQ_HZ) &&
       (peak_frequency_hz <= AUDIO_FFT_MAX_FREQ_HZ) &&
       (peak_db_x10 >= audio_fft_adaptive_min_peak_db_x10) &&
       (prominence_db_x10 >=
        audio_fft_adaptive_min_prominence_db_x10)) ? 1U : 0U;
  audio_fft_view_peak_qualified = peak_qualified;

  if ((peak_qualified != 0U) &&
      (peak_frequency_hz >= AUDIO_FFT_ADAPTIVE_MIN_HZ))
  {
    desired_bucket = peak_frequency_hz / 1000U;
    if (desired_bucket > 15U)
    {
      desired_bucket = 15U;
    }
  }
  else if (peak_qualified == 0U)
  {
    /* Hold a zoom briefly through pauses, then return idle to the full view. */
    if (required_frames <= (UINT32_MAX / 4U))
    {
      required_frames *= 4U;
    }
    else
    {
      required_frames = UINT32_MAX;
    }
  }

  if (desired_bucket == audio_fft_view_bucket_khz)
  {
    audio_fft_view_candidate_bucket_khz = desired_bucket;
    audio_fft_view_candidate_count = 0U;
    return 0U;
  }

  if (audio_fft_view_candidate_bucket_khz != desired_bucket)
  {
    audio_fft_view_candidate_bucket_khz = desired_bucket;
    audio_fft_view_candidate_count = 1U;
  }
  else if (audio_fft_view_candidate_count < UINT32_MAX)
  {
    audio_fft_view_candidate_count++;
  }

  if (audio_fft_view_candidate_count < required_frames)
  {
    return 0U;
  }

  audio_fft_view_candidate_count = 0U;
  return AudioFFT_ConfigureView(desired_bucket, 1U);
}

HAL_StatusTypeDef AudioFFT_Init(void)
{
  arm_status status;

  status = arm_rfft_fast_init_f32(&audio_fft_instance, AUDIO_FFT_SIZE);
  audio_fft_init_status = (uint32_t)status;

  if (status != ARM_MATH_SUCCESS)
  {
    return HAL_ERROR;
  }

  audio_fft_window_sum = 0.0f;
  for (uint32_t i = 0U; i < AUDIO_FFT_SIZE; i++)
  {
    float32_t phase = (2.0f * 3.14159265358979323846f * (float32_t)i) /
                      (float32_t)(AUDIO_FFT_SIZE - 1U);
    audio_fft_window[i] = 0.5f - (0.5f * cosf(phase));
    audio_fft_window_sum += audio_fft_window[i];
    audio_fft_collect_buf[i] = 0;
    audio_fft_process_buf[i] = 0;
  }

  for (uint32_t i = 0U; i < AUDIO_FFT_DISPLAY_BINS; i++)
  {
    audio_fft_bin_values[i] = 0U;
  }

  audio_fft_collect_index = 0U;
  audio_fft_ready = 0U;
  audio_fft_frame_count = 0U;
  audio_fft_process_count = 0U;
  audio_fft_drop_count = 0U;
  audio_fft_stream_reset_count = 0U;
  audio_fft_stream_reset_discarded_ready_count = 0U;
  audio_fft_stream_reset_last_collect_index = 0U;
  audio_fft_peak_bin = 0U;
  audio_fft_peak_fft_bin = 0U;
  audio_fft_peak_freq_hz = 0U;
  audio_fft_peak_value = 0U;
  audio_fft_peak_db_x10 = AUDIO_FFT_DEFAULT_FLOOR_DB * 10;
  audio_fft_peak_prominence_db_x10 = 0U;
  audio_fft_floor_db = AUDIO_FFT_DEFAULT_FLOOR_DB;
  audio_fft_adaptive_enable = 1U;
  audio_fft_adaptive_min_peak_db_x10 =
      AUDIO_FFT_ADAPTIVE_DEFAULT_MIN_DB_X10;
  audio_fft_adaptive_min_prominence_db_x10 =
      AUDIO_FFT_ADAPTIVE_DEFAULT_PROMINENCE_DB_X10;
  audio_fft_adaptive_confirm_frames =
      AUDIO_FFT_ADAPTIVE_DEFAULT_CONFIRM_FRAMES;
  audio_fft_view_change_count = 0U;
  audio_fft_view_candidate_bucket_khz = 0U;
  audio_fft_view_candidate_count = 0U;
  audio_fft_view_peak_qualified = 0U;
  audio_fft_global_peak_in_view = 1U;
  (void)AudioFFT_ConfigureView(0U, 0U);
  audio_fft_process_cycles_last = 0U;
  audio_fft_process_cycles_max = 0U;
  audio_fft_process_cycle_budget = (uint32_t)(
      ((uint64_t)SystemCoreClock * AUDIO_FFT_HOP_SIZE) /
      AUDIO_VIS_SAMPLE_RATE_HZ);
  audio_fft_process_deadline_miss_count = 0U;

  return HAL_OK;
}

void AudioFFT_ResetStream(void)
{
  uint32_t previous_ready = audio_fft_ready;
  uint32_t previous_collect_index = audio_fft_collect_index;

  /* Do not let one FFT frame contain samples from both sides of a queue gap. */
  audio_fft_ready = 0U;
  audio_fft_collect_index = 0U;
  audio_fft_stream_reset_last_collect_index = previous_collect_index;
  if (previous_ready != 0U)
  {
    audio_fft_stream_reset_discarded_ready_count++;
  }
  audio_fft_stream_reset_count++;
  audio_fft_peak_bin = 0U;
  audio_fft_peak_fft_bin = 0U;
  audio_fft_peak_freq_hz = 0U;
  audio_fft_peak_value = 0U;
  audio_fft_peak_db_x10 = audio_fft_floor_db * 10;
  audio_fft_peak_prominence_db_x10 = 0U;
  audio_fft_view_peak_qualified = 0U;
  audio_fft_global_peak_in_view = 1U;
  audio_fft_view_candidate_bucket_khz = 0U;
  audio_fft_view_candidate_count = 0U;
  (void)AudioFFT_ConfigureView(0U, 1U);
  for (uint32_t i = 0U; i < AUDIO_FFT_DISPLAY_BINS; i++)
  {
    audio_fft_bin_values[i] = 0U;
  }
  __DMB();
}

void AudioFFT_PushSamplesS32(const int32_t *samples, uint32_t len)
{
  if ((samples == NULL) || (len == 0U))
  {
    return;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    audio_fft_collect_buf[audio_fft_collect_index] = samples[i];
    audio_fft_collect_index++;

    if (audio_fft_collect_index >= AUDIO_FFT_SIZE)
    {
      if (audio_fft_ready == 0U)
      {
        for (uint32_t n = 0U; n < AUDIO_FFT_SIZE; n++)
        {
          audio_fft_process_buf[n] = audio_fft_collect_buf[n];
        }
        audio_fft_ready = 1U;
        audio_fft_frame_count++;
      }
      else
      {
        audio_fft_drop_count++;
      }

      /* Retain only the configured overlap (zero in the current build). */
      for (uint32_t n = 0U; n < (AUDIO_FFT_SIZE - AUDIO_FFT_HOP_SIZE); n++)
      {
        audio_fft_collect_buf[n] = audio_fft_collect_buf[n + AUDIO_FFT_HOP_SIZE];
      }
      audio_fft_collect_index = AUDIO_FFT_SIZE - AUDIO_FFT_HOP_SIZE;
    }
  }
}

uint8_t AudioFFT_ProcessIfReady(void)
{
  int64_t sum = 0;
  int32_t dc_offset;
  uint32_t cycle_start;
  uint32_t peak_bin = 0U;
  uint32_t raw_peak_bin = 0U;
  uint32_t global_start_bin;
  uint32_t global_end_bin;
  uint32_t adaptive_peak_frequency_hz;
  float32_t display_peak_amplitude = 0.0f;
  float32_t raw_peak_power = 0.0f;
  float32_t raw_peak_amplitude;
  float32_t noise_power_sum = 0.0f;
  uint32_t noise_power_count = 0U;
  uint32_t prominence_db_x10 = 0U;
  int32_t raw_peak_db_x10;
  float32_t bin_amplitudes[AUDIO_FFT_DISPLAY_BINS];
  int32_t floor_db = audio_fft_floor_db;
  uint8_t view_changed;

  if (audio_fft_ready == 0U)
  {
    return 0U;
  }

  cycle_start = DWT->CYCCNT;

  for (uint32_t i = 0U; i < AUDIO_FFT_SIZE; i++)
  {
    sum += audio_fft_process_buf[i];
  }
  dc_offset = (int32_t)(sum / (int64_t)AUDIO_FFT_SIZE);

  for (uint32_t i = 0U; i < AUDIO_FFT_SIZE; i++)
  {
    float32_t centered = (float32_t)(audio_fft_process_buf[i] - dc_offset);
    audio_fft_input[i] = (centered / 8388608.0f) * audio_fft_window[i];
  }

  arm_rfft_fast_f32(&audio_fft_instance,
                    audio_fft_input,
                    audio_fft_output,
                    0U);

  /*
   * Find the global peak before selecting a display view.  Otherwise a tone
   * outside the current zoom could never cause the axis to follow it.
   */
  global_start_bin =
      ((AUDIO_FFT_MIN_FREQ_HZ * AUDIO_FFT_SIZE) +
       (AUDIO_VIS_SAMPLE_RATE_HZ / 2U)) / AUDIO_VIS_SAMPLE_RATE_HZ;
  global_end_bin =
      ((AUDIO_FFT_MAX_FREQ_HZ * AUDIO_FFT_SIZE) +
       (AUDIO_VIS_SAMPLE_RATE_HZ / 2U)) / AUDIO_VIS_SAMPLE_RATE_HZ;
  if (global_start_bin < 1U)
  {
    global_start_bin = 1U;
  }
  if (global_end_bin >= (AUDIO_FFT_SIZE / 2U))
  {
    global_end_bin = (AUDIO_FFT_SIZE / 2U) - 1U;
  }

  for (uint32_t fft_bin = global_start_bin;
       fft_bin <= global_end_bin;
       fft_bin++)
  {
    float32_t real = audio_fft_output[2U * fft_bin];
    float32_t imag = audio_fft_output[(2U * fft_bin) + 1U];
    float32_t power = (real * real) + (imag * imag);

    if ((fft_bin == global_start_bin) || (power > raw_peak_power))
    {
      raw_peak_power = power;
      raw_peak_bin = fft_bin;
    }
  }

  for (uint32_t fft_bin = global_start_bin;
       fft_bin <= global_end_bin;
       fft_bin++)
  {
    uint32_t distance = (fft_bin > raw_peak_bin) ?
                        (fft_bin - raw_peak_bin) :
                        (raw_peak_bin - fft_bin);

    if (distance > 2U)
    {
      float32_t real = audio_fft_output[2U * fft_bin];
      float32_t imag = audio_fft_output[(2U * fft_bin) + 1U];

      noise_power_sum += (real * real) + (imag * imag);
      noise_power_count++;
    }
  }

  if (audio_fft_window_sum <= 0.0f)
  {
    raw_peak_amplitude = 0.0f;
  }
  else
  {
    raw_peak_amplitude =
        (2.0f * AudioFFT_SqrtApprox(raw_peak_power)) /
        audio_fft_window_sum;
  }
  if (raw_peak_amplitude < 0.000001f)
  {
    raw_peak_amplitude = 0.000001f;
  }
  if (raw_peak_amplitude > 1.0f)
  {
    raw_peak_amplitude = 1.0f;
  }
  raw_peak_db_x10 = (int32_t)(200.0f * log10f(raw_peak_amplitude));

  if ((raw_peak_power > 0.000001f) && (noise_power_count != 0U))
  {
    float32_t noise_power = noise_power_sum / (float32_t)noise_power_count;

    if (noise_power > 0.000001f)
    {
      float32_t prominence = 100.0f * log10f(raw_peak_power / noise_power);

      if (prominence > 1200.0f)
      {
        prominence = 1200.0f;
      }
      if (prominence > 0.0f)
      {
        prominence_db_x10 = (uint32_t)prominence;
      }
    }
    else
    {
      prominence_db_x10 = 1200U;
    }
  }

  audio_fft_peak_fft_bin = raw_peak_bin;
  if ((raw_peak_bin > 0U) &&
      (raw_peak_bin < ((AUDIO_FFT_SIZE / 2U) - 1U)))
  {
    float32_t left_real = audio_fft_output[2U * (raw_peak_bin - 1U)];
    float32_t left_imag =
        audio_fft_output[(2U * (raw_peak_bin - 1U)) + 1U];
    float32_t center_real = audio_fft_output[2U * raw_peak_bin];
    float32_t center_imag = audio_fft_output[(2U * raw_peak_bin) + 1U];
    float32_t right_real = audio_fft_output[2U * (raw_peak_bin + 1U)];
    float32_t right_imag =
        audio_fft_output[(2U * (raw_peak_bin + 1U)) + 1U];
    float32_t left_power =
        (left_real * left_real) + (left_imag * left_imag);
    float32_t center_power =
        (center_real * center_real) + (center_imag * center_imag);
    float32_t right_power =
        (right_real * right_real) + (right_imag * right_imag);
    float32_t denominator =
        left_power - (2.0f * center_power) + right_power;
    float32_t offset = 0.0f;

    if ((denominator > 0.000001f) || (denominator < -0.000001f))
    {
      offset = 0.5f * (left_power - right_power) / denominator;
      if (offset > 0.5f)
      {
        offset = 0.5f;
      }
      else if (offset < -0.5f)
      {
        offset = -0.5f;
      }
    }
    float32_t interpolated_frequency_hz =
        ((float32_t)raw_peak_bin + offset) *
        (float32_t)AUDIO_VIS_SAMPLE_RATE_HZ /
        (float32_t)AUDIO_FFT_SIZE;

    adaptive_peak_frequency_hz = (uint32_t)interpolated_frequency_hz;
    audio_fft_peak_freq_hz =
        (uint32_t)(interpolated_frequency_hz + 0.5f);
  }
  else
  {
    audio_fft_peak_freq_hz =
        (raw_peak_bin * AUDIO_VIS_SAMPLE_RATE_HZ) / AUDIO_FFT_SIZE;
    adaptive_peak_frequency_hz = audio_fft_peak_freq_hz;
  }

  audio_fft_peak_value =
      (uint32_t)(raw_peak_amplitude * 1000000.0f);
  audio_fft_peak_db_x10 = raw_peak_db_x10;
  audio_fft_peak_prominence_db_x10 = prominence_db_x10;
  view_changed = AudioFFT_UpdateAdaptiveView(adaptive_peak_frequency_hz,
                                             raw_peak_db_x10,
                                             prominence_db_x10);
  audio_fft_global_peak_in_view =
      ((audio_fft_peak_freq_hz >= audio_fft_view_min_hz) &&
       (audio_fft_peak_freq_hz <= audio_fft_view_max_hz)) ? 1U : 0U;

  for (uint32_t i = 0U; i < AUDIO_FFT_DISPLAY_BINS; i++)
  {
    bin_amplitudes[i] = 0.0f;
  }

  for (uint32_t display_bin = 0U; display_bin < AUDIO_FFT_DISPLAY_BINS; display_bin++)
  {
    uint32_t start_bin =
        (uint32_t)((((uint64_t)audio_fft_band_edge_hz[display_bin] *
                     AUDIO_FFT_SIZE) +
                    (AUDIO_VIS_SAMPLE_RATE_HZ - 1U)) /
                   AUDIO_VIS_SAMPLE_RATE_HZ);
    uint32_t end_bin =
        (uint32_t)((((uint64_t)audio_fft_band_edge_hz[display_bin + 1U] *
                     AUDIO_FFT_SIZE) +
                    (AUDIO_VIS_SAMPLE_RATE_HZ - 1U)) /
                   AUDIO_VIS_SAMPLE_RATE_HZ);
    float32_t band_peak_power = 0.0f;
    float32_t band_amplitude;

    if (start_bin < 1U)
    {
      start_bin = 1U;
    }
    if (end_bin <= start_bin)
    {
      end_bin = start_bin + 1U;
    }
    if ((display_bin == (AUDIO_FFT_DISPLAY_BINS - 1U)) &&
        (audio_fft_band_edge_hz[display_bin + 1U] ==
         AUDIO_FFT_MAX_FREQ_HZ))
    {
      /* Include every ordinary FFT bin below the configured upper edge. */
      end_bin++;
    }
    if (end_bin > (AUDIO_FFT_SIZE / 2U))
    {
      end_bin = AUDIO_FFT_SIZE / 2U;
    }

    for (uint32_t fft_bin = start_bin; fft_bin < end_bin; fft_bin++)
    {
      float32_t real = audio_fft_output[2U * fft_bin];
      float32_t imag = audio_fft_output[(2U * fft_bin) + 1U];
      float32_t power = (real * real) + (imag * imag);

      if (power > band_peak_power)
      {
        band_peak_power = power;
      }
    }

    if (audio_fft_window_sum <= 0.0f)
    {
      band_amplitude = 0.0f;
    }
    else
    {
      band_amplitude = (2.0f * AudioFFT_SqrtApprox(band_peak_power)) /
                       audio_fft_window_sum;
    }

    bin_amplitudes[display_bin] = band_amplitude;
    if (band_amplitude > display_peak_amplitude)
    {
      display_peak_amplitude = band_amplitude;
      peak_bin = display_bin;
    }
  }

  if (floor_db > -20)
  {
    floor_db = -20;
  }
  else if (floor_db < -120)
  {
    floor_db = -120;
  }

  for (uint32_t i = 0U; i < AUDIO_FFT_DISPLAY_BINS; i++)
  {
    float32_t amplitude = bin_amplitudes[i];
    float32_t db;
    uint32_t target;

    if (amplitude < 0.000001f)
    {
      amplitude = 0.000001f;
    }
    db = 20.0f * log10f(amplitude);
    if (db > 0.0f)
    {
      db = 0.0f;
    }
    if (db < (float32_t)floor_db)
    {
      db = (float32_t)floor_db;
    }

    target = (uint32_t)(((db - (float32_t)floor_db) * 1000.0f) /
                        (float32_t)(-floor_db));
    if ((audio_fft_process_count == 0U) || (view_changed != 0U))
    {
      audio_fft_bin_values[i] = target;
    }
    else if (target > audio_fft_bin_values[i])
    {
      audio_fft_bin_values[i] =
          (audio_fft_bin_values[i] + (target * 3U)) / 4U;
    }
    else
    {
      audio_fft_bin_values[i] =
          ((audio_fft_bin_values[i] * 7U) + target) / 8U;
    }
  }

  audio_fft_peak_bin = (audio_fft_global_peak_in_view != 0U) ?
                       peak_bin : UINT32_MAX;
  audio_fft_process_count++;
  audio_fft_ready = 0U;

  audio_fft_process_cycles_last = DWT->CYCCNT - cycle_start;
  if (audio_fft_process_cycles_last > audio_fft_process_cycles_max)
  {
    audio_fft_process_cycles_max = audio_fft_process_cycles_last;
  }
  if ((audio_fft_process_cycle_budget != 0U) &&
      (audio_fft_process_cycles_last > audio_fft_process_cycle_budget))
  {
    audio_fft_process_deadline_miss_count++;
  }

  return 1U;
}

uint8_t AudioFFT_GetDisplayBins(uint16_t *bins, uint32_t len)
{
  uint32_t count = (len < AUDIO_FFT_DISPLAY_BINS) ? len : AUDIO_FFT_DISPLAY_BINS;

  if ((bins == NULL) || (count == 0U) || (audio_fft_process_count == 0U))
  {
    return 0U;
  }

  for (uint32_t i = 0U; i < count; i++)
  {
    uint32_t h = (audio_fft_bin_values[i] * (AUDIO_VIS_SPECTRUM_H - 2U)) / 1000U;
    if (h > AUDIO_VIS_SPECTRUM_H)
    {
      h = AUDIO_VIS_SPECTRUM_H;
    }
    bins[i] = (uint16_t)h;
  }

  return 1U;
}

void AudioVisualizer_UpdateFromU16(const uint16_t *samples, uint32_t len)
{
  uint32_t sum = 0U;
  uint32_t peak = 1U;
  int32_t dc_offset;

  if ((samples == NULL) || (len == 0U))
  {
    return;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    sum += samples[i];
  }
  dc_offset = (int32_t)(sum / len);

  for (uint32_t i = 0U; i < len; i++)
  {
    uint32_t magnitude = AudioVisualizer_Abs32((int32_t)samples[i] - dc_offset);
    if (magnitude > peak)
    {
      peak = magnitude;
    }
  }

  if (AudioVisualizer_EffectiveScopeMode() == AUDIO_VIS_SCOPE_MODE_TRIGGERED)
  {
    int32_t scaled_samples[32U];
    uint32_t offset = 0U;
    uint32_t scaled_peak = peak << 12U;

    if (audio_vis_trigger_bias_valid == 0U)
    {
      audio_vis_trigger_bias_q8 = dc_offset << 8U;
      audio_vis_trigger_bias_valid = 1U;
    }
    AudioVisualizer_UpdateAutoScale(scaled_peak);
    while (offset < len)
    {
      uint32_t count = len - offset;
      if (count > (sizeof(scaled_samples) / sizeof(scaled_samples[0])))
      {
        count = sizeof(scaled_samples) / sizeof(scaled_samples[0]);
      }
      for (uint32_t i = 0U; i < count; i++)
      {
        int32_t raw_q8 = (int32_t)samples[offset + i] << 8U;

        audio_vis_trigger_bias_q8 +=
            (raw_q8 - audio_vis_trigger_bias_q8) / 4096;
        scaled_samples[i] =
            (raw_q8 - audio_vis_trigger_bias_q8) * 16;
      }
      AudioVisualizer_TriggeredPushSamples(scaled_samples, count, 0);
      offset += count;
    }

    audio_vis_peak = scaled_peak;
    audio_vis_last_update_tick = HAL_GetTick();
    audio_vis_live_active = 1U;
    audio_vis_update_count++;
    audio_vis_frame_ready = 1U;
    return;
  }

  for (uint32_t x = 0U; x < AUDIO_VIS_WIDTH; x++)
  {
    int32_t min_sample = 32767;
    int32_t max_sample = -32768;
    uint32_t start = (x * len) / AUDIO_VIS_WIDTH;
    uint32_t end = ((x + 1U) * len) / AUDIO_VIS_WIDTH;

    if (end <= start)
    {
      end = start + 1U;
    }
    if (end > len)
    {
      end = len;
    }

    for (uint32_t index = start; index < end; index++)
    {
      int32_t sample = (int32_t)samples[index] - dc_offset;
      if (sample < min_sample)
      {
        min_sample = sample;
      }
      if (sample > max_sample)
      {
        max_sample = sample;
      }
    }

    if (min_sample > max_sample)
    {
      min_sample = 0;
      max_sample = 0;
    }

    waveform_y_min[x] = AudioVisualizer_MapToY(max_sample, peak);
    waveform_y_max[x] = AudioVisualizer_MapToY(min_sample, peak);
  }

  audio_vis_peak = peak;
  audio_vis_last_update_tick = HAL_GetTick();
  audio_vis_live_active = 1U;
  for (uint32_t i = 0U; i < AUDIO_VIS_SPECTRUM_BINS; i++)
  {
    spectrum_height[i] = 0U;
  }
  audio_vis_spectrum_peak = 0U;
  audio_vis_update_count++;
  audio_vis_frame_ready = 1U;
}

void AudioVisualizer_UpdateFromS32(const int32_t *samples, uint32_t len)
{
  AudioVisualizer_UpdateFromS32Internal(samples, len, 1U);
}

void AudioVisualizer_UpdateFromCenteredS32(const int32_t *samples, uint32_t len)
{
  AudioVisualizer_UpdateFromS32Internal(samples, len, 0U);
}

void AudioVisualizer_SetScopeMode(uint8_t mode)
{
  if (mode > AUDIO_VIS_SCOPE_MODE_AUTO)
  {
    mode = AUDIO_VIS_SCOPE_MODE_AUTO;
  }

  if (audio_vis_mode != mode)
  {
    audio_vis_mode = mode;
    AudioVisualizer_ResetScopeStream();
    /* Redraw the panel once so traces from the previous mode cannot remain. */
    audio_vis_screen_initialized = 0U;
  }
}

void AudioVisualizer_ResetScopeStream(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  audio_vis_trigger_prewrite = 0U;
  audio_vis_trigger_prefill = 0U;
  audio_vis_trigger_capture_count = 0U;
  audio_vis_trigger_wait_samples = 0U;
  audio_vis_trigger_bias_q8 = 0;
  audio_vis_trigger_bias_valid = 0U;
  audio_vis_trigger_capture_active = 0U;
  audio_vis_trigger_armed = 0U;
  audio_vis_trigger_snapshot_ready = 0U;
  audio_vis_trigger_snapshot_valid = 0U;
  audio_vis_trigger_drawn_sequence = audio_vis_trigger_sequence;

  audio_vis_scope_pending_read = 0U;
  audio_vis_scope_pending_write = 0U;
  audio_vis_scope_pending_columns = 0U;
  audio_vis_scope_column_accum = 0U;
  audio_vis_scope_column_min = INT32_MAX;
  audio_vis_scope_column_max = INT32_MIN;
  audio_vis_scope_x = AUDIO_VIS_SCOPE_PLOT_X;
  audio_vis_scope_columns_per_draw = 0U;
  audio_vis_scope_elapsed_ms = 0U;
  audio_vis_frame_ready = 0U;
  audio_vis_trigger_reset_count++;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void AudioVisualizer_GetScopeDiagnostics(AudioVisualizerScopeDiagnostics *diagnostics)
{
  if (diagnostics == NULL)
  {
    return;
  }

  diagnostics->configured_mode = audio_vis_mode;
  diagnostics->effective_mode = AudioVisualizer_EffectiveScopeMode();
  diagnostics->sample_rate_hz = AUDIO_VIS_SAMPLE_RATE_HZ;
  diagnostics->sample_count = AUDIO_VIS_SCOPE_SAMPLE_COUNT;
  diagnostics->sample_period_ns =
      (1000000000U + (AUDIO_VIS_SAMPLE_RATE_HZ / 2U)) /
      AUDIO_VIS_SAMPLE_RATE_HZ;
  diagnostics->window_us =
      (uint32_t)(((uint64_t)AUDIO_VIS_SCOPE_SAMPLE_COUNT * 1000000U) /
                 AUDIO_VIS_SAMPLE_RATE_HZ);
  diagnostics->pretrigger_samples = AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES;
  diagnostics->trigger_hysteresis = AUDIO_VIS_TRIGGER_HYSTERESIS;
  diagnostics->trigger_count = audio_vis_trigger_count;
  diagnostics->auto_trigger_count = audio_vis_trigger_auto_count;
  diagnostics->stream_reset_count = audio_vis_trigger_reset_count;
  diagnostics->snapshot_sequence = audio_vis_trigger_sequence;
  diagnostics->drawn_sequence = audio_vis_trigger_drawn_sequence;
  diagnostics->snapshot_ready = audio_vis_trigger_snapshot_ready;
  diagnostics->capture_active = audio_vis_trigger_capture_active;
  diagnostics->trigger_armed = audio_vis_trigger_armed;
  diagnostics->snapshot_min = audio_vis_trigger_snapshot_min;
  diagnostics->snapshot_max = audio_vis_trigger_snapshot_max;
  diagnostics->snapshot_mean = audio_vis_trigger_snapshot_mean;
  diagnostics->snapshot_peak = audio_vis_trigger_snapshot_peak;
  diagnostics->snapshot_scale = audio_vis_trigger_snapshot_scale;
}

static void AudioVisualizer_UpdateFromS32Internal(const int32_t *samples,
                                                  uint32_t len,
                                                  uint8_t remove_block_dc)
{
  int64_t sum = 0;
  uint32_t peak = 1U;
  int32_t dc_offset;

  if ((samples == NULL) || (len == 0U))
  {
    return;
  }

  if (remove_block_dc != 0U)
  {
    for (uint32_t i = 0U; i < len; i++)
    {
      sum += samples[i];
    }
    dc_offset = (int32_t)(sum / (int64_t)len);
  }
  else
  {
    /* The caller supplied a stream-continuous, already centered signal. */
    dc_offset = 0;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    uint32_t magnitude = AudioVisualizer_Abs32(samples[i] - dc_offset);
    if (magnitude > peak)
    {
      peak = magnitude;
    }
  }

  /*
   * The live trace is generated by ScopePushSamples(), and the spectrum is
   * supplied by the CMSIS-DSP FFT.  Rebuilding 240 legacy waveform columns
   * and a second, non-FFT fallback spectrum for every 128-sample block was
   * redundant and prevented the analysis queue from keeping up in Debug.
   */
  AudioVisualizer_UpdateAutoScale(peak);
  if (AudioVisualizer_EffectiveScopeMode() == AUDIO_VIS_SCOPE_MODE_TRIGGERED)
  {
    AudioVisualizer_TriggeredPushSamples(samples, len, dc_offset);
  }
  else
  {
    AudioVisualizer_ScopePushSamples(samples, len, dc_offset);
  }

  audio_vis_peak = peak;
  audio_vis_last_update_tick = HAL_GetTick();
  audio_vis_live_active = 1U;
  audio_vis_update_count++;
  audio_vis_frame_ready = 1U;
}

static uint8_t AudioVisualizer_EffectiveScopeMode(void)
{
  uint8_t mode = audio_vis_mode;

  if (mode == AUDIO_VIS_SCOPE_MODE_AUTO)
  {
    mode = (audio_input_source == AUDIO_INPUT_SOURCE_AUX) ?
           AUDIO_VIS_SCOPE_MODE_TRIGGERED : AUDIO_VIS_SCOPE_MODE_ENVELOPE;
  }
  else if (mode > AUDIO_VIS_SCOPE_MODE_TRIGGERED)
  {
    mode = AUDIO_VIS_SCOPE_MODE_ENVELOPE;
  }

  return mode;
}

static void AudioVisualizer_TriggeredPushSamples(const int32_t *samples,
                                                 uint32_t len,
                                                 int32_t dc_offset)
{
  const int32_t hysteresis = (int32_t)AUDIO_VIS_TRIGGER_HYSTERESIS;

  if ((samples == NULL) || (len == 0U))
  {
    return;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    int32_t sample = samples[i] - dc_offset;

    if (audio_vis_trigger_capture_active != 0U)
    {
      audio_vis_trigger_snapshot[audio_vis_trigger_write_index]
                                [audio_vis_trigger_capture_count] = sample;
      audio_vis_trigger_capture_count++;
      if (audio_vis_trigger_capture_count >= AUDIO_VIS_SCOPE_SAMPLE_COUNT)
      {
        AudioVisualizer_TriggeredPublishCapture();
      }
    }
    else
    {
      uint8_t rising_crossing = 0U;
      uint8_t capture_available =
          ((audio_vis_trigger_snapshot_ready == 0U) &&
           (audio_vis_trigger_draw_active == 0U) &&
           (audio_vis_trigger_prefill >= AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES)) ?
          1U : 0U;

      if ((capture_available != 0U) &&
          (audio_vis_trigger_wait_samples < UINT32_MAX))
      {
        audio_vis_trigger_wait_samples++;
      }
      else if (capture_available == 0U)
      {
        /* Auto-trigger time starts only when a new snapshot can be accepted. */
        audio_vis_trigger_wait_samples = 0U;
      }

      if (sample <= -hysteresis)
      {
        audio_vis_trigger_armed = 1U;
      }
      if ((audio_vis_trigger_armed != 0U) && (sample >= 0))
      {
        rising_crossing = 1U;
        /* A crossing that cannot be captured must not trigger late. */
        audio_vis_trigger_armed = 0U;
      }

      if ((capture_available != 0U) && (rising_crossing != 0U))
      {
        AudioVisualizer_TriggeredStartCapture(sample, 0U);
      }
      else if ((capture_available != 0U) &&
               (audio_vis_trigger_wait_samples >=
                AUDIO_VIS_TRIGGER_AUTO_SAMPLES))
      {
        AudioVisualizer_TriggeredStartCapture(sample, 1U);
      }
    }

    AudioVisualizer_TriggeredStorePreSample(sample);
  }
}

static void AudioVisualizer_TriggeredStartCapture(int32_t trigger_sample,
                                                  uint8_t automatic)
{
  uint32_t oldest = audio_vis_trigger_prewrite;

  audio_vis_trigger_write_index = audio_vis_trigger_published_index ^ 1U;
  for (uint32_t i = 0U; i < AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES; i++)
  {
    uint32_t source = oldest + i;
    if (source >= AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES)
    {
      source -= AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES;
    }
    audio_vis_trigger_snapshot[audio_vis_trigger_write_index][i] =
        audio_vis_trigger_prebuffer[source];
  }

  audio_vis_trigger_snapshot[audio_vis_trigger_write_index]
                            [AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES] =
      trigger_sample;
  audio_vis_trigger_capture_count = AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES + 1U;
  audio_vis_trigger_capture_active = 1U;
  audio_vis_trigger_wait_samples = 0U;
  audio_vis_trigger_armed = 0U;
  if (automatic != 0U)
  {
    audio_vis_trigger_auto_count++;
  }
  else
  {
    audio_vis_trigger_count++;
  }
}

static void AudioVisualizer_TriggeredPublishCapture(void)
{
  int64_t sum = 0;
  int32_t mean;
  int32_t minimum = INT32_MAX;
  int32_t maximum = INT32_MIN;
  uint32_t peak = 0U;
  uint32_t scale = audio_vis_scope_peak_scale;
  uint32_t sequence;

  for (uint32_t i = 0U; i < AUDIO_VIS_SCOPE_SAMPLE_COUNT; i++)
  {
    sum += audio_vis_trigger_snapshot[audio_vis_trigger_write_index][i];
  }
  mean = (int32_t)(sum / (int64_t)AUDIO_VIS_SCOPE_SAMPLE_COUNT);

  /* Center once per complete snapshot; never introduce block-boundary steps. */
  for (uint32_t i = 0U; i < AUDIO_VIS_SCOPE_SAMPLE_COUNT; i++)
  {
    int32_t centered =
        audio_vis_trigger_snapshot[audio_vis_trigger_write_index][i] - mean;
    uint32_t magnitude = AudioVisualizer_Abs32(centered);

    if (centered < minimum)
    {
      minimum = centered;
    }
    if (centered > maximum)
    {
      maximum = centered;
    }
    if (magnitude > peak)
    {
      peak = magnitude;
    }
  }

  if (scale < peak)
  {
    scale = peak;
  }
  if (scale == 0U)
  {
    scale = 1U;
  }

  audio_vis_trigger_snapshot_min = minimum;
  audio_vis_trigger_snapshot_max = maximum;
  audio_vis_trigger_snapshot_mean = mean;
  audio_vis_trigger_snapshot_peak = peak;
  audio_vis_trigger_snapshot_scale = scale;
  audio_vis_trigger_published_scale[audio_vis_trigger_write_index] = scale;
  audio_vis_trigger_published_index = audio_vis_trigger_write_index;
  audio_vis_trigger_capture_active = 0U;
  audio_vis_trigger_capture_count = 0U;

  __DMB();
  sequence = audio_vis_trigger_sequence + 1U;
  if (sequence == 0U)
  {
    sequence = 1U;
  }
  audio_vis_trigger_sequence = sequence;
  audio_vis_trigger_snapshot_ready = 1U;
  audio_vis_trigger_snapshot_valid = 1U;
  audio_vis_scope_generated_columns += AUDIO_VIS_SCOPE_SAMPLE_COUNT;
  audio_vis_frame_ready = 1U;
}

static void AudioVisualizer_TriggeredStorePreSample(int32_t sample)
{
  audio_vis_trigger_prebuffer[audio_vis_trigger_prewrite] = sample;
  audio_vis_trigger_prewrite++;
  if (audio_vis_trigger_prewrite >= AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES)
  {
    audio_vis_trigger_prewrite = 0U;
  }
  if (audio_vis_trigger_prefill < AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES)
  {
    audio_vis_trigger_prefill++;
  }
}

uint8_t AudioVisualizer_IsFrameReady(void)
{
  return audio_vis_frame_ready;
}

HAL_StatusTypeDef AudioVisualizer_DrawWaveform(ST7789_HandleTypeDef *display)
{
  uint8_t ready;
  uint8_t demo_active;
  uint8_t initialized_this_draw = 0U;
  uint32_t now = HAL_GetTick();
  uint32_t live_timeout_ms = audio_vis_live_timeout_ms;
  uint16_t center_y = AUDIO_VIS_WAVEFORM_Y + (AUDIO_VIS_WAVEFORM_H / 2U);

  if (live_timeout_ms == 0U)
  {
    live_timeout_ms = 500U;
  }

  audio_vis_draw_stage = 1U;
  demo_active = ((audio_vis_force_demo_waveform != 0U) ||
                 ((audio_vis_demo_until_live != 0U) &&
                  ((audio_vis_live_active == 0U) ||
                   (audio_vis_last_update_tick == 0U) ||
                   ((now - audio_vis_last_update_tick) > live_timeout_ms)))) ? 1U : 0U;

  ready = audio_vis_frame_ready;
  if (ready != 0U)
  {
    for (uint32_t x = 0U; x < AUDIO_VIS_WIDTH; x++)
    {
      waveform_draw_y_min[x] = waveform_y_min[x];
      waveform_draw_y_max[x] = waveform_y_max[x];
    }
    for (uint32_t i = 0U; i < AUDIO_VIS_SPECTRUM_BINS; i++)
    {
      spectrum_draw_height[i] = spectrum_height[i];
    }
    if (demo_active != 0U)
    {
      AudioVisualizer_LoadDemoSpectrum();
    }
    else
    {
      AudioVisualizer_LoadFftSpectrum();
    }
    audio_vis_frame_ready = 0U;
    audio_vis_has_draw_frame = 1U;
  }
  else if (audio_vis_has_draw_frame == 0U)
  {
    for (uint32_t x = 0U; x < AUDIO_VIS_WIDTH; x++)
    {
      waveform_draw_y_min[x] = center_y;
      waveform_draw_y_max[x] = center_y;
    }
    for (uint32_t i = 0U; i < AUDIO_VIS_SPECTRUM_BINS; i++)
    {
      spectrum_draw_height[i] = 1U;
    }
    if (demo_active != 0U)
    {
      AudioVisualizer_LoadDemoSpectrum();
    }
    else
    {
      AudioVisualizer_LoadFftSpectrum();
    }
  }
  else
  {
    /*
     * Keep the previous waveform/spectrum instead of erasing to a flat line.
     * Display drawing is slow compared with I2S DMA callbacks, and this makes
     * the screen much easier to interpret while debugging.
     */
    if (demo_active != 0U)
    {
      AudioVisualizer_LoadDemoSpectrum();
    }
    else
    {
      AudioVisualizer_LoadFftSpectrum();
    }
  }

  if (audio_vis_screen_initialized == 0U)
  {
    if (ST7789_FillScreen(display, ST7789_BLACK) != HAL_OK)
    {
      audio_vis_draw_error_stage = audio_vis_draw_stage;
      return HAL_ERROR;
    }

    audio_vis_screen_initialized = 1U;
    initialized_this_draw = 1U;
    audio_vis_full_clear_count++;

    if (AudioVisualizer_DrawLabels(display) != HAL_OK)
    {
      audio_vis_draw_error_stage = audio_vis_draw_stage;
      return HAL_ERROR;
    }
  }
  audio_vis_draw_stage = 2U;

  if ((initialized_this_draw != 0U) &&
      (AudioVisualizer_DrawWaveformCenter(display) != HAL_OK))
  {
    audio_vis_draw_error_stage = audio_vis_draw_stage;
    return HAL_ERROR;
  }
  audio_vis_draw_stage = 3U;

  if (AudioVisualizer_DrawStatus(display) != HAL_OK)
  {
    audio_vis_draw_error_stage = audio_vis_draw_stage;
    return HAL_ERROR;
  }
  audio_vis_draw_stage = 4U;

  if (AudioVisualizer_DrawLevelBar(display) != HAL_OK)
  {
    audio_vis_draw_error_stage = audio_vis_draw_stage;
    return HAL_ERROR;
  }
  audio_vis_draw_stage = 5U;

  if (demo_active != 0U)
  {
    if (AudioVisualizer_DrawDemoWaveform(display) != HAL_OK)
    {
      audio_vis_draw_error_stage = audio_vis_draw_stage;
      return HAL_ERROR;
    }
  }
  else if (AudioVisualizer_EffectiveScopeMode() ==
           AUDIO_VIS_SCOPE_MODE_TRIGGERED)
  {
    if (AudioVisualizer_DrawTriggeredWaveform(display) != HAL_OK)
    {
      audio_vis_draw_error_stage = audio_vis_draw_stage;
      return HAL_ERROR;
    }
  }
  else if (AudioVisualizer_DrawLiveSampleWaveform(display) != HAL_OK)
  {
    audio_vis_draw_error_stage = audio_vis_draw_stage;
    return HAL_ERROR;
  }
  audio_vis_draw_stage = 6U;

  if (AudioVisualizer_DrawSpectrum(display) != HAL_OK)
  {
    audio_vis_draw_error_stage = audio_vis_draw_stage;
    return HAL_ERROR;
  }
  audio_vis_draw_stage = 7U;

  audio_vis_draw_count++;
  audio_vis_draw_stage = 8U;
  return HAL_OK;
}

static uint16_t AudioVisualizer_MapToY(int32_t sample, uint32_t peak)
{
  int32_t half_height = (int32_t)(AUDIO_VIS_WAVEFORM_H / 2U);
  int32_t center_y = (int32_t)AUDIO_VIS_WAVEFORM_Y + half_height;
  int32_t y;

  if (peak == 0U)
  {
    peak = 1U;
  }

  y = center_y - ((sample * (half_height - 2)) / (int32_t)peak);

  if (y < (int32_t)AUDIO_VIS_WAVEFORM_Y)
  {
    y = (int32_t)AUDIO_VIS_WAVEFORM_Y;
  }
  if (y >= (int32_t)(AUDIO_VIS_WAVEFORM_Y + AUDIO_VIS_WAVEFORM_H))
  {
    y = (int32_t)(AUDIO_VIS_WAVEFORM_Y + AUDIO_VIS_WAVEFORM_H - 1U);
  }

  return (uint16_t)y;
}

static uint32_t AudioVisualizer_Abs32(int32_t value)
{
  return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

static HAL_StatusTypeDef AudioVisualizer_DrawStatus(ST7789_HandleTypeDef *display)
{
  (void)display;
  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawLabels(ST7789_HandleTypeDef *display)
{
  if (AudioVisualizer_DrawText(display, 84U, AUDIO_VIS_WAVE_LABEL_Y, "TIME DOMAIN", ST7789_CYAN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawPanel(display,
                                AUDIO_VIS_WAVEFORM_Y,
                                AUDIO_VIS_WAVEFORM_H,
                                AUDIO_VIS_DIM_CYAN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawGrid(display,
                               AUDIO_VIS_WAVEFORM_Y,
                               AUDIO_VIS_WAVEFORM_H) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawAxes(display,
                               AUDIO_VIS_WAVEFORM_Y,
                               AUDIO_VIS_WAVEFORM_H,
                               ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawScopeTimebaseLabel(display) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawFftReadout(display) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawPanel(display,
                                AUDIO_VIS_SPECTRUM_Y,
                                AUDIO_VIS_SPECTRUM_H,
                                AUDIO_VIS_DIM_GREEN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawGrid(display,
                               AUDIO_VIS_SPECTRUM_Y,
                               AUDIO_VIS_SPECTRUM_H) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawAxes(display,
                               AUDIO_VIS_SPECTRUM_Y,
                               AUDIO_VIS_SPECTRUM_H,
                               ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawFrequencyTicks(display, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawText(display, 108U, AUDIO_VIS_FREQ_AXIS_Y, "FREQ", ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawScopeTimebaseLabel(
    ST7789_HandleTypeDef *display)
{
  const char *label;

  if (AudioVisualizer_EffectiveScopeMode() ==
      AUDIO_VIS_SCOPE_MODE_TRIGGERED)
  {
    if (audio_input_source == AUDIO_INPUT_SOURCE_AUX)
    {
      label = (aux_analysis_last_serviced_tap ==
               AUX_ANALYSIS_TAP_RAW_ADC) ? "ADC 7.4MS" : "DSP 7.4MS";
    }
    else
    {
      label = "I2S 7.4MS";
    }
  }
  else
  {
    label = "1S ENV";
  }

  return AudioVisualizer_DrawText(
      display,
      184U,
      AUDIO_VIS_WAVE_LABEL_Y,
      label,
      ST7789_GRAY);
}

static HAL_StatusTypeDef AudioVisualizer_DrawLevelBar(ST7789_HandleTypeDef *display)
{
  (void)display;
  audio_vis_level_bar_width = audio_sample_signal_smooth;
  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawSpectrum(ST7789_HandleTypeDef *display)
{
  const uint16_t plot_x = AUDIO_VIS_SCOPE_PLOT_X;
  const uint16_t plot_w = AUDIO_VIS_SCOPE_SAMPLE_COUNT;
  uint32_t now = HAL_GetTick();
  uint32_t draw_period_ms = audio_vis_spectrum_draw_period_ms;
  uint8_t axis_changed =
      (audio_vis_fft_axis_change_drawn != audio_fft_view_change_count) ?
      1U : 0U;

  if (draw_period_ms == 0U)
  {
    draw_period_ms = AUDIO_VIS_DEFAULT_FFT_DRAW_MS;
  }

  if ((audio_vis_spectrum_last_draw_tick != 0U) &&
      ((now - audio_vis_spectrum_last_draw_tick) < draw_period_ms))
  {
    audio_vis_spectrum_skip_count++;
    return HAL_OK;
  }

  audio_vis_spectrum_last_draw_tick = now;

  if (axis_changed != 0U)
  {
    if (ST7789_FillRect(
            display,
            0U,
            (uint16_t)(AUDIO_VIS_SPECTRUM_Y + AUDIO_VIS_SPECTRUM_H),
            ST7789_WIDTH,
            (uint16_t)((AUDIO_VIS_TICK_LABEL_Y + 7U) -
                       (AUDIO_VIS_SPECTRUM_Y + AUDIO_VIS_SPECTRUM_H)),
            ST7789_BLACK) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (ST7789_DrawHLine(
            display,
            0U,
            (uint16_t)(AUDIO_VIS_SPECTRUM_Y + AUDIO_VIS_SPECTRUM_H),
            ST7789_WIDTH,
            AUDIO_VIS_DIM_GREEN) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if (ST7789_FillRect(display,
                      plot_x,
                      (uint16_t)(AUDIO_VIS_SPECTRUM_Y + 1U),
                      plot_w,
                      (uint16_t)(AUDIO_VIS_SPECTRUM_H - 1U),
                      ST7789_BLACK) != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (uint32_t bin = 0U; bin < AUDIO_VIS_SPECTRUM_BINS; bin++)
  {
    uint16_t h = spectrum_draw_height[bin];
    uint16_t x = (uint16_t)(plot_x +
        ((bin * plot_w) / AUDIO_VIS_SPECTRUM_BINS));
    uint16_t next_x = (uint16_t)(plot_x +
        (((bin + 1U) * plot_w) / AUDIO_VIS_SPECTRUM_BINS));
    uint16_t bar_w = (uint16_t)(next_x - x);
    uint16_t base_y = AUDIO_VIS_SPECTRUM_Y + AUDIO_VIS_SPECTRUM_H;
    uint16_t bar_color;

    if (h == 0U)
    {
      h = 1U;
    }
    if (h >= AUDIO_VIS_SPECTRUM_H)
    {
      h = AUDIO_VIS_SPECTRUM_H - 1U;
    }
    bar_color = (h > ((AUDIO_VIS_SPECTRUM_H * 2U) / 3U)) ?
                ST7789_YELLOW : ST7789_GREEN;

    /* A one-pixel gutter separates adjacent frequency blocks. */
    if ((bin < (AUDIO_VIS_SPECTRUM_BINS - 1U)) && (bar_w > 1U))
    {
      bar_w--;
    }

    if (h != 0U)
    {
      uint16_t y = (uint16_t)(base_y - h);

      if (ST7789_FillRect(display, x, y, bar_w, h, bar_color) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }

    spectrum_prev_height[bin] = h;
  }

  if (AudioVisualizer_DrawGrid(display,
                               AUDIO_VIS_SPECTRUM_Y,
                               AUDIO_VIS_SPECTRUM_H) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawAxes(display,
                               AUDIO_VIS_SPECTRUM_Y,
                               AUDIO_VIS_SPECTRUM_H,
                               ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawFftReadout(display) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((axis_changed != 0U) &&
      (AudioVisualizer_DrawFrequencyTicks(display, 1U) != HAL_OK))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawDemoWaveform(ST7789_HandleTypeDef *display)
{
  uint16_t center_y = AUDIO_VIS_WAVEFORM_Y + (AUDIO_VIS_WAVEFORM_H / 2U);
  static uint16_t last_y = 0U;
  uint32_t x = audio_vis_scope_x;
  uint16_t y_min_seen = AUDIO_VIS_WAVEFORM_Y + AUDIO_VIS_WAVEFORM_H;
  uint16_t y_max_seen = AUDIO_VIS_WAVEFORM_Y;

  if (last_y == 0U)
  {
    last_y = center_y;
  }

  if ((x < AUDIO_VIS_SCOPE_PLOT_X) ||
      (x >= (AUDIO_VIS_SCOPE_PLOT_X + AUDIO_VIS_SCOPE_SAMPLE_COUNT)))
  {
    x = AUDIO_VIS_SCOPE_PLOT_X;
  }

  for (uint32_t col = 0U; col < AUDIO_VIS_MAX_DRAW_COLUMNS; col++)
  {
    uint32_t phase = (audio_vis_demo_phase + col) & 63U;
    uint32_t phase_fast = ((audio_vis_demo_phase * 3U) + (col * 5U)) & 31U;
    int32_t triangle;
    int32_t triangle_fast;
    int32_t sample;
    uint16_t y0;
    uint16_t y1;
    uint16_t h;

    if (phase < 32U)
    {
      triangle = (int32_t)phase - 16;
    }
    else
    {
      triangle = 48 - (int32_t)phase;
    }

    if (phase_fast < 16U)
    {
      triangle_fast = (int32_t)phase_fast - 8;
    }
    else
    {
      triangle_fast = 24 - (int32_t)phase_fast;
    }

    sample = (triangle * 1100) + (triangle_fast * 850);
    y0 = AudioVisualizer_MapToY(sample, 32768U);
    y1 = last_y;
    last_y = y0;

    if (y1 < y0)
    {
      uint16_t temp = y0;
      y0 = y1;
      y1 = temp;
    }

    if (y0 < y_min_seen)
    {
      y_min_seen = y0;
    }
    if (y1 > y_max_seen)
    {
      y_max_seen = y1;
    }

    h = (uint16_t)((y1 - y0) + 1U);
    if (h < 3U)
    {
      h = 3U;
    }

    if (AudioVisualizer_RestoreWaveformColumn(display, (uint16_t)x) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if (ST7789_DrawVLine(display, (uint16_t)x, y0, h, ST7789_MAGENTA) != HAL_OK)
    {
      return HAL_ERROR;
    }

    x++;
    if (x >= (AUDIO_VIS_SCOPE_PLOT_X + AUDIO_VIS_SCOPE_SAMPLE_COUNT))
    {
      x = AUDIO_VIS_SCOPE_PLOT_X;
      audio_vis_scope_wrap_count++;
    }
  }

  audio_vis_demo_phase += AUDIO_VIS_MAX_DRAW_COLUMNS;
  audio_vis_scope_x = x;
  audio_vis_scope_columns_per_draw = AUDIO_VIS_MAX_DRAW_COLUMNS;
  audio_vis_scope_cursor_x = x;
  audio_vis_live_peak = 32768U;
  audio_vis_live_sample_count += AUDIO_VIS_MAX_DRAW_COLUMNS;
  audio_vis_live_y_span = (uint32_t)(y_max_seen - y_min_seen);
  audio_vis_live_draw_count++;
  audio_vis_demo_draw_count++;
  audio_vis_incremental_draw_count++;

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawTriggeredWaveform(ST7789_HandleTypeDef *display)
{
  uint32_t snapshot_index;
  uint32_t snapshot_sequence;
  uint32_t scale;
  uint16_t y_min_seen = AUDIO_VIS_SCOPE_PLOT_Y + AUDIO_VIS_SCOPE_PLOT_H;
  uint16_t y_max_seen = AUDIO_VIS_SCOPE_PLOT_Y;
  const uint16_t center_y =
      AUDIO_VIS_WAVEFORM_Y + (AUDIO_VIS_WAVEFORM_H / 2U);

  if ((audio_vis_trigger_snapshot_valid == 0U) ||
      (audio_vis_trigger_snapshot_ready == 0U) ||
      (audio_vis_trigger_sequence == audio_vis_trigger_drawn_sequence))
  {
    audio_vis_scope_empty_draw_count++;
    return HAL_OK;
  }

  /* The producer will not start another capture while this flag is set. */
  audio_vis_trigger_draw_active = 1U;
  __DMB();
  snapshot_index = audio_vis_trigger_published_index;
  snapshot_sequence = audio_vis_trigger_sequence;
  scale = audio_vis_trigger_published_scale[snapshot_index];
  if (scale == 0U)
  {
    scale = 1U;
  }

  for (uint32_t sample = 0U; sample < AUDIO_VIS_SCOPE_SAMPLE_COUNT; sample++)
  {
    uint16_t y = AudioVisualizer_MapToY(
        audio_vis_trigger_snapshot[snapshot_index][sample] -
        audio_vis_trigger_snapshot_mean,
        scale);

    if (y < AUDIO_VIS_SCOPE_PLOT_Y)
    {
      y = AUDIO_VIS_SCOPE_PLOT_Y;
    }
    else if (y >= (AUDIO_VIS_SCOPE_PLOT_Y + AUDIO_VIS_SCOPE_PLOT_H))
    {
      y = AUDIO_VIS_SCOPE_PLOT_Y + AUDIO_VIS_SCOPE_PLOT_H - 1U;
    }
    audio_vis_trigger_draw_y[sample] = y;
    if (y < y_min_seen)
    {
      y_min_seen = y;
    }
    if (y > y_max_seen)
    {
      y_max_seen = y;
    }
  }

  for (uint32_t tile_sample = 0U;
       tile_sample < AUDIO_VIS_SCOPE_SAMPLE_COUNT;
       tile_sample += AUDIO_VIS_SCOPE_TILE_W)
  {
    for (uint32_t y = 0U; y < AUDIO_VIS_SCOPE_PLOT_H; y++)
    {
      uint32_t absolute_y = AUDIO_VIS_SCOPE_PLOT_Y + y;

      for (uint32_t x = 0U; x < AUDIO_VIS_SCOPE_TILE_W; x++)
      {
        uint32_t absolute_x = AUDIO_VIS_SCOPE_PLOT_X + tile_sample + x;
        uint16_t color = ST7789_BLACK;

        if (absolute_y == center_y)
        {
          color = ST7789_GRAY;
        }
        else if (((absolute_x % 40U) == 0U) ||
                 (absolute_y ==
                  (AUDIO_VIS_WAVEFORM_Y + (AUDIO_VIS_WAVEFORM_H / 4U))) ||
                 (absolute_y ==
                  (AUDIO_VIS_WAVEFORM_Y +
                   ((3U * AUDIO_VIS_WAVEFORM_H) / 4U))))
        {
          color = AUDIO_VIS_GRID_COLOR;
        }
        AudioVisualizer_TriggeredTileSet(x, y, color);
      }
    }

    for (uint32_t x = 0U; x < AUDIO_VIS_SCOPE_TILE_W; x++)
    {
      uint32_t sample = tile_sample + x;
      uint16_t current_y = audio_vis_trigger_draw_y[sample];
      uint16_t previous_y = (sample == 0U) ?
                            current_y : audio_vis_trigger_draw_y[sample - 1U];
      uint16_t y0 = (previous_y < current_y) ? previous_y : current_y;
      uint16_t y1 = (previous_y > current_y) ? previous_y : current_y;

      for (uint16_t y = y0; y <= y1; y++)
      {
        AudioVisualizer_TriggeredTileSet(
            x, (uint32_t)(y - AUDIO_VIS_SCOPE_PLOT_Y), ST7789_CYAN);
      }
    }

    if (ST7789_DrawRGB565Bitmap(
            display,
            (uint16_t)(AUDIO_VIS_SCOPE_PLOT_X + tile_sample),
            AUDIO_VIS_SCOPE_PLOT_Y,
            AUDIO_VIS_SCOPE_TILE_W,
            AUDIO_VIS_SCOPE_PLOT_H,
            audio_vis_trigger_tile,
            sizeof(audio_vis_trigger_tile)) != HAL_OK)
    {
      audio_vis_trigger_draw_active = 0U;
      return HAL_ERROR;
    }
  }

  /* Refresh the header when the selected AUX analysis tap changes. */
  if (AudioVisualizer_DrawScopeTimebaseLabel(display) != HAL_OK)
  {
    audio_vis_trigger_draw_active = 0U;
    return HAL_ERROR;
  }

  audio_vis_scope_columns_per_draw = AUDIO_VIS_SCOPE_SAMPLE_COUNT;
  audio_vis_scope_x =
      AUDIO_VIS_SCOPE_PLOT_X + AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES;
  audio_vis_scope_cursor_x = audio_vis_scope_x;
  audio_vis_scope_elapsed_ms =
      (AUDIO_VIS_SCOPE_SAMPLE_COUNT * 1000U) / AUDIO_VIS_SAMPLE_RATE_HZ;
  audio_vis_live_y_span = (uint32_t)(y_max_seen - y_min_seen);
  audio_vis_live_draw_count++;
  audio_vis_incremental_draw_count++;
  audio_vis_scope_wrap_count++;

  audio_vis_trigger_drawn_sequence = snapshot_sequence;
  __DMB();
  audio_vis_trigger_snapshot_ready =
      (audio_vis_trigger_sequence != snapshot_sequence) ? 1U : 0U;
  __DMB();
  audio_vis_trigger_draw_active = 0U;

  return HAL_OK;
}

static void AudioVisualizer_TriggeredTileSet(uint32_t x,
                                             uint32_t y,
                                             uint16_t color)
{
  uint32_t offset;

  if ((x >= AUDIO_VIS_SCOPE_TILE_W) || (y >= AUDIO_VIS_SCOPE_PLOT_H))
  {
    return;
  }

  offset = ((y * AUDIO_VIS_SCOPE_TILE_W) + x) * 2U;
  audio_vis_trigger_tile[offset] = (uint8_t)(color >> 8U);
  audio_vis_trigger_tile[offset + 1U] = (uint8_t)(color & 0xFFU);
}

static HAL_StatusTypeDef AudioVisualizer_DrawLiveSampleWaveform(ST7789_HandleTypeDef *display)
{
  uint32_t columns;
  uint32_t x;
  uint16_t y_min_seen = AUDIO_VIS_WAVEFORM_Y + AUDIO_VIS_WAVEFORM_H;
  uint16_t y_max_seen = AUDIO_VIS_WAVEFORM_Y;

  __disable_irq();
  while (audio_vis_scope_pending_columns > AUDIO_VIS_MAX_PENDING_COLUMNS)
  {
    audio_vis_scope_pending_read++;
    if (audio_vis_scope_pending_read >= AUDIO_VIS_SCOPE_SAMPLE_COUNT)
    {
      audio_vis_scope_pending_read = 0U;
    }
    audio_vis_scope_pending_columns--;
    audio_vis_scope_dropped_columns++;
  }

  columns = audio_vis_scope_pending_columns;
  if (columns > AUDIO_VIS_MAX_DRAW_COLUMNS)
  {
    columns = AUDIO_VIS_MAX_DRAW_COLUMNS;
  }
  for (uint32_t i = 0U; i < columns; i++)
  {
    audio_vis_scope_draw_y_min[i] = audio_vis_scope_pending_y_min[audio_vis_scope_pending_read];
    audio_vis_scope_draw_y_max[i] = audio_vis_scope_pending_y_max[audio_vis_scope_pending_read];
    audio_vis_scope_pending_read++;
    if (audio_vis_scope_pending_read >= AUDIO_VIS_SCOPE_SAMPLE_COUNT)
    {
      audio_vis_scope_pending_read = 0U;
    }
  }
  audio_vis_scope_pending_columns -= columns;
  __enable_irq();

  if (columns == 0U)
  {
    uint16_t center_y = AUDIO_VIS_WAVEFORM_Y + (AUDIO_VIS_WAVEFORM_H / 2U);

    columns = AUDIO_VIS_MAX_DRAW_COLUMNS;
    for (uint32_t i = 0U; i < columns; i++)
    {
      audio_vis_scope_draw_y_min[i] = (uint16_t)(center_y - 1U);
      audio_vis_scope_draw_y_max[i] = (uint16_t)(center_y + 1U);
    }
    audio_vis_scope_empty_draw_count++;
  }

  audio_vis_scope_columns_per_draw = columns;

  x = audio_vis_scope_x;
  if ((x < AUDIO_VIS_SCOPE_PLOT_X) ||
      (x >= (AUDIO_VIS_SCOPE_PLOT_X + AUDIO_VIS_SCOPE_SAMPLE_COUNT)))
  {
    x = AUDIO_VIS_SCOPE_PLOT_X;
  }

  for (uint32_t col = 0U; col < columns; col++)
  {
    uint16_t y0 = audio_vis_scope_draw_y_min[col];
    uint16_t y1 = audio_vis_scope_draw_y_max[col];
    uint16_t h;

    if (y1 < y0)
    {
      uint16_t temp = y0;
      y0 = y1;
      y1 = temp;
    }

    if (y0 < y_min_seen)
    {
      y_min_seen = y0;
    }
    if (y1 > y_max_seen)
    {
      y_max_seen = y1;
    }

    h = (uint16_t)((y1 - y0) + 1U);
    if (h < 3U)
    {
      if (y0 > AUDIO_VIS_WAVEFORM_Y)
      {
        y0--;
      }
      h = 3U;
    }

    if (AudioVisualizer_RestoreWaveformColumn(display, (uint16_t)x) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if (ST7789_DrawVLine(display, (uint16_t)x, y0, h, ST7789_CYAN) != HAL_OK)
    {
      return HAL_ERROR;
    }

    x++;
    if (x >= (AUDIO_VIS_SCOPE_PLOT_X + AUDIO_VIS_SCOPE_SAMPLE_COUNT))
    {
      x = AUDIO_VIS_SCOPE_PLOT_X;
      audio_vis_scope_wrap_count++;
    }
  }

  audio_vis_scope_x = x;
  audio_vis_scope_cursor_x = x;
  audio_vis_live_y_span = (uint32_t)(y_max_seen - y_min_seen);
  audio_vis_live_draw_count++;
  audio_vis_incremental_draw_count++;

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawWaveformCenter(ST7789_HandleTypeDef *display)
{
  return ST7789_DrawHLine(display,
                          AUDIO_VIS_SCOPE_PLOT_X,
                          AUDIO_VIS_WAVEFORM_Y + (AUDIO_VIS_WAVEFORM_H / 2U),
                          AUDIO_VIS_SCOPE_SAMPLE_COUNT,
                          ST7789_GRAY);
}

static void AudioVisualizer_UpdateAutoScale(uint32_t peak)
{
  uint32_t target;
  uint32_t scale;

  if (audio_vis_auto_gain_enable == 0U)
  {
    return;
  }

  target = peak * 2U;
  if (target < AUDIO_VIS_MIN_PEAK_SCALE)
  {
    target = AUDIO_VIS_MIN_PEAK_SCALE;
  }
  if (target > AUDIO_VIS_MAX_PEAK_SCALE)
  {
    target = AUDIO_VIS_MAX_PEAK_SCALE;
  }

  scale = audio_vis_scope_peak_scale;
  if (scale == 0U)
  {
    scale = target;
  }
  else if (target > scale)
  {
    scale = ((scale * 3U) + target) / 4U;
  }
  else
  {
    scale = ((scale * 31U) + target) / 32U;
  }

  if (scale < AUDIO_VIS_MIN_PEAK_SCALE)
  {
    scale = AUDIO_VIS_MIN_PEAK_SCALE;
  }
  if (scale > AUDIO_VIS_MAX_PEAK_SCALE)
  {
    scale = AUDIO_VIS_MAX_PEAK_SCALE;
  }

  audio_vis_scope_peak_scale = scale;
}

static void AudioVisualizer_ScopePushSamples(const int32_t *samples,
                                             uint32_t len,
                                             int32_t dc_offset)
{
  uint32_t sweep_ms = audio_vis_scope_sweep_ms;
  uint32_t samples_per_sweep;
  uint32_t pixels_per_sample_q16;
  uint32_t peak = audio_vis_scope_peak_scale;

  if ((samples == NULL) || (len == 0U))
  {
    return;
  }

  if (sweep_ms < 100U)
  {
    sweep_ms = 100U;
  }

  samples_per_sweep = (AUDIO_VIS_SAMPLE_RATE_HZ * sweep_ms) / 1000U;
  if (samples_per_sweep < AUDIO_VIS_SCOPE_SAMPLE_COUNT)
  {
    samples_per_sweep = AUDIO_VIS_SCOPE_SAMPLE_COUNT;
  }

  pixels_per_sample_q16 =
      (AUDIO_VIS_SCOPE_SAMPLE_COUNT * AUDIO_VIS_SCOPE_Q16) /
      samples_per_sweep;
  if (pixels_per_sample_q16 == 0U)
  {
    pixels_per_sample_q16 = 1U;
  }

  if (peak == 0U)
  {
    peak = 1U;
  }
  audio_vis_live_peak = peak;
  audio_vis_live_sample_count += len;

  for (uint32_t i = 0U; i < len; i++)
  {
    int32_t sample = samples[i] - dc_offset;

    if (sample < audio_vis_scope_column_min)
    {
      audio_vis_scope_column_min = sample;
    }
    if (sample > audio_vis_scope_column_max)
    {
      audio_vis_scope_column_max = sample;
    }

    audio_vis_scope_column_accum += pixels_per_sample_q16;
    while (audio_vis_scope_column_accum >= AUDIO_VIS_SCOPE_Q16)
    {
      int32_t min_sample = audio_vis_scope_column_min;
      int32_t max_sample = audio_vis_scope_column_max;

      if (min_sample > max_sample)
      {
        min_sample = 0;
        max_sample = 0;
      }

      AudioVisualizer_ScopePushColumn(min_sample, max_sample);

      audio_vis_scope_column_min = INT32_MAX;
      audio_vis_scope_column_max = INT32_MIN;
      audio_vis_scope_column_accum -= AUDIO_VIS_SCOPE_Q16;
    }
  }
}

static void AudioVisualizer_ScopePushColumn(int32_t min_sample,
                                            int32_t max_sample)
{
  uint32_t peak = audio_vis_scope_peak_scale;
  uint16_t y0;
  uint16_t y1;

  if (peak == 0U)
  {
    peak = 1U;
  }

  y0 = AudioVisualizer_MapToY(max_sample, peak);
  y1 = AudioVisualizer_MapToY(min_sample, peak);

  if (audio_vis_scope_pending_columns >= AUDIO_VIS_SCOPE_SAMPLE_COUNT)
  {
    audio_vis_scope_pending_read++;
    if (audio_vis_scope_pending_read >= AUDIO_VIS_SCOPE_SAMPLE_COUNT)
    {
      audio_vis_scope_pending_read = 0U;
    }
    audio_vis_scope_pending_columns--;
    audio_vis_scope_dropped_columns++;
  }

  audio_vis_scope_pending_y_min[audio_vis_scope_pending_write] = y0;
  audio_vis_scope_pending_y_max[audio_vis_scope_pending_write] = y1;
  audio_vis_scope_pending_write++;
  if (audio_vis_scope_pending_write >= AUDIO_VIS_SCOPE_SAMPLE_COUNT)
  {
    audio_vis_scope_pending_write = 0U;
  }

  audio_vis_scope_pending_columns++;
  audio_vis_scope_generated_columns++;
}

static HAL_StatusTypeDef AudioVisualizer_DrawPanel(ST7789_HandleTypeDef *display,
                                                   uint16_t y,
                                                   uint16_t h,
                                                   uint16_t border_color)
{
  if (ST7789_DrawHLine(display, 0U, y, ST7789_WIDTH, border_color) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (ST7789_DrawHLine(display, 0U, (uint16_t)(y + h), ST7789_WIDTH, border_color) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (ST7789_DrawVLine(display, 0U, y, (uint16_t)(h + 1U), border_color) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (ST7789_DrawVLine(display, (uint16_t)(ST7789_WIDTH - 1U), y, (uint16_t)(h + 1U), border_color) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawGrid(ST7789_HandleTypeDef *display,
                                                  uint16_t y,
                                                  uint16_t h)
{
  if (y == AUDIO_VIS_SPECTRUM_Y)
  {
    uint32_t tick_count = audio_fft_axis_tick_count;

    if (tick_count > AUDIO_FFT_AXIS_TICK_COUNT)
    {
      tick_count = AUDIO_FFT_AXIS_TICK_COUNT;
    }
    for (uint32_t tick = 1U; (tick + 1U) < tick_count; tick++)
    {
      uint16_t x = AudioVisualizer_FrequencyToSpectrumX(
          audio_fft_axis_tick_hz[tick]);

      if (ST7789_DrawVLine(display,
                           x,
                           (uint16_t)(y + 1U),
                           (uint16_t)(h - 1U),
                           AUDIO_VIS_GRID_COLOR) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
  }
  else
  {
    for (uint16_t x = 40U; x < ST7789_WIDTH; x = (uint16_t)(x + 40U))
    {
      if (ST7789_DrawVLine(display,
                           x,
                           (uint16_t)(y + 1U),
                           (uint16_t)(h - 1U),
                           AUDIO_VIS_GRID_COLOR) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
  }

  for (uint16_t row = 1U; row < 4U; row++)
  {
    uint16_t grid_y = (uint16_t)(y + ((row * h) / 4U));
    if (ST7789_DrawHLine(display, 1U, grid_y, (uint16_t)(ST7789_WIDTH - 2U), AUDIO_VIS_GRID_COLOR) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawAxes(ST7789_HandleTypeDef *display,
                                                  uint16_t y,
                                                  uint16_t h,
                                                  uint16_t color)
{
  uint16_t bottom_y = (uint16_t)(y + h);

  for (uint16_t row = 0U; row <= 4U; row++)
  {
    uint16_t tick_y = (uint16_t)(y + ((row * h) / 4U));
    if (ST7789_DrawHLine(display,
                         0U,
                         tick_y,
                         4U,
                         color) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if (y == AUDIO_VIS_SPECTRUM_Y)
  {
    uint32_t tick_count = audio_fft_axis_tick_count;

    if (tick_count > AUDIO_FFT_AXIS_TICK_COUNT)
    {
      tick_count = AUDIO_FFT_AXIS_TICK_COUNT;
    }
    for (uint32_t tick = 0U; tick < tick_count; tick++)
    {
      uint16_t x = AudioVisualizer_FrequencyToSpectrumX(
          audio_fft_axis_tick_hz[tick]);

      if (ST7789_DrawVLine(display,
                           x,
                           bottom_y,
                           4U,
                           color) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
  }
  else
  {
    for (uint16_t x = 40U; x < ST7789_WIDTH; x = (uint16_t)(x + 40U))
    {
      if (ST7789_DrawVLine(display,
                           x,
                           (uint16_t)(bottom_y - 3U),
                           4U,
                           color) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
  }

  return HAL_OK;
}

static uint16_t AudioVisualizer_FrequencyToSpectrumX(uint32_t frequency_hz)
{
  const uint32_t plot_x = AUDIO_VIS_SCOPE_PLOT_X;
  const uint32_t plot_w = AUDIO_VIS_SCOPE_SAMPLE_COUNT;

  if (frequency_hz <= audio_fft_band_edge_hz[0])
  {
    return (uint16_t)plot_x;
  }
  if (frequency_hz >= audio_fft_band_edge_hz[AUDIO_FFT_DISPLAY_BINS])
  {
    return (uint16_t)(plot_x + plot_w - 1U);
  }

  for (uint32_t band = 0U; band < AUDIO_FFT_DISPLAY_BINS; band++)
  {
    uint32_t low_hz = audio_fft_band_edge_hz[band];
    uint32_t high_hz = audio_fft_band_edge_hz[band + 1U];

    if ((frequency_hz >= low_hz) && (frequency_hz <= high_hz))
    {
      uint32_t segment_x0 = plot_x +
          ((band * plot_w) / AUDIO_FFT_DISPLAY_BINS);
      uint32_t segment_x1 = plot_x +
          (((band + 1U) * plot_w) / AUDIO_FFT_DISPLAY_BINS);
      uint32_t x = segment_x0;

      if (high_hz > low_hz)
      {
        x += (uint32_t)(((uint64_t)(frequency_hz - low_hz) *
                         (segment_x1 - segment_x0)) /
                        (high_hz - low_hz));
      }
      if (x >= (plot_x + plot_w))
      {
        x = plot_x + plot_w - 1U;
      }
      return (uint16_t)x;
    }
  }

  return (uint16_t)(plot_x + plot_w - 1U);
}

static uint32_t AudioVisualizer_FormatFrequencyLabel(char *text,
                                                     uint32_t frequency_hz)
{
  char reversed[5U];
  uint32_t count = 0U;

  if (text == NULL)
  {
    return 0U;
  }
  if ((frequency_hz >= 1000U) &&
      ((frequency_hz % 1000U) == 0U) &&
      ((frequency_hz / 1000U) <= 99U))
  {
    uint32_t khz = frequency_hz / 1000U;

    if (khz < 10U)
    {
      text[0] = (char)('0' + khz);
      text[1] = 'K';
      text[2] = '\0';
      return 2U;
    }
    text[0] = (char)('0' + (khz / 10U));
    text[1] = (char)('0' + (khz % 10U));
    text[2] = 'K';
    text[3] = '\0';
    return 3U;
  }

  do
  {
    reversed[count] = (char)('0' + (frequency_hz % 10U));
    frequency_hz /= 10U;
    count++;
  } while ((frequency_hz != 0U) && (count < sizeof(reversed)));

  for (uint32_t i = 0U; i < count; i++)
  {
    text[i] = reversed[count - 1U - i];
  }
  text[count] = '\0';
  return count;
}

static HAL_StatusTypeDef AudioVisualizer_DrawFrequencyTicks(ST7789_HandleTypeDef *display,
                                                            uint8_t draw_labels)
{
  uint32_t tick_count = audio_fft_axis_tick_count;

  if (tick_count > AUDIO_FFT_AXIS_TICK_COUNT)
  {
    tick_count = AUDIO_FFT_AXIS_TICK_COUNT;
  }

  for (uint32_t i = 0U; i < tick_count; i++)
  {
    char tick_label[6U];
    uint32_t label_length = AudioVisualizer_FormatFrequencyLabel(
        tick_label, audio_fft_axis_tick_hz[i]);
    uint32_t label_width = label_length * 6U;
    uint16_t tick_x = AudioVisualizer_FrequencyToSpectrumX(
        audio_fft_axis_tick_hz[i]);
    uint16_t label_x;

    if (tick_x > (label_width / 2U))
    {
      label_x = (uint16_t)(tick_x - (label_width / 2U));
    }
    else
    {
      label_x = 0U;
    }
    if (((uint32_t)label_x + label_width) > ST7789_WIDTH)
    {
      label_x = (uint16_t)(ST7789_WIDTH - label_width);
    }

    if (ST7789_DrawVLine(display,
                         tick_x,
                         (uint16_t)(AUDIO_VIS_SPECTRUM_Y +
                                    AUDIO_VIS_SPECTRUM_H),
                         4U,
                         ST7789_GRAY) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if ((draw_labels != 0U) &&
        (AudioVisualizer_DrawText(display,
                                  label_x,
                                  AUDIO_VIS_TICK_LABEL_Y,
                                  tick_label,
                                  ST7789_GRAY) != HAL_OK))
    {
      return HAL_ERROR;
    }
  }

  if (draw_labels != 0U)
  {
    audio_vis_fft_axis_change_drawn = audio_fft_view_change_count;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawFftReadout(ST7789_HandleTypeDef *display)
{
  char range_text[16U];
  char minimum_text[6U];
  char maximum_text[6U];
  char peak_text[11U] = "PK=     HZ";
  char db_text[7] = "DB=-00";
  uint32_t frequency_hz = audio_fft_peak_freq_hz;
  int32_t peak_db_x10 = audio_fft_peak_db_x10;
  uint32_t peak_db;
  uint32_t range_index = 0U;
  uint32_t minimum_length;
  uint32_t maximum_length;
  int32_t digit_index = 7;

  minimum_length = AudioVisualizer_FormatFrequencyLabel(
      minimum_text, audio_fft_view_min_hz);
  maximum_length = AudioVisualizer_FormatFrequencyLabel(
      maximum_text, audio_fft_view_max_hz);
  range_text[range_index++] = 'F';
  range_text[range_index++] = 'F';
  range_text[range_index++] = 'T';
  range_text[range_index++] = ' ';
  for (uint32_t i = 0U; i < minimum_length; i++)
  {
    range_text[range_index++] = minimum_text[i];
  }
  range_text[range_index++] = '-';
  for (uint32_t i = 0U; i < maximum_length; i++)
  {
    range_text[range_index++] = maximum_text[i];
  }
  range_text[range_index] = '\0';

  if (frequency_hz > AUDIO_FFT_MAX_FREQ_HZ)
  {
    frequency_hz = AUDIO_FFT_MAX_FREQ_HZ;
  }
  do
  {
    peak_text[digit_index] = (char)('0' + (frequency_hz % 10U));
    frequency_hz /= 10U;
    digit_index--;
  } while ((frequency_hz != 0U) && (digit_index >= 3));

  if (peak_db_x10 >= 0)
  {
    peak_db = 0U;
  }
  else
  {
    peak_db = (uint32_t)((-peak_db_x10 + 5) / 10);
  }
  if (peak_db > 99U)
  {
    peak_db = 99U;
  }
  db_text[4] = (char)('0' + ((peak_db / 10U) % 10U));
  db_text[5] = (char)('0' + (peak_db % 10U));

  if (ST7789_FillRect(display,
                      0U,
                      AUDIO_VIS_FREQ_LABEL_Y,
                      ST7789_WIDTH,
                      7U,
                      ST7789_BLACK) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (AudioVisualizer_DrawText(display,
                               2U,
                               AUDIO_VIS_FREQ_LABEL_Y,
                               range_text,
                               ST7789_GREEN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (AudioVisualizer_DrawText(display,
                               92U,
                               AUDIO_VIS_FREQ_LABEL_Y,
                               peak_text,
                               ST7789_GREEN) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return AudioVisualizer_DrawText(display,
                                  158U,
                                  AUDIO_VIS_FREQ_LABEL_Y,
                                  db_text,
                                  ST7789_GREEN);
}

static HAL_StatusTypeDef AudioVisualizer_RestoreWaveformColumn(ST7789_HandleTypeDef *display,
                                                               uint16_t x)
{
  if (ST7789_DrawVLine(display,
                       x,
                       AUDIO_VIS_WAVEFORM_Y,
                       AUDIO_VIS_WAVEFORM_H,
                       ST7789_BLACK) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((x == 0U) || (x == (ST7789_WIDTH - 1U)))
  {
    if (ST7789_DrawVLine(display,
                         x,
                         AUDIO_VIS_WAVEFORM_Y,
                         (uint16_t)(AUDIO_VIS_WAVEFORM_H + 1U),
                         AUDIO_VIS_DIM_CYAN) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if (ST7789_DrawPixel(display,
                       x,
                       (uint16_t)(AUDIO_VIS_WAVEFORM_Y + (AUDIO_VIS_WAVEFORM_H / 2U)),
                       ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (ST7789_DrawPixel(display, x, AUDIO_VIS_WAVEFORM_Y, AUDIO_VIS_DIM_CYAN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (ST7789_DrawPixel(display,
                       x,
                       (uint16_t)(AUDIO_VIS_WAVEFORM_Y + AUDIO_VIS_WAVEFORM_H),
                       AUDIO_VIS_DIM_CYAN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawText(ST7789_HandleTypeDef *display,
                                                  uint16_t x,
                                                  uint16_t y,
                                                  const char *text,
                                                  uint16_t color)
{
  if ((display == NULL) || (text == NULL))
  {
    return HAL_ERROR;
  }

  while (*text != '\0')
  {
    if (AudioVisualizer_DrawChar(display, x, y, *text, color) != HAL_OK)
    {
      return HAL_ERROR;
    }
    x = (uint16_t)(x + 6U);
    text++;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawChar(ST7789_HandleTypeDef *display,
                                                  uint16_t x,
                                                  uint16_t y,
                                                  char ch,
                                                  uint16_t color)
{
  const uint8_t *glyph = AudioVisualizer_Font5x7(ch);
  uint8_t bitmap[6U * 7U * 2U];

  if (glyph == NULL)
  {
    return HAL_OK;
  }

  for (uint32_t row = 0U; row < 7U; row++)
  {
    for (uint32_t col = 0U; col < 6U; col++)
    {
      uint16_t pixel_color =
          ((col < 5U) && ((glyph[col] & (1U << row)) != 0U)) ?
          color : ST7789_BLACK;
      uint32_t offset = ((row * 6U) + col) * 2U;

      bitmap[offset] = (uint8_t)(pixel_color >> 8U);
      bitmap[offset + 1U] = (uint8_t)(pixel_color & 0xFFU);
    }
  }

  return ST7789_DrawRGB565Bitmap(display,
                                 x,
                                 y,
                                 6U,
                                 7U,
                                 bitmap,
                                 sizeof(bitmap));
}

static const uint8_t *AudioVisualizer_Font5x7(char ch)
{
  static const uint8_t glyph_A[5] = {0x7EU, 0x09U, 0x09U, 0x09U, 0x7EU};
  static const uint8_t glyph_B[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U};
  static const uint8_t glyph_C[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
  static const uint8_t glyph_D[5] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
  static const uint8_t glyph_E[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
  static const uint8_t glyph_F[5] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U};
  static const uint8_t glyph_G[5] = {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU};
  static const uint8_t glyph_H[5] = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU};
  static const uint8_t glyph_I[5] = {0x41U, 0x41U, 0x7FU, 0x41U, 0x41U};
  static const uint8_t glyph_K[5] = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};
  static const uint8_t glyph_L[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
  static const uint8_t glyph_M[5] = {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU};
  static const uint8_t glyph_N[5] = {0x7FU, 0x02U, 0x04U, 0x08U, 0x7FU};
  static const uint8_t glyph_O[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
  static const uint8_t glyph_P[5] = {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U};
  static const uint8_t glyph_Q[5] = {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU};
  static const uint8_t glyph_R[5] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U};
  static const uint8_t glyph_S[5] = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
  static const uint8_t glyph_T[5] = {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U};
  static const uint8_t glyph_U[5] = {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU};
  static const uint8_t glyph_V[5] = {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU};
  static const uint8_t glyph_W[5] = {0x7FU, 0x20U, 0x18U, 0x20U, 0x7FU};
  static const uint8_t glyph_X[5] = {0x63U, 0x14U, 0x08U, 0x14U, 0x63U};
  static const uint8_t glyph_Y[5] = {0x07U, 0x08U, 0x70U, 0x08U, 0x07U};
  static const uint8_t glyph_Z[5] = {0x61U, 0x51U, 0x49U, 0x45U, 0x43U};
  static const uint8_t glyph_0[5] = {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU};
  static const uint8_t glyph_1[5] = {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U};
  static const uint8_t glyph_2[5] = {0x42U, 0x61U, 0x51U, 0x49U, 0x46U};
  static const uint8_t glyph_3[5] = {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U};
  static const uint8_t glyph_4[5] = {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U};
  static const uint8_t glyph_5[5] = {0x27U, 0x45U, 0x45U, 0x45U, 0x39U};
  static const uint8_t glyph_6[5] = {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U};
  static const uint8_t glyph_7[5] = {0x01U, 0x71U, 0x09U, 0x05U, 0x03U};
  static const uint8_t glyph_8[5] = {0x36U, 0x49U, 0x49U, 0x49U, 0x36U};
  static const uint8_t glyph_9[5] = {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU};
  static const uint8_t glyph_equal[5] = {0x14U, 0x14U, 0x14U, 0x14U, 0x14U};
  static const uint8_t glyph_slash[5] = {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};
  static const uint8_t glyph_minus[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};

  switch (ch)
  {
    case 'A': return glyph_A;
    case 'B': return glyph_B;
    case 'C': return glyph_C;
    case 'D': return glyph_D;
    case 'E': return glyph_E;
    case 'F': return glyph_F;
    case 'G': return glyph_G;
    case 'H': return glyph_H;
    case 'I': return glyph_I;
    case 'K': return glyph_K;
    case 'L': return glyph_L;
    case 'M': return glyph_M;
    case 'N': return glyph_N;
    case 'O': return glyph_O;
    case 'P': return glyph_P;
    case 'Q': return glyph_Q;
    case 'R': return glyph_R;
    case 'S': return glyph_S;
    case 'T': return glyph_T;
    case 'U': return glyph_U;
    case 'V': return glyph_V;
    case 'W': return glyph_W;
    case 'X': return glyph_X;
    case 'Y': return glyph_Y;
    case 'Z': return glyph_Z;
    case '0': return glyph_0;
    case '1': return glyph_1;
    case '2': return glyph_2;
    case '3': return glyph_3;
    case '4': return glyph_4;
    case '5': return glyph_5;
    case '6': return glyph_6;
    case '7': return glyph_7;
    case '8': return glyph_8;
    case '9': return glyph_9;
    case '=': return glyph_equal;
    case '/': return glyph_slash;
    case '-': return glyph_minus;
    default: return NULL;
  }
}

static void AudioVisualizer_LoadFftSpectrum(void)
{
  uint16_t fft_bins[AUDIO_VIS_SPECTRUM_BINS];

  if (AudioFFT_GetDisplayBins(fft_bins, AUDIO_VIS_SPECTRUM_BINS) == 0U)
  {
    audio_vis_fft_empty_count++;
    return;
  }

  for (uint32_t i = 0U; i < AUDIO_VIS_SPECTRUM_BINS; i++)
  {
    spectrum_draw_height[i] = fft_bins[i];
    audio_vis_fft_debug_bins[i] = fft_bins[i];
  }
  audio_vis_fft_draw_count++;
}

static void AudioVisualizer_LoadDemoSpectrum(void)
{
  static uint32_t phase = 0U;

  for (uint32_t i = 0U; i < AUDIO_VIS_SPECTRUM_BINS; i++)
  {
    uint32_t distance;
    uint32_t ripple;
    uint32_t h;

    if (i >= 7U)
    {
      distance = i - 7U;
    }
    else
    {
      distance = 7U - i;
    }

    h = 64U;
    if ((distance * 8U) < h)
    {
      h -= distance * 8U;
    }
    else
    {
      h = 8U;
    }

    ripple = ((phase + (i * 3U)) & 7U);
    h += ripple;

    if (h > AUDIO_VIS_SPECTRUM_H)
    {
      h = AUDIO_VIS_SPECTRUM_H;
    }

    spectrum_draw_height[i] = (uint16_t)h;
    audio_vis_fft_debug_bins[i] = (uint16_t)h;
  }

  phase++;
}

static float32_t AudioFFT_SqrtApprox(float32_t value)
{
  float32_t result;

  if (value <= 0.0f)
  {
    return 0.0f;
  }

  arm_sqrt_f32(value, &result);
  return result;
}
