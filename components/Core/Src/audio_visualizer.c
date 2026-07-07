#include "audio_visualizer.h"
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
#define AUDIO_VIS_AXIS_X       14U
#define AUDIO_VIS_GRID_COLOR   0x2104U
#define AUDIO_VIS_BORDER_COLOR 0x39E7U
#define AUDIO_VIS_DIM_CYAN     0x03EFU
#define AUDIO_VIS_DIM_GREEN    0x03E0U
#define AUDIO_VIS_SAMPLE_RATE_HZ 16000U
#define AUDIO_VIS_SCOPE_Q16     65536U
#define AUDIO_VIS_MAX_DRAW_COLUMNS 12U
#define AUDIO_VIS_MAX_PENDING_COLUMNS 36U
#define AUDIO_VIS_DEFAULT_SWEEP_MS 1000U
#define AUDIO_VIS_DEFAULT_FFT_DRAW_MS 100U
#define AUDIO_VIS_MIN_PEAK_SCALE 32768U
#define AUDIO_VIS_MAX_PEAK_SCALE 8388608U

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
volatile uint32_t audio_vis_scope_x = 0U;
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
volatile uint8_t audio_vis_mode = 0U;

volatile uint32_t audio_fft_init_status = HAL_ERROR;
volatile uint32_t audio_fft_frame_count = 0U;
volatile uint32_t audio_fft_process_count = 0U;
volatile uint32_t audio_fft_drop_count = 0U;
volatile uint32_t audio_fft_ready = 0U;
volatile uint32_t audio_fft_collect_index = 0U;
volatile uint32_t audio_fft_peak_bin = 0U;
volatile uint32_t audio_fft_peak_fft_bin = 0U;
volatile uint32_t audio_fft_peak_freq_hz = 0U;
volatile uint32_t audio_fft_peak_value = 0U;
volatile uint32_t audio_fft_bin_values[AUDIO_FFT_DISPLAY_BINS];

static uint16_t waveform_y_min[AUDIO_VIS_WIDTH];
static uint16_t waveform_y_max[AUDIO_VIS_WIDTH];
static uint16_t waveform_draw_y_min[AUDIO_VIS_WIDTH];
static uint16_t waveform_draw_y_max[AUDIO_VIS_WIDTH];
static uint16_t spectrum_height[AUDIO_VIS_SPECTRUM_BINS];
static uint16_t spectrum_draw_height[AUDIO_VIS_SPECTRUM_BINS];
static uint16_t spectrum_prev_height[AUDIO_VIS_SPECTRUM_BINS];
static uint8_t audio_vis_has_draw_frame = 0U;
static uint8_t audio_vis_screen_initialized = 0U;
static int32_t audio_vis_source_samples[AUDIO_SAMPLE_BUF_LEN];
static volatile uint32_t audio_vis_source_sample_count = 0U;
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

static arm_rfft_fast_instance_f32 audio_fft_instance;
static int32_t audio_fft_collect_buf[AUDIO_FFT_SIZE];
static int32_t audio_fft_process_buf[AUDIO_FFT_SIZE];
static float32_t audio_fft_window[AUDIO_FFT_SIZE];
static float32_t audio_fft_input[AUDIO_FFT_SIZE];
static float32_t audio_fft_output[AUDIO_FFT_SIZE];

static uint16_t AudioVisualizer_MapToY(int32_t sample, uint32_t peak);
static uint32_t AudioVisualizer_Abs32(int32_t value);
static void AudioVisualizer_UpdateSpectrumS32(const int32_t *samples,
                                              uint32_t len,
                                              int32_t dc_offset);
static HAL_StatusTypeDef AudioVisualizer_DrawStatus(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawLabels(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawLevelBar(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawSpectrum(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawLiveSampleWaveform(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawDemoWaveform(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_DrawWaveformCenter(ST7789_HandleTypeDef *display);
static void AudioVisualizer_ScopePushSamples(const int32_t *samples,
                                             uint32_t len,
                                             int32_t dc_offset);
static void AudioVisualizer_UpdateAutoScale(uint32_t peak);
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
static HAL_StatusTypeDef AudioVisualizer_DrawFrequencyTicks(ST7789_HandleTypeDef *display);
static HAL_StatusTypeDef AudioVisualizer_RestoreWaveformColumn(ST7789_HandleTypeDef *display,
                                                               uint16_t x);
static HAL_StatusTypeDef AudioVisualizer_DrawScopeCursor(ST7789_HandleTypeDef *display,
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

HAL_StatusTypeDef AudioFFT_Init(void)
{
  arm_status status;

  status = arm_rfft_fast_init_f32(&audio_fft_instance, AUDIO_FFT_SIZE);
  audio_fft_init_status = (uint32_t)status;

  if (status != ARM_MATH_SUCCESS)
  {
    return HAL_ERROR;
  }

  for (uint32_t i = 0U; i < AUDIO_FFT_SIZE; i++)
  {
    float32_t phase = (2.0f * 3.14159265358979323846f * (float32_t)i) /
                      (float32_t)(AUDIO_FFT_SIZE - 1U);
    audio_fft_window[i] = 0.5f - (0.5f * cosf(phase));
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
  audio_fft_peak_bin = 0U;
  audio_fft_peak_fft_bin = 0U;
  audio_fft_peak_freq_hz = 0U;
  audio_fft_peak_value = 0U;

  return HAL_OK;
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

      audio_fft_collect_index = 0U;
    }
  }
}

uint8_t AudioFFT_ProcessIfReady(void)
{
  int64_t sum = 0;
  int32_t dc_offset;
  uint32_t peak_bin = 0U;
  uint32_t raw_peak_bin = 0U;
  float32_t peak_value = 0.0f;
  float32_t raw_peak_power = 0.0f;
  float32_t bin_magnitudes[AUDIO_FFT_DISPLAY_BINS];

  if (audio_fft_ready == 0U)
  {
    return 0U;
  }

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

  for (uint32_t i = 0U; i < AUDIO_FFT_DISPLAY_BINS; i++)
  {
    bin_magnitudes[i] = 0.0f;
  }

  for (uint32_t display_bin = 0U; display_bin < AUDIO_FFT_DISPLAY_BINS; display_bin++)
  {
    uint32_t start_bin = 2U + ((display_bin * ((AUDIO_FFT_SIZE / 2U) - 2U)) /
                               AUDIO_FFT_DISPLAY_BINS);
    uint32_t end_bin = 2U + (((display_bin + 1U) * ((AUDIO_FFT_SIZE / 2U) - 2U)) /
                             AUDIO_FFT_DISPLAY_BINS);
    float32_t power_sum = 0.0f;
    uint32_t power_count = 0U;
    float32_t band_mag;

    if (end_bin <= start_bin)
    {
      end_bin = start_bin + 1U;
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

      power_sum += power;
      if (power > raw_peak_power)
      {
        raw_peak_power = power;
        raw_peak_bin = fft_bin;
      }
      power_count++;
    }

    if (power_count == 0U)
    {
      band_mag = 0.0f;
    }
    else
    {
      band_mag = AudioFFT_SqrtApprox(power_sum / (float32_t)power_count);
    }

    bin_magnitudes[display_bin] = band_mag;
    if (band_mag > peak_value)
    {
      peak_value = band_mag;
      peak_bin = display_bin;
    }
  }

  if (peak_value <= 0.000001f)
  {
    peak_value = 0.000001f;
  }

  for (uint32_t i = 0U; i < AUDIO_FFT_DISPLAY_BINS; i++)
  {
    float32_t normalized = bin_magnitudes[i] / peak_value;
    float32_t compressed = AudioFFT_SqrtApprox(normalized);

    if (compressed > 1.0f)
    {
      compressed = 1.0f;
    }

    uint32_t target = (uint32_t)(compressed * 1000.0f);
    if (audio_fft_process_count == 0U)
    {
      audio_fft_bin_values[i] = target;
    }
    else
    {
      audio_fft_bin_values[i] = ((audio_fft_bin_values[i] * 3U) + target) / 4U;
    }
  }

  audio_fft_peak_bin = peak_bin;
  audio_fft_peak_fft_bin = raw_peak_bin;
  audio_fft_peak_freq_hz = (raw_peak_bin * AUDIO_VIS_SAMPLE_RATE_HZ) / AUDIO_FFT_SIZE;
  audio_fft_peak_value = (uint32_t)(peak_value * 1000000.0f);
  audio_fft_process_count++;
  audio_fft_ready = 0U;

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
  int64_t sum = 0;
  uint32_t peak = 1U;
  int32_t dc_offset;
  uint32_t copy_len;

  if ((samples == NULL) || (len == 0U))
  {
    return;
  }

  copy_len = (len < AUDIO_SAMPLE_BUF_LEN) ? len : AUDIO_SAMPLE_BUF_LEN;
  for (uint32_t i = 0U; i < copy_len; i++)
  {
    audio_vis_source_samples[i] = samples[i];
  }
  audio_vis_source_sample_count = copy_len;

  for (uint32_t i = 0U; i < len; i++)
  {
    sum += samples[i];
  }
  dc_offset = (int32_t)(sum / (int64_t)len);

  for (uint32_t i = 0U; i < len; i++)
  {
    uint32_t magnitude = AudioVisualizer_Abs32(samples[i] - dc_offset);
    if (magnitude > peak)
    {
      peak = magnitude;
    }
  }

  for (uint32_t x = 0U; x < AUDIO_VIS_WIDTH; x++)
  {
    int32_t min_sample = INT32_MAX;
    int32_t max_sample = INT32_MIN;
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
      int32_t sample = samples[index] - dc_offset;
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

  AudioVisualizer_UpdateSpectrumS32(samples, len, dc_offset);
  AudioVisualizer_UpdateAutoScale(peak);
  AudioVisualizer_ScopePushSamples(samples, len, dc_offset);

  audio_vis_peak = peak;
  audio_vis_last_update_tick = HAL_GetTick();
  audio_vis_live_active = 1U;
  audio_vis_update_count++;
  audio_vis_frame_ready = 1U;
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

static void AudioVisualizer_UpdateSpectrumS32(const int32_t *samples,
                                              uint32_t len,
                                              int32_t dc_offset)
{
  uint32_t magnitudes[AUDIO_VIS_SPECTRUM_BINS];
  uint32_t max_magnitude = 1U;

  for (uint32_t bin = 0U; bin < AUDIO_VIS_SPECTRUM_BINS; bin++)
  {
    uint64_t energy = 0U;
    uint32_t start = (bin * len) / AUDIO_VIS_SPECTRUM_BINS;
    uint32_t end = ((bin + 1U) * len) / AUDIO_VIS_SPECTRUM_BINS;
    int32_t previous = 0;

    if (end <= start)
    {
      end = start + 1U;
    }
    if (end > len)
    {
      end = len;
    }

    for (uint32_t i = start; i < end; i++)
    {
      int32_t sample = samples[i] - dc_offset;
      int32_t delta = sample - previous;
      previous = sample;

      /*
       * Fast fallback until the first CMSIS-DSP FFT window is ready.
       * The displayed spectrum is overwritten by AudioVisualizer_LoadFftSpectrum()
       * as soon as AudioFFT_ProcessIfReady() produces real bins.
       */
      energy += AudioVisualizer_Abs32(delta);
    }

    magnitudes[bin] = (uint32_t)(energy / ((end - start) + 1U));
    if (magnitudes[bin] > max_magnitude)
    {
      max_magnitude = magnitudes[bin];
    }
  }

  audio_vis_spectrum_peak = max_magnitude;
  for (uint32_t bin = 0U; bin < AUDIO_VIS_SPECTRUM_BINS; bin++)
  {
    uint32_t h = (magnitudes[bin] * (AUDIO_VIS_SPECTRUM_H - 2U)) / max_magnitude;
    if (h > AUDIO_VIS_SPECTRUM_H)
    {
      h = AUDIO_VIS_SPECTRUM_H;
    }
    spectrum_height[bin] = (uint16_t)h;
  }
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

  if (AudioVisualizer_DrawText(display, 2U, (uint16_t)(AUDIO_VIS_WAVEFORM_Y + 4U), "AMP", ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawText(display,
                               108U,
                               (uint16_t)(AUDIO_VIS_WAVEFORM_Y + AUDIO_VIS_WAVEFORM_H - 10U),
                               "TIME",
                               ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawText(display, 54U, AUDIO_VIS_FREQ_LABEL_Y, "FREQUENCY DOMAIN / FFT", ST7789_GREEN) != HAL_OK)
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

  if (AudioVisualizer_DrawText(display, 2U, (uint16_t)(AUDIO_VIS_SPECTRUM_Y + 4U), "MAG", ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawFrequencyTicks(display) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawText(display, 108U, AUDIO_VIS_FREQ_AXIS_Y, "FREQ", ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawLevelBar(ST7789_HandleTypeDef *display)
{
  (void)display;
  audio_vis_level_bar_width = audio_sample_signal_smooth;
  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawSpectrum(ST7789_HandleTypeDef *display)
{
  uint16_t plot_x = (uint16_t)(AUDIO_VIS_AXIS_X + 2U);
  uint16_t plot_w = (uint16_t)(ST7789_WIDTH - plot_x);
  uint16_t bin_width = plot_w / AUDIO_VIS_SPECTRUM_BINS;
  uint32_t now = HAL_GetTick();
  uint32_t draw_period_ms = audio_vis_spectrum_draw_period_ms;

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

  for (uint32_t bin = 0U; bin < AUDIO_VIS_SPECTRUM_BINS; bin++)
  {
    uint16_t h = spectrum_draw_height[bin];
    uint16_t x = (uint16_t)(plot_x + (bin * bin_width));
    uint16_t bar_w = (uint16_t)(bin_width - 2U);
    uint16_t base_y = AUDIO_VIS_SPECTRUM_Y + AUDIO_VIS_SPECTRUM_H;
    uint16_t bar_color;

    if (h == 0U)
    {
      h = 1U;
    }
    if (h > AUDIO_VIS_SPECTRUM_H)
    {
      h = AUDIO_VIS_SPECTRUM_H;
    }
    bar_color = (h > ((AUDIO_VIS_SPECTRUM_H * 2U) / 3U)) ?
                ST7789_YELLOW : ST7789_GREEN;

    if (ST7789_FillRect(display,
                        x,
                        (uint16_t)(AUDIO_VIS_SPECTRUM_Y + 1U),
                        bar_w,
                        (uint16_t)(AUDIO_VIS_SPECTRUM_H - 1U),
                        ST7789_BLACK) != HAL_OK)
    {
      return HAL_ERROR;
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

  if (AudioVisualizer_DrawText(display, 2U, (uint16_t)(AUDIO_VIS_SPECTRUM_Y + 4U), "MAG", ST7789_GRAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawFrequencyTicks(display) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (AudioVisualizer_DrawText(display, 108U, AUDIO_VIS_FREQ_AXIS_Y, "FREQ", ST7789_GRAY) != HAL_OK)
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

  if (x >= AUDIO_VIS_WIDTH)
  {
    x = 0U;
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
    if (x >= AUDIO_VIS_WIDTH)
    {
      x = 0U;
      audio_vis_scope_wrap_count++;
    }
  }

  audio_vis_demo_phase += AUDIO_VIS_MAX_DRAW_COLUMNS;
  audio_vis_scope_x = x;
  audio_vis_scope_columns_per_draw = AUDIO_VIS_MAX_DRAW_COLUMNS;
  if (AudioVisualizer_DrawScopeCursor(display, (uint16_t)x) != HAL_OK)
  {
    return HAL_ERROR;
  }
  audio_vis_live_peak = 32768U;
  audio_vis_live_sample_count += AUDIO_VIS_MAX_DRAW_COLUMNS;
  audio_vis_live_y_span = (uint32_t)(y_max_seen - y_min_seen);
  audio_vis_live_draw_count++;
  audio_vis_demo_draw_count++;
  audio_vis_incremental_draw_count++;

  return HAL_OK;
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
    if (audio_vis_scope_pending_read >= AUDIO_VIS_WIDTH)
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
    if (audio_vis_scope_pending_read >= AUDIO_VIS_WIDTH)
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
  if (x >= AUDIO_VIS_WIDTH)
  {
    x = 0U;
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
    if (x >= AUDIO_VIS_WIDTH)
    {
      x = 0U;
      audio_vis_scope_wrap_count++;
    }
  }

  audio_vis_scope_x = x;
  if (AudioVisualizer_DrawScopeCursor(display, (uint16_t)x) != HAL_OK)
  {
    return HAL_ERROR;
  }
  audio_vis_live_y_span = (uint32_t)(y_max_seen - y_min_seen);
  audio_vis_live_draw_count++;
  audio_vis_incremental_draw_count++;

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawWaveformCenter(ST7789_HandleTypeDef *display)
{
  return ST7789_DrawHLine(display,
                          0U,
                          AUDIO_VIS_WAVEFORM_Y + (AUDIO_VIS_WAVEFORM_H / 2U),
                          ST7789_WIDTH,
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
  if (samples_per_sweep < AUDIO_VIS_WIDTH)
  {
    samples_per_sweep = AUDIO_VIS_WIDTH;
  }

  pixels_per_sample_q16 = (AUDIO_VIS_WIDTH * AUDIO_VIS_SCOPE_Q16) / samples_per_sweep;
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

  if (audio_vis_scope_pending_columns >= AUDIO_VIS_WIDTH)
  {
    audio_vis_scope_pending_read++;
    if (audio_vis_scope_pending_read >= AUDIO_VIS_WIDTH)
    {
      audio_vis_scope_pending_read = 0U;
    }
    audio_vis_scope_pending_columns--;
    audio_vis_scope_dropped_columns++;
  }

  audio_vis_scope_pending_y_min[audio_vis_scope_pending_write] = y0;
  audio_vis_scope_pending_y_max[audio_vis_scope_pending_write] = y1;
  audio_vis_scope_pending_write++;
  if (audio_vis_scope_pending_write >= AUDIO_VIS_WIDTH)
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
  for (uint16_t x = 40U; x < ST7789_WIDTH; x = (uint16_t)(x + 40U))
  {
    if (ST7789_DrawVLine(display, x, (uint16_t)(y + 1U), (uint16_t)(h - 1U), AUDIO_VIS_GRID_COLOR) != HAL_OK)
    {
      return HAL_ERROR;
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

  if (ST7789_DrawVLine(display,
                       AUDIO_VIS_AXIS_X,
                       (uint16_t)(y + 2U),
                       (uint16_t)(h - 1U),
                       color) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (ST7789_DrawHLine(display,
                       AUDIO_VIS_AXIS_X,
                       bottom_y,
                       (uint16_t)(ST7789_WIDTH - AUDIO_VIS_AXIS_X),
                       color) != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (uint16_t row = 0U; row <= 4U; row++)
  {
    uint16_t tick_y = (uint16_t)(y + ((row * h) / 4U));
    if (ST7789_DrawHLine(display,
                         (uint16_t)(AUDIO_VIS_AXIS_X - 3U),
                         tick_y,
                         7U,
                         color) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

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

  return HAL_OK;
}

static HAL_StatusTypeDef AudioVisualizer_DrawFrequencyTicks(ST7789_HandleTypeDef *display)
{
  static const uint16_t tick_x[5] = {16U, 72U, 128U, 184U, 228U};
  static const uint16_t label_x[5] = {16U, 66U, 122U, 178U, 228U};
  static const char *tick_label[5] = {"0", "2K", "4K", "6K", "8K"};

  for (uint32_t i = 0U; i < 5U; i++)
  {
    if (ST7789_DrawVLine(display,
                         tick_x[i],
                         (uint16_t)(AUDIO_VIS_SPECTRUM_Y + AUDIO_VIS_SPECTRUM_H - 3U),
                         4U,
                         ST7789_GRAY) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if (AudioVisualizer_DrawText(display,
                                 label_x[i],
                                 AUDIO_VIS_TICK_LABEL_Y,
                                 tick_label[i],
                                 ST7789_GRAY) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
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

static HAL_StatusTypeDef AudioVisualizer_DrawScopeCursor(ST7789_HandleTypeDef *display,
                                                         uint16_t x)
{
  static uint8_t cursor_valid = 0U;
  static uint16_t cursor_last_x = 0U;

  if (x >= AUDIO_VIS_WIDTH)
  {
    x = 0U;
  }

  if ((cursor_valid != 0U) && (cursor_last_x < AUDIO_VIS_WIDTH))
  {
    if (AudioVisualizer_RestoreWaveformColumn(display, cursor_last_x) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  if (ST7789_DrawVLine(display,
                       x,
                       (uint16_t)(AUDIO_VIS_WAVEFORM_Y + 1U),
                       (uint16_t)(AUDIO_VIS_WAVEFORM_H - 1U),
                       ST7789_YELLOW) != HAL_OK)
  {
    return HAL_ERROR;
  }

  cursor_last_x = x;
  cursor_valid = 1U;
  audio_vis_scope_cursor_x = x;
  audio_vis_scope_cursor_draw_count++;

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

  if (glyph == NULL)
  {
    return HAL_OK;
  }

  for (uint32_t col = 0U; col < 5U; col++)
  {
    uint8_t bits = glyph[col];
    for (uint32_t row = 0U; row < 7U; row++)
    {
      if ((bits & (1U << row)) != 0U)
      {
        if (ST7789_DrawPixel(display,
                             (uint16_t)(x + col),
                             (uint16_t)(y + row),
                             color) != HAL_OK)
        {
          return HAL_ERROR;
        }
      }
    }
  }

  return HAL_OK;
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
