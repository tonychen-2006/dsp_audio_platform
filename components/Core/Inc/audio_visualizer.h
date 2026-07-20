#ifndef AUDIO_VISUALIZER_H
#define AUDIO_VISUALIZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "display_st7789.h"

#define AUDIO_VIS_WIDTH         ST7789_WIDTH
#define AUDIO_VIS_WAVEFORM_Y    18U
#define AUDIO_VIS_WAVEFORM_H    84U
#define AUDIO_VIS_SPECTRUM_Y    122U
#define AUDIO_VIS_SPECTRUM_H    92U
#define AUDIO_VIS_SPECTRUM_BINS 16U

/*
 * AUTO uses a coherent, one-sample-per-pixel scope for AUX and keeps the
 * legacy envelope sweep for the I2S microphone.  The explicit modes are
 * useful from Live Expressions and for A/B testing.
 */
#define AUDIO_VIS_SCOPE_MODE_ENVELOPE  0U
#define AUDIO_VIS_SCOPE_MODE_TRIGGERED 1U
#define AUDIO_VIS_SCOPE_MODE_AUTO      2U

#define AUDIO_VIS_SCOPE_PLOT_X             1U
#define AUDIO_VIS_SCOPE_SAMPLE_COUNT       238U
#define AUDIO_VIS_SCOPE_PRETRIGGER_SAMPLES 32U

typedef struct
{
  uint32_t configured_mode;
  uint32_t effective_mode;
  uint32_t sample_rate_hz;
  uint32_t sample_count;
  uint32_t sample_period_ns;
  uint32_t window_us;
  uint32_t pretrigger_samples;
  uint32_t trigger_hysteresis;
  uint32_t trigger_count;
  uint32_t auto_trigger_count;
  uint32_t stream_reset_count;
  uint32_t snapshot_sequence;
  uint32_t drawn_sequence;
  uint32_t snapshot_ready;
  uint32_t capture_active;
  uint32_t trigger_armed;
  int32_t snapshot_min;
  int32_t snapshot_max;
  int32_t snapshot_mean;
  uint32_t snapshot_peak;
  uint32_t snapshot_scale;
} AudioVisualizerScopeDiagnostics;

extern volatile uint8_t audio_vis_frame_ready;
extern volatile uint32_t audio_vis_update_count;
extern volatile uint32_t audio_vis_draw_count;
extern volatile uint32_t audio_vis_peak;
extern volatile uint32_t audio_vis_level_bar_width;
extern volatile uint32_t audio_vis_spectrum_peak;
extern volatile uint32_t audio_vis_live_draw_count;
extern volatile uint32_t audio_vis_live_peak;
extern volatile uint32_t audio_vis_live_sample_count;
extern volatile uint32_t audio_vis_live_y_span;
extern volatile uint32_t audio_vis_draw_stage;
extern volatile uint32_t audio_vis_draw_error_stage;
extern volatile uint32_t audio_vis_fft_draw_count;
extern volatile uint32_t audio_vis_fft_empty_count;
extern volatile uint32_t audio_vis_fft_axis_change_drawn;
extern volatile uint16_t audio_vis_fft_debug_bins[AUDIO_VIS_SPECTRUM_BINS];
extern volatile uint32_t audio_vis_full_clear_count;
extern volatile uint32_t audio_vis_incremental_draw_count;
extern volatile uint32_t audio_vis_scope_x;
extern volatile uint32_t audio_vis_scope_wrap_count;
extern volatile uint32_t audio_vis_scope_columns_per_draw;
extern volatile uint32_t audio_vis_scope_sweep_ms;
extern volatile uint32_t audio_vis_scope_elapsed_ms;
extern volatile uint32_t audio_vis_scope_column_accum;
extern volatile uint32_t audio_vis_scope_pending_columns;
extern volatile uint32_t audio_vis_scope_generated_columns;
extern volatile uint32_t audio_vis_scope_dropped_columns;
extern volatile uint32_t audio_vis_scope_empty_draw_count;
extern volatile uint32_t audio_vis_scope_cursor_x;
extern volatile uint32_t audio_vis_scope_cursor_draw_count;
extern volatile uint32_t audio_vis_scope_peak_scale;
extern volatile uint32_t audio_vis_spectrum_draw_period_ms;
extern volatile uint32_t audio_vis_spectrum_skip_count;
extern volatile uint32_t audio_vis_last_update_tick;
extern volatile uint32_t audio_vis_live_timeout_ms;
extern volatile uint32_t audio_vis_demo_draw_count;
extern volatile uint8_t audio_vis_auto_gain_enable;
extern volatile uint8_t audio_vis_demo_until_live;
extern volatile uint8_t audio_vis_live_active;
extern volatile uint8_t audio_vis_force_demo_waveform;
extern volatile uint8_t audio_vis_mode;

void AudioVisualizer_UpdateFromU16(const uint16_t *samples, uint32_t len);
void AudioVisualizer_UpdateFromS32(const int32_t *samples, uint32_t len);
void AudioVisualizer_UpdateFromCenteredS32(const int32_t *samples, uint32_t len);
void AudioVisualizer_SetScopeMode(uint8_t mode);
void AudioVisualizer_ResetScopeStream(void);
void AudioVisualizer_GetScopeDiagnostics(AudioVisualizerScopeDiagnostics *diagnostics);
uint8_t AudioVisualizer_IsFrameReady(void);
HAL_StatusTypeDef AudioVisualizer_DrawWaveform(ST7789_HandleTypeDef *display);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_VISUALIZER_H */
