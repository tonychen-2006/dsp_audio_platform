#include "audio_capture.h"
#include "audio_config.h"
#include "audio_fft.h"
#include "audio_output.h"
#include "audio_samples.h"
#include "audio_visualizer.h"
#include "arm_math.h"

/* The new buffered AUX front end provides analog headroom; use unity mapping. */
#define AUX_OUTPUT_DEFAULT_GAIN_Q8 256U
#define AUX_OUTPUT_DC_TRACK_SHIFT 10U
#define AUX_OUTPUT_LIMIT_THRESHOLD 28000
#define AUX_OUTPUT_LIMIT_MAX 32000
#define AUX_OUTPUT_HIGHPASS_CUTOFF_HZ 80U
#define AUX_OUTPUT_LOWPASS_CUTOFF_HZ 9000U
#define AUX_ADC_NEAR_RAIL_MARGIN_CODES 64U
#define AUX_OUTPUT_GATE_OPEN_PEAK_S16 4096U
#define AUX_OUTPUT_GATE_PEAK_MIN_AVG_S16 200U
#define AUX_OUTPUT_GATE_OPEN_AVG_S16 500U
#define AUX_OUTPUT_GATE_CLOSE_AVG_S16 300U
#define AUX_OUTPUT_GATE_ATTACK_STEP_Q16 1024U
#define AUX_OUTPUT_GATE_RELEASE_STEP_Q16 26U
#define AUX_OUTPUT_GATE_HOLD_SAMPLES 1280U

#define I2S_MIC_DEFAULT_GAIN_Q8 256U
#define I2S_MIC_MAX_GAIN_Q8 4096U
#define I2S_MIC_LIMIT_THRESHOLD 28000
#define I2S_MIC_LIMIT_MAX 32000
#define I2S_MIC_DC_TRACK_SHIFT 11U
#define I2S_MIC_GATE_OPEN_PEAK_S16 3072U
#define I2S_MIC_GATE_PEAK_MIN_AVG_S16 96U
#define I2S_MIC_GATE_OPEN_AVG_S16 384U
#define I2S_MIC_GATE_CLOSE_AVG_S16 192U
#define I2S_MIC_GATE_FLOOR_GAIN_Q16 4096U
#define I2S_MIC_GATE_ATTACK_STEP_Q16 1024U
#define I2S_MIC_GATE_RELEASE_STEP_Q16 14U
#define I2S_MIC_GATE_HOLD_SAMPLES 2560U
#define I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT 32U
#define I2S_MIC_ANALYSIS_MAX_BLOCKS_PER_SERVICE 4U

/*
 * Gate timing at the fixed 32 kHz output rate:
 *   attack  = 65536 / (1024 * 32000) = 2 ms
 *   hold    = 1280 / 32000 = 40 ms
 *   release = 65536 / (26 * 32000) ~= 79 ms
 * Average-level hysteresis lets the gate close even when isolated idle spikes
 * remain. A much higher peak threshold still preserves real transients.
 */
#define AUX_ANALYSIS_QUEUE_BLOCK_COUNT 128U
#define AUX_ANALYSIS_MAX_BLOCKS_PER_SERVICE 4U

uint16_t i2s_rx_buf[I2S_BUF_LEN];
volatile uint32_t audio_avg = 0U;
volatile uint32_t audio_min = 0U;
volatile uint32_t audio_max = 0U;
volatile uint32_t audio_nonzero_count = 0U;
volatile uint32_t audio_zero_count = 0U;
volatile uint32_t audio_changed_count = 0U;
volatile uint32_t audio_last_sample = 0U;
volatile uint16_t audio_debug_samples[16];
volatile uint32_t audio_process_count = 0U;
volatile uint32_t audio_start_status = HAL_ERROR;
volatile uint32_t audio_half_count = 0U;
volatile uint32_t audio_full_count = 0U;
volatile uint8_t audio_ready = 0U;
volatile uint8_t audio_input_source = AUDIO_INPUT_SOURCE_I2S;
volatile uint32_t audio_i2s_error_count = 0U;
volatile uint32_t audio_i2s_dma_error_code = 0U;

volatile uint32_t i2s_mic_gain_q8 = I2S_MIC_DEFAULT_GAIN_Q8;
volatile int32_t i2s_mic_dc_s24 = 0;
volatile uint32_t i2s_mic_left_abs_avg = 0U;
volatile uint32_t i2s_mic_right_abs_avg = 0U;
volatile uint32_t i2s_mic_selected_slot = AUDIO_I2S_SLOT_LEFT;
volatile uint32_t i2s_mic_highpass_enable = 1U;
volatile uint32_t i2s_mic_highpass_peak_s16 = 0U;
volatile uint32_t i2s_mic_lowpass_enable = 1U;
volatile uint32_t i2s_mic_lowpass_peak_s16 = 0U;
volatile uint32_t i2s_mic_gate_enable = 1U;
volatile uint32_t i2s_mic_gate_open = 0U;
volatile uint32_t i2s_mic_gate_gain_q8 = I2S_MIC_GATE_FLOOR_GAIN_Q16 >> 8;
volatile uint32_t i2s_mic_gate_detector_avg_s16 = 0U;
volatile uint32_t i2s_mic_gate_open_peak_threshold_s16 =
    I2S_MIC_GATE_OPEN_PEAK_S16;
volatile uint32_t i2s_mic_gate_peak_min_avg_s16 =
    I2S_MIC_GATE_PEAK_MIN_AVG_S16;
volatile uint32_t i2s_mic_gate_open_avg_threshold_s16 =
    I2S_MIC_GATE_OPEN_AVG_S16;
volatile uint32_t i2s_mic_gate_close_avg_threshold_s16 =
    I2S_MIC_GATE_CLOSE_AVG_S16;
volatile uint32_t i2s_mic_gate_floor_gain_q16 =
    I2S_MIC_GATE_FLOOR_GAIN_Q16;
volatile uint32_t i2s_mic_gate_hold_remaining_samples = 0U;
volatile uint32_t i2s_mic_output_peak_s16 = 0U;
volatile uint32_t i2s_mic_output_abs_avg_s16 = 0U;
volatile uint32_t i2s_mic_limiter_count = 0U;
volatile int32_t i2s_mic_limiter_last_input = 0;
volatile int16_t i2s_mic_limiter_last_output = 0;
volatile uint32_t i2s_mic_processed_sample_count = 0U;
volatile uint32_t i2s_mic_process_cycles_last = 0U;
volatile uint32_t i2s_mic_process_cycles_max = 0U;
volatile uint32_t i2s_mic_process_cycle_budget = 0U;
volatile uint32_t i2s_mic_process_deadline_miss_count = 0U;
volatile uint32_t i2s_mic_analysis_queue_count = 0U;
volatile uint32_t i2s_mic_analysis_queue_max = 0U;
volatile uint32_t i2s_mic_analysis_drop_count = 0U;
volatile uint32_t i2s_mic_analysis_service_count = 0U;
const uint32_t i2s_mic_analysis_queue_capacity =
    I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT - 1U;
volatile uint32_t i2s_mic_analysis_source_sequence = 0U;
volatile uint32_t i2s_mic_analysis_last_serviced_sequence = 0U;
volatile uint32_t i2s_mic_analysis_discontinuity_count = 0U;

int32_t aux_sample_buf[AUX_CAPTURE_BUF_LEN];
volatile uint32_t aux_adc_raw = 0U;
volatile uint32_t aux_adc_avg = 0U;
volatile uint32_t aux_adc_min = 0U;
volatile uint32_t aux_adc_max = 0U;
volatile uint32_t aux_adc_peak = 0U;
volatile uint32_t aux_adc_clip_active = 0U;
volatile uint32_t aux_adc_clip_sample_count = 0U;
volatile uint32_t aux_adc_clip_block_count = 0U;
volatile uint32_t aux_adc_headroom_codes = 0U;
volatile uint32_t aux_adc_abs_avg = 0U;
volatile uint32_t aux_adc_signal_smooth = 0U;
volatile uint32_t aux_adc_bias_mv = 0U;
volatile uint32_t aux_adc_vref_mv = 3070U;
volatile uint32_t aux_adc_sample_count = 0U;
volatile uint32_t aux_adc_block_count = 0U;
volatile uint32_t aux_adc_error_count = 0U;
volatile uint32_t aux_adc_start_status = HAL_ERROR;
volatile uint32_t aux_adc_timer_start_status = HAL_ERROR;
volatile uint32_t aux_adc_poll_status = HAL_OK;
volatile uint32_t aux_adc_dma_mode = 0xFFFFFFFFU;
volatile uint8_t aux_adc_dma_is_circular = 0U;
volatile uint32_t aux_adc_hal_state = 0U;
volatile uint32_t aux_adc_error_code = 0U;
volatile uint32_t aux_adc_dma_state = 0U;
volatile uint32_t aux_adc_dma_error_code = 0U;
volatile uint32_t aux_adc_dma_ndtr = 0U;
volatile uint32_t aux_adc_timer_counter = 0U;
volatile uint32_t aux_adc_timer_cr1 = 0U;
volatile uint32_t aux_adc_timer_sr = 0U;
volatile uint32_t aux_adc_half_count = 0U;
volatile uint32_t aux_adc_full_count = 0U;
volatile uint32_t aux_adc_pending_count = 0U;
volatile uint32_t aux_adc_service_count = 0U;
volatile uint32_t aux_adc_overrun_count = 0U;
volatile uint32_t aux_analysis_queue_count = 0U;
volatile uint32_t aux_analysis_queue_max = 0U;
volatile uint32_t aux_analysis_drop_count = 0U;
volatile uint32_t aux_analysis_service_count = 0U;
const uint32_t aux_analysis_queue_capacity =
    AUX_ANALYSIS_QUEUE_BLOCK_COUNT - 1U;
volatile uint32_t aux_analysis_source_sequence = 0U;
volatile uint32_t aux_analysis_last_serviced_sequence = 0U;
volatile uint32_t aux_analysis_discontinuity_count = 0U;
volatile uint32_t aux_analysis_tap = AUX_ANALYSIS_TAP_RAW_ADC;
volatile uint32_t aux_analysis_last_serviced_tap = AUX_ANALYSIS_TAP_RAW_ADC;
volatile uint32_t aux_analysis_raw_origin_adc_q8 = 2048U << 8;
volatile uint32_t aux_analysis_raw_origin_valid = 0U;
volatile uint32_t aux_output_realtime_enable = 1U;
volatile uint32_t aux_output_realtime_push_count = 0U;
volatile uint32_t aux_output_realtime_peak = 0U;
volatile uint32_t aux_output_realtime_avg = 0U;
volatile uint32_t aux_output_realtime_min = 0U;
volatile uint32_t aux_output_realtime_max = 0U;
volatile uint32_t aux_output_realtime_abs_avg = 0U;
volatile uint32_t aux_output_realtime_bias_mv = 0U;
volatile uint32_t aux_output_process_cycles_last = 0U;
volatile uint32_t aux_output_process_cycles_max = 0U;
volatile uint32_t aux_output_process_cycle_budget = 0U;
volatile uint32_t aux_output_process_deadline_miss_count = 0U;
volatile uint32_t aux_output_gain_q8 = AUX_OUTPUT_DEFAULT_GAIN_Q8;
volatile uint32_t aux_output_bias_adc = 2048U;
volatile uint32_t aux_output_peak_s16 = 0U;
volatile uint32_t aux_output_abs_avg_s16 = 0U;
volatile uint32_t aux_output_limiter_count = 0U;
volatile int32_t aux_output_limiter_last_input = 0;
volatile int16_t aux_output_limiter_last_output = 0;
volatile int16_t aux_output_last_sample_s16 = 0;
/* A clean line-level AUX source should remain continuous at low volume. */
volatile uint32_t aux_output_gate_enable = 0U;
volatile uint32_t aux_output_gate_open = 1U;
volatile uint32_t aux_output_gate_gain_q8 = 256U;
volatile uint32_t aux_output_gate_detector_avg_s16 = 0U;
volatile uint32_t aux_output_gate_open_peak_threshold_s16 =
    AUX_OUTPUT_GATE_OPEN_PEAK_S16;
volatile uint32_t aux_output_gate_peak_min_avg_s16 =
    AUX_OUTPUT_GATE_PEAK_MIN_AVG_S16;
volatile uint32_t aux_output_gate_open_avg_threshold_s16 =
    AUX_OUTPUT_GATE_OPEN_AVG_S16;
volatile uint32_t aux_output_gate_close_avg_threshold_s16 =
    AUX_OUTPUT_GATE_CLOSE_AVG_S16;
volatile uint32_t aux_output_gate_hold_remaining_samples = 0U;
volatile uint32_t aux_output_gate_peak_qualified = 0U;
volatile uint32_t aux_output_gate_rejected_peak_count = 0U;
volatile uint32_t aux_output_gate_open_count = 0U;
volatile uint32_t aux_output_gate_close_count = 0U;
volatile uint32_t aux_output_highpass_enable = 1U;
const uint32_t aux_output_highpass_cutoff_hz = AUX_OUTPUT_HIGHPASS_CUTOFF_HZ;
volatile uint32_t aux_output_highpass_peak_s16 = 0U;
volatile uint32_t aux_output_lowpass_enable = 1U;
const uint32_t aux_output_lowpass_cutoff_hz = AUX_OUTPUT_LOWPASS_CUTOFF_HZ;
volatile uint32_t aux_output_lowpass_peak_s16 = 0U;
volatile uint16_t aux_adc_debug_samples[16];
volatile uint8_t aux_ready = 0U;
uint16_t aux_adc_dma_buf[AUX_ADC_DMA_BUF_LEN];

static I2S_HandleTypeDef *audio_i2s_handle = NULL;
static ADC_HandleTypeDef *aux_adc_handle = NULL;
static TIM_HandleTypeDef *aux_timer_handle = NULL;
static uint32_t audio_block_len = I2S_BUF_LEN / 2U;
static int64_t i2s_mic_dc_estimate_q16 = 0;
static uint8_t i2s_mic_dc_initialized = 0U;
static uint32_t i2s_mic_gate_gain_current_q16 = I2S_MIC_GATE_FLOOR_GAIN_Q16;
static uint8_t i2s_mic_processing_slot_valid = 0U;
static uint8_t i2s_mic_processing_slot = AUDIO_I2S_SLOT_LEFT;
static arm_biquad_cascade_df2T_instance_f32 i2s_mic_highpass_filter;
static arm_biquad_cascade_df2T_instance_f32 i2s_mic_lowpass_filter;
static float32_t i2s_mic_highpass_state[4];
static float32_t i2s_mic_lowpass_state[4];
static float32_t i2s_mic_filter_buf[AUDIO_SAMPLE_BUF_LEN];
static int16_t i2s_mic_output_s16_buf[AUDIO_SAMPLE_BUF_LEN];
static int32_t i2s_mic_analysis_s32_buf[AUDIO_SAMPLE_BUF_LEN];
static int16_t i2s_mic_analysis_queue[I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT]
                                      [AUDIO_SAMPLE_BUF_LEN];
static uint16_t i2s_mic_analysis_queue_len[I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT];
static uint32_t i2s_mic_analysis_queue_sequence[
    I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT];
static volatile uint32_t i2s_mic_analysis_queue_read = 0U;
static volatile uint32_t i2s_mic_analysis_queue_write = 0U;
static uint32_t aux_sample_index = 0U;
static int32_t aux_adc_analysis_dc_estimate_q8 = 2048 << 8;
static int32_t aux_adc_output_dc_estimate_q8 = 2048 << 8;
static uint32_t aux_output_gate_gain_current_q16 = 65536U;
static arm_biquad_cascade_df2T_instance_f32 aux_output_highpass_filter;
static arm_biquad_cascade_df2T_instance_f32 aux_output_lowpass_filter;
static float32_t aux_output_highpass_state[4];
static float32_t aux_output_lowpass_state[4];
static float32_t aux_output_filter_buf[AUX_CAPTURE_BUF_LEN];
static uint16_t aux_adc_raw_buf[AUX_CAPTURE_BUF_LEN];
static int16_t aux_output_s16_buf[AUX_CAPTURE_BUF_LEN];
static uint16_t aux_adc_pending_half_buf[AUX_CAPTURE_BUF_LEN];
static uint16_t aux_adc_pending_full_buf[AUX_CAPTURE_BUF_LEN];
static uint16_t aux_adc_service_buf[AUX_CAPTURE_BUF_LEN];
static int16_t aux_output_service_buf[AUX_CAPTURE_BUF_LEN];
static int32_t aux_analysis_s32_buf[AUX_CAPTURE_BUF_LEN];
/* RAW_ADC stores exact positive 0..4095 codes; POST_DSP stores signed S16. */
static int16_t aux_analysis_queue[AUX_ANALYSIS_QUEUE_BLOCK_COUNT][AUX_CAPTURE_BUF_LEN];
static uint32_t aux_analysis_queue_sequence[AUX_ANALYSIS_QUEUE_BLOCK_COUNT];
static uint8_t aux_analysis_queue_tap[AUX_ANALYSIS_QUEUE_BLOCK_COUNT];
static volatile uint8_t aux_adc_half_pending = 0U;
static volatile uint8_t aux_adc_full_pending = 0U;
static volatile uint32_t aux_adc_half_sequence = 0U;
static volatile uint32_t aux_adc_full_sequence = 0U;
static volatile uint32_t aux_analysis_queue_read = 0U;
static volatile uint32_t aux_analysis_queue_write = 0U;

/*
 * CMSIS-DSP DF2T coefficients use {b0, b1, b2, -a1, -a2}.
 * These are fourth-order Butterworth filters designed for the shared 32 kHz
 * stream rate. AUX and I2S-microphone paths use independent filter state.
 * Their 80 Hz / 9 kHz corners keep the requested 100 Hz - 8 kHz playback
 * band within about 1 dB while rejecting more out-of-band noise.
 */
static const float32_t aux_output_highpass_coeffs[10] =
{
   0.985635106f, -1.971270212f, 0.985635106f,
   1.971148609f, -0.971391815f,
   0.993963670f, -1.987927340f, 0.993963670f,
   1.987804710f, -0.988049971f
};
static const float32_t aux_output_lowpass_coeffs[10] =
{
  0.313486468f, 0.626972936f, 0.313486468f,
 -0.204698088f, -0.049247784f,
  0.434473937f, 0.868947875f, 0.434473937f,
 -0.283699596f, -0.454196154f
};

static uint16_t AudioCapture_DmaSizeFrames(I2S_HandleTypeDef *hi2s);
static uint8_t AudioCapture_IsActiveI2S(I2S_HandleTypeDef *hi2s);
static void I2sMic_ResetProcessing(void);
static void I2sMic_ResetSignalState(void);
static uint32_t I2sMic_ProcessBlock(const int32_t *samples, uint32_t len);
static int16_t I2sMic_LimitOutput16(int32_t value);
static void I2sMic_QueueAnalysisBlock(const int16_t *samples, uint32_t len);
static void I2sMic_ServiceAnalysis(void);
static void I2sMic_RecordProcessCycles(uint32_t cycle_start,
                                       uint32_t sample_count);
static uint8_t AuxCapture_IsActiveAdc(ADC_HandleTypeDef *hadc);
static void AuxCapture_ResetStats(void);
static void AuxCapture_QueueDmaBlock(const uint16_t *raw_buf, uint8_t half_block);
static void AuxCapture_QueueAnalysisBlock(const uint16_t *raw_buf,
                                          const int16_t *processed_buf,
                                          uint8_t processed_valid);
static void AuxCapture_ServiceAnalysis(void);
static void AuxCapture_RecordOutputCycles(uint32_t cycle_start);
static int16_t AuxCapture_LimitOutput16(int32_t value);
static uint8_t AuxCapture_PushOutputRawBlock(const uint16_t *raw_buf, uint32_t len);
static void AuxCapture_ProcessRawBlock(const uint16_t *raw_buf, uint32_t len);

HAL_StatusTypeDef AudioCapture_Start(I2S_HandleTypeDef *hi2s)
{
  HAL_StatusTypeDef status;

  if ((hi2s == NULL) || (hi2s->hdmarx == NULL) ||
      (hi2s->Init.Mode != I2S_MODE_MASTER_RX) ||
      (hi2s->Init.Standard != I2S_STANDARD_PHILIPS) ||
      (hi2s->Init.DataFormat != I2S_DATAFORMAT_24B) ||
      (hi2s->Init.AudioFreq != I2S_AUDIOFREQ_32K) ||
      ((hi2s->hdmarx != NULL) &&
       (hi2s->hdmarx->Init.Mode != DMA_CIRCULAR)))
  {
    audio_start_status = HAL_ERROR;
    audio_i2s_dma_error_code =
        ((hi2s == NULL) || (hi2s->hdmarx == NULL)) ?
        0xFFFFFFFFU : hi2s->hdmarx->ErrorCode;
    return HAL_ERROR;
  }

  audio_i2s_handle = hi2s;
  audio_ready = 0U;
  audio_i2s_error_count = 0U;
  audio_i2s_dma_error_code = hi2s->hdmarx->ErrorCode;
  audio_half_count = 0U;
  audio_full_count = 0U;
  AudioSamples_ResetI2sSlotSelection();
  I2sMic_ResetProcessing();

  status = HAL_I2S_Receive_DMA(hi2s,
                               i2s_rx_buf,
                               AudioCapture_DmaSizeFrames(hi2s));
  audio_start_status = status;

  return status;
}

void AudioCapture_ProcessBlock(uint16_t *buf, uint32_t len)
{
  uint32_t sum = 0U;
  uint32_t nonzero_count = 0U;
  uint32_t changed_count = 0U;
  uint16_t min_value = UINT16_MAX;
  uint16_t max_value = 0U;
  uint16_t previous_sample;
  uint32_t sample_count;
  uint32_t processed_count;
  uint32_t cycle_start;

  if ((buf == NULL) || (len == 0U))
  {
    return;
  }

  previous_sample = buf[0];

  for (uint32_t i = 0U; i < len; i++)
  {
    uint16_t sample = buf[i];

    sum += sample;
    audio_last_sample = sample;
    if (i < 16U)
    {
      audio_debug_samples[i] = sample;
    }
    if (sample != 0U)
    {
      nonzero_count++;
    }
    if (sample != previous_sample)
    {
      changed_count++;
    }
    previous_sample = sample;
    if (sample < min_value)
    {
      min_value = sample;
    }
    if (sample > max_value)
    {
      max_value = sample;
    }
  }

  audio_avg = sum / len;
  audio_min = min_value;
  audio_max = max_value;
  audio_nonzero_count = nonzero_count;
  audio_zero_count = len - nonzero_count;
  audio_changed_count = changed_count;
  audio_process_count++;
  audio_ready = 1U;

  /* Keep inactive-source I2S clocks from overwriting AUX FFT/level state. */
  if (audio_input_source != AUDIO_INPUT_SOURCE_I2S)
  {
    return;
  }

  cycle_start = DWT->CYCCNT;
  sample_count = AudioSamples_UnpackI2s32(buf, len);
  if (sample_count > 0U)
  {
    i2s_mic_left_abs_avg = audio_sample_left_abs_avg;
    i2s_mic_right_abs_avg = audio_sample_right_abs_avg;
    i2s_mic_selected_slot = audio_sample_active_slot;
    if (audio_sample_slot_locked == 0U)
    {
      return;
    }
    if ((i2s_mic_processing_slot_valid == 0U) ||
        (i2s_mic_processing_slot != audio_sample_active_slot))
    {
      i2s_mic_processing_slot = audio_sample_active_slot;
      i2s_mic_processing_slot_valid = 1U;
      I2sMic_ResetSignalState();
    }
    processed_count = I2sMic_ProcessBlock(audio_sample_buf, sample_count);
    if (processed_count > 0U)
    {
      AudioOutput_PushSamplesS16(i2s_mic_output_s16_buf, processed_count);
      I2sMic_QueueAnalysisBlock(i2s_mic_output_s16_buf, processed_count);
      I2sMic_RecordProcessCycles(cycle_start, processed_count);
    }
  }
}

void AudioCapture_Service(void)
{
  I2sMic_ServiceAnalysis();
}

HAL_StatusTypeDef AuxCapture_Start(ADC_HandleTypeDef *hadc)
{
  if (hadc == NULL)
  {
    aux_adc_start_status = HAL_ERROR;
    return HAL_ERROR;
  }

  aux_adc_handle = hadc;
  aux_timer_handle = NULL;
  AuxCapture_ResetStats();
  aux_adc_start_status = HAL_OK;

  return HAL_OK;
}

HAL_StatusTypeDef AuxCapture_StartDma(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim)
{
  HAL_StatusTypeDef status;

  if ((hadc == NULL) || (htim == NULL))
  {
    aux_adc_start_status = HAL_ERROR;
    return HAL_ERROR;
  }

  aux_adc_handle = hadc;
  aux_timer_handle = htim;
  AuxCapture_ResetStats();

  if (hadc->DMA_Handle == NULL)
  {
    aux_adc_dma_mode = 0xFFFFFFFFU;
    aux_adc_dma_is_circular = 0U;
    aux_adc_start_status = HAL_ERROR;
    return HAL_ERROR;
  }

  aux_adc_dma_mode = hadc->DMA_Handle->Init.Mode;
  aux_adc_dma_is_circular = (hadc->DMA_Handle->Init.Mode == DMA_CIRCULAR) ? 1U : 0U;

  /*
   * Start ADC DMA first. TIM2 then provides the fixed audio sample clock
   * through TRGO, so the ADC begins converting on timer update events.
   */
  status = HAL_ADC_Start_DMA(hadc, (uint32_t *)aux_adc_dma_buf, AUX_ADC_DMA_BUF_LEN);
  aux_adc_start_status = status;
  if (status != HAL_OK)
  {
    aux_adc_error_count++;
    return status;
  }

  status = HAL_TIM_Base_Start(htim);
  aux_adc_timer_start_status = status;
  if (status != HAL_OK)
  {
    aux_adc_error_count++;
  }

  AuxCapture_UpdateDiagnostics();

  return status;
}

void AuxCapture_Poll(void)
{
  HAL_StatusTypeDef status;
  uint32_t raw;

  if (aux_adc_handle == NULL)
  {
    return;
  }

  status = HAL_ADC_Start(aux_adc_handle);
  if (status != HAL_OK)
  {
    aux_adc_poll_status = status;
    aux_adc_error_count++;
    return;
  }

  status = HAL_ADC_PollForConversion(aux_adc_handle, 1U);
  if (status != HAL_OK)
  {
    aux_adc_poll_status = status;
    aux_adc_error_count++;
    (void)HAL_ADC_Stop(aux_adc_handle);
    return;
  }

  raw = HAL_ADC_GetValue(aux_adc_handle);
  (void)HAL_ADC_Stop(aux_adc_handle);

  if (raw > 4095U)
  {
    raw = 4095U;
  }

  aux_adc_raw = raw;
  aux_adc_poll_status = HAL_OK;

  if (aux_sample_index < 16U)
  {
    aux_adc_debug_samples[aux_sample_index] = (uint16_t)raw;
  }
  aux_adc_raw_buf[aux_sample_index] = (uint16_t)raw;

  aux_sample_index++;

  if (aux_sample_index >= AUX_CAPTURE_BUF_LEN)
  {
    AuxCapture_ProcessRawBlock(aux_adc_raw_buf, AUX_CAPTURE_BUF_LEN);
    aux_sample_index = 0U;
  }
}

void AuxCapture_Service(void)
{
  uint8_t process_half = 0U;
  uint8_t process_full = 0U;
  uint32_t primask;
  uint32_t half_sequence = 0U;
  uint32_t full_sequence = 0U;

  primask = __get_PRIMASK();
  __disable_irq();
  if (aux_adc_half_pending != 0U)
  {
    aux_adc_half_pending = 0U;
    aux_adc_pending_count = (uint32_t)aux_adc_full_pending;
    half_sequence = aux_adc_half_sequence;
    process_half = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (process_half != 0U)
  {
    for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
    {
      aux_adc_service_buf[i] = aux_adc_pending_half_buf[i];
    }
    __DMB();
    if ((half_sequence == aux_adc_half_sequence) &&
        ((half_sequence & 1U) == 0U))
    {
      AuxCapture_ProcessRawBlock(aux_adc_service_buf, AUX_CAPTURE_BUF_LEN);
      aux_adc_service_count++;
    }
    else
    {
      aux_adc_overrun_count++;
    }
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (aux_adc_full_pending != 0U)
  {
    aux_adc_full_pending = 0U;
    aux_adc_pending_count = (uint32_t)aux_adc_half_pending;
    full_sequence = aux_adc_full_sequence;
    process_full = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (process_full != 0U)
  {
    for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
    {
      aux_adc_service_buf[i] = aux_adc_pending_full_buf[i];
    }
    __DMB();
    if ((full_sequence == aux_adc_full_sequence) &&
        ((full_sequence & 1U) == 0U))
    {
      AuxCapture_ProcessRawBlock(aux_adc_service_buf, AUX_CAPTURE_BUF_LEN);
      aux_adc_service_count++;
    }
    else
    {
      aux_adc_overrun_count++;
    }
  }

  AuxCapture_ServiceAnalysis();
}

void AuxCapture_UpdateDiagnostics(void)
{
  if (aux_adc_handle != NULL)
  {
    aux_adc_hal_state = aux_adc_handle->State;
    aux_adc_error_code = aux_adc_handle->ErrorCode;

    if (aux_adc_handle->DMA_Handle != NULL)
    {
      aux_adc_dma_state = aux_adc_handle->DMA_Handle->State;
      aux_adc_dma_error_code = aux_adc_handle->DMA_Handle->ErrorCode;
      aux_adc_dma_ndtr = __HAL_DMA_GET_COUNTER(aux_adc_handle->DMA_Handle);
    }
    else
    {
      aux_adc_dma_state = 0xFFFFFFFFU;
      aux_adc_dma_error_code = 0xFFFFFFFFU;
      aux_adc_dma_ndtr = 0xFFFFFFFFU;
    }
  }

  if (aux_timer_handle != NULL)
  {
    aux_adc_timer_counter = __HAL_TIM_GET_COUNTER(aux_timer_handle);
    aux_adc_timer_cr1 = aux_timer_handle->Instance->CR1;
    aux_adc_timer_sr = aux_timer_handle->Instance->SR;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (AuxCapture_IsActiveAdc(hadc) != 0U)
  {
    uint8_t processed_valid;
    uint32_t cycle_start = DWT->CYCCNT;

    aux_adc_half_count++;
    processed_valid =
        AuxCapture_PushOutputRawBlock(&aux_adc_dma_buf[0], AUX_CAPTURE_BUF_LEN);
    AuxCapture_QueueAnalysisBlock(&aux_adc_dma_buf[0],
                                  aux_output_s16_buf,
                                  processed_valid);
    /*
     * The realtime path above has already conditioned and queued this block
     * for I2S output.  The deferred two-slot copy is only the legacy fallback;
     * queueing it as well lets a blocking display draw overwrite diagnostic
     * staging buffers and report a misleading ADC "overrun".
     */
    if (aux_output_realtime_enable == 0U)
    {
      AuxCapture_QueueDmaBlock(&aux_adc_dma_buf[0], 1U);
    }
    if (processed_valid != 0U)
    {
      AuxCapture_RecordOutputCycles(cycle_start);
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (AuxCapture_IsActiveAdc(hadc) != 0U)
  {
    uint8_t processed_valid;
    uint32_t cycle_start = DWT->CYCCNT;

    aux_adc_full_count++;
    processed_valid =
        AuxCapture_PushOutputRawBlock(&aux_adc_dma_buf[AUX_CAPTURE_BUF_LEN],
                                      AUX_CAPTURE_BUF_LEN);
    AuxCapture_QueueAnalysisBlock(
        &aux_adc_dma_buf[AUX_CAPTURE_BUF_LEN],
        aux_output_s16_buf,
        processed_valid);
    if (aux_output_realtime_enable == 0U)
    {
      AuxCapture_QueueDmaBlock(&aux_adc_dma_buf[AUX_CAPTURE_BUF_LEN], 0U);
    }
    if (processed_valid != 0U)
    {
      AuxCapture_RecordOutputCycles(cycle_start);
    }
  }
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (AudioCapture_IsActiveI2S(hi2s) != 0U)
  {
    audio_half_count++;
    AudioCapture_ProcessBlock(&i2s_rx_buf[0], audio_block_len);
  }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (AudioCapture_IsActiveI2S(hi2s) != 0U)
  {
    audio_full_count++;
    AudioCapture_ProcessBlock(&i2s_rx_buf[audio_block_len], audio_block_len);
  }
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
  AudioOutput_HandleI2sError(hi2s);
  if (AudioCapture_IsActiveI2S(hi2s) != 0U)
  {
    audio_i2s_error_count++;
    audio_ready = 0U;
    if (hi2s->hdmarx != NULL)
    {
      audio_i2s_dma_error_code = hi2s->hdmarx->ErrorCode;
    }
  }
}

static uint16_t AudioCapture_DmaSizeFrames(I2S_HandleTypeDef *hi2s)
{
  audio_block_len = I2S_BUF_LEN / 2U;

  if ((hi2s->Init.DataFormat == I2S_DATAFORMAT_24B) ||
      (hi2s->Init.DataFormat == I2S_DATAFORMAT_32B))
  {
    /*
     * For 24/32-bit I2S on STM32F4 HAL, Size is the number of I2S frames.
     * DMA still stores two uint16_t half-words per frame.
     */
    return I2S_BUF_LEN / 2U;
  }

  return I2S_BUF_LEN;
}

static uint8_t AudioCapture_IsActiveI2S(I2S_HandleTypeDef *hi2s)
{
  if ((hi2s == NULL) || (audio_i2s_handle == NULL))
  {
    return 0U;
  }

  return (hi2s->Instance == audio_i2s_handle->Instance) ? 1U : 0U;
}

static void I2sMic_ResetProcessing(void)
{
  i2s_mic_gain_q8 = I2S_MIC_DEFAULT_GAIN_Q8;
  i2s_mic_dc_s24 = 0;
  i2s_mic_left_abs_avg = 0U;
  i2s_mic_right_abs_avg = 0U;
  i2s_mic_selected_slot = AUDIO_I2S_SLOT_LEFT;
  i2s_mic_highpass_enable = 1U;
  i2s_mic_highpass_peak_s16 = 0U;
  i2s_mic_lowpass_enable = 1U;
  i2s_mic_lowpass_peak_s16 = 0U;
  i2s_mic_gate_enable = 1U;
  i2s_mic_gate_open = 0U;
  i2s_mic_gate_gain_q8 = I2S_MIC_GATE_FLOOR_GAIN_Q16 >> 8;
  i2s_mic_gate_detector_avg_s16 = 0U;
  i2s_mic_gate_open_peak_threshold_s16 = I2S_MIC_GATE_OPEN_PEAK_S16;
  i2s_mic_gate_peak_min_avg_s16 = I2S_MIC_GATE_PEAK_MIN_AVG_S16;
  i2s_mic_gate_open_avg_threshold_s16 = I2S_MIC_GATE_OPEN_AVG_S16;
  i2s_mic_gate_close_avg_threshold_s16 = I2S_MIC_GATE_CLOSE_AVG_S16;
  i2s_mic_gate_floor_gain_q16 = I2S_MIC_GATE_FLOOR_GAIN_Q16;
  i2s_mic_gate_hold_remaining_samples = 0U;
  i2s_mic_output_peak_s16 = 0U;
  i2s_mic_output_abs_avg_s16 = 0U;
  i2s_mic_limiter_count = 0U;
  i2s_mic_limiter_last_input = 0;
  i2s_mic_limiter_last_output = 0;
  i2s_mic_processed_sample_count = 0U;
  i2s_mic_process_cycles_last = 0U;
  i2s_mic_process_cycles_max = 0U;
  i2s_mic_process_cycle_budget = 0U;
  i2s_mic_process_deadline_miss_count = 0U;
  i2s_mic_analysis_queue_count = 0U;
  i2s_mic_analysis_queue_max = 0U;
  i2s_mic_analysis_drop_count = 0U;
  i2s_mic_analysis_service_count = 0U;
  i2s_mic_analysis_source_sequence = 0U;
  i2s_mic_analysis_last_serviced_sequence = 0U;
  i2s_mic_analysis_discontinuity_count = 0U;
  i2s_mic_analysis_queue_read = 0U;
  i2s_mic_analysis_queue_write = 0U;
  i2s_mic_processing_slot_valid = 0U;
  i2s_mic_processing_slot = AUDIO_I2S_SLOT_LEFT;
  I2sMic_ResetSignalState();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  for (uint32_t i = 0U; i < AUDIO_SAMPLE_BUF_LEN; i++)
  {
    i2s_mic_filter_buf[i] = 0.0f;
    i2s_mic_output_s16_buf[i] = 0;
    i2s_mic_analysis_s32_buf[i] = 0;
  }
  for (uint32_t block = 0U;
       block < I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT;
       block++)
  {
    i2s_mic_analysis_queue_len[block] = 0U;
    i2s_mic_analysis_queue_sequence[block] = 0U;
    for (uint32_t i = 0U; i < AUDIO_SAMPLE_BUF_LEN; i++)
    {
      i2s_mic_analysis_queue[block][i] = 0;
    }
  }
}

static void I2sMic_ResetSignalState(void)
{
  uint32_t gate_floor_q16 = i2s_mic_gate_floor_gain_q16;

  if (gate_floor_q16 > 65536U)
  {
    gate_floor_q16 = 65536U;
  }
  i2s_mic_dc_estimate_q16 = 0;
  i2s_mic_dc_initialized = 0U;
  i2s_mic_gate_gain_current_q16 = gate_floor_q16;
  i2s_mic_gate_hold_remaining_samples = 0U;
  i2s_mic_gate_open = 0U;
  i2s_mic_gate_gain_q8 = gate_floor_q16 >> 8;
  i2s_mic_highpass_peak_s16 = 0U;
  i2s_mic_lowpass_peak_s16 = 0U;
  i2s_mic_output_peak_s16 = 0U;
  i2s_mic_output_abs_avg_s16 = 0U;
  arm_biquad_cascade_df2T_init_f32(&i2s_mic_highpass_filter,
                                    2U,
                                    aux_output_highpass_coeffs,
                                    i2s_mic_highpass_state);
  arm_biquad_cascade_df2T_init_f32(&i2s_mic_lowpass_filter,
                                    2U,
                                    aux_output_lowpass_coeffs,
                                    i2s_mic_lowpass_state);
}

static uint32_t I2sMic_ProcessBlock(const int32_t *samples, uint32_t len)
{
  uint64_t filtered_abs_sum = 0U;
  uint64_t output_abs_sum = 0U;
  uint32_t highpass_peak = 0U;
  uint32_t lowpass_peak = 0U;
  uint32_t output_peak = 0U;
  uint32_t detector_avg;
  uint32_t gate_target_q16;
  uint32_t gate_floor_q16;
  uint32_t gate_open_avg;
  uint32_t gate_close_avg;
  uint32_t gate_open_peak;
  uint32_t gate_peak_min_avg;
  uint32_t mic_gain_q8;

  if ((samples == NULL) || (len == 0U))
  {
    return 0U;
  }
  if (len > AUDIO_SAMPLE_BUF_LEN)
  {
    len = AUDIO_SAMPLE_BUF_LEN;
  }

  if (i2s_mic_dc_initialized == 0U)
  {
    int64_t sum = 0;
    for (uint32_t i = 0U; i < len; i++)
    {
      sum += samples[i];
    }
    i2s_mic_dc_estimate_q16 =
        (sum / (int64_t)len) * 65536LL;
    i2s_mic_dc_initialized = 1U;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    int64_t input_q16 = (int64_t)samples[i] * 65536LL;
    int32_t ac_sample_s24;

    i2s_mic_dc_estimate_q16 +=
        (input_q16 - i2s_mic_dc_estimate_q16) /
        (1LL << I2S_MIC_DC_TRACK_SHIFT);
    ac_sample_s24 = samples[i] -
        (int32_t)(i2s_mic_dc_estimate_q16 / 65536LL);
    i2s_mic_filter_buf[i] = (float32_t)ac_sample_s24 * (1.0f / 256.0f);
  }
  i2s_mic_dc_s24 = (int32_t)(i2s_mic_dc_estimate_q16 / 65536LL);

  if (i2s_mic_highpass_enable != 0U)
  {
    arm_biquad_cascade_df2T_f32(&i2s_mic_highpass_filter,
                                 i2s_mic_filter_buf,
                                 i2s_mic_filter_buf,
                                 len);
  }
  else
  {
    for (uint32_t i = 0U;
         i < (sizeof(i2s_mic_highpass_state) /
              sizeof(i2s_mic_highpass_state[0]));
         i++)
    {
      i2s_mic_highpass_state[i] = 0.0f;
    }
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    float32_t sample = i2s_mic_filter_buf[i];
    uint32_t magnitude = (sample < 0.0f) ?
                         (uint32_t)(-sample) : (uint32_t)sample;
    if (magnitude > highpass_peak)
    {
      highpass_peak = magnitude;
    }
  }

  if (i2s_mic_lowpass_enable != 0U)
  {
    arm_biquad_cascade_df2T_f32(&i2s_mic_lowpass_filter,
                                 i2s_mic_filter_buf,
                                 i2s_mic_filter_buf,
                                 len);
  }
  else
  {
    for (uint32_t i = 0U;
         i < (sizeof(i2s_mic_lowpass_state) /
              sizeof(i2s_mic_lowpass_state[0]));
         i++)
    {
      i2s_mic_lowpass_state[i] = 0.0f;
    }
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    float32_t sample = i2s_mic_filter_buf[i];
    uint32_t magnitude = (sample < 0.0f) ?
                         (uint32_t)(-sample) : (uint32_t)sample;
    filtered_abs_sum += magnitude;
    if (magnitude > lowpass_peak)
    {
      lowpass_peak = magnitude;
    }
  }

  detector_avg = (uint32_t)(filtered_abs_sum / len);
  gate_floor_q16 = i2s_mic_gate_floor_gain_q16;
  if (gate_floor_q16 > 65536U)
  {
    gate_floor_q16 = 65536U;
  }
  gate_open_avg = i2s_mic_gate_open_avg_threshold_s16;
  gate_close_avg = i2s_mic_gate_close_avg_threshold_s16;
  gate_open_peak = i2s_mic_gate_open_peak_threshold_s16;
  gate_peak_min_avg = i2s_mic_gate_peak_min_avg_s16;
  mic_gain_q8 = i2s_mic_gain_q8;
  if (gate_close_avg > 32766U)
  {
    gate_close_avg = 32766U;
    i2s_mic_gate_close_avg_threshold_s16 = gate_close_avg;
  }
  if (gate_open_avg > 32767U)
  {
    gate_open_avg = 32767U;
    i2s_mic_gate_open_avg_threshold_s16 = gate_open_avg;
  }
  if (gate_open_avg <= gate_close_avg)
  {
    gate_open_avg = gate_close_avg + 1U;
    i2s_mic_gate_open_avg_threshold_s16 = gate_open_avg;
  }
  if (gate_open_peak > 32767U)
  {
    gate_open_peak = 32767U;
    i2s_mic_gate_open_peak_threshold_s16 = gate_open_peak;
  }
  if (gate_peak_min_avg > 32767U)
  {
    gate_peak_min_avg = 32767U;
    i2s_mic_gate_peak_min_avg_s16 = gate_peak_min_avg;
  }
  if (mic_gain_q8 > I2S_MIC_MAX_GAIN_Q8)
  {
    mic_gain_q8 = I2S_MIC_MAX_GAIN_Q8;
    i2s_mic_gain_q8 = mic_gain_q8;
  }

  if (i2s_mic_gate_enable == 0U)
  {
    gate_target_q16 = 65536U;
    i2s_mic_gate_open = 1U;
    i2s_mic_gate_hold_remaining_samples = I2S_MIC_GATE_HOLD_SAMPLES;
  }
  else if (((lowpass_peak >= gate_open_peak) &&
            (detector_avg >= gate_peak_min_avg)) ||
           (detector_avg >= gate_open_avg))
  {
    gate_target_q16 = 65536U;
    i2s_mic_gate_open = 1U;
    i2s_mic_gate_hold_remaining_samples = I2S_MIC_GATE_HOLD_SAMPLES;
  }
  else if (i2s_mic_gate_hold_remaining_samples != 0U)
  {
    i2s_mic_gate_hold_remaining_samples =
        (i2s_mic_gate_hold_remaining_samples > len) ?
        (i2s_mic_gate_hold_remaining_samples - len) : 0U;
    gate_target_q16 = 65536U;
    i2s_mic_gate_open = 1U;
  }
  else
  {
    i2s_mic_gate_hold_remaining_samples = 0U;
    if (detector_avg <= gate_close_avg)
    {
      gate_target_q16 = gate_floor_q16;
    }
    else
    {
      gate_target_q16 = gate_floor_q16 +
          (uint32_t)(((uint64_t)(65536U - gate_floor_q16) *
                      (detector_avg - gate_close_avg)) /
                     (gate_open_avg - gate_close_avg));
    }
    i2s_mic_gate_open =
        (gate_target_q16 > gate_floor_q16) ? 1U : 0U;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    float32_t filtered_sample = i2s_mic_filter_buf[i];
    int32_t filtered_s16;
    int32_t expanded_sample;
    int32_t scaled_sample;
    int64_t scaled_sample_s64;
    int16_t output_sample;
    uint32_t magnitude;

    if (filtered_sample > 2147483000.0f)
    {
      filtered_s16 = 2147483000;
    }
    else if (filtered_sample < -2147483000.0f)
    {
      filtered_s16 = -2147483000;
    }
    else
    {
      filtered_s16 = (filtered_sample >= 0.0f) ?
                     (int32_t)(filtered_sample + 0.5f) :
                     (int32_t)(filtered_sample - 0.5f);
    }

    if (i2s_mic_gate_gain_current_q16 < gate_target_q16)
    {
      i2s_mic_gate_gain_current_q16 += I2S_MIC_GATE_ATTACK_STEP_Q16;
      if (i2s_mic_gate_gain_current_q16 > gate_target_q16)
      {
        i2s_mic_gate_gain_current_q16 = gate_target_q16;
      }
    }
    else if (i2s_mic_gate_gain_current_q16 > gate_target_q16)
    {
      if (i2s_mic_gate_gain_current_q16 >
          (gate_target_q16 + I2S_MIC_GATE_RELEASE_STEP_Q16))
      {
        i2s_mic_gate_gain_current_q16 -= I2S_MIC_GATE_RELEASE_STEP_Q16;
      }
      else
      {
        i2s_mic_gate_gain_current_q16 = gate_target_q16;
      }
    }

    expanded_sample = (int32_t)(((int64_t)filtered_s16 *
                                 i2s_mic_gate_gain_current_q16) / 65536LL);
    scaled_sample_s64 = ((int64_t)expanded_sample * mic_gain_q8) / 256LL;
    if (scaled_sample_s64 > INT32_MAX)
    {
      scaled_sample = INT32_MAX;
    }
    else if (scaled_sample_s64 < INT32_MIN)
    {
      scaled_sample = INT32_MIN;
    }
    else
    {
      scaled_sample = (int32_t)scaled_sample_s64;
    }
    output_sample = I2sMic_LimitOutput16(scaled_sample);
    i2s_mic_output_s16_buf[i] = output_sample;
    magnitude = (output_sample < 0) ?
                (uint32_t)(-output_sample) : (uint32_t)output_sample;
    output_abs_sum += magnitude;
    if (magnitude > output_peak)
    {
      output_peak = magnitude;
    }
  }

  i2s_mic_highpass_peak_s16 = highpass_peak;
  i2s_mic_lowpass_peak_s16 = lowpass_peak;
  i2s_mic_gate_detector_avg_s16 = detector_avg;
  i2s_mic_gate_gain_q8 = i2s_mic_gate_gain_current_q16 >> 8;
  i2s_mic_output_peak_s16 = output_peak;
  i2s_mic_output_abs_avg_s16 = (uint32_t)(output_abs_sum / len);
  i2s_mic_processed_sample_count += len;
  return len;
}

static int16_t I2sMic_LimitOutput16(int32_t value)
{
  int64_t magnitude = value;
  int64_t limited;

  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }
  if (magnitude <= I2S_MIC_LIMIT_THRESHOLD)
  {
    return (int16_t)value;
  }

  i2s_mic_limiter_count++;
  i2s_mic_limiter_last_input = value;
  limited = I2S_MIC_LIMIT_THRESHOLD +
            ((magnitude - I2S_MIC_LIMIT_THRESHOLD) / 8);
  if (limited > I2S_MIC_LIMIT_MAX)
  {
    limited = I2S_MIC_LIMIT_MAX;
  }
  if (value < 0)
  {
    limited = -limited;
  }
  i2s_mic_limiter_last_output = (int16_t)limited;
  return (int16_t)limited;
}

static void I2sMic_QueueAnalysisBlock(const int16_t *samples, uint32_t len)
{
  uint32_t write_index;
  uint32_t read_index;
  uint32_t next_index;
  uint32_t available;
  uint32_t source_sequence;

  if ((samples == NULL) || (len == 0U))
  {
    return;
  }
  if (len > AUDIO_SAMPLE_BUF_LEN)
  {
    len = AUDIO_SAMPLE_BUF_LEN;
  }

  source_sequence = i2s_mic_analysis_source_sequence + 1U;
  i2s_mic_analysis_source_sequence = source_sequence;

  write_index = i2s_mic_analysis_queue_write;
  read_index = i2s_mic_analysis_queue_read;
  next_index = write_index + 1U;
  if (next_index >= I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT)
  {
    next_index = 0U;
  }
  if (next_index == read_index)
  {
    i2s_mic_analysis_drop_count++;
    return;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    i2s_mic_analysis_queue[write_index][i] = samples[i];
  }
  i2s_mic_analysis_queue_len[write_index] = (uint16_t)len;
  i2s_mic_analysis_queue_sequence[write_index] = source_sequence;
  __DMB();
  i2s_mic_analysis_queue_write = next_index;

  available = (next_index >= read_index) ?
              (next_index - read_index) :
              (I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT - read_index + next_index);
  i2s_mic_analysis_queue_count = available;
  if (available > i2s_mic_analysis_queue_max)
  {
    i2s_mic_analysis_queue_max = available;
  }
}

static void I2sMic_ServiceAnalysis(void)
{
  for (uint32_t block = 0U;
       block < I2S_MIC_ANALYSIS_MAX_BLOCKS_PER_SERVICE;
       block++)
  {
    uint32_t read_index = i2s_mic_analysis_queue_read;
    uint32_t write_index = i2s_mic_analysis_queue_write;
    uint32_t next_index;
    uint32_t len;
    uint32_t available;
    uint32_t source_sequence;

    if (read_index == write_index)
    {
      i2s_mic_analysis_queue_count = 0U;
      break;
    }

    __DMB();
    len = i2s_mic_analysis_queue_len[read_index];
    source_sequence = i2s_mic_analysis_queue_sequence[read_index];
    if (len > AUDIO_SAMPLE_BUF_LEN)
    {
      len = AUDIO_SAMPLE_BUF_LEN;
    }
    for (uint32_t i = 0U; i < len; i++)
    {
      i2s_mic_analysis_s32_buf[i] =
          (int32_t)i2s_mic_analysis_queue[read_index][i] * 256;
    }

    next_index = read_index + 1U;
    if (next_index >= I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT)
    {
      next_index = 0U;
    }
    __DMB();
    i2s_mic_analysis_queue_read = next_index;

    if ((i2s_mic_analysis_last_serviced_sequence != 0U) &&
        (source_sequence !=
         (i2s_mic_analysis_last_serviced_sequence + 1U)))
    {
      i2s_mic_analysis_discontinuity_count++;
      AudioFFT_ResetStream();
      AudioVisualizer_ResetScopeStream();
    }
    i2s_mic_analysis_last_serviced_sequence = source_sequence;

    if (len > 0U)
    {
      AudioFFT_PushSamplesS32(i2s_mic_analysis_s32_buf, len);
      AudioVisualizer_UpdateFromCenteredS32(i2s_mic_analysis_s32_buf, len);
      i2s_mic_analysis_service_count++;
    }

    write_index = i2s_mic_analysis_queue_write;
    available = (write_index >= next_index) ?
                (write_index - next_index) :
                (I2S_MIC_ANALYSIS_QUEUE_BLOCK_COUNT - next_index + write_index);
    i2s_mic_analysis_queue_count = available;
  }
}

static void I2sMic_RecordProcessCycles(uint32_t cycle_start,
                                       uint32_t sample_count)
{
  uint32_t elapsed_cycles = DWT->CYCCNT - cycle_start;
  uint32_t cycle_budget = (uint32_t)(
      ((uint64_t)SystemCoreClock * sample_count) /
      AUDIO_STREAM_SAMPLE_RATE_HZ);

  i2s_mic_process_cycles_last = elapsed_cycles;
  i2s_mic_process_cycle_budget = cycle_budget;
  if (elapsed_cycles > i2s_mic_process_cycles_max)
  {
    i2s_mic_process_cycles_max = elapsed_cycles;
  }
  if ((cycle_budget != 0U) && (elapsed_cycles > cycle_budget))
  {
    i2s_mic_process_deadline_miss_count++;
  }
}

static uint8_t AuxCapture_IsActiveAdc(ADC_HandleTypeDef *hadc)
{
  if ((hadc == NULL) || (aux_adc_handle == NULL))
  {
    return 0U;
  }

  return (hadc->Instance == aux_adc_handle->Instance) ? 1U : 0U;
}

static void AuxCapture_ResetStats(void)
{
  AudioSamples_ResetNoiseMetrics();
  aux_sample_index = 0U;
  aux_adc_analysis_dc_estimate_q8 = 2048 << 8;
  aux_adc_output_dc_estimate_q8 = 2048 << 8;
  arm_biquad_cascade_df2T_init_f32(&aux_output_highpass_filter,
                                    2U,
                                    aux_output_highpass_coeffs,
                                    aux_output_highpass_state);
  arm_biquad_cascade_df2T_init_f32(&aux_output_lowpass_filter,
                                    2U,
                                    aux_output_lowpass_coeffs,
                                    aux_output_lowpass_state);
  aux_ready = 0U;
  aux_adc_raw = 0U;
  aux_adc_avg = 0U;
  aux_adc_min = 0U;
  aux_adc_max = 0U;
  aux_adc_peak = 0U;
  aux_adc_clip_active = 0U;
  aux_adc_clip_sample_count = 0U;
  aux_adc_clip_block_count = 0U;
  aux_adc_headroom_codes = 0U;
  aux_adc_abs_avg = 0U;
  aux_adc_signal_smooth = 0U;
  aux_adc_bias_mv = 0U;
  aux_adc_vref_mv = 3070U;
  aux_adc_sample_count = 0U;
  aux_adc_block_count = 0U;
  aux_adc_error_count = 0U;
  aux_adc_poll_status = HAL_OK;
  aux_adc_timer_start_status = HAL_ERROR;
  aux_adc_hal_state = 0U;
  aux_adc_error_code = 0U;
  aux_adc_dma_state = 0U;
  aux_adc_dma_error_code = 0U;
  aux_adc_dma_ndtr = 0U;
  aux_adc_timer_counter = 0U;
  aux_adc_timer_cr1 = 0U;
  aux_adc_timer_sr = 0U;
  aux_adc_half_count = 0U;
  aux_adc_full_count = 0U;
  aux_adc_pending_count = 0U;
  aux_adc_service_count = 0U;
  aux_adc_overrun_count = 0U;
  aux_analysis_queue_count = 0U;
  aux_analysis_queue_max = 0U;
  aux_analysis_drop_count = 0U;
  aux_analysis_service_count = 0U;
  aux_analysis_source_sequence = 0U;
  aux_analysis_last_serviced_sequence = 0U;
  aux_analysis_discontinuity_count = 0U;
  aux_analysis_tap = AUX_ANALYSIS_TAP_RAW_ADC;
  aux_analysis_last_serviced_tap = AUX_ANALYSIS_TAP_RAW_ADC;
  aux_analysis_raw_origin_adc_q8 = 2048U << 8;
  aux_analysis_raw_origin_valid = 0U;
  aux_analysis_queue_read = 0U;
  aux_analysis_queue_write = 0U;
  aux_output_realtime_push_count = 0U;
  aux_output_realtime_peak = 0U;
  aux_output_realtime_avg = 0U;
  aux_output_realtime_min = 0U;
  aux_output_realtime_max = 0U;
  aux_output_realtime_abs_avg = 0U;
  aux_output_realtime_bias_mv = 0U;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
  aux_output_process_cycles_last = 0U;
  aux_output_process_cycles_max = 0U;
  aux_output_process_cycle_budget = (uint32_t)(
      ((uint64_t)SystemCoreClock * AUX_CAPTURE_BUF_LEN) /
      AUDIO_STREAM_SAMPLE_RATE_HZ);
  aux_output_process_deadline_miss_count = 0U;
  aux_output_gain_q8 = AUX_OUTPUT_DEFAULT_GAIN_Q8;
  aux_output_bias_adc = 2048U;
  aux_output_peak_s16 = 0U;
  aux_output_abs_avg_s16 = 0U;
  aux_output_limiter_count = 0U;
  aux_output_limiter_last_input = 0;
  aux_output_limiter_last_output = 0;
  aux_output_last_sample_s16 = 0;
  aux_output_gate_enable = 0U;
  aux_output_gate_open = 1U;
  aux_output_gate_gain_q8 = 256U;
  aux_output_gate_detector_avg_s16 = 0U;
  aux_output_gate_open_peak_threshold_s16 = AUX_OUTPUT_GATE_OPEN_PEAK_S16;
  aux_output_gate_peak_min_avg_s16 = AUX_OUTPUT_GATE_PEAK_MIN_AVG_S16;
  aux_output_gate_open_avg_threshold_s16 = AUX_OUTPUT_GATE_OPEN_AVG_S16;
  aux_output_gate_close_avg_threshold_s16 = AUX_OUTPUT_GATE_CLOSE_AVG_S16;
  aux_output_gate_hold_remaining_samples = 0U;
  aux_output_gate_peak_qualified = 0U;
  aux_output_gate_rejected_peak_count = 0U;
  aux_output_gate_open_count = 0U;
  aux_output_gate_close_count = 0U;
  aux_output_gate_gain_current_q16 = 65536U;
  aux_output_highpass_enable = 1U;
  aux_output_highpass_peak_s16 = 0U;
  aux_output_lowpass_enable = 1U;
  aux_output_lowpass_peak_s16 = 0U;
  aux_adc_half_pending = 0U;
  aux_adc_full_pending = 0U;
  aux_adc_half_sequence = 0U;
  aux_adc_full_sequence = 0U;

  for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
  {
    aux_sample_buf[i] = 0;
    aux_output_s16_buf[i] = 0;
    aux_output_filter_buf[i] = 0.0f;
    aux_adc_raw_buf[i] = 2048U;
    aux_adc_pending_half_buf[i] = 2048U;
    aux_adc_pending_full_buf[i] = 2048U;
    aux_adc_service_buf[i] = 2048U;
    aux_output_service_buf[i] = 0;
    aux_analysis_s32_buf[i] = 0;
  }
  for (uint32_t block = 0U; block < AUX_ANALYSIS_QUEUE_BLOCK_COUNT; block++)
  {
    aux_analysis_queue_sequence[block] = 0U;
    aux_analysis_queue_tap[block] = AUX_ANALYSIS_TAP_RAW_ADC;
    for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
    {
      aux_analysis_queue[block][i] = 0;
    }
  }
  for (uint32_t i = 0U; i < AUX_ADC_DMA_BUF_LEN; i++)
  {
    aux_adc_dma_buf[i] = 2048U;
  }
  for (uint32_t i = 0U; i < 16U; i++)
  {
    aux_adc_debug_samples[i] = 2048U;
  }
}

static void AuxCapture_QueueDmaBlock(const uint16_t *raw_buf, uint8_t half_block)
{
  uint16_t *target_buf;
  volatile uint8_t *pending_flag;
  volatile uint32_t *sequence;

  if (raw_buf == NULL)
  {
    return;
  }

  if (half_block != 0U)
  {
    target_buf = aux_adc_pending_half_buf;
    pending_flag = &aux_adc_half_pending;
    sequence = &aux_adc_half_sequence;
  }
  else
  {
    target_buf = aux_adc_pending_full_buf;
    pending_flag = &aux_adc_full_pending;
    sequence = &aux_adc_full_sequence;
  }

  if (*pending_flag != 0U)
  {
    aux_adc_overrun_count++;
  }

  (*sequence)++;
  __DMB();
  for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
  {
    target_buf[i] = raw_buf[i];
  }
  __DMB();
  (*sequence)++;

  *pending_flag = 1U;
  aux_adc_pending_count = (uint32_t)aux_adc_half_pending +
                          (uint32_t)aux_adc_full_pending;
}

static void AuxCapture_QueueAnalysisBlock(const uint16_t *raw_buf,
                                          const int16_t *processed_buf,
                                          uint8_t processed_valid)
{
  uint32_t write_index;
  uint32_t next_index;
  uint32_t read_index;
  uint32_t available;
  uint32_t source_sequence;
  uint32_t tap = aux_analysis_tap;

  if (tap != AUX_ANALYSIS_TAP_POST_DSP)
  {
    tap = AUX_ANALYSIS_TAP_RAW_ADC;
  }

  if ((audio_input_source != AUDIO_INPUT_SOURCE_AUX) ||
      ((tap == AUX_ANALYSIS_TAP_RAW_ADC) && (raw_buf == NULL)) ||
      ((tap == AUX_ANALYSIS_TAP_POST_DSP) &&
       ((processed_buf == NULL) || (processed_valid == 0U))))
  {
    return;
  }

  source_sequence = aux_analysis_source_sequence + 1U;
  aux_analysis_source_sequence = source_sequence;

  write_index = aux_analysis_queue_write;
  read_index = aux_analysis_queue_read;
  next_index = write_index + 1U;
  if (next_index >= AUX_ANALYSIS_QUEUE_BLOCK_COUNT)
  {
    next_index = 0U;
  }

  if (next_index == read_index)
  {
    aux_analysis_drop_count++;
    return;
  }

  for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
  {
    if (tap == AUX_ANALYSIS_TAP_RAW_ADC)
    {
      uint32_t raw = raw_buf[i];
      if (raw > 4095U)
      {
        raw = 4095U;
      }
      aux_analysis_queue[write_index][i] = (int16_t)raw;
    }
    else
    {
      aux_analysis_queue[write_index][i] = processed_buf[i];
    }
  }
  aux_analysis_queue_sequence[write_index] = source_sequence;
  aux_analysis_queue_tap[write_index] = (uint8_t)tap;
  __DMB();
  aux_analysis_queue_write = next_index;

  available = (next_index >= read_index) ?
              (next_index - read_index) :
              (AUX_ANALYSIS_QUEUE_BLOCK_COUNT - read_index + next_index);
  aux_analysis_queue_count = available;
  if (available > aux_analysis_queue_max)
  {
    aux_analysis_queue_max = available;
  }
}

static void AuxCapture_ServiceAnalysis(void)
{
  for (uint32_t block = 0U;
       block < AUX_ANALYSIS_MAX_BLOCKS_PER_SERVICE;
       block++)
  {
    uint32_t read_index = aux_analysis_queue_read;
    uint32_t write_index = aux_analysis_queue_write;
    uint32_t next_index;
    uint32_t available;
    uint32_t source_sequence;
    uint32_t block_tap;
    uint8_t reset_analysis_stream = 0U;

    if (read_index == write_index)
    {
      aux_analysis_queue_count = 0U;
      break;
    }

    __DMB();
    source_sequence = aux_analysis_queue_sequence[read_index];
    block_tap = aux_analysis_queue_tap[read_index];
    if (block_tap != AUX_ANALYSIS_TAP_POST_DSP)
    {
      block_tap = AUX_ANALYSIS_TAP_RAW_ADC;
    }
    for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
    {
      aux_output_service_buf[i] = aux_analysis_queue[read_index][i];
    }

    next_index = read_index + 1U;
    if (next_index >= AUX_ANALYSIS_QUEUE_BLOCK_COUNT)
    {
      next_index = 0U;
    }
    __DMB();
    aux_analysis_queue_read = next_index;

    if ((aux_analysis_last_serviced_sequence != 0U) &&
        (source_sequence != (aux_analysis_last_serviced_sequence + 1U)))
    {
      aux_analysis_discontinuity_count++;
      reset_analysis_stream = 1U;
    }
    if ((aux_analysis_last_serviced_sequence != 0U) &&
        (block_tap != aux_analysis_last_serviced_tap))
    {
      /* Never combine RAW_ADC and POST_DSP samples in one analysis frame. */
      reset_analysis_stream = 1U;
    }

    if (block_tap == AUX_ANALYSIS_TAP_RAW_ADC)
    {
      if ((aux_analysis_raw_origin_valid == 0U) ||
          (reset_analysis_stream != 0U))
      {
        uint64_t raw_sum = 0U;

        for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
        {
          raw_sum += (uint16_t)aux_output_service_buf[i];
        }
        /* One stable affine origin avoids artificial 128-sample DC steps. */
        aux_analysis_raw_origin_adc_q8 = (uint32_t)(
            ((raw_sum * 256U) + (AUX_CAPTURE_BUF_LEN / 2U)) /
            AUX_CAPTURE_BUF_LEN);
        aux_analysis_raw_origin_valid = 1U;
      }

      for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
      {
        int32_t raw_q8 = (int32_t)(uint16_t)aux_output_service_buf[i] * 256;
        int32_t centered_q8 =
            raw_q8 - (int32_t)aux_analysis_raw_origin_adc_q8;

        /* This is an exact, stream-continuous affine map into signed S24. */
        aux_analysis_s32_buf[i] = centered_q8 * 16;
      }
    }
    else
    {
      aux_analysis_raw_origin_valid = 0U;
      for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
      {
        aux_analysis_s32_buf[i] = (int32_t)aux_output_service_buf[i] * 256;
      }
    }
    if (reset_analysis_stream != 0U)
    {
      AudioFFT_ResetStream();
      AudioVisualizer_ResetScopeStream();
      AudioSamples_ResetNoiseMetrics();
    }
    aux_analysis_last_serviced_sequence = source_sequence;
    aux_analysis_last_serviced_tap = block_tap;
    AudioSamples_UpdateFromS32(aux_analysis_s32_buf, AUX_CAPTURE_BUF_LEN);
    AudioFFT_PushSamplesS32(aux_analysis_s32_buf, AUX_CAPTURE_BUF_LEN);
    AudioVisualizer_UpdateFromCenteredS32(aux_analysis_s32_buf,
                                          AUX_CAPTURE_BUF_LEN);
    aux_analysis_service_count++;

    write_index = aux_analysis_queue_write;
    available = (write_index >= next_index) ?
                (write_index - next_index) :
                (AUX_ANALYSIS_QUEUE_BLOCK_COUNT - next_index + write_index);
    aux_analysis_queue_count = available;
  }
}

static void AuxCapture_RecordOutputCycles(uint32_t cycle_start)
{
  uint32_t elapsed_cycles = DWT->CYCCNT - cycle_start;

  aux_output_process_cycles_last = elapsed_cycles;
  if (elapsed_cycles > aux_output_process_cycles_max)
  {
    aux_output_process_cycles_max = elapsed_cycles;
  }
  if ((aux_output_process_cycle_budget != 0U) &&
      (elapsed_cycles > aux_output_process_cycle_budget))
  {
    aux_output_process_deadline_miss_count++;
  }
}

static int16_t AuxCapture_LimitOutput16(int32_t value)
{
  int32_t magnitude = value;
  int32_t limited;

  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }

  if (magnitude <= AUX_OUTPUT_LIMIT_THRESHOLD)
  {
    return (int16_t)value;
  }

  aux_output_limiter_count++;
  aux_output_limiter_last_input = value;

  limited = AUX_OUTPUT_LIMIT_THRESHOLD +
            ((magnitude - AUX_OUTPUT_LIMIT_THRESHOLD) / 8);
  if (limited > AUX_OUTPUT_LIMIT_MAX)
  {
    limited = AUX_OUTPUT_LIMIT_MAX;
  }

  if (value < 0)
  {
    limited = -limited;
  }

  aux_output_limiter_last_output = (int16_t)limited;
  return (int16_t)limited;
}

static uint8_t AuxCapture_PushOutputRawBlock(const uint16_t *raw_buf, uint32_t len)
{
  uint32_t peak = 0U;
  uint32_t min_value = UINT32_MAX;
  uint32_t max_value = 0U;
  uint32_t sum = 0U;
  uint32_t abs_sum = 0U;
  uint32_t output_peak = 0U;
  uint32_t output_abs_sum = 0U;
  uint32_t highpass_peak = 0U;
  uint32_t lowpass_peak = 0U;
  uint32_t post_gate_peak = 0U;
  uint32_t post_gate_abs_sum = 0U;
  uint32_t pre_gate_abs_avg;
  uint32_t gate_target_q16;
  uint32_t gate_was_open;
  uint32_t peak_candidate;
  uint32_t peak_qualified;
  uint32_t clip_samples = 0U;
  uint32_t headroom_codes = 4095U;

  if ((raw_buf == NULL) || (len == 0U) ||
      (audio_input_source != AUDIO_INPUT_SOURCE_AUX) ||
      (aux_output_realtime_enable == 0U))
  {
    return 0U;
  }

  if (len > AUX_CAPTURE_BUF_LEN)
  {
    len = AUX_CAPTURE_BUF_LEN;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    uint32_t raw = raw_buf[i];
    uint32_t sample_headroom;
    int32_t raw_q8;
    int32_t centered_q8;
    int32_t scaled_sample;
    uint32_t magnitude;

    if (raw > 4095U)
    {
      raw = 4095U;
    }

    sample_headroom = raw;
    if ((4095U - raw) < sample_headroom)
    {
      sample_headroom = 4095U - raw;
    }
    if (sample_headroom < headroom_codes)
    {
      headroom_codes = sample_headroom;
    }
    if ((raw <= AUX_ADC_NEAR_RAIL_MARGIN_CODES) ||
        (raw >= (4095U - AUX_ADC_NEAR_RAIL_MARGIN_CODES)))
    {
      clip_samples++;
    }

    raw_q8 = (int32_t)raw << 8;
    aux_adc_output_dc_estimate_q8 +=
        (raw_q8 - aux_adc_output_dc_estimate_q8) >> AUX_OUTPUT_DC_TRACK_SHIFT;
    centered_q8 = raw_q8 - aux_adc_output_dc_estimate_q8;
    scaled_sample = (int32_t)(((int64_t)centered_q8 *
                               16LL *
                               (int64_t)aux_output_gain_q8) >> 16);
    aux_output_filter_buf[i] = (float32_t)scaled_sample;

    if (raw < min_value)
    {
      min_value = raw;
    }
    if (raw > max_value)
    {
      max_value = raw;
    }
    sum += raw;

    magnitude = (centered_q8 < 0) ?
                (uint32_t)((-centered_q8) >> 8) :
                (uint32_t)(centered_q8 >> 8);
    abs_sum += magnitude;
    if (magnitude > peak)
    {
      peak = magnitude;
    }

  }

  aux_adc_clip_active = (clip_samples != 0U) ? 1U : 0U;
  aux_adc_clip_sample_count += clip_samples;
  aux_adc_headroom_codes = headroom_codes;
  if (clip_samples != 0U)
  {
    aux_adc_clip_block_count++;
  }

  if (aux_output_highpass_enable != 0U)
  {
    arm_biquad_cascade_df2T_f32(&aux_output_highpass_filter,
                                 aux_output_filter_buf,
                                 aux_output_filter_buf,
                                 len);
  }
  else
  {
    for (uint32_t i = 0U;
         i < (sizeof(aux_output_highpass_state) /
              sizeof(aux_output_highpass_state[0]));
         i++)
    {
      aux_output_highpass_state[i] = 0.0f;
    }
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    float32_t sample = aux_output_filter_buf[i];
    uint32_t magnitude = (sample < 0.0f) ?
                         (uint32_t)(-sample) : (uint32_t)sample;

    if (magnitude > highpass_peak)
    {
      highpass_peak = magnitude;
    }
  }

  if (aux_output_lowpass_enable != 0U)
  {
    arm_biquad_cascade_df2T_f32(&aux_output_lowpass_filter,
                                 aux_output_filter_buf,
                                 aux_output_filter_buf,
                                 len);
  }
  else
  {
    for (uint32_t i = 0U;
         i < (sizeof(aux_output_lowpass_state) /
              sizeof(aux_output_lowpass_state[0]));
         i++)
    {
      aux_output_lowpass_state[i] = 0.0f;
    }
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    float32_t filtered_sample = aux_output_filter_buf[i];
    int32_t scaled_sample;
    int16_t output_sample;
    uint32_t filtered_magnitude;
    uint32_t output_magnitude;

    if (filtered_sample > 2147483000.0f)
    {
      scaled_sample = 2147483000;
    }
    else if (filtered_sample < -2147483000.0f)
    {
      scaled_sample = -2147483000;
    }
    else
    {
      scaled_sample = (filtered_sample >= 0.0f) ?
                      (int32_t)(filtered_sample + 0.5f) :
                      (int32_t)(filtered_sample - 0.5f);
    }

    filtered_magnitude = (scaled_sample < 0) ?
                         (uint32_t)(-scaled_sample) :
                         (uint32_t)scaled_sample;
    if (filtered_magnitude > lowpass_peak)
    {
      lowpass_peak = filtered_magnitude;
    }

    output_sample = AuxCapture_LimitOutput16(scaled_sample);
    aux_output_s16_buf[i] = output_sample;
    output_magnitude = (output_sample < 0) ?
                       (uint32_t)(-output_sample) :
                       (uint32_t)output_sample;
    output_abs_sum += output_magnitude;
    if (output_magnitude > output_peak)
    {
      output_peak = output_magnitude;
    }
  }

  pre_gate_abs_avg = output_abs_sum / len;
  aux_output_gate_detector_avg_s16 = pre_gate_abs_avg;
  gate_was_open = aux_output_gate_open;
  peak_candidate =
      (output_peak >= aux_output_gate_open_peak_threshold_s16) ? 1U : 0U;
  peak_qualified =
      ((peak_candidate != 0U) &&
       (pre_gate_abs_avg >= aux_output_gate_peak_min_avg_s16)) ? 1U : 0U;
  aux_output_gate_peak_qualified = peak_qualified;

  /*
   * A single pickup spike used to open the AUX path at full gain for the
   * complete hold/release interval. Qualifying the peak with a small block
   * average rejects narrow electrical impulses while the independent average
   * trigger still passes quiet sustained audio and music normally.
   */
  if ((peak_candidate != 0U) && (peak_qualified == 0U) &&
      (pre_gate_abs_avg < aux_output_gate_open_avg_threshold_s16))
  {
    aux_output_gate_rejected_peak_count++;
  }

  if (aux_output_gate_enable == 0U)
  {
    gate_target_q16 = 65536U;
    aux_output_gate_open = 1U;
    aux_output_gate_gain_current_q16 = 65536U;
    aux_output_gate_hold_remaining_samples = AUX_OUTPUT_GATE_HOLD_SAMPLES;
  }
  else if ((peak_qualified != 0U) ||
           (pre_gate_abs_avg >= aux_output_gate_open_avg_threshold_s16))
  {
    gate_target_q16 = 65536U;
    aux_output_gate_open = 1U;
    aux_output_gate_hold_remaining_samples = AUX_OUTPUT_GATE_HOLD_SAMPLES;
  }
  else if (aux_output_gate_hold_remaining_samples != 0U)
  {
    aux_output_gate_hold_remaining_samples =
        (aux_output_gate_hold_remaining_samples > len) ?
        (aux_output_gate_hold_remaining_samples - len) : 0U;
    gate_target_q16 = 65536U;
    aux_output_gate_open = 1U;
  }
  else if (pre_gate_abs_avg <= aux_output_gate_close_avg_threshold_s16)
  {
    gate_target_q16 = 0U;
    aux_output_gate_open = 0U;
  }
  else
  {
    gate_target_q16 = (aux_output_gate_open != 0U) ? 65536U : 0U;
  }

  if ((gate_was_open == 0U) && (aux_output_gate_open != 0U))
  {
    aux_output_gate_open_count++;
  }
  else if ((gate_was_open != 0U) && (aux_output_gate_open == 0U))
  {
    aux_output_gate_close_count++;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    int32_t gated_sample;
    uint32_t output_magnitude;

    if (aux_output_gate_gain_current_q16 < gate_target_q16)
    {
      aux_output_gate_gain_current_q16 += AUX_OUTPUT_GATE_ATTACK_STEP_Q16;
      if (aux_output_gate_gain_current_q16 > gate_target_q16)
      {
        aux_output_gate_gain_current_q16 = gate_target_q16;
      }
    }
    else if (aux_output_gate_gain_current_q16 > gate_target_q16)
    {
      if (aux_output_gate_gain_current_q16 > AUX_OUTPUT_GATE_RELEASE_STEP_Q16)
      {
        aux_output_gate_gain_current_q16 -= AUX_OUTPUT_GATE_RELEASE_STEP_Q16;
      }
      else
      {
        aux_output_gate_gain_current_q16 = 0U;
      }
    }

    gated_sample = ((int32_t)aux_output_s16_buf[i] *
                    (int32_t)aux_output_gate_gain_current_q16) >> 16;
    aux_output_s16_buf[i] = (int16_t)gated_sample;

    output_magnitude = (gated_sample < 0) ?
                       (uint32_t)(-gated_sample) :
                       (uint32_t)gated_sample;
    post_gate_abs_sum += output_magnitude;
    if (output_magnitude > post_gate_peak)
    {
      post_gate_peak = output_magnitude;
    }
  }

  AudioOutput_PushSamplesS16(aux_output_s16_buf, len);
  aux_output_realtime_push_count += len;
  aux_output_realtime_peak = peak;
  aux_output_realtime_avg = sum / len;
  aux_output_realtime_min = min_value;
  aux_output_realtime_max = max_value;
  aux_output_realtime_abs_avg = abs_sum / len;
  aux_output_realtime_bias_mv = (aux_output_realtime_avg * aux_adc_vref_mv) / 4095U;
  aux_output_bias_adc = (uint32_t)(aux_adc_output_dc_estimate_q8 >> 8);
  aux_output_peak_s16 = post_gate_peak;
  aux_output_abs_avg_s16 = post_gate_abs_sum / len;
  aux_output_last_sample_s16 = aux_output_s16_buf[len - 1U];
  aux_output_gate_gain_q8 = aux_output_gate_gain_current_q16 >> 8;
  aux_output_highpass_peak_s16 = highpass_peak;
  aux_output_lowpass_peak_s16 = lowpass_peak;
  return 1U;
}

static void AuxCapture_ProcessRawBlock(const uint16_t *raw_buf, uint32_t len)
{
  uint32_t min_value = UINT32_MAX;
  uint32_t max_value = 0U;
  uint64_t sum = 0U;
  uint64_t abs_sum = 0U;
  uint32_t peak = 0U;

  if ((raw_buf == NULL) || (len == 0U))
  {
    return;
  }

  if (len > AUX_CAPTURE_BUF_LEN)
  {
    len = AUX_CAPTURE_BUF_LEN;
  }

  for (uint32_t i = 0U; i < len; i++)
  {
    uint32_t raw = raw_buf[i];
    int32_t raw_q8;
    int32_t centered_q8;
    int32_t centered;
    uint32_t magnitude;

    if (raw > 4095U)
    {
      raw = 4095U;
    }

    aux_adc_raw_buf[i] = (uint16_t)raw;
    if (i < 16U)
    {
      aux_adc_debug_samples[i] = (uint16_t)raw;
    }

    /*
     * Keep fractional precision in the diagnostic DC tracker. The former
     * integer-code tracker stopped moving inside a +/-127-code dead band and
     * could report bias error as idle signal even though the realtime path was
     * correctly centered.
     */
    raw_q8 = (int32_t)raw << 8;
    aux_adc_analysis_dc_estimate_q8 +=
        (raw_q8 - aux_adc_analysis_dc_estimate_q8) / 128;
    centered_q8 = raw_q8 - aux_adc_analysis_dc_estimate_q8;
    centered = centered_q8 / 256;
    aux_sample_buf[i] = centered * 4096;
    magnitude = (centered < 0) ? (uint32_t)(-centered) : (uint32_t)centered;

    if (raw < min_value)
    {
      min_value = raw;
    }
    if (raw > max_value)
    {
      max_value = raw;
    }

    sum += raw;
    abs_sum += magnitude;
    if (magnitude > peak)
    {
      peak = magnitude;
    }
  }

  aux_adc_raw = aux_adc_raw_buf[len - 1U];
  aux_adc_avg = (uint32_t)(sum / len);
  aux_adc_min = min_value;
  aux_adc_max = max_value;
  aux_adc_peak = peak;
  aux_adc_abs_avg = (uint32_t)(abs_sum / len);
  aux_adc_signal_smooth = ((aux_adc_signal_smooth * 7U) + aux_adc_abs_avg) / 8U;
  aux_adc_bias_mv = (aux_adc_avg * aux_adc_vref_mv) / 4095U;
  aux_adc_sample_count += len;
  aux_adc_block_count++;
  aux_ready = 1U;

  if (audio_input_source == AUDIO_INPUT_SOURCE_AUX)
  {
    if ((aux_output_realtime_enable == 0U) || (aux_timer_handle == NULL))
    {
      AudioSamples_UpdateFromS32(aux_sample_buf, len);
      AudioOutput_PushSamplesS32(aux_sample_buf, len);
      AudioFFT_PushSamplesS32(aux_sample_buf, len);
      AudioVisualizer_UpdateFromS32(aux_sample_buf, len);
    }
  }
}
