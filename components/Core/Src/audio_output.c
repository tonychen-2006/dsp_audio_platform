#include "audio_output.h"

#define AUDIO_OUT_MIC_RING_LEN 1024U
#define AUDIO_OUT_DEFAULT_MIC_GAIN_Q8 512U
#define AUDIO_OUT_LIMIT_THRESHOLD 28000
#define AUDIO_OUT_LIMIT_MAX 32000

uint16_t audio_tx_buf[AUDIO_OUT_BUF_LEN];
volatile uint32_t audio_out_start_status = HAL_ERROR;
volatile uint32_t audio_out_i2s_state_before_start = 0U;
volatile uint32_t audio_out_i2s_state_after_start = 0U;
volatile uint32_t audio_out_i2s_error_code = 0U;
volatile uint32_t audio_out_dma_state_before_start = 0U;
volatile uint32_t audio_out_dma_state_after_start = 0U;
volatile uint32_t audio_out_dma_error_code = 0U;
volatile uint32_t audio_out_hdmatx_is_null = 1U;
volatile uint32_t audio_out_half_count = 0U;
volatile uint32_t audio_out_full_count = 0U;
volatile uint32_t audio_out_fill_count = 0U;
volatile uint32_t audio_out_last_sample = 0U;
volatile int16_t audio_out_last_sample_s16 = 0;
volatile uint32_t audio_out_tx_peak = 0U;
volatile uint32_t audio_out_tx_abs_avg = 0U;
volatile uint32_t audio_out_tx_level_smooth = 0U;
volatile uint32_t audio_out_tx_debug_update_count = 0U;
volatile uint32_t audio_out_tx_zero_block_count = 0U;
volatile uint32_t audio_out_tx_nonzero_block_count = 0U;
volatile int16_t audio_out_tx_debug_samples[16];
volatile uint8_t audio_out_force_test_tone = 0U;
volatile uint32_t audio_out_mode_debug = AUDIO_OUTPUT_MODE_TEST_TONE;
volatile uint32_t audio_out_test_tone_divider = 2U;
volatile uint32_t audio_out_block_len_debug = 0U;
volatile uint32_t audio_out_dma_size_debug = 0U;
volatile uint32_t audio_out_mic_push_count = 0U;
volatile uint32_t audio_out_mic_pop_count = 0U;
volatile uint32_t audio_out_mic_underrun_count = 0U;
volatile uint32_t audio_out_mic_overrun_count = 0U;
volatile uint32_t audio_out_mic_available = 0U;
volatile uint32_t audio_out_mic_peak = 0U;
volatile uint32_t audio_out_mic_prefill_threshold = 256U;
volatile uint32_t audio_out_mic_wait_count = 0U;
volatile uint8_t audio_out_mic_streaming = 0U;
volatile uint32_t audio_out_mic_available_min = AUDIO_OUT_MIC_RING_LEN;
volatile uint32_t audio_out_mic_available_max = 0U;
/*
 * Q8 gain for mic monitor:
 *   256  = 1x
 *   512  = 2x
 *   1024 = 4x
 *   2048 = 8x
 *   4096 = 16x
 *
 * A strong AUX source can already reach most of the ADC range. Start at 2x
 * so test tones have headroom; raise this from Live Expressions for quiet
 * sources like phone/laptop music output.
 */
volatile uint32_t audio_out_mic_gain_q8 = AUDIO_OUT_DEFAULT_MIC_GAIN_Q8;
volatile uint32_t audio_out_mic_repeat_factor = 1U;
volatile uint32_t audio_out_mic_hold_count = 0U;
volatile int16_t audio_out_mic_last_sample_s16 = 0;
volatile uint32_t audio_out_limiter_count = 0U;
volatile int32_t audio_out_limiter_last_input = 0;
volatile int16_t audio_out_limiter_last_output = 0;
volatile uint8_t audio_out_ready = 0U;
volatile uint32_t audio_pwm_out_start_status = HAL_ERROR;
volatile uint32_t audio_pwm_out_pwm_start_status = HAL_ERROR;
volatile uint32_t audio_pwm_out_timer_start_status = HAL_ERROR;
volatile uint32_t audio_pwm_out_sample_count = 0U;
volatile uint32_t audio_pwm_out_samples_per_sec = 0U;
volatile uint32_t audio_pwm_out_last_duty = 0U;
volatile uint32_t audio_pwm_out_peak = 0U;
volatile uint32_t audio_pwm_out_last_peak = 0U;
volatile uint32_t audio_pwm_out_level_smooth = 0U;
volatile uint32_t audio_pwm_out_clip_count = 0U;
volatile int16_t audio_pwm_out_last_sample_s16 = 0;
volatile uint8_t audio_pwm_out_ready = 0U;

static int16_t audio_out_mic_ring[AUDIO_OUT_MIC_RING_LEN];
static I2S_HandleTypeDef *audio_out_i2s_handle = NULL;
static TIM_HandleTypeDef *audio_pwm_out_pwm_handle = NULL;
static TIM_HandleTypeDef *audio_pwm_out_sample_handle = NULL;
static uint32_t audio_pwm_out_pwm_channel = TIM_CHANNEL_1;
static uint32_t audio_pwm_out_period = 255U;
static uint32_t audio_out_block_len = AUDIO_OUT_BUF_LEN / 2U;
static AudioOutput_Mode_t audio_out_mode = AUDIO_OUTPUT_MODE_TEST_TONE;
static uint32_t audio_out_phase = 0U;
static volatile uint32_t audio_out_mic_write_index = 0U;
static volatile uint32_t audio_out_mic_read_index = 0U;
static int32_t audio_out_mic_dc_estimate = 0;
static uint32_t audio_out_mic_repeat_remaining = 0U;
static uint32_t audio_pwm_out_rate_last_tick = 0U;
static uint32_t audio_pwm_out_rate_window_count = 0U;

static const int16_t audio_out_sine_lut[32] =
{
      0,   6393,  12539,  18204,  23170,  27245,  30273,  32137,
  32767,  32137,  30273,  27245,  23170,  18204,  12539,   6393,
      0,  -6393, -12539, -18204, -23170, -27245, -30273, -32137,
 -32767, -32137, -30273, -27245, -23170, -18204, -12539,  -6393
};

static uint16_t AudioOutput_DmaSizeFrames(I2S_HandleTypeDef *hi2s);
static uint8_t AudioOutput_IsActiveI2S(I2S_HandleTypeDef *hi2s);
static int16_t AudioOutput_Limit16(int32_t value);
static int16_t AudioOutput_ConvertMicSample(int32_t sample);
static int16_t AudioOutput_ReadMicSample(void);
static int16_t AudioOutput_NextSample16(void);
static uint32_t AudioOutput_Abs16(int16_t value);
static uint32_t AudioOutput_MicAvailable(void);
static void AudioOutput_UpdateAvailableDebug(uint32_t available);
static void AudioOutput_UpdateTxDebug(const uint16_t *buf, uint32_t len);
static void AudioOutput_WriteStereo16(uint16_t *buf, uint32_t len);
static void AudioOutput_WriteStereo32Frame(uint16_t *buf, uint32_t len);

HAL_StatusTypeDef AudioOutput_Start(I2S_HandleTypeDef *hi2s)
{
  HAL_StatusTypeDef status;
  uint32_t prefilled_available;
  uint16_t dma_size;

  if (hi2s == NULL)
  {
    audio_out_start_status = HAL_ERROR;
    audio_out_hdmatx_is_null = 1U;
    return HAL_ERROR;
  }

  audio_out_i2s_state_before_start = hi2s->State;
  audio_out_i2s_error_code = hi2s->ErrorCode;
  audio_out_hdmatx_is_null = (hi2s->hdmatx == NULL) ? 1U : 0U;
  if (hi2s->hdmatx != NULL)
  {
    audio_out_dma_state_before_start = hi2s->hdmatx->State;
    audio_out_dma_error_code = hi2s->hdmatx->ErrorCode;
  }
  else
  {
    audio_out_dma_state_before_start = 0xFFFFFFFFU;
    audio_out_dma_error_code = 0xFFFFFFFFU;
  }

  prefilled_available = AudioOutput_MicAvailable();

  audio_out_i2s_handle = hi2s;
  audio_out_ready = 0U;
  audio_out_half_count = 0U;
  audio_out_full_count = 0U;
  audio_out_fill_count = 0U;
  audio_out_last_sample = 0U;
  audio_out_last_sample_s16 = 0;
  audio_out_tx_peak = 0U;
  audio_out_tx_abs_avg = 0U;
  audio_out_tx_level_smooth = 0U;
  audio_out_tx_debug_update_count = 0U;
  audio_out_tx_zero_block_count = 0U;
  audio_out_tx_nonzero_block_count = 0U;
  audio_out_phase = 0U;
  audio_out_mic_underrun_count = 0U;
  audio_out_mic_overrun_count = 0U;
  audio_out_mic_peak = 0U;
  audio_out_mic_wait_count = 0U;
  audio_out_mic_streaming = 0U;
  audio_out_mic_available_min = AUDIO_OUT_MIC_RING_LEN;
  audio_out_mic_available_max = 0U;
  audio_out_mic_hold_count = 0U;
  audio_out_mic_repeat_remaining = 0U;
  audio_out_limiter_count = 0U;
  audio_out_limiter_last_input = 0;
  audio_out_limiter_last_output = 0;

  if (prefilled_available == 0U)
  {
    audio_out_mic_push_count = 0U;
    audio_out_mic_pop_count = 0U;
    AudioOutput_UpdateAvailableDebug(0U);
    audio_out_mic_last_sample_s16 = 0;
    audio_out_mic_write_index = 0U;
    audio_out_mic_read_index = 0U;
    audio_out_mic_dc_estimate = 0;
  }
  else
  {
    audio_out_mic_pop_count = 0U;
    AudioOutput_UpdateAvailableDebug(prefilled_available);
  }

  dma_size = AudioOutput_DmaSizeFrames(hi2s);
  audio_out_dma_size_debug = dma_size;
  audio_out_block_len_debug = audio_out_block_len;

  AudioOutput_FillBlock(&audio_tx_buf[0], audio_out_block_len);
  AudioOutput_FillBlock(&audio_tx_buf[audio_out_block_len], audio_out_block_len);

  status = HAL_I2S_Transmit_DMA(hi2s, audio_tx_buf, dma_size);
  audio_out_start_status = status;
  audio_out_i2s_state_after_start = hi2s->State;
  audio_out_i2s_error_code = hi2s->ErrorCode;
  if (hi2s->hdmatx != NULL)
  {
    audio_out_dma_state_after_start = hi2s->hdmatx->State;
    audio_out_dma_error_code = hi2s->hdmatx->ErrorCode;
  }
  else
  {
    audio_out_dma_state_after_start = 0xFFFFFFFFU;
    audio_out_dma_error_code = 0xFFFFFFFFU;
  }

  return status;
}

void AudioOutput_SetMode(AudioOutput_Mode_t mode)
{
  audio_out_mode = mode;
  audio_out_mode_debug = (uint32_t)mode;
}

void AudioOutput_FillBlock(uint16_t *buf, uint32_t len)
{
  if ((buf == NULL) || (len == 0U))
  {
    return;
  }

  if ((audio_out_i2s_handle != NULL) &&
      ((audio_out_i2s_handle->Init.DataFormat == I2S_DATAFORMAT_24B) ||
       (audio_out_i2s_handle->Init.DataFormat == I2S_DATAFORMAT_32B)))
  {
    AudioOutput_WriteStereo32Frame(buf, len);
  }
  else
  {
    AudioOutput_WriteStereo16(buf, len);
  }

  AudioOutput_UpdateTxDebug(buf, len);
  audio_out_fill_count++;
  audio_out_ready = 1U;
}

void AudioOutput_PushSamplesS32(const int32_t *samples, uint32_t count)
{
  if ((samples == NULL) || (count == 0U))
  {
    return;
  }

  for (uint32_t i = 0U; i < count; i++)
  {
    uint32_t next_index;
    int16_t sample16 = AudioOutput_ConvertMicSample(samples[i]);

    next_index = audio_out_mic_write_index + 1U;
    if (next_index >= AUDIO_OUT_MIC_RING_LEN)
    {
      next_index = 0U;
    }

    if (next_index == audio_out_mic_read_index)
    {
      audio_out_mic_overrun_count++;
      audio_out_mic_read_index++;
      if (audio_out_mic_read_index >= AUDIO_OUT_MIC_RING_LEN)
      {
        audio_out_mic_read_index = 0U;
      }
    }

    audio_out_mic_ring[audio_out_mic_write_index] = sample16;
    audio_out_mic_write_index = next_index;
    audio_out_mic_push_count++;
  }

  AudioOutput_UpdateAvailableDebug(AudioOutput_MicAvailable());
}

HAL_StatusTypeDef AudioPwmOutput_Start(TIM_HandleTypeDef *pwm_htim,
                                       uint32_t pwm_channel,
                                       TIM_HandleTypeDef *sample_htim)
{
  HAL_StatusTypeDef status;
  uint32_t prefilled_available;
  uint32_t midpoint;

  if ((pwm_htim == NULL) || (sample_htim == NULL))
  {
    audio_pwm_out_start_status = HAL_ERROR;
    return HAL_ERROR;
  }

  prefilled_available = AudioOutput_MicAvailable();
  audio_pwm_out_pwm_handle = pwm_htim;
  audio_pwm_out_sample_handle = sample_htim;
  audio_pwm_out_pwm_channel = pwm_channel;
  audio_pwm_out_period = __HAL_TIM_GET_AUTORELOAD(pwm_htim);
  midpoint = (audio_pwm_out_period + 1U) / 2U;

  audio_pwm_out_sample_count = 0U;
  audio_pwm_out_samples_per_sec = 0U;
  audio_pwm_out_last_duty = midpoint;
  audio_pwm_out_peak = 0U;
  audio_pwm_out_last_peak = 0U;
  audio_pwm_out_level_smooth = 0U;
  audio_pwm_out_clip_count = 0U;
  audio_pwm_out_last_sample_s16 = 0;
  audio_pwm_out_ready = 0U;
  audio_out_mic_underrun_count = 0U;
  audio_out_mic_overrun_count = 0U;
  audio_out_mic_peak = 0U;
  audio_out_mic_wait_count = 0U;
  audio_out_mic_streaming = 0U;
  audio_out_mic_available_min = AUDIO_OUT_MIC_RING_LEN;
  audio_out_mic_available_max = 0U;
  audio_out_mic_hold_count = 0U;
  audio_out_mic_repeat_remaining = 0U;
  audio_out_limiter_count = 0U;
  audio_out_limiter_last_input = 0;
  audio_out_limiter_last_output = 0;
  audio_pwm_out_rate_last_tick = HAL_GetTick();
  audio_pwm_out_rate_window_count = 0U;

  if (prefilled_available == 0U)
  {
    audio_out_mic_push_count = 0U;
    audio_out_mic_pop_count = 0U;
    AudioOutput_UpdateAvailableDebug(0U);
    audio_out_mic_last_sample_s16 = 0;
    audio_out_mic_write_index = 0U;
    audio_out_mic_read_index = 0U;
    audio_out_mic_dc_estimate = 0;
  }
  else
  {
    audio_out_mic_pop_count = 0U;
    AudioOutput_UpdateAvailableDebug(prefilled_available);
  }

  __HAL_TIM_SET_COMPARE(pwm_htim, pwm_channel, midpoint);

  status = HAL_TIM_PWM_Start(pwm_htim, pwm_channel);
  audio_pwm_out_pwm_start_status = status;
  if (status != HAL_OK)
  {
    audio_pwm_out_start_status = status;
    return status;
  }

  status = HAL_TIM_Base_Start_IT(sample_htim);
  audio_pwm_out_timer_start_status = status;
  audio_pwm_out_start_status = status;
  if (status == HAL_OK)
  {
    audio_pwm_out_ready = 1U;
    audio_out_ready = 1U;
  }

  return status;
}

void AudioPwmOutput_HandleSampleTimer(TIM_HandleTypeDef *htim)
{
  int16_t sample;
  uint32_t duty;
  uint32_t magnitude;
  uint32_t pwm_steps;

  if ((htim == NULL) ||
      (audio_pwm_out_sample_handle == NULL) ||
      (htim->Instance != audio_pwm_out_sample_handle->Instance) ||
      (audio_pwm_out_pwm_handle == NULL))
  {
    return;
  }

  sample = AudioOutput_NextSample16();
  pwm_steps = audio_pwm_out_period + 1U;
  duty = (uint32_t)((((int64_t)sample + 32768LL) *
                     (int64_t)pwm_steps) >> 16);
  if (duty > audio_pwm_out_period)
  {
    duty = audio_pwm_out_period;
  }
  __HAL_TIM_SET_COMPARE(audio_pwm_out_pwm_handle,
                        audio_pwm_out_pwm_channel,
                        duty);

  magnitude = AudioOutput_Abs16(sample);
  if ((sample >= 32767) || (sample <= -32768))
  {
    audio_pwm_out_clip_count++;
  }
  if (magnitude > audio_pwm_out_peak)
  {
    audio_pwm_out_peak = magnitude;
  }
  audio_pwm_out_last_peak = magnitude;
  audio_pwm_out_level_smooth = ((audio_pwm_out_level_smooth * 31U) + magnitude) / 32U;
  audio_pwm_out_last_sample_s16 = sample;
  audio_pwm_out_last_duty = duty;
  audio_pwm_out_sample_count++;
  audio_pwm_out_rate_window_count++;
  if ((audio_pwm_out_rate_window_count & 0x3FU) == 0U)
  {
    uint32_t now_tick = HAL_GetTick();
    uint32_t elapsed_ms = now_tick - audio_pwm_out_rate_last_tick;

    if (elapsed_ms >= 1000U)
    {
      audio_pwm_out_samples_per_sec =
          (audio_pwm_out_rate_window_count * 1000U) / elapsed_ms;
      audio_pwm_out_rate_window_count = 0U;
      audio_pwm_out_rate_last_tick = now_tick;
    }
  }
}

uint8_t AudioPwmOutput_HandleSampleTimerIrq(TIM_HandleTypeDef *htim)
{
  if ((htim == NULL) ||
      (audio_pwm_out_sample_handle == NULL) ||
      (htim->Instance != audio_pwm_out_sample_handle->Instance))
  {
    return 0U;
  }

  if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
      (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET))
  {
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    AudioPwmOutput_HandleSampleTimer(htim);
    return 1U;
  }

  return 0U;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (AudioOutput_IsActiveI2S(hi2s) != 0U)
  {
    audio_out_half_count++;
    AudioOutput_FillBlock(&audio_tx_buf[0], audio_out_block_len);
  }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (AudioOutput_IsActiveI2S(hi2s) != 0U)
  {
    audio_out_full_count++;
    AudioOutput_FillBlock(&audio_tx_buf[audio_out_block_len], audio_out_block_len);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  AudioPwmOutput_HandleSampleTimer(htim);
}

static uint16_t AudioOutput_DmaSizeFrames(I2S_HandleTypeDef *hi2s)
{
  audio_out_block_len = AUDIO_OUT_BUF_LEN / 2U;

  if ((hi2s->Init.DataFormat == I2S_DATAFORMAT_24B) ||
      (hi2s->Init.DataFormat == I2S_DATAFORMAT_32B))
  {
    /*
     * For 24/32-bit I2S on STM32F4 HAL, Size is the number of I2S frames.
     * DMA still transfers two uint16_t half-words per frame.
     */
    return AUDIO_OUT_BUF_LEN / 2U;
  }

  return AUDIO_OUT_BUF_LEN;
}

static uint8_t AudioOutput_IsActiveI2S(I2S_HandleTypeDef *hi2s)
{
  if ((hi2s == NULL) || (audio_out_i2s_handle == NULL))
  {
    return 0U;
  }

  return (hi2s->Instance == audio_out_i2s_handle->Instance) ? 1U : 0U;
}

static int16_t AudioOutput_Limit16(int32_t value)
{
  int32_t magnitude = value;
  int32_t limited;

  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }

  if (magnitude <= AUDIO_OUT_LIMIT_THRESHOLD)
  {
    return (int16_t)value;
  }

  audio_out_limiter_count++;
  audio_out_limiter_last_input = value;

  limited = AUDIO_OUT_LIMIT_THRESHOLD +
            ((magnitude - AUDIO_OUT_LIMIT_THRESHOLD) / 8);
  if (limited > AUDIO_OUT_LIMIT_MAX)
  {
    limited = AUDIO_OUT_LIMIT_MAX;
  }

  if (value < 0)
  {
    limited = -limited;
  }

  audio_out_limiter_last_output = (int16_t)limited;
  return (int16_t)limited;
}

static int16_t AudioOutput_ConvertMicSample(int32_t sample)
{
  int32_t ac_sample;
  int32_t scaled_sample;
  uint32_t magnitude;

  /*
   * Small DC blocker. I2S MEMS mics can have an offset; headphones/speakers
   * really do not enjoy DC. The estimate moves slowly, leaving speech/taps.
   */
  audio_out_mic_dc_estimate += (sample - audio_out_mic_dc_estimate) / 256;
  ac_sample = sample - audio_out_mic_dc_estimate;

  /*
   * The mic unpacker gives a signed 24-bit sample in int32_t form. Shift it
   * down to 16-bit audio range first, then apply Q8 gain.
   */
  scaled_sample = (int32_t)(((int64_t)(ac_sample >> 8) *
                             (int64_t)audio_out_mic_gain_q8) >> 8);

  magnitude = (scaled_sample < 0) ? (uint32_t)(-scaled_sample) : (uint32_t)scaled_sample;
  if (magnitude > audio_out_mic_peak)
  {
    audio_out_mic_peak = magnitude;
  }

  return AudioOutput_Limit16(scaled_sample);
}

static int16_t AudioOutput_ReadMicSample(void)
{
  int16_t sample;
  uint32_t available;

  if (audio_out_mic_repeat_remaining > 0U)
  {
    audio_out_mic_repeat_remaining--;
    audio_out_mic_hold_count++;
    return audio_out_mic_last_sample_s16;
  }

  available = AudioOutput_MicAvailable();
  AudioOutput_UpdateAvailableDebug(available);

  /*
   * Let the producer build a small cushion before the DAC starts consuming.
   * Without this, the I2S TX DMA can start first and repeatedly output the
   * previous sample whenever the input side jitters or has not filled yet.
   */
  if (audio_out_mic_streaming == 0U)
  {
    if (available < audio_out_mic_prefill_threshold)
    {
      audio_out_mic_wait_count++;
      return 0;
    }
    audio_out_mic_streaming = 1U;
  }

  if (available == 0U)
  {
    audio_out_mic_underrun_count++;
    audio_out_mic_streaming = 0U;
    AudioOutput_UpdateAvailableDebug(0U);
    return 0;
  }

  sample = audio_out_mic_ring[audio_out_mic_read_index];
  audio_out_mic_last_sample_s16 = sample;
  audio_out_mic_read_index++;
  if (audio_out_mic_read_index >= AUDIO_OUT_MIC_RING_LEN)
  {
    audio_out_mic_read_index = 0U;
  }

  audio_out_mic_pop_count++;
  if (audio_out_mic_repeat_factor > 1U)
  {
    audio_out_mic_repeat_remaining = audio_out_mic_repeat_factor - 1U;
  }

  AudioOutput_UpdateAvailableDebug(AudioOutput_MicAvailable());

  return sample;
}

static int16_t AudioOutput_NextSample16(void)
{
  int16_t sample = 0;

  if ((audio_out_mode == AUDIO_OUTPUT_MODE_TEST_TONE) ||
      (audio_out_force_test_tone != 0U))
  {
    uint32_t divider = audio_out_test_tone_divider;
    if (divider == 0U)
    {
      divider = 1U;
    }
    sample = (int16_t)(audio_out_sine_lut[audio_out_phase & 31U] / (int16_t)divider);
    audio_out_phase++;
  }
  else if (audio_out_mode == AUDIO_OUTPUT_MODE_MIC_MONITOR)
  {
    sample = AudioOutput_ReadMicSample();
  }

  audio_out_last_sample = (uint16_t)sample;
  audio_out_last_sample_s16 = sample;
  return sample;
}

static uint32_t AudioOutput_Abs16(int16_t value)
{
  return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

static uint32_t AudioOutput_MicAvailable(void)
{
  uint32_t write_index = audio_out_mic_write_index;
  uint32_t read_index = audio_out_mic_read_index;

  if (write_index >= read_index)
  {
    return write_index - read_index;
  }

  return AUDIO_OUT_MIC_RING_LEN - read_index + write_index;
}

static void AudioOutput_UpdateAvailableDebug(uint32_t available)
{
  audio_out_mic_available = available;
  if (available < audio_out_mic_available_min)
  {
    audio_out_mic_available_min = available;
  }
  if (available > audio_out_mic_available_max)
  {
    audio_out_mic_available_max = available;
  }
}

static void AudioOutput_UpdateTxDebug(const uint16_t *buf, uint32_t len)
{
  uint64_t abs_sum = 0U;
  uint32_t peak = 0U;
  uint32_t sample_count = 0U;
  uint32_t debug_index = 0U;

  if ((buf == NULL) || (len == 0U))
  {
    return;
  }

  if ((audio_out_i2s_handle != NULL) &&
      ((audio_out_i2s_handle->Init.DataFormat == I2S_DATAFORMAT_24B) ||
       (audio_out_i2s_handle->Init.DataFormat == I2S_DATAFORMAT_32B)))
  {
    for (uint32_t i = 0U; i < len; i += 2U)
    {
      int16_t sample = (int16_t)buf[i];
      uint32_t magnitude = AudioOutput_Abs16(sample);

      if (debug_index < 16U)
      {
        audio_out_tx_debug_samples[debug_index] = sample;
        debug_index++;
      }
      abs_sum += magnitude;
      if (magnitude > peak)
      {
        peak = magnitude;
      }
      sample_count++;
    }
  }
  else
  {
    for (uint32_t i = 0U; i < len; i++)
    {
      int16_t sample = (int16_t)buf[i];
      uint32_t magnitude = AudioOutput_Abs16(sample);

      if (debug_index < 16U)
      {
        audio_out_tx_debug_samples[debug_index] = sample;
        debug_index++;
      }
      abs_sum += magnitude;
      if (magnitude > peak)
      {
        peak = magnitude;
      }
      sample_count++;
    }
  }

  if (sample_count == 0U)
  {
    return;
  }

  audio_out_tx_peak = peak;
  audio_out_tx_abs_avg = (uint32_t)(abs_sum / sample_count);
  audio_out_tx_level_smooth = ((audio_out_tx_level_smooth * 7U) +
                               audio_out_tx_abs_avg) / 8U;
  audio_out_tx_debug_update_count++;
  if (peak == 0U)
  {
    audio_out_tx_zero_block_count++;
  }
  else
  {
    audio_out_tx_nonzero_block_count++;
  }
}

static void AudioOutput_WriteStereo16(uint16_t *buf, uint32_t len)
{
  for (uint32_t i = 0U; i < len; i += 2U)
  {
    int16_t sample = AudioOutput_NextSample16();

    buf[i] = (uint16_t)sample;
    if ((i + 1U) < len)
    {
      buf[i + 1U] = (uint16_t)sample;
    }
  }
}

static void AudioOutput_WriteStereo32Frame(uint16_t *buf, uint32_t len)
{
  for (uint32_t i = 0U; i < len; i += 4U)
  {
    int16_t sample = AudioOutput_NextSample16();
    uint16_t sample_hi = (uint16_t)sample;
    uint16_t sample_lo = 0U;

    buf[i] = sample_hi;
    if ((i + 1U) < len)
    {
      buf[i + 1U] = sample_lo;
    }
    if ((i + 2U) < len)
    {
      buf[i + 2U] = sample_hi;
    }
    if ((i + 3U) < len)
    {
      buf[i + 3U] = sample_lo;
    }
  }
}
