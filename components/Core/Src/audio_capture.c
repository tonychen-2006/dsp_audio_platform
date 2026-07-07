#include "audio_capture.h"
#include "audio_fft.h"
#include "audio_output.h"
#include "audio_samples.h"
#include "audio_visualizer.h"

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

int32_t aux_sample_buf[AUX_CAPTURE_BUF_LEN];
volatile uint32_t aux_adc_raw = 0U;
volatile uint32_t aux_adc_avg = 0U;
volatile uint32_t aux_adc_min = 0U;
volatile uint32_t aux_adc_max = 0U;
volatile uint32_t aux_adc_peak = 0U;
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
volatile uint32_t aux_output_realtime_enable = 1U;
volatile uint32_t aux_output_realtime_push_count = 0U;
volatile uint32_t aux_output_realtime_peak = 0U;
volatile uint32_t aux_output_realtime_avg = 0U;
volatile uint32_t aux_output_realtime_min = 0U;
volatile uint32_t aux_output_realtime_max = 0U;
volatile uint32_t aux_output_realtime_abs_avg = 0U;
volatile uint32_t aux_output_realtime_bias_mv = 0U;
volatile uint16_t aux_adc_debug_samples[16];
volatile uint8_t aux_ready = 0U;
uint16_t aux_adc_dma_buf[AUX_ADC_DMA_BUF_LEN];

static I2S_HandleTypeDef *audio_i2s_handle = NULL;
static ADC_HandleTypeDef *aux_adc_handle = NULL;
static TIM_HandleTypeDef *aux_timer_handle = NULL;
static uint32_t audio_block_len = I2S_BUF_LEN / 2U;
static uint32_t aux_sample_index = 0U;
static int32_t aux_adc_dc_estimate = 2048;
static int32_t aux_adc_output_dc_estimate = 2048;
static uint16_t aux_adc_raw_buf[AUX_CAPTURE_BUF_LEN];
static int32_t aux_output_sample_buf[AUX_CAPTURE_BUF_LEN];
static uint16_t aux_adc_pending_half_buf[AUX_CAPTURE_BUF_LEN];
static uint16_t aux_adc_pending_full_buf[AUX_CAPTURE_BUF_LEN];
static uint16_t aux_adc_service_buf[AUX_CAPTURE_BUF_LEN];
static volatile uint8_t aux_adc_half_pending = 0U;
static volatile uint8_t aux_adc_full_pending = 0U;

static uint16_t AudioCapture_DmaSizeFrames(I2S_HandleTypeDef *hi2s);
static uint8_t AudioCapture_IsActiveI2S(I2S_HandleTypeDef *hi2s);
static uint8_t AuxCapture_IsActiveAdc(ADC_HandleTypeDef *hadc);
static void AuxCapture_ResetStats(void);
static void AuxCapture_QueueDmaBlock(const uint16_t *raw_buf, uint8_t half_block);
static void AuxCapture_PushOutputRawBlock(const uint16_t *raw_buf, uint32_t len);
static void AuxCapture_ProcessRawBlock(const uint16_t *raw_buf, uint32_t len);

HAL_StatusTypeDef AudioCapture_Start(I2S_HandleTypeDef *hi2s)
{
  HAL_StatusTypeDef status;

  if (hi2s == NULL)
  {
    audio_start_status = HAL_ERROR;
    return HAL_ERROR;
  }

  audio_i2s_handle = hi2s;
  audio_ready = 0U;

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

  sample_count = AudioSamples_UnpackI2s32(buf, len);
  if ((sample_count > 0U) && (audio_input_source == AUDIO_INPUT_SOURCE_I2S))
  {
    AudioOutput_PushSamplesS32(audio_sample_buf, sample_count);
    AudioFFT_PushSamplesS32(audio_sample_buf, sample_count);
    AudioVisualizer_UpdateFromS32(audio_sample_buf, sample_count);
  }
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
  int32_t centered;

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

  /*
   * Track the actual bias point. With the AUX conditioning circuit connected
   * and no audio playing, this should settle near ADC code 2048 (~1.65 V).
   */
  aux_adc_dc_estimate += ((int32_t)raw - aux_adc_dc_estimate) / 128;
  centered = (int32_t)raw - aux_adc_dc_estimate;

  if (aux_sample_index < 16U)
  {
    aux_adc_debug_samples[aux_sample_index] = (uint16_t)raw;
  }
  aux_adc_raw_buf[aux_sample_index] = (uint16_t)raw;

  /*
   * Convert 12-bit ADC audio around mid-supply into the same rough signed
   * 24-bit scale used by the I2S mic path.
   */
  aux_sample_buf[aux_sample_index] = centered << 12;
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

  __disable_irq();
  if (aux_adc_half_pending != 0U)
  {
    aux_adc_half_pending = 0U;
    aux_adc_pending_count = (uint32_t)aux_adc_full_pending;
    process_half = 1U;
  }
  __enable_irq();

  if (process_half != 0U)
  {
    for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
    {
      aux_adc_service_buf[i] = aux_adc_pending_half_buf[i];
    }
    AuxCapture_ProcessRawBlock(aux_adc_service_buf, AUX_CAPTURE_BUF_LEN);
    aux_adc_service_count++;
  }

  __disable_irq();
  if (aux_adc_full_pending != 0U)
  {
    aux_adc_full_pending = 0U;
    aux_adc_pending_count = (uint32_t)aux_adc_half_pending;
    process_full = 1U;
  }
  __enable_irq();

  if (process_full != 0U)
  {
    for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
    {
      aux_adc_service_buf[i] = aux_adc_pending_full_buf[i];
    }
    AuxCapture_ProcessRawBlock(aux_adc_service_buf, AUX_CAPTURE_BUF_LEN);
    aux_adc_service_count++;
  }
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
    aux_adc_half_count++;
    AuxCapture_QueueDmaBlock(&aux_adc_dma_buf[0], 1U);
    AuxCapture_PushOutputRawBlock(&aux_adc_dma_buf[0], AUX_CAPTURE_BUF_LEN);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (AuxCapture_IsActiveAdc(hadc) != 0U)
  {
    aux_adc_full_count++;
    AuxCapture_QueueDmaBlock(&aux_adc_dma_buf[AUX_CAPTURE_BUF_LEN], 0U);
    AuxCapture_PushOutputRawBlock(&aux_adc_dma_buf[AUX_CAPTURE_BUF_LEN], AUX_CAPTURE_BUF_LEN);
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
  aux_sample_index = 0U;
  aux_adc_dc_estimate = 2048;
  aux_adc_output_dc_estimate = 2048;
  aux_ready = 0U;
  aux_adc_raw = 0U;
  aux_adc_avg = 0U;
  aux_adc_min = 0U;
  aux_adc_max = 0U;
  aux_adc_peak = 0U;
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
  aux_output_realtime_push_count = 0U;
  aux_output_realtime_peak = 0U;
  aux_output_realtime_avg = 0U;
  aux_output_realtime_min = 0U;
  aux_output_realtime_max = 0U;
  aux_output_realtime_abs_avg = 0U;
  aux_output_realtime_bias_mv = 0U;
  aux_adc_half_pending = 0U;
  aux_adc_full_pending = 0U;

  for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
  {
    aux_sample_buf[i] = 0;
    aux_output_sample_buf[i] = 0;
    aux_adc_raw_buf[i] = 2048U;
    aux_adc_pending_half_buf[i] = 2048U;
    aux_adc_pending_full_buf[i] = 2048U;
    aux_adc_service_buf[i] = 2048U;
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

  if (raw_buf == NULL)
  {
    return;
  }

  if (half_block != 0U)
  {
    target_buf = aux_adc_pending_half_buf;
    pending_flag = &aux_adc_half_pending;
  }
  else
  {
    target_buf = aux_adc_pending_full_buf;
    pending_flag = &aux_adc_full_pending;
  }

  if (*pending_flag != 0U)
  {
    aux_adc_overrun_count++;
  }

  for (uint32_t i = 0U; i < AUX_CAPTURE_BUF_LEN; i++)
  {
    target_buf[i] = raw_buf[i];
  }

  *pending_flag = 1U;
  aux_adc_pending_count = (uint32_t)aux_adc_half_pending +
                          (uint32_t)aux_adc_full_pending;
}

static void AuxCapture_PushOutputRawBlock(const uint16_t *raw_buf, uint32_t len)
{
  uint32_t peak = 0U;
  uint32_t min_value = UINT32_MAX;
  uint32_t max_value = 0U;
  uint32_t sum = 0U;
  uint32_t abs_sum = 0U;

  if ((raw_buf == NULL) || (len == 0U) ||
      (audio_input_source != AUDIO_INPUT_SOURCE_AUX) ||
      (aux_output_realtime_enable == 0U))
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
    int32_t centered;
    uint32_t magnitude;

    if (raw > 4095U)
    {
      raw = 4095U;
    }

    aux_adc_output_dc_estimate += ((int32_t)raw - aux_adc_output_dc_estimate) / 128;
    centered = (int32_t)raw - aux_adc_output_dc_estimate;
    aux_output_sample_buf[i] = centered << 12;

    if (raw < min_value)
    {
      min_value = raw;
    }
    if (raw > max_value)
    {
      max_value = raw;
    }
    sum += raw;

    magnitude = (centered < 0) ? (uint32_t)(-centered) : (uint32_t)centered;
    abs_sum += magnitude;
    if (magnitude > peak)
    {
      peak = magnitude;
    }
  }

  AudioOutput_PushSamplesS32(aux_output_sample_buf, len);
  aux_output_realtime_push_count += len;
  aux_output_realtime_peak = peak;
  aux_output_realtime_avg = sum / len;
  aux_output_realtime_min = min_value;
  aux_output_realtime_max = max_value;
  aux_output_realtime_abs_avg = abs_sum / len;
  aux_output_realtime_bias_mv = (aux_output_realtime_avg * aux_adc_vref_mv) / 4095U;
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
     * Slow DC tracker removes the 1.65 V bias created by the AUX hardware.
     */
    aux_adc_dc_estimate += ((int32_t)raw - aux_adc_dc_estimate) / 128;
    centered = (int32_t)raw - aux_adc_dc_estimate;
    aux_sample_buf[i] = centered << 12;
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
    AudioSamples_UpdateFromS32(aux_sample_buf, len);
    if ((aux_output_realtime_enable == 0U) || (aux_timer_handle == NULL))
    {
      AudioOutput_PushSamplesS32(aux_sample_buf, len);
    }
    AudioFFT_PushSamplesS32(aux_sample_buf, len);
    AudioVisualizer_UpdateFromS32(aux_sample_buf, len);
  }
}
