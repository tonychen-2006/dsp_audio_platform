#include "audio_samples.h"

int32_t audio_sample_buf[AUDIO_SAMPLE_BUF_LEN];
volatile uint32_t audio_sample_count = 0U;
volatile int32_t audio_sample_avg = 0;
volatile int32_t audio_sample_min = 0;
volatile int32_t audio_sample_max = 0;
volatile uint32_t audio_sample_abs_avg = 0U;
volatile uint32_t audio_sample_peak = 0U;
volatile uint32_t audio_sample_level_smooth = 0U;
volatile uint32_t audio_sample_noise_floor = 0U;
volatile uint32_t audio_sample_signal_level = 0U;
volatile uint32_t audio_sample_signal_smooth = 0U;
volatile uint32_t audio_sample_left_abs_avg = 0U;
volatile uint32_t audio_sample_right_abs_avg = 0U;
volatile uint8_t audio_sample_active_slot = 0U;
volatile uint32_t audio_sample_process_count = 0U;
volatile uint32_t audio_sample_debug_raw_left = 0U;
volatile uint32_t audio_sample_debug_raw_right = 0U;

static int32_t AudioSamples_Raw32ToSigned24(uint32_t raw);
static uint32_t AudioSamples_Abs32(int32_t value);
static void AudioSamples_UpdateNoiseMetrics(uint32_t abs_avg);

uint32_t AudioSamples_UnpackI2s32(const uint16_t *raw_buf, uint32_t raw_len)
{
  uint32_t out_count = 0U;
  int64_t sum = 0;
  uint64_t abs_sum = 0U;
  int32_t min_sample = INT32_MAX;
  int32_t max_sample = INT32_MIN;
  uint32_t peak = 0U;
  uint64_t left_abs_sum = 0U;
  uint64_t right_abs_sum = 0U;
  uint32_t frame_count = 0U;
  uint8_t active_slot;

  if ((raw_buf == NULL) || (raw_len < 4U))
  {
    audio_sample_count = 0U;
    return 0U;
  }

  for (uint32_t i = 0U; (i + 3U) < raw_len; i += 4U)
  {
    uint32_t raw_left = ((uint32_t)raw_buf[i] << 16U) | raw_buf[i + 1U];
    uint32_t raw_right = ((uint32_t)raw_buf[i + 2U] << 16U) | raw_buf[i + 3U];

    left_abs_sum += AudioSamples_Abs32(AudioSamples_Raw32ToSigned24(raw_left));
    right_abs_sum += AudioSamples_Abs32(AudioSamples_Raw32ToSigned24(raw_right));
    frame_count++;
  }

  if (frame_count == 0U)
  {
    audio_sample_count = 0U;
    return 0U;
  }

  audio_sample_left_abs_avg = (uint32_t)(left_abs_sum / frame_count);
  audio_sample_right_abs_avg = (uint32_t)(right_abs_sum / frame_count);
  active_slot = (audio_sample_right_abs_avg > audio_sample_left_abs_avg) ? 1U : 0U;
  audio_sample_active_slot = active_slot;

  for (uint32_t i = 0U; (i + 3U) < raw_len; i += 4U)
  {
    uint32_t raw_left = ((uint32_t)raw_buf[i] << 16U) | raw_buf[i + 1U];
    uint32_t raw_right = ((uint32_t)raw_buf[i + 2U] << 16U) | raw_buf[i + 3U];
    int32_t left_sample = AudioSamples_Raw32ToSigned24(raw_left);
    int32_t right_sample = AudioSamples_Raw32ToSigned24(raw_right);
    int32_t sample;
    uint32_t magnitude;

    if (out_count == 0U)
    {
      audio_sample_debug_raw_left = raw_left;
      audio_sample_debug_raw_right = raw_right;
    }

    if (active_slot != 0U)
    {
      sample = right_sample;
    }
    else
    {
      sample = left_sample;
    }

    audio_sample_buf[out_count] = sample;
    out_count++;

    sum += sample;
    magnitude = AudioSamples_Abs32(sample);
    abs_sum += magnitude;

    if (sample < min_sample)
    {
      min_sample = sample;
    }
    if (sample > max_sample)
    {
      max_sample = sample;
    }
    if (magnitude > peak)
    {
      peak = magnitude;
    }

    if (out_count >= AUDIO_SAMPLE_BUF_LEN)
    {
      break;
    }
  }

  if (out_count == 0U)
  {
    audio_sample_count = 0U;
    return 0U;
  }

  audio_sample_count = out_count;
  audio_sample_avg = (int32_t)(sum / (int64_t)out_count);
  audio_sample_min = min_sample;
  audio_sample_max = max_sample;
  audio_sample_abs_avg = (uint32_t)(abs_sum / out_count);
  audio_sample_peak = peak;
  audio_sample_level_smooth = ((audio_sample_level_smooth * 15U) + audio_sample_abs_avg) / 16U;
  AudioSamples_UpdateNoiseMetrics(audio_sample_abs_avg);
  audio_sample_process_count++;

  return out_count;
}

void AudioSamples_UpdateFromS32(const int32_t *samples, uint32_t len)
{
  int64_t sum = 0;
  uint64_t abs_sum = 0U;
  int32_t min_sample = INT32_MAX;
  int32_t max_sample = INT32_MIN;
  uint32_t peak = 0U;
  uint32_t copy_len;

  if ((samples == NULL) || (len == 0U))
  {
    audio_sample_count = 0U;
    return;
  }

  copy_len = (len < AUDIO_SAMPLE_BUF_LEN) ? len : AUDIO_SAMPLE_BUF_LEN;

  for (uint32_t i = 0U; i < copy_len; i++)
  {
    int32_t sample = samples[i];
    uint32_t magnitude = AudioSamples_Abs32(sample);

    audio_sample_buf[i] = sample;
    sum += sample;
    abs_sum += magnitude;

    if (sample < min_sample)
    {
      min_sample = sample;
    }
    if (sample > max_sample)
    {
      max_sample = sample;
    }
    if (magnitude > peak)
    {
      peak = magnitude;
    }
  }

  audio_sample_count = copy_len;
  audio_sample_avg = (int32_t)(sum / (int64_t)copy_len);
  audio_sample_min = min_sample;
  audio_sample_max = max_sample;
  audio_sample_abs_avg = (uint32_t)(abs_sum / copy_len);
  audio_sample_peak = peak;
  audio_sample_level_smooth = ((audio_sample_level_smooth * 15U) +
                               audio_sample_abs_avg) / 16U;
  audio_sample_left_abs_avg = audio_sample_abs_avg;
  audio_sample_right_abs_avg = 0U;
  audio_sample_active_slot = 0U;
  audio_sample_debug_raw_left = (uint32_t)audio_sample_buf[0];
  audio_sample_debug_raw_right = 0U;
  AudioSamples_UpdateNoiseMetrics(audio_sample_abs_avg);
  audio_sample_process_count++;
}

static int32_t AudioSamples_Raw32ToSigned24(uint32_t raw)
{
  /*
   * Most I2S MEMS mics left-align a signed 24-bit sample in a 32-bit slot.
   * Casting to int32_t preserves the sign bit, then shifting right by 8 gives
   * a sign-extended 24-bit sample in a normal int32_t.
   */
  return ((int32_t)raw) >> 8;
}

static uint32_t AudioSamples_Abs32(int32_t value)
{
  return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

static void AudioSamples_UpdateNoiseMetrics(uint32_t abs_avg)
{
  uint32_t floor = audio_sample_noise_floor;
  uint32_t signal;

  /*
   * audio_sample_peak is intentionally spiky. This estimates the background
   * level from the block average and then reports the signal above that floor.
   *
   * If the room gets quieter, the floor moves down fairly quickly.
   * If the room gets louder, the floor moves up slowly so speech/taps still
   * show up as signal instead of being swallowed immediately.
   */
  if ((audio_sample_process_count == 0U) || (floor == 0U))
  {
    floor = abs_avg;
  }
  else if (abs_avg < floor)
  {
    floor = ((floor * 7U) + abs_avg) / 8U;
  }
  else
  {
    floor = ((floor * 255U) + abs_avg) / 256U;
  }

  audio_sample_noise_floor = floor;

  if (abs_avg > floor)
  {
    signal = abs_avg - floor;
  }
  else
  {
    signal = 0U;
  }

  audio_sample_signal_level = signal;
  audio_sample_signal_smooth = ((audio_sample_signal_smooth * 7U) + signal) / 8U;
}
