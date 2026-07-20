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
volatile uint8_t audio_sample_noise_floor_frozen = 0U;
volatile uint32_t audio_sample_noise_floor_freeze_count = 0U;
volatile uint32_t audio_sample_signal_level = 0U;
volatile uint32_t audio_sample_signal_smooth = 0U;
volatile uint32_t audio_sample_left_abs_avg = 0U;
volatile uint32_t audio_sample_right_abs_avg = 0U;
volatile uint8_t audio_sample_active_slot = 0U;
volatile uint8_t audio_sample_slot_mode = AUDIO_I2S_SLOT_AUTO;
volatile uint8_t audio_sample_slot_locked = 0U;
volatile uint32_t audio_sample_slot_detect_blocks = 0U;
volatile uint32_t audio_sample_slot_lock_count = 0U;
volatile uint32_t audio_sample_process_count = 0U;
volatile uint32_t audio_sample_debug_raw_left = 0U;
volatile uint32_t audio_sample_debug_raw_right = 0U;

static int32_t AudioSamples_Raw32ToSigned24(uint32_t raw);
static uint32_t AudioSamples_Abs32(int32_t value);
static uint8_t AudioSamples_SelectedGateOpen(void);
static void AudioSamples_UpdateNoiseMetrics(uint32_t abs_avg);
static uint8_t AudioSamples_SelectI2sSlot(uint32_t left_abs_avg,
                                         uint32_t right_abs_avg);

#define AUDIO_I2S_SLOT_AUTO_EARLY_BLOCKS 2U
#define AUDIO_I2S_SLOT_AUTO_LOCK_BLOCKS 8U
#define AUDIO_I2S_SLOT_AUTO_RATIO       4U
#define AUDIO_I2S_SLOT_AUTO_MIN_LEVEL   64U

static uint8_t audio_sample_slot_mode_cached = UINT8_MAX;
static uint8_t audio_sample_slot_auto_value = AUDIO_I2S_SLOT_LEFT;
static uint64_t audio_sample_slot_left_accum = 0U;
static uint64_t audio_sample_slot_right_accum = 0U;

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
  active_slot = AudioSamples_SelectI2sSlot(audio_sample_left_abs_avg,
                                           audio_sample_right_abs_avg);
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

void AudioSamples_ResetI2sSlotSelection(void)
{
  audio_sample_slot_mode_cached = UINT8_MAX;
  audio_sample_slot_auto_value = AUDIO_I2S_SLOT_LEFT;
  audio_sample_slot_left_accum = 0U;
  audio_sample_slot_right_accum = 0U;
  audio_sample_active_slot = AUDIO_I2S_SLOT_LEFT;
  audio_sample_slot_locked = 0U;
  audio_sample_slot_detect_blocks = 0U;
  audio_sample_slot_lock_count = 0U;
  AudioSamples_ResetNoiseMetrics();
}

void AudioSamples_ResetNoiseMetrics(void)
{
  audio_sample_noise_floor = 0U;
  audio_sample_noise_floor_frozen = 0U;
  audio_sample_noise_floor_freeze_count = 0U;
  audio_sample_signal_level = 0U;
  audio_sample_signal_smooth = 0U;
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

static uint8_t AudioSamples_SelectedGateOpen(void)
{
  if (audio_input_source == AUDIO_INPUT_SOURCE_AUX)
  {
    return (aux_output_gate_open != 0U) ? 1U : 0U;
  }

  return (i2s_mic_gate_open != 0U) ? 1U : 0U;
}

static uint8_t AudioSamples_SelectI2sSlot(uint32_t left_abs_avg,
                                         uint32_t right_abs_avg)
{
  uint8_t mode = audio_sample_slot_mode;

  if (mode > AUDIO_I2S_SLOT_AUTO)
  {
    mode = AUDIO_I2S_SLOT_AUTO;
    audio_sample_slot_mode = mode;
  }

  if (mode != audio_sample_slot_mode_cached)
  {
    audio_sample_slot_mode_cached = mode;
    audio_sample_slot_locked = 0U;
    audio_sample_slot_detect_blocks = 0U;
    audio_sample_slot_left_accum = 0U;
    audio_sample_slot_right_accum = 0U;
  }

  if (mode != AUDIO_I2S_SLOT_AUTO)
  {
    audio_sample_slot_locked = 1U;
    audio_sample_slot_auto_value = mode;
    return mode;
  }

  if (audio_sample_slot_locked == 0U)
  {
    audio_sample_slot_left_accum += left_abs_avg;
    audio_sample_slot_right_accum += right_abs_avg;
    audio_sample_slot_detect_blocks++;

    if ((audio_sample_slot_detect_blocks >=
         AUDIO_I2S_SLOT_AUTO_EARLY_BLOCKS) &&
        (left_abs_avg >= AUDIO_I2S_SLOT_AUTO_MIN_LEVEL) &&
        ((uint64_t)left_abs_avg >=
         ((uint64_t)right_abs_avg * AUDIO_I2S_SLOT_AUTO_RATIO)))
    {
      audio_sample_slot_auto_value = AUDIO_I2S_SLOT_LEFT;
      audio_sample_slot_locked = 1U;
    }
    else if ((audio_sample_slot_detect_blocks >=
              AUDIO_I2S_SLOT_AUTO_EARLY_BLOCKS) &&
             (right_abs_avg >= AUDIO_I2S_SLOT_AUTO_MIN_LEVEL) &&
             ((uint64_t)right_abs_avg >=
              ((uint64_t)left_abs_avg * AUDIO_I2S_SLOT_AUTO_RATIO)))
    {
      audio_sample_slot_auto_value = AUDIO_I2S_SLOT_RIGHT;
      audio_sample_slot_locked = 1U;
    }
    else if (audio_sample_slot_detect_blocks >=
             AUDIO_I2S_SLOT_AUTO_LOCK_BLOCKS)
    {
      audio_sample_slot_auto_value =
          (audio_sample_slot_right_accum > audio_sample_slot_left_accum) ?
          AUDIO_I2S_SLOT_RIGHT : AUDIO_I2S_SLOT_LEFT;
      audio_sample_slot_locked = 1U;
    }

    if (audio_sample_slot_locked != 0U)
    {
      audio_sample_slot_lock_count++;
    }
  }

  if (audio_sample_slot_locked != 0U)
  {
    return audio_sample_slot_auto_value;
  }

  /* Detection lasts only a few startup blocks; use the louder slot meanwhile. */
  return (right_abs_avg > left_abs_avg) ?
         AUDIO_I2S_SLOT_RIGHT : AUDIO_I2S_SLOT_LEFT;
}

static void AudioSamples_UpdateNoiseMetrics(uint32_t abs_avg)
{
  uint32_t floor = audio_sample_noise_floor;
  uint32_t signal;
  uint8_t freeze_upward = AudioSamples_SelectedGateOpen();

  /*
   * audio_sample_peak is intentionally spiky. This estimates the background
   * level from the block average and then reports the signal above that floor.
   *
   * If the room gets quieter, the floor moves down fairly quickly. Upward
   * movement is allowed only while the selected input gate is closed; this
   * prevents sustained audio from being learned as background noise.
   */
  audio_sample_noise_floor_frozen = freeze_upward;
  if (freeze_upward != 0U)
  {
    audio_sample_noise_floor_freeze_count++;
  }

  if (abs_avg < floor)
  {
    floor = ((floor * 7U) + abs_avg) / 8U;
  }
  else if (freeze_upward == 0U)
  {
    if ((audio_sample_process_count == 0U) || (floor == 0U))
    {
      floor = abs_avg;
    }
    else
    {
      floor = ((floor * 255U) + abs_avg) / 256U;
    }
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
