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
#include "audio_capture.h"
#include "audio_fft.h"
#include "audio_output.h"
#include "audio_visualizer.h"
#include "display_st7789.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_ENABLE 1U
#define DISPLAY_BOOT_PATTERN_ENABLE 0U
#define DISPLAY_DIRECT_SWEEP_TEST 0U
#define DISPLAY_DRAW_PERIOD_MS 50U
#define AUDIO_FFT_PROCESS_ENABLE 1U
#define AUDIO_CAPTURE_ENABLE 0U
#define AUX_CAPTURE_ENABLE 1U
#define AUX_CAPTURE_USE_DMA 1U
#define AUDIO_INPUT_DEFAULT_SOURCE AUDIO_INPUT_SOURCE_AUX
#define AUDIO_OUTPUT_ENABLE 1U
#define AUDIO_OUTPUT_USE_PWM 1U
#define AUDIO_OUTPUT_DEFAULT_MODE AUDIO_OUTPUT_MODE_MIC_MONITOR
#define AUDIO_OUTPUT_BOOT_TEST_TONE_MS 0U
#define DMA_IRQ_PRIORITY 5U

#if ((AUDIO_OUTPUT_ENABLE != 0U) && (AUDIO_OUTPUT_USE_PWM == 0U))
/* Change this to the CubeMX-generated I2S TX handle, for example hi2s3. */
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
volatile uint32_t display_startup_draw_status = HAL_ERROR;
volatile uint32_t display_direct_test_count = 0U;
volatile uint32_t display_direct_test_x = 0U;
volatile uint32_t display_direct_test_status = HAL_ERROR;
volatile uint8_t aux_capture_start_failed = 0U;
volatile uint8_t audio_output_start_failed = 0U;
volatile uint32_t diag_aux_blocks_per_sec = 0U;
volatile uint32_t diag_aux_samples_per_sec = 0U;
volatile uint32_t diag_aux_realtime_samples_per_sec = 0U;
volatile uint32_t diag_i2s_blocks_per_sec = 0U;
volatile uint32_t diag_output_blocks_per_sec = 0U;
volatile uint32_t diag_output_debug_updates_per_sec = 0U;
volatile uint32_t diag_fft_process_per_sec = 0U;
volatile uint32_t diag_display_draws_per_sec = 0U;
volatile uint8_t audio_output_boot_test_active = 0U;
volatile uint32_t audio_output_boot_test_start_tick = 0U;
static uint32_t display_last_draw_tick = 0U;
static uint32_t diag_last_tick = 0U;
static uint32_t diag_last_aux_block_count = 0U;
static uint32_t diag_last_aux_realtime_push_count = 0U;
static uint32_t diag_last_i2s_block_count = 0U;
static uint32_t diag_last_output_block_count = 0U;
static uint32_t diag_last_output_debug_update_count = 0U;
static uint32_t diag_last_fft_process_count = 0U;
static uint32_t diag_last_display_draw_count = 0U;

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
   * Start capture before reconstruction. With DMA capture, service the queued
   * AUX blocks during this short prefill window so the output ring has
   * live samples before the speaker output starts consuming it.
   */
  uint32_t output_prefill_start_tick = HAL_GetTick();
  while ((HAL_GetTick() - output_prefill_start_tick) < 30U)
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if ((AUX_CAPTURE_ENABLE != 0U) && (AUX_CAPTURE_USE_DMA == 0U) && (DISPLAY_DIRECT_SWEEP_TEST == 0U))
    AuxCapture_Poll();
#elif ((AUX_CAPTURE_ENABLE != 0U) && (AUX_CAPTURE_USE_DMA != 0U) && (DISPLAY_DIRECT_SWEEP_TEST == 0U))
    AuxCapture_Service();
#endif

#if ((DISPLAY_DIRECT_SWEEP_TEST == 0U) && (AUDIO_FFT_PROCESS_ENABLE != 0U))
    (void)AudioFFT_ProcessIfReady();
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
    if ((display_ready == 1U) &&
        (display_waveform_enable != 0U) &&
        ((HAL_GetTick() - display_last_draw_tick) >= DISPLAY_DRAW_PERIOD_MS))
    {
      display_last_draw_tick = HAL_GetTick();
      if (AudioVisualizer_DrawWaveform(&hdisplay) != HAL_OK)
      {
        display_draw_error_count++;
      }
      else
      {
        display_waveform_draw_count++;
      }
    }
    else if ((display_ready == 1U) &&
             (display_waveform_enable != 0U) &&
             (AudioVisualizer_IsFrameReady() == 0U))
    {
      display_waveform_wait_count++;
    }
#endif

    if ((HAL_GetTick() - diag_last_tick) >= 1000U)
    {
      uint32_t aux_blocks = aux_adc_block_count;
      uint32_t aux_realtime_samples = aux_output_realtime_push_count;
      uint32_t i2s_blocks = audio_half_count + audio_full_count;
      uint32_t output_blocks = audio_out_half_count + audio_out_full_count;
      uint32_t output_debug_updates = audio_out_tx_debug_update_count;
      uint32_t fft_process_count = audio_fft_process_count;
      uint32_t display_draw_count = display_waveform_draw_count;

      AuxCapture_UpdateDiagnostics();
      diag_last_tick = HAL_GetTick();

      diag_aux_blocks_per_sec = aux_blocks - diag_last_aux_block_count;
      diag_aux_samples_per_sec = diag_aux_blocks_per_sec * AUX_CAPTURE_BUF_LEN;
      diag_aux_realtime_samples_per_sec =
          aux_realtime_samples - diag_last_aux_realtime_push_count;
      diag_i2s_blocks_per_sec = i2s_blocks - diag_last_i2s_block_count;
      diag_output_blocks_per_sec = output_blocks - diag_last_output_block_count;
      diag_output_debug_updates_per_sec =
          output_debug_updates - diag_last_output_debug_update_count;
      diag_fft_process_per_sec = fft_process_count - diag_last_fft_process_count;
      diag_display_draws_per_sec = display_draw_count - diag_last_display_draw_count;

      diag_last_aux_block_count = aux_blocks;
      diag_last_aux_realtime_push_count = aux_realtime_samples;
      diag_last_i2s_block_count = i2s_blocks;
      diag_last_output_block_count = output_blocks;
      diag_last_output_debug_update_count = output_debug_updates;
      diag_last_fft_process_count = fft_process_count;
      diag_last_display_draw_count = display_draw_count;
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
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
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
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
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
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
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
  htim2.Init.Period = 5249;
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
  htim3.Init.Period = 5249;
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
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
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
