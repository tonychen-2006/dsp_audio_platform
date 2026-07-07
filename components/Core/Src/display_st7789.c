#include "display_st7789.h"

#define ST7789_NOP      0x00U
#define ST7789_SWRESET  0x01U
#define ST7789_SLPOUT   0x11U
#define ST7789_NORON    0x13U
#define ST7789_INVON    0x21U
#define ST7789_DISPON   0x29U
#define ST7789_CASET    0x2AU
#define ST7789_RASET    0x2BU
#define ST7789_RAMWR    0x2CU
#define ST7789_MADCTL   0x36U
#define ST7789_COLMOD   0x3AU
#define ST7789_SPI_TIMEOUT_MS 100U

volatile uint32_t st7789_dma_tx_count = 0U;
volatile uint32_t st7789_blocking_tx_count = 0U;
volatile uint32_t st7789_dma_error_count = 0U;

#ifdef HAL_SPI_MODULE_ENABLED

static HAL_StatusTypeDef ST7789_WriteCommand(ST7789_HandleTypeDef *display, uint8_t cmd);
static HAL_StatusTypeDef ST7789_WriteData(ST7789_HandleTypeDef *display,
                                          const uint8_t *data,
                                          uint16_t len);
static HAL_StatusTypeDef ST7789_SetAddressWindow(ST7789_HandleTypeDef *display,
                                                 uint16_t x0,
                                                 uint16_t y0,
                                                 uint16_t x1,
                                                 uint16_t y1);
static void ST7789_Select(ST7789_HandleTypeDef *display);
static void ST7789_Unselect(ST7789_HandleTypeDef *display);
static void ST7789_Reset(ST7789_HandleTypeDef *display);

HAL_StatusTypeDef ST7789_Init(ST7789_HandleTypeDef *display)
{
  uint8_t data;

  if ((display == NULL) || (display->hspi == NULL) ||
      (display->dc_port == NULL) || (display->dc_pin == 0U))
  {
    return HAL_ERROR;
  }

  ST7789_Unselect(display);
  HAL_GPIO_WritePin(display->dc_port, display->dc_pin, GPIO_PIN_SET);
  ST7789_Reset(display);

  if (ST7789_WriteCommand(display, ST7789_SWRESET) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(150U);

  if (ST7789_WriteCommand(display, ST7789_SLPOUT) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(120U);

  data = 0x55U; /* 16-bit RGB565 */
  if ((ST7789_WriteCommand(display, ST7789_COLMOD) != HAL_OK) ||
      (ST7789_WriteData(display, &data, 1U) != HAL_OK))
  {
    return HAL_ERROR;
  }

  data = 0x00U; /* Normal orientation, RGB order */
  if ((ST7789_WriteCommand(display, ST7789_MADCTL) != HAL_OK) ||
      (ST7789_WriteData(display, &data, 1U) != HAL_OK))
  {
    return HAL_ERROR;
  }

  if ((ST7789_WriteCommand(display, ST7789_INVON) != HAL_OK) ||
      (ST7789_WriteCommand(display, ST7789_NORON) != HAL_OK))
  {
    return HAL_ERROR;
  }

  if (ST7789_WriteCommand(display, ST7789_DISPON) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(120U);

  if (display->bl_port != NULL)
  {
    HAL_GPIO_WritePin(display->bl_port, display->bl_pin, GPIO_PIN_SET);
  }

  return HAL_OK;
}

HAL_StatusTypeDef ST7789_FillScreen(ST7789_HandleTypeDef *display, uint16_t color)
{
  return ST7789_FillRect(display, 0U, 0U, ST7789_WIDTH, ST7789_HEIGHT, color);
}

HAL_StatusTypeDef ST7789_FillRect(ST7789_HandleTypeDef *display,
                                  uint16_t x,
                                  uint16_t y,
                                  uint16_t w,
                                  uint16_t h,
                                  uint16_t color)
{
  uint8_t color_data[64U * 2U];
  uint32_t pixels;
  uint32_t chunk_pixels;

  if ((display == NULL) || (x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT) ||
      (w == 0U) || (h == 0U))
  {
    return HAL_ERROR;
  }

  if ((x + w) > ST7789_WIDTH)
  {
    w = ST7789_WIDTH - x;
  }
  if ((y + h) > ST7789_HEIGHT)
  {
    h = ST7789_HEIGHT - y;
  }

  for (uint32_t i = 0U; i < 64U; i++)
  {
    color_data[(i * 2U)] = (uint8_t)(color >> 8U);
    color_data[(i * 2U) + 1U] = (uint8_t)(color & 0xFFU);
  }

  if (ST7789_SetAddressWindow(display, x, y, x + w - 1U, y + h - 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  pixels = (uint32_t)w * (uint32_t)h;
  while (pixels > 0U)
  {
    chunk_pixels = (pixels > 64U) ? 64U : pixels;
    if (ST7789_WriteData(display, color_data, (uint16_t)(chunk_pixels * 2U)) != HAL_OK)
    {
      return HAL_ERROR;
    }
    pixels -= chunk_pixels;
  }

  return HAL_OK;
}

HAL_StatusTypeDef ST7789_DrawPixel(ST7789_HandleTypeDef *display,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t color)
{
  return ST7789_FillRect(display, x, y, 1U, 1U, color);
}

HAL_StatusTypeDef ST7789_DrawHLine(ST7789_HandleTypeDef *display,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t w,
                                   uint16_t color)
{
  return ST7789_FillRect(display, x, y, w, 1U, color);
}

HAL_StatusTypeDef ST7789_DrawVLine(ST7789_HandleTypeDef *display,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t h,
                                   uint16_t color)
{
  return ST7789_FillRect(display, x, y, 1U, h, color);
}

HAL_StatusTypeDef ST7789_DrawRGB565Bitmap(ST7789_HandleTypeDef *display,
                                           uint16_t x,
                                           uint16_t y,
                                           uint16_t w,
                                           uint16_t h,
                                           const uint8_t *data,
                                           uint32_t len)
{
  uint32_t expected_len = (uint32_t)w * (uint32_t)h * 2U;
  uint32_t offset = 0U;

  if ((display == NULL) || (data == NULL) ||
      (x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT) ||
      (w == 0U) || (h == 0U) || (len < expected_len))
  {
    return HAL_ERROR;
  }

  if (((uint32_t)x + (uint32_t)w) > ST7789_WIDTH)
  {
    return HAL_ERROR;
  }
  if (((uint32_t)y + (uint32_t)h) > ST7789_HEIGHT)
  {
    return HAL_ERROR;
  }

  if (ST7789_SetAddressWindow(display, x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  while (offset < expected_len)
  {
    uint32_t remaining = expected_len - offset;
    uint16_t chunk = (remaining > 4096U) ? 4096U : (uint16_t)remaining;

    if (ST7789_WriteData(display, &data[offset], chunk) != HAL_OK)
    {
      return HAL_ERROR;
    }
    offset += chunk;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef ST7789_WriteCommand(ST7789_HandleTypeDef *display, uint8_t cmd)
{
  HAL_StatusTypeDef status;

  ST7789_Select(display);
  HAL_GPIO_WritePin(display->dc_port, display->dc_pin, GPIO_PIN_RESET);
  st7789_blocking_tx_count++;
  status = HAL_SPI_Transmit(display->hspi, &cmd, 1U, ST7789_SPI_TIMEOUT_MS);
  ST7789_Unselect(display);

  return status;
}

static HAL_StatusTypeDef ST7789_WriteData(ST7789_HandleTypeDef *display,
                                          const uint8_t *data,
                                          uint16_t len)
{
  HAL_StatusTypeDef status;

  ST7789_Select(display);
  HAL_GPIO_WritePin(display->dc_port, display->dc_pin, GPIO_PIN_SET);

  st7789_blocking_tx_count++;
  status = HAL_SPI_Transmit(display->hspi, (uint8_t *)data, len, ST7789_SPI_TIMEOUT_MS);
  ST7789_Unselect(display);

  return status;
}

static HAL_StatusTypeDef ST7789_SetAddressWindow(ST7789_HandleTypeDef *display,
                                                 uint16_t x0,
                                                 uint16_t y0,
                                                 uint16_t x1,
                                                 uint16_t y1)
{
  uint8_t data[4];

  x0 += display->x_offset;
  x1 += display->x_offset;
  y0 += display->y_offset;
  y1 += display->y_offset;

  data[0] = (uint8_t)(x0 >> 8U);
  data[1] = (uint8_t)(x0 & 0xFFU);
  data[2] = (uint8_t)(x1 >> 8U);
  data[3] = (uint8_t)(x1 & 0xFFU);
  if ((ST7789_WriteCommand(display, ST7789_CASET) != HAL_OK) ||
      (ST7789_WriteData(display, data, 4U) != HAL_OK))
  {
    return HAL_ERROR;
  }

  data[0] = (uint8_t)(y0 >> 8U);
  data[1] = (uint8_t)(y0 & 0xFFU);
  data[2] = (uint8_t)(y1 >> 8U);
  data[3] = (uint8_t)(y1 & 0xFFU);
  if ((ST7789_WriteCommand(display, ST7789_RASET) != HAL_OK) ||
      (ST7789_WriteData(display, data, 4U) != HAL_OK))
  {
    return HAL_ERROR;
  }

  return ST7789_WriteCommand(display, ST7789_RAMWR);
}

static void ST7789_Select(ST7789_HandleTypeDef *display)
{
  if (display->cs_port != NULL)
  {
    HAL_GPIO_WritePin(display->cs_port, display->cs_pin, GPIO_PIN_RESET);
  }
}

static void ST7789_Unselect(ST7789_HandleTypeDef *display)
{
  if (display->cs_port != NULL)
  {
    HAL_GPIO_WritePin(display->cs_port, display->cs_pin, GPIO_PIN_SET);
  }
}

static void ST7789_Reset(ST7789_HandleTypeDef *display)
{
  if (display->rst_port != NULL)
  {
    HAL_GPIO_WritePin(display->rst_port, display->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(20U);
    HAL_GPIO_WritePin(display->rst_port, display->rst_pin, GPIO_PIN_SET);
    HAL_Delay(120U);
  }
}

#else

HAL_StatusTypeDef ST7789_Init(ST7789_HandleTypeDef *display)
{
  (void)display;
  return HAL_ERROR;
}

HAL_StatusTypeDef ST7789_FillScreen(ST7789_HandleTypeDef *display, uint16_t color)
{
  (void)display;
  (void)color;
  return HAL_ERROR;
}

HAL_StatusTypeDef ST7789_FillRect(ST7789_HandleTypeDef *display,
                                  uint16_t x,
                                  uint16_t y,
                                  uint16_t w,
                                  uint16_t h,
                                  uint16_t color)
{
  (void)display;
  (void)x;
  (void)y;
  (void)w;
  (void)h;
  (void)color;
  return HAL_ERROR;
}

HAL_StatusTypeDef ST7789_DrawPixel(ST7789_HandleTypeDef *display,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t color)
{
  return ST7789_FillRect(display, x, y, 1U, 1U, color);
}

HAL_StatusTypeDef ST7789_DrawHLine(ST7789_HandleTypeDef *display,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t w,
                                   uint16_t color)
{
  return ST7789_FillRect(display, x, y, w, 1U, color);
}

HAL_StatusTypeDef ST7789_DrawVLine(ST7789_HandleTypeDef *display,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t h,
                                   uint16_t color)
{
  return ST7789_FillRect(display, x, y, 1U, h, color);
}

#endif /* HAL_SPI_MODULE_ENABLED */
