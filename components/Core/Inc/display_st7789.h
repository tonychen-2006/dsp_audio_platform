#ifndef DISPLAY_ST7789_H
#define DISPLAY_ST7789_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define ST7789_WIDTH   240U
#define ST7789_HEIGHT  240U

#define ST7789_BLACK   0x0000U
#define ST7789_WHITE   0xFFFFU
#define ST7789_RED     0xF800U
#define ST7789_GREEN   0x07E0U
#define ST7789_BLUE    0x001FU
#define ST7789_YELLOW  0xFFE0U
#define ST7789_CYAN    0x07FFU
#define ST7789_MAGENTA 0xF81FU
#define ST7789_GRAY    0x8410U

extern volatile uint32_t st7789_dma_tx_count;
extern volatile uint32_t st7789_blocking_tx_count;
extern volatile uint32_t st7789_dma_error_count;

typedef struct
{
#ifdef HAL_SPI_MODULE_ENABLED
  SPI_HandleTypeDef *hspi;
#endif
  GPIO_TypeDef *dc_port;
  uint16_t dc_pin;
  GPIO_TypeDef *rst_port;
  uint16_t rst_pin;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  GPIO_TypeDef *bl_port;
  uint16_t bl_pin;
  uint16_t x_offset;
  uint16_t y_offset;
} ST7789_HandleTypeDef;

HAL_StatusTypeDef ST7789_Init(ST7789_HandleTypeDef *display);
HAL_StatusTypeDef ST7789_FillScreen(ST7789_HandleTypeDef *display, uint16_t color);
HAL_StatusTypeDef ST7789_FillRect(ST7789_HandleTypeDef *display,
                                  uint16_t x,
                                  uint16_t y,
                                  uint16_t w,
                                  uint16_t h,
                                  uint16_t color);
HAL_StatusTypeDef ST7789_DrawPixel(ST7789_HandleTypeDef *display,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t color);
HAL_StatusTypeDef ST7789_DrawHLine(ST7789_HandleTypeDef *display,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t w,
                                   uint16_t color);
HAL_StatusTypeDef ST7789_DrawVLine(ST7789_HandleTypeDef *display,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t h,
                                   uint16_t color);
HAL_StatusTypeDef ST7789_DrawRGB565Bitmap(ST7789_HandleTypeDef *display,
                                           uint16_t x,
                                           uint16_t y,
                                           uint16_t w,
                                           uint16_t h,
                                           const uint8_t *data,
                                           uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_ST7789_H */
