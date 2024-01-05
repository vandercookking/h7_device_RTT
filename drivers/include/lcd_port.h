/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-01-08     zylx         first version
 */

#ifndef __LCD_PORT_H__
#define __LCD_PORT_H__
#include <stdint.h>

#define LCD_BACKLIGHT_USING_PWM
#define LCD_PWM_DEV_NAME    "pwm1"
#define LCD_PWM_DEV_CHANNEL 1
/* 3.1 inch screen, 800 * 480 */

/* LCD reset pin */
#define LCD_RESET_PIN GPIO_PIN_3
#define LCD_RESET_GPIO_PORT GPIOG

/* LCD tearing effect pin */
#define LCD_TE_PIN GPIO_PIN_2
#define LCD_TE_GPIO_PORT GPIOJ



/* armfly 5 inch screen, 800 * 480 */
#define LCD_WIDTH           480
#define LCD_HEIGHT          1600
#define LCD_BITS_PER_PIXEL  16
#define LCD_BUF_SIZE        (LCD_WIDTH * LCD_HEIGHT * LCD_BITS_PER_PIXEL / 8)
#define LCD_PIXEL_FORMAT    RTGRAPHIC_PIXEL_FORMAT_RGB565

#define LCD_HSYNC_WIDTH     96
#define LCD_VSYNC_HEIGHT    8
#define LCD_HBP             20
#define LCD_VBP             10
#define LCD_HFP             20
#define LCD_VFP             20

#define LCD_HSYNC           8
#define LCD_VSYNC           8

//////////////////////////////////////////
#define DSI_FREQ 30000U
#define LTDC_FREQ 26400U

#define ST7701S_FORMAT_RGB888 0x70 /* Pixel  RGB888 : 24 bpp*/
#define ST7701S_FORMAT_RBG565 0x50 /* Pixel  RGB565 : 16 bpp */

#define LCD_PIXEL_FORMAT_ARGB8888        0x00000000U   /*!< ARGB8888 LTDC pixel format */
#define LCD_PIXEL_FORMAT_RGB888          0x00000001U   /*!< RGB888 LTDC pixel format   */
#define LCD_PIXEL_FORMAT_RGB565          0x00000002U   /*!< RGB565 LTDC pixel format   */

#define ST7701S_480X800_HSYNC 8 /* Horizontal synchronization */
#define ST7701S_480X800_HBP 20  /* Horizontal back porch      */
#define ST7701S_480X800_HFP 20  /* Horizontal front porch     */
#define ST7701S_480X800_VSYNC 8 /* Vertical synchronization   */
#define ST7701S_480X800_VBP 8   /* Vertical back porch        */
#define ST7701S_480X800_VFP 20  /* Vertical front porch       */

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))



typedef struct {
  uint32_t xres;
  uint32_t yres;
  uint32_t bbp;
  uint32_t pixel_format;
  uint32_t fb_base;
} LCD_PARAMS;


typedef struct time_sequence {
  int hsync;
  int hbp;
  int hfp;

  int vsync;
  int vbp;
  int vfp;
} LCD_TIME_SEQUENCE;

extern void fb_fill_rect(uint32_t x_pos, uint32_t y_pos, uint32_t width,
        uint32_t height, uint32_t color);

extern void fb_write_pixel(uint32_t x_pos, uint32_t y_pos, uint32_t color);
extern void fb_fill_rect(uint32_t x_pos, uint32_t y_pos, uint32_t width,
                  uint32_t height, uint32_t color);
extern void fb_draw_hline(uint32_t x_pos, uint32_t y_pos, uint32_t len,
                   uint32_t color);
extern void fb_draw_vline(uint32_t x_pos, uint32_t y_pos, uint32_t len,
                   uint32_t color);

extern void stm32_mipi_display_on(void);
extern void stm32_mipi_display_off(void);

#endif /* __LCD_PORT_H__ */
