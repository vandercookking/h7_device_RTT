/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2019-05-23     WillianChan    first version
 */

#include <board.h>

#ifdef BSP_USING_LCD_MIPI
#include <lcd_port.h>
#include "sdram_port.h"
#include <string.h>
#include "drv_mipi.h"
#define LOG_TAG             "drv.mipi"
#include <drv_log.h>

#define DRV_DEBUG

DSI_HandleTypeDef   hdsi;
DSI_VidCfgTypeDef   hdsi_video;
LTDC_HandleTypeDef  hltdc;
///////////////////////////////
LCD_PARAMS lcd_params;

DMA2D_HandleTypeDef hlcd_dma2d;

LCD_TIME_SEQUENCE lcd_time_seq = {.hsync = ST7701S_480X800_HSYNC,
                                  .hfp = ST7701S_480X800_HFP,
                                  .hbp = ST7701S_480X800_HBP,
                                  .vsync = ST7701S_480X800_VSYNC,
                                  .vfp = ST7701S_480X800_VFP,
                                  .vbp = ST7701S_480X800_VBP};

#define CONVERTRGB5652ARGB8888(Color)                                   \
  ((((((((Color) >> (11U)) & 0x1FU) * 527U) + 23U) >> (6U)) << (16U)) | \
   (((((((Color) >> (5U)) & 0x3FU) * 259U) + 33U) >> (6U)) << (8U)) |   \
   (((((Color)&0x1FU) * 527U) + 23U) >> (6U)) | (0xFF000000U))

struct stm32_lcd
{
    struct rt_device parent;
    struct rt_device_graphic_info info;
};
struct stm32_lcd lcd;


static void fb_fill_buffer(uint32_t *dest, uint32_t x_size, uint32_t y_size,
                           uint32_t offset, uint32_t color) {
  uint32_t output_color_mode, input_color = color;

  switch (lcd_params.pixel_format) {
    case LTDC_PIXEL_FORMAT_RGB565:
      output_color_mode = DMA2D_OUTPUT_RGB565; /* RGB565 */
      input_color = CONVERTRGB5652ARGB8888(color);
      break;
    case LTDC_PIXEL_FORMAT_RGB888:
    default:
      output_color_mode = DMA2D_OUTPUT_ARGB8888; /* ARGB8888 */
      break;
  }

  /* Register to memory mode with ARGB8888 as color Mode */
  hlcd_dma2d.Init.Mode = DMA2D_R2M;
  hlcd_dma2d.Init.ColorMode = output_color_mode;
  hlcd_dma2d.Init.OutputOffset = offset;

  hlcd_dma2d.Instance = DMA2D;

  /* DMA2D Initialization */
  if (HAL_DMA2D_Init(&hlcd_dma2d) == HAL_OK) {
    if (HAL_DMA2D_ConfigLayer(&hlcd_dma2d, 1) == HAL_OK) {
      if (HAL_DMA2D_Start(&hlcd_dma2d, input_color, (uint32_t)dest, x_size,
                          y_size) == HAL_OK) {
        /* Polling For DMA transfer */
        (void)HAL_DMA2D_PollForTransfer(&hlcd_dma2d, 25);
      }
    }
  }
}

void fb_fill_rect(uint32_t x_pos, uint32_t y_pos, uint32_t width,
                  uint32_t height, uint32_t color) {
  /* Get the rectangle start address */
  uint32_t address = lcd_params.fb_base +
                     ((lcd_params.bbp) * (lcd_params.xres * y_pos + x_pos));

  /* Fill the rectangle */
  fb_fill_buffer((uint32_t *)address, width, height, (lcd_params.xres - width),
                 color);
}

void fb_draw_hline(uint32_t x_pos, uint32_t y_pos, uint32_t len,
                   uint32_t color) {
  uint32_t address = lcd_params.fb_base +
                     ((lcd_params.bbp) * (lcd_params.xres * y_pos + x_pos));
  fb_fill_buffer((uint32_t *)address, len, 1, 0, color);
}

void fb_draw_vline(uint32_t x_pos, uint32_t y_pos, uint32_t len,
                   uint32_t color) {
  uint32_t address = lcd_params.fb_base +
                     ((lcd_params.bbp) * (lcd_params.xres * y_pos + x_pos));
  fb_fill_buffer((uint32_t *)address, 1, len, lcd_params.xres - 1, color);
}


static void ltcd_msp_init(LTDC_HandleTypeDef *hltdc) {
  if (hltdc->Instance == LTDC) {
    /** Enable the LTDC clock */
    __HAL_RCC_LTDC_CLK_ENABLE();

    /** Toggle Sw reset of LTDC IP */
    __HAL_RCC_LTDC_FORCE_RESET();
    __HAL_RCC_LTDC_RELEASE_RESET();
  }
}
static void dma2d_msp_init(DMA2D_HandleTypeDef *hdma2d) {
  if (hdma2d->Instance == DMA2D) {
    /** Enable the DMA2D clock */
    __HAL_RCC_DMA2D_CLK_ENABLE();

    /** Toggle Sw reset of DMA2D IP */
    __HAL_RCC_DMA2D_FORCE_RESET();
    __HAL_RCC_DMA2D_RELEASE_RESET();
  }
}

static void dsi_msp_init(DSI_HandleTypeDef *hdsi) {
  if (hdsi->Instance == DSI) {
    /** Enable DSI Host and wrapper clocks */
    __HAL_RCC_DSI_CLK_ENABLE();

    /** Soft Reset the DSI Host and wrapper */
    __HAL_RCC_DSI_FORCE_RESET();
    __HAL_RCC_DSI_RELEASE_RESET();
  }
}

HAL_StatusTypeDef dsi_host_init(DSI_HandleTypeDef *hdsi, uint32_t Width,
                                uint32_t Height, uint32_t PixelFormat) {
    DSI_PLLInitTypeDef PLLInit;
    DSI_VidCfgTypeDef hdsi_video;

    hdsi->Instance = DSI;
    hdsi->Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
    hdsi->Init.TXEscapeCkdiv = 4;
    hdsi->Init.NumberOfLanes = DSI_TWO_DATA_LANES;
    PLLInit.PLLNDIV = 96;
    PLLInit.PLLIDF = DSI_PLL_IN_DIV5;
    PLLInit.PLLODF = DSI_PLL_OUT_DIV2;
    if (HAL_DSI_Init(hdsi, &PLLInit) != HAL_OK) {
    return HAL_ERROR;
    }

    /* Timing parameters for all Video modes */
    /*
    The lane byte clock is set 62500 Khz
    The pixel clock is set to 27429 Khz
    */
    hdsi_video.VirtualChannelID = 0;
    hdsi_video.ColorCoding = PixelFormat;
    hdsi_video.LooselyPacked = DSI_LOOSELY_PACKED_DISABLE;
    hdsi_video.Mode = DSI_VID_MODE_BURST;
    hdsi_video.PacketSize = Width;
    hdsi_video.NumberOfChunks = 0;
    hdsi_video.NullPacketSize = 0xFFFU;
    hdsi_video.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
    hdsi_video.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
    hdsi_video.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
    hdsi_video.HorizontalSyncActive = (lcd_time_seq.hsync * DSI_FREQ) / LTDC_FREQ;
    hdsi_video.HorizontalBackPorch = (lcd_time_seq.hbp * DSI_FREQ) / LTDC_FREQ;
    hdsi_video.HorizontalLine =
      ((Width + lcd_time_seq.hsync + lcd_time_seq.hbp + lcd_time_seq.hfp) *
       DSI_FREQ) /
      LTDC_FREQ;
    hdsi_video.VerticalSyncActive = lcd_time_seq.vsync;
    hdsi_video.VerticalBackPorch = lcd_time_seq.vbp;
    hdsi_video.VerticalFrontPorch = lcd_time_seq.vfp;
    hdsi_video.VerticalActive = Height;
    hdsi_video.LPCommandEnable = DSI_LP_COMMAND_DISABLE;
    hdsi_video.LPLargestPacketSize = 0;
    hdsi_video.LPVACTLargestPacketSize = 0;

    hdsi_video.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;
    hdsi_video.LPHorizontalBackPorchEnable = DSI_LP_HBP_ENABLE;
    hdsi_video.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;
    hdsi_video.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;
    hdsi_video.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;
    hdsi_video.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE;
    hdsi_video.FrameBTAAcknowledgeEnable = DSI_FBTAA_DISABLE;

    if (HAL_DSI_ConfigVideoMode(hdsi, &hdsi_video) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ltdc_clock_config(LTDC_HandleTypeDef *hltdc) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLL3.PLL3M = 5U;
  PeriphClkInitStruct.PLL3.PLL3N = 132U;
  PeriphClkInitStruct.PLL3.PLL3P = 2U;
  PeriphClkInitStruct.PLL3.PLL3Q = 2U;
  PeriphClkInitStruct.PLL3.PLL3R = 25U;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLLCFGR_PLL3RGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0U;
  return HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

HAL_StatusTypeDef hltdc_init(LTDC_HandleTypeDef *hltdc, uint32_t Width,
                            uint32_t Height) {
  hltdc->Instance = LTDC;
  hltdc->Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc->Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc->Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc->Init.PCPolarity = LTDC_PCPOLARITY_IPC;

  hltdc->Init.HorizontalSync = lcd_time_seq.hsync - 1;
  hltdc->Init.AccumulatedHBP = lcd_time_seq.hsync + lcd_time_seq.hbp - 1;
  hltdc->Init.AccumulatedActiveW =
      lcd_time_seq.hsync + Width + lcd_time_seq.hbp - 1;
  hltdc->Init.TotalWidth =
      lcd_time_seq.hsync + Width + lcd_time_seq.hbp + lcd_time_seq.hfp - 1;
  hltdc->Init.VerticalSync = lcd_time_seq.vsync - 1;
  hltdc->Init.AccumulatedVBP = lcd_time_seq.vsync + lcd_time_seq.vbp - 1;
  hltdc->Init.AccumulatedActiveH =
      lcd_time_seq.vsync + Height + lcd_time_seq.vbp - 1;
  hltdc->Init.TotalHeigh =
      lcd_time_seq.vsync + Height + lcd_time_seq.vbp + lcd_time_seq.vfp - 1;

  hltdc->Init.Backcolor.Blue = 0x00;
  hltdc->Init.Backcolor.Green = 0x00;
  hltdc->Init.Backcolor.Red = 0x00;

  return HAL_LTDC_Init(hltdc);
}

void st7701_dsi_write(uint16_t reg, uint8_t *seq, uint16_t len) {
  if (len <= 1) {
    HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, reg,
                       (uint32_t)seq[0]);
  } else {
    HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, len, reg, seq);
  }
}

#define st7701_dsi(reg, seq...)              \
  {                                          \
    uint8_t d[] = {seq};                     \
    st7701_dsi_write(reg, d, ARRAY_SIZE(d)); \
  }



void st7701_init_sequence(void) {
    /* Command2, BK1 */

    st7701_dsi(0xff, 0x77, 0x01, 0x00, 0x00, 0x13);

    st7701_dsi(0xef, 0x08);

    st7701_dsi(0xff, 0x77, 0x01, 0x00, 0x00, 0x10);

    st7701_dsi(0xc0, 0x63, 0x00);

    st7701_dsi(0xc1, 0x14, 0x0C);

    st7701_dsi(0xc2, 0x37, 0x02);

    st7701_dsi(0xcc, 0x10);

    st7701_dsi(0xB0, 0x06, 0x10, 0x16, 0x0D, 0x11, 0x06, 0x08, 0x07, 0x08, 0x22,

               0x04, 0x14, 0x0F, 0x29, 0x2F, 0x1F);

    st7701_dsi(0xB1, 0x0F, 0x18, 0x1E, 0x0C, 0x0F, 0x06, 0x08, 0x0A, 0x09, 0x24,

               0x05, 0x10, 0x11, 0x2A, 0x34, 0x1F);

    st7701_dsi(0xff, 0x77, 0x01, 0x00, 0x00, 0x11);

    st7701_dsi(0xb0, 0x4D);

    st7701_dsi(0xb1, 0x4D);

    st7701_dsi(0xb2, 0x81);

    st7701_dsi(0xb3, 0x80);

    st7701_dsi(0xb5, 0x4E);

    st7701_dsi(0xb7, 0x85);

    st7701_dsi(0xb8, 0x32);

    st7701_dsi(0xBB, 0x03);

    st7701_dsi(0xc1, 0x08);

    st7701_dsi(0xc2, 0x08);

    st7701_dsi(0xd0, 0x88);

    st7701_dsi(0xe0, 0x00, 0x00, 0x02);

    st7701_dsi(0xE1, 0x06, 0x28, 0x08, 0x28, 0x05, 0x28, 0x07, 0x28, 0x0E, 0x33,

               0x33);

    st7701_dsi(0xE2, 0x30, 0x30, 0x33, 0x33, 0x34, 0x00, 0x00, 0x00, 0x34, 0x00,

               0x00, 0x00);

    st7701_dsi(0xe3, 0x00, 0x00, 0x33, 0x33);

    st7701_dsi(0xe4, 0x44, 0x44);

    st7701_dsi(0xE5, 0x09, 0x2F, 0x2C, 0x8C, 0x0B, 0x31, 0x2C, 0x8C, 0x0D, 0x33,

               0x2C, 0x8C, 0x0F, 0x35, 0x2C, 0x8C);

    st7701_dsi(0xE6, 0x00, 0x00, 0x33, 0x33);

    st7701_dsi(0xE7, 0x44, 0x44);

    st7701_dsi(0xE8, 0x08, 0x2E, 0x2C, 0x8C, 0x0A, 0x30, 0x2C, 0x8C, 0x0C, 0x32,

               0x2C, 0x8C, 0x0E, 0x34, 0x2C, 0x8C);

    st7701_dsi(0xE9, 0x36, 0x00);

    st7701_dsi(0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40);

    st7701_dsi(0xED, 0xFF, 0xFC, 0xB2, 0x45, 0x67, 0xFA, 0x01, 0xFF, 0xFF, 0x10,

               0xAF, 0x76, 0x54, 0x2B, 0xCF, 0xFF);

    st7701_dsi(0xef, 0x08, 0x08, 0x08, 0x45, 0x3f, 0x54);

    st7701_dsi(0xff, 0x77, 0x01, 0x00, 0x00, 0x13);

    st7701_dsi(0xe8, 0x00, 0x0e);

    st7701_dsi(0xff, 0x77, 0x01, 0x00, 0x00, 0x00);

    st7701_dsi(0x11);

    HAL_Delay(120);

    st7701_dsi(0xff, 0x77, 0x01, 0x00, 0x00, 0x13);

    st7701_dsi(0xe8, 0x00, 0x0c);

    HAL_Delay(10);

    st7701_dsi(0xe8, 0x00, 0x00);

    st7701_dsi(0xff, 0x77, 0x01, 0x00, 0x00, 0x00);

    st7701_dsi(0x36, 0x00);

    st7701_dsi(MIPI_DCS_SET_TEAR_ON, 0x00);

    st7701_dsi(MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x2C);

    st7701_dsi(MIPI_DCS_SET_PIXEL_FORMAT, 0x50);

    st7701_dsi(0x29);

    HAL_Delay(20);

    st7701_dsi(0xff, 0x77, 0x01, 0x00, 0x00, 0x10);

    st7701_dsi(0xe5, 0x00, 0x00);
}

void ltdc_layer_init(uint16_t index, uint32_t framebuffer)
{
    LTDC_LayerCfgTypeDef layer_cfg;

    layer_cfg.WindowX0        = 0;
    layer_cfg.WindowX1        = LCD_WIDTH;
    layer_cfg.WindowY0        = 0;
    layer_cfg.WindowY1        = LCD_HEIGHT;
    layer_cfg.PixelFormat     = DSI_RGB565;
    layer_cfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    layer_cfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    layer_cfg.Alpha           = 255;
    layer_cfg.Alpha0          = 0;
    layer_cfg.ImageWidth      = LCD_WIDTH;
    layer_cfg.ImageHeight     = LCD_HEIGHT;
    layer_cfg.Backcolor.Blue  = 0;
    layer_cfg.Backcolor.Green = 0;
    layer_cfg.Backcolor.Red   = 0;
    layer_cfg.FBStartAdress   = framebuffer;

    HAL_LTDC_ConfigLayer(&hltdc, &layer_cfg, index);
}

void lcd_mipi_pin_init(void)
{
    GPIO_InitTypeDef gpio_init_structure;

    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* Configure the GPIO Reset pin */
    gpio_init_structure.Pin = LCD_RESET_PIN;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_RESET_GPIO_PORT, &gpio_init_structure);

    /* Activate XRES active low */
    HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN,
                    GPIO_PIN_SET); /* Deactivate XRES */
    HAL_Delay(120);

    /* LCD_TE_CTRL GPIO configuration */
    __HAL_RCC_GPIOJ_CLK_ENABLE();

    gpio_init_structure.Pin = LCD_TE_PIN;
    gpio_init_structure.Mode = GPIO_MODE_INPUT;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(LCD_TE_GPIO_PORT, &gpio_init_structure);

    HAL_GPIO_WritePin(LCD_TE_GPIO_PORT, LCD_TE_PIN, GPIO_PIN_SET);
}

rt_err_t ltdc_init(uint32_t framebuffer)
{
    uint32_t pixel_format;
    uint32_t  dsi_pixel_format,ltdc_pixel_format;

    __HAL_RCC_LTDC_FORCE_RESET();
    __HAL_RCC_LTDC_RELEASE_RESET();

    __HAL_RCC_DMA2D_FORCE_RESET();
    __HAL_RCC_DMA2D_RELEASE_RESET();

    lcd_mipi_pin_init();

    pixel_format = LCD_PIXEL_FORMAT_RGB565;

    if (pixel_format == LCD_PIXEL_FORMAT_RGB565) {
        ltdc_pixel_format = LTDC_PIXEL_FORMAT_RGB565;
        dsi_pixel_format = DSI_RGB565;
        lcd_params.bbp = 2;
    } else {
        ltdc_pixel_format = LCD_PIXEL_FORMAT_ARGB8888;
        dsi_pixel_format = DSI_RGB888;
        lcd_params.bbp = 4;
    }

    lcd_params.pixel_format = ltdc_pixel_format;
    lcd_params.xres = LCD_WIDTH;
    lcd_params.yres = LCD_HEIGHT;
    lcd_params.fb_base = framebuffer;

    /* Initializes peripherals instance value */
    hltdc.Instance = LTDC;
    hlcd_dma2d.Instance = DMA2D;
    hdsi.Instance = DSI;

    ltcd_msp_init(&hltdc);

    dma2d_msp_init(&hlcd_dma2d);

    dsi_msp_init(&hdsi);

    dsi_host_init(&hdsi, LCD_WIDTH, LCD_HEIGHT, dsi_pixel_format);

    ltdc_clock_config(&hltdc);

    hltdc_init(&hltdc, LCD_WIDTH, LCD_HEIGHT);

    ltdc_layer_init(0,framebuffer);

    HAL_DSI_Start(&(hdsi));

    /* Enable the DSI BTW for read operations */
    (void)HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA);

    st7701_init_sequence();

    rt_kprintf("lcd init ok ...\n");

    return RT_EOK;
}



void LTDC_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_LTDC_IRQHandler(&hltdc);
    rt_interrupt_leave();
}

static rt_err_t stm32_lcd_init(rt_device_t device)
{
    lcd.info.width          = LCD_WIDTH;
    lcd.info.height         = LCD_HEIGHT;
    lcd.info.pixel_format   = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    lcd.info.bits_per_pixel = 16;
//    lcd.info.framebuffer    = (void *)rt_malloc_align(LCD_WIDTH * LCD_HEIGHT * (lcd.info.bits_per_pixel / 8), 32);
    lcd.info.framebuffer  =  (rt_uint8_t *)rt_memheap_alloc(&system_heap, LCD_WIDTH * LCD_HEIGHT * (lcd.info.bits_per_pixel / 8));

    memset(lcd.info.framebuffer, 0xa5, LCD_WIDTH * LCD_HEIGHT * (lcd.info.bits_per_pixel / 8));
    ltdc_init((uint32_t)lcd.info.framebuffer);
//    ltdc_layer_init(0, (uint32_t)lcd.info.framebuffer);

    return RT_EOK;
}

static rt_err_t stm32_lcd_control(rt_device_t device, int cmd, void *args)
{
    switch(cmd)
    {
    case RTGRAPHIC_CTRL_RECT_UPDATE:
        break;

    case RTGRAPHIC_CTRL_POWERON:
        stm32_mipi_display_on();
        break;

    case RTGRAPHIC_CTRL_POWEROFF:
        stm32_mipi_display_off();
        break;

    case RTGRAPHIC_CTRL_GET_INFO:
        rt_memcpy(args, &lcd.info, sizeof(lcd.info));
        break;

    case RTGRAPHIC_CTRL_SET_MODE:
        break;

    case RTGRAPHIC_CTRL_GET_EXT:
        break;
    }

    return RT_EOK;
}

int rt_hw_lcd_init(void)
{
    rt_err_t ret;

    rt_memset(&lcd, 0x00, sizeof(lcd));

    rt_kprintf("start lcd init  ...\n");

    lcd.parent.type    = RT_Device_Class_Graphic;
    lcd.parent.init    = stm32_lcd_init;
    lcd.parent.open    = RT_NULL;
    lcd.parent.close   = RT_NULL;
    lcd.parent.read    = RT_NULL;
    lcd.parent.write   = RT_NULL;
    lcd.parent.control = stm32_lcd_control;

    lcd.parent.user_data = (void *)&lcd.info;

    ret = rt_device_register(&lcd.parent, "lcd", RT_DEVICE_FLAG_RDWR);

    stm32_lcd_init(&lcd.parent);

    return ret;
}
INIT_DEVICE_EXPORT(rt_hw_lcd_init);


rt_weak void stm32_mipi_display_on(void)
{
    rt_kprintf("please Implementation function %s\n", __func__);
}

rt_weak void stm32_mipi_display_off(void)
{
    rt_kprintf("please Implementation function %s\n", __func__);

    extern void display_clear(void);
    display_clear();
}

#ifdef DRV_DEBUG
#ifdef FINSH_USING_MSH
int lcd_test(void)
{

    rt_kprintf("start lcd test ...\n");
    //test lcd only

//    st7701_dsi(0xff,0x77,0x01,0x00,0x00,0x12);
//
//    st7701_dsi(0xd1,0x81);
//
//    st7701_dsi(0xd2,0x00);

    //end test lcd

    extern void display_clear(void);
    display_clear();

//    struct drv_lcd_device *lcd;
//    lcd = (struct drv_lcd_device *)rt_device_find("lcd");


//        if (lcd->lcd_info.pixel_format == RTGRAPHIC_PIXEL_FORMAT_RGB565)
//        {
//            /* red */
//            for (int i = 0; i < LCD_BUF_SIZE / 2; i++)
//            {
//                lcd->lcd_info.framebuffer[2 * i] = 0x00;
//                lcd->lcd_info.framebuffer[2 * i + 1] = 0xF8;
//            }
//            lcd->parent.control(&lcd->parent, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
//            rt_thread_mdelay(1000);
//            /* green */
//            for (int i = 0; i < LCD_BUF_SIZE / 2; i++)
//            {
//                lcd->lcd_info.framebuffer[2 * i] = 0xE0;
//                lcd->lcd_info.framebuffer[2 * i + 1] = 0x07;
//            }
//            lcd->parent.control(&lcd->parent, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
//            rt_thread_mdelay(1000);
//            /* blue */
//            for (int i = 0; i < LCD_BUF_SIZE / 2; i++)
//            {
//                lcd->lcd_info.framebuffer[2 * i] = 0x1F;
//                lcd->lcd_info.framebuffer[2 * i + 1] = 0x00;
//            }
//        }
//        else if (lcd->lcd_info.pixel_format == RTGRAPHIC_PIXEL_FORMAT_RGB888)
//        {
//            /* red */
//            for (int i = 0; i < LCD_BUF_SIZE / 3; i++)
//            {
//                lcd->lcd_info.framebuffer[3 * i] = 0x00;
//                lcd->lcd_info.framebuffer[3 * i + 1] = 0x00;
//                lcd->lcd_info.framebuffer[3 * i + 2] = 0xff;
//            }
//            lcd->parent.control(&lcd->parent, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
//            rt_thread_mdelay(1000);
//            /* green */
//            for (int i = 0; i < LCD_BUF_SIZE / 3; i++)
//            {
//                lcd->lcd_info.framebuffer[3 * i] = 0x00;
//                lcd->lcd_info.framebuffer[3 * i + 1] = 0xff;
//                lcd->lcd_info.framebuffer[3 * i + 2] = 0x00;
//            }
//            lcd->parent.control(&lcd->parent, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
//            rt_thread_mdelay(1000);
//            /* blue */
//            for (int i = 0; i < LCD_BUF_SIZE / 3; i++)
//            {
//                lcd->lcd_info.framebuffer[3 * i] = 0xff;
//                lcd->lcd_info.framebuffer[3 * i + 1] = 0x00;
//                lcd->lcd_info.framebuffer[3 * i + 2] = 0x00;
//            }
//        }

//        lcd->parent.control(&lcd->parent, RTGRAPHIC_CTRL_RECT_UPDATE, RT_NULL);
//        rt_thread_mdelay(1000);
        return RT_EOK;
}
MSH_CMD_EXPORT(lcd_test, lcd test);
#endif /* FINSH_USING_MSH */
#endif /*DRV_DEBUG*/

#endif /* BSP_USING_LCD_MIPI */
