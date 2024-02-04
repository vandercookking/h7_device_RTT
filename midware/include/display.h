#ifndef __DISPLAY_H__
#define __DISPLAY_H__
#include "lcd_port.h"
#include <drv_common.h>

#define MAX_DISPLAY_RESX 480
#define MAX_DISPLAY_RESY 800

#define DISPLAY_RESX 480
#define DISPLAY_RESY 800

#define BOARD_MODE 1
#define BOOT_MODE 2
#define PIXEL_STEP 5

#define RGB16(R, G, B) ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | ((B & 0xF8) >> 3)
#define COLOR_WHITE 0xFFFF
#define COLOR_BLACK 0x0000
#define COLOR_RED RGB16(0xFF, 0x00, 0x00)    // red
#define COLOR_BLUE RGB16(0x00, 0x00, 0xFF)   // blue
#define COLOR_GREEN RGB16(0x00, 0xFF, 0x00)  // green
#define COLOR_DARK RGB16(0x33, 0x33, 0x33)
#define COLOR_GRAY RGB16(0x99, 0x99, 0x99)


extern void display_clear(void);
extern void display_bar(int x, int y, int w, int h, uint16_t c);
extern void display_drw_point(uint32_t x_pos,uint32_t y_pos,uint32_t color);

#endif
