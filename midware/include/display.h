#ifndef __DISPLAY_H__
#define __DISPLAY_H__
#include "lcd_port.h"
#include <drv_common.h>

#define MAX_DISPLAY_RESX 480
#define MAX_DISPLAY_RESY 1600

#define DISPLAY_RESX 480
#define DISPLAY_RESY 800

extern void display_clear(void);
extern void display_bar(int x, int y, int w, int h, uint16_t c);

#endif
