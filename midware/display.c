#include "display.h"

//static struct {
//  int x, y;
//} DISPLAY_OFFSET;


void display_clear(void) {
  fb_fill_rect(0, 0, MAX_DISPLAY_RESX, MAX_DISPLAY_RESY, 0x0000);
}

void display_bar(int x, int y, int w, int h, uint16_t c) {
  fb_fill_rect(x, y, w, h, c);
}


static inline void clamp_coords(int x, int y, int w, int h, int *x0, int *y0,
                                int *x1, int *y1) {
  *x0 = MAX(x, 0);
  *y0 = MAX(y, 0);
  *x1 = MIN(x + w - 1, DISPLAY_RESX - 1);
  *y1 = MIN(y + h - 1, DISPLAY_RESY - 1);
}

#if 0
void display_image(int x, int y, int w, int h, const void *data,
                   uint32_t datalen) {
  x += DISPLAY_OFFSET.x;
  y += DISPLAY_OFFSET.y;
  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  int x_pos = 0, y_pos = 0;
  clamp_coords(x, y, w, h, &x0, &y0, &x1, &y1);
  x_pos = x0;
  y_pos = y0;
  display_set_window(x0, y0, x1, y1);
  x0 -= x;
  x1 -= x;
  y0 -= y;
  y1 -= y;

  struct uzlib_uncomp decomp = {0};
  uint8_t decomp_window[UZLIB_WINDOW_SIZE] = {0};
  uint8_t decomp_out[2] = {0};
  uzlib_prepare(&decomp, decomp_window, data, datalen, decomp_out,
                sizeof(decomp_out));

  for (uint32_t pos = 0; pos < w * h; pos++) {
    int st = uzlib_uncompress(&decomp);
    if (st == TINF_DONE) break;  // all OK
    if (st < 0) break;           // error
    const int px = pos % w;
    const int py = pos / w;
    if (px >= x0 && px <= x1 && py >= y0 && py <= y1) {
      fb_write_pixel(x_pos + px, y_pos + py,
                     (decomp_out[0] << 8) | decomp_out[1]);
    }
    decomp.dest = (uint8_t *)&decomp_out;
  }
}
#endif

