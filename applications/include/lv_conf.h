#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_16_SWAP 0
#define LV_COLOR_DEPTH 16
#define LV_USE_PERF_MONITOR 1

#include <rtconfig.h>
#define LV_HOR_RES_MAX          480    //水平方向
#define LV_VER_RES_MAX          800    //垂直方向

//我们要使能的demo 假如你要使能其他的可以去，packages文件夹下找到
//lv_conf_template.h 下找到对应的宏复制过来哈

#define LV_USE_DEMO_RTT_MUSIC       0
#define LV_DEMO_RTT_MUSIC_AUTO_PLAY 0

#define LV_FONT_MONTSERRAT_12       1
#define LV_FONT_MONTSERRAT_16       1



#endif
