/**
 * @file lv_port_indev_templ.c
 *
 */

/*Copy this file as "lv_port_indev.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev.h"
#include "lvgl.h"
#include "drivers/touch.h"
#include <drv_log.h>
#include <board.h>
#include "rtdevice.h"
#include "gt911.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

//static void touchpad_init(void);
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
//static bool touchpad_is_pressed(void);
//static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_touchpad;
lv_indev_t * indev_button;


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void)
{
    /**
     * Here you will find example implementation of input devices supported by LittelvGL:
     *  - Touchpad
     *  - Mouse (with cursor support)
     *  - Keypad (supports GUI usage only with key)
     *  - Encoder (supports GUI usage only with: left, right, push)
     *  - Button (external buttons to press points on the screen)
     *
     *  The `..._read()` function are only examples.
     *  You should shape them according to your hardware
     */

    static lv_indev_drv_t indev_drv;

    /*------------------
     * Touchpad
     * -----------------*/

    /*Initialize your touchpad if you have*/
//    touchpad_init();

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    indev_touchpad = lv_indev_drv_register(&indev_drv);

    /*Later you should create group(s) with `lv_group_t * group = lv_group_create()`,
     *add objects to the group with `lv_group_add_obj(group, obj)`
     *and assign this input device to group to navigate in it:
     *`lv_indev_set_group(indev_keypad, group);`*/
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*------------------
 * Touchpad
 * -----------------*/
//int rt_hw_gt911_port(void)
//{
//    struct rt_touch_config cfg;
//    rt_uint8_t rst_pin;
//
//    rst_pin = GT911_RST_PIN;
//    cfg.dev_name = "i2c1";
//    cfg.irq_pin.pin = GT911_IRQ_PIN;
//    cfg.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;
//    cfg.user_data = &rst_pin;
//
//    rt_hw_gt911_init("gt911", &cfg);
//
//    return 0;
//}

rt_device_t  touch_dev;
static int lv_hw_touch_init(void)
{
    struct rt_touch_config cfg;

    cfg.dev_name = "i2c1";/* 使用的I2C设备名 */

    rt_hw_gt911_init("touch", &cfg);

//    rt_hw_gt911_port();

    touch_dev = rt_device_find("touch");
    if (rt_device_open(touch_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        LOG_E("Can't open touch device:%s", "touch");
        return -RT_ERROR;
    }

    return RT_EOK;
}

INIT_COMPONENT_EXPORT(lv_hw_touch_init);



#if 0
/*Initialize your touchpad*/
static void touchpad_init(void)
{
    /*Your code comes here*/
    rt_hw_gt911_port();
}
#endif

/*Will be called by the library to read the touchpad*/
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
//    static lv_coord_t last_x = 0;
//    static lv_coord_t last_y = 0;
//
//    /*Save the pressed coordinates and the state*/
//    if(touchpad_is_pressed()) {
//        touchpad_get_xy(&last_x, &last_y);
//        data->state = LV_INDEV_STATE_PR;
//    }
//    else {
//        data->state = LV_INDEV_STATE_REL;
//    }
//
//    /*Set the last pressed coordinates*/
//    data->point.x = last_x;
//    data->point.y = last_y;

    struct rt_touch_data *read_data;
    /* 可以将内存分配这个步骤改为全局变量，以提高读取效率 */
    read_data = (struct rt_touch_data *)rt_calloc(1, sizeof(struct rt_touch_data));

    rt_device_read(touch_dev, 0, read_data, 1);

    /* 没有触摸事件直接返回*/
    if (read_data->event == RT_TOUCH_EVENT_NONE)
        return;

    /* 这里需要注意的是：触摸驱动的原点可能和LCD的原点不一致，所以需要我们进行一些处理 */

    data->point.x = read_data->x_coordinate;
    data->point.y = read_data->y_coordinate;

    if (read_data->event == RT_TOUCH_EVENT_DOWN)
        data->state = LV_INDEV_STATE_PR;
    if (read_data->event == RT_TOUCH_EVENT_MOVE)
        data->state = LV_INDEV_STATE_PR;
    if (read_data->event == RT_TOUCH_EVENT_UP)
        data->state = LV_INDEV_STATE_REL;
}
#if 0

/*Return true is the touchpad is pressed*/
static bool touchpad_is_pressed(void)
{
    /*Your code comes here*/

    return false;
}

/*Get the x and y coordinates if the touchpad is pressed*/
static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y)
{
    /*Your code comes here*/

    (*x) = 0;
    (*y) = 0;
}
#endif

//INIT_ENV_EXPORT(rt_hw_gt911_port);


#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
