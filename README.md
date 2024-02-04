# dev_h7_pro
使用RT Studio 创建和编译的工程

## 创建工程

使用RT Studio创建基于STM32H747XI芯片的工程，串口配置为PA0-TX PA1-RX。

## 添加SDRAM

在board.h中添加#define BSP_USING_SDRAM，编译下载后，使用sdram_test测试读写文件情况。

![avatar](/png/sdram_test.png)

添加 memheap方法 管理多块内存。添加 #define RT_USING_MEMHEAP 和 #define RT_USING_MEMHEAP_AS_HEAP把宏控制的部分添加到编译中。

另外memheap.c隐藏，并且未被添加到工程中。需要在RT Studio软件工程中配置“过滤器和定制-->RTT Excluded Resource”取消勾选，并且在斜杠文件上右键选择"资源配置-->添加构建"，即可编译成功。

![avatar](/png/memheap.png)

## 添加LVGL并在屏幕上运行calendar demo

1.添加mipi lcd相关配置

2.添加lvgl packge

3.添加i2c相关驱动配置

3.适配GT911触摸芯片数据处理相关

![avatar](png/lvgl_reset.png)

![avatar](/png/list_thread.png)

![avatar](/png/list_device.png)
