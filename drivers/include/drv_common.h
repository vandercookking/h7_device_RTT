/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-7      SummerGift   first version
 */

#ifndef __DRV_COMMON_H__

#define __DRV_COMMON_H__

#include <rtthread.h>
#include <rthw.h>
#include <board.h>
#include <stm32h7xx.h>

#ifdef __cplusplus
extern "C"
{
#endif

void _Error_Handler(char *s, int num);

#ifndef Error_Handler
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#endif

#define DMA_NOT_AVAILABLE ((DMA_INSTANCE_TYPE *)0xFFFFFFFFU)

#define __STM32_PORT(port)  GPIO##port##_BASE
#define GET_PIN(PORTx,PIN) (rt_base_t)((16 * ( ((rt_base_t)__STM32_PORT(PORTx) - (rt_base_t)GPIOA_BASE)/(0x0400UL) )) + PIN)
#define STM32_FLASH_START_ADRESS       ROM_START
#define STM32_FLASH_SIZE               ROM_SIZE
#define STM32_FLASH_END_ADDRESS        ROM_END

#define STM32_SRAM1_SIZE               RAM_SIZE
#define STM32_SRAM1_START              RAM_START
#define STM32_SRAM1_END                RAM_END

#define STMRE_SRAM2_SEZE               RAM2_SIZE
#define STM32_SRAM2_START              RAM2_START
#define STM32_SRAM2_END                RAM2_END

#define STMRE_SRAM3_SEZE               RAM3_SIZE
#define STM32_SRAM3_START              RAM3_START
#define STM32_SRAM3_END                RAM3_END

#if defined(__CC_ARM) || defined(__CLANG_ARM)
extern int Image$RW_IRAM1$ZI$Limit;
#define HEAP_BEGIN      ((void *)&Image$RW_IRAM1$ZI$Limit)
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#else
extern int __bss_end;
#define HEAP_BEGIN      ((void *)&__bss_end)
#endif

#define HEAP_END                       STM32_SRAM1_END
#ifdef __cplusplus
}
#endif


#ifndef MIN
#define MIN(a, b)       \
  ({                    \
    typeof(a) _a = (a); \
    typeof(b) _b = (b); \
    _a < _b ? _a : _b;  \
  })
#endif
#ifndef MAX
#define MAX(a, b)       \
  ({                    \
    typeof(a) _a = (a); \
    typeof(b) _b = (b); \
    _a > _b ? _a : _b;  \
  })
#endif

#endif
