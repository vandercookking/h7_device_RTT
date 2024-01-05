/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-7      SummerGift   first version
 */

#include "drv_common.h"
#include "board.h"

#ifdef RT_USING_FINSH
#include <finsh.h>
static void reboot(uint8_t argc, char **argv)
{
    rt_hw_cpu_reset();
}
MSH_CMD_EXPORT(reboot,Reboot System);
#endif /* RT_USING_FINSH */

static uint32_t _systick_ms = 1;

/* SysTick configuration */
void rt_hw_systick_init(void)
{
#if 0
#if defined (SOC_SERIES_STM32H7)
    HAL_SYSTICK_Config((HAL_RCCEx_GetD1SysClockFreq()) / RT_TICK_PER_SECOND);
#else
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / RT_TICK_PER_SECOND);
#endif
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
#endif
    HAL_SYSTICK_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    NVIC_SetPriority(SysTick_IRQn, 0xFF);

    _systick_ms = 1000u / RT_TICK_PER_SECOND;
    if(_systick_ms == 0)
        _systick_ms = 1;
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_IncTick();
    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

uint32_t HAL_GetTick(void)
{
    return rt_tick_get() * 1000 / RT_TICK_PER_SECOND;
}

void HAL_SuspendTick(void)
{
}

void HAL_ResumeTick(void)
{
}

void HAL_Delay(__IO uint32_t Delay)
{
    __IO uint64_t tmpmrd = 0;

    /* interrupt is not enable, just to delay some time. */
//    for (tmpmrd = 0; tmpmrd < (655*Delay); tmpmrd ++)
    for (tmpmrd = 0; tmpmrd < (655*Delay); tmpmrd ++)
        ;
}

/* re-implement tick interface for STM32 HAL */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char *s, int num)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler */
}

/**
 * This function will delay for some us.
 *
 * @param us the delay time of us
 */
void rt_hw_us_delay(rt_uint32_t us)
{
    rt_uint32_t start, now, delta, reload, us_tick;
    start = SysTick->VAL;
    reload = SysTick->LOAD;
    us_tick = SystemCoreClock / 1000000UL;
    do {
        now = SysTick->VAL;
        delta = start > now ? start - now : reload + start - now;
    } while(delta < us_tick * us);
}

void reset_flags_reset(void) {
  RCC->RSR |= RCC_RSR_RMVF;  // clear the reset flags
}

void PVD_Init(void)
{
    // enable the PVD (programmable voltage detector).
    // select the "2.7V" threshold (level 5).
    // this detector will be active regardless of the
    // flash option byte BOR setting.

    PWR_PVDTypeDef pvd_config;
    pvd_config.PVDLevel = PWR_PVDLEVEL_5;
    pvd_config.Mode = PWR_PVD_MODE_IT_RISING_FALLING;
    HAL_PWR_ConfigPVD(&pvd_config);
    HAL_PWR_EnablePVD();
    NVIC_EnableIRQ(PVD_AVD_IRQn);
}
extern void clk_init(char *clk_source, int source_freq, int target_freq);

/**
 * This function will initial STM32 board.
 */
void hw_board_init(char *clock_src, int32_t clock_src_freq, int32_t clock_target_freq)
{
    reset_flags_reset();

    /* HAL_Init() function is called at the beginning of the program */
    HAL_Init();

    PVD_Init();

#ifdef SCB_EnableICache
    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();
#endif

#ifdef SCB_EnableDCache
    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();
#endif

    /* System clock initialization */
//    SystemClock_Config();


    /* enable interrupt */
    __set_PRIMASK(0);
    /* System clock initialization */
    clk_init(clock_src, clock_src_freq, clock_target_freq);
    /* disbale interrupt */
    __set_PRIMASK(1);

    rt_hw_systick_init();

    /* Pin driver initialization is open by default */
#ifdef RT_USING_PIN
    extern int rt_hw_pin_init(void);
    rt_hw_pin_init();
#endif

    /* USART driver initialization is open by default */
#ifdef RT_USING_SERIAL
    extern int rt_hw_usart_init(void);
    rt_hw_usart_init();
#endif

}
