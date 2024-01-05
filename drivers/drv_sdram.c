/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-04     zylx         first version
 */

#include <board.h>
#include "sdram.h"
#include "rtdef.h"

#ifdef BSP_USING_SDRAM
#include <sdram_port.h>

#define DRV_DEBUG
#define LOG_TAG             "drv.sdram"
#include <drv_log.h>

static SDRAM_HandleTypeDef hsdram[1];
static FMC_SDRAM_CommandTypeDef command;
#define FMC_SDRAM_TIMEOUT ((uint32_t)0xFFFF)
#ifdef RT_USING_MEMHEAP_AS_HEAP
struct rt_memheap system_heap;
#endif

/**
  * @brief
  * @param  hsdram: SDRAM handle
  * @param  Command: Pointer to SDRAM command structure
  * @retval None
  */
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
    __IO uint32_t tmpmrd = 0;

    /* Configure a clock configuration enable command */
    Command->CommandMode           = FMC_SDRAM_CMD_CLK_ENABLE;
    Command->CommandTarget         = FMC_SDRAM_CMD_TARGET_BANK2;
    Command->AutoRefreshNumber     = 1;
    Command->ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, FMC_SDRAM_TIMEOUT);

    /* Insert 100 ms delay */
    /* interrupt is not enable, just to delay some time. */
    for (tmpmrd = 0; tmpmrd < 0xffff; tmpmrd ++)
        ;

    /* Configure a PALL (precharge all) command */
    Command->CommandMode            = FMC_SDRAM_CMD_PALL;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
    Command->AutoRefreshNumber      = 1;
    Command->ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, FMC_SDRAM_TIMEOUT);

    /* Configure a Auto-Refresh command */
    Command->CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
    Command->AutoRefreshNumber      = 2;
    Command->ModeRegisterDefinition = 0;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, FMC_SDRAM_TIMEOUT);

  /* Program the external memory mode register */
  tmpmrd = (uint32_t)FMC_SDRAM_DEVICE_BURST_LENGTH_1 |
           FMC_SDRAM_DEVICE_BURST_TYPE_SEQUENTIAL |
           FMC_SDRAM_DEVICE_CAS_LATENCY_2 |
           FMC_SDRAM_DEVICE_OPERATING_MODE_STANDARD |
           FMC_SDRAM_DEVICE_WRITEBURST_MODE_SINGLE;

    Command->CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
    Command->CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2;
    Command->AutoRefreshNumber      = 1;
    Command->ModeRegisterDefinition = tmpmrd;

    /* Send the command */
    HAL_SDRAM_SendCommand(hsdram, Command, FMC_SDRAM_TIMEOUT);

    /* Set the device refresh counter */
    HAL_SDRAM_ProgramRefreshRate(hsdram, SDRAM_REFRESH_COUNT);
}
static int SDRAM_IO_Init(void)
{
    int result = RT_EOK;

    GPIO_InitTypeDef gpio_init_structure;

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FMC;
    PeriphClkInitStruct.PLL2.PLL2M = 5;
    PeriphClkInitStruct.PLL2.PLL2N = 120;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    return HAL_ERROR;
    }

    /* Enable FMC clock */
    __HAL_RCC_FMC_CLK_ENABLE();
    __FMC_NORSRAM_DISABLE(FMC_NORSRAM_DEVICE, FMC_NORSRAM_BANK1);
    __FMC_NORSRAM_DISABLE(FMC_NORSRAM_DEVICE, FMC_NORSRAM_BANK2);
    __FMC_NORSRAM_DISABLE(FMC_NORSRAM_DEVICE, FMC_NORSRAM_BANK3);
    __FMC_NORSRAM_DISABLE(FMC_NORSRAM_DEVICE, FMC_NORSRAM_BANK4);

    /* Enable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    /* Common GPIO configuration */
    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_structure.Alternate = GPIO_AF12_FMC;

    /* GPIOD configuration */
    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
                            GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;

    HAL_GPIO_Init(GPIOD, &gpio_init_structure);

    /* GPIOE configuration */
    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 |
                            GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                            GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                            GPIO_PIN_15;

    HAL_GPIO_Init(GPIOE, &gpio_init_structure);
    /* GPIOF configuration */
    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                            GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 |
                            GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                            GPIO_PIN_15;

    HAL_GPIO_Init(GPIOF, &gpio_init_structure);
    /* GPIOG configuration */
    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 |
                            GPIO_PIN_2 /*| GPIO_PIN_3 */ | GPIO_PIN_4 |
                            GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOG, &gpio_init_structure);

    gpio_init_structure.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio_init_structure);
    /* GPIOH configuration */
    gpio_init_structure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |
                            GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                            GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                            GPIO_PIN_15;

    HAL_GPIO_Init(GPIOH, &gpio_init_structure);

    /* GPIOI configuration */
    gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                            GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                            GPIO_PIN_9 | GPIO_PIN_10;

    HAL_GPIO_Init(GPIOI, &gpio_init_structure);

    return result;
}

static int SDRAM_Init(void)
{
    int result = RT_EOK;
    FMC_SDRAM_TimingTypeDef SDRAM_Timing;

    result = SDRAM_IO_Init();

    /* SDRAM device configuration */
    hsdram[0].Instance = FMC_SDRAM_DEVICE;

    /* SDRAM handle configuration */
    hsdram[0].Init.SDBank = FMC_SDRAM_BANK2;
    hsdram[0].Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
    hsdram[0].Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
    hsdram[0].Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
    hsdram[0].Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
    hsdram[0].Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
    hsdram[0].Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
    hsdram[0].Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
    hsdram[0].Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
    hsdram[0].Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;

    /* Timing configuration for 100Mhz as SDRAM clock frequency (System clock is
   * up to 200Mhz) */
    SDRAM_Timing.LoadToActiveDelay    = LOADTOACTIVEDELAY;
    SDRAM_Timing.ExitSelfRefreshDelay = EXITSELFREFRESHDELAY;
    SDRAM_Timing.SelfRefreshTime      = SELFREFRESHTIME;
    SDRAM_Timing.RowCycleDelay        = ROWCYCLEDELAY;
    SDRAM_Timing.WriteRecoveryTime    = WRITERECOVERYTIME;
    SDRAM_Timing.RPDelay              = RPDELAY;
    SDRAM_Timing.RCDDelay             = RCDDELAY;

    /* Initialize the SDRAM controller */
    if (HAL_SDRAM_Init(&hsdram[0], &SDRAM_Timing) != HAL_OK)
    {
        LOG_E("SDRAM init failed!");
        result = -RT_ERROR;
    }
    else
    {
        /* Program the SDRAM external device */
        SDRAM_Initialization_Sequence(&hsdram[0], &command);
        LOG_D("sdram init success, mapped at 0x%X, size is %d bytes, data width is %d", SDRAM_BANK_ADDR, SDRAM_SIZE, SDRAM_DATA_WIDTH);
#ifdef RT_USING_MEMHEAP_AS_HEAP
        /* If RT_USING_MEMHEAP_AS_HEAP is enabled, SDRAM is initialized to the heap */
        rt_memheap_init(&system_heap, "sdram", (void *)SDRAM_BANK_ADDR, SDRAM_SIZE);
#endif
    }

    return result;
}
INIT_BOARD_EXPORT(SDRAM_Init);

#ifdef DRV_DEBUG
#ifdef FINSH_USING_MSH
int sdram_test(void)
{
    int i = 0;
    uint32_t start_time = 0, time_cast = 0;
#if SDRAM_DATA_WIDTH == 8
    char data_width = 1;
    uint8_t data = 0;
#elif SDRAM_DATA_WIDTH == 16
    char data_width = 2;
    uint16_t data = 0;
#else
    char data_width = 4;
    uint32_t data = 0;
#endif

    /* write data */
    LOG_D("Writing the %ld bytes data, waiting....", SDRAM_SIZE);
    start_time = rt_tick_get();
    for (i = 0; i < SDRAM_SIZE / data_width; i++)
    {
#if SDRAM_DATA_WIDTH == 8
        *(__IO uint8_t *)(SDRAM_BANK_ADDR + i * data_width) = (uint8_t)(i % 100);
#elif SDRAM_DATA_WIDTH == 16
        *(__IO uint16_t *)(SDRAM_BANK_ADDR + i * data_width) = (uint16_t)(i % 1000);
#else
        *(__IO uint32_t *)(SDRAM_BANK_ADDR + i * data_width) = (uint32_t)(i % 1000);
#endif
    }
    time_cast = rt_tick_get() - start_time;
    LOG_D("Write data success, total time: %d.%03dS.", time_cast / RT_TICK_PER_SECOND,
          time_cast % RT_TICK_PER_SECOND / ((RT_TICK_PER_SECOND * 1 + 999) / 1000));

    /* read data */
    LOG_D("start Reading and verifying data, waiting....");
    for (i = 0; i < SDRAM_SIZE / data_width; i++)
    {
#if SDRAM_DATA_WIDTH == 8
        data = *(__IO uint8_t *)(SDRAM_BANK_ADDR + i * data_width);
        if (data != i % 100)
        {
            LOG_E("SDRAM test failed!");
            break;
        }
#elif SDRAM_DATA_WIDTH == 16
        data = *(__IO uint16_t *)(SDRAM_BANK_ADDR + i * data_width);
        if (data != i % 1000)
        {
            LOG_E("SDRAM test failed!");
            break;
        }
#else
        data = *(__IO uint32_t *)(SDRAM_BANK_ADDR + i * data_width);
        if (data != i % 1000)
        {
            LOG_E("SDRAM test failed!");
            break;
        }
#endif
    }

    if (i >= SDRAM_SIZE / data_width)
    {
        LOG_D("SDRAM test success!");
    }

    return RT_EOK;
}
void malloc_test(void)
{
    int size = 100 * 1024;  // 100KBytes
    rt_uint8_t * ptr = RT_NULL;

    ptr = rt_malloc(size);
    if(ptr != RT_NULL)
    {
        LOG_D("ptr = %p", ptr); // 打印申请到的空间的首地址
    }
    else
    {
        LOG_E("malloc failed");
    }
}

void sdram_malloc_test(void)
{
    int size = 100 * 1024;  // 100KBytes
    uint8_t *ptr;

    ptr = rt_memheap_alloc(&system_heap, size);
    if(ptr != RT_NULL)
    {
        LOG_D("ptr = %p", ptr); // 打印申请到的空间的首地址
    }
    else
    {
        LOG_E("sdram malloc failed");
    }
}
MSH_CMD_EXPORT(sdram_malloc_test, sdram malloc test)
MSH_CMD_EXPORT(malloc_test, malloc test)

MSH_CMD_EXPORT(sdram_test, sdram test)
#endif /* FINSH_USING_MSH */
#endif /* DRV_DEBUG */
#endif /* BSP_USING_SDRAM */
