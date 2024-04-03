/*
* Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
*/
#ifndef _HC32F4_CLOCK_
#define _HC32F4_CLOCK_

#include <autoconf.h>

#if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > 168000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE5
#elif (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > 132000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE4
#elif (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > 99000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE3
#elif (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > 66000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE2
#elif (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > 33000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE1
#else
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE0
#endif

#if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > 126000000)
#define GPIO_RD_WAIT GPIO_RD_WAIT3
#elif (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > 84000000)
#define GPIO_RD_WAIT GPIO_RD_WAIT2
#elif (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > 42000000)
#define GPIO_RD_WAIT GPIO_RD_WAIT1
#else
#define GPIO_RD_WAIT GPIO_RD_WAIT0
#endif

#if CONFIG_HC32_CLK_XTAL
#if ((CONFIG_HC32_XTAL_FREQ > 20000000) && (CONFIG_HC32_XTAL_FREQ <= 25000000))
#define XTAL_DRV	CLK_XTAL_DRV_HIGH
#elif (CONFIG_HC32_XTAL_FREQ > 16000000)
#define XTAL_DRV	CLK_XTAL_DRV_MID
#elif (CONFIG_HC32_XTAL_FREQ > 8000000)
#define XTAL_DRV	CLK_XTAL_DRV_LOW
#elif (CONFIG_HC32_XTAL_FREQ > 4000000)
#define XTAL_DRV	CLK_XTAL_DRV_ULOW
#else
#error "xtal clock frequency not compatible"
#endif
#endif

#endif // !_HC32F4_CLOCK_
