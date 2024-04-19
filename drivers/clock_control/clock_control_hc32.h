/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _HC32F4_CLOCK_
#define _HC32F4_CLOCK_

#include <zephyr/devicetree.h>
#include <autoconf.h>


#if (CORE_CLK_FREQ > 168000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE5
#elif (CORE_CLK_FREQ > 132000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE4
#elif (CORE_CLK_FREQ > 99000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE3
#elif (CORE_CLK_FREQ > 66000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE2
#elif (CORE_CLK_FREQ > 33000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE1
#else
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE0
#endif

#if (CORE_CLK_FREQ > 126000000)
#define GPIO_RD_WAIT GPIO_RD_WAIT3
#elif (CORE_CLK_FREQ > 84000000)
#define GPIO_RD_WAIT GPIO_RD_WAIT2
#elif (CORE_CLK_FREQ > 42000000)
#define GPIO_RD_WAIT GPIO_RD_WAIT1
#else
#define GPIO_RD_WAIT GPIO_RD_WAIT0
#endif

#if HC32_XTAL_ENABLED
#if ((HC32_XTAL_FREQ > 20000000) && (HC32_XTAL_FREQ <= 25000000))
#define XTAL_DRV	CLK_XTAL_DRV_HIGH
#elif (HC32_XTAL_FREQ > 16000000)
#define XTAL_DRV	CLK_XTAL_DRV_MID
#elif (HC32_XTAL_FREQ > 8000000)
#define XTAL_DRV	CLK_XTAL_DRV_LOW
#elif (HC32_XTAL_FREQ > 4000000)
#define XTAL_DRV	CLK_XTAL_DRV_ULOW
#else
#error "xtal clock frequency not compatible"
#endif
#endif

#define HC32_CLOCK_CONTROL_NAME "hc32-cc"

struct hc32_modules_clock_config {
	uint32_t addr;
};
#endif // !_HC32F4_CLOCK_
