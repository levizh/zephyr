/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <cortex_m/exc.h>
#include <soc.h>
#include <cmsis_core.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int xhsc_hc32f4_init(void)
{
	LL_PERIPH_WE(LL_PERIPH_ALL);

	/* Enable Flash cache for both instruction and data */
	EFM_CacheCmd(ENABLE);

	return 0;
}

SYS_INIT(xhsc_hc32f4_init, PRE_KERNEL_1, 0);

