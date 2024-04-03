/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <arch/cpu.h>
#include <cortex_m/exc.h>
#include <soc.h>

/**
 * Function Name: System_CLK_Init
 *
 * \brief This function is called by the start-up code for the selected device.
 * It performs all of the necessary device configuration based on the design
 * settings. This includes settings for the platform resources and peripheral
 * clock.
 *
 */
static int System_CLK_Init(struct device *arg)
{
	ARG_UNUSED(arg);

	LL_PERIPH_WE(LL_PERIPH_ALL);

	return 0;
}

SYS_INIT(System_CLK_Init, PRE_KERNEL_1, 0);

