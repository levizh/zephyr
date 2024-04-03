/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HC32_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HC32_CLOCK_CONTROL_H_

#include "soc.h"
#include <clock_control.h>
#include <dt-bindings/clock/hc32f4_clock.h>

#define HC32_HCLK_DIV_FN(v)		CLK_HCLK_DIV##v
#define HC32_HCLK_DIV(v)		HC32_HCLK_DIV_FN(v)
#define HC32_EXCLK_DIV_FN(v)	CLK_EXCLK_DIV##v
#define HC32_EXCLK_DIV(v)		HC32_EXCLK_DIV_FN(v)
#define HC32_PCLK_FN(n,v)		CLK_PCLK##n##_DIV##v	
#define HC32_PCLK(n,v)			HC32_PCLK_FN(n,v)

#define HC32_CLK_MODULES_BIT(bit)	(1UL << (bit))
#define HC32_CLK_MODULES_OFFSET(n)	(4 * n)

#define CORE_CLK_FREQ	CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
#define SYS_CLK_FREQ	CORE_CLK_FREQ * CONFIG_HC32_BUS_HCLK_DIV
#define PCLK0_FREQ		SYS_CLK_FREQ / CONFIG_HC32_BUS_PCLK0_DIV
#define PCLK1_FREQ		SYS_CLK_FREQ / CONFIG_HC32_BUS_PCLK1_DIV
#define PCLK2_FREQ		SYS_CLK_FREQ / CONFIG_HC32_BUS_PCLK2_DIV
#define PCLK3_FREQ		SYS_CLK_FREQ / CONFIG_HC32_BUS_PCLK3_DIV
#define PCLK4_FREQ		SYS_CLK_FREQ / CONFIG_HC32_BUS_PCLK4_DIV

#define HC32_CLOCK_CONTROL_NAME "hc32-cc"
#define CLOCK_CONTROL	HC32_CLOCK_CONTROL_NAME

#define HC32_MODULES_CLOCK_INFO(clk_index, node_id)				\
	{								\
	.bus = DT_##node_id##_CLOCK_BUS_##clk_index,	\
	.fcg = DT_##node_id##_CLOCK_FCG_##clk_index,		\
	.bits = DT_##node_id##_CLOCK_BITS_##clk_index		\
	},

#define NODE_ID_PASTE(compatible, inst)	compatible##_##inst

/**
 * get peripherals clock info for device tree
 * @arg inst: the instance serial number of compatible
 * @arg compatible: device tree compatible.must be uppercase when invoked 
 * @arg num: the number of clocks properties in peripheral node
 * example:
 * HC32_MODULES_CLOCKS(3,XHSC_HC32_GPIO, 2);
 * The above line will generate the below:
 * {
 * 		{
 * 			.bus = DT_XHSC_HC32_GPIO_3_CLOCK_BUS_0,
 * 			.fcg = DT_XHSC_HC32_GPIO_3_CLOCK_FCG_0,
 *			.bits = DT_XHSC_HC32_GPIO_3_CLOCK_BITS_0
 *		},
 *		{
 *			.bus = DT_XHSC_HC32_GPIO_3_CLOCK_BUS_1,
 *			.fcg = DT_XHSC_HC32_GPIO_3_CLOCK_FCG_1,
 *			.bits = DT_XHSC_HC32_GPIO_3_CLOCK_BITS_1
 *		}
 * }
 */
#define HC32_MODULES_CLOCKS(inst,compatible, num)					\
	{								\
		UTIL_LISTIFY(num,				\
			HC32_MODULES_CLOCK_INFO, NODE_ID_PASTE(compatible, inst))		\
	}

struct hc32_modules_clock_sys {
	uint32_t bus;
	uint32_t fcg;
	uint32_t bits;
};

struct hc32_modules_clock_config {
	uint32_t addr;
};
#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HC32_CLOCK_CONTROL_H_ */
