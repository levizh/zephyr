/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_specialized_watchdog

#include <zephyr/drivers/watchdog.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys_clock.h>
#include "wdt_hc32_swdt.h"

#define LOG_LEVEL CONFIG_SWDT_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_hc32_swdt);


/* SWDT run state */
#define SWDT_RUN_STATE_START            1
#define SWDT_RUN_STATE_STOP             0

/* SWDT clock configuration items */
#define SWDT_CLK_CONFIG_NUM             16
#define SWDT_LRC_CLK_FREQ               10000


static const struct swdt_hc32_data swdt_clk_config[SWDT_CLK_CONFIG_NUM] = {
	{SWDT_CLK_DIV1,    SWDT_CNT_PERIOD256},
	{SWDT_CLK_DIV16,   SWDT_CNT_PERIOD256},
	{SWDT_CLK_DIV32,   SWDT_CNT_PERIOD256},
	{SWDT_CLK_DIV64,   SWDT_CNT_PERIOD256},
	{SWDT_CLK_DIV128,  SWDT_CNT_PERIOD256},
	{SWDT_CLK_DIV256,  SWDT_CNT_PERIOD256},
	{SWDT_CLK_DIV32,   SWDT_CNT_PERIOD4096},
	{SWDT_CLK_DIV64,   SWDT_CNT_PERIOD4096},
	{SWDT_CLK_DIV128,  SWDT_CNT_PERIOD4096},
	{SWDT_CLK_DIV256,  SWDT_CNT_PERIOD4096},
	{SWDT_CLK_DIV128,  SWDT_CNT_PERIOD16384},
	{SWDT_CLK_DIV256,  SWDT_CNT_PERIOD16384},
	{SWDT_CLK_DIV2048, SWDT_CNT_PERIOD4096},
	{SWDT_CLK_DIV256,  SWDT_CNT_PERIOD65536},
	{SWDT_CLK_DIV2048, SWDT_CNT_PERIOD16384},
	{SWDT_CLK_DIV2048, SWDT_CNT_PERIOD65536}
};

/**
 * @brief Calculates the timeout in microseconds.
 * @param dev       Pointer to device structure.
 * @param clk_div   clock division factor
 * @param period    The period value.
 * @return The timeout calculated in microseconds.
 */
static uint32_t swdt_hc32_get_timeout(const struct device *dev,
				      uint32_t clk_div, uint32_t period)
{
	uint32_t divVal  = BIT((clk_div) >> SWDT_CR_CKS_POS);
	float swdt_freq = (float)SWDT_LRC_CLK_FREQ / divVal;
	uint32_t periodVal = 65536;

	ARG_UNUSED(dev);
	switch (period) {
	case SWDT_CNT_PERIOD256:
		periodVal = 256;
		break;
	case SWDT_CNT_PERIOD4096:
		periodVal = 4096;
		break;
	case SWDT_CNT_PERIOD16384:
		periodVal = 16384;
		break;
	case SWDT_CNT_PERIOD65536:
		periodVal = 65536;
		break;
	}

	return (uint32_t)(USEC_PER_SEC * (periodVal / swdt_freq));
}

/**
 * @brief Find timeout match configuration.
 * @param dev Pointer to device structure.
 * @param timeout Timeout value in microseconds.
 * @return The index of the best match.
 */
static int swdt_hc32_find_index(const struct device *dev, uint32_t timeout)
{
	int matchIdx;
	uint32_t curVal;
	uint32_t minVal, maxVal;

	ARG_UNUSED(dev);
	minVal = swdt_hc32_get_timeout(dev, swdt_clk_config[0].ClockDiv,
				       swdt_clk_config[0].CountPeriod);
	maxVal = swdt_hc32_get_timeout(dev,
				       swdt_clk_config[SWDT_CLK_CONFIG_NUM - 1].ClockDiv,
				       swdt_clk_config[SWDT_CLK_CONFIG_NUM - 1].CountPeriod);
	if ((timeout < minVal) || (timeout > maxVal)) {
		return -EINVAL;
	}
	if (timeout == minVal) {
		return 0;
	}

	for (matchIdx = SWDT_CLK_CONFIG_NUM - 2; matchIdx >= 0 ; matchIdx--) {
		curVal = swdt_hc32_get_timeout(dev, swdt_clk_config[matchIdx].ClockDiv,
					       swdt_clk_config[matchIdx].CountPeriod);
		if (timeout > curVal) {
			matchIdx += 1;
			break;
		}
	}

	return matchIdx;
}

static int swdt_hc32_setup(const struct device *dev, uint8_t options)
{
	stc_swdt_init_t swdt_init;
	struct swdt_hc32_data *data = SWDT_HC32_DATA(dev);

	if (data->RunState != SWDT_RUN_STATE_STOP) {
		return -EBUSY;
	}

	/* Deactivate running when debugger is attached. */
	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		DBGC_PeriphCmd(DBGC_PERIPH_SWDT, DISABLE);
	}
	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		swdt_init.u32LPMCount    = SWDT_LPM_CNT_STOP;
	} else {
		swdt_init.u32LPMCount    = SWDT_LPM_CNT_CONTINUE;
	}

	swdt_init.u32CountPeriod     = data->CountPeriod;
	swdt_init.u32ClockDiv        = data->ClockDiv;
	swdt_init.u32RefreshRange    = SWDT_RANGE_0TO100PCT;
	swdt_init.u32ExceptionType   = SWDT_EXP_TYPE_RST;
	(void)SWDT_Init(&swdt_init);
	/* First reload counter to start SWDT */
	SWDT_FeedDog();
	data->RunState = SWDT_RUN_STATE_START;

	return 0;
}

static int swdt_hc32_disable(const struct device *dev)
{
	/* watchdog cannot be stopped once started unless SOC gets a reset */
	ARG_UNUSED(dev);

	return -EPERM;
}

static int swdt_hc32_install_timeout(const struct device *dev,
				     const struct wdt_timeout_cfg *config)
{
	struct swdt_hc32_data *data = SWDT_HC32_DATA(dev);
	uint32_t timeout = config->window.max * USEC_PER_MSEC;
	uint32_t calculated_timeout;
	int tabIndex;

	if (config->callback != NULL) {
		return -ENOTSUP;
	}
	if (data->RunState != SWDT_RUN_STATE_STOP) {
		return -EBUSY;
	}

	tabIndex = swdt_hc32_find_index(dev, timeout);
	if (tabIndex < 0) {
		/* One of the parameters provided is invalid */
		return -EINVAL;
	}
	calculated_timeout = swdt_hc32_get_timeout(dev,
						   swdt_clk_config[tabIndex].ClockDiv,
						   swdt_clk_config[tabIndex].CountPeriod);
	LOG_DBG("Desired SWDT: %d us", timeout);
	LOG_DBG("Set SWDT:     %d us", calculated_timeout);

	/* save configuration */
	data->ClockDiv      = swdt_clk_config[tabIndex].ClockDiv;
	data->CountPeriod   = swdt_clk_config[tabIndex].CountPeriod;

	return 0;
}

static int swdt_hc32_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	SWDT_FeedDog();

	return 0;
}

static const struct wdt_driver_api swdt_hc32_api = {
	.setup           = swdt_hc32_setup,
	.disable         = swdt_hc32_disable,
	.install_timeout = swdt_hc32_install_timeout,
	.feed            = swdt_hc32_feed,
};

static int swdt_hc32_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static struct swdt_hc32_data swdt_hc32_dev_data = {
	.RunState   = SWDT_RUN_STATE_STOP,
};

DEVICE_DT_INST_DEFINE(0, swdt_hc32_init, NULL,
		      &swdt_hc32_dev_data, NULL,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &swdt_hc32_api);
