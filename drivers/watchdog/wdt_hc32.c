/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_watchdog

#include <zephyr/drivers/watchdog.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys_clock.h>
#include "wdt_hc32.h"

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_hc32);


/* WDT run state */
#define WDT_RUN_STATE_START             1
#define WDT_RUN_STATE_STOP              0

/* WDT clock configuration items */
#define WDT_CLK_CONFIG_NUM              16


static const struct wdt_hc32_data wdt_clk_config[WDT_CLK_CONFIG_NUM] = {
	{WDT_CLK_DIV4,    WDT_CNT_PERIOD256},
	{WDT_CLK_DIV64,   WDT_CNT_PERIOD256},
	{WDT_CLK_DIV128,  WDT_CNT_PERIOD256},
	{WDT_CLK_DIV256,  WDT_CNT_PERIOD256},
	{WDT_CLK_DIV512,  WDT_CNT_PERIOD256},
	{WDT_CLK_DIV1024, WDT_CNT_PERIOD256},
	{WDT_CLK_DIV2048, WDT_CNT_PERIOD256},
	{WDT_CLK_DIV256,  WDT_CNT_PERIOD4096},
	{WDT_CLK_DIV512,  WDT_CNT_PERIOD4096},
	{WDT_CLK_DIV1024, WDT_CNT_PERIOD4096},
	{WDT_CLK_DIV2048, WDT_CNT_PERIOD4096},
	{WDT_CLK_DIV1024, WDT_CNT_PERIOD16384},
	{WDT_CLK_DIV2048, WDT_CNT_PERIOD16384},
	{WDT_CLK_DIV1024, WDT_CNT_PERIOD65536},
	{WDT_CLK_DIV2048, WDT_CNT_PERIOD65536},
	{WDT_CLK_DIV8192, WDT_CNT_PERIOD65536}
};

/**
 * @brief Calculates the timeout in microseconds.
 * @param dev       Pointer to device structure.
 * @param clk_div   clock division factor
 * @param period    The period value.
 * @return The timeout calculated in microseconds.
 */
static uint32_t wdt_hc32_get_timeout(const struct device *dev,
				     uint32_t clk_div, uint32_t period)
{
	uint32_t divVal  = BIT((clk_div) >> WDT_CR_CKS_POS);
	float wdt_freq = (float)CLK_GetBusClockFreq(CLK_BUS_PCLK3) / divVal;
	uint32_t periodVal = 65536;

	ARG_UNUSED(dev);
	switch (period) {
	case WDT_CNT_PERIOD256:
		periodVal = 256;
		break;
	case WDT_CNT_PERIOD4096:
		periodVal = 4096;
		break;
	case WDT_CNT_PERIOD16384:
		periodVal = 16384;
		break;
	case WDT_CNT_PERIOD65536:
		periodVal = 65536;
		break;
	}

	return (uint32_t)(USEC_PER_SEC * (periodVal / wdt_freq));
}

/**
 * @brief Find timeout match configuration.
 * @param dev Pointer to device structure.
 * @param timeout Timeout value in microseconds.
 * @return The index of the best match.
 */
static int wdt_hc32_find_index(const struct device *dev, uint32_t timeout)
{
	int matchIdx;
	uint32_t curVal;
	uint32_t minVal, maxVal;

	ARG_UNUSED(dev);
	minVal = wdt_hc32_get_timeout(dev, wdt_clk_config[0].ClockDiv,
				      wdt_clk_config[0].CountPeriod);
	maxVal = wdt_hc32_get_timeout(dev,
				      wdt_clk_config[WDT_CLK_CONFIG_NUM - 1].ClockDiv,
				      wdt_clk_config[WDT_CLK_CONFIG_NUM - 1].CountPeriod);
	if ((timeout < minVal) || (timeout > maxVal)) {
		return -EINVAL;
	}
	if (timeout == minVal) {
		return 0;
	}

	for (matchIdx = WDT_CLK_CONFIG_NUM - 2; matchIdx >= 0 ; matchIdx--) {
		curVal = wdt_hc32_get_timeout(dev, wdt_clk_config[matchIdx].ClockDiv,
					      wdt_clk_config[matchIdx].CountPeriod);
		if (timeout > curVal) {
			matchIdx += 1;
			break;
		}
	}

	return matchIdx;
}

static int wdt_hc32_setup(const struct device *dev, uint8_t options)
{
	stc_wdt_init_t wdt_init;
	struct wdt_hc32_data *data = WDT_HC32_DATA(dev);

	if (data->RunState != WDT_RUN_STATE_STOP) {
		return -EBUSY;
	}

	/* Deactivate running when debugger is attached. */
	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		DBGC_PeriphCmd(DBGC_PERIPH_WDT, DISABLE);
	}
	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		wdt_init.u32LPMCount    = WDT_LPM_CNT_STOP;
	} else {
		wdt_init.u32LPMCount    = WDT_LPM_CNT_CONTINUE;
	}

	wdt_init.u32CountPeriod     = data->CountPeriod;
	wdt_init.u32ClockDiv        = data->ClockDiv;
	wdt_init.u32RefreshRange    = WDT_RANGE_0TO100PCT;
	wdt_init.u32ExceptionType   = WDT_EXP_TYPE_RST;
	(void)WDT_Init(&wdt_init);
	/* First reload counter to start WDT */
	WDT_FeedDog();
	data->RunState = WDT_RUN_STATE_START;

	return 0;
}

static int wdt_hc32_disable(const struct device *dev)
{
	/* watchdog cannot be stopped once started unless SOC gets a reset */
	ARG_UNUSED(dev);

	return -EPERM;
}

static int wdt_hc32_install_timeout(const struct device *dev,
				    const struct wdt_timeout_cfg *config)
{
	struct wdt_hc32_data *data = WDT_HC32_DATA(dev);
	uint32_t timeout = config->window.max * USEC_PER_MSEC;
	uint32_t calculated_timeout;
	int tabIndex;

	if (config->callback != NULL) {
		return -ENOTSUP;
	}
	if (data->RunState != WDT_RUN_STATE_STOP) {
		return -EBUSY;
	}

	tabIndex = wdt_hc32_find_index(dev, timeout);
	if (tabIndex < 0) {
		/* One of the parameters provided is invalid */
		return -EINVAL;
	}
	calculated_timeout = wdt_hc32_get_timeout(dev,
						  wdt_clk_config[tabIndex].ClockDiv,
						  wdt_clk_config[tabIndex].CountPeriod);
	LOG_DBG("Desired WDT: %d us", timeout);
	LOG_DBG("Set WDT:     %d us", calculated_timeout);

	/* save configuration */
	data->ClockDiv      = wdt_clk_config[tabIndex].ClockDiv;
	data->CountPeriod   = wdt_clk_config[tabIndex].CountPeriod;

	return 0;
}

static int wdt_hc32_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	WDT_FeedDog();

	return 0;
}

static const struct wdt_driver_api wdt_hc32_api = {
	.setup           = wdt_hc32_setup,
	.disable         = wdt_hc32_disable,
	.install_timeout = wdt_hc32_install_timeout,
	.feed            = wdt_hc32_feed,
};

static int wdt_hc32_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static struct wdt_hc32_data wdt_hc32_dev_data = {
	.RunState   = WDT_RUN_STATE_STOP,
};

DEVICE_DT_INST_DEFINE(0, wdt_hc32_init, NULL,
		      &wdt_hc32_dev_data, NULL,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &wdt_hc32_api);
