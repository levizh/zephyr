/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "flash_hc32_qspi.h"

#define DT_DRV_COMPAT xhsc_hc32_qspi
LOG_MODULE_REGISTER(xhsc_hc32_qspi, CONFIG_FLASH_LOG_LEVEL);

void qspi_hc32_command_mode(const struct device *dev, bool flag)
{
	ARG_UNUSED(dev);

	if (true == flag) {
		QSPI_EnterDirectCommMode();
	} else {
		QSPI_ExitDirectCommMode();
	}
}

int qspi_hc32_read_data(const struct device *dev, uint8_t *val)
{
	ARG_UNUSED(dev);
	*val = QSPI_ReadDirectCommValue();
	return 0;
}

int qspi_hc32_write_data(const struct device *dev, uint8_t val)
{
	ARG_UNUSED(dev);
	QSPI_WriteDirectCommValue(val);
	return 0;
}

static int qspi_hc32_clock_enable(const struct device *dev)
{
	int err = 0;
	const struct qspi_hc32_config *config = dev->config;
	struct qspi_hc32_data *data = dev->data;
	const struct device *const clk = DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE);

	/* Get qspi dev_clock node */
	data->dev_clock = clk;

	if (!device_is_ready(data->dev_clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}
	/* Enable qspi clock */
	err = clock_control_on(data->dev_clock,
				(clock_control_subsys_t)config->clk_cfg);
	if (err != 0) {
		LOG_ERR("Could not enable qspi clock");
		return err;
	}
	return 0;
}

int qspi_hc32_init(const struct device *dev)
{
	int err = 0;
	stc_qspi_init_t stcQspiInit;
	const struct qspi_hc32_config *config = dev->config;
	struct qspi_hc32_data *data = dev->data;

	 LOG_INF("Initializing qspi ... ");
	 if (config == NULL || data == NULL) {
	 	LOG_ERR("Wrong parameter");
	 	return -EINVAL;
	 }

	/* Init qspi pointer address in data structure */
	data->dev_qspi = dev;
	/* Enable qspi clock */
	err = qspi_hc32_clock_enable(dev);
	if (err < 0) {
		LOG_ERR("Failed to enable qspi clock");
		return err;
	}
	/* Config qspi pin */
	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("Failed to config qspi pin");
		return err;
	}

	(void)QSPI_StructInit(&stcQspiInit);
	stcQspiInit.u32ClockDiv = (config->qlk_div - 1) << QSPI_CR_DIV_POS;
	stcQspiInit.u32ReadMode = config->mode << QSPI_CR_MDSEL_POS;
	stcQspiInit.u32DummyCycle = config->dummy_cycle << QSPI_FCR_DMCYCN_POS;
	stcQspiInit.u32AddrWidth = config->addr_width;
	stcQspiInit.u32SetupTime = config->cs_set_cycle << QSPI_FCR_SSNLD_POS;
	stcQspiInit.u32ReleaseTime = config->cs_rel_cycle << QSPI_FCR_SSNHD_POS;
	stcQspiInit.u32IntervalTime = QSPI_QSSN_INTERVAL_QSCK2;
	stcQspiInit.u32PrefetchMode   = QSPI_PREFETCH_MD_EDGE_STOP;
	(void)QSPI_Init(&stcQspiInit);

	LOG_INF("Initializing qspi done");

	return err;
}

#define QSPI_HC32_CLOCK_DECL(index)											\
	static const struct hc32_modules_clock_sys qspi_fcg_config_##index[]	\
		= HC32_MODULES_CLOCKS(DT_DRV_INST(index));

#define QSPI_HC32_INIT(index)												\
	QSPI_HC32_CLOCK_DECL(index)												\
	PINCTRL_DT_INST_DEFINE(index);											\
	static const struct qspi_hc32_config qspi_hc32_cfg_##index = {			\
		.base = (CM_QSPI_TypeDef *)DT_INST_REG_ADDR(index),					\
		.clk_cfg = qspi_fcg_config_##index,									\
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),					\
		.qlk_div = DT_INST_PROP(index, clock_divider),						\
		.mode = DT_INST_PROP(index, read_mode),								\
		.addr_width = DT_INST_PROP(index, addr_width),						\
		.cs_set_cycle = DT_INST_PROP(index, cs_setup_time),					\
		.cs_rel_cycle = DT_INST_PROP(index, cs_release_time),				\
		.dummy_cycle = DT_INST_PROP(index, dummy_cycles),					\
	};																		\
	static struct qspi_hc32_data qspi_hc32_data_##index;					\
	DEVICE_DT_INST_DEFINE(index,											\
				&qspi_hc32_init, NULL,										\
				&qspi_hc32_data_##index, &qspi_hc32_cfg_##index,			\
				POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,					\
				NULL);

DT_INST_FOREACH_STATUS_OKAY(QSPI_HC32_INIT)