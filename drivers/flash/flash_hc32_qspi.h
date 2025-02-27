/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HC32_QSPI_NOR_H
#define HC32_QSPI_NOR_H

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>

/* Config structure definition */
struct qspi_hc32_config {
	CM_QSPI_TypeDef *base;
	const struct pinctrl_dev_config *pin_cfg;
	const struct hc32_modules_clock_sys *clk_cfg;
	uint32_t clk_prescaler;
	uint32_t qlk_div;
	uint32_t mode;
	uint32_t addr_width;
	uint32_t cs_set_cycle;
	uint32_t cs_rel_cycle;
	uint32_t dummy_cycle;
};

/* Data structure definition */
struct qspi_hc32_data {
	const struct device *dev_qspi;
	const struct device *dev_clock;
};

void qspi_hc32_command_mode(const struct device *dev, bool flag);
int qspi_hc32_read_data(const struct device *dev, uint8_t *data);
int qspi_hc32_write_data(const struct device *dev, uint8_t val);

#endif
