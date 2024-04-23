/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_DRIVERS_I2C_I2C_HC32_H_
#define ZEPHYR_DRIVERS_I2C_I2C_HC32_H_

#include <gpio.h>
#include <dma.h>

typedef void (*irq_config_func_t)(struct device *port);

struct i2c_hc32_config {
#ifdef CONFIG_I2C_HC32_INTERRUPT
	irq_config_func_t irq_config_func;
#endif
	const struct hc32_modules_clock_sys *mod_clk;
	CM_I2C_TypeDef *i2c;

	uint32_t bitrate;
#ifdef CONFIG_I2C_HC32_DMA
	uint32_t dma_timeout;
#endif
	uint32_t time_out;
};

struct i2c_hc32_data {
#ifdef CONFIG_I2C_HC32_DMA
	struct device *dma_dev[2];
	struct dma_config *dma_conf;
	uint8_t uints[2];
	uint8_t channel[2];
	const char *name[2];
#endif
#if defined (CONFIG_I2C_HC32_INTERRUPT) || (CONFIG_I2C_HC32_DMA)
	struct k_sem device_sync_sem;
#endif
	struct k_sem bus_mutex;
	uint32_t dev_config;
	uint16_t slaver_addr;
	uint16_t dir;
	uint32_t len;
	uint8_t *dat;
	struct i2c_msg *msg;

#ifdef CONFIG_I2C_SLAVE
	bool master_active;
	struct i2c_slave_config *slave_cfg;
	bool slave_attached;
#endif
};

int32_t hc32_i2c_transaction(struct device *dev,
			    struct i2c_msg msg, uint8_t *next_msg_flags,
			    uint16_t periph);
int i2c_hc32_runtime_configure(struct device *dev, u32_t config);

#endif	/* ZEPHYR_DRIVERS_I2C_I2C_HC32_H_ */
