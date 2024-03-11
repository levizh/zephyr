/*
 * Copyright (c) 2016 BayLibre, SAS
 * Copyright (c) 2017 Linaro Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_DRIVERS_I2C_I2C_HC32_H_
#define ZEPHYR_DRIVERS_I2C_I2C_HC32_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/dma.h>

typedef void (*irq_config_func_t)(const struct device *port);

struct i2c_hc32_config {
#ifdef CONFIG_I2C_HC32_INTERRUPT
	irq_config_func_t irq_config_func;
#endif
#ifdef CONFIG_I2C_HC32_DMA
	dma_callback_t dma_callbcck;
#endif
	const struct hc32_modules_clock_sys *mod_clk;
	CM_I2C_TypeDef *i2c;
	uint32_t bitrate;
	const struct pinctrl_dev_config *pcfg;
	struct gpio_dt_spec scl;
	struct gpio_dt_spec sda;
	uint32_t time_out;
};

struct i2c_hc32_data {
#ifdef CONFIG_I2C_HC32_INTERRUPT
	struct k_sem device_sync_sem;
#endif
	struct k_sem bus_mutex;
	uint32_t dev_config;
	uint16_t slaver_addr;
	uint16_t dir;
	uint32_t len;
	uint8_t *dat;
	struct i2c_msg *msg;
};

int32_t hc32_i2c_transaction(const struct device *dev,
			    struct i2c_msg msg, uint8_t *next_msg_flags,
			    uint16_t periph);
int i2c_hc32_runtime_configure(const struct device *dev, uint32_t config);

void hc32_i2c_event_isr(void *arg);
void hc32_i2c_error_isr(void *arg);

#endif	/* ZEPHYR_DRIVERS_I2C_I2C_HC32_H_ */
