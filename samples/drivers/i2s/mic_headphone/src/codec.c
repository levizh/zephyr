/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "codec.h"
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>

#if DT_ON_BUS(WM8731_NODE, i2c)

#define WM8731_I2C_NODE DT_BUS(WM8731_NODE)
#define WM8731_I2C_ADDR DT_REG_ADDR(WM8731_NODE)

bool init_wm8731_i2c(void)
{
	const struct device *const i2c_dev = DEVICE_DT_GET(WM8731_I2C_NODE);

	/* Initialization data for WM8731 registers. */
	static const uint8_t init[][2] = {
		{ 0x1E, 0x00 },
		
		{ 0x0C, 0x11 },

		{ 0x08, 0x15 },

		{ 0x10, 0x18 },

		{ 0x04, 0x6F },

		{ 0x06, 0x6F },

		{ 0x0E, 0x0E },

		{ 0x12, 0x01 },

		{ 0x0C, 0x01 },

		{ 0x0A, 0x00 }
	};

	if (!device_is_ready(i2c_dev)) {
		printk("%s is not ready\n", i2c_dev->name);
		return false;
	}

	for (int i = 0; i < ARRAY_SIZE(init); ++i) {
		const uint8_t *entry = init[i];
		int ret;

		ret = i2c_reg_write_byte(i2c_dev, WM8731_I2C_ADDR,
					 entry[0], entry[1]);
		if (ret < 0) {
			printk("Initialization step %d failed\n", i);
			return false;
		}
	}

	return true;
}

#endif /* DT_ON_BUS(WM8731_NODE, i2c) */
