/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_HC32_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_HC32_H_

/**
 * @file header for HC32 GPIO
 */

#include <gpio.h>

/**
 * @brief configuration of GPIO device
 */
struct gpio_hc32_config {
	/* port PCRxy register base address, TODO:check if used */
	u16_t *base;
	/* IO port */
	u8_t port;
};

/**
 * @brief driver data
 */
struct gpio_hc32_data {
	/* Enabled INT pins generating a cb */
	u32_t cb_pins;
	/* user ISR cb */
	sys_slist_t cb;
};

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_HC32_H_ */
