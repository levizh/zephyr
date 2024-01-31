/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <misc/util.h>
#include <pinmux.h>
#include <gpio/gpio_hc32.h>

#define OFFSET_BY_PIN    4u
#define FUNC_MAX         63u
#define PIN_MAX          15u

static int pinmux_set(struct device *dev, u32_t pin, u32_t func)
{
	const struct gpio_hc32_config *cfg = dev->config->config_info;
	u8_t port = cfg->port;

	if ((func > FUNC_MAX)||(pin > PIN_MAX)) {
		return -EINVAL;
	}

	GPIO_SetFunc(port, BIT(pin), func);
	return 0;
}

static int pinmux_get(struct device *dev, u32_t pin, u32_t *func)
{
	const struct gpio_hc32_config *cfg = dev->config->config_info;
	u16_t *PCR_reg = cfg->base;
	u16_t *PFSR_reg = (u16_t *)(PCR_reg + pin * OFFSET_BY_PIN + 2u);

	if (pin > PIN_MAX) {
		return -EINVAL;
	}

	*func = (*PFSR_reg) & GPIO_PFSR_FSEL;
	return 0;
}

static int pinmux_pullup(struct device *dev, u32_t pin, u8_t func)
{
	const struct gpio_hc32_config *cfg = dev->config->config_info;
	u16_t *PCR_reg = cfg->base;

	if (pin > PIN_MAX) {
		return -EINVAL;
	}
	switch (func) {
	case PINMUX_PULLUP_DISABLE:
		CLR_REG16_BIT(*PCR_reg, GPIO_PCR_PUU);
	case PINMUX_PULLUP_ENABLE:
		SET_REG16_BIT(*PCR_reg, GPIO_PCR_PUU);
	}
	return 0;
}

static int pinmux_input(struct device *dev, u32_t pin, u8_t func)
{
const struct gpio_hc32_config *cfg = dev->config->config_info;
	u16_t *PCR_reg = cfg->base;

	if (pin > PIN_MAX) {
		return -EINVAL;
	}
	switch (func) {
	case PINMUX_INPUT_ENABLED:
		CLR_REG16_BIT(*PCR_reg, GPIO_PCR_POUTE);
	case PINMUX_OUTPUT_ENABLED:
		SET_REG16_BIT(*PCR_reg, GPIO_PCR_POUTE);
	}
	return 0;
}

static struct pinmux_driver_api api_funcs = {
	.set = pinmux_set,
	.get = pinmux_get,
	.pullup = pinmux_pullup,
	.input = pinmux_input
};

static int pinmux_initialize(struct device *device)
{
	ARG_UNUSED(device);
	return 0;
}

/* Initialize using PRE_KERNEL_1 priority so that GPIO can use the pin
 * mux driver.
 */
DEVICE_AND_API_INIT(pmux_dev, CONFIG_PINMUX_NAME,
		    &pinmux_initialize, NULL, NULL,
		    PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
