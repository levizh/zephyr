/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <misc/util.h>

#include "gpio_hc32.h"
#include "gpio_utils.h"

/**
 * @brief Common GPIO driver for HC32 MCUs.
 */

/**
 * @brief EXTI interrupt callback
 */
static void gpio_hc32_isr(int line, void *arg)
{
	struct device *dev = arg;
	struct gpio_hc32_data *data = dev->driver_data;

	if ((BIT(line) & data->cb_pins) != 0) {
		gpio_fire_callbacks(&data->cb, dev, BIT(line));
	}
}

/**
 * @brief Configure pin or port
 */
static int gpio_hc32_config(struct device *dev, int access_op,
				u32_t pin, int flags)
{
	const struct gpio_hc32_config *cfg = dev->config->config_info;
	u8_t port = cfg->port;
	stc_gpio_init_t stc_gpio_init;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	GPIO_StructInit(&stc_gpio_init);
	/* figure out if we can map the requested GPIO
	 * configuration
	 */

	if((flags & GPIO_DIR_MASK) != 0)
	{
		stc_gpio_init.u16PinDir = PIN_DIR_OUT;
	}
		
	if((flags & GPIO_INT) != 0)
	{
		stc_gpio_init.u16ExtInt = PIN_EXTINT_ON;
	}

	if((flags & GPIO_POL_MASK) != 0)
	{
		stc_gpio_init.u16Invert = PIN_INVT_ON;
	}

	if((flags & GPIO_PUD_MASK) == 1)
	{
		stc_gpio_init.u16PullUp = PIN_PU_ON;
	}
	else if((flags & GPIO_PUD_MASK) == 2)
	{
		return -ENOTSUP;
	}

	/* TODO:1,int edge(INTC module), 2,driver strength select */
	GPIO_Init(port, BIT(pin), &stc_gpio_init);

#if 0
	if ((flags & GPIO_INT) != 0) {

		} else {
			/* Level trigger interrupts not supported */
			return -ENOTSUP;
		}

	}
#endif
	return 0;
}

/**
 * @brief Set the pin or port output
 */
static int gpio_hc32_write(struct device *dev, int access_op,
				u32_t pin, u32_t value)
{
	const struct gpio_hc32_config *cfg = dev->config->config_info;
	u8_t port = cfg->port;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	if (value != 0U) {
		GPIO_SetPins(port, BIT(pin));
	} else {
		GPIO_ResetPins(port, BIT(pin));
	}

	return 0;
}

/**
 * @brief Read the pin or port status
 */
static int gpio_hc32_read(struct device *dev, int access_op,
				u32_t pin, u32_t *value)
{
	const struct gpio_hc32_config *cfg = dev->config->config_info;
	u8_t port = cfg->port;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	*value = (u32_t)GPIO_ReadInputPins(port, BIT(pin));
	
	return 0;
}

static int gpio_hc32_manage_callback(struct device *dev,
					struct gpio_callback *callback,
					bool set)
{
	struct gpio_hc32_data *data = dev->driver_data;

	return gpio_manage_callback(&data->cb, callback, set);
}

static int gpio_hc32_enable_callback(struct device *dev,
					int access_op, u32_t pin)
{
	struct gpio_hc32_data *data = dev->driver_data;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins |= BIT(pin);

	return 0;
}

static int gpio_hc32_disable_callback(struct device *dev,
					int access_op, u32_t pin)
{
	struct gpio_hc32_data *data = dev->driver_data;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins &= ~BIT(pin);

	return 0;
}

static const struct gpio_driver_api gpio_hc32_driver = {
	.config = gpio_hc32_config,
	.write = gpio_hc32_write,
	.read = gpio_hc32_read,
	.manage_callback = gpio_hc32_manage_callback,
	.enable_callback = gpio_hc32_enable_callback,
	.disable_callback = gpio_hc32_disable_callback,

};

/**
 * @brief Initialize GPIO port
 *
 * Perform basic initialization of a GPIO port. The code will
 * enable the clock for corresponding peripheral.
 *
 * @param dev GPIO device struct
 *
 * @return 0
 */
static int gpio_hc32_init(struct device *device)
{
	return 0;
}

#define GPIO_DEVICE_INIT(__name, __suffix, __base_addr, __port) \
static const struct gpio_hc32_config gpio_hc32_cfg_##__suffix = {\
	.base = (u16_t *)__base_addr,	\
	.port = __port,					\
};									\
static struct gpio_hc32_data gpio_hc32_data_##__suffix;		\
DEVICE_AND_API_INIT(gpio_hc32_##__suffix,				\
			__name,						\
			gpio_hc32_init,					\
			&gpio_hc32_data_##__suffix,			\
			&gpio_hc32_cfg_##__suffix,			\
			POST_KERNEL,					\
			CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,			\
			&gpio_hc32_driver);

#define GPIO_DEVICE_INIT_HC32(__suffix, __SUFFIX, __numb)		\
	GPIO_DEVICE_INIT(DT_XHSC_HC32_GPIO_##__numb##_LABEL,		\
			__suffix,						\
			DT_XHSC_HC32_GPIO_##__numb##_BASE_ADDRESS, 	\
			GPIO_PORT_##__SUFFIX					)

#ifdef CONFIG_GPIO_HC32
GPIO_DEVICE_INIT_HC32(a, A, 0);
GPIO_DEVICE_INIT_HC32(b, B, 1);
GPIO_DEVICE_INIT_HC32(c, C, 2);
GPIO_DEVICE_INIT_HC32(d, D, 3);
GPIO_DEVICE_INIT_HC32(e, E, 4);
#if defined(CONFIG_SOC_HC32F460)
GPIO_DEVICE_INIT_HC32(h, H, 5);
#elif defined (CONFIG_SOC_HC32F4A0)
GPIO_DEVICE_INIT_HC32(f, F, 5);
GPIO_DEVICE_INIT_HC32(g, G, 6);
GPIO_DEVICE_INIT_HC32(h, H, 7);
GPIO_DEVICE_INIT_HC32(i, I, 8);
#endif
#endif /* CONFIG_GPIO_HC32 */
