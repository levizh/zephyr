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
#include <interrupt_controller/intc_hc32.h>
#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_hc32);

/**
 * @brief Common GPIO driver for HC32 MCUs.
 */
#define GPIO_INT_EDGE_RISING           ( GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)
#define GPIO_INT_EDGE_FALLING          ( GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW )
#define GPIO_INT_EDGE_BOTH             ( GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE)
#define GPIO_INT_LEVEL_LOW             ( GPIO_INT_LEVEL| GPIO_INT_ACTIVE_LOW )
#define GPIO_INT_MASK                  ( GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH | GPIO_INT_DOUBLE_EDGE)

#if defined (HC32F460) || defined (HC32F4A0)
#define GPIO_PIN_NUM_MAX      (16U)
#endif

/**
 * @brief EXTI interrupt callback
 */
static void gpio_hc32_isr(int line, void *arg)
{
	struct device *dev = arg;
	struct gpio_hc32_data *data = dev->driver_data;
	if (BIT(line) & (data->cb_pins)) {
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

	if (pin >= GPIO_PIN_NUM_MAX) {
		LOG_ERR("pin out range 0-15");
		return -ENOTSUP;
	}

	GPIO_StructInit(&stc_gpio_init);
	if ((flags & GPIO_DIR_MASK) != 0) {
		stc_gpio_init.u16PinDir = PIN_DIR_OUT;
		if (flags & GPIO_PUD_MASK) {
			/* No pull up or down in out mode*/
			return -ENOTSUP;
		}
	} else {
		/* Input */
		stc_gpio_init.u16PinDir = PIN_DIR_IN;
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			stc_gpio_init.u16PullUp = PIN_PU_ON;
		} else if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
			return -ENOTSUP;
		}
	}

	if ((flags & GPIO_POL_MASK) != 0) {
		stc_gpio_init.u16Invert = PIN_INVT_ON;
	}

	if ((flags & GPIO_INT) != 0) {
		hc32_extint_disable(port, pin);
		stc_gpio_init.u16ExtInt = PIN_EXTINT_OFF;
		stc_extint_init_t stcExtIntInit;
		EXTINT_StructInit(&stcExtIntInit);

		if ((flags & GPIO_INT_MASK) == GPIO_INT_EDGE_BOTH) {
			stcExtIntInit.u32Edge = EXTINT_TRIG_BOTH;
		} else if ((flags & GPIO_INT_MASK) == GPIO_INT_EDGE_RISING) {
			stcExtIntInit.u32Edge = EXTINT_TRIG_RISING;
		} else if ((flags & GPIO_INT_MASK) == GPIO_INT_EDGE_FALLING) {
			stcExtIntInit.u32Edge = EXTINT_TRIG_FALLING;
		} else if ((flags & GPIO_INT_MASK) == GPIO_INT_LEVEL_LOW) {
			stcExtIntInit.u32Edge = EXTINT_TRIG_LOW;
		} else {
			/* Not support hight level set and others for int */
			return -ENOTSUP;
		}

		if (flags & GPIO_INT_DEBOUNCE) {
			stcExtIntInit.u32Filter = EXTINT_FILTER_ON;
		}
		EXTINT_Init(BIT(pin), &stcExtIntInit);
		if (hc32_extint_set_callback(pin, gpio_hc32_isr, dev)) {
			return -EBUSY;
		}
	} else {
		hc32_extint_disable(port, pin);
	}

	/* pin range:0-15 */
	return GPIO_Init(port, BIT(pin), &stc_gpio_init);
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
	int ret;
	ret = gpio_manage_callback(&data->cb, callback, set);

	if ((false == set) && sys_slist_is_empty(&data->cb)) {
		for (uint8_t u8PinPos = 0U; u8PinPos < GPIO_PIN_NUM_MAX; u8PinPos++) {
			if (data->cb_pins & BIT(u8PinPos)) {
				hc32_extint_unset_callback(u8PinPos);
			}
		}
	}
	return ret;
}

static int gpio_hc32_enable_callback(struct device *dev,
				     int access_op, u32_t pin)
{
	struct gpio_hc32_data *data = dev->driver_data;
	const struct gpio_hc32_config *cfg = dev->config->config_info;
	u8_t port = cfg->port;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	if (hc32_extint_enable(port, pin)) {
		return -EBUSY;
	}
	data->cb_pins |= BIT(pin);
	return 0;
}

static int gpio_hc32_disable_callback(struct device *dev,
				      int access_op, u32_t pin)
{
	struct gpio_hc32_data *data = dev->driver_data;
	const struct gpio_hc32_config *cfg = dev->config->config_info;
	u8_t port = cfg->port;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins &= ~BIT(pin);
	hc32_extint_disable(port, pin);

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
 * @param dev GPIO device struct
 *
 * @return 0
 */
static int gpio_hc32_init(struct device *dev)
{
	return 0;
}

#define GPIO_DEVICE_INIT(__name, __suffix, __base_addr, __port)  \
static const struct gpio_hc32_config gpio_hc32_cfg_##__suffix = {\
    .base = (u16_t *)__base_addr,   \
    .port = __port,                 \
};                                  \
static struct gpio_hc32_data gpio_hc32_data_##__suffix;     \
DEVICE_AND_API_INIT(gpio_hc32_##__suffix,                   \
            __name,                                         \
            gpio_hc32_init,                                 \
            &gpio_hc32_data_##__suffix,                     \
            &gpio_hc32_cfg_##__suffix,                      \
            POST_KERNEL,                                    \
            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,            \
            &gpio_hc32_driver);

#define GPIO_DEVICE_INIT_HC32(__suffix, __SUFFIX, __numb)   \
    GPIO_DEVICE_INIT(DT_XHSC_HC32_GPIO_##__numb##_LABEL,    \
            __suffix,                                       \
            DT_XHSC_HC32_GPIO_##__numb##_BASE_ADDRESS,      \
            GPIO_PORT_##__SUFFIX                    )

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
