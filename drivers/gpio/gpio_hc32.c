/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_gpio

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include "gpio_hc32.h"
#include <zephyr/drivers/gpio/gpio_utils.h>

/**
 * @brief Configure pin or port
 */
static int gpio_hc32_configure(const struct device *port, gpio_pin_t pin,
                            gpio_flags_t flags)
{
    const struct gpio_hc32_config *cfg = port->config;;
    uint8_t hc32_port = cfg->port;
    stc_gpio_init_t stc_gpio_init;

    GPIO_StructInit(&stc_gpio_init);
    /* figure out if we can map the requested GPIO
     * configuration
     */

    if ((flags & GPIO_OUTPUT) != 0U) {
        /* ouput */
        stc_gpio_init.u16PinDir = PIN_DIR_OUT;
        if ((flags & GPIO_SINGLE_ENDED) != 0) {
            if (flags & GPIO_LINE_OPEN_DRAIN) {
                stc_gpio_init.u16PinOutputType = PIN_OUT_TYPE_NMOS;
            } else  {
                /* Output can't be open source */
                return -ENOTSUP;
            }
        } else {
            stc_gpio_init.u16PinOutputType = PIN_OUT_TYPE_CMOS;
        }

        if (((flags & GPIO_PULL_UP) != 0) || ((flags & GPIO_PULL_DOWN) != 0)) {
            /* No pull up or down in out mode*/
            return -ENOTSUP;
        }

        if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
            stc_gpio_init.u16PinState = PIN_STAT_SET;
        } else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
            stc_gpio_init.u16PinState = PIN_STAT_RST;
        } else if ((flags & GPIO_OUTPUT_INIT_LOGICAL) != 0U) {
            /* Don't support logical set */
            return -ENOTSUP;
        }
    } else if ((flags & GPIO_INPUT) != 0) {
        /* Input */
        stc_gpio_init.u16PinDir = PIN_DIR_IN;

        if ((flags & GPIO_PULL_UP) != 0) {
            stc_gpio_init.u16PullUp = PIN_PU_ON;
        } else if ((flags & GPIO_PULL_DOWN) != 0) {
            /* No pull down */
            return -ENOTSUP;
        }
    } else {
        /* Deactivated: Analog */
        stc_gpio_init.u16PinAttr = PIN_ATTR_ANALOG;
    }

    /* TODO:1,int edge(INTC module) */
    GPIO_Init(hc32_port, BIT(pin), &stc_gpio_init);

#if 0
    if ((flags & GPIO_INT_ENABLE) != 0) {
        stc_gpio_init.u16ExtInt = PIN_EXTINT_ON;
    }
#endif
    return 0;
}

static int gpio_hc32_get_config(const struct device *port, gpio_pin_t pin,
                                gpio_flags_t flags)
{
    const struct gpio_hc32_config *cfg = port->config;;
    uint16_t PCR_value = *cfg->base;

    if ((PCR_value & GPIO_PCR_DDIS) == 0U) {
        if (((PCR_value) & GPIO_PCR_POUTE) != 0U) {
            flags |= GPIO_OUTPUT;
        } else {
            flags |= GPIO_INPUT;
            if (((PCR_value) & GPIO_PCR_PUU) != 0U){
                flags |= GPIO_PULL_UP;
            }
        }
    } else {
        flags &= ~(GPIO_OUTPUT | GPIO_INPUT);
    }

    return flags;
}

static int gpio_hc32_port_get_raw(const struct device *dev, uint32_t *value)
{
    const struct gpio_hc32_config *cfg = dev->config;
    uint8_t port = cfg->port;

    *value = (uint32_t)GPIO_ReadInputPort(port);

    return 0;
}

static int gpio_hc32_port_set_masked_raw(const struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
    const struct gpio_hc32_config *cfg = dev->config;
    uint8_t port = cfg->port;
    GPIO_WritePort(port, mask);
     return 0;
}

static int gpio_hc32_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins)
{
    const struct gpio_hc32_config *cfg = dev->config;
    uint8_t port = cfg->port;
    GPIO_SetPins(port, pins);
    return 0;
}

static int gpio_hc32_port_clear_bits_raw(const struct device *dev,
					  gpio_port_pins_t pins)
{
    const struct gpio_hc32_config *cfg = dev->config;
    uint8_t port = cfg->port;
    GPIO_ResetPins(port, pins);
    return 0;
}

static int gpio_hc32_port_toggle_bits(const struct device *dev,
				       gpio_port_pins_t pins)
{
    const struct gpio_hc32_config *cfg = dev->config;
    uint8_t port = cfg->port;
    GPIO_TogglePins(port, pins);
    return 0;
}

static int gpio_hc32_pin_interrupt_configure(const struct device *dev,
					      gpio_pin_t pin,
					      enum gpio_int_mode mode,
					      enum gpio_int_trig trig)
{
    /* FIXME: */
    return 0;
}

static int gpio_hc32_manage_callback(const struct device *dev,
                                     struct gpio_callback *callback,
                                     bool set)
{
    struct gpio_hc32_data *data = dev->data;

    return gpio_manage_callback(&data->cb, callback, set);
}

static uint32_t gpio_hc32_get_pending_int(const struct device *dev)
{
    /* FIXME: */
    return 0;
}

static const struct gpio_driver_api gpio_hc32_driver = {
    .pin_configure = gpio_hc32_configure,
#if defined(CONFIG_GPIO_GET_CONFIG)
    .pin_get_config = gpio_hc32_get_config,
#endif /* CONFIG_GPIO_GET_CONFIG */
    .port_get_raw = gpio_hc32_port_get_raw,
    .port_set_masked_raw = gpio_hc32_port_set_masked_raw,
    .port_set_bits_raw = gpio_hc32_port_set_bits_raw,
    .port_clear_bits_raw = gpio_hc32_port_clear_bits_raw,
    .port_toggle_bits = gpio_hc32_port_toggle_bits,
    .pin_interrupt_configure = gpio_hc32_pin_interrupt_configure,
    .manage_callback = gpio_hc32_manage_callback,
    .get_pending_int = gpio_hc32_get_pending_int,
};


static int gpio_hc32_init(const struct device *dev)
{
    return 0;
}

#define GPIO_HC32_DEFINE(n) \
	static const struct gpio_hc32_config gpio_hc32_cfg_##n = {	\
		.common = {						     \
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),\
		},														\
		.base =	DT_INST_REG_ADDR(n),						\
		.port = n, 			\
	};\
	static struct gpio_hc32_data gpio_hc32_data_##n;	\
	DEVICE_DT_INST_DEFINE(n, gpio_hc32_init, NULL,     \
	        &gpio_hc32_data_##n, &gpio_hc32_cfg_##n,     \
	        POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,     \
	        &gpio_hc32_driver);

DT_INST_FOREACH_STATUS_OKAY(GPIO_HC32_DEFINE)
