/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <hc32_ll.h>

/**
 * @brief Array containing pointers to each GPIO port.
 *
 * Entries will be NULL if the GPIO port is not enabled.
 */
static const struct device *const gpio_ports[] = {
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioa)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiob)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioc)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiod)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioe)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiof)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiog)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioh)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioi)),
};

/** Number of GPIO ports. */
static const size_t gpio_ports_cnt = ARRAY_SIZE(gpio_ports);

static inline bool HC32_pin_is_valid(uint16_t pin)
{
	return ((BIT(pin) & GPIO_PIN_ALL) != 0);
}

static int hc32_pin_configure(const uint32_t pin_mux, const uint32_t pin_cfg)
{
	uint8_t port_num = HC32_PORT(pin_mux);
	uint16_t pin_num = HC32_PIN(pin_mux);
	uint8_t mode = HC32_MODE(pin_mux);
	uint16_t func_num = HC32_FUNC_NUM(pin_mux);
	stc_gpio_init_t stc_gpio_init;

	GPIO_StructInit(&stc_gpio_init);

	if ((port_num >= gpio_ports_cnt) || (HC32_pin_is_valid(pin_num))) {
		return -EINVAL;
	}

	switch (mode) {
	case HC32_ANALOG:
		stc_gpio_init.u16PinAttr = PIN_ATTR_ANALOG;
		goto end;
		break;

	case HC32_GPIO:
		func_num = 0;
		stc_gpio_init.u16PinAttr = PIN_ATTR_DIGITAL;
		if (HC32_INPUT_ENABLE == HC32_PIN_EN_DIR(pin_cfg)) {
			/* input */
			stc_gpio_init.u16PinDir = PIN_DIR_IN;
		} else {
			/* output */
			stc_gpio_init.u16PinDir = PIN_DIR_OUT;
			if (HC32_OUTPUT_HIGH == HC32_OUT_LEVEL(pin_cfg)) {
				stc_gpio_init.u16PinState = PIN_STAT_SET;
			}
		}
		break;

	case HC32_FUNC:
		GPIO_SetFunc(port_num, pin_num, func_num);
		break;

	case HC32_SUBFUNC:
		GPIO_SetSubFunc(func_num);
		GPIO_SubFuncCmd(port_num, pin_num, ENABLE);
		break;

	default:
		break;
	}

	if (HC32_PULL_UP == HC32_PIN_BIAS(pin_cfg)) {
		stc_gpio_init.u16PullUp = PIN_PU_ON;
	}
	if (HC32_PUSH_PULL == HC32_PIN_DRV(pin_cfg)) {
		stc_gpio_init.u16PinOutputType = PIN_OUT_TYPE_CMOS;
	} else {
		stc_gpio_init.u16PinOutputType = PIN_OUT_TYPE_NMOS;
	}

	switch (HC32_PIN_DRIVER_STRENGTH(pin_cfg)) {
	case HC32_DRIVER_STRENGTH_LOW:
		stc_gpio_init.u16PinDrv = PIN_LOW_DRV;
		break;

	case HC32_DRIVER_STRENGTH_MEDIUM:
		stc_gpio_init.u16PinDrv = PIN_MID_DRV;
		break;

	case HC32_DRIVER_STRENGTH_HIGH:
		stc_gpio_init.u16PinDrv = PIN_HIGH_DRV;
		break;

	default:
		break;
	}

end:
	return GPIO_Init(port_num, BIT(pin_num), &stc_gpio_init);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	uint32_t pin_mux, pin_cfg;
	int ret = 0;

	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pin_mux = pins[i].pinmux;
		pin_cfg = pins[i].pincfg;

		ret = hc32_pin_configure(pin_mux, pin_cfg);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}
