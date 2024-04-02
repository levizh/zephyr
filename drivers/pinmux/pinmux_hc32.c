/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <misc/util.h>
#include <pinmux.h>
#include <soc.h>

#define PINMUX_GPIO_BASE   0x40053c00u
#define PINMUX_GPIO_OFFSET 0x40u
#define OFFSET_BY_PIN      4u
#define FUNC_MAX           63u
#define PIN_MAX            15u

/* func num range 0-63, analog type is 0xff */
#define FUNC_ANALOG  0xFFU
#define FUNC_GPIO    0U


struct pinmux_hc32_config {
	/* port PCRxy register base address */
	uint32_t base;
	/* IO port */
	uint8_t port;
};

static int pinmux_set(struct device *dev, u32_t pin, u32_t func)
{
	const struct pinmux_hc32_config *cfg = dev->config->config_info;
	uint8_t port = cfg->port;

	if (((func > FUNC_MAX) && (func != FUNC_ANALOG)) || (pin > PIN_MAX)) {
		return -EINVAL;
	}
	if (func == FUNC_ANALOG) {
		GPIO_AnalogCmd(port, BIT(pin), ENABLE);
	} else {
		GPIO_AnalogCmd(port, BIT(pin), DISABLE);
		GPIO_SetFunc(port, BIT(pin), func);
	}
	return 0;
}

static int pinmux_get(struct device *dev, u32_t pin, u32_t *func)
{
	const struct pinmux_hc32_config *cfg = dev->config->config_info;
	volatile uint16_t *PCR_reg = (uint16_t *)(cfg->base + pin * OFFSET_BY_PIN);
	volatile uint16_t *PFSR_reg = (uint16_t *)(PCR_reg + 1u);

	if (pin > PIN_MAX) {
		return -EINVAL;
	}
	if (*PCR_reg & GPIO_PCR_DDIS) {
		*func = FUNC_ANALOG;
	} else {
		*func = (*PFSR_reg) & GPIO_PFSR_FSEL;
	}
	return 0;
}

static int pinmux_pullup(struct device *dev, u32_t pin, uint8_t func)
{
	const struct pinmux_hc32_config *cfg = dev->config->config_info;
	volatile uint16_t *PCR_reg = (uint16_t *)(cfg->base + pin * OFFSET_BY_PIN);

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

static int pinmux_input(struct device *dev, u32_t pin, uint8_t func)
{
	const struct pinmux_hc32_config *cfg = dev->config->config_info;
	volatile uint16_t *PCR_reg = (uint16_t *)(cfg->base + pin * OFFSET_BY_PIN);

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
#define PINMUX_HC32_INIT(__SUFFIX, __suffix, num)                                   \
static const struct pinmux_hc32_config pinmux_hc32_port##__suffix##_config = { \
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET * num,        \
	.port = GPIO_PORT_##__SUFFIX,                                            \
};\
DEVICE_AND_API_INIT(pinmux_port##__suffix, CONFIG_PINMUX_HC32_PORT##__SUFFIX##_NAME,\
		    &pinmux_initialize, NULL, &pinmux_hc32_port##__suffix##_config,       \
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                \
		    &api_funcs);


/* GPIOA */
PINMUX_HC32_INIT(A, a, 0);
/* GPIOB */
PINMUX_HC32_INIT(B, b, 1);
/* GPIOC */
PINMUX_HC32_INIT(C, c, 2);
/* GPIOD */
PINMUX_HC32_INIT(D, d, 3);
/* GPIOE */
PINMUX_HC32_INIT(E, e, 4);

#if defined(CONFIG_SOC_HC32F460)
/* GPIOH */
PINMUX_HC32_INIT(H, h, 5);

#elif defined (CONFIG_SOC_HC32F4A0)
/* GPIOF */
PINMUX_HC32_INIT(F, f ,5);
/* GPIOG */
PINMUX_HC32_INIT(G, g, 6);
/* GPIOH */
PINMUX_HC32_INIT(H, h, 7);
/* GPIOI */
PINMUX_HC32_INIT(I, i, 8);

#endif /* CONFIG_SOC_HC32F460 */

