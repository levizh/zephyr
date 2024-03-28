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
/* GPIOA */
#ifdef CONFIG_PINMUX_HC32_PORTA_NAME
static const struct pinmux_hc32_config pinmux_hc32_porta_config = {
	.base = PINMUX_GPIO_BASE,
	.port = GPIO_PORT_A,
};
DEVICE_AND_API_INIT(pinmux_porta, CONFIG_PINMUX_HC32_PORTA_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_porta_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#ifdef CONFIG_PINMUX_HC32_PORTB_NAME
/* GPIOB */
static const struct pinmux_hc32_config pinmux_hc32_portb_config = {
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET,
	.port = GPIO_PORT_B,
};
DEVICE_AND_API_INIT(pinmux_portb, CONFIG_PINMUX_HC32_PORTB_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_portb_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#ifdef CONFIG_PINMUX_HC32_PORTC_NAME
/* GPIOC */
static const struct pinmux_hc32_config pinmux_hc32_portc_config = {
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET * 2u,
	.port = GPIO_PORT_C,
};
DEVICE_AND_API_INIT(pinmux_portc, CONFIG_PINMUX_HC32_PORTC_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_portc_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#ifdef CONFIG_PINMUX_HC32_PORTD_NAME
/* GPIOD */
static const struct pinmux_hc32_config pinmux_hc32_portd_config = {
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET * 3u,
	.port = GPIO_PORT_D,
};
DEVICE_AND_API_INIT(pinmux_portd, CONFIG_PINMUX_HC32_PORTD_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_portd_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#ifdef CONFIG_PINMUX_HC32_PORTE_NAME
/* GPIOE */
static const struct pinmux_hc32_config pinmux_hc32_porte_config = {
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET * 4u,
	.port = GPIO_PORT_E,
};
DEVICE_AND_API_INIT(pinmux_porte, CONFIG_PINMUX_HC32_PORTE_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_porte_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#if defined(CONFIG_SOC_HC32F460)
#ifdef CONFIG_PINMUX_HC32_PORTH_NAME
/* GPIOH */
static const struct pinmux_hc32_config pinmux_hc32_porth_config = {
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET * 5u,
	.port = GPIO_PORT_H,
};
DEVICE_AND_API_INIT(pinmux_porth, CONFIG_PINMUX_HC32_PORTH_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_porth_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#elif defined (CONFIG_SOC_HC32F4A0)
#ifdef CONFIG_PINMUX_HC32_PORTF_NAME
/* GPIOF */
static const struct pinmux_hc32_config pinmux_hc32_portf_config = {
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET * 5u,
	.port = GPIO_PORT_F,
};
DEVICE_AND_API_INIT(pinmux_portf, CONFIG_PINMUX_HC32_PORTF_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_portf_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#ifdef CONFIG_PINMUX_HC32_PORTG_NAME
/* GPIOG */
static const struct pinmux_hc32_config pinmux_hc32_portg_config = {
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET * 6u,
	.port = GPIO_PORT_G,
};
DEVICE_AND_API_INIT(pinmux_portg, CONFIG_PINMUX_HC32_PORTG_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_portg_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#ifdef CONFIG_PINMUX_HC32_PORTH_NAME
/* GPIOH */
static const struct pinmux_hc32_config pinmux_hc32_porth_config = {
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET * 7u,
	.port = GPIO_PORT_H,
};
DEVICE_AND_API_INIT(pinmux_porth, CONFIG_PINMUX_HC32_PORTH_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_porth_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#ifdef CONFIG_PINMUX_HC32_PORTI_NAME
/* GPIOI */
static const struct pinmux_hc32_config pinmux_hc32_porti_config = {
	.base = PINMUX_GPIO_BASE + PINMUX_GPIO_OFFSET * 8u,
	.port = GPIO_PORT_I,
};
DEVICE_AND_API_INIT(pinmux_porti, CONFIG_PINMUX_HC32_PORTI_NAME,
		    &pinmux_initialize, NULL, &pinmux_hc32_porti_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &api_funcs);
#endif

#endif /* CONFIG_SOC_HC32F460 */

