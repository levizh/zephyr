/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <pinmux.h>

static int hc32_pinmux_init(struct device *dev)
{
	ARG_UNUSED(dev);

#ifdef CONFIG_PINMUX_HC32
	struct device *porta =
		device_get_binding(CONFIG_PINMUX_HC32_PORTA_NAME);

	struct device *portb =
		device_get_binding(CONFIG_PINMUX_HC32_PORTB_NAME);

	struct device *portc =
		device_get_binding(CONFIG_PINMUX_HC32_PORTC_NAME);

	struct device *porte =
		device_get_binding(CONFIG_PINMUX_HC32_PORTE_NAME);

	/* SPI0 CS0, SCK, SOUT, SIN */
	pinmux_pin_set(porta, 8, 43); /* SCK */
	pinmux_pin_set(portb, 0, 40); /* MOSI */
	pinmux_pin_set(portc, 5, 41); /* MISO */

	/* i2c3 */
	pinmux_pin_set(porte, 15, 49); /* scl */
	pinmux_pin_set(portb, 5, 48); /* sda */

	/* default console uart4 */
	pinmux_pin_set(porte, 6, 36); /* tx */
	pinmux_pin_set(portb, 9, 37); /* rx */

	/* uart2 */
	pinmux_pin_set(porta, 2, 36); /* tx */
	pinmux_pin_set(porta, 3, 37); /* rx */

#endif
	return 0;
}

SYS_INIT(hc32_pinmux_init, PRE_KERNEL_2, CONFIG_PINMUX_INIT_PRIORITY);
