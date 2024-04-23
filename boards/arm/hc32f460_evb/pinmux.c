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

	struct device *portd =
		device_get_binding(CONFIG_PINMUX_HC32_PORTD_NAME);

	struct device *porte =
		device_get_binding(CONFIG_PINMUX_HC32_PORTE_NAME);

	/* SPI0 CS0, SCK, SOUT, SIN */
	pinmux_pin_set(porta, 8, 43); /* SCK */
	pinmux_pin_set(portb, 0, 40); /* MOSI */
	pinmux_pin_set(portc, 5, 41); /* MISO */

	/* i2c3 */
	pinmux_pin_set(porte, 15, 49); /* scl */
	pinmux_pin_set(portb, 5, 48); /* sda */

	/* uart4, default console */
	pinmux_pin_set(porte, 6, 36); /* tx */
	pinmux_pin_set(portb, 9, 37); /* rx */

	/* uart2, used in async uart test */
	pinmux_pin_set(porta, 2, 36); /* tx */
	pinmux_pin_set(porta, 3, 37); /* rx */

	/* qspi */
	pinmux_pin_set(portc, 7, 7); /* cs */
	pinmux_pin_set(portc, 6, 7); /* sck */
	pinmux_pin_set(portd, 8, 7); /* IO0 */
	pinmux_pin_set(portd, 9, 7); /* IO1 */
	pinmux_pin_set(portd, 10, 7); /* IO2 */
	pinmux_pin_set(portd, 11, 7); /* IO3 */

	/* i2c1 */
	pinmux_pin_set(portd, 1, 49); /* scl */
	pinmux_pin_set(portd, 2, 48); /* sda */

#if defined(DT_CAN_1_NAME)
	/* can1 */
	pinmux_pin_set(portb, 7, 50); /* tx */
	pinmux_pin_set(portb, 6, 51); /* rx */
#endif

	/* adc1 */
	pinmux_pin_set(porta, 0, 0xFF); /* adc1_in0 */
	pinmux_pin_set(porta, 1, 0xFF); /* adc1_in1 */

#endif

	return 0;
}

SYS_INIT(hc32_pinmux_init, PRE_KERNEL_2, CONFIG_PINMUX_INIT_PRIORITY);
