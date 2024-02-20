/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HC32_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HC32_PINCTRL_H_

#define HC32_PORT(_mux) \
	(((_mux) >> HC32_PORT_SHIFT) & HC32_PORT_MSK)

#define HC32_PIN(_mux) \
	(((_mux) >> HC32_PIN_SHIFT) & HC32_PIN_MSK)

#define HC32_MODE(_mux) \
	(((_mux) >> HC32_MODE_SHIFT) & HC32_MODE_MSK)

#define HC32_FUNC_NUM(_mux) \
	(((_mux) >> HC32_FUNC_SHIFT) & HC32_FUNC_MSK)

#define HC32_PIN_BIAS(_cfg) \
	(((_cfg) >> HC32_PUR_SHIFT) & HC32_PUR_MSK)

#define HC32_PIN_DRV(_cfg) \
	(((_cfg) >> HC32_NOD_SHIFT) & HC32_NOD_MSK)

#define HC32_OUT_LEVEL(_cfg) \
	(((_cfg) >> HC32_OTYPE_SHIFT) & HC32_OTYPE_MSK)

#define HC32_PIN_EN_DIR(_cfg) \
	(((_cfg) >> HC32_DIREN_SHIFT) & HC32_DIREN_MSK)

#define HC32_PIN_DRIVER_STRENGTH(_cfg) \
	(((_cfg) >> HC32_STRENGTH_SHIFT) & HC32_STRENGTH_MSK)

/* pinmux filed in pinctrl_soc_pin*/
/**
 * @brief Pin modes
 */
#define HC32_ANALOG     0x00
#define HC32_GPIO       0x01
#define HC32_FUNC       0x02
#define HC32_SUBFUNC    0x03

/**
 * @brief Macro to generate pinmux int using port, pin number and mode arguments
 * This is inspired from Linux equivalent st,HC32f429-pinctrl binding
 */

#define HC32_FUNC_SHIFT  10U
#define HC32_FUNC_MSK    0xFFU
#define HC32_MODE_SHIFT  8U
#define HC32_MODE_MSK    0x03U
#define HC32_PIN_SHIFT   4U
#define HC32_PIN_MSK     0x0FU
#define HC32_PORT_SHIFT  0U
#define HC32_PORT_MSK    0x0FU

/**
 * @brief Pin configuration configuration bit field.
 *
 * Fields:
 * - func_num [ 10 ：17]
 * - mode [ 8 : 9 ]
 * - pin  [ 4 : 7 ]
 * - port [ 0 : 3 ]
 *
 * @param port Port ('A'..'E', 'H')
 * @param pin Pin (0..15)
 * @param mode Mode (ANALOG, FUC, SUBFUC).
 * @param func_num range:0--63.
 */
#define HC32_PINMUX(port, pin, mode, func_num)					       \
		(((((port) - 'A') & HC32_PORT_MSK) << HC32_PORT_SHIFT) |    \
		(((pin) & HC32_PIN_MSK) << HC32_PIN_SHIFT) |	       \
		(((HC32_ ## mode) & HC32_MODE_MSK) << HC32_MODE_SHIFT) |   \
		(((func_num) & HC32_FUNC_MSK) << HC32_FUNC_SHIFT))


/* pincfg filed in pinctrl_soc_pin*/
/**
 * @brief Definitions cfg bit pos and mask
 */
#define HC32_NO_PULL                0x0
#define HC32_PULL_UP                0x1
#define HC32_PUSH_PULL              0x0
#define HC32_OPEN_DRAIN             0x1
#define HC32_OUTPUT_LOW             0x0
#define HC32_OUTPUT_HIGH            0x1
#define HC32_INPUT_ENABLE           0x0
#define HC32_OUTPUT_ENABLE          0x1

#define HC32_DRIVER_STRENGTH_LOW    0x0
#define HC32_DRIVER_STRENGTH_MEDIUM 0x1
#define HC32_DRIVER_STRENGTH_HIGH   0x2

/** PUR field mask. */
#define HC32_PUR_MSK         0x01U
/** PUR field position. */
#define HC32_PUR_SHIFT       0U
/** NOD field mask. */
#define HC32_NOD_MSK         0x01U
/** NOD field position. */
#define HC32_NOD_SHIFT       1U
/** OTYPE field mask. */
#define HC32_OTYPE_MSK       0x01U
/** OTYPE field position. */
#define HC32_OTYPE_SHIFT     2U
/** DIREN field mask. */
#define HC32_DIREN_MSK       0x01U
/** DIREN field position. */
#define HC32_DIREN_SHIFT     3U
/** STRENGTH field mask. */
#define HC32_STRENGTH_MSK    0x03U
/** STRENGTH field position. */
#define HC32_STRENGTH_SHIFT  4U


#endif	/* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HC32_PINCTRL_H_ */
