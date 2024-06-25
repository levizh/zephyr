/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_HC32F460_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_HC32F460_CLOCK_H_

#include "hc32f4_common_clock.h"

#define HC32_FCG0_PERIPH_SRAMH		0
#define HC32_FCG0_PERIPH_SRAM12		4
#define HC32_FCG0_PERIPH_SRAM3		8
#define HC32_FCG0_PERIPH_SRAMRET	10
#define HC32_FCG0_PERIPH_DMA1		14
#define HC32_FCG0_PERIPH_DMA2		15
#define HC32_FCG0_PERIPH_FCM		16
#define HC32_FCG0_PERIPH_AOS		17
#define HC32_FCG0_PERIPH_AES		20
#define HC32_FCG0_PERIPH_HASH		21
#define HC32_FCG0_PERIPH_TRNG		22
#define HC32_FCG0_PERIPH_CRC		23
#define HC32_FCG0_PERIPH_DCU1		24
#define HC32_FCG0_PERIPH_DCU2		25
#define HC32_FCG0_PERIPH_DCU3		26
#define HC32_FCG0_PERIPH_DCU4		27
#define HC32_FCG0_PERIPH_KEY		31

#define HC32_FCG1_PERIPH_QSPI		3
#define HC32_FCG1_PERIPH_I2C1		4
#define HC32_FCG1_PERIPH_I2C2		5
#define HC32_FCG1_PERIPH_I2C3		6
#define HC32_FCG1_PERIPH_USBFS		8
#define HC32_FCG1_PERIPH_SDIOC1		10
#define HC32_FCG1_PERIPH_SDIOC2		11
#define HC32_FCG1_PERIPH_I2S1		12
#define HC32_FCG1_PERIPH_I2S2		13
#define HC32_FCG1_PERIPH_I2S3		14
#define HC32_FCG1_PERIPH_I2S4		15
#define HC32_FCG1_PERIPH_SPI1		16
#define HC32_FCG1_PERIPH_SPI2		17
#define HC32_FCG1_PERIPH_SPI3		18
#define HC32_FCG1_PERIPH_SPI4		19
#define HC32_FCG1_PERIPH_USART1		24
#define HC32_FCG1_PERIPH_USART2		25
#define HC32_FCG1_PERIPH_USART3		26
#define HC32_FCG1_PERIPH_USART4		27

#define HC32_FCG2_PERIPH_TMR0_1		0
#define HC32_FCG2_PERIPH_TMR0_2		1
#define HC32_FCG2_PERIPH_TMRA_1		2
#define HC32_FCG2_PERIPH_TMRA_2		3
#define HC32_FCG2_PERIPH_TMRA_3		4
#define HC32_FCG2_PERIPH_TMRA_4		5
#define HC32_FCG2_PERIPH_TMRA_5		6
#define HC32_FCG2_PERIPH_TMRA_6		7
#define HC32_FCG2_PERIPH_TMR4_1		8
#define HC32_FCG2_PERIPH_TMR4_2		9
#define HC32_FCG2_PERIPH_TMR4_3		10
#define HC32_FCG2_PERIPH_EMB		15
#define HC32_FCG2_PERIPH_TMR6_1		16
#define HC32_FCG2_PERIPH_TMR6_2		17
#define HC32_FCG2_PERIPH_TMR6_3		18

#define HC32_FCG3_PERIPH_ADC1		0
#define HC32_FCG3_PERIPH_ADC2		1
#define HC32_FCG3_PERIPH_CMP		8
#define HC32_FCG3_PERIPH_OTS		12

#define HC32_CLK_CONF_USB				(4)
#define HC32_CLK_CONF_I2S				(5)
#define HC32_CLK_CONF_MAX				HC32_CLK_CONF_I2S
#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_HC32F460_CLOCK_H_ */
