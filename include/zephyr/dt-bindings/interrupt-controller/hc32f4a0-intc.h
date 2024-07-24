/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_HC32F4A0_INTC_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_HC32F4A0_INTC_H_

#include <zephyr/dt-bindings/interrupt-controller/hc32f4a0-int-evt.h>


/* Ext ch0 */
#define HC32_EXTINT0_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT0_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch1 */
#define HC32_EXTINT1_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT1_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch2 */
#define HC32_EXTINT2_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT2_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch3 */
#define HC32_EXTINT3_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT3_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch4 */
#define HC32_EXTINT4_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT4_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch5 */
#define HC32_EXTINT5_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT5_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch6 */
#define HC32_EXTINT6_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT6_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch7 */
#define HC32_EXTINT7_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT7_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch8 */
#define HC32_EXTINT8_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT8_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch9 */
#define HC32_EXTINT9_IRQ_NUM             INT128_IRQn
#define HC32_EXTINT9_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch10 */
#define HC32_EXTINT10_IRQ_NUM            INT128_IRQn
#define HC32_EXTINT10_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch11 */
#define HC32_EXTINT11_IRQ_NUM            INT128_IRQn
#define HC32_EXTINT11_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch12 */
#define HC32_EXTINT12_IRQ_NUM            INT128_IRQn
#define HC32_EXTINT12_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch13 */
#define HC32_EXTINT13_IRQ_NUM            INT128_IRQn
#define HC32_EXTINT13_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch14 */
#define HC32_EXTINT14_IRQ_NUM            INT128_IRQn
#define HC32_EXTINT14_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch15 */
#define HC32_EXTINT15_IRQ_NUM            INT128_IRQn
#define HC32_EXTINT15_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

/* DMA1 ch0 */
#define HC32_DMA1_CH0_IRQ_NUM            INT038_IRQn
#define HC32_DMA1_CH0_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA1 ch1 */
#define HC32_DMA1_CH1_IRQ_NUM            INT039_IRQn
#define HC32_DMA1_CH1_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA1 ch2 */
#define HC32_DMA1_CH2_IRQ_NUM            INT040_IRQn
#define HC32_DMA1_CH2_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA1 ch3 */
#define HC32_DMA1_CH3_IRQ_NUM            INT041_IRQn
#define HC32_DMA1_CH3_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA1 err */
#define HC32_DMA1_ERR_IRQ_NUM            INT042_IRQn
#define HC32_DMA1_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

/* DMA2 ch0 */
#define HC32_DMA2_CH0_IRQ_NUM            INT043_IRQn
#define HC32_DMA2_CH0_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA2 ch1 */
#define HC32_DMA2_CH1_IRQ_NUM            INT018_IRQn
#define HC32_DMA2_CH1_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA2 ch2 */
#define HC32_DMA2_CH2_IRQ_NUM            INT019_IRQn
#define HC32_DMA2_CH2_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA2 ch3 */
#define HC32_DMA2_CH3_IRQ_NUM            INT020_IRQn
#define HC32_DMA2_CH3_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA2 err */
#define HC32_DMA2_ERR_IRQ_NUM            INT021_IRQn
#define HC32_DMA2_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

/* USART */
#define HC32_UART1_RXERR_IRQ_NUM         INT010_IRQn
#define HC32_UART1_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_UART1_RX_IRQ_NUM            INT014_IRQn
#define HC32_UART1_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_UART1_TX_IRQ_NUM            INT015_IRQn
#define HC32_UART1_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_UART1_TX_CPLT_IRQ_NUM       INT016_IRQn
#define HC32_UART1_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_UART1_RXTO_IRQ_NUM          INT011_IRQn
#define HC32_UART1_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_UART2_RXERR_IRQ_NUM         INT086_IRQn
#define HC32_UART2_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_UART2_RX_IRQ_NUM            INT087_IRQn
#define HC32_UART2_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_UART2_TX_IRQ_NUM            INT088_IRQn
#define HC32_UART2_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_UART2_TX_CPLT_IRQ_NUM       INT089_IRQn
#define HC32_UART2_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_UART2_RXTO_IRQ_NUM          INT090_IRQn
#define HC32_UART2_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_UART3_RXERR_IRQ_NUM         INT092_IRQn
#define HC32_UART3_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_UART3_RX_IRQ_NUM            INT093_IRQn
#define HC32_UART3_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_UART3_TX_IRQ_NUM            INT094_IRQn
#define HC32_UART3_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_UART3_TX_CPLT_IRQ_NUM       INT095_IRQn
#define HC32_UART3_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_UART3_RXTO_IRQ_NUM          INT096_IRQn
#define HC32_UART3_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_UART4_RXERR_IRQ_NUM         INT092_IRQn
#define HC32_UART4_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_UART4_RX_IRQ_NUM            INT093_IRQn
#define HC32_UART4_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_UART4_TX_IRQ_NUM            INT094_IRQn
#define HC32_UART4_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_UART4_RXTO_IRQ_NUM          INT096_IRQn
#define HC32_UART4_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_UART4_TX_CPLT_IRQ_NUM       INT095_IRQn
#define HC32_UART4_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

/* SPI */
#define HC32_SPI1_ERR_IRQ_NUM            INT086_IRQn
#define HC32_SPI1_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI1_RBF_IRQ_NUM            INT087_IRQn
#define HC32_SPI1_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI1_TBE_IRQ_NUM            INT088_IRQn
#define HC32_SPI1_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI2_ERR_IRQ_NUM            INT089_IRQn
#define HC32_SPI2_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI2_RBF_IRQ_NUM            INT090_IRQn
#define HC32_SPI2_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI2_TBE_IRQ_NUM            INT091_IRQn
#define HC32_SPI2_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI3_ERR_IRQ_NUM            INT092_IRQn
#define HC32_SPI3_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI3_RBF_IRQ_NUM            INT093_IRQn
#define HC32_SPI3_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI3_TBE_IRQ_NUM            INT094_IRQn
#define HC32_SPI3_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI4_ERR_IRQ_NUM            INT095_IRQn
#define HC32_SPI4_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI4_RBF_IRQ_NUM            INT096_IRQn
#define HC32_SPI4_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI4_TBE_IRQ_NUM            INT097_IRQn
#define HC32_SPI4_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI5_ERR_IRQ_NUM            INT098_IRQn
#define HC32_SPI5_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI5_RBF_IRQ_NUM            INT099_IRQn
#define HC32_SPI5_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI5_TBE_IRQ_NUM            INT100_IRQn
#define HC32_SPI5_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI6_ERR_IRQ_NUM            INT101_IRQn
#define HC32_SPI6_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI6_RBF_IRQ_NUM            INT102_IRQn
#define HC32_SPI6_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI6_TBE_IRQ_NUM            INT103_IRQn
#define HC32_SPI6_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

/* RTC */
#define HC32_RTC_ALARM_IRQ_NUM           INT044_IRQn
#define HC32_RTC_ALARM_IRQ_PRIO          DDL_IRQ_PRIO_DEFAULT

#define HC32_RTC_PRD_IRQ_NUM             INT045_IRQn
#define HC32_RTC_PRD_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT

/* TimerA1*/
#define HC32_TMRA1_OVF_IRQ_NUM          INT074_IRQn
#define HC32_TMRA1_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA1_UDF_IRQ_NUM          INT075_IRQn
#define HC32_TMRA1_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA1_CMP_IRQ_NUM          INT076_IRQn
#define HC32_TMRA1_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA2*/
#define HC32_TMRA2_OVF_IRQ_NUM          INT077_IRQn
#define HC32_TMRA2_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA2_UDF_IRQ_NUM          INT078_IRQn
#define HC32_TMRA2_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA2_CMP_IRQ_NUM          INT079_IRQn
#define HC32_TMRA2_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA3*/
#define HC32_TMRA3_OVF_IRQ_NUM          INT080_IRQn
#define HC32_TMRA3_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA3_UDF_IRQ_NUM          INT081_IRQn
#define HC32_TMRA3_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA3_CMP_IRQ_NUM          INT082_IRQn
#define HC32_TMRA3_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA4*/
#define HC32_TMRA4_OVF_IRQ_NUM          INT083_IRQn
#define HC32_TMRA4_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA4_UDF_IRQ_NUM          INT084_IRQn
#define HC32_TMRA4_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA4_CMP_IRQ_NUM          INT085_IRQn
#define HC32_TMRA4_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA5*/
#define HC32_TMRA5_OVF_IRQ_NUM          INT092_IRQn
#define HC32_TMRA5_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA5_UDF_IRQ_NUM          INT093_IRQn
#define HC32_TMRA5_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA5_CMP_IRQ_NUM          INT094_IRQn
#define HC32_TMRA5_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA6*/
#define HC32_TMRA6_OVF_IRQ_NUM          INT092_IRQn
#define HC32_TMRA6_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA6_UDF_IRQ_NUM          INT093_IRQn
#define HC32_TMRA6_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA6_CMP_IRQ_NUM          INT094_IRQn
#define HC32_TMRA6_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA7*/
#define HC32_TMRA7_OVF_IRQ_NUM          INT095_IRQn
#define HC32_TMRA7_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA7_UDF_IRQ_NUM          INT096_IRQn
#define HC32_TMRA7_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA7_CMP_IRQ_NUM          INT097_IRQn
#define HC32_TMRA7_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA8*/
#define HC32_TMRA8_OVF_IRQ_NUM          INT095_IRQn
#define HC32_TMRA8_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA8_UDF_IRQ_NUM          INT096_IRQn
#define HC32_TMRA8_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA8_CMP_IRQ_NUM          INT097_IRQn
#define HC32_TMRA8_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA9*/
#define HC32_TMRA9_OVF_IRQ_NUM          INT098_IRQn
#define HC32_TMRA9_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA9_UDF_IRQ_NUM          INT099_IRQn
#define HC32_TMRA9_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA9_CMP_IRQ_NUM          INT100_IRQn
#define HC32_TMRA9_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA10*/
#define HC32_TMRA10_OVF_IRQ_NUM          INT098_IRQn
#define HC32_TMRA10_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA10_UDF_IRQ_NUM          INT099_IRQn
#define HC32_TMRA10_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA10_CMP_IRQ_NUM          INT100_IRQn
#define HC32_TMRA10_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA11*/
#define HC32_TMRA11_OVF_IRQ_NUM          INT101_IRQn
#define HC32_TMRA11_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA11_UDF_IRQ_NUM          INT102_IRQn
#define HC32_TMRA11_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA11_CMP_IRQ_NUM          INT103_IRQn
#define HC32_TMRA11_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* TimerA12*/
#define HC32_TMRA12_OVF_IRQ_NUM          INT101_IRQn
#define HC32_TMRA12_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA12_UDF_IRQ_NUM          INT102_IRQn
#define HC32_TMRA12_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMRA12_CMP_IRQ_NUM          INT103_IRQn
#define HC32_TMRA12_CMP_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer41*/
#define HC32_TMR41_CMP_UH_IRQ_NUM       INT092_IRQn
#define HC32_TMR41_CMP_UH_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR41_CMP_UL_IRQ_NUM       INT093_IRQn
#define HC32_TMR41_CMP_UL_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR41_CMP_VH_IRQ_NUM       INT094_IRQn
#define HC32_TMR41_CMP_VH_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR41_CMP_VL_IRQ_NUM       INT095_IRQn
#define HC32_TMR41_CMP_VL_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR41_CMP_WH_IRQ_NUM       INT012_IRQn
#define HC32_TMR41_CMP_WH_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR41_CMP_WL_IRQ_NUM       INT013_IRQn
#define HC32_TMR41_CMP_WL_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR41_OVF_IRQ_NUM          INT096_IRQn
#define HC32_TMR41_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR41_UDF_IRQ_NUM          INT097_IRQn
#define HC32_TMR41_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer42*/
#define HC32_TMR42_CMP_UH_IRQ_NUM       INT092_IRQn
#define HC32_TMR42_CMP_UH_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR42_CMP_UL_IRQ_NUM       INT093_IRQn
#define HC32_TMR42_CMP_UL_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR42_CMP_VH_IRQ_NUM       INT094_IRQn
#define HC32_TMR42_CMP_VH_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR42_CMP_VL_IRQ_NUM       INT095_IRQn
#define HC32_TMR42_CMP_VL_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR42_CMP_WH_IRQ_NUM       INT012_IRQn
#define HC32_TMR42_CMP_WH_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR42_CMP_WL_IRQ_NUM       INT013_IRQn
#define HC32_TMR42_CMP_WL_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR42_OVF_IRQ_NUM          INT096_IRQn
#define HC32_TMR42_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR42_UDF_IRQ_NUM          INT097_IRQn
#define HC32_TMR42_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer43 */
#define HC32_TMR43_CMP_UH_IRQ_NUM       INT098_IRQn
#define HC32_TMR43_CMP_UH_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR43_CMP_UL_IRQ_NUM       INT099_IRQn
#define HC32_TMR43_CMP_UL_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR43_CMP_VH_IRQ_NUM       INT100_IRQn
#define HC32_TMR43_CMP_VH_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR43_CMP_VL_IRQ_NUM       INT101_IRQn
#define HC32_TMR43_CMP_VL_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR43_CMP_WH_IRQ_NUM       INT012_IRQn
#define HC32_TMR43_CMP_WH_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR43_CMP_WL_IRQ_NUM       INT013_IRQn
#define HC32_TMR43_CMP_WL_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR43_OVF_IRQ_NUM          INT102_IRQn
#define HC32_TMR43_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR43_UDF_IRQ_NUM          INT103_IRQn
#define HC32_TMR43_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer61 interrupt */
#define HC32_TMR61_CMPA_IRQ_NUM         INT056_IRQn
#define HC32_TMR61_CMPA_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR61_CMPB_IRQ_NUM         INT057_IRQn
#define HC32_TMR61_CMPB_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR61_CMPC_IRQ_NUM         INT058_IRQn
#define HC32_TMR61_CMPC_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR61_CMPD_IRQ_NUM         INT059_IRQn
#define HC32_TMR61_CMPD_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR61_CMPE_IRQ_NUM         INT060_IRQn
#define HC32_TMR61_CMPE_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR61_CMPF_IRQ_NUM         INT061_IRQn
#define HC32_TMR61_CMPF_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR61_OVF_IRQ_NUM          INT030_IRQn
#define HC32_TMR61_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR61_UDF_IRQ_NUM          INT031_IRQn
#define HC32_TMR61_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer62 interrupt */
#define HC32_TMR62_CMPA_IRQ_NUM         INT056_IRQn
#define HC32_TMR62_CMPA_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR62_CMPB_IRQ_NUM         INT057_IRQn
#define HC32_TMR62_CMPB_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR62_CMPC_IRQ_NUM         INT058_IRQn
#define HC32_TMR62_CMPC_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR62_CMPD_IRQ_NUM         INT059_IRQn
#define HC32_TMR62_CMPD_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR62_CMPE_IRQ_NUM         INT060_IRQn
#define HC32_TMR62_CMPE_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR62_CMPF_IRQ_NUM         INT061_IRQn
#define HC32_TMR62_CMPF_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR62_OVF_IRQ_NUM          INT030_IRQn
#define HC32_TMR62_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR62_UDF_IRQ_NUM          INT031_IRQn
#define HC32_TMR62_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer63 interrupt */
#define HC32_TMR63_CMPA_IRQ_NUM         INT062_IRQn
#define HC32_TMR63_CMPA_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR63_CMPB_IRQ_NUM         INT063_IRQn
#define HC32_TMR63_CMPB_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR63_CMPC_IRQ_NUM         INT064_IRQn
#define HC32_TMR63_CMPC_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR63_CMPD_IRQ_NUM         INT065_IRQn
#define HC32_TMR63_CMPD_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR63_CMPE_IRQ_NUM         INT066_IRQn
#define HC32_TMR63_CMPE_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR63_CMPF_IRQ_NUM         INT067_IRQn
#define HC32_TMR63_CMPF_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR63_OVF_IRQ_NUM          INT028_IRQn
#define HC32_TMR63_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR63_UDF_IRQ_NUM          INT029_IRQn
#define HC32_TMR63_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer64 interrupt */
#define HC32_TMR64_CMPA_IRQ_NUM         INT068_IRQn
#define HC32_TMR64_CMPA_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR64_CMPB_IRQ_NUM         INT069_IRQn
#define HC32_TMR64_CMPB_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR64_CMPC_IRQ_NUM         INT070_IRQn
#define HC32_TMR64_CMPC_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR64_CMPD_IRQ_NUM         INT071_IRQn
#define HC32_TMR64_CMPD_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR64_CMPE_IRQ_NUM         INT072_IRQn
#define HC32_TMR64_CMPE_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR64_CMPF_IRQ_NUM         INT073_IRQn
#define HC32_TMR64_CMPF_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR64_OVF_IRQ_NUM          INT026_IRQn
#define HC32_TMR64_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR64_UDF_IRQ_NUM          INT027_IRQn
#define HC32_TMR64_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer65 interrupt */
#define HC32_TMR65_CMPA_IRQ_NUM         INT074_IRQn
#define HC32_TMR65_CMPA_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR65_CMPB_IRQ_NUM         INT075_IRQn
#define HC32_TMR65_CMPB_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR65_CMPC_IRQ_NUM         INT076_IRQn
#define HC32_TMR65_CMPC_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR65_CMPD_IRQ_NUM         INT077_IRQn
#define HC32_TMR65_CMPD_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR65_CMPE_IRQ_NUM         INT078_IRQn
#define HC32_TMR65_CMPE_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR65_CMPF_IRQ_NUM         INT079_IRQn
#define HC32_TMR65_CMPF_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR65_OVF_IRQ_NUM          INT024_IRQn
#define HC32_TMR65_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR65_UDF_IRQ_NUM          INT025_IRQn
#define HC32_TMR65_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer66 interrupt */
#define HC32_TMR66_CMPA_IRQ_NUM         INT074_IRQn
#define HC32_TMR66_CMPA_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR66_CMPB_IRQ_NUM         INT075_IRQn
#define HC32_TMR66_CMPB_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR66_CMPC_IRQ_NUM         INT076_IRQn
#define HC32_TMR66_CMPC_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR66_CMPD_IRQ_NUM         INT077_IRQn
#define HC32_TMR66_CMPD_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR66_CMPE_IRQ_NUM         INT078_IRQn
#define HC32_TMR66_CMPE_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR66_CMPF_IRQ_NUM         INT079_IRQn
#define HC32_TMR66_CMPF_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR66_OVF_IRQ_NUM          INT024_IRQn
#define HC32_TMR66_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR66_UDF_IRQ_NUM          INT025_IRQn
#define HC32_TMR66_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer67 interrupt */
#define HC32_TMR67_CMPA_IRQ_NUM         INT080_IRQn
#define HC32_TMR67_CMPA_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR67_CMPB_IRQ_NUM         INT081_IRQn
#define HC32_TMR67_CMPB_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR67_CMPC_IRQ_NUM         INT082_IRQn
#define HC32_TMR67_CMPC_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR67_CMPD_IRQ_NUM         INT083_IRQn
#define HC32_TMR67_CMPD_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR67_CMPE_IRQ_NUM         INT084_IRQn
#define HC32_TMR67_CMPE_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR67_CMPF_IRQ_NUM         INT085_IRQn
#define HC32_TMR67_CMPF_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR67_OVF_IRQ_NUM          INT022_IRQn
#define HC32_TMR67_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR67_UDF_IRQ_NUM          INT023_IRQn
#define HC32_TMR67_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* Timer68 interrupt */
#define HC32_TMR68_CMPA_IRQ_NUM         INT080_IRQn
#define HC32_TMR68_CMPA_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR68_CMPB_IRQ_NUM         INT081_IRQn
#define HC32_TMR68_CMPB_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR68_CMPC_IRQ_NUM         INT082_IRQn
#define HC32_TMR68_CMPC_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR68_CMPD_IRQ_NUM         INT083_IRQn
#define HC32_TMR68_CMPD_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR68_CMPE_IRQ_NUM         INT084_IRQn
#define HC32_TMR68_CMPE_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR68_CMPF_IRQ_NUM         INT085_IRQn
#define HC32_TMR68_CMPF_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR68_OVF_IRQ_NUM          INT022_IRQn
#define HC32_TMR68_OVF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

#define HC32_TMR68_UDF_IRQ_NUM          INT023_IRQn
#define HC32_TMR68_UDF_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT

/* I2C share */
#define HC32_I2C_IRQ_SHARE1_NUM			INT141_IRQn
#define HC32_I2C_IRQ_SHARE1_PRIO		DDL_IRQ_PRIO_DEFAULT
/* I2C1 */
#define HC32_I2C1_EE_IRQ_NUM			HC32_I2C_IRQ_SHARE1_NUM
#define HC32_I2C1_EE_IRQ_PRIO			HC32_I2C_IRQ_SHARE1_PRIO
#define HC32_I2C1_TE_IRQ_NUM			HC32_I2C_IRQ_SHARE1_NUM
#define HC32_I2C1_TE_IRQ_PRIO			HC32_I2C_IRQ_SHARE1_PRIO
#define HC32_I2C1_TX_IRQ_NUM			HC32_I2C_IRQ_SHARE1_NUM
#define HC32_I2C1_TX_IRQ_PRIO			HC32_I2C_IRQ_SHARE1_PRIO
#define HC32_I2C1_RX_IRQ_NUM			HC32_I2C_IRQ_SHARE1_NUM
#define HC32_I2C1_RX_IRQ_PRIO			HC32_I2C_IRQ_SHARE1_PRIO
/* I2C2 */
#define HC32_I2C2_EE_IRQ_NUM			INT110_IRQn
#define HC32_I2C2_EE_IRQ_PRIO			DDL_IRQ_PRIO_DEFAULT
#define HC32_I2C2_TE_IRQ_NUM			INT111_IRQn
#define HC32_I2C2_TE_IRQ_PRIO			DDL_IRQ_PRIO_DEFAULT
#define HC32_I2C2_TX_IRQ_NUM			INT112_IRQn
#define HC32_I2C2_TX_IRQ_PRIO			DDL_IRQ_PRIO_DEFAULT
#define HC32_I2C2_RX_IRQ_NUM			INT113_IRQn
#define HC32_I2C2_RX_IRQ_PRIO			DDL_IRQ_PRIO_DEFAULT
/* I2C3 */
#define HC32_I2C3_EE_IRQ_NUM			HC32_I2C_IRQ_SHARE1_NUM
#define HC32_I2C3_EE_IRQ_PRIO			HC32_I2C_IRQ_SHARE1_PRIO
#define HC32_I2C3_TE_IRQ_NUM			HC32_I2C_IRQ_SHARE1_NUM
#define HC32_I2C3_TE_IRQ_PRIO			HC32_I2C_IRQ_SHARE1_PRIO
#define HC32_I2C3_TX_IRQ_NUM			HC32_I2C_IRQ_SHARE1_NUM
#define HC32_I2C3_TX_IRQ_PRIO			HC32_I2C_IRQ_SHARE1_PRIO
#define HC32_I2C3_RX_IRQ_NUM			HC32_I2C_IRQ_SHARE1_NUM
#define HC32_I2C3_RX_IRQ_PRIO			HC32_I2C_IRQ_SHARE1_PRIO

#define HC32_I2C_IRQ_SHARE2_NUM			INT142_IRQn
#define HC32_I2C_IRQ_SHARE2_PRIO		DDL_IRQ_PRIO_DEFAULT
/* I2C4 */
#define HC32_I2C4_EE_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C4_EE_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
#define HC32_I2C4_TE_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C4_TE_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
#define HC32_I2C4_TX_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C4_TX_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
#define HC32_I2C4_RX_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C4_RX_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
/* I2C5 */
#define HC32_I2C5_EE_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C5_EE_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
#define HC32_I2C5_TE_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C5_TE_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
#define HC32_I2C5_TX_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C5_TX_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
#define HC32_I2C5_RX_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C5_RX_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
/* I2C6 */
#define HC32_I2C6_EE_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C6_EE_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
#define HC32_I2C6_TE_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C6_TE_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
#define HC32_I2C6_TX_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C6_TX_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO
#define HC32_I2C6_RX_IRQ_NUM			HC32_I2C_IRQ_SHARE2_NUM
#define HC32_I2C6_RX_IRQ_PRIO			HC32_I2C_IRQ_SHARE2_PRIO

/* ADC */
#define HC32_ADC1_EOCA_IRQ_NUM           INT122_IRQn
#define HC32_ADC1_EOCA_IRQ_PRIO          DDL_IRQ_PRIO_DEFAULT
#define HC32_ADC1_EOCB_IRQ_NUM           INT123_IRQn
#define HC32_ADC1_EOCB_IRQ_PRIO          DDL_IRQ_PRIO_DEFAULT
#define HC32_ADC2_EOCA_IRQ_NUM           INT122_IRQn
#define HC32_ADC2_EOCA_IRQ_PRIO          DDL_IRQ_PRIO_DEFAULT
#define HC32_ADC2_EOCB_IRQ_NUM           INT123_IRQn
#define HC32_ADC2_EOCB_IRQ_PRIO          DDL_IRQ_PRIO_DEFAULT
#define HC32_ADC3_EOCA_IRQ_NUM           INT122_IRQn
#define HC32_ADC3_EOCA_IRQ_PRIO          DDL_IRQ_PRIO_DEFAULT
#define HC32_ADC3_EOCB_IRQ_NUM           INT123_IRQn
#define HC32_ADC3_EOCB_IRQ_PRIO          DDL_IRQ_PRIO_DEFAULT

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_HC32F4A0_INTC_H_ */
