/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_HC32_INTC_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_HC32_INTC_H_

// #if defined(HC32F460)
#include <zephyr/dt-bindings/interrupt-controller/hc32f460-intc.h>
// #else
// #error "unsupported chip"
// #endif

/* Ext ch0 */
#define HC32_EXTINT0_IRQ_NUM             INT022_IRQn
#define HC32_EXTINT0_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch1 */
#define HC32_EXTINT1_IRQ_NUM             INT023_IRQn
#define HC32_EXTINT1_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch2 */
#define HC32_EXTINT2_IRQ_NUM             INT024_IRQn
#define HC32_EXTINT2_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch3 */
#define HC32_EXTINT3_IRQ_NUM             INT025_IRQn
#define HC32_EXTINT3_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch4 */
#define HC32_EXTINT4_IRQ_NUM             INT026_IRQn
#define HC32_EXTINT4_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch5 */
#define HC32_EXTINT5_IRQ_NUM             INT027_IRQn
#define HC32_EXTINT5_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch6 */
#define HC32_EXTINT6_IRQ_NUM             INT028_IRQn
#define HC32_EXTINT6_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch7 */
#define HC32_EXTINT7_IRQ_NUM             INT029_IRQn
#define HC32_EXTINT7_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch8 */
#define HC32_EXTINT8_IRQ_NUM             INT030_IRQn
#define HC32_EXTINT8_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch9 */
#define HC32_EXTINT9_IRQ_NUM             INT031_IRQn
#define HC32_EXTINT9_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
/* Ext ch10 */
#define HC32_EXTINT10_IRQ_NUM            INT032_IRQn
#define HC32_EXTINT10_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch11 */
#define HC32_EXTINT11_IRQ_NUM            INT033_IRQn
#define HC32_EXTINT11_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch12 */
#define HC32_EXTINT12_IRQ_NUM            INT034_IRQn
#define HC32_EXTINT12_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch13 */
#define HC32_EXTINT13_IRQ_NUM            INT035_IRQn
#define HC32_EXTINT13_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch14 */
#define HC32_EXTINT14_IRQ_NUM            INT036_IRQn
#define HC32_EXTINT14_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* Ext ch15 */
#define HC32_EXTINT15_IRQ_NUM            INT037_IRQn
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

#if defined(HC32_USING_UART1)
#define HC32_UART1_RXERR_IRQ_NUM         INT012_IRQn
#define HC32_UART1_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT
#define HC32_UART1_RX_IRQ_NUM            INT082_IRQn
#define HC32_UART1_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define HC32_UART1_TX_IRQ_NUM            INT081_IRQn
#define HC32_UART1_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#if defined(HC32_UART1_RX_USING_DMA)
#define HC32_UART1_RXTO_IRQ_NUM          INT013_IRQn
#define HC32_UART1_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT
#endif

#if defined(RT_USING_SERIAL_V1) && defined(HC32_UART1_TX_USING_MA)
#define HC32_UART1_TX_CPLT_IRQ_NUM       INT080_IRQn
#define HC32_UART1_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#elif defined(RT_USING_SERIAL_V2)
#define HC32_UART1_TX_CPLT_IRQ_NUM       INT080_IRQn
#define HC32_UART1_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif
#endif /* HC32_USING_UART1 */

#if defined(HC32_USING_UART2)
#define HC32_UART2_RXERR_IRQ_NUM         INT014_IRQn
#define HC32_UART2_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT
#define HC32_UART2_RX_IRQ_NUM            INT085_IRQn
#define HC32_UART2_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define HC32_UART2_TX_IRQ_NUM            INT084_IRQn
#define HC32_UART2_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#if defined(HC32_UART2_RX_USING_DMA)
#define HC32_UART2_RXTO_IRQ_NUM          INT015_IRQn
#define HC32_UART2_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT
#endif

#if defined(RT_USING_SERIAL_V1) && defined(HC32_UART2_TX_USING_MA)
#define HC32_UART2_TX_CPLT_IRQ_NUM       INT083_IRQn
#define HC32_UART2_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#elif defined(RT_USING_SERIAL_V2)
#define HC32_UART2_TX_CPLT_IRQ_NUM       INT083_IRQn
#define HC32_UART2_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif
#endif /* HC32_USING_UART2 */

#if defined(HC32_USING_UART3)
#define HC32_UART3_RXERR_IRQ_NUM         INT016_IRQn
#define HC32_UART3_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT
#define HC32_UART3_RX_IRQ_NUM            INT088_IRQn
#define HC32_UART3_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define HC32_UART3_TX_IRQ_NUM            INT087_IRQn
#define HC32_UART3_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#if defined(HC32_UART3_RX_USING_DMA)
#define HC32_UART3_RXTO_IRQ_NUM          INT017_IRQn
#define HC32_UART3_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT
#endif

#if defined(RT_USING_SERIAL_V1) && defined(HC32_UART3_TX_USING_DMA)
#define HC32_UART3_TX_CPLT_IRQ_NUM       INT086_IRQn
#define HC32_UART3_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#elif defined(RT_USING_SERIAL_V2)
#define HC32_UART3_TX_CPLT_IRQ_NUM       INT086_IRQn
#define HC32_UART3_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif
#endif /* HC32_USING_UART3 */

#if defined(HC32_USING_UART4)
#define HC32_UART4_RXERR_IRQ_NUM         INT018_IRQn
#define HC32_UART4_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT
#define HC32_UART4_RX_IRQ_NUM            INT091_IRQn
#define HC32_UART4_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define HC32_UART4_TX_IRQ_NUM            INT090_IRQn
#define HC32_UART4_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#if defined(HC32_UART4_RX_USING_DMA)
#define HC32_UART4_RXTO_IRQ_NUM          INT019_IRQn
#define HC32_UART4_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT
#endif

#if defined(RT_USING_SERIAL_V1) && defined(HC32_UART4_TX_USING_DMA)
#define HC32_UART4_TX_CPLT_IRQ_NUM       INT089_IRQn
#define HC32_UART4_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#elif defined(RT_USING_SERIAL_V2)
#define HC32_UART4_TX_CPLT_IRQ_NUM       INT089_IRQn
#define HC32_UART4_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif
#endif /* HC32_USING_UART4 */

#define HC32_SPI1_ERR_IRQ_NUM            INT008_IRQn
#define HC32_SPI1_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI1_RBF_IRQ_NUM            INT009_IRQn
#define HC32_SPI1_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI1_TBE_IRQ_NUM            INT010_IRQn
#define HC32_SPI1_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI2_ERR_IRQ_NUM            INT011_IRQn
#define HC32_SPI2_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI2_RBF_IRQ_NUM            INT012_IRQn
#define HC32_SPI2_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI2_TBE_IRQ_NUM            INT013_IRQn
#define HC32_SPI2_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI3_ERR_IRQ_NUM            INT014_IRQn
#define HC32_SPI3_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI3_RBF_IRQ_NUM            INT015_IRQn
#define HC32_SPI3_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI3_TBE_IRQ_NUM            INT016_IRQn
#define HC32_SPI3_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI4_ERR_IRQ_NUM            INT017_IRQn
#define HC32_SPI4_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI4_RBF_IRQ_NUM            INT018_IRQn
#define HC32_SPI4_RBF_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#define HC32_SPI4_TBE_IRQ_NUM            INT019_IRQn
#define HC32_SPI4_TBE_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#if defined(HC32_USING_CAN1)
#define HC32_CAN1_IRQ_NUM                INT126_IRQn
#define HC32_CAN1_IRQ_PRIO               DDL_IRQ_PRIO_DEFAULT
#endif /* HC32_USING_CAN1 */

#if defined(HC32_USING_SDIO1)
#define HC32_SDIO1_IRQ_NUM               INT122_IRQn
#define HC32_SDIO1_IRQ_PRIO              DDL_IRQ_PRIO_DEFAULT
#endif /* HC32_USING_SDIO1 */

#if defined(HC32_USING_SDIO2)
#define HC32_SDIO2_IRQ_NUM               INT124_IRQn
#define HC32_SDIO2_IRQ_PRIO              DDL_IRQ_PRIO_DEFAULT
#endif /* HC32_USING_SDIO2 */

/* RTC */
#define HC32_RTC_ALARM_IRQ_NUM           INT044_IRQn
#define HC32_RTC_ALARM_IRQ_PRIO          DDL_IRQ_PRIO_DEFAULT

#define HC32_RTC_PRD_IRQ_NUM             INT045_IRQn
#define HC32_RTC_PRD_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT

#if defined(HC32_USING_USBD) || defined(HSP_USING_USBH)
#define HC32_USB_GLB_IRQ_NUM             INT003_IRQn
#define HC32_USB_GLB_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#endif/* HC32_USING_USBD */

#if defined (HC32_USING_QSPI)
#define HC32_QSPI_ERR_IRQ_NUM            INT004_IRQn
#define HC32_QSPI_ERR_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#endif /* HC32_USING_QSPI */

#if defined(HC32_USING_PULSE_ENCODER_TMRA_1)
#define HC32_PULSE_ENCODER_TMRA_1_OVF_IRQ_NUM   INT080_IRQn
#define HC32_PULSE_ENCODER_TMRA_1_OVF_IRQ_PRIO  DDL_IRQ_PRIO_DEFAULT
#define HC32_PULSE_ENCODER_TMRA_1_UDF_IRQ_NUM   INT081_IRQn
#define HC32_PULSE_ENCODER_TMRA_1_UDF_IRQ_PRIO  DDL_IRQ_PRIO_DEFAULT
#endif/* HC32_USING_PULSE_ENCODER_TMRA_1 */

#if defined(HC32_USING_PULSE_ENCODER_TMR6_1)
#define HC32_PULSE_ENCODER_TMR6_1_OVF_IRQ_NUM   INT050_IRQn
#define HC32_PULSE_ENCODER_TMR6_1_OVF_IRQ_PRIO  DDL_IRQ_PRIO_DEFAULT
#define HC32_PULSE_ENCODER_TMR6_1_UDF_IRQ_NUM   INT051_IRQn
#define HC32_PULSE_ENCODER_TMR6_1_UDF_IRQ_PRIO  DDL_IRQ_PRIO_DEFAULT
#endif/* HC32_USING_PULSE_ENCODER_TMR6_1 */

#if defined(HC32_USING_TMR0_1A)
#define HC32_USING_TMR0_1A_IRQ_NUM       INT046_IRQn
#define HC32_USING_TMR0_1A_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif/* HC32_USING_TMR0_1A */
#if defined(HC32_USING_TMR0_1B)
#define HC32_USING_TMR0_1B_IRQ_NUM       INT047_IRQn
#define HC32_USING_TMR0_1B_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif/* HC32_USING_TMR0_1B */
#if defined(HC32_USING_TMR0_2A)
#define HC32_USING_TMR0_2A_IRQ_NUM       INT048_IRQn
#define HC32_USING_TMR0_2A_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif/* HC32_USING_TMR0_2A */
#if defined(HC32_USING_TMR0_2B)
#define HC32_USING_TMR0_2B_IRQ_NUM       INT049_IRQn
#define HC32_USING_TMR0_2B_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif/* HC32_USING_TMR0_2B */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_HC32_INTC_H_ */
