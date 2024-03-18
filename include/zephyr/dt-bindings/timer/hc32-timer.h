/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_TIMER_HC32_TIMER_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_TIMER_HC32_TIMER_H_


/* Timer count clock division values */
#define HC32_TMR_CNT_CLK_DIV1           (0U)
#define HC32_TMR_CNT_CLK_DIV2           (1U)
#define HC32_TMR_CNT_CLK_DIV4           (2U)
#define HC32_TMR_CNT_CLK_DIV8           (3U)
#define HC32_TMR_CNT_CLK_DIV16          (4U)
#define HC32_TMR_CNT_CLK_DIV32          (5U)
#define HC32_TMR_CNT_CLK_DIV64          (6U)
#define HC32_TMR_CNT_CLK_DIV128         (7U)
#define HC32_TMR_CNT_CLK_DIV256         (8U)
#define HC32_TMR_CNT_CLK_DIV512         (9U)
#define HC32_TMR_CNT_CLK_DIV1024        (10U)

/* Timer count mode values */
#define HC32_TMR_CNT_MD_SAWTOOTH        0x0000U
#define HC32_TMR_CNT_MD_TRIANGLE        0x0001U

/* Timer count direction values */
#define HC32_TMR_CNT_DIR_UP             0x0000U
#define HC32_TMR_CNT_DIR_DOWN           0x0001U

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_TIMER_HC32_TIMER_H_ */
