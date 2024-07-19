/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_TIMER_HC32_TIMER_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_TIMER_HC32_TIMER_H_


/* Timer count clock division values */
#define HC32_TMR_CNT_CLK_DIV1              (0U)
#define HC32_TMR_CNT_CLK_DIV2              (1U)
#define HC32_TMR_CNT_CLK_DIV4              (2U)
#define HC32_TMR_CNT_CLK_DIV8              (3U)
#define HC32_TMR_CNT_CLK_DIV16             (4U)
#define HC32_TMR_CNT_CLK_DIV32             (5U)
#define HC32_TMR_CNT_CLK_DIV64             (6U)
#define HC32_TMR_CNT_CLK_DIV128            (7U)
#define HC32_TMR_CNT_CLK_DIV256            (8U)
#define HC32_TMR_CNT_CLK_DIV512            (9U)
#define HC32_TMR_CNT_CLK_DIV1024           (10U)

/* Timer6 count clock division values for HC32 460/45x series */
#define HC32_460_45X_TMR6_CNT_CLK_DIV1     (0U)
#define HC32_460_45X_TMR6_CNT_CLK_DIV2     (1U)
#define HC32_460_45X_TMR6_CNT_CLK_DIV4     (2U)
#define HC32_460_45X_TMR6_CNT_CLK_DIV8     (3U)
#define HC32_460_45X_TMR6_CNT_CLK_DIV16    (4U)
#define HC32_460_45X_TMR6_CNT_CLK_DIV64    (5U)
#define HC32_460_45X_TMR6_CNT_CLK_DIV256   (6U)
#define HC32_460_45X_TMR6_CNT_CLK_DIV1024  (7U)

/* Timer count mode values */
#define HC32_TMR_CNT_MD_SAWTOOTH        (0x0000U)
#define HC32_TMR_CNT_MD_TRIANGLE        (0x0001U)

/* Timer count direction values */
#define HC32_TMR_CNT_DIR_UP             (0x0000U)
#define HC32_TMR_CNT_DIR_DOWN           (0x0001U)

/* Timer6 capture channel define */
#define CHANNEL_CAPTURE_TMR6_PWMA       (0x01U)
#define CHANNEL_CAPTURE_TMR6_PWMB       (0x02U)
#define CHANNEL_CAPTURE_TMR6_TRIGA      (0x03U)
#define CHANNEL_CAPTURE_TMR6_TRIGB      (0x04U)
#define CHANNEL_CAPTURE_TMR6_TRIGC      (0x05U)
#define CHANNEL_CAPTURE_TMR6_TRIGD      (0x06U)
#if defined(HC32F460)
#define CHANNEL_CAPTURE_TMR6_MAX_CH     (0x05U)
#elif defined(HC32F4A0)
#define CHANNEL_CAPTURE_TMR6_MAX_CH     (0x07U)
#endif

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_TIMER_HC32_TIMER_H_ */
