/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_QSPI_QSPI_HC32_H_
#define ZEPHYR_DRIVERS_QSPI_QSPI_HC32_H_

#include <spi.h>
#include "spi_context.h"
#include <clock_control/hc32_clock_control.h>
#include <clock_control.h>
#include <dma/dma_hc32.h>
#include <dt-bindings/dma/hc32_dma.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (HC32F460) || defined (HC32F4A0)
#define HC32_QSPI_PSC_MAX           64
#endif

#define ROM_BLOCK_SIZE              0x04000000u

#ifdef CONFIG_QSPI_HC32_DMA
#define HC32_DMA_MAX_BLOCK_CNT      (65535U)

#define TX_OPERATION                0x01
#define RX_OPERATION                0X02
#define QSPI_HC32_DMA_ERROR_FLAG    0x01
#define QSPI_HC32_DMA_RX_DONE_FLAG  0x02
#define QSPI_HC32_DMA_TX_DONE_FLAG  0x04

struct qspi_dma_config {
	struct device *dev;
	uint8_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	struct dma_hc32_config_user_data dma_cfg_user_data;
};

#endif /* CONFIG_QSPI_HC32_DMA */

struct qspi_instruction {
	uint8_t content;
	uint8_t lines;
};

/* address */
struct qspi_address {
	uint32_t content;
	uint8_t bytes;
	uint8_t lines;
};

/* dummy cycles */
struct qspi_dummy {
	/* default XIP_Enable = 0u, XIP mode disabled */
	uint8_t content;
	uint8_t dummy_cycles;
	uint8_t XIP_Enable;
};

/* Private structures */
struct qspi_hc32_config {
	/* please set cs cfg(port, pin) same with pinmux.c files */
	struct spi_config config;
	/* if no instruction, please set NULL */
	struct qspi_instruction *instruction;
	/* if no address, please set NULL */
	struct qspi_address *address;
	/* if no dummy cycles, please set NULL */
	struct qspi_dummy *dummy;

	/* number of lines in qspi data stage */
	uint8_t data_lines;
};

struct qspi_hc32_cfg {
	struct hc32_modules_clock_sys qspi_clk;
};

struct qspi_hc32_data {
	struct spi_context ctx;
#ifdef CONFIG_QSPI_HC32_DMA
	struct k_sem status_sem;
	volatile uint32_t status_flags;
	struct qspi_dma_config dma_rx;
	struct qspi_dma_config dma_tx;
#endif
};

static inline size_t spi_context_all_buf_length(const struct spi_buf_set *bufs)
{
	size_t len = 0;
	if (bufs) {
		for (uint8_t i = 0; i < bufs->count; i++) {
			len += (bufs->buffers[i]).len;
		}
	}
	return len;
}

#ifdef __cplusplus
}
#endif
#endif /* ZEPHYR_DRIVERS_QSPI_QSPI_HC32_H_ */
