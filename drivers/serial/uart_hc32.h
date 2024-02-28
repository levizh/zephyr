/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for UART port on HC32 family processor.
 *
 */

#ifndef ZEPHYR_DRIVERS_SERIAL_UART_HC32_H_
#define ZEPHYR_DRIVERS_SERIAL_UART_HC32_H_

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/uart.h>

#include <hc32_ll_usart.h>

#define HC32_UART_DEFAULT_BAUDRATE	115200
#define HC32_UART_DEFAULT_PARITY	UART_CFG_PARITY_NONE
#define HC32_UART_DEFAULT_STOP_BITS	UART_CFG_STOP_BITS_1
#define HC32_UART_DEFAULT_DATA_BITS	UART_CFG_DATA_BITS_8

#if CONFIG_UART_INTERRUPT_DRIVEN
enum UART_TYPE{
#ifdef CONFIG_UART_EI_INTERRUPT_DRIVEN
    UART_INT_IDX_EI,
#endif /* CONFIG_UART_EI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_RI_INTERRUPT_DRIVEN
    UART_INT_IDX_RI,
#endif /* CONFIG_UART_RI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_TI_INTERRUPT_DRIVEN
    UART_INT_IDX_TI,
#endif /* CONFIG_UART_TI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_TCI_INTERRUPT_DRIVEN
    UART_INT_IDX_TCI,
 #endif /* CONFIG_UART_TI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_RTO_INTERRUPT_DRIVEN
    UART_INT_IDX_RTO,
#endif /* CONFIG_UART_RTO_INTERRUPT_DRIVEN */

    UART_INT_NUM
};
#endif /* UART_INTERRUPT_DRIVEN */

/* device config */
struct uart_hc32_config {
	/* USART instance */
	CM_USART_TypeDef *usart;
	/* clock subsystem driving this peripheral */
	const struct hc32_pclken *pclken;
	/* number of clock subsystems */
	size_t pclk_len;
	/* switch to enable single wire / half duplex feature */
	bool single_wire;
	/* pin muxing */
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_ASYNC_API
struct uart_dma_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	int fifo_threshold;
	struct dma_block_config blk_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	size_t offset;
	volatile size_t counter;
	int32_t timeout;
	struct k_work_delayable timeout_work;
	bool enabled;
};
#endif

struct hc32_usart_cb_data {
    /** Callback function */
    uart_irq_callback_user_data_t cb;
    /** User data. */
    void *user_data;
};

/* driver data */
struct uart_hc32_data {
	/* clock device */
	const struct device *clock;
	/* uart config */
	struct uart_config *uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	struct hc32_usart_cb_data user_cb[UART_INT_NUM];
#endif

#ifdef CONFIG_UART_ASYNC_API
	const struct device *uart_dev;
	uart_callback_t async_cb;
	void *async_user_data;
	struct uart_dma_stream dma_rx;
	struct uart_dma_stream dma_tx;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
#endif
};

#endif	/* ZEPHYR_DRIVERS_SERIAL_UART_HC32_H_ */
