/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_uart

/**
 * @brief Driver for UART port on HC32 family processor.
 * @note  LPUART and U(S)ART have the same base and
 *		majority of operations are performed the same way.
 *		Please validate for newly added series.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>
#include <zephyr/init.h>
// #include <zephyr/drivers/interrupt_controller/exti_hc32.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device.h>

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma/dma_hc32.h>
#include <zephyr/drivers/dma.h>
#endif

#include <zephyr/linker/sections.h>
#include "uart_hc32.h"
#include <hc32_ll_usart.h>
#include <hc32_ll_fcg.h>
#include <hc32_ll_interrupts.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(uart_hc32, CONFIG_UART_LOG_LEVEL);

/* This symbol takes the value 1 if one of the device instances */
/* is configured in dts with a domain clock */
#if HC32_DT_INST_DEV_DOMAIN_CLOCK_SUPPORT
#define HC32_UART_DOMAIN_CLOCK_SUPPORT 1
#else
#define HC32_UART_DOMAIN_CLOCK_SUPPORT 0
#endif

static inline void uart_hc32_set_baudrate(const struct device *dev, uint32_t baud_rate)
{
	// const struct uart_hc32_config *config = dev->config;
	// struct uart_hc32_data *data = dev->data;

}

static inline void uart_hc32_set_parity(const struct device *dev,
					 uint32_t parity)
{
	const struct uart_hc32_config *config = dev->config;

	USART_SetParity(config->usart, parity);
}

static inline uint32_t uart_hc32_get_parity(const struct device *dev)
{
	// const struct uart_hc32_config *config = dev->config;

	return 0;
}

static inline void uart_hc32_set_stopbits(const struct device *dev,
					   uint32_t stopbits)
{
	const struct uart_hc32_config *config = dev->config;

	USART_SetStopBit(config->usart, stopbits);
}

static inline uint32_t uart_hc32_get_stopbits(const struct device *dev)
{
	// const struct uart_hc32_config *config = dev->config;

	return 0;
}

static inline void uart_hc32_set_databits(const struct device *dev,
					   uint32_t databits)
{
	const struct uart_hc32_config *config = dev->config;

	USART_SetDataWidth(config->usart, databits);
}

static inline uint32_t uart_hc32_get_databits(const struct device *dev)
{
	// const struct uart_hc32_config *config = dev->config;

	return 0;
}

static inline void uart_hc32_set_hwctrl(const struct device *dev,
					 uint32_t hwctrl)
{
	const struct uart_hc32_config *config = dev->config;

	USART_SetHWFlowControl(config->usart, hwctrl);
}

static inline uint32_t uart_hc32_get_hwctrl(const struct device *dev)
{
	// const struct uart_hc32_config *config = dev->config;

	return 0;
}

static inline uint32_t uart_hc32_cfg2ll_parity(enum uart_config_parity parity)
{
	switch (parity) {
	case UART_CFG_PARITY_ODD:
		return USART_PARITY_ODD;
	case UART_CFG_PARITY_EVEN:
		return USART_PARITY_EVEN;
	case UART_CFG_PARITY_NONE:
	default:
		return USART_PARITY_NONE;
	}
}

static inline enum uart_config_parity uart_hc32_ll2cfg_parity(uint32_t parity)
{
	switch (parity) {
	case USART_PARITY_ODD:
		return UART_CFG_PARITY_ODD;
	case USART_PARITY_EVEN:
		return UART_CFG_PARITY_EVEN;
	case USART_PARITY_NONE:
	default:
		return UART_CFG_PARITY_NONE;
	}
}

static inline uint32_t uart_hc32_cfg2ll_stopbits(
		enum uart_config_stop_bits sb)
{

	switch (sb) {
	/* Some MCU's don't support 0.5 stop bits */
#ifdef USART_STOPBIT_2BIT
	case UART_CFG_STOP_BITS_2:
		return USART_STOPBIT_2BIT;
#endif	/* USART_STOPBIT_2BIT */

#ifdef USART_STOPBIT_1BIT
	case UART_CFG_STOP_BITS_1:
#endif	/* USART_STOPBIT_1BIT */
	default:
		return USART_STOPBIT_1BIT;
		}
}

static inline enum uart_config_stop_bits uart_hc32_ll2cfg_stopbits(uint32_t sb)
{
	switch (sb) {
#ifdef USART_STOPBIT_2BIT
	case USART_STOPBIT_2BIT:
		return UART_CFG_STOP_BITS_2;
#endif	/* USART_STOPBIT_1BIT */

#ifdef USART_STOPBIT_1BIT
		case USART_STOPBIT_1BIT:
#endif	/* USART_STOPBIT_1BIT */
		default:
			return UART_CFG_STOP_BITS_1;
	}
}

static inline uint32_t uart_hc32_cfg2ll_databits(enum uart_config_data_bits db)
{
	switch (db) {
/* Some MCU's don't support 9B datawidth */
#ifdef USART_DATA_WIDTH_9BIT
	case UART_CFG_DATA_BITS_9:
		return USART_DATA_WIDTH_9BIT;
#endif	/* USART_DATA_WIDTH_9BIT */
	case UART_CFG_DATA_BITS_8:
	default:
		return USART_DATA_WIDTH_8BIT;
	}
}

static inline enum
	uart_config_data_bits uart_hc32_ll2cfg_databits(uint32_t db, uint32_t p)
{
	switch (db) {
/* Some MCU's don't support 9B datawidth */
#ifdef USART_DATA_WIDTH_9BIT
	case USART_DATA_WIDTH_9BIT:
		return UART_CFG_DATA_BITS_9;
#endif	/* USART_DATA_WIDTH_9BIT */
	case USART_DATA_WIDTH_8BIT:
	default:
		return UART_CFG_DATA_BITS_8;
	}
}

/**
 * @brief  Get LL hardware flow control define from
 *         Zephyr hardware flow control option.
 * @note   Supports only UART_CFG_FLOW_CTRL_RTS_CTS and UART_CFG_FLOW_CTRL_RS485.
 * @param  fc: Zephyr hardware flow control option.
 * @retval LL_USART_HWCONTROL_RTS_CTS, or LL_USART_HWCONTROL_NONE.
 */
static inline uint32_t uart_hc32_cfg2ll_hwctrl(enum uart_config_flow_control fc)
{
	// if (fc == UART_CFG_FLOW_CTRL_RTS_CTS) {
	// 	return USART_HW_FLOWCTRL_CTS;
	// } else if (fc == UART_CFG_FLOW_CTRL_RS485) {
	// 	/* Driver Enable is handled separately */
	// 	return LL_USART_HWCONTROL_NONE;
	// }

	return USART_HW_FLOWCTRL_CTS;
}

/**
 * @brief  Get Zephyr hardware flow control option from
 *         LL hardware flow control define.
 * @note   Supports only LL_USART_HWCONTROL_RTS_CTS.
 * @param  fc: LL hardware flow control definition.
 * @retval UART_CFG_FLOW_CTRL_RTS_CTS, or UART_CFG_FLOW_CTRL_NONE.
 */
static inline enum uart_config_flow_control uart_hc32_ll2cfg_hwctrl(uint32_t fc)
{
	// if (fc == LL_USART_HWCONTROL_RTS_CTS) {
	// 	return UART_CFG_FLOW_CTRL_RTS_CTS;
	// }

	return UART_CFG_FLOW_CTRL_NONE;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_hc32_configure(const struct device *dev,
				const struct uart_config *cfg)
{
	// const struct uart_hc32_config *config = dev->config;
	struct uart_hc32_data *data = dev->data;
	struct uart_config *uart_cfg = data->uart_cfg;
	const uint32_t parity = uart_hc32_cfg2ll_parity(cfg->parity);
	const uint32_t stopbits = uart_hc32_cfg2ll_stopbits(cfg->stop_bits);
	const uint32_t databits = uart_hc32_cfg2ll_databits(cfg->data_bits);

	/* Hardware doesn't support mark or space parity */
	if ((cfg->parity == UART_CFG_PARITY_MARK) ||
		(cfg->parity == UART_CFG_PARITY_SPACE)) {
		return -ENOTSUP;
	}

	/* Driver does not supports parity + 9 databits */
	if ((cfg->parity != UART_CFG_PARITY_NONE) &&
		(cfg->data_bits == USART_DATA_WIDTH_9BIT)) {
		return -ENOTSUP;
	}

	/* When the transformed ll stop bits don't match with what was requested, then it's not
	 * supported
	 */
	if (uart_hc32_ll2cfg_stopbits(stopbits) != cfg->stop_bits) {
		return -ENOTSUP;
	}

	/* When the transformed ll databits don't match with what was requested, then it's not
	 * supported
	 */
	if (uart_hc32_ll2cfg_databits(databits, parity) != cfg->data_bits) {
		return -ENOTSUP;
	}

// 	/* Driver supports only RTS/CTS and RS485 flow control */
// 	if (!(cfg->flow_ctrl == UART_CFG_FLOW_CTRL_NONE
// 		|| (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_RTS_CTS &&
// 			IS_UART_HWFLOW_INHCANCE(config->usart))
// #if HAS_DRIVER_ENABLE
// 		|| (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_RS485 &&
// 			IS_UART_DRIVER_ENABLE_INHCANCE(config->usart))
// #endif
// 		)) {
// 		return -ENOTSUP;
// 	}

	// USART_FuncCmd(config->usart, DISABLE);

	// USART_FuncCmd(config->usart, ENABLE);

	/* Upon successful configuration, persist the syscall-passed
	 * uart_config.
	 * This allows restoring it, should the device return from a low-power
	 * mode in which register contents are lost.
	 */
	*uart_cfg = *cfg;

	return 0;
}

static int uart_hc32_config_get(const struct device *dev,
				 struct uart_config *cfg)
{
	struct uart_hc32_data *data = dev->data;
	struct uart_config *uart_cfg = data->uart_cfg;

	cfg->baudrate = uart_cfg->baudrate;
	cfg->parity = uart_hc32_ll2cfg_parity(uart_hc32_get_parity(dev));
	cfg->stop_bits = uart_hc32_ll2cfg_stopbits(
		uart_hc32_get_stopbits(dev));
	cfg->data_bits = uart_hc32_ll2cfg_databits(
		uart_hc32_get_databits(dev), uart_hc32_get_parity(dev));
	cfg->flow_ctrl = uart_hc32_ll2cfg_hwctrl(
		uart_hc32_get_hwctrl(dev));
#if HAS_DRIVER_ENABLE
	if (uart_hc32_get_driver_enable(dev)) {
		cfg->flow_ctrl = UART_CFG_FLOW_CTRL_RS485;
	}
#endif
	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

// typedef void (*poll_in_fn)(
// 	const struct uart_hc32_config *config,
// 	void *in);

// static int uart_hc32_poll_in_visitor(const struct device *dev, void *in,
// 			poll_in_fn get_fn)
// {
// 	const struct uart_hc32_config *config = dev->config;

// 	// /* Clear overrun error flag */
// 	// if (LL_USART_IsActiveFlag_ORE(config->usart)) {
// 	// 	LL_USART_ClearFlag_ORE(config->usart);
// 	// }

// 	// /*
// 	//  * On hc32 F4X, F1X, and F2X, the RXNE flag is affected (cleared) by
// 	//  * the uart_err_check function call (on errors flags clearing)
// 	//  */
// 	// if (!USART_GetStatus(config->usart, USART_FLAG_RX_FULL)) {
// 	// 	return -1;
// 	// }

// 	get_fn(config, in);

// 	return 0;
// }

// typedef void (*poll_out_fn)(
// 	const struct uart_hc32_config *config, void *out);

// static void uart_hc32_poll_out_visitor(const struct device *dev, void *out,
// 			poll_out_fn set_fn)
// {
// 	const struct uart_hc32_config *config = dev->config;
// 	unsigned int key;

// 	/* Wait for TXE flag to be raised
// 	 * When TXE flag is raised, we lock interrupts to prevent interrupts (notably that of usart)
// 	 * or thread switch. Then, we can safely send our character. The character sent will be
// 	 * interlaced with the characters potentially send with interrupt transmission API
// 	 */
// // 	while (1) {
// // 		if (USART_GetStatus(config->usart, USART_FLAG_TX_CPLT)) {
// // 			key = irq_lock();
// // 			if (USART_GetStatus(config->usart, USART_FLAG_TX_CPLT)) {
// // 				break;
// // 			}
// // 			irq_unlock(key);
// // 		}
// // 	}

// 	set_fn(config, out);
// 	irq_unlock(key);
// }


static int uart_hc32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_hc32_config *config = dev->config;

	if (SET != USART_GetStatus(config->usart, USART_FLAG_RX_FULL))
	{
		return EIO;
	}

	if (USART_GetStatus(config->usart, USART_FLAG_OVERRUN | \
										USART_FLAG_RX_TIMEOUT)) {
		return EIO;
	}

	*c = (uint8_t)(USART_ReadData(config->usart) & 0xFFU);
	USART_ClearStatus(config->usart, USART_FLAG_RX_FULL);

	return 0;
}

static void uart_hc32_poll_out(const struct device *dev, unsigned char c)
{
	unsigned int key;
	const struct uart_hc32_config *config = dev->config;
	// (void)USART_UART_Trans(config->usart, (void *)&c, 1U, 0xFF);
	key = irq_lock();
	while (1)
	{
		if (USART_GetStatus(config->usart, USART_FLAG_TX_EMPTY)) {
			break;
		}
	}
	USART_WriteData(config->usart, (uint16_t)c);
	irq_unlock(key);
}

#ifdef CONFIG_UART_WIDE_DATA

static void poll_in_u9(const struct uart_hc32_config *config, void *in)
{
	// *((uint16_t *)in) = LL_USART_ReceiveData9(config->usart);
	return 0;
}

static void poll_out_u9(const struct uart_hc32_config *config, void *out)
{
	(void)USART_UART_Trans(config->usart, out, 2U, 0xFF);
}

#endif /* CONFIG_UART_WIDE_DATA */

static int uart_hc32_err_check(const struct device *dev)
{
	// const struct uart_hc32_config *config = dev->config;
	uint32_t err = 0U;

	/* Check for errors, then clear them.
	 * Some SoC clear all error flags when at least
	 * one is cleared. (e.g. F4X, F1X, and F2X).
	 * The hc32 F4X, F1X, and F2X also reads the usart DR when clearing Errors
	 */
// 	if (LL_USART_IsActiveFlag_ORE(config->usart)) {
// 		err |= UART_ERROR_OVERRUN;
// 	}

// 	if (LL_USART_IsActiveFlag_PE(config->usart)) {
// 		err |= UART_ERROR_PARITY;
// 	}

// 	if (LL_USART_IsActiveFlag_FE(config->usart)) {
// 		err |= UART_ERROR_FRAMING;
// 	}

// 	if (LL_USART_IsActiveFlag_NE(config->usart)) {
// 		err |= UART_ERROR_NOISE;
// 	}

// #if !defined(CONFIG_SOC_SERIES_HC32F0X) || defined(USART_LIN_SUPPORT)
// 	if (LL_USART_IsActiveFlag_LBD(config->usart)) {
// 		err |= UART_BREAK;
// 	}

// 	if (err & UART_BREAK) {
// 		LL_USART_ClearFlag_LBD(config->usart);
// 	}
// #endif
// 	/* Clearing error :
// 	 * the hc32 F4X, F1X, and F2X sw sequence is reading the usart SR
// 	 * then the usart DR to clear the Error flags ORE, PE, FE, NE
// 	 * --> so is the RXNE flag also cleared !
// 	 */
// 	if (err & UART_ERROR_OVERRUN) {
// 		LL_USART_ClearFlag_ORE(config->usart);
// 	}

// 	if (err & UART_ERROR_PARITY) {
// 		LL_USART_ClearFlag_PE(config->usart);
// 	}

// 	if (err & UART_ERROR_FRAMING) {
// 		LL_USART_ClearFlag_FE(config->usart);
// 	}

// 	if (err & UART_ERROR_NOISE) {
// 		LL_USART_ClearFlag_NE(config->usart);
// 	}

	return err;
}

static inline void __uart_hc32_get_clock(const struct device *dev)
{
	// struct uart_hc32_data *data = dev->data;
	// const struct device *const clk = DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE);

	// data->clock = clk;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

typedef void (*fifo_fill_fn)(const struct uart_hc32_config *config, const void *tx_data,
				 const uint8_t offset);

static int uart_hc32_fifo_fill_visitor(const struct device *dev,
			const void *tx_data, int size, fifo_fill_fn fill_fn)
{
	// const struct uart_hc32_config *config = dev->config;
	uint8_t num_tx = 0U;
	// unsigned int key;

	// if (!USART_GetStatus(config->usart, USART_FLAG_TX_CPLT)) {
	// 	return num_tx;
	// }

	/* Lock interrupts to prevent nested interrupts or thread switch */
	// key = irq_lock();

	// while ((size - num_tx > 0) && USART_GetStatus(config->usart, USART_FLAG_TX_CPLT)) {
	// 	/* TXE flag will be cleared with byte write to DR|RDR register */

	// 	/* Send a character */
	// 	fill_fn(config, tx_data, num_tx);
	// 	num_tx++;
	// }

	// irq_unlock(key);

	return num_tx;
}

static void fifo_fill_with_u8(const struct uart_hc32_config *config,
					 const void *tx_data, const uint8_t offset)
{
	// const uint8_t *data = (const uint8_t *)tx_data;
	/* Send a character (8bit) */
	// LL_USART_TransmitData8(config->usart, data[offset]);
}

static int uart_hc32_fifo_fill(const struct device *dev,
			const uint8_t *tx_data, int size)
{
	if (uart_hc32_ll2cfg_databits(uart_hc32_get_databits(dev),
			uart_hc32_get_parity(dev)) == USART_DATA_WIDTH_9BIT) {
		return -ENOTSUP;
	}
	return uart_hc32_fifo_fill_visitor(dev, (const void *)tx_data, size,
						fifo_fill_with_u8);
}

typedef void (*fifo_read_fn)(const struct uart_hc32_config *config, void *rx_data,
				 const uint8_t offset);

static int uart_hc32_fifo_read_visitor(const struct device *dev,
			void *rx_data, const int size, fifo_read_fn read_fn)
{
	// const struct uart_hc32_config *config = dev->config;
	uint8_t num_rx = 0U;

	// while ((size - num_rx > 0) && USART_GetStatus(config->usart, USART_FLAG_RX_FULL)) {
	// 	/* RXNE flag will be cleared upon read from DR|RDR register */

	// 	read_fn(config, rx_data, num_rx);
	// 	num_rx++;

	// 	/* Clear overrun error flag */
	// 	if (LL_USART_IsActiveFlag_ORE(config->usart)) {
	// 		LL_USART_ClearFlag_ORE(config->usart);
	// 		/*
	// 		 * On hc32 F4X, F1X, and F2X, the RXNE flag is affected (cleared) by
	// 		 * the uart_err_check function call (on errors flags clearing)
	// 		 */
	// 	}
	// }

	return num_rx;
}

static void fifo_read_with_u8(const struct uart_hc32_config *config,
				void *rx_data, const uint8_t offset)
{
	uint8_t *data = (uint8_t *)rx_data;

	USART_UART_Receive(config->usart, &data[offset], 1U, 0xFFU);
}

static int uart_hc32_fifo_read(const struct device *dev,
			uint8_t *rx_data, const int size)
{
	if (uart_hc32_ll2cfg_databits(uart_hc32_get_databits(dev),
			uart_hc32_get_parity(dev)) == USART_DATA_WIDTH_9BIT) {
		return -ENOTSUP;
	}
	return uart_hc32_fifo_read_visitor(dev, (void *)rx_data, size,
						fifo_read_with_u8);
}

// #ifdef CONFIG_UART_WIDE_DATA

// static void fifo_fill_with_u16(const struct uart_hc32_config *config,
// 					  const void *tx_data, const uint8_t offset)
// {
// 	const uint16_t *data = (const uint16_t *)tx_data;

// 	/* Send a character (9bit) */
// 	LL_USART_TransmitData9(config->usart, data[offset]);
// }

// static int uart_hc32_fifo_fill_u16(const struct device *dev, const uint16_t *tx_data, int size)
// {
// 	if (uart_hc32_ll2cfg_databits(uart_hc32_get_databits(dev), uart_hc32_get_parity(dev)) !=
// 		USART_DATA_WIDTH_9BIT) {
// 		return -ENOTSUP;
// 	}
// 	return uart_hc32_fifo_fill_visitor(dev, (const void *)tx_data, size,
// 						fifo_fill_with_u16);
// }

// static void fifo_read_with_u16(const struct uart_hc32_config *config, void *rx_data,
// 					  const uint8_t offset)
// {
// 	uint16_t *data = (uint16_t *)rx_data;

// 	data[offset] = LL_USART_ReceiveData9(config->usart);
// }

// static int uart_hc32_fifo_read_u16(const struct device *dev, uint16_t *rx_data, const int size)
// {
// 	if (uart_hc32_ll2cfg_databits(uart_hc32_get_databits(dev), uart_hc32_get_parity(dev)) !=
// 		USART_DATA_WIDTH_9BIT) {
// 		return -ENOTSUP;
// 	}
// 	return uart_hc32_fifo_read_visitor(dev, (void *)rx_data, size,
// 						fifo_read_with_u16);
// }

// #endif

static void uart_hc32_irq_tx_enable(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;
	struct uart_hc32_data *data = dev->data;
	stc_irq_signin_config_t stcIrqSigninConfig;

	// stcIrqSigninConfig.enIRQn = config->irq;
	// stcIrqSigninConfig.enIntSrc = INT_SRC_USART4_TI;
	// stcIrqSigninConfig.pfnCallback = NULL;
	// (void)INTC_IrqSignOut(stcIrqSigninConfig.enIRQn);
	// (void)INTC_IrqSignIn(&stcIrqSigninConfig);
#ifdef CONFIG_UART_TI_INTERRUPT_DRIVEN
	USART_FuncCmd(config->usart, USART_INT_TX_EMPTY, ENABLE);
#endif /* CONFIG_UART_TI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_TCI_INTERRUPT_DRIVEN
	USART_FuncCmd(config->usart, USART_INT_TX_CPLT, ENABLE);
#endif /* CONFIG_UART_TCI_INTERRUPT_DRIVEN */
}

static void uart_hc32_irq_tx_disable(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;

#ifdef CONFIG_UART_TI_INTERRUPT_DRIVEN
	USART_FuncCmd(config->usart, USART_INT_TX_EMPTY, DISABLE);
#endif /* CONFIG_UART_TI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_TCI_INTERRUPT_DRIVEN
	USART_FuncCmd(config->usart, USART_INT_TX_CPLT, DISABLE);
#endif /* CONFIG_UART_TCI_INTERRUPT_DRIVEN */
}

static int uart_hc32_irq_tx_ready(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;

	return USART_GetStatus(config->usart, USART_FLAG_TX_EMPTY);
}

static int uart_hc32_irq_tx_complete(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;

	return USART_GetStatus(config->usart, USART_FLAG_TX_CPLT);
}

static void uart_hc32_irq_rx_enable(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;
	stc_irq_signin_config_t stcIrqSigninConfig;

	// stcIrqSigninConfig.enIRQn = config->irq;
	// stcIrqSigninConfig.enIntSrc = INT_SRC_USART4_RI;
	// stcIrqSigninConfig.pfnCallback = NULL;
	// (void)INTC_IrqSignOut(stcIrqSigninConfig.enIRQn);
	// (void)INTC_IrqSignIn(&stcIrqSigninConfig);
	USART_FuncCmd(config->usart, USART_INT_RX, ENABLE);
}

static void uart_hc32_irq_rx_disable(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;

	USART_FuncCmd(config->usart, USART_INT_RX, DISABLE);
}

static int uart_hc32_irq_rx_ready(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;
	/*
	 * On hc32 F4X, F1X, and F2X, the RXNE flag is affected (cleared) by
	 * the uart_err_check function call (on errors flags clearing)
	 */
	return USART_GetStatus(config->usart, USART_FLAG_RX_FULL);
}

static void uart_hc32_irq_err_enable(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;

	/* Enable FE, ORE interruptions */

	/* Enable parity error interruption */
	USART_FuncCmd(config->usart, USART_RX_TIMEOUT |
					 USART_INT_RX_TIMEOUT, ENABLE);
}

static void uart_hc32_irq_err_disable(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;

	USART_FuncCmd(config->usart, USART_RX_TIMEOUT |
					 USART_INT_RX_TIMEOUT, DISABLE);
}

static int uart_hc32_irq_is_pending(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;

	return USART_GetStatus(config->usart, USART_FLAG_ALL);
}

static int uart_hc32_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_hc32_irq_callback_set(const struct device *dev,
					uart_irq_callback_user_data_t cb,
					void *cb_data)
{
	int i;
	struct uart_hc32_data *data = dev->data;

	for (i = 0; i < 5; i++)
	{
		data->user_cb[i].cb = cb;
		data->user_cb[i].user_data = cb_data;
	}

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	data->async_cb = NULL;
	data->async_user_data = NULL;
#endif
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API

static inline void async_user_callback(struct uart_hc32_data *data,
				struct uart_event *event)
{
	if (data->async_cb) {
		data->async_cb(data->uart_dev, event, data->async_user_data);
	}
}

static inline void async_evt_rx_rdy(struct uart_hc32_data *data)
{
	//LOG_DBG("rx_rdy: (%d %d)", data->dma_rx.offset, data->dma_rx.counter);

	struct uart_event event = {
		.type = UART_RX_RDY,
		.data.rx.buf = data->dma_rx.buffer,
		.data.rx.len = data->dma_rx.counter - data->dma_rx.offset,
		.data.rx.offset = data->dma_rx.offset
	};

	/* update the current pos for new data */
	data->dma_rx.offset = data->dma_rx.counter;

	/* send event only for new data */
	if (event.data.rx.len > 0) {
		async_user_callback(data, &event);
	}
}

static inline void async_evt_rx_err(struct uart_hc32_data *data, int err_code)
{
	//LOG_DBG("rx error: %d", err_code);

	struct uart_event event = {
		.type = UART_RX_HCOPPED,
		.data.rx_stop.reason = err_code,
		.data.rx_stop.data.len = data->dma_rx.counter,
		.data.rx_stop.data.offset = 0,
		.data.rx_stop.data.buf = data->dma_rx.buffer
	};

	async_user_callback(data, &event);
}

static inline void async_evt_tx_done(struct uart_hc32_data *data)
{
	//LOG_DBG("tx done: %d", data->dma_tx.counter);

	struct uart_event event = {
		.type = UART_TX_DONE,
		.data.tx.buf = data->dma_tx.buffer,
		.data.tx.len = data->dma_tx.counter
	};

	/* Reset tx buffer */
	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;

	async_user_callback(data, &event);
}

static inline void async_evt_tx_abort(struct uart_hc32_data *data)
{
	//LOG_DBG("tx abort: %d", data->dma_tx.counter);

	struct uart_event event = {
		.type = UART_TX_ABORTED,
		.data.tx.buf = data->dma_tx.buffer,
		.data.tx.len = data->dma_tx.counter
	};

	/* Reset tx buffer */
	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;

	async_user_callback(data, &event);
}

static inline void async_evt_rx_buf_request(struct uart_hc32_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEHC,
	};

	async_user_callback(data, &evt);
}

static inline void async_evt_rx_buf_release(struct uart_hc32_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = data->dma_rx.buffer,
	};

	async_user_callback(data, &evt);
}

static inline void async_timer_start(struct k_work_delayable *work,
					 int32_t timeout)
{
	if ((timeout != SYS_FOREVER_US) && (timeout != 0)) {
		/* start timer */
		//LOG_DBG("async timer started for %d us", timeout);
		k_work_reschedule(work, K_USEC(timeout));
	}
}

static void uart_hc32_dma_rx_flush(const struct device *dev)
{
	struct dma_status stat;
	struct uart_hc32_data *data = dev->data;

	if (dma_get_status(data->dma_rx.dma_dev,
				data->dma_rx.dma_channel, &stat) == 0) {
		size_t rx_rcv_len = data->dma_rx.buffer_length -
					stat.pending_length;
		if (rx_rcv_len > data->dma_rx.offset) {
			data->dma_rx.counter = rx_rcv_len;

			async_evt_rx_rdy(data);
		}
	}
}

#endif /* CONFIG_UART_ASYNC_API */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || \
	defined(CONFIG_UART_ASYNC_API)

static void INTC_SetIntSrc(en_int_src_t enIntSrc, IRQn_Type enIRQn)
{
	stc_irq_signin_config_t stcIrqSigninConfig;

	stcIrqSigninConfig.enIRQn = enIRQn;
	stcIrqSigninConfig.enIntSrc = enIntSrc;
	stcIrqSigninConfig.pfnCallback = NULL;
	(void)INTC_IrqSignOut(stcIrqSigninConfig.enIRQn);
	(void)INTC_IrqSignIn(&stcIrqSigninConfig);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

// static void uart_hc32_isr(const struct device *dev)
// {
// 	struct uart_hc32_data *data = dev->data;
// #if defined(CONFIG_UART_ASYNC_API)
// 	const struct uart_hc32_config *config = dev->config;
// #endif

// #ifdef CONFIG_UART_INTERRUPT_DRIVEN
// 	if (data->user_cb) {
// 		data->user_cb[](dev, data->user_data);
// 	}
// #endif /* CONFIG_UART_INTERRUPT_DRIVEN */

// #ifdef CONFIG_UART_ASYNC_API
// 	if (LL_USART_IsEnabledIT_IDLE(config->usart) &&
// 			LL_USART_IsActiveFlag_IDLE(config->usart)) {

// 		LL_USART_ClearFlag_IDLE(config->usart);

// 		//LOG_DBG("idle interrupt occurred");

// 		if (data->dma_rx.timeout == 0) {
// 			uart_hc32_dma_rx_flush(dev);
// 		} else {
// 			/* Start the RX timer not null */
// 			async_timer_start(&data->dma_rx.timeout_work,
// 								data->dma_rx.timeout);
// 		}
// 	} else if (USART_GetStatus(config->usart, USART_FLAG_TX_CPLT) &&
// 			USART_GetStatus(config->usart, USART_FLAG_TX_CPLT)) {

// 		USART_FuncCmd(config->usart, USART_FLAG_TX_CPLT, DISABLE);
// 		USART_ClearStatus(config->usart, USART_FLAG_TX_CPLT);
// 		/* Generate TX_DONE event when transmission is done */
// 		async_evt_tx_done(data);
// 	} else if (USART_GetStatus(config->usart, USART_FLAG_RX_FULL)) {
// #ifdef USART_SR_RXNE
// 		/* clear the RXNE flag, because Rx data was not read */
// 		USART_ClearStatus(config->usart, USART_FLAG_RX_FULL);
// #else
// 		/* clear the RXNE by flushing the fifo, because Rx data was not read */
// 		LL_USART_RequestRxDataFlush(config->usart);
// #endif /* USART_SR_RXNE */
// 	}

// 	/* Clear errors */
// 	uart_hc32_err_check(dev);
// }
// #endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

#ifdef CONFIG_UART_ASYNC_API

static int uart_hc32_async_callback_set(const struct device *dev,
					 uart_callback_t callback,
					 void *user_data)
{
	struct uart_hc32_data *data = dev->data;

	data->async_cb = callback;
	data->async_user_data = user_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	data->user_cb = NULL;
	data->user_data = NULL;
#endif

	return 0;
}

static inline void uart_hc32_dma_tx_enable(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;

	LL_USART_EnableDMAReq_TX(config->usart);
}

static inline void uart_hc32_dma_tx_disable(const struct device *dev)
{
#if DT_HAS_COMPAT_HCATUS_OKAY(st_hc32u5_dma)
	ARG_UNUSED(dev);

	/*
	 * Errata Sheet ES0499 : HC32U575xx and HC32U585xx device errata
	 * USART does not generate DMA requests after setting/clearing DMAT bit
	 * (also seen on hc32H5 serie)
	 */
#else
	const struct uart_hc32_config *config = dev->config;

	LL_USART_DisableDMAReq_TX(config->usart);
#endif /* ! st_hc32u5_dma */
}

static inline void uart_hc32_dma_rx_enable(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;
	struct uart_hc32_data *data = dev->data;

	LL_USART_EnableDMAReq_RX(config->usart);

	data->dma_rx.enabled = true;
}

static inline void uart_hc32_dma_rx_disable(const struct device *dev)
{
	struct uart_hc32_data *data = dev->data;

	data->dma_rx.enabled = false;
}

static int uart_hc32_async_rx_disable(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;
	struct uart_hc32_data *data = dev->data;
	struct uart_event disabled_event = {
		.type = UART_RX_DISABLED
	};

	if (!data->dma_rx.enabled) {
		async_user_callback(data, &disabled_event);
		return -EFAULT;
	}

	LL_USART_DisableIT_IDLE(config->usart);

	uart_hc32_dma_rx_flush(dev);

	async_evt_rx_buf_release(data);

	uart_hc32_dma_rx_disable(dev);

	(void)k_work_cancel_delayable(&data->dma_rx.timeout_work);

	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

	if (data->rx_next_buffer) {
		struct uart_event rx_next_buf_release_evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf.buf = data->rx_next_buffer,
		};
		async_user_callback(data, &rx_next_buf_release_evt);
	}

	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	/* When async rx is disabled, enable interruptible instance of uart to function normally */
	USART_FuncCmd(config->usart, USART_FLAG_RX_FULL, ENABLE);

	//LOG_DBG("rx: disabled");

	async_user_callback(data, &disabled_event);

	return 0;
}

void uart_hc32_dma_tx_cb(const struct device *dma_dev, void *user_data,
				   uint32_t channel, int status)
{
	const struct device *uart_dev = user_data;
	struct uart_hc32_data *data = uart_dev->data;
	struct dma_status stat;
	unsigned int key = irq_lock();

	/* Disable TX */
	uart_hc32_dma_tx_disable(uart_dev);

	(void)k_work_cancel_delayable(&data->dma_tx.timeout_work);

	if (!dma_get_status(data->dma_tx.dma_dev,
				data->dma_tx.dma_channel, &stat)) {
		data->dma_tx.counter = data->dma_tx.buffer_length -
					stat.pending_length;
	}

	data->dma_tx.buffer_length = 0;

	irq_unlock(key);
}

static void uart_hc32_dma_replace_buffer(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;
	struct uart_hc32_data *data = dev->data;

	/* Replace the buffer and reload the DMA */
	//LOG_DBG("Replacing RX buffer: %d", data->rx_next_buffer_len);

	/* reload DMA */
	data->dma_rx.offset = 0;
	data->dma_rx.counter = 0;
	data->dma_rx.buffer = data->rx_next_buffer;
	data->dma_rx.buffer_length = data->rx_next_buffer_len;
	data->dma_rx.blk_cfg.block_size = data->dma_rx.buffer_length;
	data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	dma_reload(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
			data->dma_rx.blk_cfg.source_address,
			data->dma_rx.blk_cfg.dest_address,
			data->dma_rx.blk_cfg.block_size);

	dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

	LL_USART_ClearFlag_IDLE(config->usart);

	/* Request next buffer */
	async_evt_rx_buf_request(data);
}

void uart_hc32_dma_rx_cb(const struct device *dma_dev, void *user_data,
				   uint32_t channel, int status)
{
	const struct device *uart_dev = user_data;
	struct uart_hc32_data *data = uart_dev->data;

	if (status < 0) {
		async_evt_rx_err(data, status);
		return;
	}

	(void)k_work_cancel_delayable(&data->dma_rx.timeout_work);

	/* true since this functions occurs when buffer if full */
	data->dma_rx.counter = data->dma_rx.buffer_length;

	async_evt_rx_rdy(data);

	if (data->rx_next_buffer != NULL) {
		async_evt_rx_buf_release(data);

		/* replace the buffer when the current
		 * is full and not the same as the next
		 * one.
		 */
		uart_hc32_dma_replace_buffer(uart_dev);
	} else {
		/* Buffer full without valid next buffer,
		 * an UART_RX_DISABLED event must be generated,
		 * but uart_hc32_async_rx_disable() cannot be
		 * called in ISR context. So force the RX timeout
		 * to minimum value and let the RX timeout to do the job.
		 */
		k_work_reschedule(&data->dma_rx.timeout_work, K_TICKS(1));
	}
}

static int uart_hc32_async_tx(const struct device *dev,
		const uint8_t *tx_data, size_t buf_size, int32_t timeout)
{
	const struct uart_hc32_config *config = dev->config;
	struct uart_hc32_data *data = dev->data;
	int ret;

	if (data->dma_tx.dma_dev == NULL) {
		return -ENODEV;
	}

	if (data->dma_tx.buffer_length != 0) {
		return -EBUSY;
	}

	data->dma_tx.buffer = (uint8_t *)tx_data;
	data->dma_tx.buffer_length = buf_size;
	data->dma_tx.timeout = timeout;

	//LOG_DBG("tx: l=%d", data->dma_tx.buffer_length);

	/* Clear TC flag */
	USART_ClearStatus(config->usart, USART_FLAG_TX_CPLT);

	/* Enable TC interrupt so we can signal correct TX done */
	USART_FuncCmd(config->usart, USART_FLAG_TX_CPLT, ENABLE);

	/* set source address */
	data->dma_tx.blk_cfg.source_address = (uint32_t)data->dma_tx.buffer;
	data->dma_tx.blk_cfg.block_size = data->dma_tx.buffer_length;

	ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel,
				&data->dma_tx.dma_cfg);

	if (ret != 0) {
		//LOG_ERR("dma tx config error!");
		return -EINVAL;
	}

	if (dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel)) {
		//LOG_ERR("UART err: TX DMA start failed!");
		return -EFAULT;
	}

	/* Start TX timer */
	async_timer_start(&data->dma_tx.timeout_work, data->dma_tx.timeout);
	/* Enable TX DMA requests */
	uart_hc32_dma_tx_enable(dev);

	return 0;
}

static int uart_hc32_async_rx_enable(const struct device *dev,
		uint8_t *rx_buf, size_t buf_size, int32_t timeout)
{
	const struct uart_hc32_config *config = dev->config;
	struct uart_hc32_data *data = dev->data;
	int ret;

	if (data->dma_rx.dma_dev == NULL) {
		return -ENODEV;
	}

	if (data->dma_rx.enabled) {
		//LOG_WRN("RX was already enabled");
		return -EBUSY;
	}

	data->dma_rx.offset = 0;
	data->dma_rx.buffer = rx_buf;
	data->dma_rx.buffer_length = buf_size;
	data->dma_rx.counter = 0;
	data->dma_rx.timeout = timeout;

	/* Disable RX interrupts to let DMA to handle it */
	LL_USART_DisableIT_RXNE(config->usart);

	data->dma_rx.blk_cfg.block_size = buf_size;
	data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;

	ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
				&data->dma_rx.dma_cfg);

	if (ret != 0) {
		//LOG_ERR("UART ERR: RX DMA config failed!");
		return -EINVAL;
	}

	if (dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel)) {
		//LOG_ERR("UART ERR: RX DMA start failed!");
		return -EFAULT;
	}

	/* Enable RX DMA requests */
	uart_hc32_dma_rx_enable(dev);

	/* Enable IRQ IDLE to define the end of a
	 * RX DMA transaction.
	 */
	LL_USART_ClearFlag_IDLE(config->usart);
	LL_USART_EnableIT_IDLE(config->usart);

	LL_USART_EnableIT_ERROR(config->usart);

	/* Request next buffer */
	async_evt_rx_buf_request(data);

	//LOG_DBG("async rx enabled");

	return ret;
}

static int uart_hc32_async_tx_abort(const struct device *dev)
{
	struct uart_hc32_data *data = dev->data;
	size_t tx_buffer_length = data->dma_tx.buffer_length;
	struct dma_status stat;

	if (tx_buffer_length == 0) {
		return -EFAULT;
	}

	(void)k_work_cancel_delayable(&data->dma_tx.timeout_work);
	if (!dma_get_status(data->dma_tx.dma_dev,
				data->dma_tx.dma_channel, &stat)) {
		data->dma_tx.counter = tx_buffer_length - stat.pending_length;
	}

#if DT_HAS_COMPAT_HCATUS_OKAY(st_hc32u5_dma)
	dma_suspend(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
#endif /* st_hc32u5_dma */
	dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	async_evt_tx_abort(data);

	return 0;
}

static void uart_hc32_async_rx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_dma_stream *rx_stream = CONTAINER_OF(dwork,
			struct uart_dma_stream, timeout_work);
	struct uart_hc32_data *data = CONTAINER_OF(rx_stream,
			struct uart_hc32_data, dma_rx);
	const struct device *dev = data->uart_dev;

	//LOG_DBG("rx timeout");

	if (data->dma_rx.counter == data->dma_rx.buffer_length) {
		uart_hc32_async_rx_disable(dev);
	} else {
		uart_hc32_dma_rx_flush(dev);
	}
}

static void uart_hc32_async_tx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_dma_stream *tx_stream = CONTAINER_OF(dwork,
			struct uart_dma_stream, timeout_work);
	struct uart_hc32_data *data = CONTAINER_OF(tx_stream,
			struct uart_hc32_data, dma_tx);
	const struct device *dev = data->uart_dev;

	uart_hc32_async_tx_abort(dev);

	//LOG_DBG("tx: async timeout");
}

static int uart_hc32_async_rx_buf_rsp(const struct device *dev, uint8_t *buf,
					   size_t len)
{
	struct uart_hc32_data *data = dev->data;

	//LOG_DBG("replace buffer (%d)", len);
	data->rx_next_buffer = buf;
	data->rx_next_buffer_len = len;

	return 0;
}

static int uart_hc32_async_init(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;
	struct uart_hc32_data *data = dev->data;

	data->uart_dev = dev;

	if (data->dma_rx.dma_dev != NULL) {
		if (!device_is_ready(data->dma_rx.dma_dev)) {
			return -ENODEV;
		}
	}

	if (data->dma_tx.dma_dev != NULL) {
		if (!device_is_ready(data->dma_tx.dma_dev)) {
			return -ENODEV;
		}
	}

	/* Disable both TX and RX DMA requests */
	uart_hc32_dma_rx_disable(dev);
	uart_hc32_dma_tx_disable(dev);

	k_work_init_delayable(&data->dma_rx.timeout_work,
				uart_hc32_async_rx_timeout);
	k_work_init_delayable(&data->dma_tx.timeout_work,
				uart_hc32_async_tx_timeout);

	/* Configure dma rx config */
	memset(&data->dma_rx.blk_cfg, 0, sizeof(data->dma_rx.blk_cfg));

	data->dma_rx.blk_cfg.dest_address = 0; /* dest not ready */

	if (data->dma_rx.src_addr_increment) {
		data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	if (data->dma_rx.dst_addr_increment) {
		data->dma_rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->dma_rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	/* RX disable circular buffer */
	data->dma_rx.blk_cfg.source_reload_en  = 0;
	data->dma_rx.blk_cfg.dest_reload_en = 0;
	data->dma_rx.blk_cfg.fifo_mode_control = data->dma_rx.fifo_threshold;

	data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
	data->dma_rx.dma_cfg.user_data = (void *)dev;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	/* Configure dma tx config */
	memset(&data->dma_tx.blk_cfg, 0, sizeof(data->dma_tx.blk_cfg));

	data->dma_tx.blk_cfg.source_address = 0; /* not ready */

	if (data->dma_tx.src_addr_increment) {
		data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	if (data->dma_tx.dst_addr_increment) {
		data->dma_tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->dma_tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	data->dma_tx.blk_cfg.fifo_mode_control = data->dma_tx.fifo_threshold;

	data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
	data->dma_tx.dma_cfg.user_data = (void *)dev;

	return 0;
}

#ifdef CONFIG_UART_WIDE_DATA

static int uart_hc32_async_tx_u16(const struct device *dev,
			const uint16_t *tx_data, size_t buf_size, int32_t timeout)
{
	return uart_hc32_async_tx(dev, (const uint8_t *)tx_data,
				buf_size * 2, timeout);
}

static int uart_hc32_async_rx_enable_u16(const struct device *dev,
			uint16_t *buf, size_t len, int32_t timeout)
{
	return uart_hc32_async_rx_enable(dev, (uint8_t *)buf, len * 2, timeout);
}

static int uart_hc32_async_rx_buf_rsp_u16(const struct device *dev,
			uint16_t *buf, size_t len)
{
	return uart_hc32_async_rx_buf_rsp(dev, (uint8_t *)buf, len * 2);
}

#endif /* CONFIG_UART_WIDE_DATA */

#endif /* CONFIG_UART_ASYNC_API */

static const struct uart_driver_api uart_hc32_driver_api = {
	.poll_in = uart_hc32_poll_in,
	.poll_out = uart_hc32_poll_out,
#ifdef CONFIG_UART_WIDE_DATA
	.poll_in_u16 = uart_hc32_poll_in_u16,
	.poll_out_u16 = uart_hc32_poll_out_u16,
#endif
	.err_check = uart_hc32_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_hc32_configure,
	.config_get = uart_hc32_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_hc32_fifo_fill,
	.fifo_read = uart_hc32_fifo_read,
	.irq_tx_enable = uart_hc32_irq_tx_enable,
	.irq_tx_disable = uart_hc32_irq_tx_disable,
	.irq_tx_ready = uart_hc32_irq_tx_ready,
	.irq_tx_complete = uart_hc32_irq_tx_complete,
	.irq_rx_enable = uart_hc32_irq_rx_enable,
	.irq_rx_disable = uart_hc32_irq_rx_disable,
	.irq_rx_ready = uart_hc32_irq_rx_ready,
	.irq_err_enable = uart_hc32_irq_err_enable,
	.irq_err_disable = uart_hc32_irq_err_disable,
	.irq_is_pending = uart_hc32_irq_is_pending,
	.irq_update = uart_hc32_irq_update,
	.irq_callback_set = uart_hc32_irq_callback_set,
#ifdef CONFIG_UART_WIDE_DATA
	.fifo_fill_u16 = uart_hc32_fifo_fill_u16,
	.fifo_read_u16 = uart_hc32_fifo_read_u16,
#endif
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_hc32_async_callback_set,
	.tx = uart_hc32_async_tx,
	.tx_abort = uart_hc32_async_tx_abort,
	.rx_enable = uart_hc32_async_rx_enable,
	.rx_disable = uart_hc32_async_rx_disable,
	.rx_buf_rsp = uart_hc32_async_rx_buf_rsp,
#ifdef CONFIG_UART_WIDE_DATA
	.tx_u16 = uart_hc32_async_tx_u16,
	.rx_enable_u16 = uart_hc32_async_rx_enable_u16,
	.rx_buf_rsp_u16 = uart_hc32_async_rx_buf_rsp_u16,
#endif
#endif /* CONFIG_UART_ASYNC_API */
};

// static int uart_hc32_clocks_enable(const struct device *dev)
// {
// 	// const struct uart_hc32_config *config = dev->config;
// 	struct uart_hc32_data *data = dev->data;
// 	// int err;

// 	// __uart_hc32_get_clock(dev);

// 	if (!device_is_ready(data->clock)) {
// 		// LOG_ERR("clock control device not ready");
// 		return -ENODEV;
// 	}

// 	return 0;
// }

static int uart_hc32_registers_configure(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;
	struct uart_hc32_data *data = dev->data;
	struct uart_config *uart_cfg = data->uart_cfg;
	stc_usart_uart_init_t stcUartInit;

	USART_FuncCmd(config->usart, USART_FUNC_ALL, DISABLE);

	(void)USART_UART_StructInit(&stcUartInit);
	stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
	stcUartInit.u32Baudrate = uart_cfg->baudrate;
	stcUartInit.u32StopBit = uart_hc32_cfg2ll_stopbits(uart_cfg->stop_bits);
	stcUartInit.u32Parity = uart_hc32_cfg2ll_parity(uart_cfg->parity);

	// /* Enable the single wire / half-duplex mode */
	// if (config->single_wire) {
	// 	USART_HalfDuplex_Init(config->usart, &stcUartInit, NULL);
	// } else {
	//	 (void)USART_UART_Init(config->usart, stcUartInit, NULL);
	// }
	(void)USART_UART_Init(config->usart, &stcUartInit, NULL);
	USART_FuncCmd(config->usart, USART_TX | USART_RX, ENABLE);

	return 0;
}

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_hc32_init(const struct device *dev)
{
	const struct uart_hc32_config *config = dev->config;
	int err;

	// err = uart_hc32_clocks_enable(dev);
	// if (err < 0) {
	// 	return err;
	// }

	/* Configure dt provided device signals when available */
	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}
	FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART4, ENABLE);
	err = uart_hc32_registers_configure(dev);
	if (err < 0) {
		return err;
	}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || \
	defined(CONFIG_UART_ASYNC_API)
	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

/* config DMA  */
// #ifdef CONFIG_UART_ASYNC_API
// 	return uart_hc32_async_init(dev);
// #else
	return 0;
// #endif
}

#ifdef CONFIG_UART_ASYNC_API

/* src_dev and dest_dev should be 'MEMORY' or 'PERIPHERAL'. */
#define UART_DMA_CHANNEL_INIT(index, dir, dir_cap, src_dev, dest_dev)	\
	.dma_dev = DEVICE_DT_GET(HC32_DMA_CTLR(index, dir)),			\
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),	\
	.dma_cfg = {							\
		.dma_slot = HC32_DMA_SLOT(index, dir, slot),\
		.channel_direction = HC32_DMA_CONFIG_DIRECTION(	\
					HC32_DMA_CHANNEL_CONFIG(index, dir)),\
		.channel_priority = HC32_DMA_CONFIG_PRIORITY(		\
				HC32_DMA_CHANNEL_CONFIG(index, dir)),	\
		.source_data_size = HC32_DMA_CONFIG_##src_dev##_DATA_SIZE(\
					HC32_DMA_CHANNEL_CONFIG(index, dir)),\
		.dest_data_size = HC32_DMA_CONFIG_##dest_dev##_DATA_SIZE(\
				HC32_DMA_CHANNEL_CONFIG(index, dir)),\
		.source_burst_length = 1, /* SINGLE transfer */		\
		.dest_burst_length = 1,					\
		.block_count = 1,					\
		.dma_callback = uart_hc32_dma_##dir##_cb,		\
	},								\
	.src_addr_increment = HC32_DMA_CONFIG_##src_dev##_ADDR_INC(	\
				HC32_DMA_CHANNEL_CONFIG(index, dir)),	\
	.dst_addr_increment = HC32_DMA_CONFIG_##dest_dev##_ADDR_INC(	\
				HC32_DMA_CHANNEL_CONFIG(index, dir)),	\
	.fifo_threshold = HC32_DMA_FEATURES_FIFO_THRESHOLD(		\
				HC32_DMA_FEATURES(index, dir)),		\

#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define HC32_UART_IRQ_HANDLER_FUNC(index)				\
	.irq_config_func = usart_hc32_config_func_##index,
#else
#define HC32_UART_IRQ_HANDLER_FUNC(index) /* Not used */
#endif

#ifdef CONFIG_UART_ASYNC_API
#define UART_DMA_CHANNEL(index, dir, DIR, src, dest)			\
.dma_##dir = {								\
	COND_CODE_1(DT_INST_DMAS_HAS_NAME(index, dir),			\
		 (UART_DMA_CHANNEL_INIT(index, dir, DIR, src, dest)),	\
		 (NULL))						\
	},
#else
#define UART_DMA_CHANNEL(index, dir, DIR, src, dest)
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#ifdef CONFIG_UART_EI_INTERRUPT_DRIVEN
#define USART_EI_IRQ_CONFIG(index)											\
	IRQ_CONNECT(															\
		DT_INST_IRQ_BY_IDX(index, 0, irq),									\
		DT_INST_IRQ_BY_IDX(index, 0, priority),								\
		DT_CAT(usart_hc32_rx_error_isr_, index),							\
		DEVICE_DT_INST_GET(index),											\
		0);																	\
	INTC_SetIntSrc(															\
		DT_INST_IRQ_BY_IDX(index, 0, irq),									\
		DT_INST_PROP_BY_IDX(index, enintsrc, 0));							\
	irq_enable(DT_INST_IRQ_BY_IDX(index, 0, irq));
#else /* CONFIG_UART_EI_INTERRUPT_DRIVEN */
#define USART_EI_IRQ_CONFIG(index)
#endif /* CONFIG_UART_EI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_RI_INTERRUPT_DRIVEN
#define USART_RI_IRQ_CONFIG(index)											\
	IRQ_CONNECT(															\
		DT_INST_IRQ_BY_IDX(index, 1, irq),									\
		DT_INST_IRQ_BY_IDX(index, 1, priority),								\
		DT_CAT(usart_hc32_rx_full_isr_, index),								\
		DEVICE_DT_INST_GET(index),											\
		0);																	\
	INTC_SetIntSrc(															\
		DT_INST_IRQ_BY_IDX(index, 1, irq), 									\
		DT_INST_PROP_BY_IDX(index, enintsrc, 1)); 							\
	irq_enable(DT_INST_IRQ_BY_IDX(index, 1, irq));
#else /* CONFIG_UART_RI_INTERRUPT_DRIVEN */
#define USART_RI_IRQ_CONFIG(index)
#endif /* CONFIG_UART_RI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_TI_INTERRUPT_DRIVEN
#define USART_TI_IRQ_CONFIG(index)											\
	IRQ_CONNECT(															\
		DT_INST_IRQ_BY_IDX(index, 2, irq),									\
		DT_INST_IRQ_BY_IDX(index, 2, priority),								\
		DT_CAT(usart_hc32_tx_empty_isr_, index),							\
		DEVICE_DT_INST_GET(index),											\
		0);																	\
	INTC_SetIntSrc(															\
		DT_INST_IRQ_BY_IDX(index, 2, irq), 									\
		DT_INST_PROP_BY_IDX(index, enintsrc, 2)); 							\
	irq_enable(DT_INST_IRQ_BY_IDX(index, 2, irq));
#else /* CONFIG_UART_TI_INTERRUPT_DRIVEN */
#define USART_TI_IRQ_CONFIG(index)
#endif /* CONFIG_UART_TI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_TCI_INTERRUPT_DRIVEN
#define USART_TCI_IRQ_CONFIG(index)											\
	IRQ_CONNECT(															\
		DT_INST_IRQ_BY_IDX(index, 3, irq),									\
		DT_INST_IRQ_BY_IDX(index, 3, priority),								\
		DT_CAT(usart_hc32_tx_complete_isr_, index),							\
		DEVICE_DT_INST_GET(index),											\
		0);																	\
	INTC_SetIntSrc(															\
		DT_INST_IRQ_BY_IDX(index, 3, irq), 									\
		DT_INST_PROP_BY_IDX(index, enintsrc, 3)); 							\
	irq_enable(DT_INST_IRQ_BY_IDX(index, 3, irq));
#else /* CONFIG_UART_TCI_INTERRUPT_DRIVEN */
#define USART_TCI_IRQ_CONFIG(index)
#endif /* CONFIG_UART_TCI_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_RTO_INTERRUPT_DRIVEN
#define USART_RTO_IRQ_CONFIG(index)											\
																			\
	IRQ_CONNECT(															\
		DT_INST_IRQ_BY_IDX(index, 4, irq),									\
		DT_INST_IRQ_BY_IDX(index, 4, priority),								\
		DT_CAT(usart_hc32_rx_timeout_isr_, index),							\
		DEVICE_DT_INST_GET(index),											\
		0);																	\
	INTC_SetIntSrc(															\
		DT_INST_IRQ_BY_IDX(index, 4, irq), 									\
		DT_INST_PROP_BY_IDX(index, enintsrc, 4));							\
	irq_enable(DT_INST_IRQ_BY_IDX(index, 4, irq));
#else /* CONFIG_UART_RTO_INTERRUPT_DRIVEN */
#define USART_RTO_IRQ_CONFIG(index)
#endif /* CONFIG_UART_RTO_INTERRUPT_DRIVEN */

#define HC32_UART_IRQ_HANDLER_DECL(index)									\
	static void usart_hc32_config_func_##index(const struct device *dev);	\
	static void	usart_hc32_rx_error_isr_##index(const struct device *dev);	\
	static void usart_hc32_rx_full_isr_##index(const struct device *dev);	\
	static void usart_hc32_tx_empty_isr_##index(const struct device *dev);	\
	static void usart_hc32_tx_complete_isr_##index(const struct device *dev);\
	static void	usart_hc32_rx_timeout_isr_##index(const struct device *dev)

#define HC32_UART_IRQ_HANDLER(index)										\
	static void usart_hc32_rx_error_isr_##index(const struct device *dev)	\
	{																		\
		struct uart_hc32_data *const data = dev->data;						\
																			\
		if (data->user_cb[UART_INT_IDX_EI].cb) {							\
			data->user_cb[UART_INT_IDX_EI].cb(dev,							\
				data->user_cb[UART_INT_IDX_EI].user_data);					\
		}																	\
	}																		\
																			\
	static void usart_hc32_rx_full_isr_##index(const struct device *dev)	\
	{																		\
		struct uart_hc32_data *const data = dev->data;						\
																			\
		if (data->user_cb[UART_INT_IDX_RI].cb) {							\
			data->user_cb[UART_INT_IDX_RI].cb(dev,							\
			data->user_cb[UART_INT_IDX_RI].user_data);						\
		}																	\
	}																		\
																			\
	static void 															\
		usart_hc32_tx_empty_isr_##index(const struct device *dev)			\
	{																		\
		struct uart_hc32_data *const data = dev->data;						\
																			\
		if (data->user_cb[UART_INT_IDX_TI].cb) {							\
			data->user_cb[UART_INT_IDX_TI].cb(dev,							\
				data->user_cb[UART_INT_IDX_TI].user_data);					\
		}																	\
	}																		\
																			\
	static void usart_hc32_tx_complete_isr_##index(const struct device *dev)\
	{																		\
		struct uart_hc32_data *const data = dev->data;						\
																			\
		if (data->user_cb[UART_INT_IDX_TCI].cb) {							\
			data->user_cb[UART_INT_IDX_TCI].cb(dev,							\
				data->user_cb[UART_INT_IDX_TCI].user_data);					\
		}																	\
	}																		\
																			\
	static void usart_hc32_rx_timeout_isr_##index(const struct device *dev)	\
		{																	\
			struct uart_hc32_data *const data = dev->data;					\
																			\
			if (data->user_cb[UART_INT_IDX_RTO].cb) {						\
				data->user_cb[UART_INT_IDX_RTO].cb(dev, 					\
					data->user_cb[UART_INT_IDX_RTO].user_data);				\
			}																\
		}																	\
																			\
	static void usart_hc32_config_func_##index(const struct device *dev)	\
	{											   						\
		USART_EI_IRQ_CONFIG(index)											\
		USART_RI_IRQ_CONFIG(index)											\
		USART_TI_IRQ_CONFIG(index)											\
		USART_TCI_IRQ_CONFIG(index)											\
		USART_RTO_IRQ_CONFIG(index)											\
	}

#else /* CONFIG_UART_INTERRUPT_DRIVEN */
#define HC32_UART_IRQ_HANDLER_DECL(index)
#define HC32_UART_IRQ_HANDLER(index)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define HC32_UART_INIT(index)										\
	HC32_UART_IRQ_HANDLER_DECL(index);								\
	PINCTRL_DT_INST_DEFINE(index);									\
	static struct uart_config uart_cfg_##index = {					\
		.baudrate  = DT_INST_PROP_OR(index, current_speed,			\
						HC32_UART_DEFAULT_BAUDRATE),				\
		.parity	= DT_INST_ENUM_IDX_OR(index, parity,				\
						HC32_UART_DEFAULT_PARITY),					\
		.stop_bits = DT_INST_ENUM_IDX_OR(index, stop_bits,			\
						HC32_UART_DEFAULT_STOP_BITS),				\
		.data_bits = DT_INST_ENUM_IDX_OR(index, data_bits,			\
						HC32_UART_DEFAULT_DATA_BITS),				\
		.flow_ctrl = DT_INST_PROP(index, hw_flow_control)			\
						? UART_CFG_FLOW_CTRL_RTS_CTS				\
						: UART_CFG_FLOW_CTRL_NONE,					\
	};																\
	static const struct uart_hc32_config uart_hc32_cfg_##index = {	\
		.usart = (CM_USART_TypeDef *)DT_INST_REG_ADDR(index),		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),				\
		.single_wire = DT_INST_PROP_OR(index, single_wire, false),	\
		HC32_UART_IRQ_HANDLER_FUNC(index)							\
	};																\
	static struct uart_hc32_data uart_hc32_data_##index = {			\
		.uart_cfg = &uart_cfg_##index,								\
		UART_DMA_CHANNEL(index, rx, RX, PERIPHERAL, MEMORY)			\
		UART_DMA_CHANNEL(index, tx, TX, MEMORY, PERIPHERAL)			\
	};																\
	DEVICE_DT_INST_DEFINE(index,									\
				&uart_hc32_init,									\
				PM_DEVICE_DT_INST_GET(index),						\
				&uart_hc32_data_##index, &uart_hc32_cfg_##index,	\
				PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,			\
				&uart_hc32_driver_api);								\
	HC32_UART_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(HC32_UART_INIT)
