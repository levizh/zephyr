
/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <misc/util.h>
#include <kernel.h>
#include <arch/cpu.h>
#include <misc/__assert.h>
#include <soc.h>
#include <init.h>
#include <uart.h>
#include <clock_control.h>
#include <clock_control/hc32_clock_control.h>
#include <pinmux.h>
#include <interrupt_controller/intc_hc32.h>

#include <linker/sections.h>

#ifdef CONFIG_UART_ASYNC_API
#include <drivers/dma/dma_hc32.h>
#include <dma.h>
#endif /* CONFIG_UART_ASYNC_API */

#include <logging/log.h>
#define LOG_LEVEL LOG_LEVEL_ERR
LOG_MODULE_REGISTER(uart_hc32);

#if CONFIG_UART_INTERRUPT_DRIVEN
enum usart_isr_type{
	UART_INT_IDX_EI,
	UART_INT_IDX_RI,
	UART_INT_IDX_TI,
	UART_INT_IDX_TCI,
	UART_INT_IDX_RTO,
	UART_INT_NUM
};
#endif /* UART_INTERRUPT_DRIVEN */

struct hc32_usart_cb_data {
	/** Callback function */
	uart_irq_callback_user_data_t user_cb;
	/** User data. */
	void *user_data;
};

#ifdef CONFIG_UART_ASYNC_API

#ifdef DT_XHSC_HC32_USART_0
#ifdef DT_XHSC_HC32_USART_0_DMA_RX
#define DMA_RX_FLAG_0	1
#else
#define DMA_RX_FLAG_0	0
#endif

#ifdef DT_XHSC_HC32_USART_0_DMA_TX
#define DMA_TX_FLAG_0	1
#else
#define DMA_TX_FLAG_0	0
#endif
#endif /* DT_XHSC_HC32_USART_0 */

#ifdef DT_XHSC_HC32_USART_1
#ifdef DT_XHSC_HC32_USART_1_DMA_RX
#define DMA_RX_FLAG_1	1
#else
#define DMA_RX_FLAG_1	0
#endif

#ifdef DT_XHSC_HC32_USART_1_DMA_TX
#define DMA_TX_FLAG_1	1
#else
#define DMA_TX_FLAG_1	0
#endif
#endif /* DT_XHSC_HC32_USART_1 */

#ifdef DT_XHSC_HC32_USART_2
#ifdef DT_XHSC_HC32_USART_2_DMA_RX
#define DMA_RX_FLAG_2	1
#else
#define DMA_RX_FLAG_2	0
#endif

#ifdef DT_XHSC_HC32_USART_2_DMA_RX
#define DMA_TX_FLAG_2	1
#else
#define DMA_TX_FLAG_2	0
#endif
#endif /* DT_XHSC_HC32_USART_2 */

#ifdef DT_XHSC_HC32_USART_2
#ifdef DT_XHSC_HC32_USART_2_DMA_RX
#define DMA_RX_FLAG_2	1
#else
#define DMA_RX_FLAG_2	0
#endif

#ifdef DT_XHSC_HC32_USART_2_DMA_TX
#define DMA_TX_FLAG_2	1
#else
#define DMA_TX_FLAG_2	0
#endif
#endif /* DT_XHSC_HC32_USART_2 */

#ifdef DT_XHSC_HC32_USART_3
#ifdef DT_XHSC_HC32_USART_3_DMA_RX
#define DMA_RX_FLAG_3	1
#else
#define DMA_RX_FLAG_3	0
#endif

#ifdef DT_XHSC_HC32_USART_3_DMA_TX
#define DMA_TX_FLAG_3	1
#else
#define DMA_TX_FLAG_3	0
#endif
#endif /* DT_XHSC_HC32_USART_3 */

struct usart_hc32_dma_config {
	char dma_unit;
	struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	size_t offset;
	volatile size_t counter;
	int32_t timeout;
	struct k_timer timer;
	bool enabled;
	struct dma_hc32_config_user_data user_cfg;
	struct dma_block_config dma_blk_cfg;
};
#endif /* CONFIG_UART_ASYNC_API */

/* device config */
struct uart_hc32_config {
	struct uart_device_config uart_cfg;
	/* clock subsystem driving this peripheral */
	struct hc32_modules_clock_sys uart_clock;
};

/* driver data */
struct uart_hc32_data {
	/* Baud rate */
	uint32_t baud_rate;
	/* clock device */
	struct device *clock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	struct hc32_usart_cb_data cb[UART_INT_NUM];
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
	struct usart_hc32_dma_config dma_rx;
	struct usart_hc32_dma_config dma_tx;
	const struct device *uart_dev;
	uart_callback_t async_cb;
	void *async_user_data;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
#endif /* CONFIG_UART_ASYNC_API */
};

#ifdef CONFIG_UART_ASYNC_API
static void inline uart_hc32_async_rx_timeout(struct k_timer *timer);
#endif /* CONFIG_UART_ASYNC_API */

/* convenience defines */
#define DEV_USART_CFG(dev)												\
	((const struct uart_device_config * const)(dev)->config->config_info)

#define DEV_USART_DATA(dev)												\
	((struct uart_hc32_data * const)(dev)->driver_data)

#define DEV_USART_BASE(dev)												\
	((CM_USART_TypeDef *)(DEV_USART_CFG(dev)->base))

static inline void uart_hc32_set_baudrate(struct device *dev,
					 uint32_t baud_rate)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	(void)USART_SetBaudrate(USARTx, baud_rate, NULL);
}

static inline void uart_hc32_set_parity(struct device *dev, uint32_t parity)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_SetParity(USARTx, parity);
}

static inline uint32_t uart_hc32_get_parity(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	return USART_GetParity(USARTx);
}

static inline void uart_hc32_set_stopbits(struct device *dev, uint32_t stopbits)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_SetStopBit(USARTx, stopbits);
}

static inline uint32_t uart_hc32_get_stopbits(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStopBit(USARTx);
}

static inline void uart_hc32_set_databits(struct device *dev, uint32_t databits)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_SetDataWidth(USARTx, databits);
}

static inline uint32_t uart_hc32_get_databits(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetDataWidth(USARTx);
}

static inline void uart_hc32_set_hwctrl(const struct device *dev,
					 uint32_t hwctrl)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_SetHWFlowControl(USARTx, hwctrl);
}

static inline uint32_t uart_hc32_get_hwctrl(const struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetHWFlowControl(USARTx);
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

static inline uint32_t uart_hc32_cfg2ll_stopbits(enum uart_config_stop_bits sb)
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
	case UART_CFG_DATA_BITS_8:
	default:
		return USART_DATA_WIDTH_8BIT;
	}
}

static inline enum uart_config_data_bits uart_hc32_ll2cfg_databits(uint32_t db)
{
	switch (db) {
	case USART_DATA_WIDTH_8BIT:
	default:
		return UART_CFG_DATA_BITS_8;
	}
}

static inline uint32_t uart_hc32_cfg2ll_hwctrl(enum uart_config_flow_control fc)
{
	/* default config */
	return USART_HW_FLOWCTRL_RTS;
}

static inline enum uart_config_flow_control uart_hc32_ll2cfg_hwctrl(uint32_t fc)
{
	/* DDL driver compatible with cfg, just return none */
	return UART_CFG_FLOW_CTRL_NONE;
}

static int uart_device_configure(struct device *dev,
				const struct uart_config *cfg)
{
	struct uart_hc32_data *data = DEV_USART_DATA(dev);
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	uint32_t parity;
	uint32_t stopbits;
	uint32_t databits;

	/* Hardware doesn't support mark or space parity */
	if ((UART_CFG_PARITY_NONE != cfg->parity) &&
			(UART_CFG_PARITY_ODD != cfg->parity) &&
			(UART_CFG_PARITY_EVEN != cfg->parity)) {
		return -ENOTSUP;
	}
	/* Driver does not supports parity + 9 databits */
	if ((cfg->parity != UART_CFG_PARITY_NONE) &&
		(cfg->data_bits == USART_DATA_WIDTH_9BIT)) {
		return -ENOTSUP;
	}

	if (UART_CFG_STOP_BITS_1 != cfg->stop_bits
#if defined(USART_STOPBIT_2BIT)
		&& UART_CFG_STOP_BITS_2 != cfg->stop_bits
#endif
	) {
		return -ENOTSUP;
	}

	parity = uart_hc32_cfg2ll_parity(cfg->parity);
	stopbits = uart_hc32_cfg2ll_stopbits(cfg->stop_bits);
	databits = uart_hc32_cfg2ll_databits(cfg->data_bits);

	USART_FuncCmd(USARTx, USART_TX | USART_RX, DISABLE);

	if (parity != uart_hc32_get_parity(dev)) {
		uart_hc32_set_parity(dev, parity);
	}

	if (stopbits != uart_hc32_get_stopbits(dev)) {
		uart_hc32_set_stopbits(dev, stopbits);
	}

	if (databits != uart_hc32_get_databits(dev)) {
		uart_hc32_set_databits(dev, databits);
	}

	if (cfg->baudrate != data->baud_rate) {
		uart_hc32_set_baudrate(dev, cfg->baudrate);
		data->baud_rate = cfg->baudrate;
	}

	USART_FuncCmd(USARTx, USART_TX | USART_RX, DISABLE);

	return 0;
};

static int uart_device_config_get(struct device *dev, struct uart_config *cfg)
{
	struct uart_hc32_data *data = DEV_USART_DATA(dev);

	cfg->baudrate = data->baud_rate;
	cfg->parity = uart_hc32_ll2cfg_parity(uart_hc32_get_parity(dev));
	cfg->stop_bits = uart_hc32_ll2cfg_stopbits(uart_hc32_get_stopbits(dev));
	cfg->data_bits = uart_hc32_ll2cfg_databits(uart_hc32_get_databits(dev));
	cfg->flow_ctrl = uart_hc32_ll2cfg_hwctrl(uart_hc32_get_hwctrl(dev));

	return 0;
}

static int uart_hc32_poll_in(struct device *dev, unsigned char *c)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	if (USART_GetStatus(USARTx, USART_FLAG_OVERRUN | USART_FLAG_RX_TIMEOUT)) {
		USART_ClearStatus(USARTx, USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR);
	}

	if (SET != USART_GetStatus(USARTx, USART_FLAG_RX_FULL))
	{
		return EIO;
	}

	*c = (unsigned char)USART_ReadData(USARTx);

	return 0;
}

static void uart_hc32_poll_out(struct device *dev,
					unsigned char c)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	/* Wait for TXE flag to be raised */
	while (RESET == USART_GetStatus(USARTx, USART_FLAG_TX_EMPTY)) {
		;
	}

	USART_WriteData(USARTx, (uint16_t)c);
}

#ifdef CONFIG_UART_ASYNC_API

static inline void async_user_callback(struct uart_hc32_data *data,
				struct uart_event *event)
{
	if (data->async_cb) {
		data->async_cb(event, data->async_user_data);
	}
}

int uart_hc32_async_tx_abort(struct device *dev)
{
	struct uart_event event;
	struct uart_hc32_data *data = dev->driver_data;
	size_t tx_buffer_length = data->dma_tx.buffer_length;

	if (tx_buffer_length == 0) {
		return -EFAULT;
	}

	dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);

	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;

	event.type = UART_TX_ABORTED;
	event.data.tx.buf = data->dma_tx.buffer;
	event.data.tx.len = data->dma_tx.counter;
	async_user_callback(data, &event);

	return 0;
}

static inline void async_evt_rx_rdy(struct uart_hc32_data *data)
{
	struct dma_hc32_status status;
	struct uart_event event = {
		.type = UART_RX_RDY,
		.data.rx.buf = data->dma_rx.buffer,
		.data.rx.offset = data->dma_rx.offset,
		.data.rx.len = data->dma_rx.counter - data->dma_rx.offset
	};

	async_user_callback(data, &event);

	if (0 != dma_hc32_get_status(data->dma_rx.dma_dev,
				data->dma_rx.dma_channel, &status)) {
		LOG_ERR("failed to get dma rx info");
	}
	data->dma_rx.counter = data->dma_rx.buffer_length - status.pending_length;
	data->dma_rx.offset = data->dma_rx.buffer_length - status.pending_length;
}

static inline void async_evt_rx_buf_request(struct uart_hc32_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	async_user_callback(data, &evt);
}

static void uart_hc32_dma_replace_buffer(const struct device *dev)
{
	struct uart_hc32_data *data = dev->driver_data;

	/* Replace the buffer and reload the DMA */
	LOG_DBG("Replacing RX buffer: %d", data->rx_next_buffer_len);

	/* stop DMA before reconfigure */
	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
	/* reload DMA */
	data->dma_rx.offset = 0;
	data->dma_rx.counter = 0;
	data->dma_rx.buffer = data->rx_next_buffer;
	data->dma_rx.buffer_length = data->rx_next_buffer_len;
	data->dma_rx.dma_cfg.head_block->block_size = data->dma_rx.buffer_length;
	data->dma_rx.dma_cfg.head_block->dest_address = (uint32_t)data->dma_rx.buffer;
	dma_reload(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
			data->dma_rx.dma_cfg.head_block->source_address,
			data->dma_rx.dma_cfg.head_block->dest_address,
			data->dma_rx.dma_cfg.head_block->block_size);

	dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

	/* Clear rx next after using */
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;
	/* Request next buffer if needed */
	async_evt_rx_buf_request(data);

	k_timer_init(&data->dma_rx.timer, uart_hc32_async_rx_timeout, NULL);
	k_timer_start(&data->dma_rx.timer, data->dma_rx.timeout, 0);
}

static inline void async_evt_tx_abort(struct uart_hc32_data *data)
{
	LOG_DBG("tx abort: %d", data->dma_tx.counter);

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

static void inline uart_hc32_async_rx_timeout(struct k_timer *timer)
{
	struct usart_hc32_dma_config *dma_rx =
		CONTAINER_OF(timer, struct usart_hc32_dma_config, timer);
	struct uart_hc32_data *data =
		CONTAINER_OF(dma_rx, struct uart_hc32_data, dma_rx);

	k_timer_stop(timer);

	LOG_ERR("rx timeout");

	async_evt_rx_rdy(data);

	k_timer_init(&data->dma_rx.timer, uart_hc32_async_rx_timeout, NULL);
	k_timer_start(&data->dma_rx.timer, data->dma_rx.timeout, 0);
}

static inline void async_evt_rx_err(struct uart_hc32_data *data, int err_code)
{
	LOG_DBG("rx error: %d", err_code);

	struct uart_event event = {
		.type = UART_RX_STOPPED,
		.data.rx_stop.reason = err_code,
		.data.rx_stop.data.len = data->dma_rx.counter,
		.data.rx_stop.data.offset = 0,
		.data.rx_stop.data.buf = data->dma_rx.buffer
	};

	async_user_callback(data, &event);
}

static inline void async_evt_tx_done(struct uart_hc32_data *data)
{
	LOG_DBG("tx done: %d", data->dma_tx.counter);

	struct uart_event event = {
		.type = UART_TX_DONE,
		.data.tx.buf = data->dma_tx.buffer,
		.data.tx.len = data->dma_tx.counter
	};

	/* Reset tx buffer */
	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;
	/* Send tx done to callback */
	async_user_callback(data, &event);
}

static inline void async_evt_rx_buf_release(struct uart_hc32_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = data->dma_rx.buffer,
	};

	async_user_callback(data, &evt);
}

static int uart_hc32_async_rx_disable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	struct uart_hc32_data *data = dev->driver_data;
	struct uart_event event, dis_evt;

	/* stop timer */
	k_timer_stop(&data->dma_rx.timer);
	async_evt_rx_buf_release(data);

	if (!data->dma_rx.enabled) {
		event.type = UART_RX_DISABLED;
		async_user_callback(data, &event);
		return -EFAULT;
	}
	data->dma_rx.enabled = false;

	/* Stop dma */
	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
	if (data->rx_next_buffer) {
		event.type = UART_RX_BUF_RELEASED;
		event.data.rx_buf.buf = data->rx_next_buffer;
		async_user_callback(data, &event);
	}

	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	/* When async rx is disabled, enable instance of uart to function normally */
	USART_FuncCmd(USARTx, USART_INT_RX, ENABLE);
	USART_ClearStatus(USARTx, USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN);
	LOG_DBG("dma rx disabled");

	/* Disable */
	dis_evt.type = UART_RX_DISABLED;
	async_user_callback(data, &dis_evt);


	return 0;
}

static int uart_hc32_async_rx_buf_rsp(struct device *dev, u8_t *buf, size_t len)
{
	struct uart_hc32_data *data = dev->driver_data;

	LOG_DBG("replace buffer (%d)", len);

	data->rx_next_buffer = buf;
	data->rx_next_buffer_len = len;

	return 0;
}

static void uart_hc32_dma_tx_cb(void *user_data, uint32_t channel, int status)
{
	struct dma_hc32_config_user_data *cfg = user_data;
	struct device *uart_dev = cfg->user_data;
	struct uart_hc32_data *data = uart_dev->driver_data;

	// unsigned int key = irq_lock();

	/* Disable TX */
	data->dma_tx.buffer_length = 0;
	async_evt_tx_done(data);

	// irq_unlock(key);
}

static void uart_hc32_dma_rx_cb(void *user_data, uint32_t channel, int status)
{
	struct dma_hc32_config_user_data *cfg = user_data;
	struct device *uart_dev = cfg->user_data;
	struct uart_hc32_data *data = uart_dev->driver_data;

	k_timer_stop(&data->dma_rx.timer);

	if (status < 0) {
		async_evt_rx_err(data, status);
		return;
	}

	/* true since this functions occurs when buffer if full */
	data->dma_rx.counter = data->dma_rx.buffer_length;
	async_evt_rx_rdy(data);
	data->dma_rx.offset = data->dma_rx.buffer_length;

	if (data->rx_next_buffer != NULL) {
		uart_hc32_dma_replace_buffer(uart_dev);
	} else {
		/* Send rx buf release to callback */
		async_evt_rx_buf_release(data);
		/* Disable dma rx */
		uart_hc32_async_rx_disable(uart_dev);
	}
}

static inline int usart_dma_config(struct usart_hc32_dma_config *config,
	const uint8_t *src_buf, const uint8_t *dst_buf, size_t len, u32_t timeout)
{
	struct device *dma_dev;
	int ret;

	dma_dev = device_get_binding(
			dma_hc32_get_device_name(config->dma_unit));
	config->dma_dev = dma_dev;
	if (config->dma_dev == NULL) {
		LOG_ERR("dma device error!");
		return -ENODEV;
	}

	config->timeout = timeout;
	config->dma_blk_cfg.source_address = (uint32_t)src_buf;
	config->dma_blk_cfg.dest_address = (uint32_t)dst_buf;
	config->dma_blk_cfg.block_size = len;
	config->buffer_length = len;
	if (config->dma_cfg.channel_direction == PERIPHERAL_TO_MEMORY) {
		config->dma_cfg.dma_callback = uart_hc32_dma_rx_cb;
		config->dma_blk_cfg.source_addr_adj = 0x02; /* fixed address */
		config->dma_blk_cfg.dest_addr_adj = 0x00; /* increment */
	} else {
		config->dma_cfg.dma_callback = uart_hc32_dma_tx_cb;
		config->dma_blk_cfg.source_addr_adj = 0x00; /* increment */
		config->dma_blk_cfg.dest_addr_adj = 0x02; /* fixed address */
	}
	config->dma_cfg.head_block = &config->dma_blk_cfg;
	ret = dma_config(dma_dev, config->dma_channel, &config->dma_cfg);
	if (ret != 0) {
		LOG_ERR("dma ch%d config error!", config->dma_unit);
		return -EINVAL;
	}

	/* Start and enable TX DMA requests */
	ret = dma_start(dma_dev, config->dma_channel);
	if (ret != 0) {
		LOG_ERR("dma ch%d start failed!", config->dma_unit);
		return -EFAULT;
	}

	return ret;
}

static int uart_hc32_async_rx_enable(struct device *dev, u8_t *buf, size_t len,
			 u32_t timeout)
{
	struct uart_event event;
	struct uart_hc32_data *data = DEV_USART_DATA(dev);
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	int ret;

	if (data->dma_rx.enabled) {
		LOG_WRN("RX was already enabled");
		return -EBUSY;
	}

	USART_FuncCmd(USARTx, USART_RX | USART_INT_RX, DISABLE);

	/* Configure dma rx */
	data->dma_rx.user_cfg.user_data = (void *)dev;
	data->dma_rx.dma_cfg.callback_arg = (void *)&data->dma_rx.user_cfg;
	data->dma_rx.dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	data->dma_rx.buffer = buf;
	data->dma_rx.buffer_length = len;
	data->dma_rx.timeout = timeout;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0U;
	data->dma_rx.offset = 0;
	data->dma_rx.counter = 0;
	/* Config dma */
	ret = usart_dma_config(&data->dma_rx, (uint8_t *)&USARTx->RDR, buf,
							len, timeout);
	if (ret != 0) {
		LOG_ERR("failed to config rx dma");
		return ret;
	}

	/*
	TC flag = 1 after init and not generate TC event request.
	To create TC event request disabling tx before dma configuration and
	enable tx after dma start
	*/
	USART_FuncCmd(USARTx, USART_RX, ENABLE);

	if (true == USART_GetStatus(USARTx, USART_FLAG_OVERRUN)) {
		USART_ClearStatus(USARTx, USART_FLAG_OVERRUN);
	}
	(void)USART_ReadData(USARTx);/* To clear RXNE */
	data->dma_rx.enabled = true;

	event.type = UART_RX_BUF_REQUEST;
	async_user_callback(data, &event);

	k_timer_init(&data->dma_rx.timer, uart_hc32_async_rx_timeout, NULL);
	k_timer_start(&data->dma_rx.timer, timeout, 0);

	return 0;
}

static int uart_hc32_async_init(struct device *dev)
{
	/* Disable both TX and RX DMA requests */
	uart_hc32_async_rx_disable(dev);

	((struct uart_hc32_data *)dev->driver_data)->uart_dev = dev;

	return 0;
}

int uart_hc32_async_callback_set(struct device *dev, uart_callback_t callback,
			void *user_data)
{
	struct uart_hc32_data *data = dev->driver_data;

	data->async_cb = callback;
	data->async_user_data = user_data;

	return 0;
}

int uart_hc32_async_tx(struct device *dev, const u8_t *buf, size_t len,
		  u32_t timeout)
{
	struct uart_hc32_data *data = DEV_USART_DATA(dev);
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	int ret;

	if (data->dma_tx.buffer_length != 0) {
		return -EBUSY;
	}

	LOG_DBG("tx: l=%d", data->dma_tx.buffer_length);

	/*
	TC flag = 1 after init and not generate TC event request.
	To create TC event request disabling tx before dma configuration and
	enable tx after dma start
	*/
	/* Disable TX when TC flag raised */
	while (RESET == USART_GetStatus(USARTx, USART_FLAG_TX_CPLT)) {
		;
	}
	USART_FuncCmd(USARTx, USART_TX | USART_INT_TX_EMPTY, DISABLE);

	/* Configure dma tx */
	data->dma_tx.user_cfg.user_data = (void *)dev;
	data->dma_tx.dma_cfg.callback_arg = (void *)&data->dma_tx.user_cfg;
	data->dma_tx.dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	/* Config dma */
	ret = usart_dma_config(&data->dma_tx, buf, (uint8_t *)&USARTx->TDR,
							len, timeout);
	if (ret != 0) {
		LOG_ERR("failed to config tx dma");
		return ret;
	}
	/*
	TC flag = 1 after init and not generate TC event request.
	To create TC event request disabling tx before dma configuration and
	enable tx after dma start
	*/
	USART_FuncCmd(USARTx, USART_TX, ENABLE);

	/* Start TX timer */

	return 0;
}

#endif /* CONFIG_UART_ASYNC_API */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_hc32_fifo_fill(struct device *dev, const u8_t *tx_data,
				  int size)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	u8_t num_tx = 0U;

	if (!USART_GetStatus(USARTx, USART_FLAG_TX_EMPTY)) {
		return num_tx;
	}

	while ((size - num_tx > 0) &&
		USART_GetStatus(USARTx, USART_FLAG_TX_EMPTY)) {
		/* Send a character (8bit , parity none) */
		uart_hc32_poll_out(dev, tx_data[num_tx++]);
	}

	return num_tx;
}

static int uart_hc32_fifo_read(struct device *dev, u8_t *rx_data,
				  const int size)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	u8_t num_rx = 0U;
	int ret;

	while ((size - num_rx > 0) && USART_GetStatus(USARTx, USART_FLAG_RX_FULL)) {
		/* Receive a character (8bit , parity none) */
		ret = uart_hc32_poll_in(dev, rx_data + num_rx++);
		if (0 != ret) {
			return ret;
		}

		/* Clear overrun error flag */
		if (USART_GetStatus(USARTx, USART_FLAG_OVERRUN)) {
			USART_ClearStatus(USARTx, USART_FLAG_OVERRUN);
		}
	}
	return num_rx;
}

static void uart_hc32_irq_tx_enable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_INT_TX_EMPTY | USART_INT_TX_CPLT, ENABLE);
}

static void uart_hc32_irq_tx_disable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_INT_TX_EMPTY | USART_INT_TX_CPLT, DISABLE);
}

static int uart_hc32_irq_tx_ready(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStatus(USARTx, USART_FLAG_TX_EMPTY);
}

static int uart_hc32_irq_tx_complete(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStatus(USARTx, USART_FLAG_TX_CPLT);
}

static void uart_hc32_irq_rx_enable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_INT_RX, ENABLE);
}

static void uart_hc32_irq_rx_disable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_INT_RX, DISABLE);
}

static int uart_hc32_irq_rx_ready(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStatus(USARTx, USART_FLAG_RX_FULL);
}

static void uart_hc32_irq_err_enable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	/* Enable FE, ORE parity error interruption */
	USART_FuncCmd(USARTx, USART_RX_TIMEOUT | USART_INT_RX_TIMEOUT, ENABLE);
}

static void uart_hc32_irq_err_disable(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	USART_FuncCmd(USARTx, USART_RX_TIMEOUT | USART_INT_RX_TIMEOUT, DISABLE);
}

static int uart_hc32_irq_is_pending(struct device *dev)
{
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);

	return USART_GetStatus(USARTx, USART_FLAG_ALL);
}

static int uart_hc32_irq_update(struct device *dev)
{
	return 1;
}

/*
  user may use void *user_data = x to specify irq type, the x may be:
  0：usart_hc32f4_rx_error_isr
  1：usart_hc32f4_rx_full_isr
  2：usart_hc32f4_tx_empty_isr
  3：usart_hc32f4_tx_complete_isr
  4：usart_hc32f4_rx_timeout_isr
  others or user_data = NULL：set cb for all interrupt handlers
*/
static void uart_hc32_irq_callback_set(struct device *dev,
					uart_irq_callback_user_data_t cb,
					void *user_data)
{
	uint32_t i;
	struct uart_hc32_data *data = DEV_USART_DATA(dev);

	if ((user_data == NULL) || (*(uint32_t *)user_data >= UART_INT_NUM)) {
		for (i = 0; i< UART_INT_NUM; i++) {
			data->cb[i].user_cb = cb;
			data->cb[i].user_data = user_data;
		}
	} else {
		i = *(uint32_t *)user_data;
		data->cb[i].user_cb = cb;
		data->cb[i].user_data = user_data;
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_hc32_driver_api = {
	.poll_in = uart_hc32_poll_in,
	.poll_out = uart_hc32_poll_out,
	.configure = uart_device_configure,
	.config_get = uart_device_config_get,
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_hc32_async_callback_set,
	.tx = uart_hc32_async_tx,
	.tx_abort = uart_hc32_async_tx_abort,
	.rx_enable = uart_hc32_async_rx_enable,
	.rx_buf_rsp = uart_hc32_async_rx_buf_rsp,
	.rx_disable = uart_hc32_async_rx_disable,
#endif /* CONFIG_UART_ASYNC_API */
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
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
};

static int uart_hc32_registers_configure(const struct device *dev)
{
	struct uart_hc32_data *data = DEV_USART_DATA(dev);
	CM_USART_TypeDef *USARTx = DEV_USART_BASE(dev);
	stc_usart_uart_init_t stcUartInit;

	USART_FuncCmd(USARTx, USART_FUNC_ALL, DISABLE);

	(void)USART_UART_StructInit(&stcUartInit);
	stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
	stcUartInit.u32Baudrate = data->baud_rate;

	(void)USART_UART_Init(USARTx, &stcUartInit, NULL);
	USART_FuncCmd(USARTx, USART_TX | USART_RX, ENABLE);

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
static int uart_hc32_init(struct device *dev)
{
	struct device* clock_dev;
	const struct uart_hc32_config *hc32_config = dev->config->config_info;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	const struct uart_device_config *config = DEV_USART_CFG(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
	int err;

	clock_dev = device_get_binding(CLOCK_CONTROL);
	err = clock_control_on(clock_dev,
							(clock_control_subsys_t)&hc32_config->uart_clock);
	if (err < 0) {
		return err;
	}

	err = uart_hc32_registers_configure(dev);
	if (err < 0) {
		return err;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	/* config DMA  */
#ifdef CONFIG_UART_ASYNC_API
	return uart_hc32_async_init(dev);
#else
	return 0;
#endif
}


#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#define USART_IRQ_ISR_CONFIG(isr_name_prefix, isr_idx, index)				\
	IRQ_CONNECT(DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx,					\
		DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx##_PRIORITY,				\
		isr_name_prefix##_##index,											\
		DEVICE_GET(uart_hc32_##index),										\
		0);																	\
	hc32_intc_irq_signin(													\
		DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx,							\
		DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx##_INT_SRC);				\
	irq_enable(DT_XHSC_HC32_USART_##index##_IRQ_##isr_idx);

#define UART_HC32_IRQ_HANDLER_DECL(index)									\
	static void usart_hc32_isr_func_cfg_##index(struct device *dev);		\
	static void	usart_hc32_rx_error_isr_##index(const struct device *dev);	\
	static void usart_hc32_rx_full_isr_##index(const struct device *dev);	\
	static void usart_hc32_tx_empty_isr_##index(const struct device *dev);	\
	static void usart_hc32_tx_complete_isr_##index(const struct device *dev);\
	static void	usart_hc32_rx_timeout_isr_##index(const struct device *dev)

static inline void usart_hc32_common_isr(const struct device *dev,
					enum usart_isr_type enIdx)
{
	struct uart_hc32_data *const data = DEV_USART_DATA(dev);

	if (data->cb[enIdx].user_cb) {
		data->cb[enIdx].user_cb(data->cb[enIdx].user_data);
	}
}

#define UART_HC32_ISR_DEF(index)											\
	static void usart_hc32_rx_error_isr_##index(const struct device *dev)	\
	{																		\
		usart_hc32_common_isr(dev, UART_INT_IDX_EI);						\
	}																		\
																			\
	static void usart_hc32_rx_full_isr_##index(const struct device *dev)	\
	{																		\
		usart_hc32_common_isr(dev, UART_INT_IDX_RI);						\
	}																		\
																			\
	static void 															\
		usart_hc32_tx_empty_isr_##index(const struct device *dev)			\
	{																		\
		usart_hc32_common_isr(dev, UART_INT_IDX_TI);						\
	}																		\
																			\
	static void usart_hc32_tx_complete_isr_##index(const struct device *dev)\
	{																		\
		usart_hc32_common_isr(dev, UART_INT_IDX_TCI);						\
	}																		\
																			\
	static void usart_hc32_rx_timeout_isr_##index(const struct device *dev)	\
	{																		\
		usart_hc32_common_isr(dev, UART_INT_IDX_TCI);						\
	}																		\
																			\
	static void usart_hc32_isr_func_cfg_##index(struct device *dev)			\
	{																		\
		USART_IRQ_ISR_CONFIG(usart_hc32_rx_error_isr, 0, index)				\
		USART_IRQ_ISR_CONFIG(usart_hc32_rx_full_isr, 1, index)				\
		USART_IRQ_ISR_CONFIG(usart_hc32_tx_empty_isr, 2, index)				\
		USART_IRQ_ISR_CONFIG(usart_hc32_tx_complete_isr, 3, index)			\
		USART_IRQ_ISR_CONFIG(usart_hc32_rx_timeout_isr, 4, index)			\
	}

#define UART_HC32_ISR_FUN_CONFIG(index)										\
	.irq_config_func = usart_hc32_isr_func_cfg_##index,

#else  /* CONFIG_UART_INTERRUPT_DRIVEN */
#define UART_HC32_IRQ_HANDLER_DECL(index)
#define UART_HC32_ISR_DEF(index)
#define UART_HC32_ISR_FUN_CONFIG(index)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API

#define M_COND_CODE_1(flag_1, code)	COND_CODE_1(flag_1, code, ())

#define UART_HC32_DMA_CFG(dir, cfg_val)										\
	.dma_##dir = {															\
		.dma_unit = HC32_DT_GET_DMA_UNIT(cfg_val),							\
		.dma_channel = (uint8_t)HC32_DT_GET_DMA_CH(cfg_val), 				\
		.user_cfg = {														\
			.slot = HC32_DT_GET_DMA_SLOT(cfg_val), 							\
		},																	\
		.dma_cfg = {														\
			.source_burst_length = 1,  /* SINGLE transfer */				\
			.source_data_size = 1, 											\
			.dest_burst_length = 1,	/* SINGLE transfer */					\
			.dest_data_size = 1,											\
			.block_count = 1,												\
		},																	\
	},

#define UART_HC32_DMA_CFG_PRE(index)										\
	M_COND_CODE_1(															\
		DMA_RX_FLAG_##index, 												\
		(UART_HC32_DMA_CFG(rx, DT_XHSC_HC32_USART_##index##_DMA_RX)) 		\
	) 																		\
	M_COND_CODE_1(															\
		DMA_TX_FLAG_##index, 												\
		(UART_HC32_DMA_CFG(tx, DT_XHSC_HC32_USART_##index##_DMA_TX)) 		\
	)
#else /* CONFIG_UART_ASYNC_API */
#define UART_HC32_DMA_CFG_PRE(index)
#endif /* CONFIG_UART_ASYNC_API */

#define UART_HC32_INIT(index)												\
UART_HC32_IRQ_HANDLER_DECL(index);											\
static const struct uart_hc32_config uart_hc32_cfg_##index = {				\
	.uart_cfg = {															\
		.base = (u8_t *)DT_XHSC_HC32_USART_##index##_BASE_ADDRESS,			\
		UART_HC32_ISR_FUN_CONFIG(index)										\
	},																		\
	.uart_clock = {															\
		.bus = DT_XHSC_HC32_USART_##index##_CLOCK_BUS,						\
		.fcg = DT_XHSC_HC32_USART_##index##_CLOCK_FCG,						\
		.bits = DT_XHSC_HC32_USART_##index##_CLOCK_BITS,					\
	},																		\
};																			\
static struct uart_hc32_data uart_hc32_data_##index = {						\
	.baud_rate = DT_XHSC_HC32_USART_##index##_CURRENT_SPEED,				\
	UART_HC32_DMA_CFG_PRE(index)											\
};																			\
DEVICE_AND_API_INIT(uart_hc32_##index, 										\
		DT_XHSC_HC32_USART_##index##_LABEL,									\
		&uart_hc32_init,													\
		&uart_hc32_data_##index, &uart_hc32_cfg_##index,					\
		PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,					\
		&uart_hc32_driver_api);												\
UART_HC32_ISR_DEF(index)

#ifdef DT_XHSC_HC32_USART_0
UART_HC32_INIT(0)
#endif

#ifdef DT_XHSC_HC32_USART_1
UART_HC32_INIT(1)
#endif

#ifdef DT_XHSC_HC32_USART_2
UART_HC32_INIT(2)
#endif

#ifdef DT_XHSC_HC32_USART_3
UART_HC32_INIT(3)
#endif
