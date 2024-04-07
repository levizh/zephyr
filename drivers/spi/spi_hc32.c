/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_hc32);

#include <misc/util.h>
#include <kernel.h>
#include <soc.h>
#include <errno.h>
#include <spi.h>
#include <toolchain.h>
#include "spi_context.h"

#include <clock_control/hc32_clock_control.h>
#include <clock_control.h>

/* SPI error status mask. */
#define SPI_HC32_ERR_MASK          SPI_FLAG_CLR_ALL
/* SPI communication timeout */
#define SPI_COMM_TIMEOUT_VAL         (0x20000000UL)
#define HC32_DMA_MAX_BLOCK_CNT       (65535U)

#if defined (HC32F460) || defined (HC32F4A0)
#define HC32_SPI_PSC_MAX           7
#endif

#ifdef CONFIG_SPI_HC32_DMA

#define SPI_HC32_DMA_ERROR_FLAG	    0x01
#define SPI_HC32_DMA_RX_DONE_FLAG	0x02
#define SPI_HC32_DMA_TX_DONE_FLAG	0x04
#define SPI_HC32_DMA_DONE_FLAG	\
	(SPI_HC32_DMA_RX_DONE_FLAG | SPI_HC32_DMA_TX_DONE_FLAG)
struct spi_hc32_dma_config {
	const struct device *dev;
	uint8_t channel;
	uint8_t src_addr_increment;
	uint8_t dst_addr_increment;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	struct dma_hc32_config_user_data dma_cfg_user_data;
};

#endif

struct spi_hc32_config {
	uint32_t reg;
	struct hc32_modules_clock_sys spi_clk;
#ifdef CONFIG_SPI_HC32_INTERRUPT
	void (*irq_configure)();
#endif
};

struct spi_hc32_data {
	struct spi_context ctx;
#ifdef CONFIG_SPI_HC32_DMA
	struct k_sem status_sem;
	volatile uint32_t status_flags;
	struct spi_hc32_dma_config dma_rx;
	struct spi_hc32_dma_config dma_tx;
#endif
};

#ifdef CONFIG_SPI_HC32_DMA
static __aligned(32) uint32_t dummy_rx_tx_buffer;
#endif

#ifdef CONFIG_SPI_HC32_DMA

/* dma interrupt context */
static void spi_dma_callback(const struct device *dev, void *user_data,
			     uint32_t channel, int status)
{
	/* arg directly holds the spi device */
	struct dma_hc32_config_user_data *dma_cfg_user_data  = user_data;
	struct spi_hc32_data *data = dma_cfg_user_data->user_data;
	ARG_UNUSED(dev);

	if (status < 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
		data->status_flags |= SPI_HC32_DMA_ERROR_FLAG;
	} else {
		/* identify the origin of this callback */
		if (channel == data->dma_tx.channel) {
			/* this part of the transfer ends */
			data->status_flags |= SPI_HC32_DMA_TX_DONE_FLAG;

		} else if (channel == data->dma_rx.channel) {
			/* this part of the transfer ends */
			data->status_flags |= SPI_HC32_DMA_RX_DONE_FLAG;
		} else {
			LOG_ERR("DMA callback channel %d is not valid.",
				channel);
			data->status_flags |= SPI_HC32_DMA_ERROR_FLAG;
		}
	}

	k_sem_give(&data->status_sem);
}

static int spi_hc32_dma_rx_load(const struct device *dev, const uint8_t *buf,
				size_t len)
{
	const struct spi_hc32_config *cfg = dev->config->config_info;
	struct spi_hc32_data *data = dev->driver_data;
	struct dma_block_config *blk_cfg = (struct dma_block_config *)
					   &data->dma_rx.dma_blk_cfg;
	struct dma_config *dma_cfg_rx = (struct dma_config *) &data->dma_rx.dma_cfg;
	uint8_t word_size, dft;
	int ret;

	word_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
	if (word_size <= 8) {
		dft = 1;
	} else if (word_size <= 16) {
		dft = 2;
	} else {
		dft = 4;
	}

	/* prepare the block for this RX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));
	blk_cfg->block_size = len;

	/* rx direction has periph as source and mem as dest. */
	if (buf == NULL) {
		/* if rx buff is null, then write data to dummy address. */
		blk_cfg->dest_address = (uint32_t)&dummy_rx_tx_buffer;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		blk_cfg->dest_address = (uint32_t)buf;
		if (data->dma_rx.dst_addr_increment == DMA_ADDR_ADJ_INCREMENT) {
			blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}

	blk_cfg->source_address = cfg->reg;
	if (data->dma_rx.src_addr_increment == DMA_ADDR_ADJ_INCREMENT) {
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_cfg_rx->head_block = blk_cfg;
	data->dma_rx.dma_cfg_user_data.user_data = data;
	dma_cfg_rx->user_data = (void *) &data->dma_rx.dma_cfg_user_data;
	dma_cfg_rx->source_data_size = dft;
	dma_cfg_rx->dest_data_size = dft;

	/* pass our client origin to the dma: data->dma_rx.channel */
	ret = dma_config(data->dma_rx.dev, data->dma_rx.channel, dma_cfg_rx);

	if (ret != 0) {
		return ret;
	}

	/* gives the request ID to the dma mux */
	return dma_start(data->dma_rx.dev, data->dma_rx.channel);
}

static int spi_hc32_dma_tx_load(const struct device *dev, const uint8_t *buf,
				size_t len)
{
	const struct spi_hc32_config *cfg = dev->config->config_info;
	struct spi_hc32_data *data = dev->driver_data;
	struct dma_block_config *blk_cfg = (struct dma_block_config *)
					   &data->dma_tx.dma_blk_cfg;
	struct dma_config *dma_cfg_tx = (struct dma_config *) &data->dma_tx.dma_cfg;
	uint8_t word_size, dft;
	int ret;

	word_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
	if (word_size <= 8) {
		dft = 1;
	} else if (word_size <= 16) {
		dft = 2;
	} else {
		dft = 4;
	}

	/* prepare the block for this RX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));
	blk_cfg->block_size = len;

	/* tx direction has periph as source and mem as dest. */
	if (buf == NULL) {
		/* if tx buff is null, then sends NOP on the line. */
		dummy_rx_tx_buffer = 0;
		blk_cfg->source_address = (uint32_t)&dummy_rx_tx_buffer;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		blk_cfg->source_address = (uint32_t)buf;
		if (data->dma_tx.src_addr_increment == DMA_ADDR_ADJ_INCREMENT) {
			blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}

	blk_cfg->dest_address = cfg->reg;
	if (data->dma_tx.dst_addr_increment == DMA_ADDR_ADJ_INCREMENT) {
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_cfg_tx->head_block = blk_cfg;
	data->dma_tx.dma_cfg_user_data.user_data = data;
	dma_cfg_tx->user_data = (void *) &data->dma_tx.dma_cfg_user_data;
	dma_cfg_tx->source_data_size = dft;
	dma_cfg_tx->dest_data_size = dft;

	/* pass our client origin to the dma: data->dma_rx.channel */
	ret = dma_config(data->dma_tx.dev, data->dma_tx.channel, dma_cfg_tx);

	if (ret != 0) {
		return ret;
	}

	/* gives the request ID to the dma mux */
	return dma_start(data->dma_tx.dev, data->dma_tx.channel);
}

static int wait_dma_done(const struct device *dev)
{
	struct spi_hc32_data *data = dev->driver_data;
	int res = -1;
	k_timeout_t timeout;

	/*
	 * In slave mode we do not know when the transaction will start. Hence,
	 * it doesn't make sense to have timeout in this case.
	 */
	if (IS_ENABLED(CONFIG_SPI_SLAVE) && spi_context_is_slave(&data->ctx)) {
		timeout = K_FOREVER;
	} else {
		timeout = K_MSEC(1000);
	}

	while (1) {
		res = k_sem_take(&data->status_sem, timeout);
		if (res != 0) {
			return res;
		}

		if (data->status_flags & SPI_HC32_DMA_ERROR_FLAG) {
			return -EIO;
		}

		if (SPI_HC32_DMA_DONE_FLAG == (data->status_flags & SPI_HC32_DMA_DONE_FLAG)) {
			return 0;
		}
	}

	return res;
}

static int spi_hc32_start_dma_transceive(const struct device *dev, size_t len)
{
	struct spi_hc32_data *data = dev->driver_data;
	const struct spi_hc32_config *cfg = dev->config->config_info;

	int ret;
	ret = spi_hc32_dma_rx_load(dev, data->ctx.rx_buf, len);
	if (ret != 0) {
		return ret;
	}

	ret = spi_hc32_dma_tx_load(dev, data->ctx.tx_buf, len);
	if (ret != 0) {
		return ret;
	}

	/* clear all flags */
	SPI_ClearStatus((CM_SPI_TypeDef *)cfg->reg, SPI_HC32_ERR_MASK);
	/* enable spi */
	SPI_Cmd((CM_SPI_TypeDef *)cfg->reg, ENABLE);
	return ret;
}
#endif /* CONFIG_SPI_HC32_DMA */

static int spi_hc32_get_err(const struct spi_hc32_config *cfg)
{
	uint32_t stat = SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg, SPI_HC32_ERR_MASK);

	if (stat) {
		LOG_ERR("spi %u error status detected, err = %u",
			cfg->reg, stat);

		return -EIO;
	}

	return 0;
}

/* function: read data and tx next data for polling mode and interrupt mode */
static int spi_hc32_frame_exchange(const struct device *dev)
{
	struct spi_hc32_data *data = dev->driver_data;
	const struct spi_hc32_config *cfg = dev->config->config_info;
	struct spi_context *ctx = &data->ctx;
	uint32_t tx_frame = 0U, rx_frame = 0U;
	uint8_t dft;

	uint8_t word_size = SPI_WORD_SIZE_GET(ctx->config->operation);
	if (word_size <= 8) {
		dft = 1;
	} else if (word_size <= 16) {
		dft = 2;
	} else {
		dft = 4;
	}

	if (spi_context_tx_buf_on(ctx)) {
		switch (dft) {
		case 1:
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
			break;
		case 2:
			tx_frame = UNALIGNED_GET((uint16_t *)(data->ctx.tx_buf));
			break;
		case 4:
			tx_frame = UNALIGNED_GET((uint32_t *)(data->ctx.tx_buf));
			break;
		}
	}

#ifndef CONFIG_SPI_HC32_INTERRUPT
	/* transfer data polling */
	while (SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg,
			     SPI_FLAG_TX_BUF_EMPTY) != SET) {
		/* NOP, wait for transfer buff empty */
	}
	SPI_WriteData((CM_SPI_TypeDef *)cfg->reg, tx_frame);
	spi_context_update_tx(ctx, 1, dft);
#endif

#ifdef CONFIG_SPI_HC32_INTERRUPT
	if (SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg,
			  SPI_FLAG_RX_BUF_FULL) != SET) {
		goto end;
	}
#else
	/* read data */
	while (SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg,
			     SPI_FLAG_RX_BUF_FULL) != SET) {
		/* NOP */
	}
#endif

	rx_frame = SPI_ReadData((CM_SPI_TypeDef *)cfg->reg);
	if (spi_context_rx_buf_on(ctx)) {
		switch (dft) {
		case 1:
			UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
			break;
		case 2:
			UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
			break;
		case 4:
			UNALIGNED_PUT(rx_frame, (uint32_t *)data->ctx.rx_buf);
			break;
		}
	}
	spi_context_update_rx(ctx, 1, dft);

#ifdef CONFIG_SPI_HC32_INTERRUPT
end:
	/* trasfer data interrupt */
	if (SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg,
			  SPI_FLAG_TX_BUF_EMPTY) == SET) {
		if (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {
			SPI_WriteData((CM_SPI_TypeDef *)cfg->reg, tx_frame);
			spi_context_update_tx(ctx, 1, dft);
		}
	}
#endif
	return spi_hc32_get_err(cfg);
}

static void spi_hc32_complete(const struct device *dev, int status)
{
	struct spi_hc32_data *data = dev->driver_data;
	const struct spi_hc32_config *cfg = dev->config->config_info;

#ifdef CONFIG_SPI_HC32_INTERRUPT
	SPI_IntCmd((CM_SPI_TypeDef *)cfg->reg,
		   (SPI_INT_ERR | SPI_INT_RX_BUF_FULL), DISABLE);
#endif
	if (!spi_context_is_slave(&data->ctx)) {
		while (SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg, SPI_FLAG_IDLE) != SET) {
			/* NOP, wait for spi completing data transmission  */
		}
	}
	spi_context_cs_control(&data->ctx, false);
	SPI_Cmd((CM_SPI_TypeDef *)cfg->reg, DISABLE);

	/* for tx final data */
	SPI_ReadData((CM_SPI_TypeDef *)cfg->reg);
	/* clear all flags */
	/* for OVRERF, MUST read reg then write 0 to clear */
	SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg, SPI_HC32_ERR_MASK);
	SPI_ClearStatus((CM_SPI_TypeDef *)cfg->reg, SPI_HC32_ERR_MASK);

#ifdef CONFIG_SPI_HC32_INTERRUPT
	spi_context_complete(&data->ctx, status);
#endif
}

#ifdef CONFIG_SPI_HC32_INTERRUPT
static void spi_hc32_isr(void *arg)
{
	struct device *const dev = (struct device *) arg;
	const struct spi_hc32_config *cfg = dev->config->config_info;
	struct spi_hc32_data *data = dev->driver_data;
	int err = 0;
	bool exchange = false, tx_go_on = false;

	err = spi_hc32_get_err(cfg);
	if (err) {
		spi_hc32_complete(dev, err);
		return;
	}

	tx_go_on = spi_context_tx_on(&data->ctx);
	if (tx_go_on || spi_context_rx_on(&data->ctx)) {
		err = spi_hc32_frame_exchange(dev);
		exchange = true;
	}

	if (err || !(spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx))) {
		if (err || (exchange == false) || (tx_go_on == false)) {
			spi_hc32_complete(dev, err);
		}
	}
}
#endif

static int spi_hc32_configure(struct device *dev,
			      const struct spi_config *config)
{
	const struct spi_hc32_config *cfg = dev->config->config_info;
	struct spi_hc32_data *data = dev->driver_data;
	uint32_t bus_freq;
	stc_spi_init_t stcSpiInit;
	int err = 0;
	uint8_t word_size, i;

	/* check if config have one before */
	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	/* Disable the spi module */
	SPI_Cmd((CM_SPI_TypeDef *)cfg->reg, DISABLE);
	/* Disable all interrupt */
	SPI_IntCmd((CM_SPI_TypeDef *)cfg->reg, SPI_IRQ_ALL, DISABLE);
	SPI_DeInit((CM_SPI_TypeDef *)cfg->reg);
	SPI_StructInit(&stcSpiInit);

	if ((SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE)
	    && (IS_ENABLED(CONFIG_SPI_SLAVE))) {
		stcSpiInit.u32MasterSlave = SPI_SLAVE;
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		stcSpiInit.u32FirstBit = SPI_FIRST_LSB;
	}

	/* soft cs pin */
	data->ctx.config = config;
	spi_context_cs_configure(&data->ctx);

	if (config->operation & SPI_LINES_MASK) {
		return -ENOTSUP;
	}

	word_size = SPI_WORD_SIZE_GET(config->operation);
	switch (word_size) {
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
		stcSpiInit.u32DataBits = ((word_size - 4) << SPI_CFG2_DSIZE_POS);
		break;

	case 20:
		stcSpiInit.u32DataBits = SPI_DATA_SIZE_20BIT;
		break;
	case 24:
		stcSpiInit.u32DataBits = SPI_DATA_SIZE_24BIT;
		break;
	case 32:
		stcSpiInit.u32DataBits = SPI_DATA_SIZE_32BIT;
		break;

	default:
		/* Not support other data size */
		return -ENOTSUP;
	}

	if (config->operation & SPI_MODE_CPOL) {
		stcSpiInit.u32SpiMode |= SPI_CFG2_CPOL;
	}

	if (config->operation & SPI_MODE_CPHA) {
		stcSpiInit.u32SpiMode |= SPI_CFG2_CPHA;
	}

#if 0
	/* get bus clk*/
	err = clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(clk_sys)),
				     (clock_control_subsys_t)&cfg->clk_sys,
				     &bus_freq);
#endif
	bus_freq = 100000000;
	if (err < 0) {
		LOG_ERR("Failed call clock_control_get_rate()");
		return -EIO;
	}

#if defined (HC32F460) || defined (HC32F4A0)
	if (SPI_SLAVE == stcSpiInit.u32MasterSlave) {
		i = 2U;
		bus_freq = bus_freq >> 2U;
	} else {
		i = 0U;
	}
#endif

	for (; i <= HC32_SPI_PSC_MAX; i++) {
		bus_freq = bus_freq >> 1U;
		if (bus_freq <= config->frequency) {
			stcSpiInit.u32BaudRatePrescaler = (i << SPI_CFG2_MBR_POS);
			break;
		}
	}
	if (i > HC32_SPI_PSC_MAX) {
		LOG_ERR("Unsupported frequency %uHz", config->frequency);
		return -EINVAL;
	}


	stcSpiInit.u32WireMode = SPI_3_WIRE;
#if defined (HC32F460) || defined (HC32F4A0)
	if (SPI_SLAVE == stcSpiInit.u32MasterSlave) {
		stcSpiInit.u32WireMode = SPI_4_WIRE;
	}
#endif
	err = SPI_Init((CM_SPI_TypeDef *)cfg->reg, &stcSpiInit);

	if (config->operation & SPI_MODE_LOOP) {
		SPI_LoopbackModeConfig((CM_SPI_TypeDef *)cfg->reg, SPI_LOOPBACK_MOSI);
	} else {
		SPI_LoopbackModeConfig((CM_SPI_TypeDef *)cfg->reg, SPI_LOOPBACK_INVD);
	}

	return err;
}

static int spi_hc32_release(struct device *dev,
			    const struct spi_config *config)
{
	struct spi_hc32_data *data = dev->driver_data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_SPI_HC32_DMA
/* spi transceive function through DMA */
static int transceive_dma(const struct device *dev,
			  const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs,
			  const struct spi_buf_set *rx_bufs,
			  bool asynchronous, struct k_poll_signal *signal)
{
	struct spi_hc32_data *data = dev->driver_data;
	const struct spi_hc32_config *cfg = dev->config->config_info;
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

	spi_context_lock(&data->ctx, asynchronous, signal);

	ret = spi_hc32_configure(dev, config);
	if (ret < 0) {
		goto error;
	}

	k_sem_reset(&data->status_sem);
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	while (data->ctx.rx_len > 0 || data->ctx.tx_len > 0) {
		size_t dma_len;

		if (data->ctx.rx_len == 0) {
			dma_len = data->ctx.tx_len;
		} else if (data->ctx.tx_len == 0) {
			dma_len = data->ctx.rx_len;
		} else {
			dma_len = MIN(data->ctx.tx_len, data->ctx.rx_len);
		}
		if (dma_len > HC32_DMA_MAX_BLOCK_CNT) {
			dma_len = HC32_DMA_MAX_BLOCK_CNT;
		}

		data->status_flags = 0;

		ret = spi_hc32_start_dma_transceive(dev, dma_len);
		if (ret != 0) {
			break;
		}

		ret = wait_dma_done(dev);
		if (ret != 0) {
			break;
		}
		SPI_Cmd((CM_SPI_TypeDef *)cfg->reg, DISABLE);
		dma_stop(data->dma_tx.dev, data->dma_tx.channel);
		dma_stop(data->dma_rx.dev, data->dma_rx.channel);

		/* wait until spi is no more busy */
		while (SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg, SPI_FLAG_IDLE) != SET) {
			/* NOP */
		}
		/* for tx final data */
		SPI_ReadData((CM_SPI_TypeDef *)cfg->reg);

		spi_context_update_tx(&data->ctx, 1, dma_len);
		spi_context_update_rx(&data->ctx, 1, dma_len);
	}

	dma_stop(data->dma_tx.dev, data->dma_tx.channel);
	dma_stop(data->dma_rx.dev, data->dma_rx.channel);
	spi_hc32_complete(dev, ret);
#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&data->ctx) && !ret) {
		ret = data->ctx.recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

error:
	spi_context_release(&data->ctx, ret);

	return ret;
}
#endif /* CONFIG_SPI_HC32_DMA */

static int transceive(struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous, struct k_poll_signal *signal)
{
	const struct spi_hc32_config *cfg = dev->config->config_info;
	struct spi_hc32_data *data = dev->driver_data;
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

#ifndef CONFIG_SPI_HC32_INTERRUPT
	if (asynchronous) {
		return -ENOTSUP;
	}
#endif

	spi_context_lock(&data->ctx, asynchronous, signal);

	ret = spi_hc32_configure(dev, config);
	if (ret) {
		return ret;
	}

	/* Set buffers info */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	/* clear all flags */
	SPI_ClearStatus((CM_SPI_TypeDef *)cfg->reg, SPI_HC32_ERR_MASK);
	spi_context_cs_control(&data->ctx, true);

#ifdef CONFIG_SPI_HC32_INTERRUPT
	SPI_IntCmd((CM_SPI_TypeDef *)cfg->reg,
		   (SPI_INT_ERR | SPI_INT_RX_BUF_FULL), ENABLE);
	/* enable spi */
	SPI_Cmd((CM_SPI_TypeDef *)cfg->reg, ENABLE);
	/* tx first data for rx buff full int */
	spi_hc32_frame_exchange(dev);
	ret = spi_context_wait_for_completion(&data->ctx);
#else
	/* enable spi */
	SPI_Cmd((CM_SPI_TypeDef *)cfg->reg, ENABLE);
	do {
		ret = spi_hc32_frame_exchange(dev);
		if (ret < 0) {
			break;
		}
	} while (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx));
	spi_hc32_complete(dev, ret);

#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&data->ctx) && !ret) {
		ret = data->ctx.recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

#endif /* CONFIG_SPI_HC32_INTERRUPT */

	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_hc32_transceive(struct device *dev,
			       const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
#ifdef CONFIG_SPI_HC32_DMA
	struct spi_hc32_data *data = dev->driver_data;

	if ((data->dma_tx.dev != NULL)
	    && (data->dma_rx.dev != NULL)) {
		return transceive_dma(dev, config, tx_bufs, rx_bufs, false, NULL);
	}
#endif
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_hc32_transceive_async(struct device *dev,
				     const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
				     struct k_poll_signal *async)
{
	return transceive(dev, config, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static const struct spi_driver_api api_funcs = {
	.transceive = spi_hc32_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_hc32_transceive_async,
#endif
	.release = spi_hc32_release,
};

static int spi_hc32_init(struct device *dev)
{
	int ret;
	struct device* clock;
	const struct spi_hc32_config *cfg = dev->config->config_info;
	struct spi_hc32_data *data = dev->driver_data;

	clock = device_get_binding(CLOCK_CONTROL);
	/* spi clock open */
	(void)clock_control_on(clock, (clock_control_subsys_t)&cfg->spi_clk);

	ret = SPI_DeInit((CM_SPI_TypeDef *)cfg->reg);

	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_SPI_HC32_INTERRUPT

	cfg->irq_configure(dev);
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

/* for device register */
#ifdef CONFIG_SPI_HC32_INTERRUPT

#define IRQ_SIGNIN(idx, num)     \
IRQ_CONNECT(DT_XHSC_HC32_SPI_##idx##_IRQ_##num, DT_XHSC_HC32_SPI_##idx##_IRQ_##num##_PRIORITY,         \
		    spi_hc32_isr, DEVICE_GET(spi_hc32_##idx), 0);                                              \
hc32_intc_irq_signin(DT_XHSC_HC32_SPI_##idx##_IRQ_##num, DT_XHSC_HC32_SPI_##idx##_IRQ_##num##_INT_SRC);\
irq_enable(DT_XHSC_HC32_SPI_##idx##_IRQ_##num);
#else
#define IRQ_SIGNIN(idx, num)

#endif /* CONFIG_SPI_HC32_INTERRUPT */

#if 0
#define DMA_INITIALIZER(inst, dir)                                      \
	{                                                                   \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, dir)),     \
		.channel = HC32_DMA_CHANNEL(inst, dir),                         \
		.dma_cfg_user_data = {                                          \
			.slot = DT_INST_DMAS_CELL_BY_NAME(inst, dir, slot),         \
		},                                                              \
		.dma_cfg = {							                        \
		.channel_direction = HC32_DMA_CONFIG_DIRECTION(HC32_DMA_CHANNEL_CONFIG(inst, dir)),	\
		.source_burst_length = 1,  /* SINGLE transfer */		        \
		.dest_burst_length = 1,    /* SINGLE transfer */		        \
		.block_count = 1,                                               \
		.dma_callback = spi_dma_callback,				                \
		.complete_callback_en = 0,                                      \
		.error_callback_en = 0,                                         \
		},								\
		.src_addr_increment = HC32_DMA_CONFIG_SOURCE_ADDR_INC(HC32_DMA_CHANNEL_CONFIG(inst, dir)),\
		.dst_addr_increment = HC32_DMA_CONFIG_DEST_ADDR_INC(HC32_DMA_CHANNEL_CONFIG(inst, dir)),  \
	}

#ifdef CONFIG_SPI_HC32_DMA
#define DMAS_DECL(inst)                                              \
		.dma_rx = COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, rx),       \
			    (DMA_INITIALIZER(inst, rx)), (NULL)),                \
		.dma_tx = COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, tx),       \
			    (DMA_INITIALIZER(inst, tx)), (NULL)),
#define SPI_DMA_STATUS_SEM(inst)						\
		.status_sem = Z_SEM_INITIALIZER(				\
		spi_hc32_data_##inst.status_sem, 0, 1),
#else
#define DMAS_DECL(inst)
#define SPI_DMA_STATUS_SEM(inst)
#endif
#endif

#define SPI_HC32_DEVICE_INIT(idx)                           \
DEVICE_DECLARE(spi_hc32_##idx);                             \
static struct spi_hc32_data spi_hc32_data_##idx = {         \
	SPI_CONTEXT_INIT_LOCK(spi_hc32_data_##idx, ctx),        \
	SPI_CONTEXT_INIT_SYNC(spi_hc32_data_##idx, ctx),        \
};                                                          \
static void spi_hc32_irq_config_##idx(struct device *dev)   \
{                                                           \
	IRQ_SIGNIN(idx, 0)                                      \
	IRQ_SIGNIN(idx, 1)                                      \
	IRQ_SIGNIN(idx, 2)                                      \
}                                                           \
static const struct spi_hc32_config spi_hc32_cfg_##idx = {  \
	.reg = DT_XHSC_HC32_SPI_##idx##_BASE_ADDRESS,           \
	.spi_clk ={\
		.bus = DT_XHSC_HC32_SPI_##idx##_CLOCK_BUS,          \
		.fcg = DT_XHSC_HC32_SPI_##idx##_CLOCK_FCG,          \
		.bits = DT_XHSC_HC32_SPI_##idx##_CLOCK_BITS         \
	},\
	COND_CODE_1(CONFIG_SPI_HC32_INTERRUPT,                  \
	 (.irq_configure =spi_hc32_irq_config_##idx),())        \
};\
DEVICE_AND_API_INIT(spi_hc32_##idx, DT_XHSC_HC32_SPI_##idx##_LABEL, &spi_hc32_init,    \
		    &spi_hc32_data_##idx, &spi_hc32_cfg_##idx,                                 \
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                                     \
		    &api_funcs);

#ifdef DT_XHSC_HC32_SPI_0
SPI_HC32_DEVICE_INIT(0)
#endif

#ifdef DT_XHSC_HC32_SPI_1
SPI_HC32_DEVICE_INIT(1)
#endif

#ifdef DT_XHSC_HC32_SPI_2
SPI_HC32_DEVICE_INIT(2)
#endif

#ifdef DT_XHSC_HC32_SPI_3
SPI_HC32_DEVICE_INIT(3)
#endif