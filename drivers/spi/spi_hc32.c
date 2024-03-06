/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_spi

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/spi.h>
#ifdef CONFIG_SPI_HC32_DMA
#include <zephyr/drivers/dma.h>
//#include <zephyr/drivers/dma/dma_hc32.h>
#endif

#include <hc32_ll_spi.h>
#include <hc32_ll.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(spi_hc32);

#include "spi_context.h"

/* SPI error status mask. */
#define SPI_HC32_ERR_MASK          SPI_FLAG_CLR_ALL
/* SPI communication timeout */
#define SPI_COMM_TIMEOUT_VAL       (0x20000000UL)

#if defined (HC32F460) || defined (HC32F4A0)
#define HC32_SPI_PSC_MAX           7
#endif

#ifdef CONFIG_SPI_HC32_DMA

enum spi_hc32_dma_direction {
	RX = 0,
	TX,
	NUM_OF_DIRECTION
};

struct spi_hc32_dma_config {
	const struct device *dev;
	uint32_t channel;
	uint32_t config;
	uint32_t slot;
	uint32_t fifo_threshold;
};

struct spi_hc32_dma_data {
	struct dma_config config;
	struct dma_block_config block;
	uint32_t count;
};

#endif

struct spi_hc32_config {
	uint32_t reg;
	struct hc32_modules_clock_sys clk_sys;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_HC32_DMA
	const struct spi_hc32_dma_config dma[NUM_OF_DIRECTION];
#endif
#ifdef CONFIG_SPI_HC32_INTERRUPT
	void (*irq_configure)();
#endif
};

struct spi_hc32_data {
	struct spi_context ctx;
#ifdef CONFIG_SPI_HC32_DMA
	struct spi_hc32_dma_data dma[NUM_OF_DIRECTION];
#endif
};

#ifdef CONFIG_SPI_HC32_DMA

static uint32_t dummy_tx;
static uint32_t dummy_rx;

static bool spi_hc32_dma_enabled(const struct device *dev)
{
	const struct spi_hc32_config *cfg = dev->config;

	if (cfg->dma[TX].dev && cfg->dma[RX].dev) {
		return true;
	}

	return false;
}

static size_t spi_hc32_dma_enabled_num(const struct device *dev)
{
	return spi_hc32_dma_enabled(dev) ? 2 : 0;
}

#endif

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

static int spi_hc32_configure(const struct device *dev,
			      const struct spi_config *config)
{
	struct spi_hc32_data *data = dev->data;
	const struct spi_hc32_config *cfg = dev->config;
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
	/* clear all flags */
	SPI_ClearStatus((CM_SPI_TypeDef *)cfg->reg, SPI_FLAG_CLR_ALL);
	SPI_StructInit(&stcSpiInit);

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		stcSpiInit.u32MasterSlave = SPI_SLAVE;
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		stcSpiInit.u32FirstBit = SPI_FIRST_LSB;
	}

	/* as a flag for soft cs pin*/
	if (!spi_cs_is_gpio(config)) {
		/* NOT support hw cs */
		return -ENOTSUP;
	}

	if ((config->operation & SPI_HALF_DUPLEX) || \
	    (config->operation & SPI_FRAME_FORMAT_TI)
	    || (config->operation & SPI_LINES_MASK)) {
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

	/* get bus clk*/
	err = clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(clk_sys)),
				     (clock_control_subsys_t)&cfg->clk_sys,
				     &bus_freq);
	if (err < 0) {
		LOG_ERR("Failed call clock_control_get_rate()");
		return -EIO;
	}

	for (i = 0U; i <= HC32_SPI_PSC_MAX; i++) {
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
	err = SPI_Init((CM_SPI_TypeDef *)cfg->reg, &stcSpiInit);

	if (config->operation & SPI_MODE_LOOP) {
		SPI_LoopbackModeConfig((CM_SPI_TypeDef *)cfg->reg, SPI_LOOPBACK_MOSI);
	} else {
		SPI_LoopbackModeConfig((CM_SPI_TypeDef *)cfg->reg, SPI_LOOPBACK_INVD);
	}

	data->ctx.config = config;

	return err;
}

#ifdef CONFIG_SPI_HC32_DMA
static void spi_hc32_dma_callback(const struct device *dma_dev, void *arg,
				  uint32_t channel, int status);

static uint32_t spi_hc32_dma_setup(const struct device *dev,
				   const uint32_t dir)
{
	const struct spi_hc32_config *cfg = dev->config;
	struct spi_hc32_data *data = dev->data;
	struct dma_config *dma_cfg = &data->dma[dir].config;
	struct dma_block_config *block_cfg = &data->dma[dir].block;
	const struct spi_hc32_dma_config *dma = &cfg->dma[dir];
	int ret;

	memset(dma_cfg, 0, sizeof(struct dma_config));
	memset(block_cfg, 0, sizeof(struct dma_block_config));

	dma_cfg->source_burst_length = 1;
	dma_cfg->dest_burst_length = 1;
	dma_cfg->user_data = (void *)dev;
	dma_cfg->dma_callback = spi_hc32_dma_callback;
	dma_cfg->block_count = 1U;
	dma_cfg->head_block = block_cfg;
	dma_cfg->dma_slot = cfg->dma[dir].slot;
	dma_cfg->channel_priority =
		hc32_DMA_CONFIG_PRIORITY(cfg->dma[dir].config);
	dma_cfg->channel_direction =
		dir == TX ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;

	if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
		dma_cfg->source_data_size = 1;
		dma_cfg->dest_data_size = 1;
	} else {
		dma_cfg->source_data_size = 2;
		dma_cfg->dest_data_size = 2;
	}

	block_cfg->block_size = spi_context_max_continuous_chunk(&data->ctx);

	if (dir == TX) {
		block_cfg->dest_address = (uint32_t)&SPI_DATA(cfg->reg);
		block_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		if (spi_context_tx_buf_on(&data->ctx)) {
			block_cfg->source_address = (uint32_t)data->ctx.tx_buf;
			block_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			block_cfg->source_address = (uint32_t)&dummy_tx;
			block_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}

	if (dir == RX) {
		block_cfg->source_address = (uint32_t)&SPI_DATA(cfg->reg);
		block_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		if (spi_context_rx_buf_on(&data->ctx)) {
			block_cfg->dest_address = (uint32_t)data->ctx.rx_buf;
			block_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			block_cfg->dest_address = (uint32_t)&dummy_rx;
			block_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}

	ret = dma_config(dma->dev, dma->channel, dma_cfg);
	if (ret < 0) {
		LOG_ERR("dma_config %p failed %d\n", dma->dev, ret);
		return ret;
	}

	ret = dma_start(dma->dev, dma->channel);
	if (ret < 0) {
		LOG_ERR("dma_start %p failed %d\n", dma->dev, ret);
		return ret;
	}

	return 0;
}

static int spi_hc32_start_dma_transceive(const struct device *dev)
{
	const struct spi_hc32_config *cfg = dev->config;
	struct spi_hc32_data *data = dev->data;
	const size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);
	struct dma_status stat;
	int ret = 0;

	for (size_t i = 0; i < spi_hc32_dma_enabled_num(dev); i++) {
		dma_get_status(cfg->dma[i].dev, cfg->dma[i].channel, &stat);
		if ((chunk_len != data->dma[i].count) && !stat.busy) {
			ret = spi_hc32_dma_setup(dev, i);
			if (ret < 0) {
				goto on_error;
			}
		}
	}

	SPI_CTL1(cfg->reg) |= (SPI_CTL1_DMATEN | SPI_CTL1_DMAREN);

on_error:
	if (ret < 0) {
		for (size_t i = 0; i < spi_hc32_dma_enabled_num(dev); i++) {
			dma_stop(cfg->dma[i].dev, cfg->dma[i].channel);
		}
	}
	return ret;
}
#endif

/* function: read data and tx next data for polling mode and interrupt mode */
static int spi_hc32_frame_exchange(const struct device *dev)
{
	struct spi_hc32_data *data = dev->data;
	const struct spi_hc32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint32_t tx_frame = 0U, rx_frame = 0U;
	uint8_t word_size, dft;

	word_size = SPI_WORD_SIZE_GET(ctx->config->operation);
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
	spi_context_update_tx(ctx, dft, 1);
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
	spi_context_update_rx(ctx, dft, 1);

#ifdef CONFIG_SPI_HC32_INTERRUPT
end:
	/* trasfer data interrupt */
	if (SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg,
			  SPI_FLAG_TX_BUF_EMPTY) == SET) {
		if (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {
			SPI_WriteData((CM_SPI_TypeDef *)cfg->reg, tx_frame);
			spi_context_update_tx(ctx, dft, 1);
		}
	}
#endif
	return spi_hc32_get_err(cfg);
}

uint32_t flag_all1, flag_all2;
static void spi_hc32_complete(const struct device *dev, int status)
{
	struct spi_hc32_data *data = dev->data;
	const struct spi_hc32_config *cfg = dev->config;

#ifdef CONFIG_SPI_HC32_INTERRUPT
	SPI_IntCmd((CM_SPI_TypeDef *)cfg->reg,
		   (SPI_INT_ERR | SPI_INT_TX_BUF_EMPTY | SPI_INT_RX_BUF_FULL), DISABLE);
#endif
	spi_context_cs_control(&data->ctx, false);

	SPI_Cmd((CM_SPI_TypeDef *)cfg->reg, DISABLE);
	/* TODO: 关闭DMA使能 */
	/* for tx final data */
	SPI_ReadData((CM_SPI_TypeDef *)cfg->reg);
	/* clear all flags */
	/* for OVRERF, MUST read reg then write 0 to clear */
	SPI_GetStatus((CM_SPI_TypeDef *)cfg->reg, SPI_HC32_ERR_MASK);
	SPI_ClearStatus((CM_SPI_TypeDef *)cfg->reg, SPI_HC32_ERR_MASK);

#ifdef CONFIG_SPI_HC32_DMA
	/* TODO: */
#endif

#ifdef CONFIG_SPI_HC32_INTERRUPT
	spi_context_complete(&data->ctx, dev, status);
#endif
}

static int transceive(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata)
{
	struct spi_hc32_data *data = dev->data;
	const struct spi_hc32_config *cfg = dev->config;
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

#ifndef CONFIG_SPI_HC32_INTERRUPT
	if (asynchronous) {
		return -ENOTSUP;
	}
#endif

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, config);

	ret = spi_hc32_configure(dev, config);
	if (ret < 0) {
		goto error;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

#ifdef CONFIG_SPI_HC32_INTERRUPT
#ifdef CONFIG_SPI_HC32_DMA
	/*  TODO: */

#endif /* CONFIG_SPI_HC32_DMA */
	/* clear all flags */
	SPI_ClearStatus((CM_SPI_TypeDef *)cfg->reg, SPI_HC32_ERR_MASK);
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
#endif

error:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_hc32_transceive(const struct device *dev,
			       const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_hc32_transceive_async(const struct device *dev,
				     const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
				     spi_callback_t cb,
				     void *userdata)
{
	return transceive(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif


#ifdef CONFIG_SPI_HC32_INTERRUPT
static void spi_hc32_isr(struct device *dev)
{
	const struct spi_hc32_config *cfg = dev->config;
	struct spi_hc32_data *data = dev->data;
	int err = 0;

	err = spi_hc32_get_err(cfg);
	if (err) {
		spi_hc32_complete(dev, err);
		return;
	}

	if (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {
		err = spi_hc32_frame_exchange(dev);
	}

	if (err || !(spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx))) {
		spi_hc32_complete(dev, err);
	}
}
#endif /* SPI_HC32_INTERRUPT */

#ifdef CONFIG_SPI_HC32_DMA

static bool spi_hc32_chunk_transfer_finished(const struct device *dev)
{
	struct spi_hc32_data *data = dev->data;
	struct spi_hc32_dma_data *dma = data->dma;
	const size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);

	return (MIN(dma[TX].count, dma[RX].count) >= chunk_len);
}

static void spi_hc32_dma_callback(const struct device *dma_dev, void *arg,
				  uint32_t channel, int status)
{
	const struct device *dev = (const struct device *)arg;
	const struct spi_hc32_config *cfg = dev->config;
	struct spi_hc32_data *data = dev->data;
	const size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);
	int err = 0;

	if (status < 0) {
		LOG_ERR("dma:%p ch:%d callback gets error: %d", dma_dev, channel,
			status);
		spi_hc32_complete(dev, status);
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(cfg->dma); i++) {
		if (dma_dev == cfg->dma[i].dev &&
		    channel == cfg->dma[i].channel) {
			data->dma[i].count += chunk_len;
		}
	}

	/* Check transfer finished.
	 * The transmission of this chunk is complete if both the dma[TX].count
	 * and the dma[RX].count reach greater than or equal to the chunk_len.
	 * chunk_len is zero here means the transfer is already complete.
	 */
	if (spi_hc32_chunk_transfer_finished(dev)) {
		if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
			spi_context_update_tx(&data->ctx, 1, chunk_len);
			spi_context_update_rx(&data->ctx, 1, chunk_len);
		} else {
			spi_context_update_tx(&data->ctx, 2, chunk_len);
			spi_context_update_rx(&data->ctx, 2, chunk_len);
		}

		if (spi_hc32_transfer_ongoing(data)) {
			/* Next chunk is available, reset the count and
			 * continue processing
			 */
			data->dma[TX].count = 0;
			data->dma[RX].count = 0;
		} else {
			/* All data is processed, complete the process */
			spi_context_complete(&data->ctx, dev, 0);
			return;
		}
	}

	err = spi_hc32_start_dma_transceive(dev);
	if (err) {
		spi_hc32_complete(dev, err);
	}
}
#endif /* DMA */

static int spi_hc32_release(const struct device *dev,
			    const struct spi_config *config)
{
	struct spi_hc32_data *data = dev->data;
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static struct spi_driver_api spi_hc32_driver_api = {
	.transceive = spi_hc32_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_hc32_transceive_async,
#endif
	.release = spi_hc32_release
};

int spi_hc32_init(const struct device *dev)
{
	struct spi_hc32_data *data = dev->data ;
	const struct spi_hc32_config *cfg = dev->config;
	int ret;
#ifdef CONFIG_SPI_HC32_DMA
	uint32_t ch_filter;
#endif
	/* spi clock open */
	(void)clock_control_on(DEVICE_DT_GET(DT_NODELABEL(clk_sys)),
			       (clock_control_subsys_t)&cfg->clk_sys);

	ret = SPI_DeInit((CM_SPI_TypeDef *)cfg->reg);

	if (ret < 0) {
		return ret;
	}

	/* device's pinmux to configrate */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to apply pinctrl state");
		return ret;
	}

#ifdef CONFIG_SPI_HC32_DMA
	if ((cfg->dma[RX].dev && !cfg->dma[TX].dev) ||
	    (cfg->dma[TX].dev && !cfg->dma[RX].dev)) {
		LOG_ERR("DMA must be enabled for both TX and RX channels");
		return -ENODEV;
	}
	/* TODO: dma
	for (size_t i = 0; i < spi_hc32_dma_enabled_num(dev); i++) {
		if (!deviceis_ready(cfg->dma[i].dev)) {
			LOG_ERR("DMA %s not ready", cfg->dma[i].dev->name);
			return -ENODEV;
		}

		ch_filter = BIT(cfg->dma[i].channel);
	ret = dma_request_channel(cfg->dma[i].dev, &ch_filter);
		if (ret < 0) {
			LOG_ERR("dma_request_channel failed %d", ret);
			return ret;
	}
	}*/
#endif
	/* cs initial to output mode and out logical 0*/
	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_SPI_HC32_INTERRUPT
	cfg->irq_configure(dev);
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define DMA_INITIALIZER(inst, dir)                                      \
	{                                                                   \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, dir)),     \
		.channel = DT_INST_DMAS_CELL_BY_NAME(inst, dir, channel),       \
		.slot = COND_CODE_1(                                            \
			DT_HAS_COMPAT_STATUS_OKAY(gd_hc32_dma_v1),                  \
			(DT_INST_DMAS_CELL_BY_NAME(inst, dir, slot)), (0)),         \
		.config = DT_INST_DMAS_CELL_BY_NAME(inst, dir, config),         \
		.fifo_threshold = COND_CODE_1(                                  \
			DT_HAS_COMPAT_STATUS_OKAY(gd_hc32_dma_v1),                  \
			(DT_INST_DMAS_CELL_BY_NAME(inst, dir, fifo_threshold)),     \
			(0)),						  \
	}

#define DMAS_DECL(inst)                                    \
	{                                                      \
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, rx),       \
			    (DMA_INITIALIZER(inst, rx)), ({0})),       \
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, tx),       \
			    (DMA_INITIALIZER(inst, tx)), ({0})),       \
	}

#define IDX_IRQ_CONFIGURE(idx, inst)                                                             \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, idx, irq), DT_INST_IRQ_BY_IDX(inst, idx, priority), \
			    spi_hc32_isr, DEVICE_DT_INST_GET(inst), 0);                                      \
		hc32_intc_irq_signin(DT_PHA_BY_IDX(DT_DRV_INST(inst), intcs, idx, irqn),                 \
						 DT_PHA_BY_IDX(DT_DRV_INST(inst), intcs, idx, int_src));                 \
		irq_enable(DT_INST_IRQ_BY_IDX(inst, idx, irq));

#define CONFIGURE_ALL_IRQS(inst, idxs)  LISTIFY(idxs, IDX_IRQ_CONFIGURE, (), inst)


#define HC32_IRQ_CONFIGURE(inst)                                         \
	static void spi_hc32_irq_configure_##inst(void)                      \
	{                                                                    \
		CONFIGURE_ALL_IRQS(inst, DT_PROP_LEN(DT_DRV_INST(inst), intcs)); \
	}

#define hc32_SPI_INIT(inst)                                             \
	PINCTRL_DT_INST_DEFINE(inst);                                       \
	IF_ENABLED(CONFIG_SPI_HC32_INTERRUPT, (HC32_IRQ_CONFIGURE(inst)));  \
	static struct spi_hc32_data spi_hc32_data_##inst = {                \
		SPI_CONTEXT_INIT_LOCK(spi_hc32_data_##inst, ctx),               \
		SPI_CONTEXT_INIT_SYNC(spi_hc32_data_##inst, ctx),               \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx) };      \
	static struct spi_hc32_config spi_hc32_config_##inst = {            \
		.reg = DT_INST_REG_ADDR(inst),                                  \
		.clk_sys = {   \
		.bus = DT_INST_CLOCKS_CELL(inst, bus),                          \
		.fcg = DT_INST_CLOCKS_CELL(inst, fcg),                          \
		.bits = DT_INST_CLOCKS_CELL(inst, bits)                         \
		},\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                   \
		IF_ENABLED(CONFIG_SPI_HC32_DMA, (.dma = DMAS_DECL(inst),))      \
		IF_ENABLED(CONFIG_SPI_HC32_INTERRUPT,                           \
			   (.irq_configure = spi_hc32_irq_configure_##inst)) };     \
	DEVICE_DT_INST_DEFINE(inst, &spi_hc32_init, NULL,                   \
			      &spi_hc32_data_##inst, &spi_hc32_config_##inst,       \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                \
			      &spi_hc32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(hc32_SPI_INIT)
