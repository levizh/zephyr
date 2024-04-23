/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(qspi_hc32);

#include <misc/util.h>
#include <kernel.h>
#include <soc.h>
#include <dma.h>
#include <errno.h>
#include <toolchain.h>
#include <gpio/gpio_hc32.h>
#include <interrupt_controller/intc_hc32.h>
#include "qspi_hc32.h"

#ifdef CONFIG_QSPI_HC32_DMA
/* dma interrupt context */
static void qspi_dma_callback(void *callback_arg, u32_t channel, int error_code)
{
	/* arg directly holds the spi device */
	struct dma_hc32_config_user_data *dma_cfg_user_data  = callback_arg;
	struct qspi_hc32_data *data = dma_cfg_user_data->user_data;

	if (error_code < 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
		data->status_flags |= QSPI_HC32_DMA_ERROR_FLAG;
	} else {
		/* identify the origin of this callback */
		if (channel == data->dma_tx.channel) {
			/* this part of the transfer ends */
			data->status_flags |= QSPI_HC32_DMA_TX_DONE_FLAG;

		} else if (channel == data->dma_rx.channel) {
			/* this part of the transfer ends */
			data->status_flags |= QSPI_HC32_DMA_RX_DONE_FLAG;
		} else {
			LOG_ERR("DMA callback channel %d is not valid.",
				channel);
			data->status_flags |= QSPI_HC32_DMA_ERROR_FLAG;
		}
	}

	k_sem_give(&data->status_sem);
}

static int wait_dma_done(struct qspi_hc32_data *data, uint8_t operation_type)
{
	int res = -EIO;
	s32_t timeout = K_MSEC(1000);

	while (1) {
		res = k_sem_take(&data->status_sem, timeout);
		if (res != 0) {
			return res;
		}

		if (data->status_flags & QSPI_HC32_DMA_ERROR_FLAG) {
			return -EIO;
		}

		if (TX_OPERATION == operation_type) {
			if (QSPI_HC32_DMA_TX_DONE_FLAG == (data->status_flags &
							   QSPI_HC32_DMA_TX_DONE_FLAG)) {
				return 0;
			}
		} else if (RX_OPERATION == operation_type) {
			if (QSPI_HC32_DMA_RX_DONE_FLAG == (data->status_flags &
							   QSPI_HC32_DMA_RX_DONE_FLAG)) {
				return 0;
			}
		}
	}

	return res;
}

#endif

static void hc32_qspi_word_to_byte(uint32_t u32Word, uint8_t *pu8Byte,
				   uint8_t u8Len)
{
	uint8_t u8Count = 0U;
	uint32_t u32ByteNum = u8Len - 1U;

	do {
		pu8Byte[u8Count++] = (uint8_t)(u32Word >> (u32ByteNum * 8U)) & 0xFFU;
	} while ((u32ByteNum--) != 0UL);
}

static int hc32_qspi_direct_send(struct qspi_hc32_data *data,
				 const uint8_t *tx_buff, uint32_t tx_len,
				 uint8_t lines)
{
	const uint32_t protocol = (lines) >> 1u;
	int ret = 0u;

	if ((NULL == tx_buff) || (tx_len < 1u) || ((lines != 1u) && (lines != 2u)
						   && (lines != 4u))) {
		return -EINVAL;
	}

	MODIFY_REG32(CM_QSPI->CR, (QSPI_CR_IPRSL  | QSPI_CR_APRSL     | QSPI_CR_DPRSL),
		     (protocol << QSPI_CR_IPRSL_POS | protocol << QSPI_CR_APRSL_POS | protocol <<
		      QSPI_CR_DPRSL_POS));

#ifdef CONFIG_QSPI_HC32_DMA
	uint32_t timeout = CONFIG_QSPI_HC32_DMA_TIMEOUT / 2;
	struct dma_block_config *blk_cfg = (struct dma_block_config *)
					   &data->dma_tx.dma_blk_cfg;
	struct dma_config *dma_cfg_tx = (struct dma_config *) &data->dma_tx.dma_cfg;
	k_sem_reset(&data->status_sem);

	/* prepare the block for this RX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));
	blk_cfg->block_size = tx_len;

	/* tx direction has mem as source and mem as dest. */
	blk_cfg->dest_address = (uint32_t)(&CM_QSPI->DCOM);
	blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg->source_address = (uint32_t)tx_buff;
	blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;

	dma_cfg_tx->head_block = blk_cfg;
	data->dma_tx.dma_cfg_user_data.user_data = data;
	dma_cfg_tx->callback_arg = &data->dma_tx.dma_cfg_user_data;
	dma_cfg_tx->source_data_size = 1;
	dma_cfg_tx->dest_data_size = 1;

	/* pass our client origin to the dma: data->dma_rx.channel */
	do {
		ret = dma_config(data->dma_tx.dev, data->dma_tx.channel, dma_cfg_tx);
		if (-EBUSY == ret) {
			k_sleep(1u);
			timeout--;
		}
	} while ((-EBUSY == ret) && (timeout > 0u));
	if (ret) {
		return ret;
	}

	timeout = CONFIG_QSPI_HC32_DMA_TIMEOUT / 2;
	do {
		ret = dma_start(data->dma_tx.dev, data->dma_tx.channel);
		if (-EBUSY == ret) {
			k_sleep(1u);
			timeout--;
		}
	} while ((-EBUSY == ret) && (timeout > 0u));
	if (ret) {
		return ret;
	}
	/* wait for done */
	ret = wait_dma_done(data, TX_OPERATION);
	dma_stop(data->dma_tx.dev, data->dma_tx.channel);
#else
	for (uint8_t i = 0; i < tx_len; i++) {
		QSPI_WriteDirectCommValue(tx_buff[i]);
	}
#endif
	if (!ret) {
		if (QSPI_GetStatus(QSPI_FLAG_ROM_ACCESS_ERR)) {
			LOG_ERR("QSPI occurs rom access err!");
			QSPI_ClearStatus(QSPI_FLAG_ROM_ACCESS_ERR);
			ret = -EIO;
		}
	}
	return ret;
}

static int hc32_qspi_direct_receive(struct qspi_hc32_data *data,
				    uint8_t *rx_buff, uint32_t rx_len,
				    uint8_t lines)
{
	int ret = 0u;
	const uint32_t protocol = (lines) >> 1u;
	if ((NULL == rx_buff) || (rx_len < 1u) || ((lines != 1u) && (lines != 2u)
						   && (lines != 4u))) {
		return -EINVAL;
	}

	MODIFY_REG32(CM_QSPI->CR, (QSPI_CR_IPRSL  | QSPI_CR_APRSL     | QSPI_CR_DPRSL),
		     (protocol << QSPI_CR_IPRSL_POS | protocol << QSPI_CR_APRSL_POS | protocol <<
		      QSPI_CR_DPRSL_POS));

#ifdef CONFIG_QSPI_HC32_DMA
	uint32_t timeout = CONFIG_QSPI_HC32_DMA_TIMEOUT / 2;
	struct dma_block_config *blk_cfg = (struct dma_block_config *)
					   &data->dma_rx.dma_blk_cfg;
	struct dma_config *dma_cfg_rx = (struct dma_config *) &data->dma_rx.dma_cfg;
	k_sem_reset(&data->status_sem);

	/* prepare the block for this RX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));
	blk_cfg->block_size = rx_len;

	/* rx direction has mem as source and mem as dest. */
	blk_cfg->dest_address = (uint32_t)rx_buff;
	blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk_cfg->source_address = (uint32_t)(&CM_QSPI->DCOM);
	blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	dma_cfg_rx->head_block = blk_cfg;
	data->dma_rx.dma_cfg_user_data.user_data = data;
	dma_cfg_rx->callback_arg = &data->dma_rx.dma_cfg_user_data;
	dma_cfg_rx->source_data_size = 1;
	dma_cfg_rx->dest_data_size = 1;

	/* pass our client origin to the dma: data->dma_rx.channel */
	do {
		ret = dma_config(data->dma_rx.dev, data->dma_rx.channel, dma_cfg_rx);
		if (-EBUSY == ret) {
			k_sleep(1u);
			timeout--;
		}
	} while ((-EBUSY == ret) && (timeout > 0u));
	if (ret) {
		return ret;
	}

	timeout = CONFIG_QSPI_HC32_DMA_TIMEOUT / 2;
	do {
		ret = dma_start(data->dma_rx.dev, data->dma_rx.channel);
		if (-EBUSY == ret) {
			k_sleep(1u);
			timeout--;
		}
	} while ((-EBUSY == ret) && (timeout > 0u));
	if (ret) {
		return ret;
	}
	/* wait for done */
	ret = wait_dma_done(data, RX_OPERATION);
	dma_stop(data->dma_rx.dev, data->dma_rx.channel);
#else
	for (uint8_t i = 0; i < rx_len; i++) {
		rx_buff[i] = QSPI_ReadDirectCommValue();
	}
#endif
	if (!ret) {
		if (QSPI_GetStatus(QSPI_FLAG_ROM_ACCESS_ERR)) {
			LOG_ERR("QSPI occurs rom access err!");
			QSPI_ClearStatus(QSPI_FLAG_ROM_ACCESS_ERR);
			ret = -EIO;
		}
	}
	return ret;
}

/*
*function: write data through direct communication mode.
*/
static int qspi_write(struct device *dev,
		      const struct qspi_hc32_config *cfg)
{
	struct qspi_hc32_data *data = dev->driver_data;
	const uint32_t CR_reg = CM_QSPI->CR;
	const struct gpio_hc32_config *cs_port_config =
			data->ctx.config->cs->gpio_dev->config->config_info;
	const uint8_t cs_pin = data->ctx.config->cs->gpio_pin;
	const uint16_t cs_func;
	uint8_t word_to_byte[4u];
	int ret = 0u;

	/* direct mode do not support dummy */
	if (NULL != cfg->dummy) {
		return -ENOTSUP;
	}

#ifdef CONFIG_QSPI_HC32_DMA
	if (!data->dma_tx.dev) {
		LOG_ERR("There's no tx dma dev!");
		return -EINVAL;
	}
#endif

	if (GPIO_GetFunc(cs_port_config->port, BIT(cs_pin), (uint16_t *)&cs_func)) {
		LOG_ERR("Failed to call GPIO_GetFunc()!");
		return -EIO;
	}
	/* cs set to gpio */
	spi_context_cs_configure(&data->ctx);
	GPIO_SetFunc(cs_port_config->port, BIT(cs_pin), GPIO_FUNC_0);
	spi_context_cs_control(&data->ctx, true);

	QSPI_SetReadMode(QSPI_RD_MD_CUSTOM_STANDARD_RD);
	/* disable XIP mode */
	CLR_REG32_BIT(CM_QSPI->CR, QSPI_CR_XIPE);
	/* Enter direct communication mode */
	SET_REG32_BIT(CM_QSPI->CR, QSPI_CR_DCOME);
	/* send instruct */
	ret = hc32_qspi_direct_send(data, &cfg->instruction->content, 1u,
				    cfg->instruction->lines);
	/* send address */
	if ((NULL != cfg->address) && (!ret)) {
		hc32_qspi_word_to_byte(cfg->address->content, word_to_byte,
				       cfg->address->bytes);
		ret = hc32_qspi_direct_send(data, word_to_byte, cfg->address->bytes,
					    cfg->address->lines);
	}

	/* write data */
	if ((data->ctx.tx_buf) && (!ret)) {
		while (spi_context_tx_on(&data->ctx)) {
			ret = hc32_qspi_direct_send(data, data->ctx.tx_buf, data->ctx.tx_len,
						    cfg->data_lines);
			spi_context_update_tx(&data->ctx, 1, data->ctx.tx_len);
		}
	}

	/* done */
	spi_context_cs_control(&data->ctx, false);
	/* recover CR register and exit direct mode */
	CM_QSPI->CR = CR_reg;
	/* cs exit gpio mode */
	GPIO_SetFunc(cs_port_config->port, BIT(cs_pin), cs_func);
	if (!ret) {
		if (QSPI_GetStatus(QSPI_FLAG_ROM_ACCESS_ERR)) {
			LOG_ERR("QSPI occurs rom access err!");
			QSPI_ClearStatus(QSPI_FLAG_ROM_ACCESS_ERR);
			ret = -EIO;
		}
	}
	return ret;
}

/*
* Function: read data through direct communication mode.
* Called by qspi_read()
*/
static int qspi_read_direct(struct device *dev,
			    const struct qspi_hc32_config *cfg)
{
	struct qspi_hc32_data *data = dev->driver_data;
	const uint32_t CR_reg = CM_QSPI->CR;
	const struct gpio_hc32_config *cs_port_config =
			data->ctx.config->cs->gpio_dev->config->config_info;
	const uint8_t cs_pin = data->ctx.config->cs->gpio_pin;
	const uint16_t cs_func;
	uint8_t word_to_byte[4u];
	int ret = 0u;

	/* direct mode do not support dummy */
	if (NULL != cfg->dummy) {
		return -ENOTSUP;
	}

#ifdef CONFIG_QSPI_HC32_DMA
	if ((!data->dma_tx.dev) || (!data->dma_rx.dev)) {
		LOG_ERR("There's no rx or tx dma dev!");
		return -EINVAL;
	}
#endif

	if (GPIO_GetFunc(cs_port_config->port, BIT(cs_pin), (uint16_t *)&cs_func)) {
		LOG_ERR("Failed to call GPIO_GetFunc()!");
		return -EIO;
	}

	/* cs set to gpio */
	spi_context_cs_configure(&data->ctx);
	GPIO_SetFunc(cs_port_config->port, BIT(cs_pin), GPIO_FUNC_0);
	spi_context_cs_control(&data->ctx, true);

	QSPI_SetReadMode(QSPI_RD_MD_CUSTOM_STANDARD_RD);
	/* disable XIP mode */
	QSPI_XipModeCmd(0xffu, DISABLE);
	/* Enter direct communication mode */
	SET_REG32_BIT(CM_QSPI->CR, QSPI_CR_DCOME);
	/* send instruct */
	ret = hc32_qspi_direct_send(data, &cfg->instruction->content, 1u,
				    cfg->instruction->lines);
	if (!ret) {
		/* send address */
		if (NULL != cfg->address) {
			hc32_qspi_word_to_byte(cfg->address->content, word_to_byte,
					       cfg->address->bytes);
			ret = hc32_qspi_direct_send(data, word_to_byte, cfg->address->bytes,
						    cfg->address->lines);
		}

		/* read data */
		if (!ret) {
			while (spi_context_rx_on(&data->ctx)) {
				ret = hc32_qspi_direct_receive(data, data->ctx.rx_buf, data->ctx.rx_len,
							       cfg->data_lines);
				spi_context_update_rx(&data->ctx, 1, data->ctx.rx_len);
			}
		}
	}

	/* done */
	spi_context_cs_control(&data->ctx, false);
	/* recover CR register and exit direct mode */
	CM_QSPI->CR = CR_reg;
	/* cs exit gpio mode */
	GPIO_SetFunc(cs_port_config->port, BIT(cs_pin), cs_func);
	if (!ret) {
		if (QSPI_GetStatus(QSPI_FLAG_ROM_ACCESS_ERR)) {
			LOG_ERR("QSPI occurs rom access err!");
			QSPI_ClearStatus(QSPI_FLAG_ROM_ACCESS_ERR);
			ret = -EIO;
		}
	}
	return ret;
}

/*
*function: read data through rom mode or direct communication mode.
*/
static int qspi_read(struct device *dev, const struct qspi_hc32_config *cfg)
{
	struct qspi_hc32_data *data = dev->driver_data;
	uint8_t readMode = QSPI_RD_MD_CUSTOM_STANDARD_RD;
	stc_qspi_custom_mode_t stcCustomMode;
	uint32_t address_set;
	uint32_t dummy_set = 0u;
	int ret = 0u;

	if (NULL == cfg->address) {
		if (NULL == cfg->instruction) {
			LOG_ERR("Address struct and instruction struct all set null!");
			return -EINVAL;
		} else {
			return qspi_read_direct(dev, cfg);
		}
	}

	if ((cfg->data_lines != 1u) && (cfg->data_lines != 2u)
	    && (cfg->data_lines != 4u)) {
		LOG_ERR("Data lines set error!");
		return -EINVAL;
	}

	/* update address set */
	address_set = (cfg->address->bytes - 1u) << QSPI_FCR_AWSL_POS;
	if (4u == cfg->address->bytes) {
		address_set |= QSPI_FCR_FOUR_BIC;
	}

	if (NULL != cfg->dummy) {
		readMode += 1U;
		/* update dummy cycles set */
		dummy_set = (cfg->dummy->dummy_cycles - 3u) << QSPI_FCR_DMCYCN_POS;
		/* if set XIP mode or not */
		QSPI_XipModeCmd(cfg->dummy->content, (cfg->dummy->XIP_Enable ? 1u : 0u));
	} else {
		QSPI_XipModeCmd(0xffu, DISABLE);
	}

	QSPI_SetReadMode(readMode);
	/* set address bytes and dummy cycles */
	MODIFY_REG32(CM_QSPI->FCR,
		     (QSPI_FCR_AWSL | QSPI_FCR_DMCYCN | QSPI_FCR_FOUR_BIC),
		     (address_set | dummy_set));

	if (cfg->instruction != NULL) {
		stcCustomMode.u32InstrProtocol = ((cfg->instruction->lines) >> 1u) <<
						 QSPI_CR_IPRSL_POS;
		stcCustomMode.u8InstrCode = cfg->instruction->content;
	} else {
		stcCustomMode.u32InstrProtocol = (CM_QSPI->CR & QSPI_CR_IPRSL);
		stcCustomMode.u8InstrCode = (uint8_t)(CM_QSPI->CCMD & QSPI_CCMD_RIC);
	}

	stcCustomMode.u32AddrProtocol = (cfg->address->lines >> 1u) <<
					QSPI_CR_APRSL_POS;
	stcCustomMode.u32DataProtocol = (cfg->data_lines >> 1u) << QSPI_CR_DPRSL_POS;
	QSPI_CustomReadConfig(&stcCustomMode);

	/* read process */
	uint32_t read_len;
	uint32_t next_block_len;
	uint32_t read_add = cfg->address->content;
	__IO uint8_t *pu8Read;

	while (spi_context_rx_on(&data->ctx)) {
		next_block_len = ROM_BLOCK_SIZE - (read_add % ROM_BLOCK_SIZE);
		pu8Read = (uint8_t *)((read_add % ROM_BLOCK_SIZE) + QSPI_ROM_BASE);
		if (data->ctx.rx_len < next_block_len) {
			read_len = data->ctx.rx_len;
		} else {
			read_len = next_block_len;
		}

		if ((read_add >= QSPI_EXAR_EXADR)
		    || ((read_add + read_len) >= QSPI_EXAR_EXADR)) {
			LOG_ERR("Read address out of range!");
			return -EINVAL;
		}
		/* update address blocks */
		QSPI_SelectMemoryBlock(read_add / ROM_BLOCK_SIZE);

#ifdef CONFIG_QSPI_HC32_DMA
		if (!data->dma_rx.dev) {
			LOG_ERR("There's no rx dma dev!");
			return -EINVAL;
		}
		uint32_t timeout = CONFIG_QSPI_HC32_DMA_TIMEOUT / 2u;
		struct dma_block_config *blk_cfg = (struct dma_block_config *)
						   &data->dma_rx.dma_blk_cfg;
		struct dma_config *dma_cfg_rx = (struct dma_config *) &data->dma_rx.dma_cfg;
		k_sem_reset(&data->status_sem);

		if (read_len > HC32_DMA_MAX_BLOCK_CNT) {
			read_len = HC32_DMA_MAX_BLOCK_CNT;
		}
		/* prepare the block for this RX DMA channel */
		memset(blk_cfg, 0, sizeof(struct dma_block_config));
		blk_cfg->block_size = read_len;

		/* rx direction has mem as source and mem as dest. */
		blk_cfg->dest_address = (uint32_t)data->ctx.rx_buf;
		blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		blk_cfg->source_address = (uint32_t)pu8Read;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		dma_cfg_rx->head_block = blk_cfg;
		data->dma_rx.dma_cfg_user_data.user_data = data;
		dma_cfg_rx->callback_arg = &data->dma_rx.dma_cfg_user_data;
		dma_cfg_rx->source_data_size = 1;
		dma_cfg_rx->dest_data_size = 1;

		/* pass our client origin to the dma: data->dma_rx.channel */
		do {
			ret = dma_config(data->dma_rx.dev, data->dma_rx.channel, dma_cfg_rx);
			if (-EBUSY == ret) {
				k_sleep(1u);
				timeout--;
			}
		} while ((-EBUSY == ret) && (timeout > 0u));
		if (ret) {
			return ret;
		}

		timeout = CONFIG_QSPI_HC32_DMA_TIMEOUT / 2u;
		do {
			ret = dma_start(data->dma_rx.dev, data->dma_rx.channel);
			if (-EBUSY == ret) {
				k_sleep(1u);
				timeout--;
			}
		} while ((-EBUSY == ret) && (timeout > 0u));
		if (ret) {
			return ret;
		}
		/* wait for done */
		ret = wait_dma_done(data, RX_OPERATION);
		if (ret) {
			break;
		}
		dma_stop(data->dma_rx.dev, data->dma_rx.channel);
#else
		for (uint32_t i = 0; i < read_len; i++) {
			UNALIGNED_PUT(*pu8Read++, data->ctx.rx_buf++);
		}
#endif
		spi_context_update_rx(&data->ctx, 1, read_len);
		read_add += read_len;
	}
	return ret;
}



static int qspi_hc32_configure(struct device *dev,
			       const struct spi_config *config)
{
	struct qspi_hc32_data *data = dev->driver_data;
	const struct qspi_hc32_cfg *cfg = dev->config->config_info;
	uint32_t bus_freq;
	stc_qspi_init_t stcQspiInit;
	int err = 0;
	uint8_t i;

	/* check if config have one before */
	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		return -ENOTSUP;
	}

	data->ctx.config = config;
	QSPI_DeInit();
	QSPI_StructInit(&stcQspiInit);

	if (config->operation & SPI_MODE_CPOL) {
		stcQspiInit.u32SpiMode = QSPI_SPI_MD3;
	}

	/* get bus clk*/
	err = clock_control_get_rate(device_get_binding(CLOCK_CONTROL),
				     (clock_control_subsys_t)&cfg->qspi_clk,
				     &bus_freq);

	if (err < 0) {
		LOG_ERR("Failed call clock_control_get_rate()");
		return -EIO;
	}

	for (i = 2u; i <= HC32_QSPI_PSC_MAX; i++) {
		if ((bus_freq / i) <= config->frequency) {
			stcQspiInit.u32ClockDiv = (i - 1u) << QSPI_CR_DIV_POS;
			break;
		}
	}
	if (i > HC32_QSPI_PSC_MAX) {
		LOG_ERR("Unsupported frequency %uHz", config->frequency);
		return -EINVAL;
	}
	stcQspiInit.u32ReleaseTime = QSPI_QSSN_RELEASE_DELAY_QSCK32;

	err = QSPI_Init(&stcQspiInit);
	return err;
}

static int transceive(struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous, struct k_poll_signal *signal)
{
	const struct qspi_hc32_config *cfg = (struct qspi_hc32_config *)config;
	struct qspi_hc32_data *data = dev->driver_data;
	int ret;

	if (!cfg->instruction && !cfg->address) {
		return -EINVAL;
	}

	if (asynchronous) {
		return -ENOTSUP;
	}

	if ((NULL != cfg->address) && (cfg->address->lines != 1u)
	    && (cfg->address->lines != 2u)
	    && (cfg->address->lines != 4u)) {
		LOG_ERR("Address lines set error!");
		return -EINVAL;
	}

	if ((cfg->instruction != NULL) && (cfg->instruction->lines != 1u)
	    && (cfg->instruction->lines != 2u) && (cfg->instruction->lines != 4u)) {
		LOG_ERR("Instruction lines set error!");
		return -EINVAL;
	}

	if ((cfg->dummy != NULL) && ((cfg->dummy->dummy_cycles < 3u)
				     || (cfg->dummy->dummy_cycles > 18u))) {
		LOG_ERR("Dummy_cycles set error!");
		return -EINVAL;
	}

	if ((NULL != cfg->address) && ((cfg->address->bytes > 4u)
				       || (cfg->address->bytes < 1u))) {
		LOG_ERR("Address bytes set error!");
		return -EINVAL;
	}

	spi_context_lock(&data->ctx, asynchronous, signal);

	ret = qspi_hc32_configure(dev, config);
	if (ret) {
		return ret;
	}
	/* qspi transfer data */

	/* Set buffers info */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (rx_bufs != NULL) {
		ret = qspi_read(dev, cfg);
		if (ret) {
			return ret;
		}
	} else {
		if ((NULL != cfg->address) && ((cfg->address->content >= QSPI_EXAR_EXADR)
					       || (cfg->address->content + spi_context_all_buf_length(tx_bufs) >=
						   QSPI_EXAR_EXADR))) {
			LOG_ERR("Write address out of range!");
			return -EINVAL;
		}
		ret = qspi_write(dev, cfg);
	}

	spi_context_release(&data->ctx, ret);

	return ret;
}

static int qspi_hc32_transceive(struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
#if (defined CONFIG_SPI_ASYNC)||(defined CONFIG_SPI_SLAVE)
	return -ENOTSUP;
#endif

	if ((NULL == dev) || (NULL == config)) {
		return -EINVAL;
	}

	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL);
}

static int qspi_hc32_release(struct device *dev,
			     const struct spi_config *config)
{
	struct qspi_hc32_data *data = dev->driver_data;
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static const struct spi_driver_api api_funcs = {
	.transceive = qspi_hc32_transceive,
	.release = qspi_hc32_release,
};

static int qspi_hc32_init(struct device *dev)
{
	int ret;
	struct device *clock;
	const struct qspi_hc32_cfg *cfg = dev->config->config_info;
	struct qspi_hc32_data *data = dev->driver_data;
#ifdef CONFIG_QSPI_HC32_DMA
	/* update dma tx dev and dma trig source */
	data->dma_tx.dev = device_get_binding(dma_hc32_get_device_name((uint32_t)(
									       data->dma_tx.dev)));
	if (!data->dma_tx.dev) {
		LOG_ERR("No tx dma dev found!");
		return -EINVAL;
	}

	/* update dma rx dev */
	data->dma_rx.dev = device_get_binding(dma_hc32_get_device_name((uint32_t)(
									       data->dma_rx.dev)));
	if (!data->dma_rx.dev) {
		LOG_ERR("No rx dma dev found!");
		return -EINVAL;
	}
#endif

	clock = device_get_binding(CLOCK_CONTROL);
	/* spi clock open */
	(void)clock_control_on(clock, (clock_control_subsys_t)&cfg->qspi_clk);

	ret = QSPI_DeInit();
	if (ret < 0) {
		return ret;
	}

	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

/* for device register */
#define DMA_INITIALIZER_TX(inst)                                                                       \
	{                                                                                                  \
		.dev = (struct device *)HC32_DT_GET_DMA_UNIT((uint32_t)DT_XHSC_HC32_QSPI_##inst##_DMA_TX_CFG), \
		.channel = (uint8_t)HC32_DT_GET_DMA_CH((uint32_t)DT_XHSC_HC32_QSPI_##inst##_DMA_TX_CFG),       \
		.dma_cfg_user_data = {                                                                         \
			.slot = EVT_SRC_AOS_STRG,                                                                  \
		},                                                                                             \
		.dma_cfg = {                                                                                   \
		.channel_direction = MEMORY_TO_MEMORY,	                                                       \
		.source_burst_length = 1,  /* SINGLE transfer */                                               \
		.dest_burst_length = 1,    /* SINGLE transfer */                                               \
		.block_count = 1,                                                                              \
		.dma_callback = qspi_dma_callback,                                                             \
		.complete_callback_en = 0,                                                                     \
		.error_callback_en = 0,                                                                        \
		},                                                                                             \
	}

#define DMA_INITIALIZER_RX(inst)                                                                       \
	{                                                                                                  \
		.dev = (struct device *)HC32_DT_GET_DMA_UNIT((uint32_t)DT_XHSC_HC32_QSPI_##inst##_DMA_RX_CFG), \
		.channel = (uint8_t)HC32_DT_GET_DMA_CH((uint32_t)DT_XHSC_HC32_QSPI_##inst##_DMA_RX_CFG),       \
		.dma_cfg_user_data = {                                                                         \
			.slot = EVT_SRC_AOS_STRG,                                                                  \
		},                                                                                             \
		.dma_cfg = {                                                                                   \
		.channel_direction = MEMORY_TO_MEMORY,	                                                       \
		.source_burst_length = 1,  /* SINGLE transfer */                                               \
		.dest_burst_length = 1,    /* SINGLE transfer */                                               \
		.block_count = 1,                                                                              \
		.dma_callback = qspi_dma_callback,                                                             \
		.complete_callback_en = 0,                                                                     \
		.error_callback_en = 0                                                                         \
		},                                                                                             \
	}

#ifdef CONFIG_QSPI_HC32_DMA
#define DMAS_DECL(inst)                                 \
		.dma_rx = DMA_INITIALIZER_RX(inst),             \
		.dma_tx = DMA_INITIALIZER_TX(inst),
#define QSPI_DMA_STATUS_SEM(inst)                       \
		.status_sem = Z_SEM_INITIALIZER(                \
		qspi_hc32_data_##inst.status_sem, 0, 1),
#else
#define DMAS_DECL(inst)
#define QSPI_DMA_STATUS_SEM(inst)
#endif

#define QSPI_HC32_DEVICE_INIT(idx)                           \
DEVICE_DECLARE(qspi_hc32_##idx);                             \
static struct qspi_hc32_data qspi_hc32_data_##idx = {        \
	SPI_CONTEXT_INIT_LOCK(qspi_hc32_data_##idx, ctx),        \
	SPI_CONTEXT_INIT_SYNC(qspi_hc32_data_##idx, ctx),        \
	QSPI_DMA_STATUS_SEM(idx)                                 \
	DMAS_DECL(idx)                                           \
};                                                           \
static const struct qspi_hc32_cfg qspi_hc32_cfg_##idx = {    \
	.qspi_clk ={                                             \
		.bus = DT_XHSC_HC32_QSPI_##idx##_CLOCK_BUS,          \
		.fcg = DT_XHSC_HC32_QSPI_##idx##_CLOCK_FCG,          \
		.bits = DT_XHSC_HC32_QSPI_##idx##_CLOCK_BITS         \
	},                                                       \
};\
DEVICE_AND_API_INIT(qspi_hc32_##idx, DT_XHSC_HC32_QSPI_##idx##_LABEL, &qspi_hc32_init,  \
		    &qspi_hc32_data_##idx, &qspi_hc32_cfg_##idx,                                \
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                                      \
		    &api_funcs);

QSPI_HC32_DEVICE_INIT(0)

