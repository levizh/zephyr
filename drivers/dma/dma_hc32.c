/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <dma.h>
#include <errno.h>
#include <init.h>
#include <stdio.h>
#include <soc.h>
#include <string.h>
#include <misc/util.h>
#include <drivers/dma/dma_hc32.h>
#include <interrupt_controller/intc_hc32.h>
#include <clock_control/hc32_clock_control.h>

#define LOG_LEVEL CONFIG_DMA_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(dma_hc32);

#include "hc32_ll.h"

#define DMA_MCU_MAX_BLOCK_SIZE	(1024UL)
#define DMA_MCU_MAX_BLOCK_CNT   (65535UL)
#define DMA_MCU_BLOCK_SIZE_MASK (DMA_DTCTL_BLKSIZE >> DMA_DTCTL_BLKSIZE_POS)

struct dma_hc32_config {
	u32_t base;
	u32_t channels;
	void (*fcg_configure)(void);
	void (*irq_configure)(void);
	void (*aos_configure)(void);
};

struct dma_hc32_channel {
	void (*callback)(void *callback_arg, u32_t channel,
			 int error_code);
	void *user_data;
	u32_t direction;
	u32_t data_width;
	u32_t burst_length;
	bool busy;
	u32_t aos_target;
	en_event_src_t aos_source;
	u32_t m2m_trigcnt;
	u32_t m2m_lastpkg;
	int m2m_err;
};

struct dma_hc32_data {
	struct dma_hc32_channel *channels;
};

struct dma_hc32_ddl_config {
	u32_t u32BlockSize;
	u32_t u32TransCount;
	u32_t u32DataWidth;
	u32_t u32DestAddr;
	u32_t u32SrcAddr;
	u32_t u32SrcAddrInc;
	u32_t u32DestAddrInc;
};

/* Check if the DMA init successfully for HC32F460 */
#define DMA_CH_REG(reg_base, ch)        (*(__IO uint32_t *)((uint32_t)(&(reg_base)) + ((ch) * 0x40UL)))

static int dma_hc32_init_check(CM_DMA_TypeDef *DMAx, uint32_t channel,
			       const stc_dma_init_t *pstcDmaInit)
{
	__IO uint32_t *CHCTLx;
	uint32_t u32BitsPos, u32BitsSetVal;
	uint32_t u32BlockSize = pstcDmaInit->u32BlockSize & DMA_MCU_BLOCK_SIZE_MASK;

	u32BitsPos    = DMA_CHCTL_IE | DMA_CHCTL_HSIZE | DMA_CHCTL_SINC |
			DMA_CHCTL_DINC;
	u32BitsSetVal = pstcDmaInit->u32IntEn | pstcDmaInit->u32DataWidth |
			pstcDmaInit->u32SrcAddrInc | pstcDmaInit->u32DestAddrInc;
	CHCTLx = &DMA_CH_REG(DMAx->CHCTL0, channel);

	/* read back reg val to check */
	if ((pstcDmaInit->u32SrcAddr != DMA_GetSrcAddr(DMAx, channel)) ||      \
	    (pstcDmaInit->u32DestAddr != DMA_GetDestAddr(DMAx, channel)) ||    \
	    (pstcDmaInit->u32TransCount != DMA_GetTransCount(DMAx, channel)) || \
	    (u32BlockSize != DMA_GetBlockSize(DMAx, channel)) || \
	    (u32BitsSetVal != READ_REG32_BIT(*CHCTLx, u32BitsPos))) {
		return -EBUSY;
	}
	return 0;
}

static u32_t dma_hc32_ch_is_en(CM_DMA_TypeDef *DMAx, uint32_t channel)
{
	u32_t u32ChEn;
	uint32_t u32Temp;
	u32Temp = (DMAx->CHEN & (0x01UL << channel));
	if (0UL != u32Temp) {
		u32ChEn = 1UL;
	} else {
		u32ChEn = 0UL;
	}
	return u32ChEn;
}

static void dma_hc_sw_trigger(CM_DMA_TypeDef *DMAx, uint32_t channel)
{
	__IO uint32_t *SWREQ;
	SWREQ = (__IO uint32_t *)((uint32_t)DMAx + 0x30U);
	WRITE_REG32(*SWREQ, (0xA1UL << 16U) | (0x01UL << channel));
}

static int dma_hc32_get_aos_target(CM_DMA_TypeDef *DMAx, u32_t channel,
				   u32_t *aos_target)
{
	u32_t aos_dma_trgsle_base;
	if (DMAx == CM_DMA1) {
		aos_dma_trgsle_base = AOS_DMA1_0;
	} else if (DMAx == CM_DMA2) {
		aos_dma_trgsle_base = AOS_DMA2_0;
	} else {
		LOG_ERR("DMAx error");
		return -EINVAL;
	}
	*aos_target = aos_dma_trgsle_base  + 0x04 * channel;
	return 0;
}

static void dma_hc32_m2m_calc_trigcnt_lastpkg_blocksize(u32_t *trigcnt,
							u32_t *lastpkg, u32_t *blocksize)
{
	u32_t in_blocksize = *blocksize, out_blocksize;
	u32_t m2m_trigcnt, m2m_lastpkg;
	m2m_trigcnt = in_blocksize / DMA_MCU_MAX_BLOCK_SIZE;
	m2m_lastpkg = in_blocksize % DMA_MCU_MAX_BLOCK_SIZE;
	if (m2m_lastpkg != 0) {
		m2m_trigcnt++;
	} else {
		m2m_lastpkg = DMA_MCU_MAX_BLOCK_SIZE;
	}
	if (m2m_trigcnt != 1) {
		out_blocksize = DMA_MCU_MAX_BLOCK_SIZE;
	} else {
		out_blocksize = m2m_lastpkg;
	}
	*trigcnt = m2m_trigcnt;
	*lastpkg = m2m_lastpkg;
	*blocksize = out_blocksize;

}

static void dma_hc32_clear_int_flag(CM_DMA_TypeDef *DMAx, u32_t channel)
{
	/* Int status clear */
	DMA_ClearTransCompleteStatus(DMAx,
				     (DMA_FLAG_BTC_CH0 << channel) | (DMA_FLAG_TC_CH0 << channel));
	DMA_ClearErrStatus(DMAx,
			   ((DMA_INT_REQ_ERR_CH0 << channel) | (DMA_INT_TRANS_ERR_CH0 << channel)));
}

static int dma_hc32_configure(struct device *dev, u32_t channel,
			      struct dma_config *dma_cfg)
{
	const struct dma_hc32_config *cfg = dev->config->config_info;
	const struct dma_hc32_config_user_data *dma_cfg_user_data =
			dma_cfg->callback_arg;
	struct dma_hc32_data *data = dev->driver_data;
	struct dma_hc32_ddl_config dma_ddl_cfg;
	stc_dma_init_t stcDmaInit;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	if (channel >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}

	if (data->channels[channel].busy) {
		LOG_ERR("dma channel %d is busy.", channel);
		return -EBUSY;
	}

	/* dam_cfg parameters check */

	/* These dma_cfg parameters do not care :
		dma_cfg->source_handshake
		dma_cfg->dest_handshake
		dma_cfg->channel_priority
	*/

	if ((dma_cfg->source_chaining_en != 0) || (dma_cfg->dest_chaining_en != 0)) {
		LOG_ERR("channel chaining transfer not supported now.");
		return -ENOTSUP;
	}

	if (dma_cfg->block_count != 1) {
		LOG_ERR("block_count != 1 not supported now.");
		return -ENOTSUP;
	}

	if (dma_cfg->source_data_size != dma_cfg->dest_data_size) {
		LOG_ERR("source_data_size != dest_data_size transfer not supported.");
		return -ENOTSUP;
	}

	if (dma_cfg->source_burst_length != dma_cfg->dest_burst_length) {
		LOG_ERR("source_burst_length != dest_burst_length transfer not supported.");
		return -ENOTSUP;
	}

	if (dma_cfg->source_burst_length > DMA_MCU_MAX_BLOCK_SIZE) {
		LOG_ERR("burst_length too large.");
		return -EINVAL;
	}

	/* block_cfg parameters check */
	if ((dma_cfg->head_block->source_gather_en != 0)
	    || (dma_cfg->head_block->dest_scatter_en != 0)) {
		LOG_ERR("block source_gather or dest_scatter transfer not supported now.");
		return -ENOTSUP;
	}

	if ((dma_cfg->head_block->source_reload_en != 0)
	    || (dma_cfg->head_block->dest_reload_en != 0)) {
		LOG_ERR("block source_reload or dest_reload transfer not supported now.");
		return -ENOTSUP;
	}

	if ((dma_cfg->head_block->block_size % (dma_cfg->source_data_size *
						dma_cfg->source_burst_length)) != 0) {
		LOG_ERR("block_size is not an integer multiple of (data_size*burst_length).");
		return -ENOTSUP;
	}

	if (dma_cfg->complete_callback_en != 0) {
		LOG_ERR("support callback invoked at completion only.");
		return -ENOTSUP;
	}

	/* dma_cfg->head_block->block_size is num of bytes to be transfer */
	if (dma_cfg->channel_direction == MEMORY_TO_MEMORY) {
		/* mem to mem single trig: cnt=0
		   cfg_block_size ---> mcu_dma_block_size
		   mcu_dma_block_cnt fix to 1UL */
		dma_ddl_cfg.u32BlockSize = dma_cfg->head_block->block_size /
					   dma_cfg->source_data_size;
		dma_ddl_cfg.u32TransCount = 1UL;
	} else {
		/* others peripheral trig : cnt--
		   cfg_block_size ---> mcu_dma_block_cnt
		   mcu_dma_block_size set to dma_cfg->source_burst_length */
		dma_ddl_cfg.u32BlockSize = dma_cfg->source_burst_length;
		dma_ddl_cfg.u32TransCount = dma_cfg->head_block->block_size /
					    (dma_cfg->source_data_size * dma_cfg->source_burst_length);
		if (dma_ddl_cfg.u32TransCount > DMA_MCU_MAX_BLOCK_CNT) {
			LOG_ERR("block size must be < %" PRIu32 " (%" PRIu32 ")",
				(u32_t)(DMA_MCU_MAX_BLOCK_CNT / (dma_cfg->source_data_size *
								 dma_cfg->source_burst_length)),
				dma_cfg->head_block->block_size);
			return -EINVAL;
		}
	}

	switch (dma_cfg->source_data_size) {
	case 1U:
		dma_ddl_cfg.u32DataWidth = DMA_DATAWIDTH_8BIT;
		break;
	case 2U:
		dma_ddl_cfg.u32DataWidth = DMA_DATAWIDTH_16BIT;
		break;
	case 4U:
		dma_ddl_cfg.u32DataWidth = DMA_DATAWIDTH_32BIT;
		break;
	default:
		LOG_ERR("source_data_size error.");
		return -ENOTSUP;
		break;
	}

	dma_ddl_cfg.u32SrcAddr  = dma_cfg->head_block->source_address;
	dma_ddl_cfg.u32DestAddr = dma_cfg->head_block->dest_address;

	if (dma_cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		dma_ddl_cfg.u32SrcAddrInc = DMA_SRC_ADDR_INC;
	} else if (dma_cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		dma_ddl_cfg.u32SrcAddrInc = DMA_SRC_ADDR_DEC;
	} else {
		dma_ddl_cfg.u32SrcAddrInc = DMA_SRC_ADDR_FIX;
	}

	if (dma_cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		dma_ddl_cfg.u32DestAddrInc = DMA_DEST_ADDR_INC;
	} else if (dma_cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		dma_ddl_cfg.u32DestAddrInc = DMA_DEST_ADDR_DEC;
	} else {
		dma_ddl_cfg.u32DestAddrInc = DMA_DEST_ADDR_FIX;
	}

	data->channels[channel].callback = dma_cfg->dma_callback;
	data->channels[channel].user_data = dma_cfg->callback_arg;
	data->channels[channel].direction = dma_cfg->channel_direction;
	data->channels[channel].data_width = dma_cfg->source_data_size;
	data->channels[channel].burst_length = dma_cfg->source_burst_length;
	if (data->channels[channel].direction == MEMORY_TO_MEMORY) {
		dma_hc32_m2m_calc_trigcnt_lastpkg_blocksize(
			&data->channels[channel].m2m_trigcnt, &data->channels[channel].m2m_lastpkg,
			&dma_ddl_cfg.u32BlockSize);
	}

	if (dma_cfg_user_data != NULL) {
		data->channels[channel].aos_source = dma_cfg_user_data->slot;
	} else {
		LOG_WRN("hc32 dma should use config->callback_arg to select trig source, but is NULL !");
	}
	if (0 != dma_hc32_get_aos_target(DMAx, channel,
					 &data->channels[channel].aos_target)) {
		return -EINVAL;
	}

	if ((data->channels[channel].direction == MEMORY_TO_MEMORY)
	    && (data->channels[channel].aos_source != EVT_SRC_AOS_STRG)) {
		LOG_WRN("note: dma mem to mem, but trig source not EVT_SRC_AOS_STRG !");
		data->channels[channel].aos_source = EVT_SRC_AOS_STRG;
		LOG_WRN("note: force trig source to EVT_SRC_AOS_STRG !");
	}

	if (dma_hc32_ch_is_en(DMAx, channel)) {
		if (LL_OK != DMA_ChCmd(DMAx, channel, DISABLE)) {
			LOG_ERR("DMA ch disable failed");
			return -EBUSY;
		}
	}

	/* Int status clear */
	dma_hc32_clear_int_flag(DMAx, channel);

	/* MEMORY_TO_MEMORY: use dma independent sw trigger */
	if (MEMORY_TO_MEMORY != data->channels[channel].direction) {
		AOS_SetTriggerEventSrc(data->channels[channel].aos_target,
				       data->channels[channel].aos_source);
	}

	(void)DMA_StructInit(&stcDmaInit);

	stcDmaInit.u32IntEn      = DMA_INT_ENABLE;
	stcDmaInit.u32BlockSize  = dma_ddl_cfg.u32BlockSize;
	stcDmaInit.u32TransCount = dma_ddl_cfg.u32TransCount;
	stcDmaInit.u32DataWidth  = dma_ddl_cfg.u32DataWidth;
	stcDmaInit.u32SrcAddr    = dma_ddl_cfg.u32SrcAddr;
	stcDmaInit.u32DestAddr   = dma_ddl_cfg.u32DestAddr;
	stcDmaInit.u32SrcAddrInc = dma_ddl_cfg.u32SrcAddrInc;
	stcDmaInit.u32DestAddrInc = dma_ddl_cfg.u32DestAddrInc;

	(void)DMA_Init(DMAx, channel, &stcDmaInit);
	if (0 != dma_hc32_init_check(DMAx, channel, &stcDmaInit)) {
		LOG_ERR("DMA ch init failed");
		return -EBUSY;
	}

	/* only TC enable, BTC disable */
	DMA_TransCompleteIntCmd(DMAx, DMA_INT_BTC_CH0 << channel, DISABLE);

	if (dma_cfg->error_callback_en != 0) {
		DMA_ErrIntCmd(DMAx, (DMA_INT_REQ_ERR_CH0 << channel) | (DMA_INT_TRANS_ERR_CH0 <<
									channel), DISABLE);
	}

	DMA_Cmd(DMAx, ENABLE);

	return 0;
}

static int dma_hc32_reload(struct device *dev, u32_t channel,
			   u32_t src, u32_t dst, size_t size)
{
	const struct dma_hc32_config *cfg = dev->config->config_info;
	struct dma_hc32_data *data = dev->driver_data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);
	u32_t u32TransSize;

	if (channel >= cfg->channels) {
		LOG_ERR("reload channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}

	if (data->channels[channel].busy) {
		LOG_ERR("dma channel %d is busy.", channel);
		return -EBUSY;
	}

	if ((size % (data->channels[channel].data_width *
		     data->channels[channel].burst_length)) != 0) {
		LOG_ERR("size is not an integer multiple of data_width*.burst_length");
		return -ENOTSUP;
	}

	if (data->channels[channel].direction == MEMORY_TO_MEMORY) {
		u32TransSize = size / data->channels[channel].data_width;
		dma_hc32_m2m_calc_trigcnt_lastpkg_blocksize(
			&data->channels[channel].m2m_trigcnt, &data->channels[channel].m2m_lastpkg,
			&u32TransSize);
	} else {
		u32TransSize = size / (data->channels[channel].data_width *
				       data->channels[channel].burst_length);
		if (u32TransSize > DMA_MCU_MAX_BLOCK_CNT) {
			LOG_ERR("block size must be < %" PRIu32 " (%" PRIu32 ")",
				(u32_t)(DMA_MCU_MAX_BLOCK_CNT / data->channels[channel].data_width),
				size);
			return -EINVAL;
		}
	}

	if (dma_hc32_ch_is_en(DMAx, channel)) {
		if (LL_OK != DMA_ChCmd(DMAx, channel, DISABLE)) {
			LOG_ERR("DMA ch disable failed");
			return -EBUSY;
		}
	}

	/* Int status clear */
	dma_hc32_clear_int_flag(DMAx, channel);

	if (LL_OK != DMA_SetSrcAddr(DMAx, channel, src)) {
		LOG_ERR("DMA set src addr failed");
		return -EBUSY;
	}
	if (LL_OK != DMA_SetDestAddr(DMAx, channel, dst)) {
		LOG_ERR("DMA set dest addr failed");
		return -EBUSY;
	}

	if (data->channels[channel].direction == MEMORY_TO_MEMORY) {
		u32TransSize = u32TransSize & DMA_MCU_BLOCK_SIZE_MASK;
		if (LL_OK != DMA_SetBlockSize(DMAx, channel, u32TransSize)) {
			LOG_ERR("DMA set block size failed");
			return -EBUSY;
		}
		if (LL_OK != DMA_SetTransCount(DMAx, channel, 1U)) {
			LOG_ERR("DMA set trans count failed");
			return -EBUSY;
		}
	} else {
		if (LL_OK != DMA_SetTransCount(DMAx, channel, u32TransSize)) {
			LOG_ERR("DMA set trans count failed");
			return -EBUSY;
		}
	}

	return 0;
}

static int dma_hc32_start(struct device *dev, u32_t channel)
{
	const struct dma_hc32_config *cfg = dev->config->config_info;
	struct dma_hc32_data *data = dev->driver_data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);
	unsigned int key;

	if (channel >= cfg->channels) {
		LOG_ERR("start channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}

	/* Int status clear */
	dma_hc32_clear_int_flag(DMAx, channel);

	if (LL_OK != DMA_ChCmd(DMAx, channel, ENABLE)) {
		LOG_ERR("DMA ch enable failed");
		return -EBUSY;
	}
	data->channels[channel].busy = true;
	if (data->channels[channel].direction == MEMORY_TO_MEMORY) {
		if (EVT_SRC_AOS_STRG == data->channels[channel].aos_source) {
			key = irq_lock();
			dma_hc_sw_trigger(DMAx, channel);
			data->channels[channel].m2m_trigcnt--;
			data->channels[channel].m2m_err = 0;
			irq_unlock(key);
		}
	}

	return 0;
}


static int dma_hc32_stop(struct device *dev, u32_t channel)
{
	const struct dma_hc32_config *cfg = dev->config->config_info;
	struct dma_hc32_data *data = dev->driver_data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	if (channel >= cfg->channels) {
		LOG_ERR("start channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}
	if (dma_hc32_ch_is_en(DMAx, channel)) {
		if (LL_OK != DMA_ChCmd(DMAx, channel, DISABLE)) {
			LOG_ERR("DMA ch disable failed");
			return -EBUSY;
		}
	}
	data->channels[channel].busy = false;

	return 0;
}

int dma_hc32_get_status(struct device *dev, u32_t channel,
			struct dma_hc32_status *stat)
{
	const struct dma_hc32_config *cfg = dev->config->config_info;
	struct dma_hc32_data *data = dev->driver_data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);
	uint32_t pending_trigcnt;
	uint32_t u32BlockSize;

	if (channel >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}

	u32BlockSize = DMA_GetBlockSize(DMAx, channel);
	u32BlockSize = (0UL == u32BlockSize) ? DMA_MCU_MAX_BLOCK_SIZE : u32BlockSize;

	stat->pending_length = DMA_GetTransCount(DMAx, channel) * u32BlockSize * \
			       data->channels[channel].data_width;
	if (data->channels[channel].direction == MEMORY_TO_MEMORY) {
		pending_trigcnt = data->channels[channel].m2m_trigcnt;
		if (pending_trigcnt >= 1) {
			stat->pending_length = stat->pending_length + ((pending_trigcnt - 1) *
								       DMA_MCU_MAX_BLOCK_SIZE + data->channels[channel].m2m_lastpkg) *
					       data->channels[channel].data_width;
		}
	}

	stat->dir = data->channels[channel].direction;
	stat->busy = data->channels[channel].busy;

	return 0;
}

static const struct dma_driver_api dma_hc32_driver_api = {
	.config = dma_hc32_configure,
	.reload = dma_hc32_reload,
	.start = dma_hc32_start,
	.stop = dma_hc32_stop,
};

static void dma_hc32_err_irq_handler(struct device *dev)
{
	const struct dma_hc32_config *cfg = dev->config->config_info;
	struct dma_hc32_data *data = dev->driver_data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	for (u32_t ch = 0; ch < cfg->channels; ch++) {
		if (DMAx->INTMASK0 & ((DMA_INT_REQ_ERR_CH0 << ch) |
				      (DMA_INT_TRANS_ERR_CH0 << ch))) {
			if (SET == DMA_GetErrStatus(DMAx,
						    ((DMA_INT_REQ_ERR_CH0 << ch) | (DMA_INT_TRANS_ERR_CH0 << ch)))) {
				DMA_ClearErrStatus(DMAx,
						   ((DMA_INT_REQ_ERR_CH0 << ch) | (DMA_INT_TRANS_ERR_CH0 << ch)));
				data->channels[ch].busy = false;
				if (data->channels[ch].callback) {
					data->channels[ch].callback(data->channels[ch].user_data,
								    ch, -EIO);
				}
			}
		}
	}
}

static void dma_hc32_tc_irq_handler(struct device *dev, int channel)
{
	const struct dma_hc32_config *cfg = dev->config->config_info;
	struct dma_hc32_data *data = dev->driver_data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);
	u32_t u32TransSize;
	unsigned int key;

	if (SET == DMA_GetTransCompleteStatus(DMAx, DMA_FLAG_TC_CH0 << channel)) {
		DMA_ClearTransCompleteStatus(DMAx, DMA_FLAG_TC_CH0 << channel);
		if (data->channels[channel].direction == MEMORY_TO_MEMORY) {
			if (data->channels[channel].m2m_trigcnt == 0) { /* all done */
				data->channels[channel].busy = false;
				if (data->channels[channel].callback) {
					data->channels[channel].callback(data->channels[channel].user_data,
									 channel, data->channels[channel].m2m_err);
				}
			} else {
				if (data->channels[channel].m2m_trigcnt == 1) { /* last pkg */
					u32TransSize = data->channels[channel].m2m_lastpkg;
				} else {
					u32TransSize = DMA_MCU_MAX_BLOCK_SIZE;
				}
				u32TransSize = u32TransSize & DMA_MCU_BLOCK_SIZE_MASK;
				if ((LL_OK != DMA_SetBlockSize(DMAx, channel, u32TransSize)) || \
				    (LL_OK != DMA_SetTransCount(DMAx, channel, 1U))) {
					data->channels[channel].m2m_err = -EIO;
				}

				if (LL_OK != DMA_ChCmd(DMAx, channel, ENABLE)) {
					data->channels[channel].m2m_err = -EIO;
					/* report err and return */
					if (data->channels[channel].callback) {
						data->channels[channel].callback(data->channels[channel].user_data,
										 channel, data->channels[channel].m2m_err);
					}
					return;
				}
				if (EVT_SRC_AOS_STRG == data->channels[channel].aos_source) {
					key = irq_lock();
					dma_hc_sw_trigger(DMAx, channel);
					data->channels[channel].m2m_trigcnt--;
					irq_unlock(key);
				}
			}
		} else {
			data->channels[channel].busy = false;
			if (data->channels[channel].callback) {
				data->channels[channel].callback(data->channels[channel].user_data,
								 channel, 0);
			}
		}
	}
}

static int dma_hc32_init(struct device *dev)
{
	const struct dma_hc32_config *cfg = dev->config->config_info;
	struct dma_hc32_data *data = dev->driver_data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	cfg->fcg_configure();

	for (u32_t ch = 0; ch < cfg->channels; ch++) {
		dma_hc32_clear_int_flag(DMAx, ch);
		data->channels[ch].busy = false;
	}

	cfg->aos_configure();
	cfg->irq_configure();

	return 0;
}

#define DMA_HC32_DEFINE_TC_IRQ_HANDLER(ch, inst)                      \
static void dma_hc32_tc_irq_handler_##inst##_##ch(struct device *dev) \
{                                                                     \
	dma_hc32_tc_irq_handler(dev, ch);                                 \
}

#define DMA_HC32_DEFINE_ERR_IRQ_HANDLER(inst)                         \
static void dma_hc32_err_irq_handler_##inst(struct device *dev)       \
{                                                                     \
	dma_hc32_err_irq_handler(dev);                                    \
}

#define DMA_HC32_TC_IRQ_CONFIG(ch, inst)                              \
{                                                                     \
	IRQ_CONNECT(DT_XHSC_HC32_DMA_##inst##_IRQ_TC##ch,                 \
		    DT_XHSC_HC32_DMA_##inst##_IRQ_TC##ch##_PRIORITY,          \
			dma_hc32_tc_irq_handler_##inst##_##ch,                    \
		    DEVICE_GET(dma_hc32_##inst), 0);                          \
	hc32_intc_irq_signin(DT_XHSC_HC32_DMA_##inst##_IRQ_TC##ch,        \
			     DT_XHSC_HC32_DMA_##inst##_IRQ_TC##ch##_INT_SRC);     \
	irq_enable(DT_XHSC_HC32_DMA_##inst##_IRQ_TC##ch);                 \
}

#define DMA_HC32_ERR_IRQ_CONFIG(inst)                                 \
{                                                                     \
	IRQ_CONNECT(DT_XHSC_HC32_DMA_##inst##_IRQ_ERR,                    \
		    DT_XHSC_HC32_DMA_##inst##_IRQ_ERR_PRIORITY,               \
			 dma_hc32_err_irq_handler_##inst,                         \
		    DEVICE_GET(dma_hc32_##inst), 0);                          \
	hc32_intc_irq_signin(DT_XHSC_HC32_DMA_##inst##_IRQ_ERR,           \
			     DT_XHSC_HC32_DMA_##inst##_IRQ_ERR_INT_SRC);          \
	irq_enable(DT_XHSC_HC32_DMA_##inst##_IRQ_ERR);                    \
}

#define DMA_HC32_DEFINE_IRQ_CONFIG(inst)                                              \
DEVICE_DECLARE(dma_hc32_##inst);                                                      \
static void dma_hc32_##inst##_irq_configure(void)                                     \
{                                                                                     \
	UTIL_LISTIFY(DT_XHSC_HC32_DMA_##inst##_DMA_CHANNELS, DMA_HC32_TC_IRQ_CONFIG, inst)\
	DMA_HC32_ERR_IRQ_CONFIG(inst)                                                     \
}

#define DMA_HC32_DEFINE_FCG_CONFIG(inst)                                              \
static void dma_hc32_##inst##_fcg_configure(void)                                     \
{                                                                                     \
	struct device *clock = device_get_binding(CLOCK_CONTROL);                         \
	static struct hc32_modules_clock_sys fcg_cfg[2] = {                               \
		{                                                                             \
			.bus = DT_XHSC_HC32_DMA_##inst##_CLOCK_BUS_0,                             \
			.fcg = DT_XHSC_HC32_DMA_##inst##_CLOCK_FCG_0,                             \
			.bits = DT_XHSC_HC32_DMA_##inst##_CLOCK_BITS_0                            \
		},                                                                            \
		{                                                                             \
			.bus = DT_XHSC_HC32_DMA_##inst##_CLOCK_BUS_1,                             \
			.fcg = DT_XHSC_HC32_DMA_##inst##_CLOCK_FCG_1,                             \
			.bits = DT_XHSC_HC32_DMA_##inst##_CLOCK_BITS_1                            \
		}                                                                             \
	};                                                                                \
	(void)clock_control_on(clock, (clock_control_subsys_t)&fcg_cfg[0]);               \
	(void)clock_control_on(clock, (clock_control_subsys_t)&fcg_cfg[1]);               \
}

#define DMA_HC32_AOS_CH_CONFIGURE(ch, inst)                                           \
	dma_hc32_##inst##_data.channels[ch].aos_source = EVT_SRC_AOS_STRG;

#define DMA_HC32_DEFINE_AOS_CONFIG(inst)                                                  \
static void dma_hc32_##inst##_aos_configure(void)                                         \
{                                                                                         \
	UTIL_LISTIFY(DT_XHSC_HC32_DMA_##inst##_DMA_CHANNELS, DMA_HC32_AOS_CH_CONFIGURE, inst) \
}

#define HC32_DMA_INIT(inst)                                                                   \
	UTIL_LISTIFY(DT_XHSC_HC32_DMA_##inst##_DMA_CHANNELS, DMA_HC32_DEFINE_TC_IRQ_HANDLER, inst)\
	DMA_HC32_DEFINE_ERR_IRQ_HANDLER(inst)                                                     \
	static struct dma_hc32_channel                                                            \
		dma_hc32_##inst##_channels[DT_XHSC_HC32_DMA_##inst##_DMA_CHANNELS];                   \
	static struct dma_hc32_data dma_hc32_##inst##_data = {                                    \
		.channels = dma_hc32_##inst##_channels,                                               \
	};                                                                                        \
	DMA_HC32_DEFINE_IRQ_CONFIG(inst)                                                          \
	DMA_HC32_DEFINE_FCG_CONFIG(inst)                                                          \
	DMA_HC32_DEFINE_AOS_CONFIG(inst)                                                          \
	static const struct dma_hc32_config dma_hc32_##inst##_config = {                          \
		.base = DT_XHSC_HC32_DMA_##inst##_BASE_ADDRESS,                                       \
		.channels = DT_XHSC_HC32_DMA_##inst##_DMA_CHANNELS,                                   \
		.fcg_configure = dma_hc32_##inst##_fcg_configure,                                     \
		.irq_configure = dma_hc32_##inst##_irq_configure,                                     \
		.aos_configure = dma_hc32_##inst##_aos_configure,                                     \
	};                                                                                        \
                                                                                              \
	DEVICE_AND_API_INIT(dma_hc32_##inst, DT_XHSC_HC32_DMA_##inst##_LABEL, &dma_hc32_init,     \
		    	&dma_hc32_##inst##_data, &dma_hc32_##inst##_config,                           \
		    	PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                             \
		    	(void *)&dma_hc32_driver_api);                                                \

#if defined(DT_XHSC_HC32_DMA_0)
#if (DT_XHSC_HC32_DMA_0)
HC32_DMA_INIT(0)
#endif
#endif

#if defined(DT_XHSC_HC32_DMA_0)
#if (DT_XHSC_HC32_DMA_1)
HC32_DMA_INIT(1)
#endif
#endif

#if (!defined(DT_XHSC_HC32_DMA_0)) && (!defined(DT_XHSC_HC32_DMA_1))
#error "No DMA nodes are okay in the DTS"
#endif
