/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/dma/dma_hc32.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>

#include "hc32_ll.h"

LOG_MODULE_REGISTER(dma_hc32, CONFIG_DMA_LOG_LEVEL);

#define DT_DRV_COMPAT xhsc_hc32_dma


#define AOS_DMA_INST0_CH0		(AOS_DMA1_0)
#define AOS_DMA_INST0_CH1		(AOS_DMA1_1)
#define AOS_DMA_INST0_CH2		(AOS_DMA1_2)
#define AOS_DMA_INST0_CH3		(AOS_DMA1_3)

#define AOS_DMA_INST1_CH0		(AOS_DMA2_0)
#define AOS_DMA_INST1_CH1		(AOS_DMA2_1)
#define AOS_DMA_INST1_CH2		(AOS_DMA2_2)
#define AOS_DMA_INST1_CH3		(AOS_DMA2_3)


#define DMA_MCU_MAX_BLOCK_SIZE	(1024UL)
#define DMA_MCU_MAX_BLOCK_CNT   (65535UL)

struct dma_hc32_config {
	uint32_t base;
	uint32_t channels;
	void (*irq_configure)(void);
	void (*aos_configure)(void);
};

struct dma_hc32_channel {
	dma_callback_t callback;
	void *user_data;
	uint32_t direction;
	uint32_t data_width;
	bool busy;
	uint32_t aos_target;
	en_event_src_t aos_source;
};

struct dma_hc32_data {
	struct dma_context ctx;
	struct dma_hc32_channel *channels;
};

struct dma_hc32_ddl_config {
	uint32_t u32BlockSize;
	uint32_t u32TransCount;
	uint32_t u32DataWidth;
	uint32_t u32DestAddr;
	uint32_t u32SrcAddr;
	uint32_t u32SrcAddrInc;
	uint32_t u32DestAddrInc;
};

static void dma_hc32_clear_int_flag(CM_DMA_TypeDef *DMAx, uint32_t channel)
{
	/* Int status clear */
	DMA_ClearTransCompleteStatus(DMAx,
				     (DMA_FLAG_BTC_CH0 << channel) | (DMA_FLAG_TC_CH0 << channel));
	DMA_ClearErrStatus(DMAx,
			   ((DMA_INT_REQ_ERR_CH0 << channel) | (DMA_INT_TRANS_ERR_CH0 << channel)));
}

static int dma_hc32_config(const struct device *dev, uint32_t channel,
			   struct dma_config *dma_cfg)
{
	const struct dma_hc32_config *cfg = dev->config;
	const struct dma_hc32_config_user_data *dma_cfg_user_data = dma_cfg->user_data;
	struct dma_hc32_data *data = dev->data;
	struct dma_hc32_ddl_config dma_ddl_cfg;
	stc_dma_init_t stcDmaInit;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	if (channel >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}

	/* dam_cfg parameters check */

	/* These dma_cfg parameters do not care :
		dma_cfg->source_handshake
		dma_cfg->dest_handshake
		dma_cfg->channel_priority
		dma_cfg->source_burst_length
		dma_cfg->dest_burst_length
	*/

	if ((dma_cfg->source_chaining_en != 0) || (dma_cfg->dest_chaining_en != 0)) {
		LOG_ERR("channel chaining transfer not supported now.");
		return -ENOTSUP;
	}

	if (dma_cfg->cyclic != 0) {
		LOG_ERR("cyclic transfer not supported now.");
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

	if ((dma_cfg->head_block->block_size % dma_cfg->source_data_size) != 0) {
		LOG_ERR("block_size is not an integer multiple of source_data_size.");
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
		if (dma_ddl_cfg.u32BlockSize > DMA_MCU_MAX_BLOCK_SIZE) {
			LOG_ERR("for MEMORY_TO_MEMORY block size must be < %" PRIu32 " (%" PRIu32 ")",
				(uint32_t)(DMA_MCU_MAX_BLOCK_SIZE / dma_cfg->source_data_size),
				dma_cfg->head_block->block_size);
			return -EINVAL;
		}
	} else {
		/* others peripheral trig : cnt--
		   cfg_block_size ---> mcu_dma_block_cnt
		   mcu_dma_block_size fix to 1UL */
		dma_ddl_cfg.u32BlockSize = 1UL;
		dma_ddl_cfg.u32TransCount = dma_cfg->head_block->block_size /
					    dma_cfg->source_data_size;
		if (dma_ddl_cfg.u32TransCount > DMA_MCU_MAX_BLOCK_CNT) {
			LOG_ERR("block size must be < %" PRIu32 " (%" PRIu32 ")",
				(uint32_t)(DMA_MCU_MAX_BLOCK_CNT / dma_cfg->source_data_size),
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

	if ((dma_cfg->channel_direction == MEMORY_TO_MEMORY)
	    && (dma_cfg_user_data->slot != EVT_SRC_AOS_STRG)) {
		LOG_WRN("note: dma mem to mem, but trig source not EVT_SRC_AOS_STRG !");
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
	data->channels[channel].user_data = dma_cfg->user_data;
	data->channels[channel].direction = dma_cfg->channel_direction;
	data->channels[channel].data_width = dma_cfg->source_data_size;
	data->channels[channel].aos_source = dma_cfg_user_data->slot;

	/* Int status clear */
	dma_hc32_clear_int_flag(DMAx, channel);

	AOS_SetTriggerEventSrc(data->channels[channel].aos_target,
			       data->channels[channel].aos_source);

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

	/* only TC enable, BTC disable */
	DMA_TransCompleteIntCmd(DMAx, DMA_INT_BTC_CH0 << channel, DISABLE);

	if (dma_cfg->error_callback_en == 0) {
		DMA_ErrIntCmd(DMAx, (DMA_INT_REQ_ERR_CH0 << channel) | (DMA_INT_TRANS_ERR_CH0 <<
									channel), DISABLE);
	}

	DMA_Cmd(DMAx, ENABLE);

	return 0;
}

static int dma_hc32_reload(const struct device *dev, uint32_t channel,
			   uint32_t src, uint32_t dst, size_t size)
{
	const struct dma_hc32_config *cfg = dev->config;
	struct dma_hc32_data *data = dev->data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	if (channel >= cfg->channels) {
		LOG_ERR("reload channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}

	if (data->channels[channel].busy) {
		return -EBUSY;
	}

	(void)DMA_ChCmd(DMAx, channel, DISABLE);

	DMA_SetSrcAddr(DMAx, channel, src);
	DMA_SetDestAddr(DMAx, channel, dst);
	DMA_SetBlockSize(DMAx, channel, size / data->channels[channel].data_width);

	(void)DMA_ChCmd(DMAx, channel, ENABLE);

	return 0;
}

static int dma_hc32_start(const struct device *dev, uint32_t channel)
{
	const struct dma_hc32_config *cfg = dev->config;
	struct dma_hc32_data *data = dev->data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	if (channel >= cfg->channels) {
		LOG_ERR("start channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}

	/* Int status clear */
	dma_hc32_clear_int_flag(DMAx, channel);

	(void)DMA_ChCmd(DMAx, channel, ENABLE);
	data->channels[channel].busy = true;

	if (EVT_SRC_AOS_STRG == data->channels[channel].aos_source) {
		AOS_SW_Trigger();
	}

	return 0;
}


static int dma_hc32_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_hc32_config *cfg = dev->config;
	struct dma_hc32_data *data = dev->data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	if (channel >= cfg->channels) {
		LOG_ERR("start channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}
	(void)DMA_ChCmd(DMAx, channel, DISABLE);
	data->channels[channel].busy = false;

	return 0;
}

static int dma_hc32_get_status(const struct device *dev, uint32_t channel,
			       struct dma_status *stat)
{
	const struct dma_hc32_config *cfg = dev->config;
	struct dma_hc32_data *data = dev->data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	if (channel >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")",
			cfg->channels, channel);
		return -EINVAL;
	}

	stat->pending_length = DMA_GetTransCount(DMAx, channel) * DMA_GetBlockSize(DMAx,
										   channel);
	stat->dir = data->channels[channel].direction;
	stat->busy = data->channels[channel].busy;

	return 0;
}

static const struct dma_driver_api dma_hc32_driver_api = {
	.config = dma_hc32_config,
	.reload = dma_hc32_reload,
	.start = dma_hc32_start,
	.stop = dma_hc32_stop,
	.get_status = dma_hc32_get_status,
};

static void dma_hc32_err_irq_handler(const struct device *dev)
{
	const struct dma_hc32_config *cfg = dev->config;
	struct dma_hc32_data *data = dev->data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	for (uint32_t ch = 0; ch < cfg->channels; ch++) {
		if (DMAx->INTMASK0 & ((DMA_INT_REQ_ERR_CH0 << ch) |
				      (DMA_INT_TRANS_ERR_CH0 << ch))) {
			if (SET == DMA_GetErrStatus(DMAx,
						    ((DMA_INT_REQ_ERR_CH0 << ch) | (DMA_INT_TRANS_ERR_CH0 << ch)))) {
				DMA_ClearErrStatus(DMAx,
						   ((DMA_INT_REQ_ERR_CH0 << ch) | (DMA_INT_TRANS_ERR_CH0 << ch)));
				if (data->channels[ch].callback) {
					data->channels[ch].callback(dev, data->channels[ch].user_data,
								    ch, -EIO);
				}
				data->channels[ch].busy = false;
			}
		}
	}
}

static void dma_hc32_tc_irq_handler(const struct device *dev, int channel)
{
	const struct dma_hc32_config *cfg = dev->config;
	struct dma_hc32_data *data = dev->data;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	if (SET == DMA_GetTransCompleteStatus(DMAx, DMA_FLAG_TC_CH0 << channel)) {
		DMA_ClearTransCompleteStatus(DMAx, DMA_FLAG_TC_CH0 << channel);
		if (data->channels[channel].callback) {
			data->channels[channel].callback(dev, data->channels[channel].user_data,
							 channel, DMA_STATUS_COMPLETE);
		}
	}
	data->channels[channel].busy = false;
}


#define DMA_HC32_DEFINE_TC_IRQ_HANDLER(ch, inst)				\
static void dma_hc32_tc_irq_handler_##inst##_##ch(const struct device *dev)	\
{									\
	dma_hc32_tc_irq_handler(dev, ch);				\
}

#define DMA_HC32_DEFINE_ERR_IRQ_HANDLER(inst)				\
static void dma_hc32_err_irq_handler_##inst(const struct device *dev) \
{                                  \
	dma_hc32_err_irq_handler(dev); \
}

#define DEF_ALL_IRQ_HANDLE(inst, chs) LISTIFY(chs, DMA_HC32_DEFINE_TC_IRQ_HANDLER, (), inst) \
										DMA_HC32_DEFINE_ERR_IRQ_HANDLER(inst)


static int dma_hc32_init(const struct device *dev)
{
	const struct dma_hc32_config *cfg = dev->config;
	CM_DMA_TypeDef *DMAx = ((CM_DMA_TypeDef *)cfg->base);

	FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
	if (DMAx == CM_DMA1) {
		FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE);
	} else if (DMAx == CM_DMA2) {
		FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE);
	}

	for (uint32_t ch = 0; ch < cfg->channels; ch++) {
		dma_hc32_clear_int_flag(DMAx, ch);
	}

	cfg->aos_configure();
	cfg->irq_configure();

	return 0;
}

#define DMA_CHANNELS(inst)	DT_INST_PROP(inst, dma_channels)

#define CH_IRQ_CONFIGURE(ch, inst)                                                 \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, ch, irq),                          \
		    DT_INST_IRQ_BY_IDX(inst, ch, priority), dma_hc32_tc_irq_handler_##inst##_##ch,       \
		    DEVICE_DT_INST_GET(inst), 0);                              \
	hc32_intc_irq_signin(DT_PHA_BY_IDX(DT_DRV_INST(inst), intcs, ch, irqn), \
						 DT_PHA_BY_IDX(DT_DRV_INST(inst), intcs, ch, int_src));\
	irq_enable(DT_INST_IRQ_BY_IDX(inst, ch, irq));

#define ERR_IRQ_CONFIGURE(inst)                                                 \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, DMA_CHANNELS(inst), irq),                          \
		    DT_INST_IRQ_BY_IDX(inst, DMA_CHANNELS(inst), priority), dma_hc32_err_irq_handler_##inst,  \
		    DEVICE_DT_INST_GET(inst), 0);                              \
	hc32_intc_irq_signin(DT_PHA_BY_IDX(DT_DRV_INST(inst), intcs, DMA_CHANNELS(inst), irqn), \
						 DT_PHA_BY_IDX(DT_DRV_INST(inst), intcs, DMA_CHANNELS(inst), int_src));\
	irq_enable(DT_INST_IRQ_BY_IDX(inst, DMA_CHANNELS(inst), irq));

#define CONFIGURE_ALL_IRQS(inst, chs) LISTIFY(chs, CH_IRQ_CONFIGURE, (), inst) \
									  ERR_IRQ_CONFIGURE(inst)


#define AOS_CH_CONFIGURE(ch, inst) \
	dma_hc32##inst##_data.channels[ch].aos_target = AOS_DMA_INST##inst##_CH##ch;\
	dma_hc32##inst##_data.channels[ch].aos_source = EVT_SRC_AOS_STRG;

#define CONFIGURE_ALL_AOS(inst, chs) LISTIFY(chs, AOS_CH_CONFIGURE, (), inst)

#define HC32_DMA_INIT(inst)                                                    \
	DEF_ALL_IRQ_HANDLE(inst, DMA_CHANNELS(inst))							\
	static void dma_hc32##inst##_irq_configure(void)                       \
	{                                                                      \
		CONFIGURE_ALL_IRQS(inst, DMA_CHANNELS(inst));      \
	}                                                                      \
	static struct dma_hc32_channel                                         \
		dma_hc32##inst##_channels[DMA_CHANNELS(inst)];   \
	ATOMIC_DEFINE(dma_hc32_atomic##inst, DMA_CHANNELS(inst));                       \
	static struct dma_hc32_data dma_hc32##inst##_data = {                  \
		.ctx =  {                                                      \
			.magic = DMA_MAGIC,                                    \
			.atomic = dma_hc32_atomic##inst,                       \
			.dma_channels = DMA_CHANNELS(inst),      \
		},                                                             \
		.channels = dma_hc32##inst##_channels,                         \
	};                                                                     \
	static void dma_hc32##inst##_aos_configure(void)                       \
	{                                                                      \
		CONFIGURE_ALL_AOS(inst, DMA_CHANNELS(inst));                             \
	}                                                                      \
	static const struct dma_hc32_config dma_hc32##inst##_config = {        \
		.base = DT_INST_REG_ADDR(inst),                                 \
		.channels = DMA_CHANNELS(inst),                  \
		.irq_configure = dma_hc32##inst##_irq_configure,               \
		.aos_configure = dma_hc32##inst##_aos_configure,                    \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(inst, &dma_hc32_init, NULL,                      \
			      &dma_hc32##inst##_data,                          \
			      &dma_hc32##inst##_config, POST_KERNEL,           \
			      CONFIG_DMA_INIT_PRIORITY, &dma_hc32_driver_api);



DT_INST_FOREACH_STATUS_OKAY(HC32_DMA_INIT)
