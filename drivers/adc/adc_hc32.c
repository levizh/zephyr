/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"
#include <soc.h>
#include <clock_control/hc32_clock_control.h>
#include <interrupt_controller/intc_hc32.h>

#ifdef CONFIG_ADC_ASYNC
#include <drivers/dma/dma_hc32.h>
#include <dma.h>
#endif /* CONFIG_ADC_ASYNC */

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_hc32);


#ifdef CONFIG_ADC_ASYNC
struct adc_hc32_dma_cfg {
	uint8_t dma_unit;
	uint32_t dma_channel;
	uint8_t *buffer;
	size_t buffer_length;
	struct dma_hc32_config_user_data user_cfg;
	struct device *dma_dev;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg;
};
#endif /* CONFIG_ADC_ASYNC */

#define CHANNEL_TABLE_SIZE 4

struct adc_hc32_data {
	struct adc_context ctx;
	struct device *dev_adc;
	struct device *dev_clock;
	uint32_t channels_map;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t resolution;
	uint8_t channel_table[CHANNEL_TABLE_SIZE];
	uint8_t channel_count;
	uint8_t samples_count;
	int8_t acq_time_index;
	uint8_t sample_idx;
	bool is_async_read;
	bool is_finish_sample;
	bool is_do_repeat_sample;
#ifdef CONFIG_ADC_ASYNC
	volatile int dma_error;
	struct adc_hc32_dma_cfg dma_read;
#endif /* CONFIG_ADC_ASYNC */
};

struct adc_hc32_config {
	CM_ADC_TypeDef *base;
#if defined(CONFIG_ADC_HC32_INTERRUPT)
	void (*irq_cfg_func)(struct device *dev);
#endif /* CONFIG_ADC_HC32_INTERRUPT */
	struct hc32_modules_clock_sys clk_cfg;
#if defined(CONFIG_ADC_ASYNC)
	struct adc_channel_cfg ch_cfg;
#endif /* CONFIG_ADC_ASYNC */
	int8_t sequencer_type;
};

#ifdef CONFIG_ADC_ASYNC

void adc_hc32_dma_read_cb(void *user_data, uint32_t channel, int err_code)
{
	uint8_t ch_idx;
	struct dma_hc32_config_user_data *cfg = user_data;
	struct device *adc_dev = cfg->user_data;
	struct adc_hc32_data *data = adc_dev->driver_data;
	const struct adc_hc32_config *config = adc_dev->config->config_info;
	CM_ADC_TypeDef *ADCx = config->base;

	dma_stop(data->dma_read.dma_dev, channel);
	ADC_Stop(ADCx);
	/* Disable all channels */
	for (ch_idx = 0; ch_idx < data->channel_count; ch_idx++) {
		ADC_ChCmd(ADCx, ADC_SEQ_A, (data->channel_table)[ch_idx], DISABLE);
	}

	if (err_code < 0) {
		adc_context_release(&data->ctx, err_code);
		return;
	}

	adc_context_complete(&data->ctx, err_code);
}

static int adc_hc32_async_setup(struct device *dev)
{
	struct device *dma_dev;
	struct adc_hc32_data *data = dev->driver_data;
	struct adc_hc32_dma_cfg *config;

	config = &data->dma_read;
	dma_dev = device_get_binding(dma_hc32_get_device_name(config->dma_unit));
	config->dma_dev = dma_dev;
	if (config->dma_dev == NULL) {
		LOG_ERR("dma device error!");
		return -ENODEV;
	}

	config->user_cfg.user_data = (void *)dev;
	config->dma_cfg.head_block = &config->blk_cfg;
	config->dma_cfg.callback_arg = (void *)&config->user_cfg;
	config->dma_cfg.dma_callback = adc_hc32_dma_read_cb;

	return 0;
}

static int adc_hc32_dma_start(struct device *dev,
							struct adc_sequence_options *options)
{
	int ret = 0;
	const struct adc_hc32_config *config = dev->config->config_info;
	CM_ADC_TypeDef *ADCx = config->base;
	struct adc_hc32_data *data = dev->driver_data;
	struct dma_block_config *blk_cfg = &data->dma_read.blk_cfg;

	blk_cfg->dest_address = (uint32_t)data->buffer;
	blk_cfg->source_address = (uint32_t)(&ADCx->DR0) +
								 (data->channel_table)[0U] * 2U;
	blk_cfg->block_size = data->samples_count * sizeof(uint16_t);
	blk_cfg->source_reload_en  = 0U;
	blk_cfg->dest_reload_en = 0U;
	blk_cfg->source_addr_adj = 0x02U; /* fixed address */
	blk_cfg->dest_addr_adj = 0x00U; /* increment */

	ret = dma_config(data->dma_read.dma_dev, data->dma_read.dma_channel,
					 &data->dma_read.dma_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to re-config dma: %d", ret);
		return ret;
	}

	data->dma_error = 0;
	ret = dma_start(data->dma_read.dma_dev, data->dma_read.dma_channel);
	if (ret != 0) {
		LOG_ERR("Failed to start dma: %d", ret);
		return ret;
	}

	return ret;
}
#endif /* CONFIG_ADC_ASYNC */

static void adc_hc32_start_conversion(struct device *dev)
{
	uint8_t idx;
	struct adc_hc32_data *data = dev->driver_data;
	const struct adc_hc32_config *config = dev->config->config_info;
	CM_ADC_TypeDef *ADCx = config->base;

#ifdef CONFIG_ADC_ASYNC
	if (false == data->is_async_read) {
		ADC_IntCmd(ADCx, ADC_INT_EOCA, ENABLE);
	} else {
		ADC_IntCmd(ADCx, ADC_INT_EOCA, DISABLE);
	}
#else
	/* Enable interrupt only */
	ADC_IntCmd(ADCx, ADC_INT_EOCA, ENABLE);
#endif /* CONFIG_ADC_ASYNC */

	/* Enable channels */
	for (idx = 0; idx < data->channel_count; idx++) {
		ADC_ChCmd(ADCx, ADC_SEQ_A, (data->channel_table)[idx], ENABLE);
	}
	ADC_Start(ADCx);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_hc32_data *data = CONTAINER_OF(ctx, struct adc_hc32_data, ctx);

	data->repeat_buffer = data->buffer;

#ifdef CONFIG_ADC_ASYNC
	if (true == data->is_async_read) {
		adc_hc32_dma_start(data->dev_adc, &ctx->options);
	}
#endif /* CONFIG_ADC_ASYNC */
	adc_hc32_start_conversion(data->dev_adc);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct adc_hc32_data *data = CONTAINER_OF(ctx, struct adc_hc32_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
		data->is_do_repeat_sample = true;
	}
}

/* Implementation of the ADC driver API function: adc_channel_setup. */
static int adc_hc32_channel_setup(struct device *dev,
					const struct adc_channel_cfg *channel_cfg)
{
	if (channel_cfg->acquisition_time < 0)
	{
		LOG_ERR("Error adc acquisition time");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Invalid channel gain");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Invalid channel reference");
		return -EINVAL;
	}

	return 0;
}

static int start_read(struct device *dev, const struct adc_sequence *sequence)
{
	int err, max_chs, idx, channel_cout = 0;
	const struct adc_hc32_config *config = dev->config->config_info;
	struct adc_hc32_data *data = dev->driver_data;
	CM_ADC_TypeDef *ADCx = config->base;
	stc_adc_init_t stc_adc_init_struct;

	(void)ADC_StructInit(&stc_adc_init_struct);

	data->buffer = sequence->buffer;
	data->channels_map = sequence->channels;
	data->is_finish_sample = false;
	data->samples_count = 1U;
	if (sequence->options != NULL) {
		data->samples_count += sequence->options->extra_samplings;
	}
	data->sample_idx = 0U;

	if (CM_ADC1 == ADCx) {
		max_chs = ADC_CH16;
	} else {
		max_chs = ADC_CH7;
	}

	for (idx = 0; idx < max_chs; idx++) {
		if (((data->channels_map >> idx) & 0x1U) == 0x1U) {
			(data->channel_table)[channel_cout] = idx;
			channel_cout++;
		}
	}
	data->channel_count = channel_cout;

	if (data->channel_count > max_chs) {
		LOG_ERR("channel count error");
		return -EINVAL;
	}
	/*
	check if channels valid with channel mask ((0x1U << (max_chs + 1)) - 0x1U).
	for adc0 total 17 channel, channel mask is 0x0001_FFFF,
	for adc1 total 8 channel, channel mask is 0x0000_00FF
	*/
	if (!(sequence->channels & ((0x1U << (max_chs + 1)) - 0x1U))) {
		LOG_ERR("Error channel ids");
		return -EINVAL;
	}

	if (data->samples_count > 1U) {
		stc_adc_init_struct.u16ScanMode = ADC_MD_SEQA_CONT;
	} else {
		stc_adc_init_struct.u16ScanMode = ADC_MD_SEQA_SINGLESHOT;
	}

	if (sequence->oversampling >= 9U) {
		LOG_ERR("Unsupported over sampling count");
		return -EINVAL;
	} else if (sequence->oversampling > 0U &&  sequence->oversampling < 9U) {
		ADC_ConvDataAverageConfig(ADCx,
				(sequence->oversampling - 1) << ADC_CR0_AVCNT_POS);
		for (idx = 0; idx < data->channel_count; idx++) {
			ADC_ConvDataAverageChCmd(ADCx, (data->channel_table)[idx], ENABLE);
		}
	} else {
		for (idx = 0; idx < data->channel_count; idx++) {
			ADC_ConvDataAverageChCmd(ADCx, (data->channel_table)[idx], DISABLE);
		}
	}

	if (8U == sequence->resolution) {
		stc_adc_init_struct.u16Resolution = ADC_RESOLUTION_8BIT;
	} else if (10U == sequence->resolution) {
		stc_adc_init_struct.u16Resolution = ADC_RESOLUTION_10BIT;
	} else if (12U == sequence->resolution) {
		stc_adc_init_struct.u16Resolution = ADC_RESOLUTION_12BIT;
	} else {
		LOG_ERR("Error resolution");
		return -EINVAL;
	}

	ADC_Init(ADCx, &stc_adc_init_struct);

	adc_context_start_read(&data->ctx, sequence);
	err = adc_context_wait_for_completion(&data->ctx);

	return err;
}

/* Implementation of the ADC driver API function: adc_read. */
static int adc_hc32_read(struct device *dev,
			 const struct adc_sequence *sequence)
{
	struct adc_hc32_data *data = dev->driver_data;
	int err;

	data->is_async_read = false;
	adc_context_lock(&data->ctx, false, NULL);
	err = start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

#ifdef CONFIG_ADC_ASYNC
/* Implementation of the ADC driver API function: adc_read_sync. */
static int adc_hc32_read_async(struct device *dev,
 					const struct adc_sequence *sequence,
 					struct k_poll_signal *async)
{
	struct adc_hc32_data *data = dev->driver_data;
	int error;

	data->is_async_read = true;
	adc_context_lock(&data->ctx, true, async);
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

static int adc_hc32_init(struct device *dev)
{
	int err;
	struct adc_hc32_data *data = dev->driver_data;
	const struct adc_hc32_config *config = dev->config->config_info;

	/* Init adc pointer address in data structure */
	data->dev_adc = dev;
	/* Enable adc clock */
	data->dev_clock = device_get_binding(CLOCK_CONTROL);
	err = clock_control_on(data->dev_clock,
				(clock_control_subsys_t)&config->clk_cfg);
	if (err < 0) {
		return err;
	}

	/* Register isr */
#if defined(CONFIG_ADC_HC32_INTERRUPT)
	config->irq_cfg_func(dev);
#endif /* CONFIG_ADC_HC32_INTERRUPT */

#ifdef CONFIG_ADC_ASYNC
	if (adc_hc32_async_setup(dev) < 0) {
		LOG_ERR("Failed to config dma");
		return -ENODEV;
	}
#endif /* CONFIG_ADC_ASYNC */

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api adc_hc32_driver_api = {
	.channel_setup = adc_hc32_channel_setup,
	.read          = adc_hc32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async    = adc_hc32_read_async,
#endif
};

#if defined(CONFIG_ADC_HC32_INTERRUPT)

void adc_hc32_seqa_seqb_isr(struct device *dev, uint8_t u8Flag)
{
	uint8_t ch_idx;
	struct adc_hc32_data *data = dev->driver_data;
	const struct adc_hc32_config *config = dev->config->config_info;
	CM_ADC_TypeDef *ADCx = config->base;
	struct adc_context *ctx = &data->ctx;

	/* To end last isr entering when turned off adc */
	if (data->is_finish_sample == true) {
		return;
	}

	if (ADC_GetStatus(ADCx, u8Flag) == SET) {
		ADC_ClearStatus(ADCx, u8Flag);
		for (ch_idx = 0; ch_idx < data->channel_count;) {
			*data->buffer++ =
				ADC_GetValue(ADCx, (data->channel_table)[ch_idx++]);
		}
	}
	/* Turn off repeat sample unless is set in callback by user */
	data->is_do_repeat_sample = false;
	adc_context_on_sampling_done(ctx, dev);

	if ((++data->sample_idx >= data->samples_count) &&
		(data->is_do_repeat_sample == false)) {
		ADC_Stop(ADCx);
		/* Disable all channels */
		for (ch_idx = 0; ch_idx < data->channel_count; ch_idx++) {
			ADC_ChCmd(ADCx, ADC_SEQ_A, (data->channel_table)[ch_idx], DISABLE);
		}
		ADC_ClearStatus(ADCx, ADC_FLAG_ALL);
		ADC_IntCmd(ADCx, ADC_INT_ALL, DISABLE);
		data->is_finish_sample = true;
	}
}

#define ADC_HC32_IRQ_HANDLER_DECL(index)									\
	static void adc_hc32_eoca_##index(struct device *dev);					\
	static void	adc_hc32_eocb_##index(struct device *dev);					\
	static void adc_hc32_cfg_func_##index(struct device *dev);

#define ADC_HC32_INTC_CONFIG(isr_name_prefix, isr_idx, index)				\
	IRQ_CONNECT(DT_XHSC_HC32_ADC_##index##_IRQ_##isr_idx,					\
		DT_XHSC_HC32_ADC_##index##_IRQ_##isr_idx##_PRIORITY,				\
		isr_name_prefix##_##index,											\
		DEVICE_GET(adc_hc32_##index),										\
		0);																	\
	hc32_intc_irq_signin(													\
		DT_XHSC_HC32_ADC_##index##_IRQ_##isr_idx,							\
		DT_XHSC_HC32_ADC_##index##_IRQ_##isr_idx##_INT_SRC);				\
	irq_enable(DT_XHSC_HC32_ADC_##index##_IRQ_##isr_idx);

#define ADC_HC32_IRQ_HANDLER_DEF(index)										\
	static void adc_hc32_eoca_##index(struct device *dev)					\
	{																		\
		adc_hc32_seqa_seqb_isr(dev, ADC_FLAG_EOCA);							\
	}																		\
	static void	adc_hc32_eocb_##index(struct device *dev)					\
	{																		\
		adc_hc32_seqa_seqb_isr(dev, ADC_FLAG_EOCB);							\
	}																		\
	static void adc_hc32_cfg_func_##index(struct device *dev)				\
	{																		\
		ADC_HC32_INTC_CONFIG(adc_hc32_eoca, 0, index)						\
		ADC_HC32_INTC_CONFIG(adc_hc32_eocb, 1, index)						\
	}

#define ADC_HC32_ISR_CFG_FUNC(index)										\
	.irq_cfg_func = adc_hc32_cfg_func_##index,
#endif /* CONFIG_ADC_HC32_INTERRUPT */

#ifdef CONFIG_ADC_ASYNC

#ifdef DT_XHSC_HC32_ADC_0
#ifdef DT_XHSC_HC32_ADC_0_DMA_READ
#define DMA_READ_FLAG_0	1
#else /* DT_XHSC_HC32_ADC_0_DMA_READ */
#define DMA_READ_FLAG_0	0
#endif /* DT_XHSC_HC32_ADC_0_DMA_READ */
#endif /* DT_XHSC_HC32_ADC_0 */

#ifdef DT_XHSC_HC32_ADC_1
#ifdef DT_XHSC_HC32_ADC_1_DMA_READ
#define DMA_READ_FLAG_1	1
#else /* DT_XHSC_HC32_ADC_1_DMA_READ */
#define DMA_READ_FLAG_1	0
#endif /* DT_XHSC_HC32_ADC_1_DMA_READ */
#endif /* DT_XHSC_HC32_ADC_1 */

#define UART_HC32_DMA_CFG(dir, cfg_val)										\
	.dma_read = {															\
		.dma_unit = HC32_DT_GET_DMA_UNIT(cfg_val),							\
		.dma_channel = (uint8_t)HC32_DT_GET_DMA_CH(cfg_val), 				\
		.user_cfg = {														\
			.slot = HC32_DT_GET_DMA_SLOT(cfg_val), 							\
		},																	\
		.dma_cfg = {														\
			.channel_direction = PERIPHERAL_TO_MEMORY,						\
			.source_burst_length = 1,  /* SINGLE transfer */				\
			.source_data_size = 2, 											\
			.dest_burst_length = 1,	/* SINGLE transfer */					\
			.dest_data_size = 2,											\
			.block_count = 1,												\
		},																	\
	},

#define UART_HC32_DMA_CFG_PRE(index)										\
	COND_CODE_1(DMA_READ_FLAG_##index, 										\
		(UART_HC32_DMA_CFG(rx, DT_XHSC_HC32_ADC_##index##_DMA_READ)), ())
#else /* CONFIG_ADC_ASYNC */
#define UART_HC32_DMA_CFG_PRE(index)
#endif /* CONFIG_ADC_ASYNC */

#define HC32_ADC_INIT(index)												\
	DEVICE_DECLARE(adc_hc32_##index);										\
	ADC_HC32_IRQ_HANDLER_DECL(index)										\
	static const struct adc_hc32_config adc_hc32_cfg_##index = {			\
		.base = (CM_ADC_TypeDef *)DT_XHSC_HC32_ADC_##index##_BASE_ADDRESS,	\
		.clk_cfg = {														\
			.bus = DT_XHSC_HC32_ADC_##index##_CLOCK_BUS,					\
			.fcg = DT_XHSC_HC32_ADC_##index##_CLOCK_FCG,					\
			.bits = DT_XHSC_HC32_ADC_##index##_CLOCK_BITS,					\
		},																	\
		ADC_HC32_ISR_CFG_FUNC(index)										\
	};																		\
	static struct adc_hc32_data adc_hc32_data_##index = {					\
		ADC_CONTEXT_INIT_TIMER(adc_hc32_data_##index, ctx),					\
		ADC_CONTEXT_INIT_LOCK(adc_hc32_data_##index, ctx),					\
		ADC_CONTEXT_INIT_SYNC(adc_hc32_data_##index, ctx),					\
		UART_HC32_DMA_CFG_PRE(index)										\
	};																		\
	DEVICE_AND_API_INIT(adc_hc32_##index, 									\
		DT_XHSC_HC32_ADC_##index##_LABEL,									\
		&adc_hc32_init,														\
		&adc_hc32_data_##index, &adc_hc32_cfg_##index,						\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,					\
		&adc_hc32_driver_api);												\
	ADC_HC32_IRQ_HANDLER_DEF(index)

#ifdef DT_XHSC_HC32_ADC_0
HC32_ADC_INIT(0)
#endif

#ifdef DT_XHSC_HC32_ADC_1
HC32_ADC_INIT(1)
#endif