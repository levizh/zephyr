#define DT_DRV_COMPAT xhsc_hc32_adc

#include <errno.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <soc.h>
#include <hc32_ll_adc.h>

#ifdef CONFIG_ADC_ASYNC
#include <zephyr/drivers/dma/dma_hc32.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/toolchain.h>
#include <hc32_ll_dma.h>
#endif /* CONFIG_ADC_ASYNC */

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_hc32);

#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>
#include <zephyr/mem_mgmt/mem_attr.h>

#define ADC_MAX_CHANNEL		(17U)
/* Macros to count channels */
#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
const struct adc_dt_spec channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};
#define CHANNEL_TABLE_SIZE (ARRAY_SIZE(channels))
#else
#define CHANNEL_TABLE_SIZE	(ADC_MAX_CHANNEL)
#endif /* DT_NODE_HAS_PROP */

#ifdef CONFIG_ADC_ASYNC
struct adc_hc32_dma_cfg {
	const struct device *dev_dma;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint16_t src_addr_increment;
	uint16_t dst_addr_increment;
	struct dma_block_config blk_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	size_t offset;
	volatile size_t counter;
	int32_t timeout;
	struct k_work_delayable timeout_work;
	bool enabled;
	struct dma_hc32_config_user_data user_cfg;
};
#endif /* CONFIG_ADC_ASYNC */

struct adc_hc32_data {
	struct adc_context ctx;
	const struct device *dev_adc;
	const struct device *dev_clock;
	uint32_t channels_map;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t resolution;
	uint8_t channel_table[CHANNEL_TABLE_SIZE];
	const uint8_t *sample_time;
	uint8_t channel_count;
	uint8_t samples_count;
	int8_t acq_time_index;
	bool is_async_read;
	bool is_finish_sample;
#ifdef CONFIG_ADC_ASYNC
	volatile int dma_error;
	struct adc_hc32_dma_cfg dma_read;
#endif /* CONFIG_ADC_ASYNC */
};

struct adc_hc32_config {
	CM_ADC_TypeDef *base;
#if defined(CONFIG_ADC_HC32_INTERRUPT)
	void (*irq_cfg_func)(const struct device *dev);
#endif /* CONFIG_ADC_HC32_INTERRUPT */
	const struct pinctrl_dev_config *pin_cfg;
	const struct hc32_modules_clock_sys *clk_cfg;
	const struct adc_channel_cfg *ch_cfg;
	uint32_t clk_prescaler;
	int8_t num_sampling_time_common_channels;
	int8_t sequencer_type;
};

#ifdef CONFIG_ADC_ASYNC
static int adc_hc32_async_init(const struct device *dev)
{
	int ret;
	struct adc_hc32_data *data = dev->data;

	/* Configure dma read adc */
	memset(&data->dma_read.blk_cfg, 0U, sizeof(data->dma_read.blk_cfg));

	data->dma_read.blk_cfg.source_address = 0U;
	data->dma_read.blk_cfg.source_addr_adj = data->dma_read.src_addr_increment;
	data->dma_read.blk_cfg.dest_address = 0U;
	data->dma_read.blk_cfg.dest_addr_adj = data->dma_read.dst_addr_increment;
	data->dma_read.blk_cfg.block_size = 0U;
	data->dma_read.blk_cfg.source_reload_en  = 0U;
	data->dma_read.blk_cfg.dest_reload_en = 0U;

	data->dma_read.dma_cfg.head_block = &data->dma_read.blk_cfg;
	data->dma_read.user_cfg.user_data = (void *)dev;
	data->dma_read.dma_cfg.user_data = (void *)&data->dma_read.user_cfg;

	ret = dma_config(data->dma_read.dev_dma, data->dma_read.dma_channel,
			&data->dma_read.dma_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to config dma: %d", ret);
	}

	return ret;
}

static int adc_hc32_dma_start(const struct device *dev,
				struct adc_sequence_options *options)
{
	int ret = 0;

	const struct adc_hc32_config *config = dev->config;
	CM_ADC_TypeDef *ADCx = (CM_ADC_TypeDef *)config->base;
	struct adc_hc32_data *data = dev->data;

	uint32_t dma_size, dma_addr_dst, dma_addr_src;
	const struct device *dev_dma = data->dma_read.dev_dma;
	const uint32_t dma_ch = data->dma_read.dma_channel;

	dma_addr_dst = (uint32_t)data->buffer;
	dma_addr_src = (uint32_t)(&ADCx->DR0) + data->channel_table[0] * 2;
	dma_size = CHANNEL_TABLE_SIZE * sizeof(uint16_t);

	ret = dma_reload(dev_dma, dma_ch, dma_addr_src, dma_addr_dst, dma_size);
	if (ret != 0) {
		LOG_ERR("Failed to re-config dma: %d", ret);
		return ret;
	}

	data->dma_error = 0;
	ret = dma_start(dev_dma, dma_ch);
	if (ret != 0) {
		LOG_ERR("Failed to start dma: %d", ret);
		return ret;
	}

	return ret;
}
#endif /* CONFIG_ADC_ASYNC */

static void adc_hc32_start_conversion(const struct device *dev)
{
	uint8_t idx;
	struct adc_hc32_data *data = dev->data;
	const struct adc_hc32_config *config = data->dev_adc->config;
	CM_ADC_TypeDef *ADCx = (CM_ADC_TypeDef *)config->base;

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
		ADC_ChCmd(ADCx, ADC_SEQ_A, data->channel_table[idx], ENABLE);
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
	}
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_hc32_data *data = CONTAINER_OF(ctx, struct adc_hc32_data, ctx);
	const struct adc_hc32_config *config = data->dev_adc->config;
	CM_ADC_TypeDef *ADCx = (CM_ADC_TypeDef *)config->base;

	data->is_finish_sample = true;

	ADC_Stop(ADCx);
	ADC_ClearStatus(ADCx, ADC_FLAG_ALL);
	ADC_IntCmd(ADCx, ADC_INT_ALL, DISABLE);
}

static int adc_hc32_channel_setup(const struct device *dev,
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

static int start_read(const struct device *dev,
		      const struct adc_sequence *sequence)
{
	int err, max_chs, idx;
	uint8_t *p_temp;
	const struct adc_hc32_config *config = dev->config;
	struct adc_hc32_data *data = dev->data;
	CM_ADC_TypeDef *ADCx = (CM_ADC_TypeDef *)config->base;
	stc_adc_init_t stc_adc_init_struct;

	(void)ADC_StructInit(&stc_adc_init_struct);

	data->buffer = sequence->buffer;
	data->channels_map = sequence->channels;
	data->channel_count = POPCOUNT(data->channels_map);
	data->is_finish_sample = false;
	data->samples_count = 1U;
	if (sequence->options != NULL) {
		data->samples_count += sequence->options->extra_samplings;
	}

	if (CM_ADC1 == ADCx) {
		max_chs = ADC_CH16;
	} else {
		max_chs = ADC_CH7;
	}

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

	p_temp = data->channel_table;
	for (idx = 0; idx < max_chs; idx++) {
		if (((data->channels_map >> idx) & 0x1U) == 0x1U) {
			*p_temp++ = idx;
		}
		if ((uint32_t)p_temp - (uint32_t)data->channel_table
				>= data->channel_count) {
			break;
		}
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
			ADC_ConvDataAverageChCmd(ADCx, data->channel_table[idx], ENABLE);
		}
	} else {
		for (idx = 0; idx < data->channel_count; idx++) {
			ADC_ConvDataAverageChCmd(ADCx, data->channel_table[idx], DISABLE);
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

	for (idx = 0; idx < data->channel_count; idx++) {
		if (data->sample_time[idx] >= 0x0B) {
			ADC_SetSampleTime(ADCx, data->channel_table[idx],
				data->sample_time[idx]);
		} else {
			ADC_SetSampleTime(ADCx, data->channel_table[idx], 0x40U);
		}
	}
	ADC_Init(ADCx, &stc_adc_init_struct);

	adc_context_start_read(&data->ctx, sequence);
	err = adc_context_wait_for_completion(&data->ctx);

	return err;
}

static int adc_hc32_read(const struct device *dev,
			    const struct adc_sequence *sequence)
{
	struct adc_hc32_data *data = dev->data;
	int err;

	data->is_async_read = false;
	adc_context_lock(&data->ctx, false, NULL);
	err = start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

#if defined(CONFIG_ADC_ASYNC)
static int adc_hc32_read_async(const struct device *dev,
				  const struct adc_sequence *sequence,
				  struct k_poll_signal *async)
{
	struct adc_hc32_data *data = dev->data;
	int error;

	data->is_async_read = true;
	adc_context_lock(&data->ctx, true, async);
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

static int adc_hc32_clock_enable(const struct device *dev)
{
	int err = 0;
	const struct adc_hc32_config *config = dev->config;
	struct adc_hc32_data *data = dev->data;
	const struct device *const clk = DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE);

	/* Get adc dev_clock node */
	data->dev_clock = clk;

	if (!device_is_ready(data->dev_clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}
	/* Enable adc clock */
	err = clock_control_on(data->dev_clock,
				(clock_control_subsys_t)config->clk_cfg);
	if (err != 0) {
		LOG_ERR("Could not enable adc clock");
		return err;
	}
	return 0;
}

static int adc_hc32_init(const struct device *dev)
{
	int err = 0;
	const struct adc_hc32_config *config = dev->config;
	struct adc_hc32_data *data = dev->data;

	/* Init adc pointer address in data structure */
	data->dev_adc = dev;
	/* Enable adc clock */
	err = adc_hc32_clock_enable(dev);
	if (err < 0) {
		LOG_ERR("Failed to enable adc clock");
		return err;
	}
	/* Config adc pin */
	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("Failed to config adc pin");
		return err;
	}

#ifdef CONFIG_ADC_ASYNC
	if ((data->dma_read.dev_dma == NULL) ||
	    !device_is_ready(data->dma_read.dev_dma)) {
		LOG_ERR("%s device not ready", data->dma_read.dev_dma->name);
		return -ENODEV;
	}

	err = adc_hc32_async_init(dev);
	if (err < 0)
	{
		LOG_ERR("Failed to config dma");
		return -ENODEV;
	}
#endif /* CONFIG_ADC_ASYNC */

#if defined(CONFIG_ADC_HC32_INTERRUPT)
	config->irq_cfg_func(dev);
#endif /* CONFIG_ADC_HC32_INTERRUPT */
	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api adc_hc32_driver_api = {
	.channel_setup = adc_hc32_channel_setup,
	.read = adc_hc32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_hc32_read_async,
#endif /* CONFIG_ADC_ASYNC */
	.ref_internal = DT_INST_PROP(0, vref_mv),
};

#if defined(CONFIG_ADC_HC32_INTERRUPT)

void adc_hc32_seqa_seqb_isr(const struct device *dev, uint8_t u8Flag)
{
	uint8_t channel_id_idx;
	struct adc_hc32_data *data = dev->data;
	const struct adc_hc32_config *config = dev->config;
	CM_ADC_TypeDef *ADCx = (CM_ADC_TypeDef *)config->base;

	if (data->is_finish_sample == true) {
		return;
	}

	if (ADC_GetStatus(ADCx, u8Flag) == SET) {
		ADC_ClearStatus(ADCx, u8Flag);
		for (channel_id_idx = 0; channel_id_idx < data->channel_count;) {
			*data->buffer++ =
				ADC_GetValue(ADCx, data->channel_table[channel_id_idx++]);
		}
	}

	adc_context_on_sampling_done(&data->ctx, dev);
}

#define ADC_HC32_IRQ_HANDLER_DECL(index)									\
	static void adc_hc32_eoca_##index(const struct device *dev);			\
	static void	adc_hc32_eocb_##index(const struct device *dev);			\
	static void adc_hc32_cfg_func_##index(const struct device *dev);

#define ADC_HC32_INTC_CONFIG(isr_name_prefix, isr_idx, index)				\
	IRQ_CONNECT(															\
		DT_INST_IRQ_BY_IDX(index, isr_idx, irq),							\
		DT_INST_IRQ_BY_IDX(index, isr_idx, priority),						\
		DT_CAT3(isr_name_prefix, _, index),									\
		DEVICE_DT_INST_GET(index), 0);										\
	hc32_intc_irq_signin(													\
		DT_INST_IRQ_BY_IDX(index, isr_idx, irq),							\
		DT_INST_IRQ_BY_IDX(index, isr_idx, int_src));						\
	irq_enable(DT_INST_IRQ_BY_IDX(index, isr_idx, irq));

#define ADC_HC32_IRQ_HANDLER_DEF(index)										\
	static void adc_hc32_eoca_##index(const struct device *dev)				\
	{																		\
		adc_hc32_seqa_seqb_isr(dev, ADC_FLAG_EOCA);							\
	}																		\
	static void	adc_hc32_eocb_##index(const struct device *dev)				\
	{																		\
		adc_hc32_seqa_seqb_isr(dev, ADC_FLAG_EOCB);							\
	}																		\
	static void adc_hc32_cfg_func_##index(const struct device *dev)			\
	{																		\
		ADC_HC32_INTC_CONFIG(adc_hc32_eoca, 0, index)						\
		ADC_HC32_INTC_CONFIG(adc_hc32_eocb, 1, index)						\
	}

#define ADC_HC32_IRQ_CFG_FUNC(index)										\
	.irq_cfg_func = adc_hc32_cfg_func_##index,
#else
#define ADC_HC32_IRQ_HANDLER_DECL(index)
#define ADC_HC32_IRQ_HANDLER_DEF(index)
#define ADC_HC32_IRQ_CFG_FUNC(index)
#endif /* CONFIG_ADC_HC32_INTERRUPT */

#define ADC_HC32_CLOCK_DECL(index)											\
	static const struct hc32_modules_clock_sys adc_fcg_config_##index[]		\
		= HC32_MODULES_CLOCKS(DT_DRV_INST(index));

#define ADC_SAMPLE_TIME_DEF(index)											\
	static const uint8_t sample_time_##index[] = 							\
		DT_PROP(DT_DRV_INST(index), sample_time);

#if defined(CONFIG_ADC_ASYNC)
void adc_hc32_dma_read_cb(const struct device *dev_dma, void *user_data,
				   uint32_t channel, int status)
{
	struct dma_hc32_config_user_data *cfg = user_data;
	struct device *adc_dev = cfg->user_data;
	struct adc_hc32_data *data = adc_dev->data;

	const struct adc_hc32_config *config = adc_dev->config;
	CM_ADC_TypeDef *ADCx = (CM_ADC_TypeDef *)config->base;

	dma_stop(dev_dma, channel);
	ADC_Stop(ADCx);

	if (status < 0) {
		adc_context_release(&data->ctx, status);
		return;
	}

	adc_context_complete(&data->ctx, status);
}

#define ADC_DMA_CHANNEL_INIT(index, action, src, dest)						\
	.dev_dma = DEVICE_DT_GET(HC32_DMA_CTLR(index, action)),					\
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, action, channel),		\
	.dma_cfg = {															\
		.channel_direction = HC32_DMA_CONFIG_DIRECTION(						\
				HC32_DMA_CHANNEL_CONFIG(index, action)),					\
		.source_data_size = HC32_DMA_CONFIG_DATA_SIZE(						\
				HC32_DMA_CHANNEL_CONFIG(index, action)),					\
		.dest_data_size = HC32_DMA_CONFIG_DATA_SIZE(						\
				HC32_DMA_CHANNEL_CONFIG(index, action)),					\
		.source_burst_length = 1, /* SINGLE transfer */						\
		.dest_burst_length = 1,												\
		.block_count = 1,													\
		.dma_callback = adc_hc32_dma_##action##_cb,							\
	},																		\
	.user_cfg = {															\
		.slot = HC32_DMA_SLOT(index, action),								\
	},																		\
	.src_addr_increment = HC32_DMA_CONFIG_##src##_ADDR_INC(					\
			HC32_DMA_CHANNEL_CONFIG(index, action)),						\
	.dst_addr_increment = HC32_DMA_CONFIG_##dest##_ADDR_INC(				\
			HC32_DMA_CHANNEL_CONFIG(index, action)),

#define ADC_DMA_CHANNEL(index, action, src, dest)							\
	.dma_##action = {														\
		COND_CODE_1(														\
			DT_INST_DMAS_HAS_NAME(index, action),							\
			(ADC_DMA_CHANNEL_INIT(index, action, src, dest)),				\
			(NULL))															\
	},
#else
#define ADC_DMA_CHANNEL(index, action, src, dest)
#endif /* CONFIG_ADC_ASYNC */

#define ADC_HC32_INIT(index)												\
	ADC_HC32_IRQ_HANDLER_DECL(index)										\
	ADC_HC32_CLOCK_DECL(index)												\
	ADC_SAMPLE_TIME_DEF(index)												\
	PINCTRL_DT_INST_DEFINE(index);											\
	static const struct adc_hc32_config adc_hc32_cfg_##index = {			\
		.base = (CM_ADC_TypeDef *)DT_INST_REG_ADDR(index),					\
		.clk_cfg = adc_fcg_config_##index,									\
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),					\
		ADC_HC32_IRQ_CFG_FUNC(index)										\
	};																		\
	static struct adc_hc32_data adc_hc32_data_##index = {					\
		.sample_time = sample_time_##index,									\
		ADC_CONTEXT_INIT_TIMER(adc_hc32_data_##index, ctx),					\
		ADC_CONTEXT_INIT_LOCK(adc_hc32_data_##index, ctx),					\
		ADC_CONTEXT_INIT_SYNC(adc_hc32_data_##index, ctx),					\
		ADC_DMA_CHANNEL(index, read, SOURCE, DEST)							\
	};																		\
	ADC_HC32_IRQ_HANDLER_DEF(index)											\
	DEVICE_DT_INST_DEFINE(index,											\
				&adc_hc32_init, NULL,										\
				&adc_hc32_data_##index, &adc_hc32_cfg_##index,				\
				PRE_KERNEL_1, CONFIG_ADC_INIT_PRIORITY,						\
				&adc_hc32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_HC32_INIT)
