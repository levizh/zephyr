/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_timer6_counter

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/dt-bindings/timer/hc32-timer.h>

#include "hc32_ll.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_tmr6_hc32, CONFIG_COUNTER_LOG_LEVEL);

#define TIMER6_INT_CH(ch)           (TMR6_INT_MATCH_A << ch)
#define TIMER6_FLAG_CH(ch)          (TMR6_FLAG_MATCH_A << ch)
#define TIMER6_CLK_DIV(div)         (div << TMR6_GCONR_CKDIV_POS)


struct counter_hc32_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	uint32_t guard_period;
	atomic_t cc_int_pending;
	uint32_t freq;
};

struct counter_hc32_ch_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

struct counter_hc32_config {
	struct counter_config_info info;
	struct counter_hc32_ch_data *ch_data;
	CM_TMR6_TypeDef *timer;
	uint32_t clk_div;
	uint32_t cnt_mode;
	uint32_t cnt_dir;
	void (*fcg_configure)(void);
	void (*irq_configure)(void);
	uint32_t *irqn;

	LOG_INSTANCE_PTR_DECLARE(log);
};

static uint32_t tmr6_get_unit_syn_bit(CM_TMR6_TypeDef *TMR6x)
{
	uint32_t syn_bit = 0UL;
	if (CM_TMR6_1 == TMR6x) {
		syn_bit = TMR6CR_SCLRR_SCLE1;
	} else if (CM_TMR6_2 == TMR6x) {
		syn_bit = TMR6CR_SCLRR_SCLE2;
	} else if (CM_TMR6_3 == TMR6x) {
		syn_bit = TMR6CR_SCLRR_SCLE3;
	}
	return syn_bit;
}

static void tmr6_clear_counter(CM_TMR6_TypeDef *TMR6x)
{
	TMR6_SWSyncClear(tmr6_get_unit_syn_bit(TMR6x));
}

static int counter_hc32_start(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMR6_TypeDef *timer = config->timer;

	/* enable counter */
	TMR6_Start(timer);

	return 0;
}

static int counter_hc32_stop(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMR6_TypeDef *timer = config->timer;

	/* disable counter */
	TMR6_Stop(timer);

	return 0;
}

static uint32_t counter_hc32_get_top_value(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;

	return TMR6_GetPeriodValue(config->timer, TMR6_PERIOD_REG_A);
}

static uint32_t counter_hc32_read(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;

	return TMR6_GetCountValue(config->timer);
}

static int counter_hc32_get_value(const struct device *dev, uint32_t *ticks)
{
	*ticks = counter_hc32_read(dev);
	return 0;
}

static uint32_t counter_hc32_ticks_add(uint32_t val1, uint32_t val2,
				       uint32_t top)
{
	uint32_t to_top;

	if (likely(IS_BIT_MASK(top))) {
		return (val1 + val2) & top;
	}

	to_top = top - val1;

	return (val2 <= to_top) ? val1 + val2 : val2 - to_top - 1U;
}

static uint32_t counter_hc32_ticks_sub(uint32_t val, uint32_t old, uint32_t top)
{
	if (likely(IS_BIT_MASK(top))) {
		return (val - old) & top;
	}

	/* if top is not 2^n-1 */
	return (val >= old) ? (val - old) : val + top + 1U - old;
}

static void counter_hc32_set_cc_int_pending(
	const struct device *dev, uint8_t chan)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;

	atomic_or(&data->cc_int_pending, BIT(chan));
	NVIC_SetPendingIRQ(config->irqn[chan]);
}

static int counter_hc32_set_cc(const struct device *dev, uint8_t chan,
			       const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;

	__ASSERT_NO_MSG(data->guard_period < counter_hc32_get_top_value(dev));
	uint32_t val = alarm_cfg->ticks;
	uint32_t flags = alarm_cfg->flags;
	bool absolute = flags & COUNTER_ALARM_CFG_ABSOLUTE;
	bool irq_on_late;
	CM_TMR6_TypeDef *timer = config->timer;
	uint32_t top = counter_hc32_get_top_value(dev);
	int err = 0;
	uint32_t prev_val;
	uint32_t now;
	uint32_t diff;
	uint32_t max_rel_val;

	__ASSERT(!READ_REG32_BIT(timer->ICONR, TIMER6_INT_CH(chan)),
		 "Expected that CC interrupt is disabled.");

	if (chan < 2U) {
		/* CHA/CHB make sure in cmp mode. */
		TMR6_SetFunc(timer, chan, TMR6_PIN_CMP_OUTPUT);
	}
	/* First take care of a risk of an event coming from CC being set to
	 * next tick. Reconfigure CC to future (now tick is the furthest
	 * future).
	 */
	now = counter_hc32_read(dev);
	prev_val = TMR6_GetCompareValue(timer, chan);
	TMR6_SetCompareValue(timer, chan, now);
	TMR6_ClearStatus(timer, TIMER6_FLAG_CH(chan));

	if (absolute) {
		max_rel_val = top - data->guard_period;
		irq_on_late = flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
	} else {
		/* If relative value is smaller than half of the counter range
		 * it is assumed that there is a risk of setting value too late
		 * and late detection algorithm must be applied. When late
		 * setting is detected, interrupt shall be triggered for
		 * immediate expiration of the timer. Detection is performed
		 * by limiting relative distance between CC and counter.
		 *
		 * Note that half of counter range is an arbitrary value.
		 */
		irq_on_late = val < (top / 2U);
		/* limit max to detect short relative being set too late. */
		max_rel_val = irq_on_late ? top / 2U : top;
		val = counter_hc32_ticks_add(now, val, top);
	}

	TMR6_SetCompareValue(timer, chan, val);

	/* Do not use val-1U here, for 16-bit timer when val=0, relative mode.
	 * Absolute: diff may be 0 and > max_rel_val
	 * Relative: diff will not be 0.
	 */
	diff = counter_hc32_ticks_sub(val, counter_hc32_read(dev), top);
	if ((diff > max_rel_val) || (diff == 0UL)) {
		if (absolute) {
			err = -ETIME;
		}

		/* Interrupt is triggered always for relative alarm and
		 * for absolute depending on the flag.
		 */
		if (irq_on_late) {
			counter_hc32_set_cc_int_pending(dev, chan);
		} else {
			config->ch_data[chan].callback = NULL;
		}
	} else {
		TMR6_IntCmd(timer, TIMER6_INT_CH(chan), ENABLE);
	}

	return err;
}

static int counter_hc32_set_alarm(const struct device *dev, uint8_t chan,
				  const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_ch_data *chdata = &config->ch_data[chan];

	if (alarm_cfg->ticks >  counter_hc32_get_top_value(dev)) {
		return -EINVAL;
	}

	if (chdata->callback) {
		return -EBUSY;
	}

	chdata->callback = alarm_cfg->callback;
	chdata->user_data = alarm_cfg->user_data;

	return counter_hc32_set_cc(dev, chan, alarm_cfg);
}

static int counter_hc32_cancel_alarm(const struct device *dev, uint8_t chan)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMR6_TypeDef *timer = config->timer;

	TMR6_IntCmd(timer, TIMER6_INT_CH(chan), DISABLE);
	config->ch_data[chan].callback = NULL;

	return 0;
}

static int counter_hc32_set_top_value(const struct device *dev,
				      const struct counter_top_cfg *cfg)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMR6_TypeDef *timer = config->timer;
	struct counter_hc32_data *data = dev->data;
	int err = 0;

	for (int i = 0; i < counter_get_num_of_channels(dev); i++) {
		/* Overflow can be changed only when all alarms are
		 * disabled.
		 */
		if (config->ch_data[i].callback) {
			return -EBUSY;
		}
	}

	TMR6_IntCmd(timer, TMR6_INT_OVF, DISABLE);
	TMR6_SetPeriodValue(timer, TMR6_PERIOD_REG_A, cfg->ticks);
	TMR6_ClearStatus(timer, TMR6_FLAG_OVF);

	data->top_cb = cfg->callback;
	data->top_user_data = cfg->user_data;

	if (!(cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
		tmr6_clear_counter(timer);
	} else if (counter_hc32_read(dev) >= cfg->ticks) {
		err = -ETIME;
		if (cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
			tmr6_clear_counter(timer);
		}
	}

	if (cfg->callback) {
		TMR6_IntCmd(timer, TMR6_INT_OVF, ENABLE);
	}

	return err;
}

static uint32_t counter_hc32_get_pending_int(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMR6_TypeDef *timer = config->timer;
	uint32_t pending = 0;

	if (SET == TMR6_GetStatus(timer, TMR6_FLAG_MATCH_A | TMR6_FLAG_MATCH_B |
				  TMR6_FLAG_MATCH_C | TMR6_FLAG_MATCH_D | TMR6_FLAG_MATCH_E |
				  TMR6_FLAG_MATCH_F)) {
		pending = 1;
	}

	return pending;
}

static uint32_t counter_hc32_get_guard_period(const struct device *dev,
					      uint32_t flags)
{
	struct counter_hc32_data *data = dev->data;

	ARG_UNUSED(flags);
	return data->guard_period;
}

static int counter_hc32_set_guard_period(const struct device *dev,
					 uint32_t guard,
					 uint32_t flags)
{
	struct counter_hc32_data *data = dev->data;

	ARG_UNUSED(flags);
	__ASSERT_NO_MSG(guard < counter_hc32_get_top_value(dev));

	data->guard_period = guard;
	return 0;
}

static uint32_t counter_hc32_get_freq(const struct device *dev)
{
	struct counter_hc32_data *data = dev->data;

	return data->freq;
}

static void counter_hc32_top_irq_handle(const struct device *dev)
{
	struct counter_hc32_data *data = dev->data;

	counter_top_callback_t cb = data->top_cb;

	__ASSERT(cb != NULL, "top event enabled - expecting callback");
	cb(dev, data->top_user_data);
}

static void counter_hc32_alarm_irq_handle(const struct device *dev,
					  uint32_t chan)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;
	CM_TMR6_TypeDef *timer = config->timer;
	uint32_t cc_val;

	struct counter_hc32_ch_data *chdata;
	counter_alarm_callback_t callback;

	atomic_and(&data->cc_int_pending, ~BIT(chan));
	TMR6_IntCmd(timer, TIMER6_INT_CH(chan), DISABLE);

	chdata = &config->ch_data[chan];
	callback = chdata->callback;
	chdata->callback = NULL;

	if (callback) {
		cc_val = TMR6_GetCompareValue(timer, chan);

		callback(dev, chan, cc_val, chdata->user_data);
	}
}

static const struct counter_driver_api counter_hc32_driver_api = {
	.start = counter_hc32_start,
	.stop = counter_hc32_stop,
	.get_value = counter_hc32_get_value,
	.set_alarm = counter_hc32_set_alarm,
	.cancel_alarm = counter_hc32_cancel_alarm,
	.set_top_value = counter_hc32_set_top_value,
	.get_pending_int = counter_hc32_get_pending_int,
	.get_top_value = counter_hc32_get_top_value,
	.get_guard_period = counter_hc32_get_guard_period,
	.set_guard_period = counter_hc32_set_guard_period,
	.get_freq = counter_hc32_get_freq,
};

void counter_hc32_irq_handler(const struct device *dev, uint32_t int_flag)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMR6_TypeDef *timer = config->timer;

	TMR6_ClearStatus(timer, int_flag);

	/* Capture compare events */
	switch (int_flag) {
	case TMR6_FLAG_MATCH_A:
		counter_hc32_alarm_irq_handle(dev, 0);
		break;
	case TMR6_FLAG_MATCH_B:
		counter_hc32_alarm_irq_handle(dev, 1);
		break;
	case TMR6_FLAG_MATCH_C:
		counter_hc32_alarm_irq_handle(dev, 2);
		break;
	case TMR6_FLAG_MATCH_D:
		counter_hc32_alarm_irq_handle(dev, 3);
		break;
	case TMR6_FLAG_MATCH_E:
		counter_hc32_alarm_irq_handle(dev, 4);
		break;
	case TMR6_FLAG_MATCH_F:
		counter_hc32_alarm_irq_handle(dev, 5);
		break;
	case TMR6_FLAG_OVF:
		counter_hc32_top_irq_handle(dev);
		break;
	default :
		break;
	}
}

static uint32_t counter_hc32_get_clk_freq(const struct device *dev)
{
	const struct counter_hc32_config *cfg = dev->config;
	uint32_t div;
	uint32_t clk_freq;

	if (cfg->clk_div <= 4UL) {
		div = 0x1UL << cfg->clk_div;
	} else {
		switch (cfg->clk_div) {
		case 5UL:
			div = 64UL;
			break;
		case 6UL:
			div = 256UL;
			break;
		case 7UL:
			div = 1024UL;
			break;
		default :
			div = 1UL;
			break;
		}
	}

	clk_freq = CLK_GetBusClockFreq(CLK_BUS_PCLK0) / div;

	return clk_freq;
}

static int counter_hc32_init_timer(const struct device *dev)
{
	const struct counter_hc32_config *cfg = dev->config;
	struct counter_hc32_data *data = dev->data;
	CM_TMR6_TypeDef *timer = cfg->timer;
	stc_tmr6_init_t stcTmr6Init;

	cfg->fcg_configure();

	TMR6_DeInit(timer);

	/* initialize timer */
	(void)TMR6_StructInit(&stcTmr6Init);
	TMR6_Stop(timer);

	/* Timer6 general count function configuration */
	stcTmr6Init.sw_count.u32ClockDiv = TIMER6_CLK_DIV(cfg->clk_div);
	stcTmr6Init.sw_count.u32CountMode = TMR6_MD_SAWTOOTH;
	stcTmr6Init.sw_count.u32CountDir = TMR6_CNT_UP;
	stcTmr6Init.u32PeriodValue = counter_get_max_top_value(dev);
	(void)TMR6_Init(timer, &stcTmr6Init);

	cfg->irq_configure();

	data->freq = counter_hc32_get_clk_freq(dev);

	return 0;
}

#define COUNTER_HC32_DEFINE_IRQ_HANDLER(inst, irq_name, int_flag)				    \
static void counter_hc32_irq_handler_##inst##_##irq_name(const struct device *dev)	\
{									                                                \
	counter_hc32_irq_handler(dev, int_flag);				                        \
}

#define DEF_ALL_IRQ_HANDLER(inst)                                       \
		COUNTER_HC32_DEFINE_IRQ_HANDLER(inst, cmpa, TMR6_FLAG_MATCH_A)  \
		COUNTER_HC32_DEFINE_IRQ_HANDLER(inst, cmpb, TMR6_FLAG_MATCH_B)  \
		COUNTER_HC32_DEFINE_IRQ_HANDLER(inst, cmpc, TMR6_FLAG_MATCH_C)  \
		COUNTER_HC32_DEFINE_IRQ_HANDLER(inst, cmpd, TMR6_FLAG_MATCH_D)  \
		COUNTER_HC32_DEFINE_IRQ_HANDLER(inst, cmpe, TMR6_FLAG_MATCH_E)  \
		COUNTER_HC32_DEFINE_IRQ_HANDLER(inst, cmpf, TMR6_FLAG_MATCH_F)  \
		COUNTER_HC32_DEFINE_IRQ_HANDLER(inst, ovf,  TMR6_FLAG_OVF)      \



#define TIMER6(inst)              DT_INST_PARENT(inst)

/** TMR6X instance from DT */
#define TMR6X(inst)               ((CM_TMR6_TypeDef *)DT_REG_ADDR(TIMER6(inst)))

#define TMR6X_COUNTER_MAX_CH(inst)  (DT_PROP(DT_DRV_INST(inst), max_ch))

#define TMR6X_IRQ_CONFIGURE(inst, irq_name)                                                                 \
	IRQ_CONNECT(DT_IRQ_BY_NAME(TIMER6(inst), irq_name, irq),                                                \
		    DT_IRQ_BY_NAME(TIMER6(inst), irq_name, priority), counter_hc32_irq_handler_##inst##_##irq_name, \
		    DEVICE_DT_INST_GET(inst), 0);                                                                   \
	hc32_intc_irq_signin(DT_IRQ_BY_NAME(TIMER6(inst), irq_name, irq),                                       \
						 DT_IRQ_BY_NAME(TIMER6(inst), irq_name, int_src));                                  \
	irq_enable(DT_IRQ_BY_NAME(TIMER6(inst), irq_name, irq));

#define COUNTER_DEVICE_INIT(inst)						  \
                                                          \
	BUILD_ASSERT(DT_PROP(TIMER6(inst), clock_division) <= HC32_460_45X_TMR6_CNT_CLK_DIV1024,  \
		     "TIMER6 clock division out of range");                                           \
	BUILD_ASSERT(DT_PROP(TIMER6(inst), count_mode) == HC32_TMR_CNT_MD_SAWTOOTH,               \
		     "TIMER6 counter only support SAWTOOTH mode");                                    \
	BUILD_ASSERT(DT_PROP(TIMER6(inst), count_direction) == HC32_TMR_CNT_DIR_UP,               \
		     "TIMER6 counter only support CNT_DIR_UP");                                       \
										  \
	DEF_ALL_IRQ_HANDLER(inst)			  \
										  \
	static struct hc32_modules_clock_sys                                                             \
		counter_hc32_##inst##_fcg_config = HC32_MODULES_CLOCK_INFO(0 , TIMER6(inst));                \
										                                                             \
	static struct counter_hc32_data counter_hc32_##inst##_data;									     \
	static struct counter_hc32_ch_data counter_hc32_##inst##_ch_data[TMR6X_COUNTER_MAX_CH(inst)];    \
										                                                             \
	static void counter_hc32_##inst##_fcg_configure(void)		                                     \
	{															                                     \
		clock_control_on(DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE), &counter_hc32_##inst##_fcg_config); \
	}															                                     \
	static void counter_hc32_##inst##_irq_configure(void)  \
	{									                   \
		TMR6X_IRQ_CONFIGURE(inst, cmpa)                    \
		TMR6X_IRQ_CONFIGURE(inst, cmpb)                    \
		TMR6X_IRQ_CONFIGURE(inst, cmpc)                    \
		TMR6X_IRQ_CONFIGURE(inst, cmpd)                    \
		TMR6X_IRQ_CONFIGURE(inst, cmpe)                    \
		TMR6X_IRQ_CONFIGURE(inst, cmpf)                    \
		TMR6X_IRQ_CONFIGURE(inst, ovf)                     \
	}									                   \
	static uint32_t counter_hc32_##inst##_irqn[TMR6X_COUNTER_MAX_CH(inst)] =  \
	{																		  \
		DT_IRQ_BY_NAME(TIMER6(inst), cmpa, irq),						      \
		DT_IRQ_BY_NAME(TIMER6(inst), cmpb, irq),							  \
		DT_IRQ_BY_NAME(TIMER6(inst), cmpc, irq),							  \
		DT_IRQ_BY_NAME(TIMER6(inst), cmpd, irq),							  \
		DT_IRQ_BY_NAME(TIMER6(inst), cmpe, irq),							  \
		DT_IRQ_BY_NAME(TIMER6(inst), cmpf, irq),							  \
	};																		  \
										                                      \
	static const struct counter_hc32_config counter_hc32_##inst##_config = {  \
		.info = {							                                  \
			.max_top_value = UINT16_MAX,			                          \
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,			                  \
			.channels = TMR6X_COUNTER_MAX_CH(inst),				              \
		},								                                      \
		.ch_data = counter_hc32_##inst##_ch_data,				              \
		.timer = TMR6X(inst),						                          \
		.clk_div = DT_PROP(TIMER6(inst), clock_division),			          \
		.cnt_mode = DT_PROP(TIMER6(inst), count_mode),			              \
		.cnt_dir = DT_PROP(TIMER6(inst), count_direction),			          \
		.fcg_configure = counter_hc32_##inst##_fcg_configure,		          \
		.irq_configure = counter_hc32_##inst##_irq_configure,		          \
		.irqn = counter_hc32_##inst##_irqn,					                  \
	};									                                      \
										                                      \
	DEVICE_DT_INST_DEFINE(inst,						                          \
			      counter_hc32_init_timer,				                      \
			      NULL,						                                  \
			      &counter_hc32_##inst##_data,				                  \
			      &counter_hc32_##inst##_config,				              \
			      PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,	              \
			      &counter_hc32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_DEVICE_INIT)
