/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_timera_counter

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>
#include <zephyr/dt-bindings/timer/hc32-timer.h>
// #include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_timer_hc32, CONFIG_COUNTER_LOG_LEVEL);


#define TIMERA_CH(ch)           (ch)
#define TIMERA_INT_CH(ch)       (TMRA_INT_CMP_CH1 << ch)
#define TIMERA_FLAG_CH(ch)      (TMRA_FLAG_CMP_CH1 << ch)
#define TIMERA_CLK_DIV(div)     (div << TMRA_BCSTRL_CKDIV_POS)

#define TIMERA_H16B_POS         (16)
#define TIMERA_H16B_MASK        (0xFFFF0000UL)
#define TIMERA_L16B_POS         (0)
#define TIMERA_L16B_MASK        (0x0000FFFFUL)


struct counter_hc32_ch_data {
	counter_alarm_callback_t callback;
	void *user_data;
	uint16_t start_tick;
	uint16_t tick_num;
};

struct counter_hc32_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	uint32_t guard_period;
	atomic_t cc_int_pending;
	uint32_t freq;
	uint16_t h16_period;
	volatile uint16_t h16b_cnt;
	struct counter_hc32_ch_data *ch_data;
};

struct counter_hc32_config {
	struct counter_config_info info;
	// struct reset_dt_spec reset;
	CM_TMRA_TypeDef *timer;
	uint32_t clock_division;
	struct hc32_modules_clock_sys clk_sys;
	void (*irq_config_func)(const struct device *dev);
	uint32_t irqn;
	LOG_INSTANCE_PTR_DECLARE(log);
};

static en_functional_state_t TMRA_GetIntEnable(CM_TMRA_TypeDef *TMRAx,
					       uint32_t u32IntType)
{
	uint32_t u32BCSTRH;
	uint32_t u32ICONR;
	en_functional_state_t enState;

	u32BCSTRH = u32IntType & (TMRA_BCSTRH_ITENUDF | TMRA_BCSTRH_ITENOVF);
	u32ICONR  = u32IntType >> 16U;
	if ((0 != (TMRAx->BCSTRH & u32BCSTRH)) || (0 != (TMRAx->ICONR & u32ICONR))) {
		enState = ENABLE;
	} else {
		enState = DISABLE;
	}

	return enState;
}

static int counter_hc32_start(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;

	/* enable counter */
	data->h16b_cnt = 0;
	TMRA_Start(config->timer);

	return 0;
}

static int counter_hc32_stop(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;

	/* disable counter */
	data->h16b_cnt = 0;
	TMRA_Stop(config->timer);

	return 0;
}

static uint32_t counter_hc32_get_top_value(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;
	uint32_t value;

	value = TMRA_GetPeriodValue(config->timer);
	value += (((uint32_t)data->h16_period) << TIMERA_H16B_POS);

	return value;
}

static uint32_t counter_hc32_read(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;
	uint32_t value;
	unsigned int s_count_lock;

	s_count_lock = irq_lock();
	value = TMRA_GetCountValue(config->timer);
	value += (((uint32_t)data->h16b_cnt) << TIMERA_H16B_POS);
	irq_unlock(s_count_lock);

	return value;
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

	return (val2 <= to_top) ? val1 + val2 : val2 - to_top;
}

static uint32_t counter_hc32_ticks_sub(uint32_t val, uint32_t old,
				       uint32_t top)
{
	if (likely(IS_BIT_MASK(top))) {
		return (val - old) & top;
	}

	/* if top is not 2^n-1 */
	return (val >= old) ? (val - old) : val + (top - old + 1U);
}

static void counter_hc32_counter_hc32_set_cc_int_pending(
	const struct device *dev, uint8_t chan)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;

	atomic_or(&data->cc_int_pending, BIT(chan));
	NVIC_SetPendingIRQ(config->irqn);
}

static int counter_hc32_set_cc(const struct device *dev, uint8_t chan,
			       const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMRA_TypeDef *timer = config->timer;
	struct counter_hc32_data *data = dev->data;
	struct counter_hc32_ch_data *ch_alarm = &data->ch_data[chan];
	__ASSERT_NO_MSG(data->guard_period < counter_hc32_get_top_value(dev));

	uint32_t val   = alarm_cfg->ticks;
	uint32_t flags = alarm_cfg->flags;
	bool absolute = flags & COUNTER_ALARM_CFG_ABSOLUTE;
	bool irq_on_late;
	uint32_t top = counter_hc32_get_top_value(dev);
	uint32_t now, diff, max_rel_val;
	uint16_t tick_val, cmp_val;
	int err = 0;

	__ASSERT(!(ENABLE == TMRA_GetIntEnable(timer, TIMERA_INT_CH(chan))),
		 "Expected that CC interrupt is disabled.");
	/* First take care of a risk of an event coming from CC being set to
	 * next tick. Reconfigure CC to future (now tick is the furthest future). */
	TMRA_SetFunc(timer, TIMERA_CH(chan), TMRA_FUNC_CMP);
	now = counter_hc32_read(dev);
	ch_alarm->start_tick = now >> TIMERA_H16B_POS;
	TMRA_SetCompareValue(timer, TIMERA_CH(chan), (now & TIMERA_L16B_MASK) - 1);
	TMRA_ClearStatus(timer, TIMERA_FLAG_CH(chan));

	if (absolute) {
		max_rel_val = top - data->guard_period;
		irq_on_late = flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
		/* calculate next compare point */
		ch_alarm->tick_num = val >> TIMERA_H16B_POS;
		cmp_val = val & TIMERA_L16B_MASK;
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
		/* calculate next compare point */
		tick_val = val >> TIMERA_H16B_POS;
		cmp_val = val & TIMERA_L16B_MASK;
		ch_alarm->tick_num = tick_val - ch_alarm->start_tick;
	}
	TMRA_SetCompareValue(timer, TIMERA_CH(chan), cmp_val);

	/* decrement value to detect also case when val == counter_hc32_read(dev).
	 * Otherwise, condition would need to include comparing diff against 0. */
	diff = counter_hc32_ticks_sub(val - 1U, counter_hc32_read(dev), top);
	if (diff > max_rel_val) {
		if (absolute) {
			err = -ETIME;
		}
		/* Interrupt is triggered always for relative alarm and
		 * for absolute depending on the flag. */
		if (irq_on_late) {
			counter_hc32_counter_hc32_set_cc_int_pending(dev, chan);
		} else {
			data->ch_data[chan].callback = NULL;
		}
	} else {
		TMRA_IntCmd(timer, TIMERA_INT_CH(chan), ENABLE);
	}

	return err;
}

static int counter_hc32_set_alarm(const struct device *dev, uint8_t chan,
				  const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_hc32_data *data = dev->data;
	struct counter_hc32_ch_data *ch_alarm = &data->ch_data[chan];

	if (alarm_cfg->ticks > counter_hc32_get_top_value(dev)) {
		return -EINVAL;
	}

	if (ch_alarm->callback) {
		return -EBUSY;
	}
	ch_alarm->callback  = alarm_cfg->callback;
	ch_alarm->user_data = alarm_cfg->user_data;

	return counter_hc32_set_cc(dev, chan, alarm_cfg);
}

static int counter_hc32_cancel_alarm(const struct device *dev, uint8_t chan)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;

	TMRA_IntCmd(config->timer, TIMERA_INT_CH(chan), DISABLE);
	data->ch_data[chan].callback = NULL;

	return 0;
}

static int counter_hc32_set_top_value(const struct device *dev,
				      const struct counter_top_cfg *cfg)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMRA_TypeDef *timer = config->timer;
	struct counter_hc32_data *data = dev->data;
	int err = 0;

	for (uint32_t i = 0; i < counter_get_num_of_channels(dev); i++) {
		/* Overflow can be changed only when all alarms are disabled. */
		if (data->ch_data[i].callback) {
			return -EBUSY;
		}
	}

	TMRA_IntCmd(timer, TMRA_INT_OVF, DISABLE);
	TMRA_SetPeriodValue(timer, (cfg->ticks & TIMERA_L16B_MASK));
	data->h16_period = cfg->ticks >> TIMERA_H16B_POS;
	TMRA_ClearStatus(timer, TMRA_FLAG_OVF);
	data->top_cb        = cfg->callback;
	data->top_user_data = cfg->user_data;

	if (!(cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
		TMRA_Stop(timer);
		TMRA_SetCountValue(timer, 0);
		data->h16b_cnt = 0;
		TMRA_Start(timer);
	} else if (counter_hc32_read(dev) >= cfg->ticks) {
		err = -ETIME;
		if (cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
			TMRA_Stop(timer);
			TMRA_SetCountValue(timer, 0);
			data->h16b_cnt = 0;
			TMRA_Start(timer);
		}
	}
	TMRA_IntCmd(timer, TMRA_INT_OVF, ENABLE);

	return err;
}

static uint32_t counter_hc32_get_pending_int(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	uint32_t pending = 0;

	pending = NVIC_GetPendingIRQ(config->irqn);

	return !!pending;
}

static uint32_t counter_hc32_get_guard_period(const struct device *dev,
					      uint32_t flags)
{
	struct counter_hc32_data *data = dev->data;

	ARG_UNUSED(flags);
	return data->guard_period;
}

static int counter_hc32_set_guard_period(const struct device *dev,
					 uint32_t guard, uint32_t flags)
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

static int counter_hc32_init_timer(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	struct counter_hc32_data *data = dev->data;
	CM_TMRA_TypeDef *timer = config->timer;
	const struct device *clk_node;
	stc_tmra_init_t stcTmraInit;
	uint32_t tim_clk;
	int ret;

	/* initialize clock and check its speed  */
	clk_node = DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE);
	if (!device_is_ready(clk_node)) {
		LOG_ERR("Could not get clock control device");
		return -ENODEV;
	}
	ret = clock_control_on(clk_node, (clock_control_subsys_t)&config->clk_sys);
	if (ret < 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return ret;
	}
	ret = clock_control_get_rate(clk_node, (clock_control_subsys_t)&config->clk_sys,
				     &tim_clk);
	if (ret < 0) {
		LOG_ERR("Could not obtain timer clock (%d)", ret);
		return ret;
	}
	data->freq = tim_clk / BIT(config->clock_division);

	// if (!device_is_ready(data->reset.dev)) {
	// 	LOG_ERR("reset controller not ready");
	// 	return -ENODEV;
	// }

	/* config enable IRQ */
	config->irq_config_func(dev);
	/* initialize timer */
	(void)TMRA_StructInit(&stcTmraInit);
	stcTmraInit.sw_count.u8ClockDiv  = TIMERA_CLK_DIV(config->clock_division);
	stcTmraInit.sw_count.u8CountMode = TMRA_MD_SAWTOOTH;
	stcTmraInit.sw_count.u8CountDir  = TMRA_DIR_UP;
	stcTmraInit.u32PeriodValue = counter_get_max_top_value(dev) & TIMERA_L16B_MASK;
	(void)TMRA_Init(timer, &stcTmraInit);
	TMRA_IntCmd(timer, TMRA_INT_OVF, ENABLE);

	return 0;
}

static const struct counter_driver_api counter_hc32_driver_api = {
	.start              = counter_hc32_start,
	.stop               = counter_hc32_stop,
	.get_value          = counter_hc32_get_value,
	.set_alarm          = counter_hc32_set_alarm,
	.cancel_alarm       = counter_hc32_cancel_alarm,
	.set_top_value      = counter_hc32_set_top_value,
	.get_pending_int    = counter_hc32_get_pending_int,
	.get_top_value      = counter_hc32_get_top_value,
	.get_guard_period   = counter_hc32_get_guard_period,
	.set_guard_period   = counter_hc32_set_guard_period,
	.get_freq           = counter_hc32_get_freq,
};

void count_hc32_alarm_irq_handler(const struct device *dev, uint32_t chan)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMRA_TypeDef *timer = config->timer;
	struct counter_hc32_data *data = dev->data;
	counter_alarm_callback_t cb;
	struct counter_hc32_ch_data *ch_alarm;
	bool hw_irq = (SET == TMRA_GetStatus(timer, TIMERA_FLAG_CH(chan))) &&
		      (ENABLE == TMRA_GetIntEnable(timer, TIMERA_INT_CH(chan)));
	bool sw_irq = data->cc_int_pending & BIT(chan);

	if (hw_irq || sw_irq) {
		ch_alarm = &data->ch_data[chan];
		if (hw_irq) {
			TMRA_ClearStatus(timer, TIMERA_FLAG_CH(chan));
			if (!sw_irq) {
				if ((data->h16b_cnt - ch_alarm->start_tick) < ch_alarm->tick_num) {
					return;
				}
			}
		}
		atomic_and(&data->cc_int_pending, ~BIT(chan));
		TMRA_IntCmd(timer, TIMERA_INT_CH(chan), DISABLE);

		cb = ch_alarm->callback;
		ch_alarm->callback = NULL;
		if (cb) {
			uint32_t cc_val = TMRA_GetCompareValue(timer, TIMERA_CH(chan));
			cb(dev, chan, cc_val, ch_alarm->user_data);
		}
	}
}

void counter_hc32_ovf_irq_handler(const struct device *dev)
{
	const struct counter_hc32_config *config = dev->config;
	CM_TMRA_TypeDef *timer = config->timer;
	struct counter_hc32_data *data = dev->data;
	counter_top_callback_t cb = data->top_cb;

	if (cb != NULL) {
		if ((SET == TMRA_GetStatus(timer, TMRA_FLAG_OVF)) &&
		    (ENABLE == TMRA_GetIntEnable(timer, TMRA_INT_OVF))) {
			TMRA_ClearStatus(timer, TMRA_FLAG_OVF);
			__ASSERT(cb != NULL, "top event enabled - expecting callback");
			cb(dev, data->top_user_data);
		}
	}
	data->h16b_cnt++;
	if (data->h16b_cnt >= data->h16_period) {
		data->h16b_cnt = 0;
	}
}

void counter_hc32_cmp_irq_handler(const struct device *dev)
{
	for (uint32_t i = 0; i < counter_get_num_of_channels(dev); i++) {
		count_hc32_alarm_irq_handler(dev, i);
	}
}

/* Timer node id */
#define TIMER(idx)              DT_INST_PARENT(idx)
/* Timer base address from DT */
#define TIMER_BASE_ADDR(idx)    ((CM_TMRA_TypeDef *)DT_REG_ADDR(TIMER(idx)))

#define COUNTER_DEVICE_INIT(idx)						                                        \
	BUILD_ASSERT(DT_PROP(TIMER(idx), clock_division) <= HC32_TMR_CNT_CLK_DIV1024,		        \
		     "TIMER clock division out of range");				                                \
	static struct counter_hc32_ch_data counter##idx##_ch_data[DT_INST_PROP(idx, ch_nums)];      \
	static struct counter_hc32_data counter##idx##_data = {		                                \
		.ch_data = counter##idx##_ch_data,			                                            \
		.h16b_cnt = 0,                                                                          \
		.h16_period = 0xFFFF,                                                                   \
	};									                                                        \
    static void counter_##idx##_hc32_irq_config(const struct device *dev)	    \
	{									                                        \
		IRQ_CONNECT(DT_IRQ_BY_NAME(TIMER(idx), ovf, irq),                       \
			        DT_IRQ_BY_NAME(TIMER(idx), ovf, priority),                  \
			        counter_hc32_ovf_irq_handler, DEVICE_DT_INST_GET(idx), 0);  \
		hc32_intc_irq_signin(DT_IRQ_BY_NAME(TIMER(idx), ovf, irq),		    	\
		                     DT_IRQ_BY_NAME(TIMER(idx), ovf, int_src));	    	\
		irq_enable(DT_IRQ_BY_NAME(TIMER(idx), ovf, irq));                       \
		IRQ_CONNECT(DT_IRQ_BY_NAME(TIMER(idx), cmp, irq),                       \
			        DT_IRQ_BY_NAME(TIMER(idx), cmp, priority),                  \
			        counter_hc32_cmp_irq_handler, DEVICE_DT_INST_GET(idx), 0);  \
		hc32_intc_irq_signin(DT_IRQ_BY_NAME(TIMER(idx), cmp, irq),			    \
		                     DT_IRQ_BY_NAME(TIMER(idx), cmp, int_src));		    \
		irq_enable(DT_IRQ_BY_NAME(TIMER(idx), cmp, irq));                       \
	}									                                        \
	static const struct counter_hc32_config counter##idx##_config = {	        \
		.info = {							                                    \
			.max_top_value = 0xFFFFFFFF,			                            \
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,			                    \
			.channels = DT_INST_PROP(idx, ch_nums),				                \
		},								                                        \
		.timer = TIMER_BASE_ADDR(idx),						                    \
		.clock_division = DT_PROP(TIMER(idx), clock_division),			        \
		.clk_sys = {							                                \
			.bus  = DT_CLOCKS_CELL(TIMER(idx), bus),                            \
			.fcg  = DT_CLOCKS_CELL(TIMER(idx), fcg),                            \
			.bits = DT_CLOCKS_CELL(TIMER(idx), bits),                           \
		},							                                            \
		.irq_config_func = counter_##idx##_hc32_irq_config,		                \
		.irqn = DT_IRQ_BY_NAME(TIMER(idx), cmp, irq),					        \
	};									                                        \
	DEVICE_DT_INST_DEFINE(idx,						            \
			      counter_hc32_init_timer,				        \
			      NULL,						                    \
			      &counter##idx##_data,				            \
			      &counter##idx##_config,				        \
			      PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,	\
			      &counter_hc32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_DEVICE_INIT)
