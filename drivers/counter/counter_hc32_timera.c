/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <clock_control/hc32_clock_control.h>
#include <interrupt_controller/intc_hc32.h>
#include <clock_control.h>
#include <misc/util.h>
#include <kernel.h>
#include <soc.h>
#include <counter.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(counter_timera_hc32, CONFIG_COUNTER_LOG_LEVEL);

/* Get dts info */
#define TIMER_IRQ_ARRAY_BY_NAME(name, offset, prop)         \
        DT_XHSC_HC32_TIMERA_##name##_IRQ_##offset##_##prop
#define TIMER_IRQ_SINGLE_BY_NAME(name, offset)              \
        DT_XHSC_HC32_TIMERA_##name##_IRQ_##offset

#define TIMER_CLOCK_ARRAY_BY_NAME(name, offset, prop)       \
        DT_XHSC_HC32_TIMERA_##name##_CLOCK_##offset##_##prop
#define TIMER_CLOCK_SINGLE_BY_NAME(name, offset)            \
        DT_XHSC_HC32_TIMERA_##name##_CLOCK_##offset

#define TIMER_PROP_ARRAY_BY_NAME(name, prop, offset)        \
        DT_XHSC_HC32_TIMERA_##name##_##prop##_##offset
#define TIMER_PROP_SINGLE_BY_NAME(name, prop)               \
        DT_XHSC_HC32_TIMERA_##name##_##prop

#define COUNTER_PROP_ARRAY_BY_NAME(name, prop, offset)      \
        DT_XHSC_HC32_TIMERA_COUNTER_##name##_COUNTER_##prop##_##offset
#define COUNTER_PROP_SINGLE_BY_NAME(name, prop)             \
        DT_XHSC_HC32_TIMERA_COUNTER_##name##_COUNTER_##prop

/* timer info */
#define TIMERA_CH(ch)           (ch)
#define TIMERA_INT_CH(ch)       (TMRA_INT_CMP_CH1 << ch)
#define TIMERA_FLAG_CH(ch)      (TMRA_FLAG_CMP_CH1 << ch)
#define TIMERA_CLK_DIV(div)     (div << TMRA_BCSTRL_CKDIV_POS)

#define TIMERA_H16B_POS         (16)
#define TIMERA_H16B_MASK        (0xFFFF0000UL)
#define TIMERA_L16B_POS         (0)
#define TIMERA_L16B_MASK        (0x0000FFFFUL)

#define IS_SHIFTED_BIT_MASK(m, s)       (!(((m) >> (s)) & (((m) >> (s)) + 1U)))
#define IS_BIT_MASK(m)                  IS_SHIFTED_BIT_MASK(m, 0)

/* convenience defines */
#define COUNTER_DEV_CFG(dev)            \
	((struct counter_hc32_config * const)(dev)->config->config_info)
#define COUNTER_DEV_DATA(dev)           \
	((struct counter_hc32_data * const)(dev)->driver_data)


struct counter_hc32_ch_data {
	counter_alarm_callback_t callback;
	void *user_data;
	uint16_t start_tick;
	uint16_t tick_num;
};

struct counter_hc32_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	atomic_t cc_int_pending;
	uint16_t h16_period;
	volatile uint16_t h16b_cnt;
	struct counter_hc32_ch_data *ch_data;
};

struct counter_hc32_config {
	struct counter_config_info info;
	CM_TMRA_TypeDef *timer;
	uint32_t clock_division;
	struct hc32_modules_clock_sys clk_sys;
	void (*irq_config_func)(struct device *dev);
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

static int counter_hc32_start(struct device *dev)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);

	/* enable counter */
	data->h16b_cnt = 0;
	TMRA_Start(config->timer);

	return 0;
}

static int counter_hc32_stop(struct device *dev)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);

	/* disable counter */
	data->h16b_cnt = 0;
	TMRA_Stop(config->timer);

	return 0;
}

static uint32_t counter_hc32_get_top_value(struct device *dev)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);
	uint32_t value;

	value = TMRA_GetPeriodValue(config->timer);
	value += (((uint32_t)data->h16_period) << TIMERA_H16B_POS);

	return value;
}

static uint32_t counter_hc32_read(struct device *dev)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);
	uint32_t value;
	unsigned int s_count_lock;

	s_count_lock = irq_lock();
	value = TMRA_GetCountValue(config->timer);
	value += (((uint32_t)data->h16b_cnt) << TIMERA_H16B_POS);
	irq_unlock(s_count_lock);

	return value;
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
	struct device *dev, uint8_t chan_id)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);

	atomic_or(&data->cc_int_pending, BIT(chan_id));
	NVIC_SetPendingIRQ(config->irqn);
}

static int counter_hc32_set_cc(struct device *dev, uint8_t chan_id,
			       const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	CM_TMRA_TypeDef *timer = config->timer;
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);
	struct counter_hc32_ch_data *ch_alarm = &data->ch_data[chan_id];

	uint32_t val   = alarm_cfg->ticks;
	bool absolute = alarm_cfg->absolute;
	bool irq_on_late;
	uint32_t top = counter_hc32_get_top_value(dev);
	uint32_t now, diff, max_rel_val;
	uint16_t tick_val, cmp_val;
	int err = 0;

	__ASSERT(!(ENABLE == TMRA_GetIntEnable(timer, TIMERA_INT_CH(chan_id))),
		 "Expected that CC interrupt is disabled.");
	/* First take care of a risk of an event coming from CC being set to
	 * next tick. Reconfigure CC to future (now tick is the furthest future). */
	TMRA_SetFunc(timer, TIMERA_CH(chan_id), TMRA_FUNC_CMP);
	now = counter_hc32_read(dev);
	ch_alarm->start_tick = now >> TIMERA_H16B_POS;
	TMRA_SetCompareValue(timer, TIMERA_CH(chan_id), (now & TIMERA_L16B_MASK) - 1);
	TMRA_ClearStatus(timer, TIMERA_FLAG_CH(chan_id));

	if (absolute) {
		max_rel_val = top;
		irq_on_late = 0;
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
	TMRA_SetCompareValue(timer, TIMERA_CH(chan_id), cmp_val);

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
			counter_hc32_counter_hc32_set_cc_int_pending(dev, chan_id);
		} else {
			data->ch_data[chan_id].callback = NULL;
		}
	} else {
		TMRA_IntCmd(timer, TIMERA_INT_CH(chan_id), ENABLE);
	}

	return err;
}

static int counter_hc32_set_alarm(struct device *dev, u8_t chan_id,
				  const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);
	struct counter_hc32_ch_data *ch_alarm = &data->ch_data[chan_id];

	if (alarm_cfg->ticks > counter_hc32_get_top_value(dev)) {
		return -EINVAL;
	}

	if (ch_alarm->callback) {
		return -EBUSY;
	}
	ch_alarm->callback  = alarm_cfg->callback;
	ch_alarm->user_data = alarm_cfg->user_data;

	return counter_hc32_set_cc(dev, chan_id, alarm_cfg);
}

static int counter_hc32_cancel_alarm(struct device *dev, uint8_t chan_id)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);

	TMRA_IntCmd(config->timer, TIMERA_INT_CH(chan_id), DISABLE);
	data->ch_data[chan_id].callback = NULL;

	return 0;
}

static int counter_hc32_set_top_value(struct device *dev, u32_t ticks,
				      counter_top_callback_t callback, void *user_data)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	CM_TMRA_TypeDef *timer = config->timer;
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);
	int err = 0;

	for (uint32_t i = 0; i < counter_get_num_of_channels(dev); i++) {
		/* Overflow can be changed only when all alarms are disabled. */
		if (data->ch_data[i].callback) {
			return -EBUSY;
		}
	}

	TMRA_IntCmd(timer, TMRA_INT_OVF, DISABLE);
	TMRA_SetPeriodValue(timer, (ticks & TIMERA_L16B_MASK));
	data->h16_period = ticks >> TIMERA_H16B_POS;
	TMRA_ClearStatus(timer, TMRA_FLAG_OVF);
	data->top_cb        = callback;
	data->top_user_data = user_data;

	TMRA_Stop(timer);
	TMRA_SetCountValue(timer, 0);
	data->h16b_cnt = 0;
	TMRA_Start(timer);
	TMRA_IntCmd(timer, TMRA_INT_OVF, ENABLE);

	return err;
}

static uint32_t counter_hc32_get_pending_int(struct device *dev)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	uint32_t pending = 0;

	pending = NVIC_GetPendingIRQ(config->irqn);

	return !!pending;
}

static uint32_t counter_hc32_get_max_relative_alarm(struct device *dev)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);

	return config->info.max_top_value;
}

static int counter_hc32_init_timer(struct device *dev)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	CM_TMRA_TypeDef *timer = config->timer;
	struct device *clk_node;
	stc_tmra_init_t stcTmraInit;
	uint32_t tim_clk;
	int ret;

	/* initialize clock and check its speed  */
	clk_node = device_get_binding(HC32_CLOCK_CONTROL_NAME);
	__ASSERT_NO_MSG(clk_node);
	ret = clock_control_on(clk_node, (clock_control_subsys_t)&config->clk_sys);
	if (ret != 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return -EIO;
	}
	ret = clock_control_get_rate(clk_node, (clock_control_subsys_t)&config->clk_sys,
				     &tim_clk);
	if (ret != 0) {
		LOG_ERR("Could not obtain timer clock (%d)", ret);
		return -EIO;
	}
	config->info.freq = tim_clk / BIT(config->clock_division);

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
	.start                  = counter_hc32_start,
	.stop                   = counter_hc32_stop,
	.read                   = counter_hc32_read,
	.set_alarm              = counter_hc32_set_alarm,
	.cancel_alarm           = counter_hc32_cancel_alarm,
	.set_top_value          = counter_hc32_set_top_value,
	.get_pending_int        = counter_hc32_get_pending_int,
	.get_top_value          = counter_hc32_get_top_value,
	.get_max_relative_alarm = counter_hc32_get_max_relative_alarm,
};

void count_hc32_alarm_irq_handler(struct device *dev, uint32_t chan_id)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	CM_TMRA_TypeDef *timer = config->timer;
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);
	counter_alarm_callback_t cb;
	struct counter_hc32_ch_data *ch_alarm;
	bool hw_irq = (SET == TMRA_GetStatus(timer, TIMERA_FLAG_CH(chan_id))) &&
		      (ENABLE == TMRA_GetIntEnable(timer, TIMERA_INT_CH(chan_id)));
	bool sw_irq = data->cc_int_pending & BIT(chan_id);

	if (hw_irq || sw_irq) {
		ch_alarm = &data->ch_data[chan_id];
		if (hw_irq) {
			TMRA_ClearStatus(timer, TIMERA_FLAG_CH(chan_id));
			if (!sw_irq) {
				if ((data->h16b_cnt - ch_alarm->start_tick) < ch_alarm->tick_num) {
					return;
				}
			}
		}
		atomic_and(&data->cc_int_pending, ~BIT(chan_id));
		TMRA_IntCmd(timer, TIMERA_INT_CH(chan_id), DISABLE);

		cb = ch_alarm->callback;
		ch_alarm->callback = NULL;
		if (cb) {
			uint32_t cc_val = TMRA_GetCompareValue(timer, TIMERA_CH(chan_id));
			cb(dev, chan_id, cc_val, ch_alarm->user_data);
		}
	}
}

void counter_hc32_ovf_irq_handler(struct device *dev)
{
	struct counter_hc32_config *config = COUNTER_DEV_CFG(dev);
	CM_TMRA_TypeDef *timer = config->timer;
	struct counter_hc32_data *data = COUNTER_DEV_DATA(dev);
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

void counter_hc32_cmp_irq_handler(struct device *dev)
{
	for (uint32_t i = 0; i < counter_get_num_of_channels(dev); i++) {
		count_hc32_alarm_irq_handler(dev, i);
	}
}

/* Timer base address */
#define TIMER_BASE_ADDR(addr)           ((CM_TMRA_TypeDef *)0x##addr)

#define HC32_COUNTER_IRQ_HANDLER_DECL(idx)				                \
	static void counter_##idx##_hc32_irq_config(struct device *dev)

#define HC32_COUNTER_IRQ_HANDLER(idx, addr)					            \
    static void counter_##idx##_hc32_irq_config(struct device *dev)	    \
	{									                                \
		IRQ_CONNECT(TIMER_IRQ_SINGLE_BY_NAME(addr, OVF),                \
			        TIMER_IRQ_ARRAY_BY_NAME(addr, OVF, PRIORITY),       \
			        counter_hc32_ovf_irq_handler,                       \
                    DEVICE_GET(timera_count_hc32_##idx), 0);            \
		hc32_intc_irq_signin(TIMER_IRQ_SINGLE_BY_NAME(addr, OVF),		\
		            TIMER_IRQ_ARRAY_BY_NAME(addr, OVF, INT_SRC));	    \
		irq_enable(TIMER_IRQ_SINGLE_BY_NAME(addr, OVF));                \
		IRQ_CONNECT(TIMER_IRQ_SINGLE_BY_NAME(addr, CMP),                \
			        TIMER_IRQ_ARRAY_BY_NAME(addr, CMP, PRIORITY),       \
			        counter_hc32_cmp_irq_handler,                       \
                    DEVICE_GET(timera_count_hc32_##idx), 0);            \
		hc32_intc_irq_signin(TIMER_IRQ_SINGLE_BY_NAME(addr, CMP),	    \
		            TIMER_IRQ_ARRAY_BY_NAME(addr, CMP, INT_SRC));	    \
		irq_enable(TIMER_IRQ_SINGLE_BY_NAME(addr, CMP));                \
	}

#define COUNTER_DEVICE_INIT(idx, addr)						                    \
	HC32_COUNTER_IRQ_HANDLER_DECL(idx);					                        \
	static struct counter_hc32_ch_data                                          \
	counter##idx##_ch_data[COUNTER_PROP_SINGLE_BY_NAME(addr, CH_NUMS)];         \
	static struct counter_hc32_data counter##idx##_data = {		                \
		.ch_data = counter##idx##_ch_data,			                            \
		.h16b_cnt = 0,                                                          \
		.h16_period = 0xFFFF,                                                   \
	};									                                        \
	static struct counter_hc32_config counter##idx##_config = {	                \
		.info = {							                                    \
			.max_top_value = 0xFFFFFFFF,			                            \
			.count_up = true,		                    	                    \
			.channels = COUNTER_PROP_SINGLE_BY_NAME(addr, CH_NUMS),				\
		},								                                        \
		.timer = TIMER_BASE_ADDR(addr),						                    \
		.clock_division = TIMER_PROP_SINGLE_BY_NAME(addr, CLOCK_DIVISION),		\
		.clk_sys = {							                                \
			.bus  = TIMER_CLOCK_SINGLE_BY_NAME(addr, BUS),                      \
			.fcg  = TIMER_CLOCK_SINGLE_BY_NAME(addr, FCG),                      \
			.bits = TIMER_CLOCK_SINGLE_BY_NAME(addr, BITS),                     \
		},							                                            \
		.irq_config_func = counter_##idx##_hc32_irq_config,		                \
		.irqn = TIMER_IRQ_SINGLE_BY_NAME(addr, CMP),					        \
	};									                                        \
	DEVICE_AND_API_INIT(timera_count_hc32_##idx,						        \
			      COUNTER_PROP_SINGLE_BY_NAME(addr, LABEL),				        \
			      &counter_hc32_init_timer,						                \
			      &counter##idx##_data,				                            \
			      &counter##idx##_config,				                        \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	            \
			      &counter_hc32_driver_api);                                    \
	HC32_COUNTER_IRQ_HANDLER(idx, addr)


#ifdef DT_XHSC_HC32_TIMERA_40015000_LABEL
#ifdef DT_XHSC_HC32_TIMERA_COUNTER_40015000_COUNTER_LABEL
COUNTER_DEVICE_INIT(0, 40015000)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_40015400_LABEL
#ifdef DT_XHSC_HC32_TIMERA_COUNTER_40015400_COUNTER_LABEL
COUNTER_DEVICE_INIT(1, 40015400)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_40015800_LABEL
#ifdef DT_XHSC_HC32_TIMERA_COUNTER_40015800_COUNTER_LABEL
COUNTER_DEVICE_INIT(2, 40015800)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_40015C00_LABEL
#ifdef DT_XHSC_HC32_TIMERA_COUNTER_40015C00_COUNTER_LABEL
COUNTER_DEVICE_INIT(3, 40015C00)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_40016000_LABEL
#ifdef DT_XHSC_HC32_TIMERA_COUNTER_40016000_COUNTER_LABEL
COUNTER_DEVICE_INIT(4, 40016000)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_40016400_LABEL
#ifdef DT_XHSC_HC32_TIMERA_COUNTER_40016400_COUNTER_LABEL
COUNTER_DEVICE_INIT(5, 40016400)
#endif
#endif
