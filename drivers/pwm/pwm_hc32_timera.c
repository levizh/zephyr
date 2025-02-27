/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_timera_pwm

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pwm.h>

#include <hc32_ll_tmra.h>
#include <hc32_ll.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(pwm_hc32_timera, CONFIG_PWM_LOG_LEVEL);

/* Timer count mode values */
#define HC32_TMR_CNT_MD_SAWTOOTH        0x0000U
#define HC32_TMR_CNT_MD_TRIANGLE        0x0001U

#ifdef CONFIG_PWM_CAPTURE

#define CAPTURE_PERIOD 0xFFFF
/**
 * @brief Capture state when in 4-channel support mode
 */
enum capture_state {
	CAPTURE_STATE_IDLE = 0U,
	CAPTURE_STATE_WAIT_FOR_PERIOD_START = 1U,
	CAPTURE_STATE_WAIT_FOR_PERIOD_END = 2U,
};

struct pwm_hc32_capture_data {
	pwm_capture_callback_handler_t callback;
	void *user_data;
	uint32_t period;
	uint32_t overflows;
	uint32_t underflows;
	bool capture_period;
	bool continuous;
	uint8_t channel;
#ifdef CONFIG_HC32_TIMERA_CAPTURE_FREQ_HIGH
	uint8_t bcstrl_value;
#endif
	enum capture_state state;
};

#endif /*CONFIG_PWM_CAPTURE*/

/** PWM data. */
struct pwm_hc32_data {
	/** Timer clock (Hz). */
	uint32_t tim_clk;
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_hc32_capture_data capture;
#endif /* CONFIG_PWM_CAPTURE */
};

/** PWM configuration. */
struct pwm_hc32_config {
	CM_TMRA_TypeDef *timera;
	uint32_t prescaler;
	uint32_t countermode;
	uint32_t direction;
	struct hc32_modules_clock_sys clk_sys;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_PWM_CAPTURE
	void (*irq_config_func)(const struct device *dev);
#endif /* CONFIG_PWM_CAPTURE */
};

/* Maximum number of timer channels */
#if defined(HC32F460)
#define TIMER_MAX_CH 8U
#elif defined(HC32F4A0)
#define TIMER_MAX_CH 4U
#endif

static int pwm_hc32_set_cycles(const struct device *dev, uint32_t channel,
			       uint32_t period_cycles, uint32_t pulse_cycles,
			       pwm_flags_t flags)
{
	const struct pwm_hc32_config *cfg = dev->config;
	static stc_tmra_pwm_init_t stcPwmInit;
	int ret;

	if (channel < 1U || channel > TIMER_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	/* (16-bit), Thus period_cycles cannot be greater than UINT16_MAX + 1. */
	if (period_cycles > UINT16_MAX + 1U) {
		return -ENOTSUP;
	}

	if (period_cycles == 0U) {
		LOG_WRN("Set period_cycles = 0");
		TMRA_Stop(cfg->timera);
		TMRA_PWM_OutputCmd(cfg->timera, channel - 1, DISABLE);
		return 0;
	}

#if defined(HC32F460) || defined(HC32F4A0)
	if (cfg->prescaler != 0U) {
		LOG_ERR("Cannot set PWM start level for clock-division isn't 1U");
		return -ENOTSUP;
	}
#endif
	TMRA_ClearStatus(cfg->timera, TMRA_FLAG_UDF | TMRA_FLAG_OVF);
	if (HC32_TMR_CNT_MD_SAWTOOTH == cfg->countermode) {/* TMRA_MD_SAWTOOTH */
		if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
			stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
			stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
			stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
			stcPwmInit.u16StopPolarity        = TMRA_PWM_LOW;
			if (pulse_cycles == 0U) {
				stcPwmInit.u16StartPolarity       = TMRA_PWM_LOW;
				stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
				stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_LOW;
			} else if (pulse_cycles == period_cycles) {
				stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
			}
		} else {
			stcPwmInit.u16StartPolarity       = TMRA_PWM_LOW;
			stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
			stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_LOW;
			stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
			if (pulse_cycles == 0U) {
				stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
				stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
				stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;

			} else if (pulse_cycles == period_cycles) {
				stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
			}
		}
		/* DIR down = 1 */
		if (cfg->direction) {
			pulse_cycles = period_cycles - pulse_cycles;
		}
		/* 0% or 100% do not to update pulse */
		if ((pulse_cycles != 0U) && (pulse_cycles != period_cycles)) {
			stcPwmInit.u32CompareValue = pulse_cycles;
		}

		if (cfg->timera->BCSTRL & TMRA_BCSTRL_START) {
			/* timera has run */
			if (cfg->direction) {
				/* wait for underflows to update next set */
				while (RESET == TMRA_GetStatus(cfg->timera, TMRA_FLAG_UDF)) {
					/* NOP */
				}
			} else {
				while (RESET == TMRA_GetStatus(cfg->timera, TMRA_FLAG_OVF)) {
					/* up direction, then wait for overflows flag */
				}
			}
		} else {
			TMRA_SetCountValue(cfg->timera, 0U);
		}
		TMRA_ClearStatus(cfg->timera, TMRA_FLAG_UDF | TMRA_FLAG_OVF);
		TMRA_SetPeriodValue(cfg->timera, (period_cycles - 1U));
	} else {
		/* TMRA_MD_TRIANGLE */
		if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
			stcPwmInit.u16StartPolarity       = TMRA_PWM_LOW;
			stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_INVT;
			stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HOLD;
			stcPwmInit.u16StopPolarity        = TMRA_PWM_LOW;
			if (pulse_cycles == 0U) {
				stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
			} else if (pulse_cycles == period_cycles) {
				stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
			}
		} else {
			stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
			stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_INVT;
			stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HOLD;
			stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
			if (pulse_cycles == 0U) {
				stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
			} else if (pulse_cycles == period_cycles) {
				stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
			}
		}
		/* 0% or 100% do not to update pulse */
		if ((pulse_cycles != 0U) && (pulse_cycles != period_cycles)) {
			stcPwmInit.u32CompareValue = (period_cycles - pulse_cycles) / 2U;
		}
		/* check if timera has start */
		if (cfg->timera->BCSTRL & TMRA_BCSTRL_START) {
			/* wait for underflows to update next set */
			while (RESET == TMRA_GetStatus(cfg->timera, TMRA_FLAG_UDF)) {
				/* NOP */
			}
			/* for TRIANGLE mode must stop timera to change the polarity and start level */
			TMRA_Stop(cfg->timera);
			TMRA_ClearStatus(cfg->timera, TMRA_FLAG_UDF);

		} else {
			TMRA_SetCountValue(cfg->timera, 1U);
		}
		TMRA_SetPeriodValue(cfg->timera, period_cycles / 2U);
	}

	ret = TMRA_PWM_Init(cfg->timera, channel - 1U, &stcPwmInit);
	if (ret) {
		LOG_ERR("Could not initialize timera channel for pwm");
		return -EIO;
	}
	TMRA_SetFunc(cfg->timera, channel - 1U, TMRA_FUNC_CMP);
	TMRA_PWM_OutputCmd(cfg->timera, channel - 1U, ENABLE);
	TMRA_Start(cfg->timera);

	return 0;
}

#ifdef CONFIG_PWM_CAPTURE
#ifdef CONFIG_HC32_TIMERA_CAPTURE_FREQ_HIGH
static uint32_t hc32_get_period(uint32_t *capt, uint32_t data_len)
{
	uint32_t diff = 0U;
	uint32_t min = 0xFFFFFFFFU;
	uint32_t max = 0U;
	if (data_len > 3U) {
		/* break off both min and max */
		for (uint32_t i = 1U; i < data_len; i++) {
			capt[i - 1U] = capt[i] >= capt[i - 1U] ? capt[i] - capt[i - 1] : (CAPTURE_PERIOD
				       -capt[i - 1U] +capt[i]);
			diff += capt[i - 1U];
			max = (max < capt[i - 1U]) ? capt[i - 1U] : max;
			min = (min > capt[i - 1U]) ? capt[i - 1U] : min;
		}
		diff = (diff - min - max) / (data_len - 3U);
	}
	return diff;
}

#else
static uint32_t hc32_get_period(const struct pwm_hc32_config *cfg,
				uint32_t first_capture, uint32_t second_capture, uint32_t over_num,
				uint32_t under_num)
{
	uint32_t period = 0U;
	switch (cfg->countermode) {
	case HC32_TMR_CNT_MD_SAWTOOTH:
		if (cfg->direction) {
			/* direction =1, down */
			period = under_num * (CAPTURE_PERIOD + 1U) + first_capture - second_capture;
		} else {
			period = over_num * (CAPTURE_PERIOD + 1U) + second_capture - first_capture;
		}
		break;

	case HC32_TMR_CNT_MD_TRIANGLE:
		if ((over_num == 0U) && (under_num == 0U)) {
			return (first_capture + second_capture - CAPTURE_PERIOD);
		}
		period = first_capture + second_capture +
			 (over_num + under_num - 1) * CAPTURE_PERIOD;
		break;
	}

	return period;
}
#endif /* CONFIG_HC32_TIMERA_CAPTURE_FREQ_HIGH */

static int pwm_hc32_configure_capture(const struct device *dev,
				      uint32_t channel, pwm_flags_t flags,
				      pwm_capture_callback_handler_t cb,
				      void *user_data)
{
	const struct pwm_hc32_config *cfg = dev->config;
	struct pwm_hc32_data *data = dev->data;
	struct pwm_hc32_capture_data *cpt = &data->capture;

	if (channel < 1U || channel > TIMER_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	/* check if timera has start */
	if (cfg->timera->BCSTRL & TMRA_BCSTRL_START) {
		LOG_ERR("Cannot set capture, the timera unit is in progress");
		return -EBUSY;
	}

	if (flags & PWM_CAPTURE_TYPE_PULSE) {
		LOG_ERR("Timera don't support pulse capture");
		return -ENOTSUP;
	}
	cpt->callback = cb;
	cpt->user_data = user_data;
	cpt->capture_period = (flags & PWM_CAPTURE_TYPE_PERIOD) ? true : false;
	cpt->continuous = (flags & PWM_CAPTURE_MODE_CONTINUOUS) ? true : false;
	cpt->channel = channel;

	TMRA_HWCaptureCondCmd(cfg->timera, channel - 1, TMRA_CAPT_COND_PWM_RISING,
			      ENABLE);
	TMRA_SetPeriodValue(cfg->timera, CAPTURE_PERIOD);
	TMRA_SetFunc(cfg->timera, channel - 1, TMRA_FUNC_CAPT);
	return 0;
}

static int pwm_hc32_enable_capture(const struct device *dev,
				   uint32_t channel)
{
	const struct pwm_hc32_config *cfg = dev->config;
	struct pwm_hc32_data *data = dev->data;
	struct pwm_hc32_capture_data *cpt = &data->capture;
	uint32_t u32IntType = TMRA_INT_CMP_CH1;

	if (channel < 1U || channel > TIMER_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	if ((!data->capture.callback) || (channel != cpt->channel)) {
		LOG_ERR("PWM capture not configured or wrong channel");
		return -EINVAL;
	}

	/* check if timera has start */
	if (cfg->timera->BCSTRL & TMRA_BCSTRL_START) {
		LOG_ERR("The timera unit is in progress, PWM capture already active");
		return -EBUSY;
	}

	if (!cpt->capture_period) {
		LOG_ERR("Only support period capture");
		return -EINVAL;
	}
#ifdef CONFIG_HC32_TIMERA_CAPTURE_FREQ_HIGH
	/* keep original BCSTRL and set to SAWTOOTH, dir up */
	cpt->bcstrl_value = cfg->timera->BCSTRL;
	TMRA_SetCountValue(cfg->timera, 0U);
	TMRA_SetCountDir(cfg->timera, TMRA_DIR_UP);
	TMRA_SetCountMode(cfg->timera, TMRA_MD_SAWTOOTH);
#else
	if (!cfg->direction) {
		/* count direction is up */
		TMRA_SetCountValue(cfg->timera, 0U);
	} else {
		if (HC32_TMR_CNT_MD_SAWTOOTH == cfg->countermode) {
			TMRA_SetCountValue(cfg->timera, CAPTURE_PERIOD);
		}
	}

	if (HC32_TMR_CNT_MD_TRIANGLE == cfg->countermode) {
		TMRA_IntCmd(cfg->timera, TMRA_INT_UDF | TMRA_INT_OVF, ENABLE);
	} else {
		if (cfg->direction) {
			TMRA_IntCmd(cfg->timera, TMRA_INT_UDF, ENABLE);
		} else {
			TMRA_IntCmd(cfg->timera, TMRA_INT_OVF, ENABLE);
		}
	}
#endif
	data->capture.state = CAPTURE_STATE_WAIT_FOR_PERIOD_START;
	data->capture.overflows = 0U;
	data->capture.underflows = 0U;

	u32IntType <<= (channel - 1);
	TMRA_IntCmd(cfg->timera, u32IntType, ENABLE);
	TMRA_ClearStatus(cfg->timera, TMRA_FLAG_ALL);
	TMRA_Start(cfg->timera);
	return 0;
}

static int pwm_hc32_disable_capture(const struct device *dev,
				    uint32_t channel)
{
	const struct pwm_hc32_config *cfg = dev->config;
#ifdef CONFIG_HC32_TIMERA_CAPTURE_FREQ_HIGH
	struct pwm_hc32_data *data = dev->data;
	struct pwm_hc32_capture_data *cpt = &data->capture;
#endif
	uint32_t u32IntType = TMRA_INT_CMP_CH1;

	if (channel < 1u || channel > TIMER_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	u32IntType <<= (channel - 1U);

	TMRA_IntCmd(cfg->timera, u32IntType | TMRA_INT_OVF | TMRA_INT_UDF, DISABLE);
#ifdef CONFIG_HC32_TIMERA_CAPTURE_FREQ_HIGH
	cfg->timera->BCSTRL = cpt->bcstrl_value;
#endif
	TMRA_Stop(cfg->timera);
	TMRA_HWCaptureCondCmd(cfg->timera, channel - 1U,
			      TMRA_CAPT_COND_PWM_RISING | TMRA_CAPT_COND_PWM_FALLING, DISABLE);
	TMRA_ClearStatus(cfg->timera, TMRA_FLAG_ALL);

	return 0;
}

static void pwm_hc32_isr(const struct device *dev)
{
	const struct pwm_hc32_config *cfg = dev->config;
	struct pwm_hc32_data *data = dev->data;
	struct pwm_hc32_capture_data *cpt = &data->capture;
	int status = 0u;

#ifdef CONFIG_HC32_TIMERA_CAPTURE_FREQ_HIGH
	uint32_t u32IntType = TMRA_INT_CMP_CH1;
	static uint32_t capture_data[6U], i = 0U;

	if (cpt->state == CAPTURE_STATE_WAIT_FOR_PERIOD_START) {
		i = 0U;
		cpt->state = CAPTURE_STATE_IDLE;
	}
	CLR_REG16_BIT(cfg->timera->STFLR, 1 << (cpt->channel - 1));
	capture_data[i++] = TMRA_GetCompareValue(cfg->timera, cpt->channel - 1);

	if (i == 6U) {
		u32IntType <<= (cpt->channel - 1);
		TMRA_IntCmd(cfg->timera, u32IntType, DISABLE);
		i = 0U;
		cpt->period = hc32_get_period(capture_data, 6U);
		cpt->callback(dev, cpt->channel, cpt->period, 0U, status, cpt->user_data);
		if (!cpt->continuous) {
			pwm_hc32_disable_capture(dev, cpt->channel);
		} else {
			TMRA_IntCmd(cfg->timera, u32IntType, ENABLE);
			TMRA_ClearStatus(cfg->timera, TMRA_FLAG_ALL);
		}
	}

#else
	static uint32_t period_overnum, period_undernum, first_capture, last_capture;
	if (SET == TMRA_GetStatus(cfg->timera, TMRA_FLAG_OVF)) {
		cpt->overflows++;
		TMRA_ClearStatus(cfg->timera, TMRA_FLAG_OVF);
	}

	if (SET == TMRA_GetStatus(cfg->timera, TMRA_FLAG_UDF)) {
		cpt->underflows++;
		TMRA_ClearStatus(cfg->timera, TMRA_FLAG_UDF);
	}

	/* if capture a rsing */
	if (READ_REG16_BIT(cfg->timera->STFLR, 1 << (cpt->channel - 1))) {
		if (cpt->state == CAPTURE_STATE_WAIT_FOR_PERIOD_START) {
			period_overnum = cpt->overflows;
			period_undernum = cpt->underflows;
			first_capture = TMRA_GetCompareValue(cfg->timera, cpt->channel - 1);
			cpt->state = CAPTURE_STATE_WAIT_FOR_PERIOD_END;
			if (HC32_TMR_CNT_MD_TRIANGLE == cfg->countermode) {
				if (cfg->timera->BCSTRL & TMRA_BCSTRL_DIR) {
					first_capture = CAPTURE_PERIOD - first_capture;
				}
			}

		} else if (cpt->state == CAPTURE_STATE_WAIT_FOR_PERIOD_END) {
			period_overnum = cpt->overflows - period_overnum;
			period_undernum = cpt->underflows - period_undernum;
			last_capture = TMRA_GetCompareValue(cfg->timera, cpt->channel - 1U);
			cpt->state = CAPTURE_STATE_IDLE;
			if (HC32_TMR_CNT_MD_TRIANGLE == cfg->countermode) {
				if ((cfg->timera->BCSTRL & TMRA_BCSTRL_DIR) == 0U) {
					/* underflows after last capture, so laster capture vaule need update */
					last_capture = CAPTURE_PERIOD - last_capture;
				}
			}
		}
		/* clear flag */
		CLR_REG16_BIT(cfg->timera->STFLR, 1U << (cpt->channel - 1U));
	}

	if (CAPTURE_STATE_IDLE == cpt->state) {
		cpt->period = hc32_get_period(cfg, first_capture,
					      last_capture, period_overnum,
					      period_undernum);
		if (!cpt->continuous) {
			pwm_hc32_disable_capture(dev, cpt->channel);
			cpt->state = CAPTURE_STATE_IDLE;
		} else {
			cpt->state = CAPTURE_STATE_WAIT_FOR_PERIOD_START;
		}
		cpt->callback(dev, cpt->channel, cpt->period, 0U, status, cpt->user_data);
		cpt->overflows = 0U;
		cpt->underflows = 0U;
		TMRA_ClearStatus(cfg->timera, TMRA_FLAG_ALL);
	}
#endif
}
#endif /* CONFIG_PWM_CAPTURE */

static int pwm_hc32_get_cycles_per_sec(const struct device *dev,
				       uint32_t channel, uint64_t *cycles)
{
	struct pwm_hc32_data *data = dev->data;
	const struct pwm_hc32_config *cfg = dev->config;
	uint32_t bus_clk = data->tim_clk;

	for (uint8_t i = 0U; i < cfg->prescaler; i++) {
		bus_clk >>= 1U;
	}
	*cycles = (uint64_t)bus_clk;

	return 0;
}

static const struct pwm_driver_api pwm_hc32_driver_api = {
	.set_cycles = pwm_hc32_set_cycles,
	.get_cycles_per_sec = pwm_hc32_get_cycles_per_sec,
#ifdef CONFIG_PWM_CAPTURE
	.configure_capture = pwm_hc32_configure_capture,
	.enable_capture = pwm_hc32_enable_capture,
	.disable_capture = pwm_hc32_disable_capture,
#endif /* CONFIG_PWM_CAPTURE */
};

static int pwm_hc32_init(const struct device *dev)
{
	const struct pwm_hc32_config *cfg = dev->config;
	struct pwm_hc32_data *data = dev->data;
	stc_tmra_init_t stcTmraInit;
	int ret;

	/* TIMERA clock open */
	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(clk_sys)),
			       (clock_control_subsys_t)&cfg->clk_sys);

	if (ret < 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return ret;
	}

	/* configure pinmux */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("PWM pinctrl setup failed (%d)", ret);
		return ret;
	}

	TMRA_DeInit(cfg->timera);
	/* initialize timer */
	(void)TMRA_StructInit(&stcTmraInit);
	stcTmraInit.sw_count.u8ClockDiv  = cfg->prescaler;
	stcTmraInit.sw_count.u8CountMode = (cfg->countermode) ? TMRA_MD_TRIANGLE :
					   TMRA_MD_SAWTOOTH;;
	if (!(cfg->countermode)) {
		/* Only SAWTOOTH mode can set dir */
		stcTmraInit.sw_count.u8CountDir  = cfg->direction ? TMRA_DIR_DOWN : TMRA_DIR_UP;
	}
	stcTmraInit.u32PeriodValue = 0xFFFFU;

	ret = TMRA_Init(cfg->timera, &stcTmraInit);
	if (ret) {
		LOG_ERR("TMRA_Init error (%d)", ret);
		return -EIO;
	}

	ret = clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(clk_sys)),
				     (clock_control_subsys_t)&cfg->clk_sys, &(data->tim_clk));
	if (ret < 0) {
		LOG_ERR("Could not obtain timera clock (%d)", ret);
		return ret;
	}
#ifdef CONFIG_PWM_CAPTURE
	cfg->irq_config_func(dev);
#endif /* CONFIG_PWM_CAPTURE */
	return 0;
}

#define PWM(inst) DT_INST_PARENT(inst)

#ifdef CONFIG_PWM_CAPTURE
#define IDX_IRQ_CONFIGURE(idx, inst)                                                           \
		IRQ_CONNECT(DT_IRQ_BY_IDX(PWM(inst),idx, irq), DT_IRQ_BY_IDX(PWM(inst),idx, priority), \
			    pwm_hc32_isr, DEVICE_DT_INST_GET(inst), 0);                                    \
		hc32_intc_irq_signin(DT_IRQ_BY_IDX(PWM(inst),idx, irq),                                \
						 DT_IRQ_BY_IDX(PWM(inst),idx, int_src));                               \
		irq_enable(DT_IRQ_BY_IDX(PWM(inst),idx, irq));

#define CONFIGURE_ALL_IRQS(inst, idxs)  LISTIFY(idxs, IDX_IRQ_CONFIGURE, (), inst)

#define HC32_IRQ_CONFIGURE(inst)                                         \
	static void pwm_hc32_irq_config_func_##inst(const struct device *dev)\
	{                                                                    \
		CONFIGURE_ALL_IRQS(inst, DT_NUM_IRQS(PWM(inst)));                \
	}

#define CAPTURE_INIT(index)                        \
	.irq_config_func = pwm_hc32_irq_config_func_##index,

#endif /* CONFIG_PWM_CAPTURE */


#define PWM_DEVICE_INIT(index)                                         \
	static struct pwm_hc32_data pwm_hc32_data_##index = {		       \
	\
	};								                                   \
	PINCTRL_DT_INST_DEFINE(index);                                     \
	IF_ENABLED(CONFIG_PWM_CAPTURE, (HC32_IRQ_CONFIGURE(index)));       \
	static const struct pwm_hc32_config pwm_hc32_config_##index = {    \
		.timera = (CM_TMRA_TypeDef *)DT_REG_ADDR(PWM(index)),	       \
		.prescaler = DT_PROP(PWM(index), clock_division),		       \
		.countermode = DT_PROP(PWM(index), count_mode),	               \
		.direction = DT_PROP(PWM(index), count_direction),	           \
		.clk_sys = {                                                   \
		.bus = DT_CLOCKS_CELL(PWM(index), bus),                        \
		.fcg = DT_CLOCKS_CELL(PWM(index), fcg),                        \
		.bits = DT_CLOCKS_CELL(PWM(index), bits)                       \
		},\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),		           \
		IF_ENABLED(CONFIG_PWM_CAPTURE, (CAPTURE_INIT(index)))          \
	};                                                                 \
	DEVICE_DT_INST_DEFINE(index, &pwm_hc32_init, NULL,                 \
			    &pwm_hc32_data_##index,                                \
			    &pwm_hc32_config_##index, POST_KERNEL,                 \
			    CONFIG_PWM_INIT_PRIORITY,                              \
			    &pwm_hc32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT)
