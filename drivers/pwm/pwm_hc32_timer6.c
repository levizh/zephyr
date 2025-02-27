/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_timer6_pwm

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>
#include <zephyr/dt-bindings/timer/hc32-timer.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pwm.h>

#include <hc32_ll_tmr6.h>
#include <hc32_ll.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(pwm_hc32_timer6, CONFIG_PWM_LOG_LEVEL);

/* Timer6 max channel value for pwm, can't changed */
#define TIME6_PWM_MAX_CH                0x02U

#if defined(HC32F460)
#define TMR6_MD_TRIANGLE                TMR6_MD_TRIANGLE_A
#endif

#ifdef CONFIG_PWM_CAPTURE
/* Capture period or pulse data nums once time, max 0xFFU */
#define CAPTURE_NUMS_ONCE               0x02U
#if defined(HC32F460)
#define CAPTURE_PERIOD                  0x0000FFFFU
#elif defined(HC32F4A0)
#define CAPTURE_PERIOD                  0xFFFFFFFFU
#endif

/**
 * @brief Capture state when in 4-channel support mode
 */
typedef enum {
	CAPTURE_STATE_IDLE = 0U,
	CAPTURE_FOR_PERIOD = 1U,
	CAPTURE_FOR_PULSE = 2U,
	CAPTURE_FOR_END = 3U,
} tmr6_capture_state;

struct pwm_hc32_capture_data {
	pwm_capture_callback_handler_t callback;
	void *user_data;
	uint32_t period;
	uint32_t pulse;
	uint8_t capture_flag;
	bool continuous;
	uint8_t channel;
	uint32_t gconr_value;
	//tmr6_capture_state state;
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
	CM_TMR6_TypeDef *timer6;
	uint32_t prescaler;
	uint32_t countermode;
	uint32_t direction;
	struct hc32_modules_clock_sys clk_sys;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_PWM_CAPTURE
	void (*irq_config_func)(const struct device *dev);
#endif /* CONFIG_PWM_CAPTURE */
};

static int pwm_hc32_set_cycles(const struct device *dev, uint32_t channel,
			       uint32_t period_cycles, uint32_t pulse_cycles,
			       pwm_flags_t flags)
{
	const struct pwm_hc32_config *cfg = dev->config;
	static stc_tmr6_pwm_init_t stcPwmInit;
	uint32_t u32polarity_high, u32polarity_low;
	int ret;

	if (channel < 1U || channel > TIME6_PWM_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	/* The period_cycles cannot be greater than UINT16_MAX + 1, except U1-U4 in hc32f4a0 */
	if (period_cycles > UINT16_MAX + 1) {
#if defined(HC32F4A0)
		if (cfg->timer6 > CM_TMR6_4) {
			return -ENOTSUP;
		}
#else
		return -ENOTSUP;
#endif
	}

	if (period_cycles == 0U) {
		LOG_WRN("Set period_cycles = 0, stop timer6");
		TMR6_Stop(cfg->timer6);
		TMR6_PWM_OutputCmd(cfg->timer6, channel - 1U, DISABLE);
		return 0;
	}

#if defined(HC32F460) || defined(HC32F4A0)
	if (cfg->prescaler != 0U) {
		LOG_ERR("Cannot set PWM start level for clock-division isn't 1U");
		return -ENOTSUP;
	}
#endif
	TMR6_PWM_StructInit(&stcPwmInit);
	TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_UDF | TMR6_FLAG_OVF);
	if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
		u32polarity_high = TMR6_PWM_HIGH;
		u32polarity_low = TMR6_PWM_LOW;
	} else {
		u32polarity_high = TMR6_PWM_LOW;
		u32polarity_low = TMR6_PWM_HIGH;
	}
#if defined(HC32F460)
	if (HC32_TMR_CNT_MD_SAWTOOTH == cfg->countermode) {/* TMR6_MD_SAWTOOTH */
		stcPwmInit.u32StartPolarity = u32polarity_high;
		stcPwmInit.u32CompareMatchPolarity = u32polarity_low;
		stcPwmInit.u32PeriodMatchPolarity = u32polarity_high;
		stcPwmInit.u32StopPolarity = u32polarity_low;
		if (pulse_cycles == 0U) {
			stcPwmInit.u32StartPolarity = u32polarity_low;
			stcPwmInit.u32CompareMatchPolarity = u32polarity_low;
			stcPwmInit.u32PeriodMatchPolarity = u32polarity_low;
			TMR6_Stop(cfg->timer6);
		} else if (pulse_cycles == period_cycles) {
			stcPwmInit.u32CompareMatchPolarity = u32polarity_high;
		}

		/* DIR down = 1 */
		if (cfg->direction) {
			pulse_cycles = period_cycles - pulse_cycles;
		}
		/* 0% or 100% do not to update pulse */
		if ((pulse_cycles != 0U) && (pulse_cycles != period_cycles)) {
			stcPwmInit.u32CompareValue = pulse_cycles;
		}

		TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_UDF | TMR6_FLAG_OVF);
		if (cfg->timer6->GCONR & TMR6_GCONR_START) {
			/* timer6 has run */
			if (cfg->direction) {
				/* wait for underflows to update next set */
				while (RESET == TMR6_GetStatus(cfg->timer6, TMR6_FLAG_UDF)) {
					/* NOP */
				}
			} else {
				while (RESET == TMR6_GetStatus(cfg->timer6, TMR6_FLAG_OVF)) {
					/* up direction, then wait for overflows flag */
				}
			}
		} else {
			if (cfg->direction) {
				TMR6_SetCountValue(cfg->timer6, (period_cycles - 1U));
			} else {
				TMR6_SetCountValue(cfg->timer6, 0U);
			}
		}

		TMR6_SetPeriodValue(cfg->timer6, TMR6_PERIOD_REG_A, (period_cycles - 1U));
		ret = TMR6_PWM_Init(cfg->timer6, channel - 1U, &stcPwmInit);
		if (ret) {
			LOG_ERR("Could not initialize timer6 channel for pwm");
			return -EIO;
		}
	} else {
		/* TMR6_MD_TRIANGLE */
		stcPwmInit.u32StartPolarity = u32polarity_low;
		stcPwmInit.u32StopPolarity = u32polarity_low;
		stcPwmInit.u32CompareMatchPolarity = TMR6_PWM_INVT;
		stcPwmInit.u32PeriodMatchPolarity = TMR6_PWM_HOLD;

		if (pulse_cycles == 0U) {
			stcPwmInit.u32CompareMatchPolarity = u32polarity_low;
		} else if (pulse_cycles == period_cycles) {
			stcPwmInit.u32StartPolarity = u32polarity_high;
			stcPwmInit.u32CompareMatchPolarity = u32polarity_high;
		}

		stcPwmInit.u32CompareValue = (period_cycles - pulse_cycles) / 2U;

		/* check if timer6 has start */
		if (cfg->timer6->GCONR & TMR6_GCONR_START) {
			TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_UDF);
			/* wait for underflows to update next set */
			while (RESET == TMR6_GetStatus(cfg->timer6, TMR6_FLAG_UDF)) {
				/* NOP */
			}
			/* for TRIANGLE mode must stop timer6 to change the polarity and start level */
			TMR6_Stop(cfg->timer6);
		} else {
			TMR6_SetCountValue(cfg->timer6, stcPwmInit.u32CompareValue);
		}
		TMR6_SetPeriodValue(cfg->timer6, TMR6_PERIOD_REG_A, period_cycles / 2U);
		ret = TMR6_PWM_Init(cfg->timer6, channel - 1U, &stcPwmInit);
		if (ret) {
			LOG_ERR("Could not initialize timer6 channel for pwm");
			return -EIO;
		}
	}
#elif defined(HC32F4A0)
	stcPwmInit.u32CountUpMatchAPolarity = TMR6_PWM_HOLD;
	stcPwmInit.u32CountDownMatchAPolarity = TMR6_PWM_HOLD;
	stcPwmInit.u32CountUpMatchBPolarity = TMR6_PWM_HOLD;
	stcPwmInit.u32CountDownMatchBPolarity = TMR6_PWM_HOLD;
	stcPwmInit.u32UdfPolarity = TMR6_PWM_HOLD;
	stcPwmInit.u32OvfPolarity = TMR6_PWM_HOLD;

	if (HC32_TMR_CNT_MD_SAWTOOTH == cfg->countermode) {/* TMR6_MD_SAWTOOTH */
		stcPwmInit.u32StartPolarity  = u32polarity_high;
		stcPwmInit.u32StopPolarity   = u32polarity_low;
		if (cfg->direction) {
			/* count down */
			stcPwmInit.u32CountDownMatchAPolarity = u32polarity_low;
			stcPwmInit.u32CountDownMatchBPolarity = u32polarity_low;
			stcPwmInit.u32UdfPolarity = u32polarity_high;

		} else { /* count up */
			stcPwmInit.u32CountUpMatchAPolarity = u32polarity_low;
			stcPwmInit.u32CountUpMatchBPolarity = u32polarity_low;
			stcPwmInit.u32OvfPolarity = u32polarity_high;
		}

		if (pulse_cycles == 0U) {
			stcPwmInit.u32StartPolarity = u32polarity_low;
			stcPwmInit.u32CountDownMatchAPolarity = u32polarity_low;
			stcPwmInit.u32CountUpMatchAPolarity = u32polarity_low;
			stcPwmInit.u32CountDownMatchBPolarity = u32polarity_low;
			stcPwmInit.u32CountUpMatchBPolarity = u32polarity_low;
			stcPwmInit.u32UdfPolarity = u32polarity_low;
			stcPwmInit.u32OvfPolarity = u32polarity_low;
		} else if (pulse_cycles == period_cycles) {
			stcPwmInit.u32CountDownMatchAPolarity = u32polarity_high;
			stcPwmInit.u32CountUpMatchAPolarity = u32polarity_high;
			stcPwmInit.u32CountDownMatchBPolarity = u32polarity_high;
			stcPwmInit.u32CountUpMatchBPolarity = u32polarity_high;
		}

		/* DIR down = 1 */
		if (cfg->direction) {
			stcPwmInit.u32CompareValue = period_cycles - pulse_cycles;
		} else {
			stcPwmInit.u32CompareValue = pulse_cycles;
		}

		TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_UDF | TMR6_FLAG_OVF);
		if (cfg->direction) {
			if (cfg->timer6->GCONR & TMR6_GCONR_START) {
				/* timer6 has run */
				/* Dir down, wait for underflows to update next set */
				while (RESET == TMR6_GetStatus(cfg->timer6, TMR6_FLAG_UDF)) {
					/* NOP */
				}
			} else {
				/* Dir down need to set count (period_cycles - 1U) */
				TMR6_SetCountValue(cfg->timer6, (period_cycles - 1U));
			}
		} else {
			if (cfg->timer6->GCONR & TMR6_GCONR_START) {
				while (RESET == TMR6_GetStatus(cfg->timer6, TMR6_FLAG_OVF)) {
					/* up direction, then wait for overflows flag */
				}
			} else {
				TMR6_SetCountValue(cfg->timer6, 0U);
			}
		}
		if (pulse_cycles == 0U) {
			TMR6_Stop(cfg->timer6);
		}
		TMR6_SetPeriodValue(cfg->timer6, TMR6_PERIOD_REG_A, (period_cycles - 1U));
		ret = TMR6_PWM_Init(cfg->timer6, channel - 1U, &stcPwmInit);
		if (ret) {
			LOG_ERR("Could not initialize timer6 channel for pwm");
			return -EIO;
		}
		TMR6_SetCompareValue(cfg->timer6, TMR6_CMP_REG_A, stcPwmInit.u32CompareValue);
	} else {
		/* TMR6_MD_TRIANGLE */
		stcPwmInit.u32StartPolarity = u32polarity_high;
		stcPwmInit.u32StopPolarity = u32polarity_low;
		stcPwmInit.u32UdfPolarity = u32polarity_high;
		stcPwmInit.u32OvfPolarity = u32polarity_high;
		stcPwmInit.u32CountUpMatchAPolarity = u32polarity_low;
		stcPwmInit.u32CountDownMatchBPolarity = u32polarity_low;

		if (pulse_cycles == 0U) {
			stcPwmInit.u32StartPolarity = u32polarity_low;
			stcPwmInit.u32UdfPolarity = u32polarity_low;
			stcPwmInit.u32OvfPolarity = u32polarity_low;
		} else if (pulse_cycles == period_cycles) {
			stcPwmInit.u32CountUpMatchAPolarity = u32polarity_high;
			stcPwmInit.u32CountDownMatchBPolarity = u32polarity_high;
		}

		/* check if timer6 has start */
		if (cfg->timer6->GCONR & TMR6_GCONR_START) {
			/* wait for underflows to update next set */
			TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_UDF);
			while (RESET == TMR6_GetStatus(cfg->timer6, TMR6_FLAG_UDF)) {
				/* NOP */
			}
			if (pulse_cycles == 0U) {
				TMR6_Stop(cfg->timer6);
				TMR6_SetCountValue(cfg->timer6, 1U);
			}
		} else {
			TMR6_SetCountValue(cfg->timer6, 1U);
		}
		TMR6_SetPeriodValue(cfg->timer6, TMR6_PERIOD_REG_A, period_cycles);
		ret = TMR6_PWM_Init(cfg->timer6, channel - 1U, &stcPwmInit);
		if (ret) {
			LOG_ERR("Could not initialize timer6 channel for pwm");
			return -EIO;
		}
		TMR6_SetCompareValue(cfg->timer6, TMR6_CMP_REG_A, pulse_cycles);
		TMR6_SetCompareValue(cfg->timer6, TMR6_CMP_REG_B, period_cycles - pulse_cycles);
	}
#endif
	TMR6_SetFunc(cfg->timer6, channel - 1U, TMR6_PIN_CMP_OUTPUT);
	TMR6_PWM_OutputCmd(cfg->timer6, channel - 1U, ENABLE);
	TMR6_Start(cfg->timer6);
	return 0;
}

#ifdef CONFIG_PWM_CAPTURE
static uint32_t hc32_get_average_value(uint32_t *capt, uint8_t data_len)
{
	uint64_t diff = 0U;
	for (uint8_t i = 0U; i < data_len; i++) {
		diff += *capt++;
	}
	return (uint32_t)(diff / data_len);
}

static int pwm_hc32_configure_capture(const struct device *dev,
				      uint32_t channel, pwm_flags_t flags,
				      pwm_capture_callback_handler_t cb,
				      void *user_data)
{
	const struct pwm_hc32_config *cfg = dev->config;
	struct pwm_hc32_data *data = dev->data;
	struct pwm_hc32_capture_data *cpt = &data->capture;
	uint32_t cond_rising = 0U, cond_falling = 0U;
	uint32_t cond_rising_clr = 0U, cond_falling_clr = 0U;
	uint32_t cond_start_rising = 0U, cond_start_falling = 0U;

	if (channel < 1U || channel >= CHANNEL_CAPTURE_TMR6_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	/* check if timer6 has start */
	if (cfg->timer6->HSTAR & TMR6_HSTAR_STAS) {
		LOG_ERR("Cannot set capture, the timer6 unit is in progress");
		return -EBUSY;
	}

	if (0U == (flags & PWM_CAPTURE_TYPE_MASK)) {
		LOG_ERR("No period or pulse set to capture!");
		return -EINVAL;
	}

	cpt->callback = cb;
	cpt->user_data = user_data;
	cpt->capture_flag = (flags & PWM_CAPTURE_TYPE_MASK);
	cpt->continuous = (flags & PWM_CAPTURE_MODE_CONTINUOUS) ? true : false;
	cpt->channel = channel;

	switch (channel) {
	case CHANNEL_CAPTURE_TMR6_PWMA:
		cond_rising = TMR6_CAPT_COND_PWMA_RISING;
		cond_falling = TMR6_CAPT_COND_PWMA_FALLING;
		cond_rising_clr = TMR6_CLR_COND_PWMA_RISING;
		cond_falling_clr = TMR6_CLR_COND_PWMA_FALLING;
		cond_start_rising = TMR6_START_COND_PWMA_RISING;
		cond_start_falling = TMR6_START_COND_PWMA_FALLING;
		break;

	case CHANNEL_CAPTURE_TMR6_PWMB:
		cond_rising = TMR6_CAPT_COND_PWMB_RISING;
		cond_falling = TMR6_CAPT_COND_PWMB_FALLING;
		cond_rising_clr = TMR6_CLR_COND_PWMB_RISING;
		cond_falling_clr = TMR6_CLR_COND_PWMB_FALLING;
		cond_start_rising = TMR6_START_COND_PWMB_RISING;
		cond_start_falling = TMR6_START_COND_PWMB_FALLING;
		break;

	case CHANNEL_CAPTURE_TMR6_TRIGA:
		cond_rising = TMR6_CAPT_COND_TRIGA_RISING;
		cond_falling = TMR6_CAPT_COND_TRIGA_FALLING;
		cond_rising_clr = TMR6_CLR_COND_TRIGA_RISING;
		cond_falling_clr = TMR6_CLR_COND_TRIGA_FALLING;
		cond_start_rising = TMR6_START_COND_TRIGA_RISING;
		cond_start_falling = TMR6_START_COND_TRIGA_FALLING;
		break;

	case CHANNEL_CAPTURE_TMR6_TRIGB:
		cond_rising = TMR6_CAPT_COND_TRIGB_RISING;
		cond_falling = TMR6_CAPT_COND_TRIGB_FALLING;
		cond_rising_clr = TMR6_CLR_COND_TRIGB_RISING;
		cond_falling_clr = TMR6_CLR_COND_TRIGB_FALLING;
		cond_start_rising = TMR6_START_COND_TRIGB_RISING;
		cond_start_falling = TMR6_START_COND_TRIGB_FALLING;
		break;
#if defined(HC32F4A0)
	case CHANNEL_CAPTURE_TMR6_TRIGC:
		cond_rising = TMR6_CAPT_COND_TRIGC_RISING;
		cond_falling = TMR6_CAPT_COND_TRIGC_FALLING;
		cond_rising_clr = TMR6_CLR_COND_TRIGC_RISING;
		cond_falling_clr = TMR6_CLR_COND_TRIGC_FALLING;
		cond_start_rising = TMR6_START_COND_TRIGC_RISING;
		cond_start_falling = TMR6_START_COND_TRIGC_FALLING;
		break;

	case CHANNEL_CAPTURE_TMR6_TRIGD:
		cond_rising = TMR6_CAPT_COND_TRIGD_RISING;
		cond_falling = TMR6_CAPT_COND_TRIGD_FALLING;
		cond_rising_clr = TMR6_CLR_COND_TRIGD_RISING;
		cond_falling_clr = TMR6_CLR_COND_TRIGD_FALLING;
		cond_start_rising = TMR6_START_COND_TRIGD_RISING;
		cond_start_falling = TMR6_START_COND_TRIGD_FALLING;
		break;
#endif
	}

	if (flags & PWM_CAPTURE_TYPE_PERIOD) {
		/* rising to clear and capture */
		if (flags & PWM_POLARITY_MASK) {
			TMR6_HWCaptureCondCmd(cfg->timer6, TMR6_CH_A, cond_falling, ENABLE);
			TMR6_HWClearCondCmd(cfg->timer6, cond_falling_clr, ENABLE);
			TMR6_HWStartCondCmd(cfg->timer6, cond_start_falling, ENABLE);
		} else {
			TMR6_HWCaptureCondCmd(cfg->timer6, TMR6_CH_A, cond_rising, ENABLE);
			TMR6_HWClearCondCmd(cfg->timer6, cond_rising_clr, ENABLE);
			TMR6_HWStartCondCmd(cfg->timer6, cond_start_rising, ENABLE);
		}
	}
	if (flags & PWM_CAPTURE_TYPE_PULSE) {
		if (flags & PWM_POLARITY_MASK) {
			TMR6_HWClearCondCmd(cfg->timer6, cond_falling_clr, ENABLE);
			TMR6_HWStartCondCmd(cfg->timer6, cond_start_falling, ENABLE);
			TMR6_HWCaptureCondCmd(cfg->timer6, TMR6_CH_B, cond_rising, ENABLE);
		} else {
			TMR6_HWClearCondCmd(cfg->timer6, cond_rising_clr, ENABLE);
			TMR6_HWStartCondCmd(cfg->timer6, cond_start_rising, ENABLE);
			TMR6_HWCaptureCondCmd(cfg->timer6, TMR6_CH_B, cond_falling, ENABLE);
		}
	}
#if defined(HC32F4A0)
	if (cfg->timer6 > CM_TMR6_4) {
		TMR6_SetPeriodValue(cfg->timer6, TMR6_PERIOD_REG_A,
				    CAPTURE_PERIOD & 0x0000ffffU);
	} else
#endif
	{
		TMR6_SetPeriodValue(cfg->timer6, TMR6_PERIOD_REG_A, CAPTURE_PERIOD);
	}
	TMR6_SetFunc(cfg->timer6, TMR6_CH_A, TMR6_PIN_CAPT_INPUT);
	TMR6_SetFunc(cfg->timer6, TMR6_CH_B, TMR6_PIN_CAPT_INPUT);
	return 0;
}

static int pwm_hc32_enable_capture(const struct device *dev,
				   uint32_t channel)
{
	const struct pwm_hc32_config *cfg = dev->config;
	struct pwm_hc32_data *data = dev->data;
	struct pwm_hc32_capture_data *cpt = &data->capture;

	if (channel < 1U || channel >= CHANNEL_CAPTURE_TMR6_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	if ((!data->capture.callback) || (channel != cpt->channel)) {
		LOG_ERR("PWM capture not configured or wrong channel");
		return -EINVAL;
	}

	/* check if timer6 has start */
	if (cfg->timer6->HSTAR & TMR6_HSTAR_STAS) {
		LOG_ERR("The timer6 unit is in progress, PWM capture already active");
		return -EBUSY;
	}

	/* Store original GCONR and set to SAWTOOTH, dir up */
	cpt->gconr_value = cfg->timer6->GCONR;
	TMR6_SetCountValue(cfg->timer6, 0U);
	TMR6_SetCountDir(cfg->timer6, TMR6_CNT_UP);
	TMR6_SetCountMode(cfg->timer6, TMR6_MD_SAWTOOTH);

	TMR6_HWClearCmd(cfg->timer6, ENABLE);
	TMR6_HWStartCmd(cfg->timer6, ENABLE);
	TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_CLR_ALL);
	TMR6_IntCmd(cfg->timer6, TMR6_INT_MATCH_A | TMR6_INT_MATCH_B, ENABLE);
	return 0;
}

static int pwm_hc32_disable_capture(const struct device *dev,
				    uint32_t channel)
{
	const struct pwm_hc32_config *cfg = dev->config;
	struct pwm_hc32_data *data = dev->data;
	struct pwm_hc32_capture_data *cpt = &data->capture;

	if (channel < 1U || channel >= CHANNEL_CAPTURE_TMR6_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}
	if (channel != cpt->channel) {
		LOG_ERR("Running channel isn't this channel!");
		return -EINVAL;
	}

	TMR6_IntCmd(cfg->timer6, TMR6_INT_ALL, DISABLE);
	TMR6_HWClearCmd(cfg->timer6, DISABLE);
	TMR6_HWStartCmd(cfg->timer6, DISABLE);
	cfg->timer6->GCONR = cpt->gconr_value;
	TMR6_Stop(cfg->timer6);
	TMR6_HWCaptureCondCmd(cfg->timer6, TMR6_CH_A, TMR6_CAPT_COND_ALL,
			      DISABLE);
	TMR6_HWCaptureCondCmd(cfg->timer6, TMR6_CH_B, TMR6_CAPT_COND_ALL,
			      DISABLE);
	TMR6_HWClearCondCmd(cfg->timer6, TMR6_CLR_COND_ALL, DISABLE);
	TMR6_HWStartCondCmd(cfg->timer6, TMR6_START_COND_ALL, DISABLE);
	TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_CLR_ALL);
	return 0;
}

static void pwm_hc32_isr(const struct device *dev)
{
	const struct pwm_hc32_config *cfg = dev->config;
	struct pwm_hc32_data *data = dev->data;
	struct pwm_hc32_capture_data *cpt = &data->capture;
	static uint8_t status = 0U;
	static uint32_t period_data[CAPTURE_NUMS_ONCE + 1U],
	       pulse_data[CAPTURE_NUMS_ONCE+1U];
	static uint8_t  i = 0U, j = 0U;

	if ((SET == TMR6_GetStatus(cfg->timer6, TMR6_FLAG_MATCH_A))
	    && (cpt->capture_flag & PWM_CAPTURE_TYPE_PERIOD)) {
		period_data[i++] = TMR6_GetCompareValue(cfg->timer6, TMR6_CMP_REG_A);
		if (i > CAPTURE_NUMS_ONCE) {
			TMR6_IntCmd(cfg->timer6, TMR6_INT_MATCH_A, DISABLE);
			status |= PWM_CAPTURE_TYPE_PERIOD;
			i = 0U;
		}
		TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_MATCH_A);
	}

	if ((SET == TMR6_GetStatus(cfg->timer6, TMR6_FLAG_MATCH_B))
	    && (cpt->capture_flag & PWM_CAPTURE_TYPE_PULSE)) {
		pulse_data[j++] = TMR6_GetCompareValue(cfg->timer6, TMR6_CMP_REG_B);
		if (j > CAPTURE_NUMS_ONCE) {
			TMR6_IntCmd(cfg->timer6, TMR6_INT_MATCH_B, DISABLE);
			status |= PWM_CAPTURE_TYPE_PULSE;
			j = 0U;
		}
		TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_MATCH_B);
	}

	if (status == cpt->capture_flag) {
		/* done */
		status = 0U;
		i = 0U;
		j = 0U;
		cpt->period = hc32_get_average_value(&period_data[1U], CAPTURE_NUMS_ONCE);
		cpt->pulse = hc32_get_average_value(&pulse_data[1U], CAPTURE_NUMS_ONCE);
		cpt->callback(dev, cpt->channel, cpt->period, cpt->pulse, 0U, cpt->user_data);
		if (!cpt->continuous) {
			pwm_hc32_disable_capture(dev, cpt->channel);
		} else {
			TMR6_IntCmd(cfg->timer6, TMR6_INT_MATCH_A | TMR6_INT_MATCH_B, ENABLE);
			TMR6_ClearStatus(cfg->timer6, TMR6_FLAG_CLR_ALL);
		}
		memset(period_data, 0U, sizeof(period_data));
		memset(pulse_data, 0U, sizeof(pulse_data));
	}
}
#endif /* CONFIG_PWM_CAPTURE */

static int pwm_hc32_get_cycles_per_sec(const struct device *dev,
				       uint32_t channel, uint64_t *cycles)
{
	struct pwm_hc32_data *data = dev->data;
	const struct pwm_hc32_config *cfg = dev->config;
	uint32_t bus_clk = data->tim_clk;

	for (uint8_t i = 0U; i < cfg->prescaler; i++) {
		bus_clk >>= 1;
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
	stc_tmr6_init_t stcTmr6Init;
	int ret;

	/* TIMER6 clock open */
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

	TMR6_DeInit(cfg->timer6);
	/* initialize timer */
	(void)TMR6_StructInit(&stcTmr6Init);
	stcTmr6Init.sw_count.u32ClockDiv  = cfg->prescaler;
	stcTmr6Init.sw_count.u32CountMode = (cfg->countermode) ? TMR6_MD_TRIANGLE :
					    TMR6_MD_SAWTOOTH;
	if (!(cfg->countermode)) {
		/* Only SAWTOOTH mode can set dir */
		stcTmr6Init.sw_count.u32CountDir  = cfg->direction ? TMR6_CNT_DOWN :
						    TMR6_CNT_UP;
	}
	stcTmr6Init.u32PeriodValue = 0xFFFFU;
#if defined(HC32F4A0)
	if (cfg->timer6 <= CM_TMR6_4) {
		stcTmr6Init.u32PeriodValue = 0xFFFFFFFFU;
	}
#endif

	ret = TMR6_Init(cfg->timer6, &stcTmr6Init);
	if (ret) {
		LOG_ERR("TMR6_Init error (%d)", ret);
		return -EIO;
	}

	ret = clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(clk_sys)),
				     (clock_control_subsys_t)&cfg->clk_sys, &(data->tim_clk));
	if (ret < 0) {
		LOG_ERR("Could not obtain timer6 clock (%d)", ret);
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
		.timer6 = (CM_TMR6_TypeDef *)DT_REG_ADDR(PWM(index)),	       \
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
