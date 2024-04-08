/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <soc.h>
#include <pwm.h>
#include <device.h>
#include <kernel.h>
#include <init.h>

#include <clock_control/hc32_clock_control.h>
#include <hc32_ll.h>

#define LOG_LEVEL CONFIG_PWM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(pwm_hc32_timera);

/* Timer count mode values */
#define HC32_TMR_CNT_MD_SAWTOOTH        0x0000U
#define HC32_TMR_CNT_MD_TRIANGLE        0x0001U

/** PWM data. */
struct pwm_hc32_data {
	/** Timer clock (Hz). */
	uint32_t tim_clk;
};

/** PWM configuration. */
struct pwm_hc32_config {
	CM_TMRA_TypeDef *timera;
	uint32_t prescaler;
	uint32_t cnt_mode;
	uint32_t direction;
	struct hc32_modules_clock_sys pwm_clk;
};

/* Maximum number of timer pwms */
#if defined(HC32F460)
#define TIMER_MAX_CH 8u
#elif defined(HC32F4A0)
#define TIMER_MAX_CH 4u
#endif

/*
 * Set the period and pulse width for a PWM pin.
 *
 * Parameters
 * dev: Pointer to PWM device structure
 * pwm: PWM pwm to set
 * period_cycles: Period (in timer count)
 * pulse_cycles: Pulse width (in timer count).
 *
 * return 0, or negative errno code
 */
static int pwm_hc32_pin_set(struct device *dev, u32_t pwm,
			    u32_t period_cycles, u32_t pulse_cycles)
{
	const struct pwm_hc32_config *cfg = dev->config->config_info;
	static stc_tmra_pwm_init_t stcPwmInit;
	int ret;

	if (pwm < 1u || pwm > TIMER_MAX_CH) {
		LOG_ERR("Invalid pwm (%d)", pwm);
		return -EINVAL;
	}

	/* (16-bit), Thus period_cycles cannot be greater than UINT16_MAX + 1. */
	if (period_cycles > UINT16_MAX + 1) {
		return -ENOTSUP;
	}

	if (period_cycles == 0u) {
		LOG_WRN("Set period_cycles = 0");
		TMRA_PWM_OutputCmd(cfg->timera, pwm, DISABLE);
		TMRA_Stop(cfg->timera);
		return 0;
	}

#if defined(HC32F460)
	if (cfg->prescaler != 0) {
		LOG_ERR("Cannot set PWM start level for clock-division isn't 1U");
		return -ENOTSUP;
	}
#endif
	TMRA_ClearStatus(cfg->timera, TMRA_FLAG_UDF | TMRA_FLAG_OVF);
	if (HC32_TMR_CNT_MD_SAWTOOTH == cfg->cnt_mode) {/* TMRA_MD_SAWTOOTH */
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

		/* DIR down = 1 */
		if (cfg->direction) {
			pulse_cycles = period_cycles - pulse_cycles;
		}
		/* 0% or 100% do not to update pulse */
		if ((pulse_cycles != 0) && (pulse_cycles != period_cycles)) {
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
			TMRA_SetCountValue(cfg->timera, 0u);
		}
		TMRA_ClearStatus(cfg->timera, TMRA_FLAG_UDF | TMRA_FLAG_OVF);
		TMRA_SetPeriodValue(cfg->timera, (period_cycles - 1));
	} else {
		/* TMRA_MD_TRIANGLE */
		stcPwmInit.u16StartPolarity       = TMRA_PWM_LOW;
		stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_INVT;
		stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HOLD;
		stcPwmInit.u16StopPolarity        = TMRA_PWM_LOW;
		if (pulse_cycles == 0U) {
			stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
		} else if (pulse_cycles == period_cycles) {
			stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
		}

		/* 0% or 100% do not to update pulse */
		if ((pulse_cycles != 0) && (pulse_cycles != period_cycles)) {
			stcPwmInit.u32CompareValue = (period_cycles - pulse_cycles) / 2;
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
			TMRA_SetCountValue(cfg->timera, 0u);
		}
		TMRA_SetPeriodValue(cfg->timera, period_cycles / 2);
	}

	ret = TMRA_PWM_Init(cfg->timera, pwm - 1u, &stcPwmInit);
	if (ret) {
		LOG_ERR("Could not initialize timer pwm for pwm");
		return -EIO;
	}

	if ((cfg->timera->BCSTRL & TMRA_BCSTRL_START) == 0u) {
		TMRA_SetFunc(cfg->timera, pwm - 1u, TMRA_FUNC_CMP);
		TMRA_Start(cfg->timera);
		TMRA_PWM_OutputCmd(cfg->timera, pwm - 1u, ENABLE);
	}

	return 0;
}

/*
 * Get the clock rate (cycles per second) for a PWM pin.
 *
 * Parameters
 * dev: Pointer to PWM device structure
 * pwm: PWM port number
 * cycles: Pointer to the memory to store clock rate (cycles per second)
 *
 * return 0, or negative errno code
 */
static int pwm_hc32_get_cycles_per_sec(struct device *dev, u32_t pwm,
				       u64_t *cycles)
{
	const struct pwm_hc32_config *cfg = dev->config->config_info;
	struct pwm_hc32_data *data = dev->driver_data;
	uint32_t bus_clk = data->tim_clk;

	for (uint8_t i = 0U; i < cfg->prescaler; i++) {
		bus_clk >>= 1;
	}
	*cycles = (uint64_t)bus_clk;

	return 0;
}

static const struct pwm_driver_api pwm_hc32_drv_api_funcs = {
	.pin_set = pwm_hc32_pin_set,
	.get_cycles_per_sec = pwm_hc32_get_cycles_per_sec,
};

static int pwm_hc32_init(struct device *dev)
{
	const struct pwm_hc32_config *cfg = dev->config->config_info;
	struct pwm_hc32_data *data = dev->driver_data;
	struct device *clock;
	stc_tmra_init_t stcTmraInit;

	int ret;

	clock = device_get_binding(CLOCK_CONTROL);
	/* spi clock open */
	ret = clock_control_on(clock, (clock_control_subsys_t)&cfg->pwm_clk);
	if (ret < 0) {
		LOG_ERR("Could not initialize pwm clock (%d)", ret);
		return ret;
	}

	TMRA_DeInit(cfg->timera);
	/* initialize timer */
	(void)TMRA_StructInit(&stcTmraInit);
	stcTmraInit.sw_count.u8ClockDiv  = cfg->prescaler;
	stcTmraInit.sw_count.u8CountMode = (cfg->cnt_mode) ? TMRA_MD_TRIANGLE :
					   TMRA_MD_SAWTOOTH;;
#if (TMRA_MD == TMRA_MD_SAWTOOTH)
	stcTmraInit.sw_count.u8CountDir  = cfg->direction ? TMRA_DIR_DOWN : TMRA_DIR_UP;
#endif
	stcTmraInit.u32PeriodValue = 0xFFFFU;

	ret = TMRA_Init(cfg->timera, &stcTmraInit);
	if (ret) {
		LOG_ERR("TMRA_Init error (%d)", ret);
		return -EIO;
	}

	ret = clock_control_get_rate(clock, (clock_control_subsys_t)&cfg->pwm_clk,
				     &(data->tim_clk));
	if (ret < 0) {
		LOG_ERR("Could not obtain timera clock (%d)", ret);
		return ret;
	}
	return 0;
}

/* PWM DEVICE INIT */
#define PWM_DEVICE_INIT_HC32(idx, ADDR_PROB)			\
static struct pwm_hc32_data pwm_hc32_data_##idx = {              \
};\
static const struct pwm_hc32_config pwm_hc32_dev_cfg_##idx = {              \
		.timera =(CM_TMRA_TypeDef *)DT_XHSC_HC32_TIMERA_PWM_##idx##_ADDR,	\
		.prescaler = DT_XHSC_HC32_TIMERA_##ADDR_PROB##_CLOCK_DIVISION,      \
		.cnt_mode = DT_XHSC_HC32_TIMERA_##ADDR_PROB##_COUNT_MODE,           \
		.direction = DT_XHSC_HC32_TIMERA_##ADDR_PROB##_COUNT_DIRECTION,     \
		.pwm_clk = {\
			.bus = DT_XHSC_HC32_TIMERA_##ADDR_PROB##_CLOCK_BUS_0,           \
			.fcg = DT_XHSC_HC32_TIMERA_##ADDR_PROB##_CLOCK_FCG_0,     \
			.bits = DT_XHSC_HC32_TIMERA_##ADDR_PROB##_CLOCK_BITS_0, \
		},\
	};\
	DEVICE_AND_API_INIT(pwm_hc32_##idx,				  \
			    DT_XHSC_HC32_TIMERA_PWM_##idx##_LABEL,  \
			    pwm_hc32_init,				  \
			    &pwm_hc32_data_##idx,			  \
			    &pwm_hc32_dev_cfg_##idx,			  \
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
			    &pwm_hc32_drv_api_funcs);

/* 16-bit timer */
#ifdef DT_XHSC_HC32_TIMERA_PWM_0
#if (DT_XHSC_HC32_TIMERA_PWM_0_ADDR == 0x40015000)
PWM_DEVICE_INIT_HC32(0, 40015000)
#elif(DT_XHSC_HC32_TIMERA_PWM_0_ADDR == 0x40015400)
PWM_DEVICE_INIT_HC32(0, 40015400)
#elif(DT_XHSC_HC32_TIMERA_PWM_0_ADDR == 0x40015800)
PWM_DEVICE_INIT_HC32(0, 40015800)
#elif(DT_XHSC_HC32_TIMERA_PWM_0_ADDR == 0x40015c00)
PWM_DEVICE_INIT_HC32(0, 40015C00)
#elif(DT_XHSC_HC32_TIMERA_PWM_0_ADDR == 0x40016000)
PWM_DEVICE_INIT_HC32(0, 40016000)
#elif(DT_XHSC_HC32_TIMERA_PWM_0_ADDR == 0x40016400)
PWM_DEVICE_INIT_HC32(0, 40016400)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_PWM_1
#if(DT_XHSC_HC32_TIMERA_PWM_1_ADDR == 0x40015400)
PWM_DEVICE_INIT_HC32(1, 40015400)
#elif(DT_XHSC_HC32_TIMERA_PWM_1_ADDR == 0x40015800)
PWM_DEVICE_INIT_HC32(1, 40015800)
#elif(DT_XHSC_HC32_TIMERA_PWM_1_ADDR == 0x40015c00)
PWM_DEVICE_INIT_HC32(1, 40015C00)
#elif(DT_XHSC_HC32_TIMERA_PWM_1_ADDR == 0x40016000)
PWM_DEVICE_INIT_HC32(1, 40016000)
#elif(DT_XHSC_HC32_TIMERA_PWM_1_ADDR == 0x40016400)
PWM_DEVICE_INIT_HC32(1, 40016400)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_PWM_2
#if(DT_XHSC_HC32_TIMERA_PWM_2_ADDR == 0x40015800)
PWM_DEVICE_INIT_HC32(2, 40015800)
#elif(DT_XHSC_HC32_TIMERA_PWM_2_ADDR == 0x40015c00)
PWM_DEVICE_INIT_HC32(2, 40015C00)
#elif(DT_XHSC_HC32_TIMERA_PWM_2_ADDR == 0x40016000)
PWM_DEVICE_INIT_HC32(2, 40016000)
#elif(DT_XHSC_HC32_TIMERA_PWM_2_ADDR == 0x40016400)
PWM_DEVICE_INIT_HC32(2, 40016400)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_PWM_3
#if(DT_XHSC_HC32_TIMERA_PWM_3_ADDR == 0x40015c00)
PWM_DEVICE_INIT_HC32(3, 40015C00)
#elif(DT_XHSC_HC32_TIMERA_PWM_3_ADDR == 0x40016000)
PWM_DEVICE_INIT_HC32(3, 40016000)
#elif(DT_XHSC_HC32_TIMERA_PWM_3_ADDR == 0x40016400)
PWM_DEVICE_INIT_HC32(3, 40016400)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_PWM_4
#if(DT_XHSC_HC32_TIMERA_PWM_4_ADDR == 0x40016000)
PWM_DEVICE_INIT_HC32(4, 40016000)
#elif(DT_XHSC_HC32_TIMERA_PWM_4_ADDR == 0x40016400)
PWM_DEVICE_INIT_HC32(4, 40016400)
#endif
#endif

#ifdef DT_XHSC_HC32_TIMERA_PWM_5
PWM_DEVICE_INIT_HC32(5, 40016400)
#endif
