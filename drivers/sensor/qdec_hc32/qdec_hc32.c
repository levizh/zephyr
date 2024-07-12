/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_qdec

/** @file
 * @brief STM32 family Quadrature Decoder (QDEC) driver.
 */

#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor/qdec_hc32.h>
#include <zephyr/dt-bindings/sensor/qdec_hc32.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <hc32_ll_tmra.h>

LOG_MODULE_REGISTER(qdec_hc32, CONFIG_SENSOR_LOG_LEVEL);

struct qdec_hc32_config {
	uint32_t encoder_cnt_cond;
	uint32_t filter_cfg;
};

/* Device configuration parameters */
struct qdec_hc32_dev_cfg {
	const struct pinctrl_dev_config *pin_config;
	const struct hc32_modules_clock_sys *mod_clk;
	struct qdec_hc32_config tmr_cfg;
	CM_TMRA_TypeDef *timer;
	uint32_t counts_per_revolution;
};

/* Device run time data */
struct qdec_hc32_dev_data {
	uint32_t revolution;
	uint32_t position;
};
/*
uint32_t qdec_cycle_cnt =0;
k_tid_t tid;
static struct k_thread qdec_thread;
static K_THREAD_STACK_DEFINE(qdec_stack, 1024);


static void qdec_entry(void *p1, void *p2, void *p3)
{
	const struct device *dev = (const struct device *)p1;
	struct qdec_hc32_dev_cfg *cfg = \
		(struct qdec_hc32_dev_cfg *)dev->config;
	CM_TMRA_TypeDef *tmr = cfg->timer;
	uint32_t last = 0, current, cnt = 0;
	uint16_t i = 0;
	uint32_t tick = sys_clock_cycle_get_32();

	while (1) {
		do {
			k_sleep(Z_TIMEOUT_TICKS(1));
			current = TMRA_GetCountValue(tmr);
			if (last < current) {
				cnt += current - last;
			} else {
				cnt += TMRA_GetPeriodValue(tmr) + current - last;
			}
			last = current;
		} while ((sys_clock_cycle_get_32() - tick) < CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
		tick = sys_clock_cycle_get_32();
		qdec_cycle_cnt = cnt;
		cnt = 0;
	}
}
*/
static int qdec_hc32_attr_set(const struct device *dev, enum sensor_channel ch,
	enum sensor_attribute attr, const struct sensor_value *val)
{
	struct qdec_hc32_dev_cfg *cfg = \
		(struct qdec_hc32_dev_cfg *)dev->config;
	CM_TMRA_TypeDef *tmr = cfg->timer;
	uint8_t tmr_up_cnt_cond, tmr_down_cnt_cond;

	if (ch != SENSOR_CHAN_RPM) {
		return -ENOTSUP;
	}

	switch ((enum sensor_attribute_qdec_hc32) attr) {
	case SENSOR_ATTR_QDEC_MODE_VAL:
		if (1 > val->val1) {
			LOG_ERR("SENSOR_ATTR_QDEC_MODE_VAL value invalid");
			return -EINVAL;
		}
		cfg->counts_per_revolution = val->val1;
		break;
	case SENSOR_ATTR_QDEC_CNT_CONDITION:
		tmr_up_cnt_cond = (uint8_t)val->val1;
		tmr_down_cnt_cond = (uint8_t)(val->val1 >> 16);
		if ((0 == IN_RANGE(tmr_up_cnt_cond, 0, TMR_CNT_COND_MAX_VAL)) || \
			(0 == IN_RANGE(tmr_down_cnt_cond, 0, TMR_CNT_COND_MAX_VAL))) {
			LOG_ERR("SENSOR_ATTR_QDEC_CNT_CONDITION value invalid");
			return -EINVAL;
		}
		cfg->tmr_cfg.encoder_cnt_cond = val->val1;
		MODIFY_REG16(tmr->HCUPR ,TMR_CNT_COND_MAX_VAL, tmr_up_cnt_cond);
		MODIFY_REG16(tmr->HCDOR ,TMR_CNT_COND_MAX_VAL, tmr_down_cnt_cond);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int qdec_hc32_attr_get(const struct device *dev, enum sensor_channel ch,
	enum sensor_attribute attr, struct sensor_value *val)
{
	const struct qdec_hc32_dev_cfg *cfg = \
		(const struct qdec_hc32_dev_cfg *)dev->config;

	if (ch != SENSOR_CHAN_RPM) {
		return -ENOTSUP;
	}

	switch ((enum sensor_attribute_qdec_hc32) attr) {
	case SENSOR_ATTR_QDEC_MODE_VAL:
		val->val1 = cfg->counts_per_revolution;
		break;
	case SENSOR_ATTR_QDEC_CNT_CONDITION:
		val->val1 = cfg->tmr_cfg.encoder_cnt_cond;
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static uint32_t qdec_calculate(const struct device *dev)
{
	const struct qdec_hc32_dev_cfg *cfg = \
		(const struct qdec_hc32_dev_cfg *)dev->config;
	CM_TMRA_TypeDef *tmr = cfg->timer;
	uint32_t last, current, cycle_cnt = 0;
	uint32_t tick = sys_clock_cycle_get_32();

	last = TMRA_GetCountValue(tmr);
	TMRA_ClearStatus(tmr, (TMRA_FLAG_OVF | TMRA_FLAG_UDF));
	
	k_sleep(Z_TIMEOUT_TICKS(1));
	do {
		
		current = TMRA_GetCountValue(tmr);
		if (TMRA_DIR_UP == TMRA_GetCountDir(tmr)) {
			if (last < current) {
				cycle_cnt += current - last;
			} else if (SET == TMRA_GetStatus(tmr, (TMRA_FLAG_OVF | TMRA_FLAG_UDF))) {
				TMRA_ClearStatus(tmr, (TMRA_FLAG_OVF | TMRA_FLAG_UDF));
				cycle_cnt += TMRA_GetPeriodValue(tmr) + current - last;
			}
		} else {
			if (last > current) {
				cycle_cnt += last - current;
			} else if (SET == TMRA_GetStatus(tmr, (TMRA_FLAG_OVF | TMRA_FLAG_UDF))) {
				TMRA_ClearStatus(tmr, (TMRA_FLAG_OVF | TMRA_FLAG_UDF));
				cycle_cnt += TMRA_GetPeriodValue(tmr) + last - current;
			}
		}
		last = current;
		k_sleep(Z_TIMEOUT_TICKS(1));
	} while ((sys_clock_cycle_get_32() - tick) < CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
	
	
	return cycle_cnt;
}

static int qdec_hc32_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct qdec_hc32_dev_data *data = (struct qdec_hc32_dev_data *)dev->data;
	const struct qdec_hc32_dev_cfg *cfg = \
		(const struct qdec_hc32_dev_cfg *)dev->config;
	CM_TMRA_TypeDef *tmr = cfg->timer;
	uint32_t cycle_cnt;

	if ((chan != SENSOR_CHAN_ALL) && (chan != SENSOR_CHAN_RPM) &&
		(chan != SENSOR_CHAN_ROTATION)) {
		return -ENOTSUP;
	}

	cycle_cnt = TMRA_GetCountValue(tmr);

	switch (chan)
	{
	case SENSOR_CHAN_RPM:
		cycle_cnt = qdec_calculate(dev);
		data->revolution = (cycle_cnt * 60) / \
		(cfg->counts_per_revolution);
		break;
	
	case SENSOR_CHAN_ROTATION:
		if (TMRA_DIR_DOWN == TMRA_GetCountDir(tmr)) {
			cycle_cnt = TMRA_GetPeriodValue(tmr) - cycle_cnt;
		}
		cycle_cnt = cycle_cnt % cfg->counts_per_revolution;
		data->position = cycle_cnt * 360 / cfg->counts_per_revolution;
		break;
	
	case SENSOR_CHAN_ALL:
		/* get rpm will blocks 1s until the calculate done.
		** the angular rotation is one second form now.
		*/
		cycle_cnt = qdec_calculate(dev);
		data->revolution = (cycle_cnt * 60) / (cfg->counts_per_revolution);
		if (TMRA_DIR_DOWN == TMRA_GetCountDir(tmr)) {
			cycle_cnt = TMRA_GetPeriodValue(tmr) - cycle_cnt;
		}
		cycle_cnt = cycle_cnt % cfg->counts_per_revolution;
		data->position = cycle_cnt * 360 / cfg->counts_per_revolution;
		break;
	
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int qdec_hc32_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val)
{
	struct qdec_hc32_dev_data *const data = dev->data;

	switch (chan)
	{
	case SENSOR_CHAN_RPM:
		val->val1 = data->revolution;
		val->val2 = 0;
		break;

	case SENSOR_CHAN_ROTATION:
		val->val1 = data->position;
		val->val2 = 0;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int qdec_hc32_initialize(const struct device *dev)
{
	const struct device *const clk = DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE);
	const struct qdec_hc32_dev_cfg *const cfg = dev->config;
	CM_TMRA_TypeDef *tmr = cfg->timer;
	stc_tmra_init_t stcTmraInit;
	int ret;

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = pinctrl_apply_state(cfg->pin_config, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("qdec pinctrl setup failed (%d)", ret);
		return ret;
	}

	ret = clock_control_on(clk,
			     (clock_control_subsys_t)&cfg->mod_clk[0]);
	if (ret < 0) {
		LOG_ERR("Could not initialize clock");
		return ret;
	}

	if (cfg->counts_per_revolution < 1) {
		LOG_ERR("Invalid number of counts per revolution (%d)",
			cfg->counts_per_revolution);
		return -EINVAL;
	}

	TMRA_DeInit(tmr);
	(void)TMRA_StructInit(&stcTmraInit);
	stcTmraInit.u8CountSrc = TMRA_CNT_SRC_HW;
	stcTmraInit.hw_count.u16CountUpCond = (uint16_t)(cfg->tmr_cfg.encoder_cnt_cond);
	stcTmraInit.hw_count.u16CountDownCond = (uint16_t)(cfg->tmr_cfg.encoder_cnt_cond >> 16);
	stcTmraInit.u32PeriodValue = \
		UINT16_MAX - (UINT16_MAX % cfg->counts_per_revolution) - 1;
	TMRA_Init(tmr, &stcTmraInit);

	WRITE_REG32(tmr->FCONR, cfg->tmr_cfg.filter_cfg);
/*
	tid = k_thread_create(&qdec_thread, qdec_stack,
			K_THREAD_STACK_SIZEOF(qdec_stack), qdec_entry,
			(void*)dev, NULL, NULL, 10, 0, K_NO_WAIT);
*/
	WRITE_REG32(tmr->CNTER, 0);

	SET_REG8_BIT(tmr->BCSTRL, TMRA_BCSTRL_START);

	return 0;
}

static const struct sensor_driver_api qdec_hc32_driver_api = {
	.attr_set = qdec_hc32_attr_set,
	.attr_get = &qdec_hc32_attr_get,
	.sample_fetch = qdec_hc32_fetch,
	.channel_get = qdec_hc32_get,
};

#define QDEC_HC32_INIT(n)								\
	PINCTRL_DT_INST_DEFINE(n);							\
	static const struct hc32_modules_clock_sys mudules_clk_##n[] =		\
				 HC32_MODULES_CLOCKS(DT_INST_PARENT(n));		\
	static struct qdec_hc32_dev_cfg qdec##n##_hc32_config = {		\
		.pin_config = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.timer = ((CM_TMRA_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n))),		\
		.mod_clk = mudules_clk_##n,								\
		.tmr_cfg = {	\
			.encoder_cnt_cond = DT_INST_PROP(n, hw_count_conditions),	\
			.filter_cfg = DT_INST_PROP_OR(n, filter_config, 0),	\
		},	\
		.counts_per_revolution = DT_INST_PROP(n, counts_per_revolution),	\
	};										\
											\
	static struct qdec_hc32_dev_data qdec##n##_hc32_data;				\
											\
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_hc32_initialize, NULL,			\
				&qdec##n##_hc32_data, &qdec##n##_hc32_config,		\
				POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,		\
				&qdec_hc32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_HC32_INIT)
