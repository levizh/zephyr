/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_rtc

#include <time.h>

#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/timeutil.h>
#include "hc32_ll.h"

LOG_MODULE_REGISTER(counter_rtc_hc32, CONFIG_COUNTER_LOG_LEVEL);

/* Seconds from 1970-01-01T00:00:00 to 2000-01-01T00:00:00 */
#define T_TIME_OFFSET (946684800)

typedef uint32_t tick_t;

struct rtc_hc32_config {
	struct counter_config_info counter_info;
};

struct rtc_hc32_data {
	counter_alarm_callback_t callback;
	uint32_t ticks;
	void *user_data;
};

static void rtc_hc32_irq_config(const struct device *dev);


static int rtc_hc32_start(const struct device *dev)
{
	ARG_UNUSED(dev);

	RTC_Cmd(ENABLE);

	return 0;
}


static int rtc_hc32_stop(const struct device *dev)
{
	ARG_UNUSED(dev);

	RTC_Cmd(DISABLE);

	return 0;
}

static int rtc_hc32_get_value(const struct device *dev, uint32_t *ticks)
{
	struct tm now = { 0 };
	time_t ts;
	stc_rtc_date_t stcCurrentDate, stcCurrentDate_temp;
	stc_rtc_time_t stcCurrentTime;
	int err = 0;

	ARG_UNUSED(dev);

	/* Read time and date registers. Make sure value of the previous register
	 * hasn't been changed while reading the next one.
	 */
	do {

		if (LL_OK == RTC_GetDate(RTC_DATA_FMT_DEC, &stcCurrentDate)) {
			/* Get current time */
			if (LL_OK == RTC_GetTime(RTC_DATA_FMT_DEC, &stcCurrentTime)) {

				if (LL_OK == RTC_GetDate(RTC_DATA_FMT_DEC, &stcCurrentDate_temp)) {
					/* Print current date and time */
					LOG_DBG("get time: 20%02d/%02d/%02d %02d:%02d:%02d ", stcCurrentDate.u8Year,
						stcCurrentDate.u8Month,
						stcCurrentDate.u8Day, stcCurrentTime.u8Hour,
						stcCurrentTime.u8Minute, stcCurrentTime.u8Second);
				} else {
					LOG_ERR("Get date again failed!");
					err = -EIO;
					return err;
				}
			} else {
				LOG_ERR("Get time failed!");
				err = -EIO;
				return err;
			}
		} else {
			LOG_ERR("Get date failed!");
			err = -EIO;
			return err;
		}

	} while (memcmp(&stcCurrentDate, &stcCurrentDate_temp, sizeof(stc_rtc_date_t)));

	/* Convert calendar datetime to UNIX timestamp */
	/* RTC start time:         1st, Jan, 2000 */
	/* time_t start:           1st, Jan, 1970 */
	now.tm_year = 100 + stcCurrentDate.u8Year;
	/* tm_mon allowed values are 0-11 */
	now.tm_mon = stcCurrentDate.u8Month - 1U;
	now.tm_mday = stcCurrentDate.u8Day;

	now.tm_hour = stcCurrentTime.u8Hour;
	now.tm_min = stcCurrentTime.u8Minute;
	now.tm_sec = stcCurrentTime.u8Second;

	ts = timeutil_timegm(&now);

	/* Return number of seconds since 2000-01-01 00:00:00 */
	ts -= T_TIME_OFFSET;

	__ASSERT(sizeof(time_t) == 8, "unexpected time_t definition");

	*ticks = ts * counter_get_frequency(dev);

	return err;
}

static int rtc_hc32_set_alarm(const struct device *dev, uint8_t chan_id,
			      const struct counter_alarm_cfg *alarm_cfg)
{
	struct tm alarm_tm;
	time_t alarm_val_s;
	struct rtc_hc32_data *data = dev->data;
	stc_rtc_alarm_t stcRtcAlarm;
	int err = 0;

	tick_t ticks = alarm_cfg->ticks;
	tick_t now;

	if (chan_id != 0U) {
		LOG_ERR("Not support alarm chan id %d", chan_id);
		return -ENOTSUP;
	}

	if (data->callback != NULL) {
		LOG_DBG("Alarm busy\n");
		return -EBUSY;
	}

	err = rtc_hc32_get_value(dev, &now);

	if (err) {
		return err;
	}

	data->callback = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) {
		ticks += now + 1;
		alarm_val_s = (time_t)(ticks / counter_get_frequency(dev)) + T_TIME_OFFSET;
	} else {
		alarm_val_s = (time_t)(ticks / counter_get_frequency(dev));
	}

	LOG_DBG("Set Alarm: %d\n", ticks);

	gmtime_r(&alarm_val_s, &alarm_tm);

	/* Apply ALARM */
	stcRtcAlarm.u8AlarmAmPm = RTC_HOUR_24H;
	stcRtcAlarm.u8AlarmHour = alarm_tm.tm_hour;
	stcRtcAlarm.u8AlarmMinute = alarm_tm.tm_min;
	stcRtcAlarm.u8AlarmWeekday = BIT(alarm_tm.tm_wday);

	LOG_DBG("set alarm: wmday = %d, hour = %d, min = %d",
		stcRtcAlarm.u8AlarmWeekday, stcRtcAlarm.u8AlarmHour, stcRtcAlarm.u8AlarmMinute);

	RTC_AlarmCmd(DISABLE);
	RTC_ClearStatus(RTC_FLAG_ALARM);

	(void)RTC_SetAlarm(RTC_DATA_FMT_DEC, &stcRtcAlarm);
	RTC_ClearStatus(RTC_FLAG_ALARM);
	RTC_IntCmd(RTC_INT_ALARM, ENABLE);
	RTC_AlarmCmd(ENABLE);

	return 0;
}


static int rtc_hc32_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	struct rtc_hc32_data *data = dev->data;

	if (chan_id != 0U) {
		LOG_ERR("Not support alarm chan id %d", chan_id);
		return -ENOTSUP;
	}

	RTC_AlarmCmd(DISABLE);
	RTC_IntCmd(RTC_INT_ALARM, ENABLE);
	RTC_ClearStatus(RTC_FLAG_ALARM);

	data->callback = NULL;

	return 0;
}


static uint32_t rtc_hc32_get_pending_int(const struct device *dev)
{
	int pending = 0;

	if (SET == RTC_GetStatus(RTC_FLAG_ALARM)) {
		RTC_ClearStatus(RTC_FLAG_ALARM);
		/* Alarm pending */
		pending = 1;
		LOG_DBG("alarm is pending");
	}
	return pending;
}


static uint32_t rtc_hc32_get_top_value(const struct device *dev)
{
	const struct counter_config_info *info = dev->config;

	return info->max_top_value;
}


static int rtc_hc32_set_top_value(const struct device *dev,
				  const struct counter_top_cfg *cfg)
{
	const struct counter_config_info *info = dev->config;

	if ((cfg->ticks != info->max_top_value) ||
	    !(cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
		return -ENOTSUP;
	} else {
		return 0;
	}


}

void rtc_hc32_alarm_irq_handler(const struct device *dev)
{
	struct rtc_hc32_data *data = dev->data;
	counter_alarm_callback_t alarm_callback = data->callback;

	uint32_t now;

	LOG_DBG("alarm irq");

	(void)rtc_hc32_get_value(dev, &now);

	RTC_AlarmCmd(DISABLE);
	RTC_IntCmd(RTC_INT_ALARM, ENABLE);
	RTC_ClearStatus(RTC_FLAG_ALARM);

	if (alarm_callback != NULL) {
		data->callback = NULL;
		alarm_callback(dev, 0, now, data->user_data);
	}
}

static void rtc_hc32_disable_ints(const struct device *dev)
{
	ARG_UNUSED(dev);
	if (ENABLE == RTC_GetCounterState()) {
		/* RTC is running, disable alarm/period ints */
		RTC_IntCmd(RTC_INT_ALL, DISABLE);
	}
}

static int rtc_hc32_calendar_init_Config(void)
{
	stc_rtc_date_t stcRtcDate;
	stc_rtc_time_t stcRtcTime;
	int err = 0;

	/* 2000-01-01 00:00:00 Saturday */

	/* Date configuration */
	stcRtcDate.u8Year    = 00U;
	stcRtcDate.u8Month   = RTC_MONTH_JANUARY;
	stcRtcDate.u8Day     = 01U;
	stcRtcDate.u8Weekday = RTC_WEEKDAY_SATURDAY;

	/* Time configuration */
	stcRtcTime.u8Hour   = 00U;
	stcRtcTime.u8Minute = 00U;
	stcRtcTime.u8Second = 00U;
	stcRtcTime.u8AmPm   = RTC_HOUR_24H;

	if (LL_OK != RTC_SetDate(RTC_DATA_FMT_DEC, &stcRtcDate)) {
		LOG_ERR("Set Date failed!\r\n");
		err = -EIO;
	}

	if (LL_OK != RTC_SetTime(RTC_DATA_FMT_DEC, &stcRtcTime)) {
		LOG_ERR("Set Time failed!\r\n");
		err = -EIO;
	}

	return err;
}

#if DT_SAME_NODE(DT_CLOCKS_CTLR(DT_DRV_INST(0)), DT_NODELABEL(clk_lrc))
#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_lrc), okay)
#define HC32_DT_RTC_CLK_SRC      (RTC_CLK_SRC_LRC)
#else
#error "rtc clk select lrc, but lrc disable in dts."
#endif
#elif DT_SAME_NODE(DT_CLOCKS_CTLR(DT_DRV_INST(0)), DT_NODELABEL(clk_xtal32))
#if DT_NODE_HAS_STATUS(DT_NODELABEL(clk_xtal32), okay)
#define HC32_DT_RTC_CLK_SRC      (RTC_CLK_SRC_XTAL32)
#else
#error "rtc clk select xtal32, but xtal32 disable in dts."
#endif
#else
#error "please select correct rtc clk src in dts."
#endif

static int rtc_hc32_configure(const struct device *dev)
{
	stc_rtc_init_t stcRtcInit;
	int err = 0;
	ARG_UNUSED(dev);

#if defined(CONFIG_COUNTER_RTC_HC32_SAVE_VALUE_BETWEEN_RESETS)
	/* RTC stopped */
	if (DISABLE == RTC_GetCounterState())
#endif
	{
		/* Reset RTC counter */
		if (LL_ERR_TIMEOUT == RTC_DeInit()) {
			LOG_ERR("Reset RTC failed!");
			err = -EIO;
		} else {
			/* Stop RTC */
			RTC_Cmd(DISABLE);
			/* Configure structure initialization */
			(void)RTC_StructInit(&stcRtcInit);

			/* Configuration RTC structure */
			stcRtcInit.u8ClockSrc   = HC32_DT_RTC_CLK_SRC;
			stcRtcInit.u8HourFormat = RTC_HOUR_FMT_24H;
			stcRtcInit.u8IntPeriod  = RTC_INT_PERIOD_INVD;
			(void)RTC_Init(&stcRtcInit);

			err = rtc_hc32_calendar_init_Config();
			if (err) {
				return err;
			}

			/* Enable period interrupt */
			RTC_ClearStatus(RTC_FLAG_CLR_ALL);
			/* Startup RTC count */
			RTC_Cmd(ENABLE);
		}
	}

	return err;
}

static int rtc_hc32_init(const struct device *dev)
{
	struct rtc_hc32_data *data = dev->data;
	int err = 0;

	data->callback = NULL;

	rtc_hc32_disable_ints(dev);

	rtc_hc32_irq_config(dev);

	err = rtc_hc32_configure(dev);

	return err;
}

static struct rtc_hc32_data rtc_data;

static const struct rtc_hc32_config rtc_config = {
	.counter_info = {
		.max_top_value = UINT32_MAX,
		.freq = 1,/* freq = 1Hz */
		.flags = COUNTER_CONFIG_INFO_COUNT_UP,
		.channels = 1,
	},
};


static const struct counter_driver_api rtc_hc32_driver_api = {
	.start = rtc_hc32_start,
	.stop = rtc_hc32_stop,
	.get_value = rtc_hc32_get_value,
	.set_alarm = rtc_hc32_set_alarm,
	.cancel_alarm = rtc_hc32_cancel_alarm,
	.set_top_value = rtc_hc32_set_top_value,
	.get_pending_int = rtc_hc32_get_pending_int,
	.get_top_value = rtc_hc32_get_top_value,
};

#define RTC_IRQ_CONFIG(inst, idx, irq_hander)                                               \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, idx, irq), DT_INST_IRQ_BY_IDX(inst, idx, priority),\
		    irq_hander, DEVICE_DT_INST_GET(inst), 0);                                       \
	hc32_intc_irq_signin(DT_INST_IRQ_BY_IDX(inst, idx, irq),                                \
			     DT_INST_IRQ_BY_IDX(inst, idx, int_src));                                   \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, idx, irq));

static void rtc_hc32_irq_config(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* ALM IRQ: idx 0 */
	RTC_IRQ_CONFIG(0, 0, rtc_hc32_alarm_irq_handler);
}

DEVICE_DT_INST_DEFINE(0, &rtc_hc32_init, NULL,
		      &rtc_data, &rtc_config, PRE_KERNEL_1,
		      CONFIG_COUNTER_INIT_PRIORITY, &rtc_hc32_driver_api);
