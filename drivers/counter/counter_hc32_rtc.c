/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Source file for the hc32 RTC driver
 *
 */

#include <time.h>

#include <clock_control/hc32_clock_control.h>
#include <clock_control.h>
#include <misc/util.h>
#include <kernel.h>
#include <soc.h>
#include <counter.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(counter_hc32_rtc, CONFIG_COUNTER_LOG_LEVEL);

#define T_TIME_OFFSET 946684800

#if CONFIG_COUNTER_RTC_HC32_CLOCK_LRC
#define HC32_RTC_CLK_SRC	RTC_CLK_SRC_LRC
#elif CONFIG_COUNTER_RTC_HC32_CLOCK_XTAL32
#define HC32_RTC_CLK_SRC	RTC_CLK_SRC_XTAL32
#endif
struct rtc_hc32_config {
	struct counter_config_info counter_info;
};

struct rtc_hc32_data {
	counter_alarm_callback_t callback;
	u32_t ticks;
	void *user_data;
	bool absolute;
};


#define DEV_DATA(dev) ((struct rtc_hc32_data *)(dev->driver_data))
#define DEV_CFG(dev)	((const struct rtc_hc32_config *)(dev->config->config_info))


static void rtc_hc32_irq_config(struct device *dev);


static int rtc_hc32_start(struct device *dev)
{
	ARG_UNUSED(dev);

	RTC_Cmd(ENABLE);

	return 0;
}


static int rtc_hc32_stop(struct device *dev)
{
	ARG_UNUSED(dev);

	RTC_Cmd(DISABLE);

	return 0;
}


static u32_t rtc_hc32_read(struct device *dev)
{
	struct tm now = { 0 };
	time_t ts;
	stc_rtc_date_t rtc_date;
	stc_rtc_time_t rtc_time;
	u32_t ticks;

	ARG_UNUSED(dev);

	if (LL_OK == RTC_GetDate(RTC_DATA_FMT_DEC, &rtc_date)) {
		if (LL_OK == RTC_GetTime(RTC_DATA_FMT_DEC, &rtc_time)) {

		} else {
			LOG_ERR("Get time failed!");
		}
	} else {
		LOG_ERR("Get date failed!");
	}
	
	/* Convert calendar datetime to UNIX timestamp */
	/* RTC start time: 1st, Jan, 2000 */
	/* time_t start:   1st, Jan, 1900 */
	now.tm_year = 100 + rtc_date.u8Year;
	/* tm_mon allowed values are 0-11 */
	now.tm_mon = rtc_date.u8Month - 1;
	now.tm_mday = rtc_date.u8Day;

	now.tm_hour = rtc_time.u8Hour;
	now.tm_min = rtc_time.u8Minute;
	now.tm_sec = rtc_time.u8Second;

	ts = mktime(&now);

	/* Return number of seconds since RTC init */
	ts -= T_TIME_OFFSET;

	__ASSERT(sizeof(time_t) == 8, "unexpected time_t definition");
	ticks = counter_us_to_ticks(dev, ts * USEC_PER_SEC);

	return ticks;
}

static int rtc_hc32_set_alarm(struct device *dev, u8_t chan_id,
				const struct counter_alarm_cfg *alarm_cfg)
{
	struct tm alarm_tm;
	time_t alarm_val;
	stc_rtc_alarm_t rtc_alarm;
	struct rtc_hc32_data *data = DEV_DATA(dev);

	u32_t now = rtc_hc32_read(dev);
	u32_t ticks = alarm_cfg->ticks;

	if (data->callback != NULL) {
		LOG_DBG("Alarm busy\n");
		return -EBUSY;
	}


	data->callback = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;
	data->absolute = alarm_cfg->absolute;

	if (!alarm_cfg->absolute) {
		ticks += now;
	}

	if (ticks < now) {
		return -ENOTSUP;
	}

	LOG_DBG("Set Alarm: %d\n", ticks);

	alarm_val = (time_t)(counter_ticks_to_us(dev, ticks) / USEC_PER_SEC);

	gmtime_r(&alarm_val, &alarm_tm);

	/* Apply ALARM_A */
	rtc_alarm.u8AlarmAmPm = RTC_HOUR_24H;
	rtc_alarm.u8AlarmHour = alarm_tm.tm_hour;
	rtc_alarm.u8AlarmMinute = alarm_tm.tm_min;
	rtc_alarm.u8AlarmWeekday = BIT(alarm_tm.tm_wday);

	RTC_AlarmCmd(DISABLE);
	RTC_ClearStatus(RTC_FLAG_ALARM);

	(void)RTC_SetAlarm(RTC_DATA_FMT_DEC, &rtc_alarm);
	RTC_ClearStatus(RTC_FLAG_ALARM);
	RTC_IntCmd(RTC_INT_ALARM, ENABLE);
	RTC_AlarmCmd(ENABLE);

	return 0;
}


static int rtc_hc32_cancel_alarm(struct device *dev, u8_t chan_id)
{
	RTC_AlarmCmd(DISABLE);
	RTC_IntCmd(RTC_INT_ALARM, ENABLE);
	RTC_ClearStatus(RTC_FLAG_ALARM);

	DEV_DATA(dev)->callback = NULL;

	return 0;
}


static u32_t rtc_hc32_get_pending_int(struct device *dev)
{
	return RTC_GetStatus(RTC_FLAG_ALARM);
}


static u32_t rtc_hc32_get_top_value(struct device *dev)
{
	const struct rtc_hc32_config *config = DEV_CFG(dev);

	return config->counter_info.max_top_value;
}


static int rtc_hc32_set_top_value(struct device *dev, u32_t ticks,
				counter_top_callback_t callback,
				void *user_data)
{
	const struct rtc_hc32_config *config = DEV_CFG(dev);

	ARG_UNUSED(dev);
	ARG_UNUSED(callback);
	ARG_UNUSED(user_data);

	if (ticks != config->counter_info.max_top_value) {
		return -ENOTSUP;
	} else {
		return 0;
	}
}


static u32_t rtc_hc32_get_max_relative_alarm(struct device *dev)
{
	const struct rtc_hc32_config *config = DEV_CFG(dev);

	return config->counter_info.max_top_value;
}


void rtc_hc32_isr(struct device *dev)
{
	struct rtc_hc32_data *data = DEV_DATA(dev);
	counter_alarm_callback_t alarm_callback = data->callback;

	u32_t now = rtc_hc32_read(dev);

	if (SET == RTC_GetStatus(RTC_FLAG_ALARM)) {
		RTC_ClearStatus(RTC_FLAG_ALARM);
		RTC_AlarmCmd(DISABLE);
		RTC_IntCmd(RTC_INT_ALARM, ENABLE);

		if (alarm_callback != NULL) {
			data->callback = NULL;
			alarm_callback(dev, 0, now, data->user_data);
		}
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

static int rtc_hc32_init(struct device *dev)
{
	stc_rtc_init_t stcRtcInit;

	DEV_DATA(dev)->callback = NULL;

	if (DISABLE == RTC_GetCounterState()) {
		if (LL_ERR_TIMEOUT == RTC_DeInit()) {
			LOG_ERR("Reset RTC failed!");
			return -EIO;
		} else {
			RTC_Cmd(DISABLE);

			(void)RTC_StructInit(&stcRtcInit);

			/* Configuration RTC structure */
			stcRtcInit.u8ClockSrc   = HC32_RTC_CLK_SRC;
			stcRtcInit.u8HourFormat = RTC_HOUR_FMT_24H;
			stcRtcInit.u8IntPeriod  = RTC_INT_PERIOD_INVD;
			(void)RTC_Init(&stcRtcInit);

			if (0 != rtc_hc32_calendar_init_Config()) {
				return -EIO;
			}

			RTC_ClearStatus(RTC_FLAG_CLR_ALL);
			RTC_IntCmd(RTC_INT_ALL, DISABLE);

			RTC_Cmd(ENABLE);
		}
	}

	rtc_hc32_irq_config(dev);

	return 0;
}

static struct rtc_hc32_data rtc_data;

static const struct rtc_hc32_config rtc_config = {
	.counter_info = {
		.max_top_value = UINT32_MAX,
		.freq = 1,
		.count_up = true,
		.channels = 1,
	},
};

static const struct counter_driver_api rtc_hc32_driver_api = {
		.start = rtc_hc32_start,
		.stop = rtc_hc32_stop,
		.read = rtc_hc32_read,
		.set_alarm = rtc_hc32_set_alarm,
		.cancel_alarm = rtc_hc32_cancel_alarm,
		.set_top_value = rtc_hc32_set_top_value,
		.get_pending_int = rtc_hc32_get_pending_int,
		.get_top_value = rtc_hc32_get_top_value,
		.get_max_relative_alarm = rtc_hc32_get_max_relative_alarm,
};

DEVICE_AND_API_INIT(rtc_hc32, "rtc_0", &rtc_hc32_init,
		    &rtc_data, &rtc_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &rtc_hc32_driver_api);

static void rtc_hc32_irq_config(struct device *dev)
{
	IRQ_CONNECT(31, 15,
		    rtc_hc32_isr, DEVICE_GET(rtc_hc32), 0);
	irq_enable(31);
}
