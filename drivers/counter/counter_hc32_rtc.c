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
#include <interrupt_controller/intc_hc32.h>
#include <clock_control.h>
#include <misc/util.h>
#include <kernel.h>
#include <soc.h>
#include <counter.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(counter_hc32_rtc, CONFIG_COUNTER_LOG_LEVEL);

/* Seconds from 1970-01-01T00:00:00 to 2000-01-01T00:00:00 */
#define T_TIME_OFFSET				(946684800UL)

/* One week: 7days * 24h * 60min * 60s */
#define MAX_RELATIVE_ALARM_SECONDS	(604800UL)

struct rtc_hc32_config {
	struct counter_config_info counter_info;
	u32_t irqn;
};

struct rtc_hc32_data {
	counter_alarm_callback_t callback;
	u32_t ticks;
	void *user_data;
	bool absolute;
};

#define DEV_DATA(dev)   ((struct rtc_hc32_data *)(dev->driver_data))
#define DEV_CFG(dev)	((const struct rtc_hc32_config *)(dev->config->config_info))

static void rtc_hc32_irq_config(struct device *dev);

static void rtc_hc32_set_alarm_int_pending(struct device *dev)
{
	const struct rtc_hc32_config *cfg = DEV_CFG(dev);

	NVIC_SetPendingIRQ(cfg->irqn);
}

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
			LOG_DBG("get time: 20%02d/%02d/%02d %02d:%02d:%02d ",
				rtc_date.u8Year, rtc_date.u8Month, rtc_date.u8Day,
				rtc_time.u8Hour, rtc_time.u8Minute, rtc_time.u8Second);
		} else {
			LOG_ERR("Get time failed!");
		}
	} else {
		LOG_ERR("Get date failed!");
	}

	/* Convert calendar datetime to UNIX timestamp */
	/* RTC start time: 1st, Jan, 2000 */
	/* time_t start:   1st, Jan, 1970 */
	now.tm_year = 100U + rtc_date.u8Year;
	/* tm_mon allowed values are 0-11 */
	now.tm_mon = rtc_date.u8Month - 1U;
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
	u32_t ticks_now_diff;

	if (data->callback != NULL) {
		LOG_DBG("Alarm busy\n");
		return -EBUSY;
	}

	data->callback = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;
	data->absolute = alarm_cfg->absolute;

	if (!alarm_cfg->absolute) {
		ticks_now_diff = ticks;
		ticks += now;
		ticks += T_TIME_OFFSET;
	} else {
		if (ticks < (now + T_TIME_OFFSET)) {
			return -EINVAL;
		}
		ticks_now_diff = now + T_TIME_OFFSET - ticks;
	}

	if (ticks_now_diff > MAX_RELATIVE_ALARM_SECONDS) {
		LOG_WRN("Alarm exceeds 7days difference from now.");
		return -EINVAL;
	}

	alarm_val = (time_t)(counter_ticks_to_us(dev, ticks) / USEC_PER_SEC);

	gmtime_r(&alarm_val, &alarm_tm);

	/* Apply ALARM */
	rtc_alarm.u8AlarmAmPm = RTC_HOUR_24H;
	rtc_alarm.u8AlarmHour = alarm_tm.tm_hour;
	rtc_alarm.u8AlarmMinute = alarm_tm.tm_min;
	rtc_alarm.u8AlarmWeekday = BIT(alarm_tm.tm_wday);

	LOG_DBG("set alarm: %02d/%02d/%02d %02d:%02d:%02d ",
		alarm_tm.tm_year + 1900, alarm_tm.tm_mon, alarm_tm.tm_mday,
		alarm_tm.tm_hour, alarm_tm.tm_min, alarm_tm.tm_sec);

	if ((ticks_now_diff < 60) && (alarm_tm.tm_sec >= ticks_now_diff)) {
		/* The HC32 RTC alarm has a minute-level accuracy.
		 * The current time difference for the alarm is less than 60 seconds,
		 * and it is not crossing the whole minute (XX:XX:00).
		 * Hence, the alarm will be triggered immediately.
		*/
		LOG_WRN("HC32 RTC Alarm accuracy is minute, alarm will trigger immediately");
		rtc_hc32_set_alarm_int_pending(dev);
	} else {
		RTC_AlarmCmd(DISABLE);
		RTC_ClearStatus(RTC_FLAG_ALARM);

		(void)RTC_SetAlarm(RTC_DATA_FMT_DEC, &rtc_alarm);
		RTC_ClearStatus(RTC_FLAG_ALARM);
		RTC_IntCmd(RTC_INT_ALARM, ENABLE);
		RTC_AlarmCmd(ENABLE);
	}

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
	ARG_UNUSED(dev);

	return MAX_RELATIVE_ALARM_SECONDS;
}


void rtc_hc32_alarm_isr(struct device *dev)
{
	struct rtc_hc32_data *data = DEV_DATA(dev);
	counter_alarm_callback_t alarm_callback = data->callback;
	u32_t now = rtc_hc32_read(dev);

	LOG_DBG("alarm isr");

	if (SET == RTC_GetStatus(RTC_FLAG_ALARM)) {
		RTC_ClearStatus(RTC_FLAG_ALARM);
		RTC_AlarmCmd(DISABLE);
		RTC_IntCmd(RTC_INT_ALARM, ENABLE);
	}
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

#if (DT_XHSC_HC32_RTC_0_CLOCK_SRC == HC32_CLK_SRC_LRC)
#if defined(CONFIG_HC32_CLK_LRC)
#define HC32_DT_RTC_CLK_SRC      (RTC_CLK_SRC_LRC)
#else
#error "rtc set src LRC, but LRC is off."
#endif
#elif (DT_XHSC_HC32_RTC_0_CLOCK_SRC == HC32_CLK_SRC_XTAL32)
#if defined(CONFIG_HC32_CLK_XTAL32)
#define HC32_DT_RTC_CLK_SRC      (RTC_CLK_SRC_XTAL32)
#else
#error "rtc set src XTAL32, but XTAL32 is off."
#endif
#else
#error "please set correct rtc clk src in dts: clock-src = HC32_CLK_SRC_LRC or HC32_CLK_SRC_XTAL32."
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

static int rtc_hc32_init(struct device *dev)
{
	struct rtc_hc32_data *data = DEV_DATA(dev);
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
		.freq = 1U,
		.count_up = true,
		.channels = 1U,
	},
	.irqn = DT_XHSC_HC32_RTC_0_IRQ_ALM,
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

DEVICE_AND_API_INIT(rtc_hc32, DT_XHSC_HC32_RTC_0_LABEL, &rtc_hc32_init,
		    &rtc_data, &rtc_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &rtc_hc32_driver_api);

static void rtc_hc32_irq_config(struct device *dev)
{
	IRQ_CONNECT(DT_XHSC_HC32_RTC_0_IRQ_ALM, DT_XHSC_HC32_RTC_0_IRQ_ALM_PRIORITY,
		    rtc_hc32_alarm_isr, DEVICE_GET(rtc_hc32), 0);
	hc32_intc_irq_signin(DT_XHSC_HC32_RTC_0_IRQ_ALM,
			     DT_XHSC_HC32_RTC_0_IRQ_ALM_INT_SRC);
	irq_enable(DT_XHSC_HC32_RTC_0_IRQ_ALM);
}
