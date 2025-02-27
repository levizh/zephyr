/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_rtc

#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/kernel.h>
#include <math.h>
#include "hc32_ll.h"

LOG_MODULE_REGISTER(rtc_hc32, CONFIG_RTC_LOG_LEVEL);


/* RTC start time: 1st, Jan, 2000 */
#define RTC_YEAR_REF (2000)
/* struct tm start time:   1st, Jan, 1900 */
#define TM_YEAR_REF  (1900)

/* ppb = 1000*ppm */
#define PPB_TO_PPM(ppb)         (ppb / 1000.0F)
#define PPM_TO_PPB(ppm)         (ppm * 1000.0F)


#define CAL_RESOLUTION_IN_PPB   (960)
#define CAL_LIMIT_IN_PPB_LOW    (-(275500 + CAL_RESOLUTION_IN_PPB))
#define CAL_LIMIT_IN_PPB_HIGHT  (212900 + CAL_RESOLUTION_IN_PPB)


/* RTC alarm time fields supported by the hc32 */
#define HC32_RTC_ALARM_TIME_MASK                                                                \
	(RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_WEEKDAY)


struct rtc_hc32_config {
	uint8_t clk_src;
};

struct rtc_hc32_data {
	struct k_mutex lock;
#ifdef CONFIG_RTC_ALARM
	rtc_alarm_callback alarm_callback;
	void *alarm_user_data;
#endif /* CONFIG_RTC_ALARM */
#ifdef CONFIG_RTC_UPDATE
	rtc_update_callback update_callback;
	void *update_user_data;
#endif /* CONFIG_RTC_UPDATE */
};

static void rtc_hc32_irq_config(const struct device *dev);

#ifdef CONFIG_RTC_ALARM
static void rtc_hc32_alarm_irq_handler(const struct device *dev)
{
	struct rtc_hc32_data *data = dev->data;
	RTC_ClearStatus(RTC_FLAG_ALARM);

	LOG_DBG("alarm irq handler");
	if (data->alarm_callback) {
		data->alarm_callback(dev, 0U, data->alarm_user_data);
	}
}
#endif

#ifdef CONFIG_RTC_UPDATE
static void rtc_hc32_prd_irq_handler(const struct device *dev)
{
	struct rtc_hc32_data *data = dev->data;

	LOG_DBG("update irq handler");
	if (data->update_callback) {
		data->update_callback(dev, data->update_user_data);
	}
}
#endif

static int rtc_hc32_set_time(const struct device *dev,
			     const struct rtc_time *timeptr)
{
	struct rtc_hc32_data *data = dev->data;
	stc_rtc_date_t stcRtcDate;
	stc_rtc_time_t stcRtcTime;

	uint32_t real_year = timeptr->tm_year + TM_YEAR_REF;

	int err = 0;

	if (real_year < RTC_YEAR_REF) {
		LOG_ERR("RTC does not support years before 2000!");
		return -EINVAL;
	}

	if (timeptr->tm_wday == -1) {
		/* day of the week is expected */
		return -EINVAL;
	}

	err = k_mutex_lock(&data->lock, K_NO_WAIT);
	if (err) {
		return err;
	}

	/* Date configuration */
	stcRtcDate.u8Year    = real_year - RTC_YEAR_REF;
	stcRtcDate.u8Month   = timeptr->tm_mon + 1U;
	stcRtcDate.u8Day     = timeptr->tm_mday;
	stcRtcDate.u8Weekday = timeptr->tm_wday;

	/* Time configuration */
	stcRtcTime.u8Hour   = timeptr->tm_hour;
	stcRtcTime.u8Minute = timeptr->tm_min;
	stcRtcTime.u8Second = timeptr->tm_sec;
	stcRtcTime.u8AmPm   = RTC_HOUR_24H;

	LOG_DBG("Set time: 20%02d/%02d/%02d %02d:%02d:%02d ", stcRtcDate.u8Year,
		stcRtcDate.u8Month, stcRtcDate.u8Day, stcRtcTime.u8Hour, stcRtcTime.u8Minute,
		stcRtcTime.u8Second);

	if (LL_OK != RTC_SetDate(RTC_DATA_FMT_DEC, &stcRtcDate)) {
		LOG_ERR("Set Date failed!");
		err = -EIO;
	}

	if (LL_OK != RTC_SetTime(RTC_DATA_FMT_DEC, &stcRtcTime)) {
		LOG_ERR("Set Time failed!");
		err = -EIO;
	}

	k_mutex_unlock(&data->lock);

	return err;
}

static int rtc_hc32_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	struct rtc_hc32_data *data = dev->data;

	stc_rtc_date_t stcCurrentDate;
	stc_rtc_time_t stcCurrentTime;

	int err = k_mutex_lock(&data->lock, K_NO_WAIT);

	if (err) {
		return err;
	}

	/* Get current date */
	if (LL_OK == RTC_GetDate(RTC_DATA_FMT_DEC, &stcCurrentDate)) {
		/* Get current time */
		if (LL_OK == RTC_GetTime(RTC_DATA_FMT_DEC, &stcCurrentTime)) {
			/* Print current date and time */
			LOG_DBG("get time: 20%02d/%02d/%02d %02d:%02d:%02d ", stcCurrentDate.u8Year,
				stcCurrentDate.u8Month,
				stcCurrentDate.u8Day, stcCurrentTime.u8Hour,
				stcCurrentTime.u8Minute, stcCurrentTime.u8Second);
		} else {
			LOG_ERR("Get time failed!");
			err = -EIO;
		}
	} else {
		LOG_ERR("Get date failed!");
		err = -EIO;
	}

	k_mutex_unlock(&data->lock);

	timeptr->tm_year = stcCurrentDate.u8Year + RTC_YEAR_REF - TM_YEAR_REF;
	/* tm_mon allowed values are 0-11 */
	timeptr->tm_mon = stcCurrentDate.u8Month - 1U;
	timeptr->tm_mday = stcCurrentDate.u8Day;
	timeptr->tm_wday = stcCurrentDate.u8Weekday;
	timeptr->tm_hour = stcCurrentTime.u8Hour;
	timeptr->tm_min = stcCurrentTime.u8Minute;
	timeptr->tm_sec = stcCurrentTime.u8Second;

	/* unknown values */
	timeptr->tm_nsec = 0;
	timeptr->tm_yday  = -1;
	timeptr->tm_isdst = -1;

	return err;
}

#ifdef CONFIG_RTC_ALARM
static int rtc_hc32_alarm_get_supported_fields(const struct device *dev,
					       uint16_t id, uint16_t *mask)
{
	ARG_UNUSED(dev);

	if (id != 0U) {
		LOG_ERR("invalid ID %d", id);
		return -EINVAL;
	}

	*mask = HC32_RTC_ALARM_TIME_MASK;

	return 0;
}

static int rtc_hc32_alarm_set_time(const struct device *dev, uint16_t id,
				   uint16_t mask,
				   const struct rtc_time *timeptr)
{
	stc_rtc_alarm_t stcRtcAlarm;
	ARG_UNUSED(dev);

	if (id != 0U) {
		LOG_ERR("invalid ID %d", id);
		return -EINVAL;
	}

	if ((mask & ~(HC32_RTC_ALARM_TIME_MASK)) != 0U) {
		LOG_ERR("unsupported alarm field mask 0x%04x", mask);
		return -EINVAL;
	}

	if ((mask & RTC_ALARM_TIME_MASK_MINUTE) != 0U) {
		stcRtcAlarm.u8AlarmMinute    = timeptr->tm_min;
	} else {
		stcRtcAlarm.u8AlarmMinute    = 0U;
	}

	if ((mask & RTC_ALARM_TIME_MASK_HOUR) != 0U) {
		stcRtcAlarm.u8AlarmHour    = timeptr->tm_hour;
	} else {
		stcRtcAlarm.u8AlarmHour    = 0U;
	}

	if ((mask & RTC_ALARM_TIME_MASK_WEEKDAY) != 0U) {
		stcRtcAlarm.u8AlarmWeekday = BIT(timeptr->tm_wday);
	} else {
		stcRtcAlarm.u8AlarmWeekday = RTC_ALARM_WEEKDAY_EVERYDAY;
	}
	stcRtcAlarm.u8AlarmAmPm    = RTC_HOUR_24H;

	LOG_DBG("set alarm: wmday = %d, hour = %d, min = %d, mask = 0x%04x",
		stcRtcAlarm.u8AlarmWeekday, stcRtcAlarm.u8AlarmHour, stcRtcAlarm.u8AlarmMinute,
		mask);

	RTC_AlarmCmd(DISABLE);
	RTC_ClearStatus(RTC_FLAG_ALARM);
	if ((mask & HC32_RTC_ALARM_TIME_MASK) != 0U) {
		(void)RTC_SetAlarm(RTC_DATA_FMT_DEC, &stcRtcAlarm);
		RTC_ClearStatus(RTC_FLAG_ALARM);
		RTC_AlarmCmd(ENABLE);
	}

	return 0;
}

static int rtc_hc32_alarm_get_time(const struct device *dev, uint16_t id,
				   uint16_t *mask,
				   struct rtc_time *timeptr)
{
	stc_rtc_alarm_t stcRtcAlarm;
	ARG_UNUSED(dev);

	if (id != 0U) {
		LOG_ERR("invalid ID %d", id);
		return -EINVAL;
	}

	(void)RTC_GetAlarm(RTC_DATA_FMT_DEC, &stcRtcAlarm);

	memset(timeptr, 0U, sizeof(*timeptr));

	*mask = HC32_RTC_ALARM_TIME_MASK;

	timeptr->tm_min = stcRtcAlarm.u8AlarmMinute;
	timeptr->tm_hour = stcRtcAlarm.u8AlarmHour;

	if (stcRtcAlarm.u8AlarmWeekday != RTC_ALARM_WEEKDAY_EVERYDAY) {
		timeptr->tm_wday = stcRtcAlarm.u8AlarmWeekday;
	} else {
		/* weekday alarm not enable */
		*mask &= ~RTC_ALARM_TIME_MASK_WEEKDAY;
	}

	LOG_DBG("get alarm: wday = %d, hour = %d, min = %d, mask = 0x%04x",
		stcRtcAlarm.u8AlarmWeekday, stcRtcAlarm.u8AlarmHour, stcRtcAlarm.u8AlarmMinute,
		*mask);

	return 0;
}

static int rtc_hc32_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct rtc_hc32_data *data = dev->data;
	int pending = 0;

	if (id != 0U) {
		LOG_ERR("invalid ID %d", id);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (SET == RTC_GetStatus(RTC_FLAG_ALARM)) {
		RTC_ClearStatus(RTC_FLAG_ALARM);
		/* Alarm pending */
		pending = 1;
		LOG_DBG("alarm is pending");
	}

	k_mutex_unlock(&data->lock);

	return pending;
}

static int rtc_hc32_alarm_set_callback(const struct device *dev, uint16_t id,
				       rtc_alarm_callback callback, void *user_data)
{
	struct rtc_hc32_data *data = dev->data;

	if (id != 0U) {
		LOG_ERR("invalid ID %d", id);
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->alarm_callback = callback;
	data->alarm_user_data = user_data;

	if (callback != NULL) {
		RTC_IntCmd(RTC_INT_ALARM, ENABLE);
	} else {
		RTC_IntCmd(RTC_INT_ALARM, DISABLE);
	}

	k_mutex_unlock(&data->lock);

	return 0;
}
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_UPDATE
static int rtc_hc32_update_set_callback(const struct device *dev,
					rtc_update_callback callback, void *user_data)
{
	struct rtc_hc32_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	data->update_callback = callback;
	data->update_user_data = user_data;

	if (callback != NULL) {
		RTC_SetIntPeriod(RTC_INT_PERIOD_PER_SEC);
		RTC_IntCmd(RTC_INT_PERIOD, ENABLE);
	} else {
		RTC_IntCmd(RTC_INT_PERIOD, DISABLE);
	}

	k_mutex_unlock(&data->lock);

	return 0;
}
#endif /* CONFIG_RTC_UPDATE */

#ifdef CONFIG_RTC_CALIBRATION
static int rtc_hc32_set_calibration(const struct device *dev,
				    int32_t calibration)
{
	struct rtc_hc32_data *data = dev->data;
	float fPPB, fPPM;
	uint32_t u32Temp;
	uint16_t u16CompRegVal;
	int32_t i32Result = 0;

	fPPB = calibration;
	if ((fPPB < CAL_LIMIT_IN_PPB_LOW) || (fPPB > CAL_LIMIT_IN_PPB_HIGHT)) {
		return -EINVAL;
	}

	fPPB *= 32768 * 32;

	fPPM = PPB_TO_PPM(fPPB);

	i32Result = fPPM / 1000000;
	if (fPPM > 0) {
		i32Result += 0x20;
	} else {
		u32Temp = fabs(i32Result);
		u32Temp = ~u32Temp + 1U;
		u32Temp &= 0x1FFU;
		u32Temp += 0x20U;
		i32Result = u32Temp;

	}
	u16CompRegVal = i32Result & 0x1FFU;

	k_mutex_lock(&data->lock, K_FOREVER);

	RTC_SetClockCompenValue(u16CompRegVal);

	RTC_ClockCompenCmd(ENABLE);

	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_hc32_get_calibration(const struct device *dev,
				    int32_t *calibration)
{
	struct rtc_hc32_data *data = dev->data;
	uint16_t u16CompRegVal;
	float fPPM;

	k_mutex_lock(&data->lock, K_FOREVER);

	u16CompRegVal = READ_REG32(bCM_RTC->ERRCRH_b.COMP8);
	u16CompRegVal = u16CompRegVal << 8U;
	u16CompRegVal |= READ_REG8(CM_RTC->ERRCRL);

	k_mutex_unlock(&data->lock);

	if ((u16CompRegVal > 0xFFU) || (u16CompRegVal < 0x20U)) {
		u16CompRegVal -= 0x20U;
		u16CompRegVal -= 1U;
		u16CompRegVal = ~u16CompRegVal;
		u16CompRegVal &= 0x1FFU;
		fPPM = -(float)u16CompRegVal * 1000000 / 32768 / 32;
	} else {
		u16CompRegVal -= 0x20U;
		fPPM = (float)u16CompRegVal * 1000000 / 32768 / 32;
	}

	*calibration = PPM_TO_PPB(fPPM);

	return 0;
}
#endif /* CONFIG_RTC_CALIBRATION */

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

	/* RTC stopped */
	if (DISABLE == RTC_GetCounterState()) {
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

	rtc_hc32_disable_ints(dev);

	rtc_hc32_irq_config(dev);

	k_mutex_init(&data->lock);

	err = rtc_hc32_configure(dev);

	return err;
}


struct rtc_driver_api rtc_hc32_driver_api = {
	.set_time = rtc_hc32_set_time,
	.get_time = rtc_hc32_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_hc32_alarm_get_supported_fields,
	.alarm_set_time = rtc_hc32_alarm_set_time,
	.alarm_get_time = rtc_hc32_alarm_get_time,
	.alarm_is_pending = rtc_hc32_alarm_is_pending,
	.alarm_set_callback = rtc_hc32_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */
#if CONFIG_RTC_UPDATE
	.update_set_callback = rtc_hc32_update_set_callback,
#endif /* CONFIG_RTC_UPDATE */
#ifdef CONFIG_RTC_CALIBRATION
	.set_calibration = rtc_hc32_set_calibration,
	.get_calibration = rtc_hc32_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */
};

static const struct rtc_hc32_config rtc_config = {
	.clk_src = HC32_DT_RTC_CLK_SRC,
};

static struct rtc_hc32_data rtc_data;

#define RTC_IRQ_CONFIG(inst, idx, irq_hander)                                               \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, idx, irq), DT_INST_IRQ_BY_IDX(inst, idx, priority),\
		    irq_hander, DEVICE_DT_INST_GET(inst), 0);                                       \
	hc32_intc_irq_signin(DT_INST_IRQ_BY_IDX(inst, idx, irq),                                \
			     DT_INST_IRQ_BY_IDX(inst, idx, int_src));                                   \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, idx, irq));                                         \

static void rtc_hc32_irq_config(const struct device *dev)
{
	ARG_UNUSED(dev);
#ifdef CONFIG_RTC_ALARM
	/* ALM IRQ: idx 0 */
	RTC_IRQ_CONFIG(0, 0, rtc_hc32_alarm_irq_handler);
#endif

#ifdef CONFIG_RTC_UPDATE
	/* PRD IRQ: idx 1 */
	RTC_IRQ_CONFIG(0, 1, rtc_hc32_prd_irq_handler);
#endif
}

DEVICE_DT_INST_DEFINE(0, &rtc_hc32_init, NULL, &rtc_data, &rtc_config,
		      PRE_KERNEL_1,
		      CONFIG_RTC_INIT_PRIORITY, &rtc_hc32_driver_api);
