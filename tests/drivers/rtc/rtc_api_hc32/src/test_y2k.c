/*
 * Copyright 2023 Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>

#include <time.h>

/* date "+%s" -d "Wed Jan  1 2020 00:00:00 GMT+0000" */
#define Y20_STAMP 1577836800UL

#define SECONDS_BEFORE 1
#define SECONDS_AFTER 1

#define RTC_TEST_START_TIME (Y20_STAMP - SECONDS_BEFORE)
#define RTC_TEST_STOP_TIME (Y20_STAMP + SECONDS_AFTER)

static const struct device *rtc = DEVICE_DT_GET(DT_ALIAS(rtc));

ZTEST(rtc_api, test_y2k)
{
	enum test_time {
		Y19,
		Y20,
	};

	static struct rtc_time rtm[2];
	struct tm *const tm[2] = {
		(struct tm *const)&rtm[0],
		(struct tm *const)&rtm[1],
	};
	const time_t t[] = {
		[Y19] = RTC_TEST_START_TIME,
		[Y20] = RTC_TEST_STOP_TIME,
	};

	/* Party like it's 1999 */
	zassert_not_null(gmtime_r(&t[Y19], tm[Y19]));
	zassert_ok(rtc_set_time(rtc, &rtm[Y19]));

	/* Living after midnight */
	k_sleep(K_SECONDS(SECONDS_BEFORE + SECONDS_AFTER));
	zassert_ok(rtc_get_time(rtc, &rtm[Y20]));

	/* It's the end of the world as we know it */
	zassert_equal(rtm[Y20].tm_year + 1900, 2020, "wrong year: %d", rtm[Y20].tm_year + 1900);
	zassert_equal(rtm[Y20].tm_mon, 0, "wrong month: %d", rtm[Y20].tm_mon);
	zassert_equal(rtm[Y20].tm_mday, 1, "wrong day-of-month: %d", rtm[Y20].tm_mday);
	//zassert_equal(rtm[Y20].tm_yday, 0, "wrong day-of-year: %d", rtm[Y20].tm_yday);
	zassert_equal(rtm[Y20].tm_wday, 3, "wrong day-of-week: %d", rtm[Y20].tm_wday);
	zassert_equal(rtm[Y20].tm_hour, 0, "wrong hour: %d", rtm[Y20].tm_hour);
	zassert_equal(rtm[Y20].tm_min, 0, "wrong minute: %d", rtm[Y20].tm_min);
	zassert_equal(rtm[Y20].tm_sec, SECONDS_AFTER, "wrong second: %d", rtm[Y20].tm_sec);
}
