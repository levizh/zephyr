/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_WATCHDOG_HC32_H_
#define ZEPHYR_DRIVERS_WATCHDOG_HC32_H_

#include <zephyr/types.h>


/* driver data */
struct wdt_hc32_data {
	uint32_t ClockDiv;
	uint32_t CountPeriod;
	uint8_t  RunState;
};

#define WDT_HC32_DATA(dev)      ((struct wdt_hc32_data *const)(dev)->data)


#endif  /* ZEPHYR_DRIVERS_WATCHDOG_HC32_H_ */
