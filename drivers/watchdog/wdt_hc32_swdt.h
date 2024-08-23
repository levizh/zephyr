/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SPECIALIZED_WATCHDOG_HC32_H_
#define ZEPHYR_DRIVERS_SPECIALIZED_WATCHDOG_HC32_H_

#include <zephyr/types.h>


/* driver data */
struct swdt_hc32_data {
	uint32_t ClockDiv;
	uint32_t CountPeriod;
	uint8_t  RunState;
};

#define SWDT_HC32_DATA(dev)     ((struct swdt_hc32_data *const)(dev)->data)


#endif  /* ZEPHYR_DRIVERS_SPECIALIZED_WATCHDOG_HC32_H_ */
