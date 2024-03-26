/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_QDEC_HC32_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_QDEC_HC32_H_

#include <zephyr/drivers/sensor.h>

enum sensor_attribute_qdec_hc32 {
	/* Number of counts per revolution */
	SENSOR_ATTR_QDEC_MODE_VAL = SENSOR_ATTR_PRIV_START,
	/* Single phase counting */
	SENSOR_ATTR_QDEC_CNT_CONDITION,
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_QDEC_HC32_H_ */
