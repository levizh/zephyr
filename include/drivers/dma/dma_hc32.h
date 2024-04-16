/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_HC32_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_HC32_H_

#include <dt-bindings/dma/hc32_dma.h>

#define HC32_DT_GET_DMA_SLOT(cfg)         ((cfg >> 12U) & 0xFFFUL)
#define HC32_DT_GET_DMA_UNIT(cfg)         ((cfg >> 8U) & 0xFUL)
#define HC32_DT_GET_DMA_CH(cfg)           ((cfg >> 0U) & 0xFFUL)

struct dma_hc32_config_user_data {
	u32_t slot; /* hc32 mcu uses this value to store the aos source that triggered the dma */
	void *user_data; /* private data from DMA client */
};

static inline const char *dma_hc32_get_device_name(u32_t unit)
{
	switch (unit) {
	case DMA1:
		return DMA1_LABEL_NAME;
	case DMA2:
		return DMA2_LABEL_NAME;
	default:
		return NULL;
	}
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_HC32_H_ */
