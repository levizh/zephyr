/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_HC32_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_HC32_H_


#define HC32_DT_GET_DMA_SLOT(cfg)         ((cfg >> 12U) & 0xFFFUL)
#define HC32_DT_GET_DMA_UNIT(cfg)         ((cfg >> 8U) & 0xFU)
#define HC32_DT_GET_DMA_CH(cfg)           ((cfg >> 0U) & 0xFFU)

struct dma_hc32_config_user_data {
	u32_t slot; /* hc32 mcu uses this value to store the aos source that triggered the dma */
	void *user_data; /* private data from DMA client */
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_HC32_H_ */
