/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_HC32_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_HC32_H_

struct dma_hc32_config_user_data {
	u32_t slot; /* hc32 mcu uses this value to store the aos source that triggered the dma */
	void *user_data; /* private data from DMA client */
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_HC32_H_ */
