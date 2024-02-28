/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_HC32_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_HC32_H_

#define HC32_DMA_CONFIG_DIRECTION(config)			  ((config >> 6) & 0x3)
#define HC32_DMA_CONFIG_SOURCE_ADDR_INC(config)		  ((config >> 9) & 0x1)
#define HC32_DMA_CONFIG_DEST_ADDR_INC(config)	      ((config >> 10) & 0x1)
#define HC32_DMA_CONFIG_DATA_WIDTH(config)	          ((config >> 11) & 0x3)

struct dma_hc32_config_user_data {
	uint32_t slot; /* hc32 mcu uses this value to store the aos source that triggered the dma */
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_HC32_H_ */
