/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_HC32_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_HC32_H_

/* get dma node_id */
#define HC32_DMA_CTLR(id, dir)						\
		DT_INST_DMAS_CTLR_BY_NAME(id, dir)

/* get dma cell: channel */
#define HC32_DMA_CHANNEL(id, dir)						\
		DT_INST_DMAS_CELL_BY_NAME(id, dir, channel)

/* get dma cell: slot */
#define HC32_DMA_SLOT(id, dir)				\
		DT_INST_DMAS_CELL_BY_NAME(id, dir, slot)

/* get dma cell: channel-config */
#define HC32_DMA_CHANNEL_CONFIG(id, dir)					\
		DT_INST_DMAS_CELL_BY_NAME(id, dir, channel_config)

/* ref dma.h */

/* 0 -> MEM_TO_MEM, 1 -> MEM_TO_PERIPH, 2 -> PERIPH_TO_MEM */
#define HC32_DMA_CONFIG_DIRECTION(config)             ((config >> 4) & 0x3)
/* 0 -> inc, 1 -> dec, 2 -> no change */
#define HC32_DMA_CONFIG_SOURCE_ADDR_INC(config)       ((config >> 6) & 0x3)
#define HC32_DMA_CONFIG_DEST_ADDR_INC(config)         ((config >> 8) & 0x3)
/* 0 -> 1 byte, 1 -> 2 bytes, 2 -> 4 bytes */
#define HC32_DMA_CONFIG_DATA_SIZE(config)             (1 << ((config >> 10) & 0x3))

struct dma_hc32_config_user_data {
	uint32_t slot; /* hc32 mcu uses this value to store the aos source that triggered the dma */
	void *user_data; /* private data from DMA client */
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_HC32_H_ */
