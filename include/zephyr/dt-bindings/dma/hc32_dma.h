/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_HC32_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_HC32_DMA_H_


/* macros for channel-cfg */
/* direction defined on bits 4-5 */
#define HC32_DMA_CH_CFG_DIRECTION(val)       ((val & 0x3) << 4)
#define HC32_DMA_MEMORY_TO_MEMORY            HC32_DMA_CH_CFG_DIRECTION(0)
#define HC32_DMA_MEMORY_TO_PERIPH            HC32_DMA_CH_CFG_DIRECTION(1)
#define HC32_DMA_PERIPH_TO_MEMORY            HC32_DMA_CH_CFG_DIRECTION(2)

/* source address increase defined on bits 6-7 */
#define HC32_DMA_CH_CFG_SOURCE_ADDR_INC(val) ((val & 0x3) << 6)
#define HC32_DMA_SOURCE_ADDR_INC             HC32_DMA_CH_CFG_SOURCE_ADDR_INC(0)
#define HC32_DMA_SOURCE_ADDR_DEC             HC32_DMA_CH_CFG_SOURCE_ADDR_INC(1)
#define HC32_DMA_SOURCE_ADDR_FIX             HC32_DMA_CH_CFG_SOURCE_ADDR_INC(2)

/* dest address increase defined on bits 8-9 */
#define HC32_DMA_CH_CFG_DEST_ADDR_INC(val)  ((val & 0x3) << 8)
#define HC32_DMA_DEST_ADDR_INC              HC32_DMA_CH_CFG_DEST_ADDR_INC(0)
#define HC32_DMA_DEST_ADDR_DEC              HC32_DMA_CH_CFG_DEST_ADDR_INC(1)
#define HC32_DMA_DEST_ADDR_FIX              HC32_DMA_CH_CFG_DEST_ADDR_INC(2)

/* data size defined on bits 10-11 */
#define HC32_DMA_CH_CFG_DATA_WIDTH(val)     ((val & 0x3) << 10)
#define HC32_DMA_DATA_WIDTH_8BIT            HC32_DMA_CH_CFG_DATA_WIDTH(0)
#define HC32_DMA_DATA_WIDTH_16BIT           HC32_DMA_CH_CFG_DATA_WIDTH(1)
#define HC32_DMA_DATA_WIDTH_32BIT           HC32_DMA_CH_CFG_DATA_WIDTH(2)



#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_HC32_DMA_H_ */
