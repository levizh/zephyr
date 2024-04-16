/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_DMA_HC32_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_DMA_HC32_DMA_H_


#define DMA1                (1U)
#define DMA1_LABEL_NAME     "DMA_1"

#define DMA2                (2U)
#define DMA2_LABEL_NAME     "DMA_2"

/* macros for dmas cfg in dts:
   |--31:24--|----23:12---|---11:8----|---7:0---|
   |---REV---|----SLOT----|---UNIT----|---CH----|
 */

#define HC32_DT_DMA_SLOT(val)         ((val & 0xFFFUL) << 12U)
#define HC32_DT_DMA_UNIT(val)         ((val & 0xFUL) << 8U)
#define HC32_DT_DMA_CH(val)           ((val & 0xFFUL) << 0U)

/* dts bindings example:
    ...
    dma-tx-cfg:
      type: int
      description: DMA tx config
      generation: define
      category: optional

    dma-rx-cfg:
      type: int
      description: DMA rx config
      generation: define
      category: optional
    ...
 */

/* dts example:

    usart4: serial@40021400 {
        ...
      dma-tx-cfg = <(HC32_DT_DMA_UNIT(DMA1) | HC32_DT_DMA_CH(0) |
                     HC32_DT_DMA_SLOT(EVT_SRC_USART4_TI))>;
      dma-rx-cfg = <(HC32_DT_DMA_UNIT(DMA2) | HC32_DT_DMA_CH(2) |
                     HC32_DT_DMA_SLOT(EVT_SRC_USART4_RI))>;
        ...
	}

 */


#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_HC32_DMA_H_ */
