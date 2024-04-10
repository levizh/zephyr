/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_DMA_HC32_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_DMA_HC32_DMA_H_


#define DMA1            (1U)
#define DMA2            (2U)

/* macros for dmas cfg in dts:
   |--31:28--|---27:24---|--23:16--|--15:12--|---11:8----|---7:0---|
   |---REV---|--TX_UNIT--|--TX_CH--|---REV---|--RX_UNIT--|--RX_CH--|
 */

#define HC32_PERIPH_TX_DMA_UNIT(val)         ((val & 0xFU) << 24U)
#define HC32_PERIPH_TX_DMA_CH(val)           ((val & 0xFFU) << 16U)

#define HC32_PERIPH_RX_DMA_UNIT(val)         ((val & 0xFU) << 8U)
#define HC32_PERIPH_TX_DMA_CH(val)           ((val & 0xFFU) << 0U)

/* dts bindings example:
    ...
    dmas:
      type: int
      description: DMA config
      generation: define
      category: optional
    ...
 */

/* dts example:

    usart4: serial@40021400 {
        ...
        dmas = <(HC32_PERIPH_TX_DMA_UNIT(DMA1) | HC32_PERIPH_TX_DMA_CH(0) |
                 HC32_PERIPH_RX_DMA_UNIT(DMA2) | HC32_PERIPH_TX_DMA_CH(2))>;
        ...
	}

 */


#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_HC32_DMA_H_ */
