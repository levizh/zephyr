/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* SoC level DTS fixup file */

#define DT_NUM_IRQ_PRIO_BITS            DT_ARM_V7M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#if defined(DT_XHSC_HC32_RTC_0_LABEL)
#define DT_RTC_0_NAME                   DT_XHSC_HC32_RTC_0_LABEL
#endif

#if defined(DT_XHSC_HC32_CAN_40070400_LABEL)
#define DT_CAN_1_NAME                   DT_XHSC_HC32_CAN_40070400_LABEL
#endif

/* End of SoC Level DTS fixup file */
