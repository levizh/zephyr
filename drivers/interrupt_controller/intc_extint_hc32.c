/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <soc.h>
#include <misc/__assert.h>
#include "intc_hc32.h"


// /* device name */
// #define HC32_EXTI_NAME          "xhsc_hc32_extint"

// #define EXTI_LINES 23


// /* wrapper for user callback */
// struct __exti_cb {
// 	hc32_exti_callback_t cb;
// 	void *data;
// };

// /* driver data */
// struct hc32_exti_data {
// 	/* per-line callbacks */
// 	struct __exti_cb cb[EXTI_LINES];
// };

// int hc32_exti_enable(int line)
// {
// 	int irqnum = 0;

// 	if (line < 32) {
// 		LL_EXTI_EnableIT_0_31(1 << line);
// 	} else {
// 		__ASSERT_NO_MSG(line);
// 	}

// 	if (line >= 5 && line <= 9) {
// 		irqnum = EXTI9_5_IRQn;
// 	} else if (line >= 10 && line <= 15) {
// 		irqnum = EXTI15_10_IRQn;
// 	} else if (line >= 0 && line <= 4) {
// 		/* pins 0..4 are mapped to EXTI0.. EXTI4 */
// 		irqnum = EXTI0_IRQn + line;
// 	} else {
// 		switch (line) {
// 		case 16:
// 			irqnum = PVD_IRQn;
// 			break;
// 		case 18:
// 			irqnum = OTG_FS_WKUP_IRQn;
// 			break;
// 		case 21:
// 			irqnum = TAMP_STAMP_IRQn;
// 			break;
// 		case 22:
// 			irqnum = RTC_WKUP_IRQn;
// 			break;
// 		default:
// 			/* No IRQ associated to this line */
// 			return -ENOTSUP;
// 		}
// 	}

// 	irq_enable(irqnum);

// 	return 0;
// }

// void hc32_exti_disable(int line)
// {
// 	if (line < 32) {
// 		LL_EXTI_DisableIT_0_31(1 << line);
// 	} else {
// 		__ASSERT_NO_MSG(line);
// 	}
// }

// /**
//  * @brief check if interrupt is pending
//  *
//  * @param line line number
//  */
// static inline int hc32_exti_is_pending(int line)
// {
// 	if (line < 32) {
// 		return LL_EXTI_IsActiveFlag_0_31(1 << line);
// 	} else {
// 		__ASSERT_NO_MSG(line);
// 		return 0;
// 	}
// }

// /**
//  * @brief clear pending interrupt bit
//  *
//  * @param line line number
//  */
// static inline void hc32_exti_clear_pending(int line)
// {
// 	if (line < 32) {
// 		LL_EXTI_ClearFlag_0_31(1 << line);
// 	} else {
// 		__ASSERT_NO_MSG(line);
// 	}
// }

// void hc32_exti_trigger(int line, int trigger)
// {
// 	if (trigger & HC32_EXTI_TRIG_RISING) {
// 		if (line < 32) {
// 			LL_EXTI_EnableRisingTrig_0_31(1 << line);
// 		} else {
// 			__ASSERT_NO_MSG(line);
// 		}
// 	}

// 	if (trigger & HC32_EXTI_TRIG_FALLING) {
// 		if (line < 32) {
// 			LL_EXTI_EnableFallingTrig_0_31(1 << line);
// 		} else {
// 			__ASSERT_NO_MSG(line);
// 		}
// 	}
// }

// /**
//  * @brief EXTI ISR handler
//  *
//  * Check EXTI lines in range @min @max for pending interrupts
//  *
//  * @param arg isr argument
//  * @param min low end of EXTI# range
//  * @param max low end of EXTI# range
//  */
// static void __hc32_exti_isr(int min, int max, void *arg)
// {
// 	struct device *dev = arg;
// 	struct hc32_exti_data *data = dev->driver_data;
// 	int line;

// 	/* see which bits are set */
// 	for (line = min; line < max; line++) {
// 		/* check if interrupt is pending */
// 		if (hc32_exti_is_pending(line)) {
// 			/* clear pending interrupt */
// 			hc32_exti_clear_pending(line);

// 			/* run callback only if one is registered */
// 			if (!data->cb[line].cb) {
// 				continue;
// 			}

// 			data->cb[line].cb(line, data->cb[line].data);
// 		}
// 	}
// }


// static inline void __hc32_exti_isr_0(void *arg)
// {
// 	__hc32_exti_isr(0, 1, arg);
// }

// static inline void __hc32_exti_isr_1(void *arg)
// {
// 	__hc32_exti_isr(1, 2, arg);
// }

// static inline void __hc32_exti_isr_2(void *arg)
// {
// 	__hc32_exti_isr(2, 3, arg);
// }

// static inline void __hc32_exti_isr_3(void *arg)
// {
// 	__hc32_exti_isr(3, 4, arg);
// }

// static inline void __hc32_exti_isr_4(void *arg)
// {
// 	__hc32_exti_isr(4, 5, arg);
// }

// static inline void __hc32_exti_isr_9_5(void *arg)
// {
// 	__hc32_exti_isr(5, 10, arg);
// }

// static inline void __hc32_exti_isr_15_10(void *arg)
// {
// 	__hc32_exti_isr(10, 16, arg);
// }


// static inline void __hc32_exti_isr_16(void *arg)
// {
// 	__hc32_exti_isr(16, 17, arg);
// }

// static inline void __hc32_exti_isr_18(void *arg)
// {
// 	__hc32_exti_isr(18, 19, arg);
// }

// static inline void __hc32_exti_isr_21(void *arg)
// {
// 	__hc32_exti_isr(21, 22, arg);
// }

// static inline void __hc32_exti_isr_22(void *arg)
// {
// 	__hc32_exti_isr(22, 23, arg);
// }


// static void __hc32_exti_connect_irqs(struct device *dev);

// /**
//  * @brief initialize EXTI device driver
//  */
// static int hc32_exti_init(struct device *dev)
// {
// 	__hc32_exti_connect_irqs(dev);

// 	return 0;
// }

// static struct hc32_exti_data exti_data;
// DEVICE_INIT(exti_hc32, HC32_EXTI_NAME, hc32_exti_init,
// 	    &exti_data, NULL,
// 	    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

// /**
//  * @brief set & unset for the interrupt callbacks
//  */
// int hc32_exti_set_callback(int line, int port, hc32_exti_callback_t cb,
// 			   void *arg)
// {
// 	struct device *dev = DEVICE_GET(exti_hc32);
// 	struct hc32_exti_data *data = dev->driver_data;

// 	if (data->cb[line].cb) {
// 		return -EBUSY;
// 	}

// 	data->cb[line].cb = cb;
// 	data->cb[line].data = arg;

// 	return 0;
// }

// void hc32_exti_unset_callback(int line)
// {
// 	struct device *dev = DEVICE_GET(exti_hc32);
// 	struct hc32_exti_data *data = dev->driver_data;

// 	data->cb[line].cb = NULL;
// 	data->cb[line].data = NULL;
// }

// /**
//  * @brief connect all interrupts
//  */
// static void __hc32_exti_connect_irqs(struct device *dev)
// {
// 	ARG_UNUSED(dev);

// 	IRQ_CONNECT(EXTI0_IRQn,
// 		    CONFIG_EXTI_HC32_EXTI0_IRQ_PRI,
// 		    __hc32_exti_isr_0, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(EXTI1_IRQn,
// 		    CONFIG_EXTI_HC32_EXTI1_IRQ_PRI,
// 		    __hc32_exti_isr_1, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(EXTI2_IRQn,
// 		    CONFIG_EXTI_HC32_EXTI2_IRQ_PRI,
// 		    __hc32_exti_isr_2, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(EXTI3_IRQn,
// 		    CONFIG_EXTI_HC32_EXTI3_IRQ_PRI,
// 		    __hc32_exti_isr_3, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(EXTI4_IRQn,
// 		    CONFIG_EXTI_HC32_EXTI4_IRQ_PRI,
// 		    __hc32_exti_isr_4, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(EXTI9_5_IRQn,
// 		    CONFIG_EXTI_HC32_EXTI9_5_IRQ_PRI,
// 		    __hc32_exti_isr_9_5, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(EXTI15_10_IRQn,
// 		    CONFIG_EXTI_HC32_EXTI15_10_IRQ_PRI,
// 		    __hc32_exti_isr_15_10, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(PVD_IRQn,
// 		    CONFIG_EXTI_HC32_PVD_IRQ_PRI,
// 		    __hc32_exti_isr_16, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(OTG_FS_WKUP_IRQn,
// 		    CONFIG_EXTI_HC32_OTG_FS_WKUP_IRQ_PRI,
// 		    __hc32_exti_isr_18, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(TAMP_STAMP_IRQn,
// 		    CONFIG_EXTI_HC32_TAMP_STAMP_IRQ_PRI,
// 		    __hc32_exti_isr_21, DEVICE_GET(exti_hc32),
// 		    0);
// 	IRQ_CONNECT(RTC_WKUP_IRQn,
// 		    CONFIG_EXTI_HC32_RTC_WKUP_IRQ_PRI,
// 		    __hc32_exti_isr_22, DEVICE_GET(exti_hc32),
// 		    0);
// }
