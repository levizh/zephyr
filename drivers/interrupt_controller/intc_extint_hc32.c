/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <soc.h>
#include <misc/__assert.h>
#include "intc_hc32.h"

#define LOG_LEVEL CONFIG_INTC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(intc_extint_hc32);


#define INTC_IRQ_ARRAY_BY_IDX(index, offset, prop)          \
        DT_XHSC_HC32_EXTINT_##index##_##IRQ##_##offset##_##prop
#define INTC_IRQ_SINGLE_BY_IDX(index, offset)               \
        DT_XHSC_HC32_EXTINT_##index##_##IRQ##_##offset

#define INTC_PROP_ARRAY_BY_IDX(index, prop, offset)         \
        DT_XHSC_HC32_EXTINT_##index##_##prop##_##offset
#define INTC_PROP_SINGLE_BY_IDX(index, prop)                \
        DT_XHSC_HC32_EXTINT_##index##_##prop

/* device name */
#define INTC_EXTINT_DEVICE_NAME         intc_extint_hc32
#define INTC_EXTINT_DRIVER_NAME         INTC_PROP_SINGLE_BY_IDX(0, LABEL)
#define INTC_EXTINT_NUM                 INTC_PROP_SINGLE_BY_IDX(0, EXTINT_NUMS)
#define INTC_EXTINT_IRQN_DEF            0xFF

static IRQn_Type extint_irq_table[INTC_EXTINT_NUM] = {[0 ... INTC_EXTINT_NUM - 1] = INTC_EXTINT_IRQN_DEF};

/* wrapper for user callback */
struct hc32_extint_cb {
	hc32_extint_callback_t cb;
	void *user;
};

/* driver data */
struct hc32_extint_data {
	/* callbacks */
	struct hc32_extint_cb cb[INTC_EXTINT_NUM];
	uint32_t extint_table;
};



/* Configure irq information */
static void hc32_irq_config(uint8_t ch, int irqn, int intsrc)
{
	/* fill irq table */
	extint_irq_table[ch] = (IRQn_Type)irqn;
#if defined(CONFIG_INTC_EXTINT_USE_SHARE_INTERRUPT)
	INTC_ShareIrqCmd(intsrc, ENABLE);
#else
	hc32_intc_irq_signin(irqn, intsrc);


#endif
}

static void hc32_extint_isr(const void *arg);
/**
 * @brief connect all interrupts
 */
static void hc32_extint_connect_irq(struct device *dev)
{
	ARG_UNUSED(dev);
#if defined(CONFIG_INTC_EXTINT_USE_SHARE_INTERRUPT)
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 0),
			INTC_IRQ_SINGLE_BY_IDX(0, 0), INTC_IRQ_ARRAY_BY_IDX(0, 0, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 1),
			INTC_IRQ_SINGLE_BY_IDX(0, 1), INTC_IRQ_ARRAY_BY_IDX(0, 1, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 2),
			INTC_IRQ_SINGLE_BY_IDX(0, 2), INTC_IRQ_ARRAY_BY_IDX(0, 2, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 3),
			INTC_IRQ_SINGLE_BY_IDX(0, 3), INTC_IRQ_ARRAY_BY_IDX(0, 3, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 4),
			INTC_IRQ_SINGLE_BY_IDX(0, 4), INTC_IRQ_ARRAY_BY_IDX(0, 4, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 5),
			INTC_IRQ_SINGLE_BY_IDX(0, 5), INTC_IRQ_ARRAY_BY_IDX(0, 5, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 6),
			INTC_IRQ_SINGLE_BY_IDX(0, 6), INTC_IRQ_ARRAY_BY_IDX(0, 6, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 7),
			INTC_IRQ_SINGLE_BY_IDX(0, 7), INTC_IRQ_ARRAY_BY_IDX(0, 7, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 8),
			INTC_IRQ_SINGLE_BY_IDX(0, 8), INTC_IRQ_ARRAY_BY_IDX(0, 8, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 9),
			INTC_IRQ_SINGLE_BY_IDX(0, 9), INTC_IRQ_ARRAY_BY_IDX(0, 9, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 10),
			INTC_IRQ_SINGLE_BY_IDX(0, 10), INTC_IRQ_ARRAY_BY_IDX(0, 10, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 11),
			INTC_IRQ_SINGLE_BY_IDX(0, 11), INTC_IRQ_ARRAY_BY_IDX(0, 11, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 12),
			INTC_IRQ_SINGLE_BY_IDX(0, 12), INTC_IRQ_ARRAY_BY_IDX(0, 12, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 13),
			INTC_IRQ_SINGLE_BY_IDX(0, 13), INTC_IRQ_ARRAY_BY_IDX(0, 13, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 14),
			INTC_IRQ_SINGLE_BY_IDX(0, 14), INTC_IRQ_ARRAY_BY_IDX(0, 14, INT_SRC));
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 15),
			INTC_IRQ_SINGLE_BY_IDX(0, 15), INTC_IRQ_ARRAY_BY_IDX(0, 15, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 0),
		    INTC_IRQ_ARRAY_BY_IDX(0, 0, PRIORITY), hc32_extint_isr, NULL, 0);
#else
	static const uint8_t extint_ch_0 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 0);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 0),
			INTC_IRQ_SINGLE_BY_IDX(0, 0), INTC_IRQ_ARRAY_BY_IDX(0, 0, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 0),
		    INTC_IRQ_ARRAY_BY_IDX(0, 0, PRIORITY), hc32_extint_isr, &extint_ch_0, 0);
	static const uint8_t extint_ch_1 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 1);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 1),
			INTC_IRQ_SINGLE_BY_IDX(0, 1), INTC_IRQ_ARRAY_BY_IDX(0, 1, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 1),
		    INTC_IRQ_ARRAY_BY_IDX(0, 1, PRIORITY), hc32_extint_isr, &extint_ch_1, 0);
	static const uint8_t extint_ch_2 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 2);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 2),
			INTC_IRQ_SINGLE_BY_IDX(0, 2), INTC_IRQ_ARRAY_BY_IDX(0, 2, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 2),
		    INTC_IRQ_ARRAY_BY_IDX(0, 2, PRIORITY), hc32_extint_isr, &extint_ch_2, 0);
	static const uint8_t extint_ch_3 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 3);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 3),
			INTC_IRQ_SINGLE_BY_IDX(0, 3), INTC_IRQ_ARRAY_BY_IDX(0, 3, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 3),
		    INTC_IRQ_ARRAY_BY_IDX(0, 3, PRIORITY), hc32_extint_isr, &extint_ch_3, 0);
	static const uint8_t extint_ch_4 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 4);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 4),
			INTC_IRQ_SINGLE_BY_IDX(0, 4), INTC_IRQ_ARRAY_BY_IDX(0, 4, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 4),
		    INTC_IRQ_ARRAY_BY_IDX(0, 4, PRIORITY), hc32_extint_isr, &extint_ch_4, 0);
	static const uint8_t extint_ch_5 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 5);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 5),
			INTC_IRQ_SINGLE_BY_IDX(0, 5), INTC_IRQ_ARRAY_BY_IDX(0, 5, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 5),
		    INTC_IRQ_ARRAY_BY_IDX(0, 5, PRIORITY), hc32_extint_isr, &extint_ch_5, 0);
	static const uint8_t extint_ch_6 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 6);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 6),
			INTC_IRQ_SINGLE_BY_IDX(0, 6), INTC_IRQ_ARRAY_BY_IDX(0, 6, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 6),
		    INTC_IRQ_ARRAY_BY_IDX(0, 6, PRIORITY), hc32_extint_isr, &extint_ch_6, 0);
	static const uint8_t extint_ch_7 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 7);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 7),
			INTC_IRQ_SINGLE_BY_IDX(0, 7), INTC_IRQ_ARRAY_BY_IDX(0, 7, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 7),
		    INTC_IRQ_ARRAY_BY_IDX(0, 7, PRIORITY), hc32_extint_isr, &extint_ch_7, 0);
	static const uint8_t extint_ch_8 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 8);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 8),
			INTC_IRQ_SINGLE_BY_IDX(0, 8), INTC_IRQ_ARRAY_BY_IDX(0, 8, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 8),
		    INTC_IRQ_ARRAY_BY_IDX(0, 8, PRIORITY), hc32_extint_isr, &extint_ch_8, 0);
	static const uint8_t extint_ch_9 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 9);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 9),
			INTC_IRQ_SINGLE_BY_IDX(0, 9), INTC_IRQ_ARRAY_BY_IDX(0, 9, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 9),
		    INTC_IRQ_ARRAY_BY_IDX(0, 9, PRIORITY), hc32_extint_isr, &extint_ch_9, 0);
	static const uint8_t extint_ch_10 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 10);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 10),
			INTC_IRQ_SINGLE_BY_IDX(0, 10), INTC_IRQ_ARRAY_BY_IDX(0, 10, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 10),
		    INTC_IRQ_ARRAY_BY_IDX(0, 10, PRIORITY), hc32_extint_isr, &extint_ch_10, 0);
	static const uint8_t extint_ch_11 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 11);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 11),
			INTC_IRQ_SINGLE_BY_IDX(0, 11), INTC_IRQ_ARRAY_BY_IDX(0, 11, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 11),
		    INTC_IRQ_ARRAY_BY_IDX(0, 11, PRIORITY), hc32_extint_isr, &extint_ch_11, 0);
	static const uint8_t extint_ch_12 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 12);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 12),
			INTC_IRQ_SINGLE_BY_IDX(0, 12), INTC_IRQ_ARRAY_BY_IDX(0, 12, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 12),
		    INTC_IRQ_ARRAY_BY_IDX(0, 12, PRIORITY), hc32_extint_isr, &extint_ch_12, 0);
	static const uint8_t extint_ch_13 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 13);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 13),
			INTC_IRQ_SINGLE_BY_IDX(0, 13), INTC_IRQ_ARRAY_BY_IDX(0, 13, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 13),
		    INTC_IRQ_ARRAY_BY_IDX(0, 13, PRIORITY), hc32_extint_isr, &extint_ch_13, 0);
	static const uint8_t extint_ch_14 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 14);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 14),
			INTC_IRQ_SINGLE_BY_IDX(0, 14), INTC_IRQ_ARRAY_BY_IDX(0, 14, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 14),
		    INTC_IRQ_ARRAY_BY_IDX(0, 14, PRIORITY), hc32_extint_isr, &extint_ch_14, 0);
	static const uint8_t extint_ch_15 = INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 15);
	hc32_irq_config(INTC_PROP_ARRAY_BY_IDX(0, EXTINT_CHS, 15),
			INTC_IRQ_SINGLE_BY_IDX(0, 15), INTC_IRQ_ARRAY_BY_IDX(0, 15, INT_SRC));
	IRQ_CONNECT(INTC_IRQ_SINGLE_BY_IDX(0, 15),
		    INTC_IRQ_ARRAY_BY_IDX(0, 15, PRIORITY), hc32_extint_isr, &extint_ch_15, 0);
#endif
}

/**
 * @brief initialize intc extint device driver
 */
static int hc32_intc_extint_init(struct device *dev)
{
	struct hc32_extint_data *data = dev->driver_data;

	data->extint_table = 0;
	hc32_extint_connect_irq(dev);

	return 0;
}

static struct hc32_extint_data extint_data;
DEVICE_INIT(INTC_EXTINT_DEVICE_NAME, INTC_EXTINT_DRIVER_NAME,
	    hc32_intc_extint_init, &extint_data, NULL,
	    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);


/**
 * @brief EXTINT ISR handler
 */
static void hc32_extint_isr(const void *arg)
{
	struct device *dev = DEVICE_GET(INTC_EXTINT_DEVICE_NAME);
	struct hc32_extint_data *data = dev->driver_data;

#if defined(CONFIG_INTC_EXTINT_USE_SHARE_INTERRUPT)
	uint32_t ch_idx;

	ARG_UNUSED(arg);
	for (ch_idx = 0; ch_idx < INTC_EXTINT_NUM; ch_idx++) {
		if (SET == EXTINT_GetExtIntStatus(BIT(ch_idx))) {
			EXTINT_ClearExtIntStatus(BIT(ch_idx));
			/* run callback only if one is registered */
			if (data->cb[ch_idx].cb != NULL) {
				data->cb[ch_idx].cb(ch_idx, data->cb[ch_idx].user);
			}
		}
	}
#else
	uint32_t ch = *(uint8_t *)arg;

	if (SET == EXTINT_GetExtIntStatus(BIT(ch))) {
		EXTINT_ClearExtIntStatus(BIT(ch));
		/* run callback only if one is registered */
		if (data->cb[ch].cb != NULL) {
			data->cb[ch].cb(ch, data->cb[ch].user);
		}
	}
#endif
}

int hc32_extint_enable(int port, int pin)
{
	int irqnum = 0;
	struct device *dev = DEVICE_GET(INTC_EXTINT_DEVICE_NAME);
	struct hc32_extint_data *data = dev->driver_data;

	if (pin >= INTC_EXTINT_NUM) {
		__ASSERT_NO_MSG(pin);
	}
	/* Get matching extint irq number from irq_table */
	irqnum = extint_irq_table[pin];
	if (irqnum == INTC_EXTINT_IRQN_DEF) {
		__ASSERT_NO_MSG(pin);
	}
	/* if pin interrupt already enable return busy */
	if ((data->extint_table & BIT((uint32_t)pin)) != 0) {
		return -EBUSY;
	}

	/* Enable requested pin interrupt */
	GPIO_ExtIntCmd((uint8_t)port, BIT((uint32_t)pin), ENABLE);
#if defined(CONFIG_INTC_EXTINT_USE_SHARE_INTERRUPT)
	if (0 == data->extint_table) {
		/* Enable extint irq interrupt */
		irq_enable(irqnum);
	}
#else
	/* Enable extint irq interrupt */
	irq_enable(irqnum);
#endif
	SET_REG32_BIT(data->extint_table, BIT((uint32_t)pin));

	return 0;
}

void hc32_extint_disable(int port, int pin)
{
	int irqnum = 0;
	struct device *dev = DEVICE_GET(INTC_EXTINT_DEVICE_NAME);
	struct hc32_extint_data *data = dev->driver_data;

	if (pin >= INTC_EXTINT_NUM) {
		__ASSERT_NO_MSG(pin);
	}
	/* Get matching extint irq number from irq_table */
	irqnum = extint_irq_table[pin];
	if (irqnum == INTC_EXTINT_IRQN_DEF) {
		__ASSERT_NO_MSG(pin);
	}

	/* Disable requested pin interrupt */
	GPIO_ExtIntCmd((uint8_t)port, BIT((uint32_t)pin), DISABLE);
	CLR_REG32_BIT(data->extint_table, BIT((uint32_t)pin));
#if defined(CONFIG_INTC_EXTINT_USE_SHARE_INTERRUPT)
	if (0 == data->extint_table) {
		/* Disable extint irq interrupt */
		irq_disable(irqnum);
	}
#else
	/* Disable extint irq interrupt */
	irq_disable(irqnum);
#endif
}

void hc32_extint_trigger(int pin, int trigger)
{
	stc_extint_init_t stcExtIntInit;

	if (pin >= INTC_EXTINT_NUM) {
		__ASSERT_NO_MSG(pin);
	}

	/* ExtInt config */
	(void)EXTINT_StructInit(&stcExtIntInit);
	switch (trigger) {
	case HC32_EXTINT_TRIG_FALLING:
		stcExtIntInit.u32Edge = EXTINT_TRIG_FALLING;
		break;
	case HC32_EXTINT_TRIG_RISING:
		stcExtIntInit.u32Edge = EXTINT_TRIG_RISING;
		break;
	case HC32_EXTINT_TRIG_BOTH:
		stcExtIntInit.u32Edge = EXTINT_TRIG_BOTH;
		break;
	case HC32_EXTINT_TRIG_LOW_LVL:
		stcExtIntInit.u32Edge = EXTINT_TRIG_LOW;
		break;
	default:
		__ASSERT_NO_MSG(trigger);
		break;
	}
	EXTINT_Init(BIT((uint32_t)pin), &stcExtIntInit);
}

int  hc32_extint_set_callback(int pin, hc32_extint_callback_t cb, void *user)
{
	struct device *dev = DEVICE_GET(INTC_EXTINT_DEVICE_NAME);
	struct hc32_extint_data *data = dev->driver_data;

	if (pin >= INTC_EXTINT_NUM) {
		__ASSERT_NO_MSG(pin);
	}

	if ((data->cb[pin].cb == cb) && (data->cb[pin].user == user)) {
		return 0;
	}
	/* if callback already exists/maybe-running return busy */
	if (data->cb[pin].cb != NULL) {
		return -EBUSY;
	}
	data->cb[pin].cb   = cb;
	data->cb[pin].user = user;

	return 0;
}

void hc32_extint_unset_callback(int pin)
{
	struct device *dev = DEVICE_GET(INTC_EXTINT_DEVICE_NAME);
	struct hc32_extint_data *data = dev->driver_data;

	if (pin >= INTC_EXTINT_NUM) {
		__ASSERT_NO_MSG(pin);
	}

	data->cb[pin].cb   = NULL;
	data->cb[pin].user = NULL;
}
