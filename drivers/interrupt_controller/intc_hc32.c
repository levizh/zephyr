/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for interrupt/event controller in HC32 MCUs
 */

#define INTC_NODE                       DT_INST(0, xhsc_hc32_intc)

#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>
#include <zephyr/irq.h>
#include <errno.h>

#define LOG_LEVEL CONFIG_INTC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(intc_hc32);


#define INTC_EXTINT_NUM                 DT_PROP(DT_NODELABEL(intc), extint_nums)
#define INTC_EXTINT_IRQN_DEFAULT        0xFF
#define INTC_EXTINT_INTSRC_DEFAULT      0x1FF


static IRQn_Type extint_irq_table[INTC_EXTINT_NUM] = {[0 ... INTC_EXTINT_NUM - 1] = INTC_EXTINT_IRQN_DEFAULT};
static en_int_src_t extint_src_table[INTC_EXTINT_NUM] = {[0 ... INTC_EXTINT_NUM - 1] = INTC_EXTINT_INTSRC_DEFAULT};

/* wrapper for user callback */
struct hc32_extint_cb {
	hc32_extint_callback_t cb;
	void *user;
};

/* driver data */
struct hc32_extint_data {
	/* callbacks */
	struct hc32_extint_cb cb[INTC_EXTINT_NUM];
};


/**
 * @brief EXTINT ISR handler
 */
static void hc32_extint_isr(const void *extint_ch)
{
	const struct device *dev = DEVICE_DT_GET(INTC_NODE);
	struct hc32_extint_data *data = dev->data;
	uint8_t ch = *(uint8_t *)extint_ch;

	if (ch >= INTC_EXTINT_NUM)  {
		__ASSERT_NO_MSG(ch);
	}

	if (SET == EXTINT_GetExtIntStatus(BIT((uint32_t)ch))) {
		EXTINT_ClearExtIntStatus(BIT((uint32_t)ch));
		/* run callback only if one is registered */
		if (data->cb[ch].cb != NULL) {
			data->cb[ch].cb(ch, data->cb[ch].user);
		}
	}
}

/* Fill irq information */
static void hc32_fill_irq_table(uint8_t ch, int irqn, int intsrc)
{
	extint_irq_table[ch] = (IRQn_Type)irqn;
	extint_src_table[ch] = (en_int_src_t)intsrc;
}

#define HC32_EXTINT_INIT(node_id, interrupts, idx)			\
	static const uint8_t extint_ch_##idx =					\
			DT_PROP_BY_IDX(node_id, extint_chs, idx); 		\
	hc32_fill_irq_table(extint_ch_##idx,					\
			DT_IRQ_BY_IDX(node_id, idx, irq), 				\
			DT_PROP_BY_IDX(node_id, int_srcs, idx));		\
	IRQ_CONNECT(DT_IRQ_BY_IDX(node_id, idx, irq),			\
			DT_IRQ_BY_IDX(node_id, idx, priority),			\
			hc32_extint_isr, &extint_ch_##idx ,				\
			0);

/**
 * @brief initialize intc device driver
 */
static int hc32_intc_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	// hc32_intc_irq_signin all extint ???
	DT_FOREACH_PROP_ELEM(DT_NODELABEL(intc), interrupt_names, HC32_EXTINT_INIT);

	return 0;
}

static struct hc32_extint_data extint_data;
DEVICE_DT_DEFINE(INTC_NODE, &hc32_intc_init,
		 NULL,
		 &extint_data, NULL,
		 PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,
		 NULL);


void hc32_extint_enable(int port, int pin)
{
	int irqnum = 0;

	if (pin >= INTC_EXTINT_NUM) {
		__ASSERT_NO_MSG(pin);
	}

	/* Get matching extint irq number from irq_table */
	irqnum = extint_irq_table[pin];
	if (irqnum == INTC_EXTINT_IRQN_DEFAULT) {
		__ASSERT_NO_MSG(pin);
	}

	/* Enable requested pin interrupt */
	GPIO_ExtIntCmd((uint8_t)port, BIT((uint32_t)pin), ENABLE);
	/* Enable extint irq interrupt */
	irq_enable(irqnum);
}

void hc32_extint_disable(int port, int pin)
{
	int irqnum = 0;

	if (pin >= INTC_EXTINT_NUM) {
		__ASSERT_NO_MSG(pin);
	}

	/* Get matching extint irq number from irq_table */
	irqnum = extint_irq_table[pin];
	if (irqnum == INTC_EXTINT_IRQN_DEFAULT) {
		__ASSERT_NO_MSG(pin);
	}

	/* Disable requested pin interrupt */
	GPIO_ExtIntCmd((uint8_t)port, BIT((uint32_t)pin), DISABLE);
	/* Disable extint irq interrupt */
	irq_disable(irqnum);
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
	const struct device *const dev = DEVICE_DT_GET(INTC_NODE);
	struct hc32_extint_data *data = dev->data;

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
	const struct device *const dev = DEVICE_DT_GET(INTC_NODE);
	struct hc32_extint_data *data = dev->data;

	if (pin >= INTC_EXTINT_NUM) {
		__ASSERT_NO_MSG(pin);
	}

	data->cb[pin].cb   = NULL;
	data->cb[pin].user = NULL;
}

void hc32_extint_get_irq_info(int pin, int *irqn, int *intsrc)
{
	if (pin >= INTC_EXTINT_NUM) {
		__ASSERT_NO_MSG(pin);
	}

	*irqn   = extint_irq_table[pin];
	*intsrc = extint_src_table[pin];
}


/* intc config */
int hc32_intc_irq_signin(int irqn, int intsrc)
{
	stc_irq_signin_config_t stcIrqSignConfig;

	stcIrqSignConfig.enIRQn      = (IRQn_Type)irqn;
	stcIrqSignConfig.enIntSrc    = (en_int_src_t)intsrc;
	stcIrqSignConfig.pfnCallback = NULL;
	if (LL_OK != INTC_IrqSignIn(&stcIrqSignConfig)) {
		LOG_ERR("intc signin failed!");
		return -EACCES;
	}

	return 0;
}

int hc32_intc_irq_signout(int irqn)
{
	if (LL_OK != INTC_IrqSignOut((IRQn_Type)irqn)) {
		LOG_ERR("intc signout failed!");
		return -EACCES;
	}

	return 0;
}
