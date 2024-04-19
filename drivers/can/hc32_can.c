/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <clock_control/hc32_clock_control.h>
#include <interrupt_controller/intc_hc32.h>
#include <clock_control.h>
#include <misc/util.h>
#include <string.h>
#include <kernel.h>
#include <soc.h>
#include <errno.h>
#include <stdbool.h>
#include <can.h>
#include "hc32_can.h"

#define LOG_LEVEL CONFIG_CAN_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(hc32_can);


/* Get dts info */
#define CAN_IRQ_ARRAY_BY_NAME(name, offset, prop)         \
        DT_XHSC_HC32_CAN_##name##_IRQ_##offset##_##prop
#define CAN_IRQ_SINGLE_BY_NAME(name, offset)              \
        DT_XHSC_HC32_CAN_##name##_IRQ_##offset

#define CAN_CLOCK_ARRAY_BY_NAME(name, offset, prop)       \
        DT_XHSC_HC32_CAN_##name##_CLOCK_##offset##_##prop
#define CAN_CLOCK_SINGLE_BY_NAME(name, offset)            \
        DT_XHSC_HC32_CAN_##name##_CLOCK_##offset

#define CAN_PROP_ARRAY_BY_NAME(name, prop, offset)        \
        DT_XHSC_HC32_CAN_##name##_##prop##_##offset
#define CAN_PROP_SINGLE_BY_NAME(name, prop)               \
        DT_XHSC_HC32_CAN_##name##_##prop


static int can_check_trans_status(CM_CAN_TypeDef *can,
				  uint32_t trans_ch, uint32_t timeout)
{
	volatile uint32_t t_cnt = 0;

	do {
		if (READ_REG8_BIT(can->TCMD, trans_ch) == 0U) {
			return LL_OK;
		}
		k_sleep(1);
	} while (++t_cnt < timeout);

	return LL_ERR_BUSY;
}

static int can_get_trans_status(CM_CAN_TypeDef *can, uint32_t trans_ch)
{
	int32_t i32Ret = LL_OK;

	if (READ_REG8_BIT(can->TCMD, trans_ch) != 0U) {
		i32Ret = LL_ERR_BUSY;
	}

	return i32Ret;
}

static void can_hc32_signal_tx_complete(struct can_mailbox *mb)
{
	if (mb->tx_callback) {
		mb->tx_callback(mb->error_flags);
	} else  {
		k_sem_give(&mb->tx_int_sem);
	}
}

static int can_hc32_get_filter_index(CM_CAN_TypeDef *can,
				     struct can_hc32_data *device_data,
				     stc_can_rx_frame_t *frame_info)
{
	int filter_id = 0;
	uint32_t usage_shifted;
	struct zcan_filter *filter_info;
	uint32_t filter_mask = (1UL << CONFIG_CAN_MAX_FILTER) - 1UL;

	do {
		usage_shifted = device_data->filter_usage >> filter_id;
		if (!(usage_shifted & 0x1)) {
			filter_info = &device_data->filter_config[filter_id];
			/* compare rtr */
			if (filter_info->rtr_mask) {
				if (filter_info->rtr != frame_info->RTR) {
					goto check;
				}
			}
			/* compare id_type */
			if (filter_info->id_type == frame_info->IDE) {
				/* compare id, id_mask */
				if (filter_info->id_type == CAN_STANDARD_IDENTIFIER) {
					if ((filter_info->std_id & filter_info->std_id_mask) ==
					    (frame_info->u32ID & filter_info->std_id_mask)) {
						return filter_id;
					}
				} else {
					if ((filter_info->ext_id & filter_info->ext_id_mask) ==
					    (frame_info->u32ID & filter_info->ext_id_mask)) {
						return filter_id;
					}
				}
			}
		}

check:
		if (usage_shifted == (filter_mask >> filter_id)) {
			LOG_INF("No match filter found");
			return CAN_TIMEOUT;
		}
		filter_id += 1;
	} while (filter_id < CONFIG_CAN_MAX_FILTER);
	LOG_INF("No match filter found");

	return CAN_TIMEOUT;
}

static void can_hc32_get_msg_fifo(stc_can_rx_frame_t *frame_info,
				  struct zcan_frame *msg)
{
	uint32_t count;

	if (frame_info->IDE) {
		msg->ext_id  = frame_info->u32ID & CAN_EXT_ID_MASK;
		msg->id_type = CAN_EXTENDED_IDENTIFIER;
	} else {
		msg->std_id  = frame_info->u32ID & CAN_STD_ID_MASK;
		msg->id_type = CAN_STANDARD_IDENTIFIER;
	}
	msg->rtr = frame_info->RTR ? CAN_REMOTEREQUEST : CAN_DATAFRAME;
	msg->dlc = frame_info->DLC;
	for (count = 0; count < CAN_MAX_DLC; count++) {
		msg->data[count] = frame_info->au8Data[count];
	}
}

static void can_hc32_rx_irq_handler(CM_CAN_TypeDef *can,
				    struct can_hc32_data *data)
{
	int filter_match_index;
	struct zcan_frame msg;
	stc_can_rx_frame_t rx_frame;
	struct k_msgq *msg_q = NULL;
	can_rx_callback_t callback = NULL;

	if (SET == CAN_GetStatus(can, CAN_FLAG_RX)) {
		/* Get all received frames. */
		while (LL_OK == CAN_GetRxFrame(can, &rx_frame)) {
			filter_match_index = can_hc32_get_filter_index(can, data, &rx_frame);
			/* match frame */
			if (filter_match_index >= 0) {
				LOG_DBG("Message on filter index %d", filter_match_index);
				can_hc32_get_msg_fifo(&rx_frame, &msg);
				if (data->rx_response[filter_match_index]) {
					if (data->response_type & (1UL << filter_match_index)) {
						msg_q = data->rx_response[filter_match_index];
						k_msgq_put(msg_q, &msg, K_NO_WAIT);
					} else {
						callback = data->rx_response[filter_match_index];
						callback(&msg);
					}
				}
			}
		}
		CAN_ClearStatus(can, CAN_FLAG_RX);
	}
}

static void can_hc32_tx_irq_handler(CM_CAN_TypeDef *can,
				    struct can_hc32_data *data)
{
	en_flag_status_t bus_off;
	en_flag_status_t trans_state;

	bus_off = CAN_GetStatus(can, CAN_FLAG_BUS_OFF);
	trans_state = CAN_GetStatus(can, CAN_FLAG_PTB_TX);
	if ((SET == trans_state) || (SET == bus_off)) {
		if (SET == trans_state) {
			data->mb0.error_flags = CAN_TX_OK;
			CAN_ClearStatus(can, CAN_FLAG_PTB_TX);
		} else {
			data->mb0.error_flags = CAN_TX_BUS_OFF;
		}
		can_hc32_signal_tx_complete(&data->mb0);
	}

	if (SET == CAN_GetStatus(can, CAN_FLAG_ERR_INT)) {
		CAN_ClearStatus(can, CAN_FLAG_ERR_INT);
	}
	if (LL_OK == can_get_trans_status(can, CAN_TX_REQ_PTB)) {
		k_sem_give(&data->tx_int_sem);
	}
}

static void can_hc32_irq_handler(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct can_hc32_data *data = CAN_DEV_DATA(dev);
	const struct can_hc32_config *config = CAN_DEV_CFG(dev);
	CM_CAN_TypeDef *can = config->can;

	can_hc32_tx_irq_handler(can, data);
	can_hc32_rx_irq_handler(can, data);
}

int can_hc32_configure(struct device *dev, enum can_mode mode,
		       u32_t bitrate)
{
	const struct can_hc32_config *config = CAN_DEV_CFG(dev);
	CM_CAN_TypeDef *can = config->can;
	struct can_hc32_data *data = CAN_DEV_DATA(dev);
	stc_can_init_t *stc_can = &data->can_init;
	struct device *clock;
	uint32_t clock_rate, prescaler;
	uint32_t work_mode;
	int ret;

	/* calculate clock division */
	clock = device_get_binding(HC32_CLOCK_CONTROL_NAME);
	__ASSERT_NO_MSG(clock);
	ret = clock_control_get_rate(clock, (clock_control_subsys_t)&config->clk_sys,
				     &clock_rate);
	if (ret != 0) {
		LOG_ERR("Could not obtain can clock (%d)", ret);
		return -EIO;
	}

	if (!bitrate) {
		bitrate = config->bus_speed;
	}
	prescaler = clock_rate / (BIT_SEG_LENGTH(config) * bitrate);
	if ((prescaler < 2) || (prescaler > 256)) {
		LOG_ERR("bitrate out of range, clock division %d", prescaler);
		return -EINVAL;
	}

	if (clock_rate % (BIT_SEG_LENGTH(config) * bitrate)) {
		LOG_ERR("Prescaler is not a natural number! "
			"prescaler = clock_rate / ((PROP_SEG1 + SEG2 + 1) * bus_speed); "
			"prescaler = %d / ((%d + %d + 1) * %d)",
			clock_rate, config->prop_phase1, config->phase2, bitrate);
	}
	__ASSERT((config->prop_phase1 >= 1) && (config->prop_phase1 <= 64),
		 "PROP_SEG1 value out of range");
	__ASSERT((config->phase2 >= 1) && (config->phase2 <= 8),
		 "SEG2 value out of range");
	__ASSERT((config->sjw >= 1) && (config->sjw <= 8),
		 "SJW value out of range");

	if (mode == CAN_NORMAL_MODE) {
		work_mode = CAN_WORK_MD_NORMAL;
	} else if (mode == CAN_SILENT_MODE) {
		work_mode = CAN_WORK_MD_SILENT;
	} else if (mode == CAN_LOOPBACK_MODE) {
		work_mode = CAN_WORK_MD_ILB;
	} else {
		LOG_ERR("Unsupported the mode (CAN_SILENT_LOOPBACK_MODE)");
		return -EINVAL;
	}

	/* Waiting for can trans to complete */
	if (LL_OK != can_check_trans_status(can, CAN_TX_REQ_PTB,
					    CONFIG_CAN_TIMEOUT_WAIT_CONFIG)) {
		LOG_ERR("CAN is sending data: %d", ret);
		return CAN_TIMEOUT;
	}
	/* Initializes CAN. */
	stc_can->stcBitCfg.u32Prescaler = prescaler;
	stc_can->stcBitCfg.u32TimeSeg1  = config->prop_phase1 + 1;
	stc_can->stcBitCfg.u32TimeSeg2  = config->phase2;
	stc_can->stcBitCfg.u32SJW       = config->sjw;
	stc_can->u8WorkMode             = work_mode;
	ret = CAN_Init(can, stc_can);
	if (ret != LL_OK) {
		LOG_ERR("CAN init failed: %d", ret);
		return -EIO;
	}
	LOG_DBG("Runtime configure of %s done", dev->config->name);

	return 0;
}

int can_hc32_send(struct device *dev, const struct zcan_frame *msg,
		  s32_t timeout, can_tx_callback_t callback_isr)
{
	const struct can_hc32_config *config = CAN_DEV_CFG(dev);
	CM_CAN_TypeDef *can = config->can;
	struct can_hc32_data *data = CAN_DEV_DATA(dev);
	struct k_mutex *tx_mutex = &data->tx_mutex;
	struct can_mailbox *mb = NULL;
	stc_can_tx_frame_t tx_frame;
	int count, ret;

	LOG_DBG("Sending %d bytes on %s. "
		"Id: 0x%x, "
		"ID type: %s, "
		"Remote Frame: %s",
		msg->dlc, dev->config->name,
		(msg->id_type == CAN_STANDARD_IDENTIFIER) ? msg->std_id : msg->ext_id,
		(msg->id_type == CAN_STANDARD_IDENTIFIER) ? "standard" : "extended",
		(msg->rtr == CAN_DATAFRAME) ? "no" : "yes");
	__ASSERT(msg->dlc == 0 || msg->data != NULL, "Dataptr is null");
	__ASSERT(msg->dlc <= CAN_MAX_DLC, "DLC > 8");

	if (SET == CAN_GetStatus(can, CAN_FLAG_BUS_OFF)) {
		return CAN_TX_BUS_OFF;
	}
	k_mutex_lock(tx_mutex, K_FOREVER);
	while (LL_OK != can_get_trans_status(can, CAN_TX_REQ_PTB)) {
		k_mutex_unlock(tx_mutex);
		LOG_DBG("Transmit buffer full. Wait with timeout (%d ms)", timeout);
		if (k_sem_take(&data->tx_int_sem, timeout)) {
			return CAN_TIMEOUT;
		}
		k_mutex_lock(tx_mutex, K_FOREVER);
	}

	if (LL_OK == can_get_trans_status(can, CAN_TX_REQ_PTB)) {
		LOG_DBG("Using mailbox 0");
		mb = &data->mb0;
	}
	mb->tx_callback = callback_isr;

	k_sem_reset(&mb->tx_int_sem);
	/* can frame setup */
	if (msg->id_type == CAN_STANDARD_IDENTIFIER) {
		tx_frame.u32ID = msg->std_id;
	} else {
		tx_frame.u32ID = msg->ext_id;
	}
	tx_frame.u32Ctrl = 0UL;
	tx_frame.IDE = msg->id_type;
	tx_frame.RTR = msg->rtr;
	tx_frame.DLC = msg->dlc;
	for (count = 0; count < CAN_MAX_DLC; count++) {
		tx_frame.au8Data[count] = msg->data[count];
	}
	/* Transmit one frame via PTB */
	ret = CAN_FillTxFrame(can, CAN_TX_BUF_PTB, &tx_frame);
	if (ret != LL_OK) {
		LOG_ERR("CAN fill frame failed: %d", ret);
		return -EIO;
	}
	CAN_StartTx(can, CAN_TX_REQ_PTB);
	k_mutex_unlock(tx_mutex);

	if (callback_isr == NULL) {
		k_sem_take(&mb->tx_int_sem, K_FOREVER);
		return mb->error_flags;
	}

	return 0;
}

static int can_hc32_set_filter(const struct zcan_filter *filter,
			       struct can_hc32_data *device_data, CM_CAN_TypeDef *can)
{
	int ret;
	int filter_id = 0;
	uint32_t usage_shifted;
	uint16_t filter_select;
	stc_can_filter_config_t can_filter;
	stc_can_init_t *stc_can = &device_data->can_init;

	/* unsupported filter rtr function */
	if (filter->id_type == CAN_STANDARD_IDENTIFIER) {
		can_filter.u32ID     = filter->std_id;
		can_filter.u32IDType = CAN_ID_STD;
		can_filter.u32IDMask = (~filter->std_id_mask) & CAN_STD_ID_MASK;
	} else {
		can_filter.u32ID     = filter->ext_id;
		can_filter.u32IDType = CAN_ID_EXT;
		can_filter.u32IDMask = (~filter->ext_id_mask) & CAN_EXT_ID_MASK;
	}
	LOG_DBG("Setting filter ID: 0x%x, mask: 0x%x",
		filter->ext_id, filter->ext_id_mask);

	do {
		usage_shifted = device_data->filter_usage >> filter_id;
		if (usage_shifted & 0x1UL) {
			device_data->filter_usage &= ~(0x1UL << filter_id);
			break;
		}
		if (!usage_shifted) {
			LOG_INF("No free filter found");
			return CAN_NO_FREE_FILTER;
		}
		filter_id += 1;
	} while (filter_id < CONFIG_CAN_MAX_FILTER);
	if (filter_id >= CONFIG_CAN_MAX_FILTER) {
		LOG_INF("No free filter found");
		return CAN_NO_FREE_FILTER;
	}

	/* Waiting for can trans to complete */
	if (LL_OK != can_check_trans_status(can, CAN_TX_REQ_PTB,
					    CONFIG_CAN_TIMEOUT_WAIT_CONFIG)) {
		LOG_ERR("CAN is sending data: %d", ret);
		return CAN_TIMEOUT;
	}
	device_data->filter_config[filter_id] = *filter;
	filter_select = CAN_FILTER1 << filter_id;
	stc_can->u16FilterSelect |= filter_select;
	device_data->filter_table[filter_id] = can_filter;
	ret = CAN_Init(can, stc_can);
	if (ret != LL_OK) {
		LOG_ERR("CAN init failed: %d", ret);
		return CAN_NO_FREE_FILTER;
	}
	LOG_DBG("Filter set! Filter number: %d ", filter_id);

	return filter_id;
}

static int can_hc32_attach(struct device *dev, void *response_ptr,
			   const struct zcan_filter *filter)
{
	int filter_id;
	const struct can_hc32_config *config = CAN_DEV_CFG(dev);
	CM_CAN_TypeDef *can = config->can;
	struct can_hc32_data *data = CAN_DEV_DATA(dev);

	filter_id = can_hc32_set_filter(filter, data, can);
	if (filter_id >= 0) {
		data->rx_response[filter_id] = response_ptr;
	}

	return filter_id;
}

int can_hc32_attach_msgq(struct device *dev, struct k_msgq *msg_q,
			 const struct zcan_filter *filter)
{
	int filter_id;
	struct can_hc32_data *data = CAN_DEV_DATA(dev);

	k_mutex_lock(&data->set_filter_mutex, K_FOREVER);
	filter_id = can_hc32_attach(dev, msg_q, filter);
	if (filter_id >= 0) {
		data->response_type |= (1UL << filter_id);
	}
	k_mutex_unlock(&data->set_filter_mutex);

	return filter_id;
}

int can_hc32_attach_isr(struct device *dev, can_rx_callback_t isr,
			const struct zcan_filter *filter)
{
	int filter_id;
	struct can_hc32_data *data = CAN_DEV_DATA(dev);

	k_mutex_lock(&data->set_filter_mutex, K_FOREVER);
	filter_id = can_hc32_attach(dev, isr, filter);
	if (filter_id >= 0) {
		data->response_type &= ~(1UL << filter_id);
	}
	k_mutex_unlock(&data->set_filter_mutex);

	return filter_id;
}

void can_hc32_detach(struct device *dev, int filter_id)
{
	const struct can_hc32_config *config = CAN_DEV_CFG(dev);
	CM_CAN_TypeDef *can = config->can;
	struct can_hc32_data *data = CAN_DEV_DATA(dev);
	stc_can_init_t *stc_can = &data->can_init;
	uint16_t filter_select;

	__ASSERT_NO_MSG((filter_id >= 0) && (filter_id < CONFIG_CAN_MAX_FILTER));

	k_mutex_lock(&data->set_filter_mutex, K_FOREVER);
	filter_select = CAN_FILTER1 << filter_id;
	stc_can->u16FilterSelect &= ~(filter_select);
	CAN_FilterCmd(can, filter_select, DISABLE);
	LOG_DBG("Detach filter number %d", filter_id);

	data->filter_usage |= (1UL << filter_id);
	data->rx_response[filter_id] = NULL;
	data->response_type &= ~(1UL << filter_id);
	k_mutex_unlock(&data->set_filter_mutex);
}

static int can_hc32_init(struct device *dev)
{
	const struct can_hc32_config *config = CAN_DEV_CFG(dev);
	CM_CAN_TypeDef *can = config->can;
	struct can_hc32_data *data = CAN_DEV_DATA(dev);
	stc_can_init_t *stc_can = &data->can_init;
	struct device *clock;
	int ret;

	k_mutex_init(&data->tx_mutex);
	k_mutex_init(&data->set_filter_mutex);
	k_sem_init(&data->tx_int_sem, 0, 1);
	k_sem_init(&data->mb0.tx_int_sem, 0, 1);
	data->mb0.tx_callback = NULL;
	data->filter_usage = (1UL << CONFIG_CAN_MAX_FILTER) - 1UL;
	data->response_type = 0;
	(void)memset(data->rx_response, 0, sizeof(void *) * CONFIG_CAN_MAX_FILTER);

	/* initialize clock */
	clock = device_get_binding(HC32_CLOCK_CONTROL_NAME);
	__ASSERT_NO_MSG(clock);
	ret = clock_control_on(clock, (clock_control_subsys_t)&config->clk_sys);
	if (ret != 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return -EIO;
	}
	/* Initializes CAN. */
	(void)CAN_StructInit(stc_can);
	stc_can->pstcFilter = data->filter_table;
	ret = can_hc32_configure(dev, CAN_NORMAL_MODE, 0);
	if (ret != 0) {
		LOG_ERR("Init can failed %d", ret);
		return ret;
	}
	CAN_FilterCmd(can, CAN_FILTER_ALL, DISABLE);
	config->irq_config(can);
	LOG_INF("Init of %s done", dev->config->name);

	return 0;
}

static const struct can_driver_api can_hc32_driver_api = {
	.configure   = can_hc32_configure,
	.send        = can_hc32_send,
	.attach_msgq = can_hc32_attach_msgq,
	.attach_isr  = can_hc32_attach_isr,
	.detach      = can_hc32_detach,
};


/* CAN base address */
#define CAN_BASE_ADDR(addr)             ((CM_CAN_TypeDef *)0x##addr)

#define HC32_CAN_IRQ_HANDLER_DECL(idx)				                \
	static void can_##idx##_hc32_irq_config(CM_CAN_TypeDef *can)

#define HC32_CAN_IRQ_HANDLER(idx, addr)					            \
    static void can_##idx##_hc32_irq_config(CM_CAN_TypeDef *can)    \
	{									                            \
		IRQ_CONNECT(CAN_IRQ_SINGLE_BY_NAME(addr, GLB),              \
			        CAN_IRQ_ARRAY_BY_NAME(addr, GLB, PRIORITY),     \
			        can_hc32_irq_handler,                           \
                    DEVICE_GET(can_hc32_##idx), 0);                 \
		hc32_intc_irq_signin(CAN_IRQ_SINGLE_BY_NAME(addr, GLB),     \
		            CAN_IRQ_ARRAY_BY_NAME(addr, GLB, INT_SRC));     \
		irq_enable(CAN_IRQ_SINGLE_BY_NAME(addr, GLB));              \
		CAN_IntCmd(can, CAN_INT_ALL, DISABLE);                      \
		CAN_IntCmd(can, (CAN_INT_PTB_TX | CAN_INT_RX |              \
				   CAN_INT_ERR_INT), ENABLE);                       \
}

#define CAN_DEVICE_INIT(idx, addr)						                    \
	HC32_CAN_IRQ_HANDLER_DECL(idx);					                        \
	static struct can_hc32_data can##idx##_data;			                \
	static const struct can_hc32_config can##idx##_config = {	            \
		.can = CAN_BASE_ADDR(addr),						                    \
		.bus_speed = CAN_PROP_SINGLE_BY_NAME(addr, BUS_SPEED),              \
		.prop_phase1 = CAN_PROP_SINGLE_BY_NAME(addr, PROP_SEG_PHASE_SEG1),  \
		.phase2 = CAN_PROP_SINGLE_BY_NAME(addr, PHASE_SEG2),                \
		.sjw = CAN_PROP_SINGLE_BY_NAME(addr, SJW),                          \
		.clk_sys = {							                            \
			.bus  = CAN_CLOCK_SINGLE_BY_NAME(addr, BUS),                    \
			.fcg  = CAN_CLOCK_SINGLE_BY_NAME(addr, FCG),                    \
			.bits = CAN_CLOCK_SINGLE_BY_NAME(addr, BITS),                   \
		},							                                        \
		.irq_config = can_##idx##_hc32_irq_config,		                    \
	};									                                    \
	DEVICE_AND_API_INIT(can_hc32_##idx,						                \
			      CAN_PROP_SINGLE_BY_NAME(addr, LABEL),				        \
			      &can_hc32_init,						                    \
			      &can##idx##_data,				                            \
			      &can##idx##_config,				                        \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	        \
			      &can_hc32_driver_api);                                    \
	HC32_CAN_IRQ_HANDLER(idx, addr)

#if defined(CONFIG_NET_SOCKETS_CAN)
#define CAN_NET_DEVICE_INIT(idx, addr)                                  \
    #include "socket_can_generic.h"                                     \
    static int socket_can_##idx##_init(struct device *dev)              \
    {                                                                   \
        struct device *can_dev = DEVICE_GET(can_hc32_##idx);            \
        struct socket_can_context *socket_context = dev->driver_data;   \
                                                                        \
        LOG_DBG("Init socket CAN device %p (%s) for dev %p (%s)",       \
            dev, dev->config->name, can_dev, can_dev->config->name);    \
        socket_context->can_dev = can_dev;                              \
        socket_context->msgq    = &socket_can_msgq;                     \
        socket_context->rx_tid  =                                       \
            k_thread_create(&socket_context->rx_thread_data,            \
                            rx_thread_stack,                            \
                            K_THREAD_STACK_SIZEOF(rx_thread_stack),     \
                            rx_thread, socket_context, NULL, NULL,      \
                            RX_THREAD_PRIORITY, 0, K_NO_WAIT);          \
        return 0;                                                       \
    }                                                                   \
    NET_DEVICE_INIT(socket_can_hc32_##idx, SOCKET_CAN_NAME_1,           \
            socket_can_##idx##_init,                                    \
            &socket_can_context_1, NULL,                                \
            CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                         \
            &socket_can_api, CANBUS_L2,                                 \
            NET_L2_GET_CTX_TYPE(CANBUS_L2), CAN_MTU);
#endif

#ifdef DT_XHSC_HC32_CAN_40070400_LABEL
CAN_DEVICE_INIT(0, 40070400)
#if defined(CONFIG_NET_SOCKETS_CAN)
CAN_NET_DEVICE_INIT(0, 40070400)
#endif /* CONFIG_NET_SOCKETS_CAN */
#endif /* DT_XHSC_HC32_CAN_40070400_LABEL */
