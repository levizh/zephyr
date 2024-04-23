/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CAN_HC32_CAN_H_
#define ZEPHYR_DRIVERS_CAN_HC32_CAN_H_

#include <can.h>


#define CAN_DEV_DATA(dev)       \
    ((struct can_hc32_data *const)(dev)->driver_data)

#define CAN_DEV_CFG(dev)        \
    ((const struct can_hc32_config *const)(dev)->config->config_info)

#define BIT_SEG_LENGTH(cfg)     ((cfg)->prop_phase1 + (cfg)->phase2 + 1)


struct can_mailbox {
	stc_can_tx_frame_t  tx_frame;
	can_tx_callback_t   tx_callback;
	struct k_sem        tx_int_sem;
	uint32_t            error_flags;
};

struct can_hc32_data {
	struct k_sem   tx_int_sem;
	struct k_mutex tx_mutex;
	struct k_mutex set_filter_mutex;
	struct can_mailbox mb0;
	uint32_t filter_usage;
	uint32_t response_type;
	void *rx_response[CONFIG_CAN_MAX_FILTER];
	struct zcan_filter filter_config[CONFIG_CAN_MAX_FILTER];
	stc_can_init_t can_init;
	stc_can_filter_config_t filter_table[CONFIG_CAN_MAX_FILTER];
};

struct can_phy_control {
	const char *name;
	u32_t		pin;
	u32_t		flag;
	int			state;
};

struct can_hc32_config {
	CM_CAN_TypeDef *can;
	uint32_t bus_speed;
	uint8_t  prop_phase1;
	uint8_t  phase2;
	uint8_t  sjw;
	struct hc32_modules_clock_sys clk_sys;
	struct can_phy_control phy_ctrl;
	void (*irq_config)(CM_CAN_TypeDef *can);
};


#endif /*ZEPHYR_DRIVERS_CAN_HC32_CAN_H_*/
