/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2S_HC32_H_
#define ZEPHYR_DRIVERS_I2S_HC32_H_

/* Device constant configuration parameters */
struct i2s_hc32_cfg {
	CM_I2S_TypeDef *i2s;
	const struct hc32_modules_clock_sys *mod_clk;
	const struct pinctrl_dev_config *pcfg;

	const struct device *dma_dev[2];
	struct dma_config *dma_conf;
	uint8_t channel[2];
	dma_callback_t dma_callbcck;

	uint32_t i2s_src_clk;
	uint32_t pll_src_cfg;
	uint32_t tx_fifo_level;
	uint32_t rx_fifo_level;
};

struct stream {
	enum i2s_state state;
	struct i2s_config cfg;
	void *mem_block;
	uint32_t len;
	uint32_t index;
};

struct i2s_hc32_data_queue {
	void *data;
	uint32_t len;
};

/* Device run time data */
struct i2s_hc32_data {
	struct k_msgq tx_queue;
	struct stream tx;
	struct k_msgq rx_queue;
	struct stream rx;
	uint32_t w_i_index;
	uint32_t w_o_index;
	uint32_t r_i_index;
	uint32_t r_o_index;
	struct i2s_hc32_data_queue *write_queue;
	struct i2s_hc32_data_queue *read_queue;
};

#endif	/* ZEPHYR_DRIVERS_I2S_HC32_H_ */
