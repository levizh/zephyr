/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "codec.h"
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define I2S_RX_NODE  DT_NODELABEL(i2s_rxtx)
#define I2S_TX_NODE  I2S_RX_NODE

#define SAMPLE_FREQUENCY    32000
#define SAMPLE_BIT_WIDTH    32
#define BYTES_PER_SAMPLE    sizeof(int32_t)
#define NUMBER_OF_CHANNELS  2
#define TIMEOUT             1000

#define BLOCK_SIZE  512 * BYTES_PER_SAMPLE
#define BLOCK_COUNT 4
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

int main(void)
{
	const struct device *const i2s_dev_rx = DEVICE_DT_GET(I2S_RX_NODE);
	const struct device *const i2s_dev_tx = DEVICE_DT_GET(I2S_TX_NODE);
	struct i2s_config config;
	void *mem_block;
	int ret;
	uint32_t block_size;

	if (!init_wm8731_i2c()) {
		return 0;
	}

	if (!device_is_ready(i2s_dev_rx)) {
		printk("%s is not ready\n", i2s_dev_rx->name);
		return 0;
	}

	if (i2s_dev_rx != i2s_dev_tx && !device_is_ready(i2s_dev_tx)) {
		printk("%s is not ready\n", i2s_dev_tx->name);
		return 0;
	}

	config.word_size = SAMPLE_BIT_WIDTH;
	config.channels = NUMBER_OF_CHANNELS;
	config.format = I2S_FMT_DATA_FORMAT_I2S;
	config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	config.frame_clk_freq = SAMPLE_FREQUENCY;
	config.mem_slab = &mem_slab;
	config.block_size = BLOCK_SIZE;
	config.timeout = TIMEOUT;
	if (i2s_configure(i2s_dev_rx, I2S_DIR_BOTH, &config)) {
		return 0;
	}

	ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
	if (ret < 0) {
		return false;
	}
	i2s_write(i2s_dev_tx, mem_block, BLOCK_SIZE);
	ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
	if (ret < 0) {
		return false;
	}
	i2s_write(i2s_dev_tx, mem_block, BLOCK_SIZE);

	if (i2s_trigger(i2s_dev_rx, I2S_DIR_BOTH, I2S_TRIGGER_START)) {
		return 0;
	}

	for (;;) {
		ret = i2s_read(i2s_dev_rx, &mem_block, &block_size);
		if (ret == 0) {
			ret = i2s_write(i2s_dev_tx, mem_block, BLOCK_SIZE);
		}
	}

	printk("Streams stopped\n");
}
