/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_i2s

#include <string.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <soc.h>
#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/drivers/dma/dma_hc32.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include "i2s_hc32.h"
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2s_hc32);

static int i2s_hc32_tx_start(const struct device *dev);
static int i2s_hc32_rx_start(const struct device *dev);

static void i2s_dma_disable(const struct device *dev, enum i2s_dir dir)
{
	const struct i2s_hc32_cfg *cfg = dev->config;
	struct i2s_hc32_data *dev_data = dev->data;

	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		dma_stop(cfg->dma_dev[0], cfg->channel[0]);
		if (dev_data->tx.mem_block != NULL) {
			k_mem_slab_free(dev_data->tx.cfg.mem_slab, dev_data->tx.mem_block);
			dev_data->tx.mem_block = NULL;
		}
		I2S_FuncCmd(cfg->i2s, I2S_FUNC_TX, DISABLE);
	}
	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		dma_stop(cfg->dma_dev[1], cfg->channel[1]);
		if (dev_data->rx.mem_block != NULL) {
			k_mem_slab_free(dev_data->rx.cfg.mem_slab, dev_data->rx.mem_block);
			dev_data->rx.mem_block = NULL;
		}
		I2S_FuncCmd(cfg->i2s, I2S_FUNC_RX, DISABLE);
	}
	
	I2S_SWReset(cfg->i2s, I2S_RST_TYPE_FIFO);
}

static void i2s_hc32_reset_queue(const struct device *dev, enum i2s_dir dir)
{
	const struct i2s_hc32_cfg *cfg = dev->config;
	struct i2s_hc32_data *dev_data = dev->data;
	struct i2s_hc32_data_queue mem_blk;

	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		while (k_msgq_get(&dev_data->tx_queue,
				  &mem_blk,
				  K_NO_WAIT) == 0) {
			k_mem_slab_free(dev_data->tx.cfg.mem_slab, mem_blk.data);
		}
		I2S_FuncCmd(cfg->i2s, I2S_FUNC_TX, DISABLE);
		MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_TXBIRQWL, I2S_TRANS_LVL4);
	}

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		while (k_msgq_get(&dev_data->rx_queue,
				  &mem_blk,
				  K_NO_WAIT) == 0) {
			k_mem_slab_free(dev_data->rx.cfg.mem_slab, mem_blk.data);
		}
		I2S_FuncCmd(cfg->i2s, I2S_FUNC_RX, DISABLE);
		MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_RXBIRQWL, I2S_RECEIVE_LVL0);
	}
	I2S_SWReset(cfg->i2s, I2S_RST_TYPE_FIFO);
}

const struct i2s_config *i2s_hc32_config_get(const struct device *dev,
						enum i2s_dir dir)
{
	struct i2s_hc32_data *dev_data = dev->data;

	if (dir == I2S_DIR_RX) {
		return &dev_data->rx.cfg;
	}

	return &dev_data->tx.cfg;
}

static int i2s_hc32_configure(const struct device *dev, enum i2s_dir dir,
				const struct i2s_config *i2s_cfg)
{
	const struct i2s_hc32_cfg *cfg = dev->config;
	struct i2s_hc32_data *dev_data = dev->data;
	struct stream *stm = (dir == I2S_DIR_RX) ? &dev_data->rx : &dev_data->tx;

	if (dir == I2S_DIR_BOTH) {
		if (dev_data->rx.state != I2S_STATE_NOT_READY &&
			dev_data->rx.state != I2S_STATE_READY &&
			dev_data->tx.state != I2S_STATE_NOT_READY &&
			dev_data->tx.state != I2S_STATE_READY) {
			LOG_ERR("invalid state");
			return -EINVAL;
		}
	} else if (stm->state != I2S_STATE_NOT_READY &&
				stm->state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0) {
		i2s_hc32_reset_queue(dev, dir);
		if (dir == I2S_DIR_BOTH) {
			dev_data->tx.state = I2S_STATE_NOT_READY;
			dev_data->rx.state = I2S_STATE_NOT_READY;
		} else {
			stm->state = I2S_STATE_NOT_READY;
		}
			
		if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
			memset(&dev_data->tx.cfg, 0, sizeof(struct i2s_config));
		}
		if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
			memset(&dev_data->rx.cfg, 0, sizeof(struct i2s_config));
		}
		return 0;
	}

	if (i2s_cfg->word_size == 16) {
		MODIFY_REG32(cfg->i2s->CFGR, (I2S_CFGR_DATLEN | I2S_CFGR_CHLEN), (I2S_DATA_LEN_16BIT | I2S_CH_LEN_16BIT));
	} else if (i2s_cfg->word_size == 24) {
		MODIFY_REG32(cfg->i2s->CFGR, (I2S_CFGR_DATLEN | I2S_CFGR_CHLEN), (I2S_DATA_LEN_24BIT | I2S_CH_LEN_32BIT));
	} else if (i2s_cfg->word_size == 32) {
		MODIFY_REG32(cfg->i2s->CFGR, (I2S_CFGR_DATLEN | I2S_CFGR_CHLEN), (I2S_DATA_LEN_32BIT | I2S_CH_LEN_32BIT));
	} else {
		LOG_ERR("invalid word size");
		return -EINVAL;
	}

	switch(i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		MODIFY_REG32(cfg->i2s->CFGR, I2S_CFGR_I2SSTD, I2S_PROTOCOL_PHILLIPS);
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		MODIFY_REG32(cfg->i2s->CFGR, I2S_CFGR_I2SSTD, I2S_PROTOCOL_PCM_SHORT);
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		MODIFY_REG32(cfg->i2s->CFGR, I2S_CFGR_I2SSTD, I2S_CFGR_I2SSTD);
		MODIFY_REG32(cfg->i2s->CFGR, I2S_CFGR_PCMSYNC_POS, I2S_CFGR_PCMSYNC);
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		MODIFY_REG32(cfg->i2s->CFGR, I2S_CFGR_I2SSTD, I2S_PROTOCOL_MSB);
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		MODIFY_REG32(cfg->i2s->CFGR, I2S_CFGR_I2SSTD, I2S_PROTOCOL_LSB);
		break;
	default:
		LOG_ERR("Unsupported data format: 0x%02x", i2s_cfg->format);
		return -EINVAL;
	}

	if ((i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) || \
		(i2s_cfg->format & I2S_FMT_BIT_CLK_INV)) {
			LOG_ERR("Unsupported stream format: 0x%02x", i2s_cfg->format);
			return -EINVAL;
	}

	if ((i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) && \
		(i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE)) {
		MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_WMS, I2S_MD_SLAVE);
		CLR_REG32_BIT(cfg->i2s->CTRL,(I2S_CTRL_CKOE | I2S_CTRL_LRCKOE));
	} else if (!(i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) || \
			   !(i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE)) {
		MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_WMS, I2S_MD_MASTER);
		SET_REG32_BIT(cfg->i2s->CTRL, (I2S_CTRL_CKOE | I2S_CTRL_LRCKOE));
	} else {
		LOG_ERR("Unsupported operation mode: 0x%02x", i2s_cfg->options);
		return -EINVAL;
	}

	if (READ_REG32_BIT(cfg->i2s->CTRL, I2S_CTRL_WMS) == I2S_CTRL_WMS && 
		READ_REG32_BIT(cfg->i2s->CTRL, I2S_CLK_SRC_EXT) == 0) {
		LOG_ERR("i2s salver clock must be exclk");
		return -EINVAL;
	}

	if (dir == I2S_DIR_RX) {
		memcpy(&dev_data->rx.cfg, i2s_cfg, sizeof(struct i2s_config));
		MODIFY_REG32(cfg->i2s->CTRL, (I2S_CTRL_DUPLEX | I2S_CTRL_SDOE), I2S_TRANS_MD_HALF_DUPLEX_RX);
	} else if (dir == I2S_DIR_TX) {
		memcpy(&dev_data->tx.cfg, i2s_cfg, sizeof(struct i2s_config));
		MODIFY_REG32(cfg->i2s->CTRL, (I2S_CTRL_DUPLEX | I2S_CTRL_SDOE), I2S_TRANS_MD_HALF_DUPLEX_TX);
	} else if (dir == I2S_DIR_BOTH) {
		memcpy(&dev_data->tx.cfg, i2s_cfg, sizeof(struct i2s_config));
		memcpy(&dev_data->rx.cfg, i2s_cfg, sizeof(struct i2s_config));
		MODIFY_REG32(cfg->i2s->CTRL, (I2S_CTRL_DUPLEX | I2S_CTRL_SDOE), I2S_TRANS_MD_FULL_DUPLEX);
	} else {
		LOG_ERR("Unsupported transfer dir : 0x%02x", dir);
		return -EINVAL;
	}

	I2S_SetAudioFreq(cfg->i2s, i2s_cfg->frame_clk_freq);

	if (dir == I2S_DIR_BOTH) {
		dev_data->tx.state = I2S_STATE_READY;
		dev_data->rx.state = I2S_STATE_READY;
	} else {
		stm->state = I2S_STATE_READY;
	}

	return 0;
}

static int i2s_hc32_full_duplex_trigger(const struct device *dev,
							enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct i2s_hc32_data *dev_data = dev->data;
	int32_t ret = 0;

	if (memcmp(&dev_data->rx.cfg, &dev_data->tx.cfg, sizeof(struct i2s_config)) != 0) {
		LOG_ERR("TX and RX configurations are different");
		return -EIO;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if ((dev_data->rx.state != I2S_STATE_READY) ||
			(dev_data->tx.state != I2S_STATE_READY)) {
				LOG_ERR("START trigger: invalid state %d%d", dev_data->rx.state
														,dev_data->tx.state);
				return -EIO;
		}
		ret = i2s_hc32_rx_start(dev);
		if (ret != 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}
		ret = i2s_hc32_tx_start(dev);
		if (ret != 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}
		dev_data->rx.state = dev_data->tx.state = I2S_STATE_RUNNING;
		break;
	case I2S_TRIGGER_STOP:
		if ((dev_data->rx.state != I2S_STATE_RUNNING) ||
			(dev_data->tx.state != I2S_STATE_RUNNING)) {
			LOG_ERR("STOP trigger: invalid state %d%d", dev_data->rx.state
														,dev_data->tx.state);
			return -EIO;
		}
		dev_data->rx.state = dev_data->tx.state = I2S_STATE_STOPPING;
		break;
	case I2S_TRIGGER_DRAIN:
		if ((dev_data->rx.state != I2S_STATE_RUNNING) ||
			(dev_data->tx.state != I2S_STATE_RUNNING)) {
			LOG_ERR("DRAIN trigger: invalid state %d%d", dev_data->rx.state
														,dev_data->tx.state);
			return -EIO;
		}
		dev_data->rx.state = I2S_STATE_STOPPING;
		break;
	case I2S_TRIGGER_DROP:
		if ((dev_data->rx.state == I2S_STATE_NOT_READY) ||
			(dev_data->tx.state == I2S_STATE_NOT_READY)) {
			LOG_ERR("DROP trigger: invalid state");
			return -EIO;
		}
		i2s_dma_disable(dev, I2S_DIR_BOTH);
		i2s_hc32_reset_queue(dev, I2S_DIR_BOTH);
		dev_data->rx.state = dev_data->tx.state = I2S_STATE_READY;
		break;
	case I2S_TRIGGER_PREPARE:
		if ((dev_data->rx.state != I2S_STATE_ERROR) ||
			(dev_data->tx.state != I2S_STATE_ERROR)) {
			LOG_ERR("prepare trigger: invalid state");
			return -EIO;
		}
		i2s_hc32_reset_queue(dev, I2S_DIR_BOTH);
		dev_data->rx.state = dev_data->tx.state = I2S_STATE_READY;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int i2s_hc32_half_duplex_trigger(const struct device *dev,
							enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct i2s_hc32_data *dev_data = dev->data;
	struct stream *i2s_stream;
	int32_t ret = 0;

	if (dir == I2S_DIR_TX) {
		i2s_stream = &dev_data->tx;
	} else if (dir == I2S_DIR_RX) {
		i2s_stream = &dev_data->rx;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (i2s_stream->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d", i2s_stream->state);
			return -EIO;
		}
		if (dir == I2S_DIR_TX) {
			ret = i2s_hc32_tx_start(dev);
		} else if (dir == I2S_DIR_RX) {
			ret = i2s_hc32_rx_start(dev);
		}
		if (ret != 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}
		i2s_stream->state = I2S_STATE_RUNNING;
		break;
	
	case I2S_TRIGGER_STOP:
		if (i2s_stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("STOP trigger: invalid state %d", i2s_stream->state);
			return -EIO;
		}
		i2s_stream->state = I2S_STATE_STOPPING;
		break;

	case I2S_TRIGGER_DRAIN:
		if (i2s_stream->state != I2S_STATE_RUNNING) {
			LOG_ERR("DRAIN trigger: invalid state %d", i2s_stream->state);
			return -EIO;
		}
		if (dir == I2S_DIR_RX) {
			i2s_stream->state = I2S_STATE_STOPPING;
		}
		break;

	case I2S_TRIGGER_DROP:
		if (i2s_stream->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state");
			return -EIO;
		}
		i2s_dma_disable(dev, dir);
		i2s_hc32_reset_queue(dev, dir);
		i2s_stream->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (i2s_stream->state != I2S_STATE_ERROR) {
			LOG_ERR("prepare trigger: invalid state");
			return -EIO;
		}
		i2s_hc32_reset_queue(dev, dir);
		i2s_stream->state = I2S_STATE_READY;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int i2s_hc32_trigger(const struct device *dev, enum i2s_dir dir,
				enum i2s_trigger_cmd cmd)
{
	if (dir == I2S_DIR_BOTH) {
		return i2s_hc32_full_duplex_trigger(dev, dir, cmd);
	} else {
		return i2s_hc32_half_duplex_trigger(dev, dir, cmd);
	}
}

static int i2s_hc32_read(const struct device *dev, void **mem_block,
			  size_t *size)
{
	struct i2s_hc32_data *dev_data = dev->data;
	struct i2s_hc32_data_queue mem_blk;
	int ret;

	if (dev_data->rx.state != I2S_STATE_RUNNING &&
		dev_data->rx.state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	ret = k_msgq_get(&dev_data->rx_queue, &mem_blk,
				SYS_TIMEOUT_MS(dev_data->rx.cfg.timeout));
	if (ret != 0) {
		LOG_ERR("empty buffer");
		return -EIO;
	}

	*mem_block = mem_blk.data;
	*size = dev_data->rx.cfg.block_size;

	return 0;
}

static int i2s_hc32_write(const struct device *dev, void *mem_block,
			   size_t size)
{
	struct i2s_hc32_data *dev_data = dev->data;
	struct i2s_hc32_data_queue *w_queue = dev_data->write_queue;
	int ret;
	
	if (dev_data->tx.state != I2S_STATE_RUNNING &&
	    dev_data->tx.state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	/*if (size != dev_data->tx.cfg.block_size) {
		LOG_ERR("This device can only write blocks of %u bytes",
			dev_data->tx.cfg.block_size);
		return -EIO;
	}*/

	dev_data->w_i_index = \
		(++dev_data->w_i_index >= (CONFIG_I2S_HC32_RX_BLOCK_COUNT + 1)) ? 0 : dev_data->w_i_index;
	if (dev_data->w_i_index == dev_data->w_o_index) {
		return -ENOMEM;
	}
	w_queue[dev_data->w_i_index].data = mem_block;
	w_queue[dev_data->w_i_index].len = size;

	ret = k_msgq_put(&dev_data->tx_queue,
			 &w_queue[dev_data->w_i_index],
			 SYS_TIMEOUT_MS(dev_data->tx.cfg.timeout));
	if (ret < 0) {
		return ret;
	}
	LOG_DBG("Queued TX %p", mem_block);

	return 0;
}

static const struct i2s_driver_api i2s_hc32_driver_api = {
	.configure = i2s_hc32_configure,
	.read = i2s_hc32_read,
	.write = i2s_hc32_write,
	.trigger = i2s_hc32_trigger,
	.config_get = i2s_hc32_config_get,
};

static int i2s_hc32_init(const struct device *dev)
{
	const struct device *const clk = DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE);
	const struct i2s_hc32_cfg *cfg = dev->config;
	struct i2s_hc32_data *dev_data = dev->data;
	int ret;

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Move pins to active/default state */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2S pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Enable device clock. */
	if (clock_control_on(clk,
			     (clock_control_subsys_t) &cfg->mod_clk[0]) != 0) {
		LOG_ERR("i2c: failure enabling clock");
		return -EIO;
	}

	k_msgq_alloc_init(&dev_data->tx_queue, sizeof(struct i2s_hc32_data_queue), CONFIG_I2S_HC32_TX_BLOCK_COUNT);
	k_msgq_alloc_init(&dev_data->rx_queue, sizeof(struct i2s_hc32_data_queue), CONFIG_I2S_HC32_RX_BLOCK_COUNT);

	if (cfg->i2s_src_clk == 0) {
		MODIFY_REG32(cfg->i2s->CTRL, (I2S_CTRL_CLKSEL | I2S_CTRL_I2SPLLSEL), \
									 I2S_CLK_SRC_PLL);
		SET_REG32_BIT(cfg->i2s->CTRL, I2S_CTRL_MCKOE );
		switch (cfg->mod_clk[0].bits) {
		case HC32_FCG1_PERIPH_I2S1:
			MODIFY_REG16(CM_CMU->I2SCKSEL, CMU_I2SCKSEL_I2S1CKSEL, cfg->pll_src_cfg << CMU_I2SCKSEL_I2S1CKSEL_POS);
			break;
		case HC32_FCG1_PERIPH_I2S2:
			MODIFY_REG16(CM_CMU->I2SCKSEL, CMU_I2SCKSEL_I2S2CKSEL, cfg->pll_src_cfg << CMU_I2SCKSEL_I2S1CKSEL_POS);
			break;
		case HC32_FCG1_PERIPH_I2S3:
			MODIFY_REG16(CM_CMU->I2SCKSEL, CMU_I2SCKSEL_I2S3CKSEL, cfg->pll_src_cfg << CMU_I2SCKSEL_I2S1CKSEL_POS);
			break;
		case HC32_FCG1_PERIPH_I2S4:
			MODIFY_REG16(CM_CMU->I2SCKSEL, CMU_I2SCKSEL_I2S4CKSEL, cfg->pll_src_cfg << CMU_I2SCKSEL_I2S1CKSEL_POS);
			break;
		default:
			break;
		}
	} else {
		MODIFY_REG32(cfg->i2s->CTRL, (I2S_CTRL_CLKSEL | I2S_CTRL_I2SPLLSEL), 
									  I2S_CLK_SRC_EXT);
		CLR_REG32_BIT(cfg->i2s->CTRL, I2S_CTRL_MCKOE );
	}

	MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_TXBIRQWL, I2S_TRANS_LVL4);
	MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_RXBIRQWL, I2S_RECEIVE_LVL0);
	
	return 0;
}

static int i2s_start_dma(const struct device *dev, uint32_t channel,
					struct dma_config *config)
{
	int ret;

	ret = dma_config(dev, channel, config);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(dev, channel);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static int i2s_reload_dma(const struct device *dev, uint32_t channel,
			uint32_t src, uint32_t dst, size_t size)		
{
	int ret;

	ret = dma_reload(dev, channel, src, dst, size);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(dev, channel);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static void i2s_hc32_tx_disable(const struct device *dev)
{
	const struct i2s_hc32_cfg *cfg = dev->config;
	uint32_t timeout = 0;

	while (SET != I2S_GetStatus(cfg->i2s, I2S_FLAG_TX_ERR)) {
		if (++timeout >= 10000) {
			break;
		}
	}
	I2S_FuncCmd(cfg->i2s, I2S_FUNC_TX, DISABLE);
	I2S_SWReset(cfg->i2s, I2S_RST_TYPE_FIFO);
	I2S_ClearStatus(cfg->i2s, I2S_FLAG_TX_ERR);
}

static void i2s_hc32_tx_dma_callback(const struct device *dev, void *user_data,
				uint32_t channel, int status)
{
	struct dma_hc32_config_user_data *hc32_user_data = \
		(struct dma_hc32_config_user_data *)user_data;
	const struct device *i2s_dev = \
		(const struct device *)hc32_user_data->user_data;
	const struct i2s_hc32_cfg *cfg = i2s_dev->config;
	struct i2s_hc32_data *dev_data = i2s_dev->data;
	struct dma_config *i2s_dma_config = cfg->dma_conf;
	struct dma_block_config *i2s_dma_block_conf = i2s_dma_config[0].head_block;
	uint32_t remain;
	int ret;
	static uint16_t i = 0;
	static uint32_t buf[6];

	dma_stop(dev, channel);

	MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_TXBIRQWL, I2S_TRANS_LVL4);

	if (status != 0) {
		dev_data->tx.state = I2S_STATE_ERROR;
		I2S_FuncCmd(cfg->i2s, I2S_FUNC_TX, DISABLE);
		I2S_SWReset(cfg->i2s, I2S_RST_TYPE_FIFO);
		return;
	}

	if (channel == cfg->channel[0]) {
		if ((i2s_dma_block_conf->source_address - 
				(uint32_t)dev_data->tx.mem_block + 
				i2s_dma_block_conf->block_size) >= dev_data->tx.len) {
			if (i < 6) {
				buf[i++] = (uint32_t)dev_data->tx.mem_block;
			}
			k_mem_slab_free(dev_data->tx.cfg.mem_slab, dev_data->tx.mem_block);
			dev_data->tx.mem_block = NULL;

			if (dev_data->tx.state == I2S_STATE_STOPPING) {
				dev_data->tx.state = I2S_STATE_READY;
				i2s_hc32_tx_disable(i2s_dev);
				return;
			}

			ret= k_msgq_get(&dev_data->tx_queue, &dev_data->tx.mem_block, K_NO_WAIT);
			if (ret == 0) {
				dev_data->w_o_index = \
				(++dev_data->w_o_index >= (CONFIG_I2S_HC32_TX_BLOCK_COUNT + 1)) ?
					0 : dev_data->w_o_index;
				i2s_dma_block_conf->source_address = (uint32_t)dev_data->tx.mem_block;
				i2s_dma_block_conf->block_size = \
					dev_data->tx.len > (UINT16_MAX / 4 * 4) ? 
					(UINT16_MAX / 4 * 4) : dev_data->tx.len;
				ret = i2s_reload_dma(dev, channel, 
								i2s_dma_block_conf->source_address, 
								(uint32_t)&cfg->i2s->TXBUF,
								i2s_dma_block_conf->block_size);
				if (ret != 0) {
					dev_data->tx.state = I2S_STATE_READY;
					dev_data->w_i_index = \
									(++dev_data->w_i_index >= 
									(CONFIG_I2S_HC32_RX_BLOCK_COUNT + 1)) ? 
									0 : dev_data->w_i_index;
					k_msgq_put(&dev_data->tx_queue, &dev_data->tx.mem_block,
															K_NO_WAIT);
					i2s_hc32_tx_disable(i2s_dev);
					return;
				}
				MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_TXBIRQWL, I2S_TRANS_LVL3);
			} else {
				dev_data->tx.state = I2S_STATE_READY;
				i2s_hc32_tx_disable(i2s_dev);
			}
		} else {
			i2s_dma_block_conf->source_address += i2s_dma_block_conf->block_size;
			remain = dev_data->tx.len - (i2s_dma_block_conf->source_address - 
				(uint32_t)dev_data->tx.mem_block);
			i2s_dma_block_conf->block_size = \
			(remain > (UINT16_MAX / 4 * 4)) ? (UINT16_MAX / 4 * 4) : remain;
			ret = i2s_reload_dma(dev, channel, 
								i2s_dma_block_conf->source_address, 
								(uint32_t)&cfg->i2s->TXBUF, 
								i2s_dma_block_conf->block_size);
			if (ret != 0) {
				dev_data->tx.state = I2S_STATE_READY;
				i2s_hc32_tx_disable(i2s_dev);
				k_mem_slab_free(dev_data->tx.cfg.mem_slab, 
									dev_data->tx.mem_block);
				return;
			}
			MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_TXBIRQWL, I2S_TRANS_LVL3);
		}
	}
}

static void i2s_hc32_rx_dma_callback(const struct device *dev, void *user_data,
				uint32_t channel, int status)
{
	struct dma_hc32_config_user_data *hc32_user_data = \
		(struct dma_hc32_config_user_data *)user_data;
	const struct device *i2s_dev = \
		(const struct device *)hc32_user_data->user_data;
	const struct i2s_hc32_cfg *cfg = i2s_dev->config;
	struct i2s_hc32_data *dev_data = i2s_dev->data;
	struct dma_config *i2s_dma_config = cfg->dma_conf;
	struct dma_block_config *i2s_dma_block_conf = i2s_dma_config[1].head_block;
	uint32_t remain;
	int ret;

	MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_RXBIRQWL, I2S_RECEIVE_LVL0);

	/*if (RESET != I2S_GetStatus(cfg->i2s, I2S_FLAG_RX_ERR)) {
		I2S_ClearStatus(cfg->i2s, I2S_FLAG_RX_ERR);
		i2s_dma_disable(i2s_dev, I2S_DIR_RX);
		dev_data->rx.state = I2S_STATE_ERROR;
		return;
	}*/

	dma_stop(dev, channel);

	if (status != 0) {
		dev_data->rx.state = I2S_STATE_ERROR;
		i2s_dma_disable(i2s_dev, I2S_DIR_RX);
		return;
	}

	if ((i2s_dma_block_conf->dest_address - (uint32_t)dev_data->rx.mem_block + 
			i2s_dma_block_conf->block_size) < dev_data->rx.cfg.block_size) {
		i2s_dma_block_conf->dest_address += i2s_dma_block_conf->block_size;
		remain = dev_data->rx.cfg.block_size -
		(i2s_dma_block_conf->dest_address - (uint32_t)dev_data->rx.mem_block);
		i2s_dma_block_conf->block_size = \
		(remain > (UINT16_MAX / 4 * 4)) ? (UINT16_MAX / 4 * 4) : remain;
		ret = i2s_reload_dma(dev, channel, (uint32_t)&cfg->i2s->RXBUF, 
				i2s_dma_block_conf->dest_address, 
				i2s_dma_block_conf->block_size);
		if (ret != 0) {
			dev_data->rx.state = I2S_STATE_READY;
			i2s_dma_disable(i2s_dev, I2S_DIR_RX);
			return;
		}
		if (RESET != I2S_GetStatus(cfg->i2s, I2S_FLAG_RX_ERR)) {
			I2S_ClearStatus(cfg->i2s, I2S_FLAG_RX_ERR);
		}
		MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_RXBIRQWL, I2S_RECEIVE_LVL3);
	} else {
		ret = k_msgq_put(&dev_data->rx_queue, &dev_data->rx.mem_block, K_NO_WAIT);
		if (ret < 0) {
			dev_data->rx.state = I2S_STATE_ERROR;
			i2s_dma_disable(i2s_dev, I2S_DIR_RX);
			return;
		}

		if (dev_data->rx.state == I2S_STATE_STOPPING) {
			dev_data->rx.state = I2S_STATE_READY;
			I2S_FuncCmd(cfg->i2s, I2S_FUNC_RX, DISABLE);
			I2S_SWReset(cfg->i2s, I2S_RST_TYPE_FIFO);
			return;
		}

		ret = k_mem_slab_alloc(dev_data->rx.cfg.mem_slab, &dev_data->rx.mem_block, 
								K_NO_WAIT);
		if (ret != 0) {
			dev_data->rx.state = I2S_STATE_ERROR;
			i2s_dma_disable(i2s_dev, I2S_DIR_RX);
			return;
		}

		i2s_dma_block_conf->dest_address = (uint32_t)dev_data->rx.mem_block;
		i2s_dma_block_conf->block_size = 
			dev_data->rx.cfg.block_size > (UINT16_MAX / 4 * 4) ? 
			(UINT16_MAX / 4 * 4) : dev_data->rx.cfg.block_size;
		ret = i2s_reload_dma(dev, channel, (uint32_t)&cfg->i2s->RXBUF, 
								i2s_dma_block_conf->dest_address, 
								i2s_dma_block_conf->block_size);
		if (ret != 0) {
			dev_data->rx.state = I2S_STATE_READY;
			i2s_dma_disable(i2s_dev, I2S_DIR_RX);
			return;
		}
		if (RESET != I2S_GetStatus(cfg->i2s, I2S_FLAG_RX_ERR)) {
			I2S_ClearStatus(cfg->i2s, I2S_FLAG_RX_ERR);
		}
		MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_RXBIRQWL, I2S_RECEIVE_LVL3);
	}
}

static int i2s_hc32_tx_start(const struct device *dev)
{
	const struct i2s_hc32_cfg *cfg = dev->config;
	struct i2s_hc32_data *dev_data = dev->data;
	struct dma_config *i2s_dma_config = cfg->dma_conf;
	struct dma_block_config *i2s_tx_dma_block_conf = \
					i2s_dma_config[0].head_block;
	int ret = 0;

	ret= k_msgq_get(&dev_data->tx_queue, &dev_data->tx.mem_block, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("No buffer in input queue to start");
		return -ENOMEM;
	}
	dev_data->w_o_index = \
		(++dev_data->w_o_index >= (CONFIG_I2S_HC32_TX_BLOCK_COUNT + 1)) ?
			0 : dev_data->w_o_index;
			
	if ((i2s_tx_dma_block_conf->dest_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) && \
	 	(i2s_tx_dma_block_conf->source_addr_adj != DMA_ADDR_ADJ_INCREMENT)) {
		return -EIO;
	}

	if (dev_data->tx.cfg.word_size == 16) {
		i2s_dma_config[0].source_data_size = i2s_dma_config[0].dest_data_size = 2;
	} else {
		i2s_dma_config[0].source_data_size = i2s_dma_config[0].dest_data_size = 4;
	}

	i2s_tx_dma_block_conf->source_address = (uint32_t)dev_data->tx.mem_block;
	i2s_tx_dma_block_conf->block_size = \
		dev_data->tx.len > (UINT16_MAX / 4 * 4) ? (UINT16_MAX / 4 * 4) : dev_data->tx.len;
	ret = i2s_start_dma(cfg->dma_dev[0], cfg->channel[0], i2s_dma_config);
	if (ret != 0) {
		dev_data->w_i_index = \
						(++dev_data->w_i_index >= 
						(CONFIG_I2S_HC32_RX_BLOCK_COUNT + 1)) ? 
						0 : dev_data->w_i_index;
		if (dev_data->w_i_index == dev_data->w_o_index) {
			return -ENOMEM;
		}
		ret = k_msgq_put(&dev_data->tx_queue, &dev_data->tx.mem_block,
															K_NO_WAIT);
		if (ret < 0) {
			return -ENOMEM;
		}
		return -EIO;
	}
	MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_TXBIRQWL, I2S_TRANS_LVL3);
	I2S_FuncCmd(cfg->i2s, I2S_FUNC_TX, ENABLE);

	return ret;
}

static int i2s_hc32_rx_start(const struct device *dev)
{
	const struct i2s_hc32_cfg *cfg = dev->config;
	struct i2s_hc32_data *dev_data = dev->data;
	struct dma_config *i2s_dma_config = cfg->dma_conf;
	struct dma_block_config *i2s_rx_dma_block_conf = \
					i2s_dma_config[1].head_block;
	int ret = 0;

	ret = k_mem_slab_alloc(dev_data->rx.cfg.mem_slab, &dev_data->rx.mem_block, 
								K_NO_WAIT);
	if (ret != 0) {
		return -ENOMEM;
	}

	if ((i2s_rx_dma_block_conf->dest_addr_adj != DMA_ADDR_ADJ_INCREMENT) && \
	 	(i2s_rx_dma_block_conf->source_addr_adj != DMA_ADDR_ADJ_NO_CHANGE)) {
		return -EIO;
	}

	if (dev_data->rx.cfg.word_size == 16) {
		i2s_dma_config[1].source_data_size = i2s_dma_config[1].dest_data_size = 2;
	} else {
		i2s_dma_config[1].source_data_size = i2s_dma_config[1].dest_data_size = 4;
	}

	i2s_rx_dma_block_conf->dest_address = (uint32_t)dev_data->rx.mem_block;
	i2s_rx_dma_block_conf->block_size = 
		dev_data->rx.cfg.block_size > (UINT16_MAX / 4 * 4) ? 
		(UINT16_MAX / 4 * 4) : dev_data->rx.cfg.block_size;
	ret = i2s_start_dma(cfg->dma_dev[1], cfg->channel[1], &i2s_dma_config[1]);
	if (ret != 0) {
		k_mem_slab_free(dev_data->rx.cfg.mem_slab, dev_data->rx.mem_block);
		return EIO;
	}

	if (RESET != I2S_GetStatus(cfg->i2s, I2S_FLAG_RX_ERR)) {
		I2S_ClearStatus(cfg->i2s, I2S_FLAG_RX_ERR);
	}
	MODIFY_REG32(cfg->i2s->CTRL, I2S_CTRL_RXBIRQWL, I2S_RECEIVE_LVL3);
	I2S_FuncCmd(cfg->i2s, I2S_FUNC_RX, ENABLE);

	return ret;
}

#define HC32_I2C_DMA_INIT(inst)	\
static void i2s_hc32_tx_dma_callback(const struct device *dev, void *user_data,	\
				uint32_t channel, int status);	\
static void i2s_hc32_rx_dma_callback(const struct device *dev, void *user_data,	\
				uint32_t channel, int status);	\
const uint32_t i2s_dma_config_vaule_##inst[2] = {HC32_DMA_CHANNEL_CONFIG(inst, tx_dma), \
								HC32_DMA_CHANNEL_CONFIG(inst, rx_dma)}; \
struct dma_block_config dma_block_config_##inst[2] = \
{	\
	{	\
		.dest_addr_adj = HC32_DMA_CONFIG_DEST_ADDR_INC(i2s_dma_config_vaule_##inst[0]), \
		.source_addr_adj = HC32_DMA_CONFIG_SOURCE_ADDR_INC(i2s_dma_config_vaule_##inst[0]), \
		.dest_address = (uint32_t)&(((CM_I2S_TypeDef *)DT_INST_REG_ADDR(inst))->TXBUF),	\
	},		\
	{	\
		.source_addr_adj = HC32_DMA_CONFIG_SOURCE_ADDR_INC(i2s_dma_config_vaule_##inst[1]), \
		.dest_addr_adj = HC32_DMA_CONFIG_DEST_ADDR_INC(i2s_dma_config_vaule_##inst[1]), \
		.source_address = (uint32_t)&(((CM_I2S_TypeDef *)DT_INST_REG_ADDR(inst))->RXBUF),	\
	}		\
};	\
struct dma_hc32_config_user_data i2s_dma_user_data_##inst[2] = \
{	\
	{	\
		.slot = HC32_DMA_SLOT(inst, tx_dma),\
		.user_data = (void*)DEVICE_DT_INST_GET(inst),	\
	},	\
	{	\
		.slot = HC32_DMA_SLOT(inst, rx_dma),\
		.user_data = (void*)DEVICE_DT_INST_GET(inst),	\
	},	\
}; 	\
struct dma_config i2s_dma_config_##inst[2] = \
{	\
	{	\
		.channel_direction = HC32_DMA_CONFIG_DIRECTION(i2s_dma_config_vaule_##inst[0]), \
		.block_count = 1,	\
		.source_burst_length = 1,	\
		.dest_burst_length = 1,	\
		.source_data_size = HC32_DMA_CONFIG_DATA_SIZE(i2s_dma_config_vaule_##inst[0]),	\
		.dest_data_size = HC32_DMA_CONFIG_DATA_SIZE(i2s_dma_config_vaule_##inst[0]),	\
		.head_block = &dma_block_config_##inst[0],	\
		.user_data = &i2s_dma_user_data_##inst[0],	\
		.dma_callback = i2s_hc32_tx_dma_callback,	\
	},	\
	{	\
		.channel_direction = HC32_DMA_CONFIG_DIRECTION(i2s_dma_config_vaule_##inst[1]), \
		.block_count = 1,	\
		.source_burst_length = 1,	\
		.dest_burst_length = 1,	\
		.source_data_size = HC32_DMA_CONFIG_DATA_SIZE(i2s_dma_config_vaule_##inst[1]),	\
		.dest_data_size = HC32_DMA_CONFIG_DATA_SIZE(i2s_dma_config_vaule_##inst[1]),	\
		.head_block = &dma_block_config_##inst[1],	\
		.user_data = &i2s_dma_user_data_##inst[1],	\
		.dma_callback = i2s_hc32_rx_dma_callback,	\
	}	\
};
#define HC32_I2S_DMA_CONFIG(inst)	\
	.dma_dev = {	\
		DEVICE_DT_GET(HC32_DMA_CTLR(inst, tx_dma)),	\
		DEVICE_DT_GET(HC32_DMA_CTLR(inst, rx_dma)),	\
	},	\
	.dma_conf = i2s_dma_config_##inst,		\
	.channel = {	\
		HC32_DMA_CHANNEL(inst, tx_dma),	\
		HC32_DMA_CHANNEL(inst, rx_dma),	\
	}	

#define I2S_HC32_INIT(index)							\
HC32_I2C_DMA_INIT(index)			\
static struct i2s_hc32_data_queue i2s_hc32_data_queue_##index[CONFIG_I2S_HC32_TX_BLOCK_COUNT + 1];	\
									\
PINCTRL_DT_INST_DEFINE(index);						\
									\
static const struct hc32_modules_clock_sys mudules_clk_##index[] =			\
				 HC32_MODULES_CLOCKS(DT_DRV_INST(index));		\
									\
static const struct i2s_hc32_cfg i2s_hc32_config_##index = {		\
	.i2s = (CM_I2S_TypeDef *)DT_INST_REG_ADDR(index),			\
	.mod_clk = mudules_clk_##index,						\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),			\
	.i2s_src_clk = DT_PROP_OR(DT_DRV_INST(index), src_clk, 0),	\
	.pll_src_cfg = DT_PROP_OR(DT_DRV_INST(index), i2s_pll_clk, 0),	\
	.tx_fifo_level = DT_PROP_OR(DT_DRV_INST(index), tx_fifo_level, 2),	\
	.rx_fifo_level = DT_PROP_OR(DT_DRV_INST(index), rx_fifo_level, 2),	\
	HC32_I2S_DMA_CONFIG(index),		\
};									\
									\
static struct i2s_hc32_data i2s_hc32_data_##index = {			\
	.write_queue = i2s_hc32_data_queue_##index,	\
};									\
DEVICE_DT_INST_DEFINE(index,						\
		      &i2s_hc32_init, NULL,			\
		      &i2s_hc32_data_##index,				\
		      &i2s_hc32_config_##index, POST_KERNEL,		\
		      CONFIG_I2S_INIT_PRIORITY, &i2s_hc32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_HC32_INIT)
