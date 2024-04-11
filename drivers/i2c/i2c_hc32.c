/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <clock_control/hc32_clock_control.h>
#include <clock_control.h>
#include <interrupt_controller/intc_hc32.h>
#include <misc/util.h>
#include <kernel.h>
#include <soc.h>
#include <i2c.h>
#include <errno.h>
#include "i2c_hc32.h"

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_hc32);

#include "i2c-priv.h"

#ifdef CONFIG_I2C_HC32_DMA
#include <drivers/dma/dma_hc32.h>
#endif
#ifdef CONFIG_I2C_HC32_BUS_RECOVERY
#include "i2c_bitbang.h"
#endif /* CONFIG_I2C_HC32_BUS_RECOVERY */

#define I2C_0_IRQ_NAME	eei
#define I2C_1_IRQ_NAME	tei
#define I2C_2_IRQ_NAME	txi
#define I2C_3_IRQ_NAME	rxi

#define HC32_I2C_TRANSFER_TIMEOUT_MSEC  500

static void hc32_hw_i2c_reset(const struct i2c_hc32_config *cfg);
static int hc32_hw_i2c_start(const struct i2c_hc32_config *cfg);
static int hc32_hw_i2c_restart(const struct i2c_hc32_config *cfg);
static int hc32_hw_i2c_send_addr(struct device *dev);
static int hc32_hw_i2c_stop(const struct i2c_hc32_config *cfg);

#ifdef CONFIG_I2C_HC32_BUS_RECOVERY
pinctrl_soc_pin_t pins[2];
struct pinctrl_state pinctl_default_sta = \
{
	.pins = &pins[0],
};
struct pinctrl_dev_config pinctl_dev_default_conf = \
{
	.states = &pinctl_default_sta,
};

static void i2c_hc32_bitbang_set_scl(void *io_context, int state)
{
	struct i2c_hc32_config *config = io_context;

	gpio_pin_set_dt(&config->scl, state);
}

static void i2c_hc32_bitbang_set_sda(void *io_context, int state)
{
	struct i2c_hc32_config *config = io_context;

	gpio_pin_set_dt(&config->sda, state);
}

static int i2c_hc32_bitbang_get_sda(void *io_context)
{
	struct i2c_hc32_config *config = io_context;

	return gpio_pin_get_dt(&config->sda) == 0 ? 0 : 1;
}

static int i2c_hc32_recover_bus(struct device *dev)
{
	uint8_t i;
	struct i2c_hc32_config *config = dev->config->config_info;
	struct i2c_hc32_data *data = dev->driver_data;
	struct i2c_bitbang bitbang_ctx;
	struct i2c_bitbang_io bitbang_io = {
		.set_scl = i2c_hc32_bitbang_set_scl,
		.set_sda = i2c_hc32_bitbang_set_sda,
		.get_sda = i2c_hc32_bitbang_get_sda,
	};
	uint32_t bitrate_cfg;
	int error = 0;

	LOG_ERR("attempting to recover bus");

	if (!gpio_is_ready_dt(&config->scl)) {
		LOG_ERR("SCL GPIO device not ready");
		return -EIO;
	}

	if (!gpio_is_ready_dt(&config->sda)) {
		LOG_ERR("SDA GPIO device not ready");
		return -EIO;
	}

	pinctl_dev_default_conf.state_cnt = config->pcfg->state_cnt;
	pinctl_default_sta.id = config->pcfg->states->id;
	pinctl_default_sta.pin_cnt = config->pcfg->states->pin_cnt;
	for (i=0; i< pinctl_default_sta.pin_cnt; i++) {
		pins[i] = config->pcfg->states->pins[i] & \
			(~(HC32_FUNC_MSK << HC32_FUNC_SHIFT));
	}

	k_sem_take(&data->bus_mutex, K_FOREVER);

	(void)pinctrl_apply_state(&pinctl_dev_default_conf, PINCTRL_STATE_DEFAULT);

	error = gpio_pin_configure_dt(&config->scl, GPIO_OUTPUT_HIGH);
	if (error != 0) {
		LOG_ERR("failed to configure SCL GPIO (err %d)", error);
		goto restore;
	}

	error = gpio_pin_configure_dt(&config->sda, GPIO_OUTPUT_HIGH);
	if (error != 0) {
		LOG_ERR("failed to configure SDA GPIO (err %d)", error);
		goto restore;
	}

	i2c_bitbang_init(&bitbang_ctx, &bitbang_io, (void *)config);

	bitrate_cfg = i2c_map_dt_bitrate(config->bitrate) | I2C_MODE_CONTROLLER;
	error = i2c_bitbang_configure(&bitbang_ctx, bitrate_cfg);
	if (error != 0) {
		LOG_ERR("failed to configure I2C bitbang (err %d)", error);
		goto restore;
	}

	error = i2c_bitbang_recover_bus(&bitbang_ctx);
	if (error != 0) {
		LOG_ERR("failed to recover bus (err %d)", error);
	}

restore:
	I2C_Cmd(config->i2c, ENABLE);
	hc32_hw_i2c_reset(config);
	I2C_Cmd(config->i2c, DISABLE);
	(void)pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	k_sem_give(&data->bus_mutex);

	return error;
}
#endif // CONFIG_I2C_HC32_BUS_RECOVERY
static void hc32_i2c_start(const struct i2c_hc32_config *cfg)
{
	CM_I2C_TypeDef *i2c = cfg->i2c;

	I2C_Cmd(i2c, ENABLE);
	hc32_hw_i2c_reset(cfg);
	hc32_hw_i2c_start(cfg);
}

static void hc32_i2c_restart(const struct i2c_hc32_config *cfg)
{
	CM_I2C_TypeDef *i2c = cfg->i2c;

	if (RESET == I2C_GetStatus(i2c, I2C_FLAG_MASTER)) {
		I2C_Cmd(i2c, ENABLE);
		hc32_hw_i2c_reset(cfg);
		SET_REG32_BIT(i2c->SR, I2C_FLAG_MASTER);
		SET_REG32_BIT(i2c->SR, I2C_FLAG_TRA);
	}
	hc32_hw_i2c_restart(cfg);
}

#ifdef CONFIG_I2C_HC32_INTERRUPT
static void hc32_i2c_transaction_end(struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	struct i2c_hc32_data *data = dev->driver_data;

	I2C_IntCmd(cfg->i2c, I2C_INT_TX_EMPTY | I2C_INT_RX_FULL | \
				I2C_INT_TX_CPLT, DISABLE);

	k_sem_give(&data->device_sync_sem);
}

void hc32_i2c_eei_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->driver_data;

	if (SET == I2C_GetStatus(i2c, I2C_FLAG_START)) {
		I2C_ClearStatus(i2c, I2C_CLR_STARTFCLR | I2C_CLR_NACKFCLR);
		I2C_IntCmd(i2c, I2C_INT_STOP | I2C_INT_NACK, ENABLE);
		if (data->dir == I2C_DIR_TX) {
			I2C_IntCmd(i2c, I2C_INT_TX_EMPTY, ENABLE);
			I2C_WriteData(i2c, (uint8_t)(data->slaver_addr << 1) | I2C_DIR_TX);
		} else {
			if (data->len == 1) {
				I2C_AckConfig(i2c, I2C_NACK);
			}
			I2C_IntCmd(i2c, I2C_INT_RX_FULL | I2C_INT_TX_CPLT, ENABLE);
			I2C_WriteData(i2c, (uint8_t)(data->slaver_addr << 1) | I2C_DIR_RX);
		}
	}

	if (SET == I2C_GetStatus(i2c, I2C_FLAG_STOP)) {
		I2C_ClearStatus(i2c, I2C_CLR_STOPFCLR);
		hc32_i2c_transaction_end(dev);
		I2C_IntCmd(i2c, I2C_INT_NACK | I2C_INT_STOP, DISABLE);
		I2C_Cmd(i2c, DISABLE);
	}

	if (SET == I2C_GetStatus(i2c, I2C_FLAG_NACKF)) {
		/* clear NACK flag*/
		I2C_ClearStatus(i2c, I2C_CLR_NACKFCLR);
		/* Stop tx or rx process*/
		I2C_IntCmd(i2c, I2C_INT_TX_EMPTY | I2C_INT_RX_FULL | \
			I2C_INT_TX_CPLT | I2C_INT_NACK, DISABLE);

		/* Generate stop condition */
		I2C_GenerateStop(i2c);
	}
}

void hc32_i2c_tei_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->driver_data;


	if (data->len == 0) {
		if (data->msg->flags & I2C_MSG_STOP) {
			I2C_IntCmd(i2c, I2C_INT_STOP, ENABLE);
			I2C_GenerateStop(i2c);
		} else {
			hc32_i2c_transaction_end(dev);
		}
	}
}

void hc32_i2c_txi_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->driver_data;

	if (data->len) {
		I2C_WriteData(i2c, *data->dat);
		data->len --;
		data->dat ++;
	} else {
		I2C_IntCmd(i2c, I2C_INT_TX_EMPTY, DISABLE);
		I2C_IntCmd(i2c, I2C_INT_TX_CPLT, ENABLE);
	}
}

void hc32_i2c_rxi_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->driver_data;

	if (data->msg->flags & I2C_MSG_STOP) {
		if (data->len == 2) {
			I2C_AckConfig(i2c, I2C_NACK);
		}
	}

	if (data->len) {
		data->len --;
		if (data->len == 0) {
			if (data->msg->flags & I2C_MSG_STOP) {
				I2C_IntCmd(i2c, I2C_INT_STOP, ENABLE);
				I2C_IntCmd(i2c, I2C_INT_RX_FULL, DISABLE);
				I2C_GenerateStop(i2c);
				I2C_AckConfig(i2c, I2C_ACK);
			} else {
				/*hc32_i2c_transaction_end(dev);*/
				k_sem_give(&data->device_sync_sem);
			}
		}
		*data->dat = I2C_ReadData(i2c);
		data->dat ++;
	}
}

static int hc32_i2c_msg_transaction(struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->driver_data;

	if (data->msg->flags & I2C_MSG_RESTART) {
		I2C_IntCmd(i2c, I2C_INT_START, ENABLE);
		if (SET == I2C_GetStatus(i2c, I2C_FLAG_BUSY)) {
			hc32_i2c_restart(cfg);
		} else {
			hc32_i2c_start(cfg);
		}
	} else {
		if (data->len) {
			if ((data->msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
				I2C_IntCmd(i2c, I2C_INT_TX_EMPTY, ENABLE);
				data->len --;
				I2C_WriteData(i2c, *data->dat++);
			} else if ((data->msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
				if (SET == I2C_GetStatus(i2c, I2C_FLAG_RX_FULL)) {
					data->len --;
					*data->dat++ = I2C_ReadData(i2c);
				}
				I2C_IntCmd(i2c, I2C_INT_RX_FULL, ENABLE);
			}
		} else {
			I2C_GenerateStop(i2c);
		}
	}
	
	if (k_sem_take(&data->device_sync_sem,
			K_MSEC(HC32_I2C_TRANSFER_TIMEOUT_MSEC)) != 0) {
		I2C_IntCmd(i2c, I2C_INT_NACK | I2C_INT_STOP, DISABLE);
		hc32_hw_i2c_stop(cfg);
		hc32_i2c_transaction_end(dev);
		I2C_Cmd(i2c, DISABLE);
		k_sem_take(&data->device_sync_sem, K_FOREVER);
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_I2C_HC32_INTERRUPT */

#ifdef CONFIG_I2C_HC32_DMA
static void hc32_dma_callback(void *user_data,
				uint32_t channel, int status)
{
	struct dma_hc32_config_user_data *hc32_user_data = \
		(struct dma_hc32_config_user_data *)user_data;
	struct device *i2c_dev = \
		(struct device *)hc32_user_data->user_data;
	struct i2c_hc32_data *data = i2c_dev->driver_data;

	k_sem_give(&data->device_sync_sem);
	dma_stop(data->dma_dev[0], channel);
}

static int hc32_i2c_dma_write(struct device *dev)
{
	uint32_t timeout = 0;
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->driver_data;
	struct dma_config *i2c_dma_config = data->dma_conf;
	struct dma_block_config *i2c_tx_dma_block_conf = \
					i2c_dma_config[0].head_block;

	if ((i2c_tx_dma_block_conf->dest_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) && \
	 	(i2c_tx_dma_block_conf->source_addr_adj != DMA_ADDR_ADJ_INCREMENT)) {
		return EFAULT;
	}

	if (data->len > 1) {
		i2c_tx_dma_block_conf->source_address = (uint32_t)(&data->dat[1]);
		//i2c_tx_dma_block_conf->dest_address = (uint32_t)&i2c->DTR;
		i2c_tx_dma_block_conf->block_size = data->len- 1;
		dma_config(data->dma_dev[0], data->channel[0], i2c_dma_config);
		dma_start(data->dma_dev[0], data->channel[0]);
	}

	I2C_WriteData(i2c, *data->dat);
	if (data->len > 1) {
		if (k_sem_take(&data->device_sync_sem,
				K_MSEC(HC32_I2C_TRANSFER_TIMEOUT_MSEC)) != 0) {
			dma_stop(data->dma_dev[0], data->channel[0]);
			return -EIO;
		}
	}

	while ((LL_OK != I2C_WaitStatus(i2c, I2C_FLAG_TX_CPLT, SET, 1)) && \
			(timeout < cfg->time_out)) {
		k_sleep(K_MSEC(1));
		timeout++;
	}

	if (timeout >= cfg->time_out) {
		return -EIO;
	}
	return 0;
}

static int hc32_i2c_dma_read(struct device *dev)
{
	uint32_t timeout = 0;
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->driver_data;
	struct dma_config *i2c_dma_config = data->dma_conf;
	struct dma_block_config *i2c_rx_dma_block_conf = \
					i2c_dma_config[1].head_block;

	if ((i2c_rx_dma_block_conf->dest_addr_adj != DMA_ADDR_ADJ_INCREMENT) && \
	 	(i2c_rx_dma_block_conf->source_addr_adj != DMA_ADDR_ADJ_NO_CHANGE)) {
		return EFAULT;
	}

	if (data->len == 1) {
		I2C_AckConfig(i2c, I2C_NACK);
	} else if (data->len > 2) {
		//i2c_rx_dma_block_conf->source_address = (uint32_t)&i2c->DRR;
		i2c_rx_dma_block_conf->dest_address = (uint32_t)(&data->dat[0]);
		i2c_rx_dma_block_conf->block_size = data->len- 2;
		dma_config(data->dma_dev[1], data->channel[1], &i2c_dma_config[1]);
		dma_start(data->dma_dev[1], data->channel[1]);
	}

	if (data->len > 2) {
		if (k_sem_take(&data->device_sync_sem,
				K_MSEC(HC32_I2C_TRANSFER_TIMEOUT_MSEC)) != 0) {
			dma_stop(data->dma_dev[1], data->channel[1]);
			return -EIO;
		}
	}
	if (data->len > 1) {
		while ((LL_OK != I2C_WaitStatus(i2c, I2C_FLAG_RX_FULL, SET, 1)) && \
			(timeout < cfg->time_out)) {
			k_sleep(K_MSEC(1));
			timeout++;
		}
		if (timeout >= cfg->time_out) {
			return -EIO;
		}
		data->dat[data->len - 2] = I2C_ReadData(i2c);
		I2C_AckConfig(i2c, I2C_NACK);
	}

	timeout = 0;
	while ((LL_OK != I2C_WaitStatus(i2c, I2C_FLAG_RX_FULL, SET, 1)) && \
		(timeout < cfg->time_out)) {
		k_sleep(K_MSEC(1));
		timeout++;
	}
	if (timeout >= cfg->time_out) {
		return -EIO;
	}
	I2C_ClearStatus(i2c, I2C_FLAG_STOP);
	I2C_GenerateStop(i2c);
	data->dat[data->len - 1] = I2C_ReadData(i2c);

	timeout = 0;
	while ((LL_OK != I2C_WaitStatus(i2c, I2C_FLAG_STOP, SET, 1)) && \
		(timeout < cfg->time_out))
	{
		k_sleep(K_MSEC(1));
		timeout++;
	}
	if (timeout >= cfg->time_out) {
		return -EIO;
	}
	I2C_AckConfig(i2c, I2C_ACK);

	return 0;
}

static int hc32_i2c_msg_transaction(struct device *dev)
{
	uint32_t ret = 0;
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->driver_data;
	
	if (data->msg->flags & I2C_MSG_RESTART) {
		if (SET == I2C_GetStatus(i2c, I2C_FLAG_BUSY)) {
			hc32_i2c_restart(cfg);
		} else {
			hc32_i2c_start(cfg);
		}

		if (0 != hc32_hw_i2c_send_addr(dev)) {
			ret = 1;
			LOG_ERR("no receive ack form device addr 0x%02x", data->slaver_addr);
		}
	}

	if (data->dir == I2C_DIR_TX) {
		if (0 != hc32_i2c_dma_write(dev)) {
			ret = 2;
			LOG_ERR("[%s:%d]I2C Write error!\n", __func__, __LINE__);
		}
	} else {
		if (0 != hc32_i2c_dma_read(dev)) {
			ret = 3;
			LOG_ERR("[%s:%d]I2C read error!\n", __func__, __LINE__);
		}
	}

	if ((data->msg->flags & I2C_MSG_STOP) || (ret > 0)) {
		I2C_ReadData(i2c);
		hc32_hw_i2c_stop(cfg);
		I2C_Cmd(i2c, DISABLE);
	}

	return ret;
}
#endif /* CONFIG_I2C_HC32_DMA */

static int hc32_i2c_config(struct device *dev)
{
	stc_i2c_init_t stcI2cInit;
	float f32Error = 0.0F;
    uint32_t I2cSrcClk;
    uint32_t I2cClkDiv;
    uint32_t I2cClkDivReg;
	uint32_t baudrate;
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	struct i2c_hc32_data *data = dev->driver_data;
	struct device *clk = device_get_binding(CLOCK_CONTROL);

	if (clock_control_get_rate(clk, \
		(clock_control_subsys_t)cfg->mod_clk, &I2cSrcClk) < 0) {
		return -EFAULT;
	}

	switch (I2C_SPEED_GET(data->dev_config)) {
	case I2C_SPEED_STANDARD:
		baudrate = I2C_BITRATE_STANDARD;
		break;
	case I2C_SPEED_FAST:
		baudrate = I2C_BITRATE_FAST;
		break;
	case I2C_SPEED_FAST_PLUS:
		baudrate = I2C_BITRATE_FAST_PLUS;
		break;
	default:
		baudrate = I2C_BITRATE_STANDARD;
		break;
	}

	I2cClkDiv = I2cSrcClk / baudrate / I2C_WIDTH_MAX_IMME;
	for (I2cClkDivReg = I2C_CLK_DIV1; I2cClkDivReg <= I2C_CLK_DIV128; I2cClkDivReg++)
	{
		if (I2cClkDiv < (1UL << I2cClkDivReg))
		{
			break;
		}
	}
	stcI2cInit.u32ClockDiv = I2cClkDivReg;
	stcI2cInit.u32SclTime = 400UL * I2cSrcClk / (1UL << I2cClkDivReg) / 1000000000UL;
	stcI2cInit.u32Baudrate = baudrate;
	if (0 == I2C_Init(cfg->i2c, &stcI2cInit, &f32Error))
	{
		I2C_BusWaitCmd(cfg->i2c, ENABLE);
		return 0;
	}
	
	return -EFAULT;
}

int i2c_hc32_runtime_configure(struct device *dev, uint32_t config)
{
	struct i2c_hc32_data *data = dev->driver_data;
	int ret;
	
	data->dev_config = config;

	k_sem_take(&data->bus_mutex, K_FOREVER);
	ret = hc32_i2c_config(dev);
	k_sem_give(&data->bus_mutex);

	return ret;
}

static void hc32_hw_i2c_reset(const struct i2c_hc32_config *cfg)
{
	I2C_SWResetCmd(cfg->i2c, ENABLE);
	I2C_SWResetCmd(cfg->i2c, DISABLE);
}

static int hc32_hw_i2c_start(const struct i2c_hc32_config *cfg)
{
	if (LL_OK != I2C_Start(cfg->i2c, cfg->time_out))
	{
		return -EFAULT;
	}

	return 0;
}

static int hc32_hw_i2c_restart(const struct i2c_hc32_config *cfg)
{
	if (LL_OK != I2C_Restart(cfg->i2c, cfg->time_out))
	{
		return -EFAULT;
	}

	return 0;
}

static int hc32_hw_i2c_stop(const struct i2c_hc32_config *cfg)
{
	if (LL_OK != I2C_Stop(cfg->i2c, cfg->time_out))
	{
		return -EFAULT;
	}
	return 0;
}

__attribute__ ((unused)) \
static int hc32_hw_i2c_send_addr(struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	struct i2c_hc32_data *data = dev->driver_data;

	if (LL_OK != I2C_TransAddr(cfg->i2c, data->slaver_addr, \
					data->dir, cfg->time_out))
    {
        return -EFAULT;
    }

	return 0;
}

#define OPERATION(msg) (((struct i2c_msg *) msg)->flags & I2C_MSG_RW_MASK)

static int i2c_hc32_transfer(struct device *dev, struct i2c_msg *msg,
					uint8_t num_msgs, uint16_t slave)
{
	int ret = 0;
	uint32_t i;
	struct i2c_hc32_data *data = dev->driver_data;
	struct i2c_msg *current, *next;

	current = msg;
	current->flags |= I2C_MSG_RESTART;

	for (i = 0; i < num_msgs; i++){
		if (i < (num_msgs - 1)) {
			next = current + 1;
		
			if (OPERATION(current) != OPERATION(next)) {
				if (!(next->flags & I2C_MSG_RESTART)) {
					return -EINVAL;
				}
			}

			if (current->flags & I2C_MSG_READ) {
				if (next->flags & I2C_MSG_READ) {
					LOG_ERR("consecutive msg flags are read,"
						"make sure the flags are correct and combine the msgs");
					return -EINVAL;
				}
			}

			if (current->flags & I2C_MSG_STOP) {
				return -EINVAL;
			}

			if (current->len == 0) {
				return -EINVAL;
			}
		} else {
			if (current->len == 0) {
				if (current->flags & I2C_MSG_READ) {
					return -EINVAL;
				}
			}
			current->flags |= I2C_MSG_STOP;
		}
		current ++;
	}

	k_sem_take(&data->bus_mutex, K_FOREVER);

	data->slaver_addr = slave;
	for (i = 0; i < num_msgs; i++){
		current = &msg[i];
		data->dat = current->buf;
		data->len = current->len;
		data->msg = &msg[i];

		if ((current->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			data->dir = I2C_DIR_TX;
		} else {
			data->dir = I2C_DIR_RX;
		}

		ret = hc32_i2c_msg_transaction(dev);
		if (0 != ret) {
			break;
		}
	}

	k_sem_give(&data->bus_mutex);

	return ret;
}

static struct i2c_driver_api api_funcs = {
	.configure = i2c_hc32_runtime_configure,
	.transfer = i2c_hc32_transfer,
#if CONFIG_I2C_HC32_BUS_RECOVERY
	.recover_bus = i2c_hc32_recover_bus,
#endif /* CONFIG_I2C_HC32_BUS_RECOVERY */
};

static int i2c_hc32_activate(struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	struct device *clk = device_get_binding(CLOCK_CONTROL);

	/* Move pins to active/default state */

	/* Enable device clock. */
	if (clock_control_on(clk,
			     (clock_control_subsys_t) &cfg->mod_clk[0]) != 0) {
		LOG_ERR("i2c: failure enabling clock");
		return -EIO;
	}

	return 0;
}
#define STRING(x)	#x
#define TO_STRING(x)	STRING(x)
#define HC32_I2C_DMA_UINT_PRASE_F(x)	DMA_##x
#define HC32_I2C_DMA_UINT_PRASE(x)	HC32_I2C_DMA_UINT_PRASE_F(x)
#if defined HC32F460
#define HC32_DMA_SUPPORT_NUM	2
#define HC32_I2C_SUPPORT_NUM	3
#endif
static int i2c_hc32_init(struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config->config_info;
	uint32_t bitrate_cfg;
	int ret;
	struct i2c_hc32_data *data = dev->driver_data;

#ifdef CONFIG_I2C_HC32_INTERRUPT
	cfg->irq_config_func(dev);
#endif
#ifdef CONFIG_I2C_HC32_DMA
	const char *tx_name;
	const char *rx_name;

	__ASSERT((data->uints[0] > HC32_DMA_SUPPORT_NUM) == 0, "uints too large");
	__ASSERT((data->uints[1] > HC32_DMA_SUPPORT_NUM) == 0, "uints too large");

	switch (data->uints[0])
	{
	case 1:
		tx_name = TO_STRING(HC32_I2C_DMA_UINT_PRASE(1));
		break;
	case 2:
		tx_name = TO_STRING(HC32_I2C_DMA_UINT_PRASE(2));
		break;
	case 3:
		tx_name = TO_STRING(HC32_I2C_DMA_UINT_PRASE(3));
		break;
	case 4:
		tx_name = TO_STRING(HC32_I2C_DMA_UINT_PRASE(4));
		break;
	default:
		return -EINVAL;
	}
	switch (data->uints[1])
	{
	case 1:
		rx_name = TO_STRING(HC32_I2C_DMA_UINT_PRASE(1));
		break;
	case 2:
		rx_name = TO_STRING(HC32_I2C_DMA_UINT_PRASE(2));
		break;
	case 3:
		rx_name = TO_STRING(HC32_I2C_DMA_UINT_PRASE(3));
		break;
	case 4:
		rx_name = TO_STRING(HC32_I2C_DMA_UINT_PRASE(4));
		break;
	default:
		return -EINVAL;
	}
	data->dma_dev[0] = device_get_binding(tx_name);
	data->dma_dev[1] = device_get_binding(rx_name);
	((struct dma_hc32_config_user_data *)(data->dma_conf[0].callback_arg))->user_data = (void *)dev;
	((struct dma_hc32_config_user_data *)(data->dma_conf[1].callback_arg))->user_data = (void *)dev;
#endif
#if defined (CONFIG_I2C_HC32_INTERRUPT) || (CONFIG_I2C_HC32_DMA)
	k_sem_init(&data->device_sync_sem, 0, UINT_MAX);
#endif
	/*
	 * initialize mutex used when multiple transfers
	 * are taking place to guarantee that each one is
	 * atomic and has exclusive access to the I2C bus.
	 */
	k_sem_init(&data->bus_mutex, 1, 1);

	i2c_hc32_activate(dev);

	I2C_DeInit(cfg->i2c);

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);

	ret = i2c_hc32_runtime_configure(dev, I2C_MODE_MASTER | bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("i2c: failure initializing");
		return ret;
	}

	return 0;
}

/* Macros for I2C instance declaration */

#ifdef CONFIG_I2C_HC32_INTERRUPT
#define IRQ_REGISTER(num, inst)	\
	IRQ_CONNECT(DT_XHSC_HC32_I2C_##inst##_IRQ_##num,	\
					DT_XHSC_HC32_I2C_##inst##_IRQ_##num##_PRIORITY,\
					_CONCAT(_CONCAT(hc32_i2c_,I2C_##num##_IRQ_NAME), _isr),	\
					DEVICE_GET(i2c_hc32_##inst), 0);		\
	irq_enable(DT_XHSC_HC32_I2C_##inst##_IRQ_##num);	\
	hc32_intc_irq_signin(DT_XHSC_HC32_I2C_##inst##_IRQ_##num, \
					DT_XHSC_HC32_I2C_##inst##_IRQ_##num##_INT_SRC);

#define HC32_I2C_IRQ_CONNECT_AND_ENABLE(index)				\
	UTIL_LISTIFY(DT_XHSC_HC32_I2C_##index##_INT_NUMS,				\
			IRQ_REGISTER, index)

#define HC32_I2C_IRQ_HANDLER_DECL(index)				\
static void i2c_hc32_irq_config_func_##index(struct device *dev)
#define HC32_I2C_IRQ_HANDLER_FUNCTION(index)				\
	.irq_config_func = i2c_hc32_irq_config_func_##index,
#define HC32_I2C_IRQ_HANDLER(index)					\
static void i2c_hc32_irq_config_func_##index(struct device *dev)	\
{									\
	HC32_I2C_IRQ_CONNECT_AND_ENABLE(index);			\
}
#define HC32_I2C_FLAG_TIMEOUT	.time_out = 0,
#else

#define HC32_I2C_IRQ_HANDLER_DECL(index)
#define HC32_I2C_IRQ_HANDLER_FUNCTION(index)
#define HC32_I2C_IRQ_HANDLER(index)
#define HC32_I2C_FLAG_TIMEOUT	.time_out = 3000,
#endif /* CONFIG_I2C_HC32_INTERRUPT */

#ifdef CONFIG_I2C_HC32_DMA
#define HC32_I2C_DMA_INIT(inst)	\
static void hc32_dma_callback(void *user_data,	\
				uint32_t channel, int status);	\
					\
struct dma_block_config dma_block_config_##inst[2] = \
{	\
	{	\
		.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE, \
		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT, \
		.dest_address = (uint32_t)&(((CM_I2C_TypeDef *)DT_XHSC_HC32_I2C_##inst##_BASE_ADDRESS)->DTR),	\
	},		\
	{	\
		.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE, \
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT, \
		.source_address = (uint32_t)&(((CM_I2C_TypeDef *)DT_XHSC_HC32_I2C_##inst##_BASE_ADDRESS)->DRR),	\
	}		\
};	\
struct dma_hc32_config_user_data i2c_dma_user_data_##inst[2] = \
{	\
	{	\
		.slot = INT_SRC_I2C3_TEI,\
	},	\
	{	\
		.slot = INT_SRC_I2C3_RXI,\
	},	\
}; 	\
struct dma_config dma_config_##inst[2] = \
{	\
	{	\
		.channel_direction = MEMORY_TO_PERIPHERAL, \
		.block_count = 1,	\
		.source_burst_length = 1,	\
		.dest_burst_length = 1,	\
		.source_data_size = 1,	\
		.dest_data_size = 1,	\
		.head_block = &dma_block_config_##inst[0],	\
		.callback_arg = &i2c_dma_user_data_##inst[0],	\
		.dma_callback = hc32_dma_callback,	\
	},	\
	{	\
		.channel_direction = PERIPHERAL_TO_MEMORY, \
		.block_count = 1,	\
		.source_burst_length = 1,	\
		.dest_burst_length = 1,	\
		.source_data_size = 1,	\
		.dest_data_size = 1,	\
		.head_block = &dma_block_config_##inst[1],	\
		.callback_arg = &i2c_dma_user_data_##inst[1],	\
		.dma_callback = hc32_dma_callback,	\
	}	\
};	\

#define HC32_I2C_DMA_CONFIG(inst)	\
	.dma_conf = dma_config_##inst,		\
	.uints = {	\
		(DT_XHSC_HC32_I2C_##inst##_DMAS >> 24) & 0xF,	\
		(DT_XHSC_HC32_I2C_##inst##_DMAS >> 8) & 0xF,	\
	},	\
	.channel = {	\
		(DT_XHSC_HC32_I2C_##inst##_DMAS >> 16) & 0xFF,	\
		(DT_XHSC_HC32_I2C_##inst##_DMAS) & 0xFF,	\
	}
#else
#define HC32_I2C_DMA_INIT(inst)
#define HC32_I2C_DMA_CONFIG(inst)
#endif /* CONFIG_I2C_HC32_DMA */

#ifdef CONFIG_I2C_HC32_BUS_RECOVERY
#define I2C_HC32_SCL_INIT(n) .scl = GPIO_DT_SPEC_INST_GET_OR(n, scl_gpios, {0}),
#define I2C_HC32_SDA_INIT(n) .sda = GPIO_DT_SPEC_INST_GET_OR(n, sda_gpios, {0}),
#else
#define I2C_HC32_SCL_INIT(n)
#define I2C_HC32_SDA_INIT(n)
#endif // #ifdef CONFIG_I2C_HC32_BUS_RECOVERY

#define HC32_I2C_INIT(index)						\
HC32_I2C_IRQ_HANDLER_DECL(index);					\
										\
HC32_I2C_DMA_INIT(index);	\
								\
static struct hc32_modules_clock_sys mudules_clk_##index[] =			\
				 HC32_MODULES_CLOCKS(index, XHSC_HC32_I2C, 1);		\
									\
static struct i2c_hc32_config i2c_hc32_cfg_##index = {		\
	.i2c = (CM_I2C_TypeDef *)DT_XHSC_HC32_I2C_##index##_BASE_ADDRESS,			\
	.mod_clk = mudules_clk_##index,					\
	HC32_I2C_IRQ_HANDLER_FUNCTION(index)				\
	.bitrate = DT_XHSC_HC32_I2C_##index##_CLOCK_FREQUENCY,		\
	I2C_HC32_SCL_INIT(index)					\
	I2C_HC32_SDA_INIT(index)					\
	HC32_I2C_FLAG_TIMEOUT			\
};									\
									\
static struct i2c_hc32_data i2c_hc32_dev_data_##index = {		\
	HC32_I2C_DMA_CONFIG(index)	\
};	\
									\
DEVICE_AND_API_INIT(i2c_hc32_##index, DT_XHSC_HC32_I2C_##index##_LABEL,	\
			i2c_hc32_init,	\
			&i2c_hc32_dev_data_##index, &i2c_hc32_cfg_##index,	\
			POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,	\
			&api_funcs);	\
									\
HC32_I2C_IRQ_HANDLER(index)


#if DT_XHSC_HC32_I2C_0
HC32_I2C_INIT(0);
#endif

#if DT_XHSC_HC32_I2C_1
HC32_I2C_INIT(1);
#endif

#if DT_XHSC_HC32_I2C_2
HC32_I2C_INIT(2);
#endif