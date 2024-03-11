/*
 * Copyright (c) 2016 BayLibre, SAS
 * Copyright (c) 2017 Linaro Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control/hc32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/interrupt_controller/intc_hc32.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include "i2c_hc32.h"

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2c_hc32);

#include "i2c-priv.h"

#define DT_DRV_COMPAT xhsc_hc32_i2c

#define HC32_I2C_TRANSFER_TIMEOUT_MSEC  500

static void hc32_hw_i2c_reset(const struct i2c_hc32_config *cfg);
static int hc32_hw_i2c_start(const struct i2c_hc32_config *cfg);
static int hc32_hw_i2c_restart(const struct i2c_hc32_config *cfg);

#ifdef CONFIG_I2C_HC32_INTERRUPT
static void hc32_i2c_transaction_end(const struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config;
	struct i2c_hc32_data *data = dev->data;

	I2C_IntCmd(cfg->i2c, I2C_INT_TX_EMPTY | I2C_INT_RX_FULL | \
				I2C_INT_TX_CPLT, DISABLE);

	k_sem_give(&data->device_sync_sem);
}

void hc32_i2c_eei_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;

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
	const struct device *dev = (const struct device *)arg;
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;


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
	const struct device *dev = (const struct device *)arg;
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;

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
	const struct device *dev = (const struct device *)arg;
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;
//	struct i2c_smg 

	if (data->msg->flags & I2C_MSG_STOP) {
		if (data->len == 2) {
			I2C_AckConfig(i2c, I2C_NACK);
		}
	}

	if (data->len) {
		*data->dat = I2C_ReadData(i2c);
		data->dat ++;
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
	}
}

static int hc32_i2c_msg_transaction(const struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;

	if (data->msg->flags & I2C_MSG_RESTART) {
		I2C_IntCmd(i2c, I2C_INT_START, ENABLE);
		if (SET == I2C_GetStatus(i2c, I2C_FLAG_BUSY)) {
			hc32_hw_i2c_restart(cfg);
		} else {
			I2C_Cmd(i2c, ENABLE);
			hc32_hw_i2c_reset(cfg);
			hc32_hw_i2c_start(cfg);
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
		hc32_i2c_transaction_end(dev);
		I2C_IntCmd(i2c, I2C_INT_NACK | I2C_INT_STOP, DISABLE);
		I2C_Cmd(i2c, DISABLE);
		k_sem_take(&data->device_sync_sem, K_FOREVER);
		return -EIO;
	}

	return 0;
}
#endif

static int hc32_i2c_config(const struct device *dev)
{
	stc_i2c_init_t stcI2cInit;
	float f32Error = 0.0F;
    uint32_t I2cSrcClk;
    uint32_t I2cClkDiv;
    uint32_t I2cClkDivReg;
	uint32_t baudrate;
	const struct i2c_hc32_config *cfg = dev->config;
	struct i2c_hc32_data *data = dev->data;
	const struct device *const clk = DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE);

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

int i2c_hc32_runtime_configure(const struct device *dev, uint32_t config)
{
	struct i2c_hc32_data *data = dev->data;
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

#define OPERATION(msg) (((struct i2c_msg *) msg)->flags & I2C_MSG_RW_MASK)

static int i2c_hc32_transfer(const struct device *dev, struct i2c_msg *msg,
					uint8_t num_msgs, uint16_t slave)
{
	uint32_t i;
	struct i2c_hc32_data *data = dev->data;
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
		hc32_i2c_msg_transaction(dev);
	}

	k_sem_give(&data->bus_mutex);

	return 0;
}

static const struct i2c_driver_api api_funcs = {
	.configure = i2c_hc32_runtime_configure,
	.transfer = i2c_hc32_transfer,
};

static int i2c_hc32_activate(const struct device *dev)
{
	int ret;
	const struct i2c_hc32_config *cfg = dev->config;
	const struct device *const clk = DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE);

	/* Move pins to active/default state */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2C pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Enable device clock. */
	if (clock_control_on(clk,
			     (clock_control_subsys_t) &cfg->mod_clk[0]) != 0) {
		LOG_ERR("i2c: failure enabling clock");
		return -EIO;
	}

	return 0;
}

static int i2c_hc32_init(const struct device *dev)
{
	const struct device *const clk = DEVICE_DT_GET(HC32_CLOCK_CONTROL_NODE);
	const struct i2c_hc32_config *cfg = dev->config;
	uint32_t bitrate_cfg;
	int ret;
	struct i2c_hc32_data *data = dev->data;

#ifdef CONFIG_I2C_HC32_INTERRUPT
	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);
	cfg->irq_config_func(dev);
#endif

	/*
	 * initialize mutex used when multiple transfers
	 * are taking place to guarantee that each one is
	 * atomic and has exclusive access to the I2C bus.
	 */
	k_sem_init(&data->bus_mutex, 1, 1);

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	i2c_hc32_activate(dev);

	I2C_DeInit(cfg->i2c);

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);

	ret = i2c_hc32_runtime_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("i2c: failure initializing");
		return ret;
	}

	return 0;
}

/* Macros for I2C instance declaration */

#ifdef CONFIG_I2C_HC32_INTERRUPT

#define DT_IRQ_NAME(node_id, prop ,idx)	DT_STRING_TOKEN_BY_IDX(node_id, prop ,idx)

#define DT_IRQ_HANDLE_NAME(node_id, prop ,idx)	\
	_CONCAT(_CONCAT(hc32_i2c_,DT_IRQ_NAME(node_id, prop ,idx)), _isr)

#define IRQ_REGISTER(node_id, prop, idx, inst)	\
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, idx, irq),	\
					DT_INST_IRQ_BY_IDX(inst, idx, priority),\
					DT_IRQ_HANDLE_NAME(node_id, prop, idx),			\
					DEVICE_DT_GET(node_id), 0);		\
	irq_enable(DT_INST_IRQ_BY_IDX(inst, idx, irq));	\
	hc32_intc_irq_signin(DT_PHA_BY_IDX(node_id, intcs, idx, irqn), \
					DT_PHA_BY_IDX(node_id, intcs, idx, int_src));

#define HC32_I2C_IRQ_CONNECT_AND_ENABLE(index)				\
	DT_FOREACH_PROP_ELEM_VARGS(DT_DRV_INST(index), \
		interrupt_names, IRQ_REGISTER, index) 

#define HC32_I2C_IRQ_HANDLER_DECL(index)				\
static void i2c_hc32_irq_config_func_##index(const struct device *dev)
#define HC32_I2C_IRQ_HANDLER_FUNCTION(index)				\
	.irq_config_func = i2c_hc32_irq_config_func_##index,
#define HC32_I2C_IRQ_HANDLER(index)					\
static void i2c_hc32_irq_config_func_##index(const struct device *dev)	\
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
#define HC32_DMA_INIT(inst)	struct dma_config_##inst[]\
{	\
	.block_size = 1,	\
	.source_data_size = 1,	\
	.dest_data_size = 1,	\
}
#else
#define HC32_DMA_INIT(inst)
#endif /* CONFIG_I2C_HC32_DMA */

#define I2C_HC32_SCL_INIT(n) .scl = GPIO_DT_SPEC_INST_GET_OR(n, scl_gpios, {0}),
#define I2C_HC32_SDA_INIT(n) .sda = GPIO_DT_SPEC_INST_GET_OR(n, sda_gpios, {0}),

#define HC32_I2C_INIT(index)						\
HC32_I2C_IRQ_HANDLER_DECL(index);					\
										\
HC32_DMA_INIT(index)	\
									\
PINCTRL_DT_INST_DEFINE(index);						\
									\
static const struct hc32_modules_clock_sys mudules_clk_##index[] =			\
				 HC32_MODULES_CLOCKS(DT_DRV_INST(index));		\
									\
static const struct i2c_hc32_config i2c_hc32_cfg_##index = {		\
	.i2c = (CM_I2C_TypeDef *)DT_INST_REG_ADDR(index),			\
	.mod_clk = mudules_clk_##index,					\
	HC32_I2C_IRQ_HANDLER_FUNCTION(index)				\
	.bitrate = DT_INST_PROP(index, clock_frequency),		\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),			\
	I2C_HC32_SCL_INIT(index)					\
	I2C_HC32_SDA_INIT(index)					\
	HC32_I2C_FLAG_TIMEOUT			\
};									\
									\
static struct i2c_hc32_data i2c_hc32_dev_data_##index;		\
									\
I2C_DEVICE_DT_INST_DEFINE(index, i2c_hc32_init,			\
			 NULL,			\
			 &i2c_hc32_dev_data_##index,			\
			 &i2c_hc32_cfg_##index,			\
			 POST_KERNEL, 40,		\
			 &api_funcs);					\
									\
HC32_I2C_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(HC32_I2C_INIT)
