/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
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

#ifdef CONFIG_I2C_HC32_DMA
#include <zephyr/drivers/dma/dma_hc32.h>
#endif
#ifdef CONFIG_I2C_HC32_BUS_RECOVERY
#include "i2c_bitbang.h"
#endif /* CONFIG_I2C_HC32_BUS_RECOVERY */

#define DT_DRV_COMPAT xhsc_hc32_i2c

#define HC32_I2C_TRANSFER_TIMEOUT_MSEC  500

static void hc32_hw_i2c_reset(const struct i2c_hc32_config *cfg);
static int hc32_hw_i2c_start(const struct i2c_hc32_config *cfg);
static int hc32_hw_i2c_restart(const struct i2c_hc32_config *cfg);
static int hc32_hw_i2c_send_addr(const struct device *dev);
static int hc32_hw_i2c_stop(const struct i2c_hc32_config *cfg);

#if CONFIG_I2C_TARGET
static void hc32_i2c_slaver_eei_isr(const struct device *dev);
static void hc32_i2c_slaver_tei_isr(const struct device *dev);
static void hc32_i2c_slaver_rxi_isr(const struct device *dev);
#endif

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
	const struct i2c_hc32_config *config = io_context;

	gpio_pin_set_dt(&config->scl, state);
}

static void i2c_hc32_bitbang_set_sda(void *io_context, int state)
{
	const struct i2c_hc32_config *config = io_context;

	gpio_pin_set_dt(&config->sda, state);
}

static int i2c_hc32_bitbang_get_sda(void *io_context)
{
	const struct i2c_hc32_config *config = io_context;

	return gpio_pin_get_dt(&config->sda) == 0 ? 0 : 1;
}

static int i2c_hc32_recover_bus(const struct device *dev)
{
	uint8_t i;
	const struct i2c_hc32_config *config = dev->config;
	struct i2c_hc32_data *data = dev->data;
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

#if CONFIG_I2C_TARGET
	if (data->slave_attached && !data->master_active) {
		hc32_i2c_slaver_eei_isr(dev);
		return;
	}
#endif

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
		I2C_IntCmd(i2c, I2C_INT_START | I2C_INT_NACK | I2C_INT_STOP, DISABLE);
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

#if CONFIG_I2C_TARGET
	if (data->slave_attached && !data->master_active) {
		hc32_i2c_slaver_tei_isr(dev);
		return;
	}
#endif

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

#if CONFIG_I2C_TARGET
	if (data->slave_attached && !data->master_active) {
		hc32_i2c_slaver_rxi_isr(dev);
		return;
	}
#endif

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

static int hc32_i2c_msg_transaction(const struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;

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
		I2C_IntCmd(i2c, I2C_INT_START | I2C_INT_NACK | I2C_INT_STOP, DISABLE);
		hc32_hw_i2c_stop(cfg);
		hc32_i2c_transaction_end(dev);
		I2C_Cmd(i2c, DISABLE);
		k_sem_take(&data->device_sync_sem, K_FOREVER);
		return -EIO;
	}

	return 0;
}

#if defined CONFIG_I2C_HC32_INTERRUPT_SHARE
#define HC32_I2C_SHARE_ISR_DEV(node_id)		DEVICE_DT_GET(node_id)
#define HC32_I2C_SHARE_ISR_NUM(node_id)		DT_IRQ_BY_IDX(node_id, 0, irq)
#if defined (HC32F4A0)
#define I2C_HC32_IRQ_SHARE1_NUM				INT141_IRQn
#define I2C_HC32_IRQ_SHARE2_NUM				INT142_IRQn
void hc32_i2c_share_isr(void)
{
	const uint32_t VSSEL141 = CM_INTC->VSSEL141;
	const uint32_t VSSEL142 = CM_INTC->VSSEL142;
	uint32_t u32Tmp1;
	uint32_t u32Tmp2;

	uint8_t i;
	const struct i2c_hc32_config *cfg;
	const struct hc32_modules_clock_sys *mod_clk;;
	uint8_t node_num = DT_NUM_INST_STATUS_OKAY(xhsc_hc32_i2c);
	const struct device *dev_i2c[] = {	\
		DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_NODELABEL(i2c_sys), HC32_I2C_SHARE_ISR_DEV, (,))	\
	};
	uint32_t isr;

	__ASM volatile ("MRS %0, ipsr" : "=r" (isr) );
	isr -= 16;

	for (i = 0; i < node_num; i++) {
		cfg = dev_i2c[i]->config;
		mod_clk = cfg->mod_clk;
		void *arg = (void *)dev_i2c[i];
	
		switch (mod_clk->bits)
		{
		case HC32_FCG1_PERIPH_I2C1:
			if (isr != I2C_HC32_IRQ_SHARE1_NUM) {
				break;
			}
			/* I2C Ch.1 Rx end */
			if (1UL == bCM_I2C1->CR2_b.RFULLIE) {
				if ((1UL == bCM_I2C1->SR_b.RFULLF) && (0UL != (VSSEL141 & BIT_MASK_16))) {
					hc32_i2c_rxi_isr(arg);
				}
			}
			/* I2C Ch.1 Tx buffer empty */
			if (1UL == bCM_I2C1->CR2_b.TEMPTYIE) {
				if ((1UL == bCM_I2C1->SR_b.TEMPTYF) && (0UL != (VSSEL141 & BIT_MASK_17))) {
					hc32_i2c_txi_isr(arg);
				}
			}
			/* I2C Ch.1 Tx end */
			if (1UL == bCM_I2C1->CR2_b.TENDIE) {
				if ((1UL == bCM_I2C1->SR_b.TENDF) && (0UL != (VSSEL141 & BIT_MASK_18))) {
					hc32_i2c_tei_isr(arg);
				}
			}
			/* I2C Ch.1 Error */
			u32Tmp1 = CM_I2C1->CR2 & (I2C_CR2_TMOUTIE | I2C_CR2_NACKIE |	\
										I2C_CR2_ARLOIE | I2C_CR2_STOPIE | \
										I2C_CR2_SLADDR1IE | I2C_CR2_SLADDR0IE | \
										I2C_CR2_STARTIE);
			u32Tmp2 = CM_I2C1->SR & (I2C_SR_TMOUTF | I2C_SR_NACKF |	\
										I2C_SR_ARLOF | I2C_SR_STOPF | I2C_SR_SLADDR1F |	\
										I2C_SR_SLADDR0F | I2C_SR_STARTF);
			if ((0UL != (u32Tmp1 & u32Tmp2)) && (0UL != (VSSEL141 & BIT_MASK_19))) {
				hc32_i2c_eei_isr(arg);
			}
			break;

		case HC32_FCG1_PERIPH_I2C2:
			if (isr != I2C_HC32_IRQ_SHARE1_NUM) {
				break;
			}
			/* I2C Ch.2 Rx end */
			if (1UL == bCM_I2C2->CR2_b.RFULLIE) {
				if ((1UL == bCM_I2C2->SR_b.RFULLF) && (0UL != (VSSEL141 & BIT_MASK_20))) {
					hc32_i2c_rxi_isr(arg);
				}
			}
			/* I2C Ch.2 Tx buffer empty */
			if (1UL == bCM_I2C2->CR2_b.TEMPTYIE) {
				if ((1UL == bCM_I2C2->SR_b.TEMPTYF) && (0UL != (VSSEL141 & BIT_MASK_21))) {
					hc32_i2c_txi_isr(arg);
				}
			}
			/* I2C Ch.2 Tx end */
			if (1UL == bCM_I2C2->CR2_b.TENDIE) {
				if ((1UL == bCM_I2C2->SR_b.TENDF) && (0UL != (VSSEL141 & BIT_MASK_22))) {
					hc32_i2c_tei_isr(arg);
				}
			}
			/* I2C Ch.2 Error */
			u32Tmp1 = CM_I2C2->CR2 & (I2C_CR2_TMOUTIE | I2C_CR2_NACKIE |	\
										I2C_CR2_ARLOIE | I2C_CR2_STOPIE | \
										I2C_CR2_SLADDR1IE | I2C_CR2_SLADDR0IE | \
										I2C_CR2_STARTIE);
			u32Tmp2 = CM_I2C2->SR & (I2C_SR_TMOUTF | I2C_SR_NACKF |	\
										I2C_SR_ARLOF | I2C_SR_STOPF | I2C_SR_SLADDR1F |	\
										I2C_SR_SLADDR0F | I2C_SR_STARTF);
			if ((0UL != (u32Tmp1 & u32Tmp2)) && (0UL != (VSSEL141 & BIT_MASK_23))) {
				hc32_i2c_eei_isr(arg);
			}
			break;
		
		case HC32_FCG1_PERIPH_I2C3:
			if (isr != I2C_HC32_IRQ_SHARE1_NUM) {
				break;
			}
			/* I2C Ch.3 Rx end */
			if (1UL == bCM_I2C3->CR2_b.RFULLIE) {
				if ((1UL == bCM_I2C3->SR_b.RFULLF) && (0UL != (VSSEL141 & BIT_MASK_24))) {
					hc32_i2c_rxi_isr(arg);
				}
			}
			/* I2C Ch.3 Tx buffer empty */
			if (1UL == bCM_I2C3->CR2_b.TEMPTYIE) {
				if ((1UL == bCM_I2C3->SR_b.TEMPTYF) && (0UL != (VSSEL141 & BIT_MASK_25))) {
					hc32_i2c_txi_isr(arg);
				}
			}
			/* I2C Ch.3 Tx end */
			if (1UL == bCM_I2C3->CR2_b.TENDIE) {
				if ((1UL == bCM_I2C3->SR_b.TENDF) && (0UL != (VSSEL141 & BIT_MASK_26))) {
					hc32_i2c_tei_isr(arg);
				}
			}
			/* I2C Ch.3 Error */
			u32Tmp1 = CM_I2C3->CR2 & (I2C_CR2_TMOUTIE | I2C_CR2_NACKIE |	\
										I2C_CR2_ARLOIE | I2C_CR2_STOPIE | \
										I2C_CR2_SLADDR1IE | I2C_CR2_SLADDR0IE | \
										I2C_CR2_STARTIE);
			u32Tmp2 = CM_I2C3->SR & (I2C_SR_TMOUTF | I2C_SR_NACKF |	\
										I2C_SR_ARLOF | I2C_SR_STOPF | I2C_SR_SLADDR1F |	\
										I2C_SR_SLADDR0F | I2C_SR_STARTF);
			if ((0UL != (u32Tmp1 & u32Tmp2)) && (0UL != (VSSEL141 & BIT_MASK_27))) {
				hc32_i2c_eei_isr(arg);
			}
			break;

		case HC32_FCG1_PERIPH_I2C4:
			if (isr != I2C_HC32_IRQ_SHARE2_NUM) {
				break;
			}
			/* I2C Ch.4 Rx end */
			if (1UL == bCM_I2C4->CR2_b.RFULLIE) {
				if ((1UL == bCM_I2C4->SR_b.RFULLF) && (0UL != (VSSEL142 & BIT_MASK_00))) {
					hc32_i2c_rxi_isr(arg);
				}
			}
			/* I2C Ch.4 Tx buffer empty */
			if (1UL == bCM_I2C4->CR2_b.TEMPTYIE) {
				if ((1UL == bCM_I2C4->SR_b.TEMPTYF) && (0UL != (VSSEL142 & BIT_MASK_01))) {
					hc32_i2c_txi_isr(arg);
				}
			}
			/* I2C Ch.4 Tx end */
			if (1UL == bCM_I2C4->CR2_b.TENDIE) {
				if ((1UL == bCM_I2C4->SR_b.TENDF) && (0UL != (VSSEL142 & BIT_MASK_02))) {
					hc32_i2c_tei_isr(arg);
				}
			}
			/* I2C Ch.4 Error */
			u32Tmp1 = CM_I2C4->CR2 & (I2C_CR2_TMOUTIE | I2C_CR2_NACKIE |	\
										I2C_CR2_ARLOIE | I2C_CR2_STOPIE | \
										I2C_CR2_SLADDR1IE | I2C_CR2_SLADDR0IE | \
										I2C_CR2_STARTIE);
			u32Tmp2 = CM_I2C4->SR & (I2C_SR_TMOUTF | I2C_SR_NACKF |	\
										I2C_SR_ARLOF | I2C_SR_STOPF | I2C_SR_SLADDR1F |	\
										I2C_SR_SLADDR0F | I2C_SR_STARTF);
			if ((0UL != (u32Tmp1 & u32Tmp2)) && (0UL != (VSSEL142 & BIT_MASK_03))) {
				hc32_i2c_eei_isr(arg);
			}
			break;

		case HC32_FCG1_PERIPH_I2C5:
			if (isr != I2C_HC32_IRQ_SHARE2_NUM) {
				break;
			}
			/* I2C Ch.5 Rx end */
			if (1UL == bCM_I2C5->CR2_b.RFULLIE) {
				if ((1UL == bCM_I2C5->SR_b.RFULLF) && (0UL != (VSSEL142 & BIT_MASK_04))) {
					hc32_i2c_rxi_isr(arg);
				}
			}
			/* I2C Ch.5 Tx buffer empty */
			if (1UL == bCM_I2C5->CR2_b.TEMPTYIE) {
				if ((1UL == bCM_I2C5->SR_b.TEMPTYF) && (0UL != (VSSEL142 & BIT_MASK_05))) {
					hc32_i2c_txi_isr(arg);
				}
			}
			/* I2C Ch.5 Tx end */
			if (1UL == bCM_I2C5->CR2_b.TENDIE) {
				if ((1UL == bCM_I2C5->SR_b.TENDF) && (0UL != (VSSEL142 & BIT_MASK_06))) {
					hc32_i2c_tei_isr(arg);
				}
			}
			/* I2C Ch.5 Error */
			u32Tmp1 = CM_I2C5->CR2 & (I2C_CR2_TMOUTIE | I2C_CR2_NACKIE |	\
										I2C_CR2_ARLOIE | I2C_CR2_STOPIE | \
										I2C_CR2_SLADDR1IE | I2C_CR2_SLADDR0IE | \
										I2C_CR2_STARTIE);
			u32Tmp2 = CM_I2C5->SR & (I2C_SR_TMOUTF | I2C_SR_NACKF |	\
										I2C_SR_ARLOF | I2C_SR_STOPF | I2C_SR_SLADDR1F |	\
										I2C_SR_SLADDR0F | I2C_SR_STARTF);
			if ((0UL != (u32Tmp1 & u32Tmp2)) && (0UL != (VSSEL142 & BIT_MASK_07))) {
				hc32_i2c_eei_isr(arg);
			}
			break;
		
		case HC32_FCG1_PERIPH_I2C6:
			if (isr != I2C_HC32_IRQ_SHARE2_NUM) {
				break;
			}
			/* I2C Ch.6 Rx end */
			if (1UL == bCM_I2C6->CR2_b.RFULLIE) {
				if ((1UL == bCM_I2C6->SR_b.RFULLF) && (0UL != (VSSEL142 & BIT_MASK_08))) {
					hc32_i2c_rxi_isr(arg);
				}
			}
			/* I2C Ch.6 Tx buffer empty */
			if (1UL == bCM_I2C6->CR2_b.TEMPTYIE) {
				if ((1UL == bCM_I2C6->SR_b.TEMPTYF) && (0UL != (VSSEL142 & BIT_MASK_09))) {
					hc32_i2c_txi_isr(arg);
				}
			}
			/* I2C Ch.6 Tx end */
			if (1UL == bCM_I2C6->CR2_b.TENDIE) {
				if ((1UL == bCM_I2C6->SR_b.TENDF) && (0UL != (VSSEL142 & BIT_MASK_10))) {
					hc32_i2c_tei_isr(arg);
				}
			}
			/* I2C Ch.6 Error */
			u32Tmp1 = CM_I2C6->CR2 & (I2C_CR2_TMOUTIE | I2C_CR2_NACKIE |	\
										I2C_CR2_ARLOIE | I2C_CR2_STOPIE | \
										I2C_CR2_SLADDR1IE | I2C_CR2_SLADDR0IE | \
										I2C_CR2_STARTIE);
			u32Tmp2 = CM_I2C6->SR & (I2C_SR_TMOUTF | I2C_SR_NACKF |	\
										I2C_SR_ARLOF | I2C_SR_STOPF | I2C_SR_SLADDR1F |	\
										I2C_SR_SLADDR0F | I2C_SR_STARTF);
			if ((0UL != (u32Tmp1 & u32Tmp2)) && (0UL != (VSSEL142 & BIT_MASK_11))) {
				hc32_i2c_eei_isr(arg);
			}
			break;

		default:
			break;
		}
	}
}
static int hc32_i2c_sys_init(const struct device *dev)
{
	uint8_t i, j, index = 0;
	uint8_t node_num = DT_NUM_INST_STATUS_OKAY(xhsc_hc32_i2c);
	const uint16_t share_iqrs_num[] = {	\
		DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_NODELABEL(i2c_sys), HC32_I2C_SHARE_ISR_NUM, (,))	\
	};
	uint16_t storage_irqs_buf[sizeof(share_iqrs_num)/sizeof(uint16_t)] = {0};

	for (i = 0; i < node_num; i++)
	{
		for (j = 0; j < node_num; j++)
		{
			if (share_iqrs_num[i] == storage_irqs_buf[j]) {
				break;
			}
		}
		if (j == node_num) {
			storage_irqs_buf[index] = share_iqrs_num[i];

			index ++;
		}
	}
	for (i = 0; i < index; i++)
	{
		/* Share irq must be ensure what all priority in devicetree are same */
		if (storage_irqs_buf[i] == I2C_HC32_IRQ_SHARE1_NUM) {
			IRQ_CONNECT(I2C_HC32_IRQ_SHARE1_NUM,	\
					DT_IRQ_BY_IDX(DT_DRV_INST(0), 0, priority),	\
					hc32_i2c_share_isr,			\
					NULL, 0);		\
			irq_enable(I2C_HC32_IRQ_SHARE1_NUM);
		} else if (storage_irqs_buf[i] == I2C_HC32_IRQ_SHARE2_NUM){
			IRQ_CONNECT(I2C_HC32_IRQ_SHARE2_NUM,	\
					DT_IRQ_BY_IDX(DT_DRV_INST(0), 0, priority),	\
					hc32_i2c_share_isr,			\
					NULL, 0);		\
			irq_enable(I2C_HC32_IRQ_SHARE2_NUM);
		}
	}
	
	return 0;
}
#endif
#endif
#endif /* CONFIG_I2C_HC32_INTERRUPT */

#ifdef CONFIG_I2C_HC32_DMA
static void hc32_dma_callback(const struct device *dev, void *user_data,
				uint32_t channel, int status)
{
	struct dma_hc32_config_user_data *hc32_user_data = \
		(struct dma_hc32_config_user_data *)user_data;
	const struct device *i2c_dev = \
		(const struct device *)hc32_user_data->user_data;
	struct i2c_hc32_data *data = i2c_dev->data;

	k_sem_give(&data->device_sync_sem);
	dma_stop(dev, channel);
}

static int hc32_i2c_dma_write(const struct device *dev)
{
	uint32_t timeout = 0;
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;
	struct dma_config *i2c_dma_config = cfg->dma_conf;
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
		dma_config(cfg->dma_dev[0], cfg->channel[0], i2c_dma_config);
		dma_start(cfg->dma_dev[0], cfg->channel[0]);
	}

	I2C_WriteData(i2c, *data->dat);
	if (data->len > 1) {
		if (k_sem_take(&data->device_sync_sem,
				K_MSEC(HC32_I2C_TRANSFER_TIMEOUT_MSEC)) != 0) {
			dma_stop(cfg->dma_dev[0], cfg->channel[0]);
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

static int hc32_i2c_dma_read(const struct device *dev)
{
	uint32_t timeout = 0;
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;
	struct dma_config *i2c_dma_config = cfg->dma_conf;
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
		dma_config(cfg->dma_dev[1], cfg->channel[1], &i2c_dma_config[1]);
		dma_start(cfg->dma_dev[1], cfg->channel[1]);
	}

	if (data->len > 2) {
		if (k_sem_take(&data->device_sync_sem,
				K_MSEC(HC32_I2C_TRANSFER_TIMEOUT_MSEC)) != 0) {
			dma_stop(cfg->dma_dev[1], cfg->channel[1]);
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

static int hc32_i2c_msg_transaction(const struct device *dev)
{
	uint32_t ret = 0;
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;
	
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

static int hc32_hw_i2c_stop(const struct i2c_hc32_config *cfg)
{
	if (LL_OK != I2C_Stop(cfg->i2c, cfg->time_out))
	{
		return -EFAULT;
	}
	return 0;
}

__attribute__ ((unused)) \
static int hc32_hw_i2c_send_addr(const struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config;
	struct i2c_hc32_data *data = dev->data;

	if (LL_OK != I2C_TransAddr(cfg->i2c, data->slaver_addr, \
					data->dir, cfg->time_out))
    {
        return -EFAULT;
    }

	return 0;
}

#define OPERATION(msg) (((struct i2c_msg *) msg)->flags & I2C_MSG_RW_MASK)

static int i2c_hc32_transfer(const struct device *dev, struct i2c_msg *msg,
					uint8_t num_msgs, uint16_t slave)
{
	int ret = 0;
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

#if defined(CONFIG_I2C_TARGET)
		data->master_active = true;
#endif

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

#if defined(CONFIG_I2C_TARGET)
	const struct i2c_hc32_config *cfg = dev->config;
	data->master_active = false;
	if (data->slave_attached) {
		I2C_Cmd(cfg->i2c, ENABLE);
		hc32_hw_i2c_reset(cfg);
	}
#endif

	k_sem_give(&data->bus_mutex);

	return ret;
}

#if defined(CONFIG_I2C_TARGET)
#ifdef CONFIG_I2C_HC32_INTERRUPT
static void hc32_i2c_slaver_eei_isr(const struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;
	const struct i2c_target_callbacks *slave_cb =
		data->slave_cfg->callbacks;
	uint8_t val;

	if (SET == I2C_GetStatus(i2c, I2C_FLAG_MATCH_ADDR0)) {
		I2C_ClearStatus(i2c, I2C_CLR_SLADDR0FCLR | I2C_CLR_NACKFCLR | I2C_FLAG_STOP);

		if (SET == I2C_GetStatus(i2c, I2C_FLAG_TRA)) {
			I2C_IntCmd(i2c, I2C_INT_TX_CPLT, ENABLE);
			slave_cb->read_requested(data->slave_cfg, &val);
			I2C_WriteData(i2c, val);
			I2C_IntCmd(i2c, I2C_INT_STOP | I2C_INT_NACK, ENABLE);
		} else {
			slave_cb->write_requested(data->slave_cfg);
			I2C_IntCmd(i2c, (I2C_INT_RX_FULL | I2C_INT_STOP | I2C_INT_NACK), ENABLE);
		}
	}

	if (SET == I2C_GetStatus(i2c, I2C_FLAG_NACKF)) {
		I2C_ClearStatus(i2c, I2C_CLR_NACKFCLR);

		if (SET == I2C_GetStatus(i2c, I2C_FLAG_TRA)) {
			I2C_IntCmd(i2c, I2C_INT_TX_CPLT, DISABLE);
			I2C_ClearStatus(i2c, I2C_CLR_TENDFCLR);
			(void)I2C_ReadData(i2c);
		} else {
			I2C_IntCmd(i2c, I2C_INT_RX_FULL, DISABLE);
		}
	}

	if (SET == I2C_GetStatus(i2c, I2C_FLAG_STOP)) {
		I2C_ClearStatus(i2c, I2C_CLR_STOPFCLR);
		I2C_IntCmd(i2c, I2C_INT_TX_CPLT | I2C_INT_RX_FULL | I2C_INT_STOP | I2C_INT_NACK, DISABLE);
		slave_cb->stop(data->slave_cfg);
	}
}

static void hc32_i2c_slaver_tei_isr(const struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;
	const struct i2c_target_callbacks *slave_cb = data->slave_cfg->callbacks;
	uint8_t val;

	if (I2C_INT_TX_CPLT == READ_REG32_BIT(i2c->CR2, I2C_INT_TX_CPLT) && \
		RESET == I2C_GetStatus(i2c, I2C_FLAG_ACKR)) {
		slave_cb->read_processed(data->slave_cfg, &val);
		I2C_WriteData(i2c, val);
	}
}

static void hc32_i2c_slaver_rxi_isr(const struct device *dev)
{
	const struct i2c_hc32_config *cfg = dev->config;
	CM_I2C_TypeDef *i2c = cfg->i2c;
	struct i2c_hc32_data *data = dev->data;
	const struct i2c_target_callbacks *slave_cb = data->slave_cfg->callbacks;

	slave_cb->write_received(data->slave_cfg, I2C_ReadData(i2c));
}
#endif

/* Attach and start I2C as slave */
int i2c_hc32_slave_register(const struct device *dev,
				struct i2c_target_config *config)
{
	const struct i2c_hc32_config *cfg = dev->config;
	struct i2c_hc32_data *data = dev->data;
	uint32_t bitrate_cfg;
	int ret;

	if (!config) {
		return -EINVAL;
	}

	if (data->slave_attached) {
		return -EBUSY;
	}

	if (data->master_active) {
		return -EBUSY;
	}

#ifndef CONFIG_I2C_HC32_INTERRUPT
	return -ENOTSUP
#endif // !CONFIG_I2C_HC32_INTERRUPT
	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);
	ret = i2c_hc32_runtime_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("i2c: failure initializing");
		return ret;
	}

	data->slave_cfg = config;

	data->slave_attached = true;

	I2C_SlaveAddrConfig(cfg->i2c, I2C_ADDR0, I2C_ADDR_7BIT, config->address);

	I2C_Cmd(cfg->i2c, ENABLE);

	hc32_hw_i2c_reset(cfg);

	I2C_IntCmd(cfg->i2c, (I2C_INT_MATCH_ADDR0 | I2C_INT_RX_FULL), ENABLE);
	return 0;
}

int i2c_hc32_slave_unregister(const struct device *dev,
				struct i2c_target_config *config)
{
	const struct i2c_hc32_config *cfg = dev->config;
	struct i2c_hc32_data *data = dev->data;

	if (!data->slave_attached) {
		return -EINVAL;
	}

	if (data->master_active) {
		return -EBUSY;
	}

	data->slave_cfg = NULL;

	data->slave_attached = false;

	I2C_Cmd(cfg->i2c, DISABLE);

	hc32_hw_i2c_reset(cfg);

	LOG_DBG("i2c: slave unregistered");

	return 0;
}
#endif // !CONFIG_I2C_TARGET

static const struct i2c_driver_api api_funcs = {
	.configure = i2c_hc32_runtime_configure,
	.transfer = i2c_hc32_transfer,
#if CONFIG_I2C_HC32_BUS_RECOVERY
	.recover_bus = i2c_hc32_recover_bus,
#endif /* CONFIG_I2C_HC32_BUS_RECOVERY */
#if CONFIG_I2C_TARGET
	.target_register = i2c_hc32_slave_register,
	.target_unregister = i2c_hc32_slave_unregister,
#endif
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
	cfg->irq_config_func(dev);
#endif
#if defined (CONFIG_I2C_HC32_INTERRUPT) || (CONFIG_I2C_HC32_DMA)
	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);
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
#ifdef CONFIG_I2C_HC32_INTERRUPT_SHARE
#define HC32_I2C_IRQ_SHARE_ENABLE(src)	\
	INTC_ShareIrqCmd(src, ENABLE);	

#define IRQ_SHARE_REGISTER(node_id, prop, idx, inst) 	\
	HC32_I2C_IRQ_SHARE_ENABLE(DT_INST_IRQ_BY_IDX(inst, idx, int_src));

#define HC32_I2C_IRQ_SHARE_CONNECT_AND_ENABLE(index)	\
	DT_FOREACH_PROP_ELEM_VARGS(DT_DRV_INST(index), \
		interrupt_names, IRQ_SHARE_REGISTER, index);	
/*	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(index, 0, irq),	\
					DT_INST_IRQ_BY_IDX(index, 0, priority),	\
					hc32_i2c_share_isr,			\
					DEVICE_DT_GET(DT_DRV_INST(index)), 0);		\
	irq_enable(DT_INST_IRQ_BY_IDX(index, 0, irq));*/
#else
#define DT_IRQ_NAME(node_id, prop ,idx)	DT_STRING_TOKEN_BY_IDX(node_id, prop ,idx)

#define DT_IRQ_HANDLE_NAME(node_id, prop ,idx)	\
	_CONCAT(_CONCAT(hc32_i2c_,DT_IRQ_NAME(node_id, prop ,idx)), _isr)

#define IRQ_REGISTER(node_id, prop, idx, inst)	\
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, idx, irq),	\
					DT_INST_IRQ_BY_IDX(inst, idx, priority),\
					DT_IRQ_HANDLE_NAME(node_id, prop, idx),			\
					DEVICE_DT_GET(node_id), 0);		\
	irq_enable(DT_INST_IRQ_BY_IDX(inst, idx, irq));	\
	hc32_intc_irq_signin(DT_INST_IRQ_BY_IDX(inst, idx, irq), \
					DT_INST_IRQ_BY_IDX(inst, idx, int_src));

#define HC32_I2C_IRQ_CONNECT_AND_ENABLE(index)				\
	DT_FOREACH_PROP_ELEM_VARGS(DT_DRV_INST(index), \
		interrupt_names, IRQ_REGISTER, index) 
#endif

#define HC32_I2C_IRQ_HANDLER_DECL(index)				\
static void i2c_hc32_irq_config_func_##index(const struct device *dev)
#define HC32_I2C_IRQ_HANDLER_FUNCTION(index)				\
	.irq_config_func = i2c_hc32_irq_config_func_##index,

#ifdef CONFIG_I2C_HC32_INTERRUPT_SHARE
#define HC32_I2C_IRQ_HANDLER(index)					\
static void i2c_hc32_irq_config_func_##index(const struct device *dev)	\
{									\
	HC32_I2C_IRQ_SHARE_CONNECT_AND_ENABLE(index);			\
}
#else
#define HC32_I2C_IRQ_HANDLER(index)					\
static void i2c_hc32_irq_config_func_##index(const struct device *dev)	\
{									\
	HC32_I2C_IRQ_CONNECT_AND_ENABLE(index);			\
}
#endif
#define HC32_I2C_FLAG_TIMEOUT	.time_out = 0,
#else

#define HC32_I2C_IRQ_HANDLER_DECL(index)
#define HC32_I2C_IRQ_HANDLER_FUNCTION(index)
#define HC32_I2C_IRQ_HANDLER(index)
#define HC32_I2C_FLAG_TIMEOUT	.time_out = 3000,
#endif /* CONFIG_I2C_HC32_INTERRUPT */

#ifdef CONFIG_I2C_HC32_DMA
#define HC32_I2C_DMA_INIT(inst)	\
static void hc32_dma_callback(const struct device *dev, void *user_data,	\
				uint32_t channel, int status);	\
const uint32_t dma_config_vaule_##inst[2] = {HC32_DMA_CHANNEL_CONFIG(inst, tx_dma), \
								HC32_DMA_CHANNEL_CONFIG(inst, rx_dma)}; \
struct dma_block_config dma_block_config_##inst[2] = \
{	\
	{	\
		.dest_addr_adj = HC32_DMA_CONFIG_DEST_ADDR_INC(dma_config_vaule_##inst[0]), \
		.source_addr_adj = HC32_DMA_CONFIG_SOURCE_ADDR_INC(dma_config_vaule_##inst[0]), \
		.dest_address = (uint32_t)&(((CM_I2C_TypeDef *)DT_INST_REG_ADDR(inst))->DTR),	\
	},		\
	{	\
		.source_addr_adj = HC32_DMA_CONFIG_SOURCE_ADDR_INC(dma_config_vaule_##inst[1]), \
		.dest_addr_adj = HC32_DMA_CONFIG_DEST_ADDR_INC(dma_config_vaule_##inst[1]), \
		.source_address = (uint32_t)&(((CM_I2C_TypeDef *)DT_INST_REG_ADDR(inst))->DRR),	\
	}		\
};	\
struct dma_hc32_config_user_data i2c_dma_user_data_##inst[2] = \
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
struct dma_config dma_config_##inst[2] = \
{	\
	{	\
		.channel_direction = HC32_DMA_CONFIG_DIRECTION(dma_config_vaule_##inst[0]), \
		.block_count = 1,	\
		.source_burst_length = 1,	\
		.dest_burst_length = 1,	\
		.source_data_size = HC32_DMA_CONFIG_DATA_SIZE(dma_config_vaule_##inst[0]),	\
		.dest_data_size = HC32_DMA_CONFIG_DATA_SIZE(dma_config_vaule_##inst[0]),	\
		.head_block = &dma_block_config_##inst[0],	\
		.user_data = &i2c_dma_user_data_##inst[0],	\
		.dma_callback = hc32_dma_callback,	\
	},	\
	{	\
		.channel_direction = HC32_DMA_CONFIG_DIRECTION(dma_config_vaule_##inst[1]), \
		.block_count = 1,	\
		.source_burst_length = 1,	\
		.dest_burst_length = 1,	\
		.source_data_size = HC32_DMA_CONFIG_DATA_SIZE(dma_config_vaule_##inst[1]),	\
		.dest_data_size = HC32_DMA_CONFIG_DATA_SIZE(dma_config_vaule_##inst[1]),	\
		.head_block = &dma_block_config_##inst[1],	\
		.user_data = &i2c_dma_user_data_##inst[1],	\
		.dma_callback = hc32_dma_callback,	\
	}	\
};	
#define HC32_I2C_DMA_CONFIG(inst)	\
	.dma_dev = {	\
		DEVICE_DT_GET(HC32_DMA_CTLR(inst, tx_dma)),	\
		DEVICE_DT_GET(HC32_DMA_CTLR(inst, rx_dma)),	\
	},	\
	.dma_conf = dma_config_##inst,		\
	.channel = {	\
		HC32_DMA_CHANNEL(inst, tx_dma),	\
		HC32_DMA_CHANNEL(inst, rx_dma),	\
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
	HC32_I2C_DMA_CONFIG(index)		\
};									\
									\
static struct i2c_hc32_data i2c_hc32_dev_data_##index;		\
									\
I2C_DEVICE_DT_INST_DEFINE(index, i2c_hc32_init,			\
			 NULL,			\
			 &i2c_hc32_dev_data_##index,			\
			 &i2c_hc32_cfg_##index,			\
			 POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,		\
			 &api_funcs);					\
									\
HC32_I2C_IRQ_HANDLER(index)	

DT_INST_FOREACH_STATUS_OKAY(HC32_I2C_INIT)

#ifdef CONFIG_I2C_HC32_INTERRUPT_SHARE
DEVICE_DT_DEFINE(DT_NODELABEL(i2c_sys),
			&hc32_i2c_sys_init,
			NULL,
			NULL, NULL,
			PRE_KERNEL_1,
			1,
			NULL);
#endif