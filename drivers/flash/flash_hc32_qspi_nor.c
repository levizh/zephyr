/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_qspi_nor

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include "flash_hc32_qspi.h"

LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_FLASH_LOG_LEVEL);

/*
W25Q64_Standard_SPI_Instructions W25Q64 Standard SPI Instructions
*/
#define W25Q64_WR_ENABLE				(0x06U)
#define W25Q64_SR_WR_ENABLE			 	(0x50U)
#define W25Q64_WR_DISABLE				(0x04U)
#define W25Q64_RELEASE_POWER_DOWN_ID	(0xABU)
#define W25Q64_MANUFACTURER_DEVICE_ID	(0x90U)
#define W25Q64_JEDEC_ID				 	(0x9FU)
#define W25Q64_RD_UNIQUE_ID			 	(0x4BU)
#define W25Q64_RD_DATA					(0x03U)
#define W25Q64_FAST_RD					(0x0BU)
#define W25Q64_PAGE_PROGRAM			 	(0x02U)
#define W25Q64_SECTOR_ERASE			 	(0x20U)
#define W25Q64_BLK_ERASE_32KB			(0x52U)
#define W25Q64_BLK_ERASE_64KB			(0xD8U)
#define W25Q64_CHIP_ERASE				(0xC7U)
#define W25Q64_RD_STATUS_REG1			(0x05U)
#define W25Q64_WR_STATUS_REG1			(0x01U)
#define W25Q64_RD_STATUS_REG2			(0x35U)
#define W25Q64_WR_STATUS_REG2			(0x31U)
#define W25Q64_RD_STATUS_REG3			(0x15U)
#define W25Q64_WR_STATUS_REG3			(0x11U)
#define W25Q64_RD_SFDP_REG				(0x5AU)
#define W25Q64_ERASE_SECURITY_REG		(0x44U)
#define W25Q64_PROGRAM_SECURITY_REG	 	(0x42U)
#define W25Q64_RD_SECURITY_REG			(0x48U)
#define W25Q64_GLOBAL_BLK_LOCK			(0x7EU)
#define W25Q64_GLOBAL_BLK_UNLOCK		(0x98U)
#define W25Q64_RD_BLK_LOCK				(0x3DU)
#define W25Q64_PERSON_BLK_LOCK			(0x36U)
#define W25Q64_PERSON_BLK_UNLOCK		(0x39U)
#define W25Q64_ERASE_PROGRAM_SUSPEND	(0x75U)
#define W25Q64_ERASE_PROGRAM_RESUME	 	(0x7AU)
#define W25Q64_POWER_DOWN				(0xB9U)
#define W25Q64_ENABLE_RST				(0x66U)
#define W25Q64_RST_DEVICE				(0x99U)

/*
 W25Q64_Dual_Quad_SPI_Instruction W25Q64 Dual Quad SPI Instruction
 */
#define W25Q64_FAST_RD_DUAL_OUTPUT		(0x3BU)
#define W25Q64_FAST_RD_DUAL_IO			(0xBBU)
#define W25Q64_MFTR_DEVICE_ID_DUAL_IO	(0x92U)
#define W25Q64_QUAD_INPUT_PAGE_PROGRAM	(0x32U)
#define W25Q64_FAST_RD_QUAD_OUTPUT		(0x6BU)
#define W25Q64_MFTR_DEVICE_ID_QUAD_IO	(0x94U)
#define W25Q64_FAST_RD_QUAD_IO			(0xEBU)
#define W25Q64_SET_BURST_WITH_WRAP		(0x77U)

#define W25Q64_FLAG_BUSY				(0x01U)
#define W25Q64_FLAG_SUSPEND				(0x80U)
#define W25Q64_UNIQUE_ID_SIZE			(8U)
#define W25Q64_DUMMY_BYTE_VALUE			(0xFFU)

#define HC32_QSPI_NOR_NODE				DT_INST(0, DT_DRV_COMPAT)
#define HC32_QSPI_NOR_ADDR				DT_REG_ADDR(HC32_QSPI_NOR_NODE)
#define HC32_QSPI_NOR_SIZE				DT_PROP(HC32_QSPI_NOR_NODE, size)
#define HC32_QSPI_NOR_PRG_SIZE			DT_PROP(HC32_QSPI_NOR_NODE, page_size)
#define HC32_QSPI_NOR_SECTOR_BYTE		DT_PROP(HC32_QSPI_NOR_NODE, sector_size)
#define HC32_QSPI_NODE					DT_INST(0, xhsc_hc32_qspi)
#define HC32_QSPI_NOR_WIDTH				DT_PROP(HC32_QSPI_NODE, addr_width)
#define HC32_QSPI_NOR_WIDTH_BYTE		((HC32_QSPI_NOR_WIDTH & 0xF) + 1)

struct flash_hc32_qspi_nor_data {
	struct k_sem mutex;
};

/* Instruction format */
struct qspi_hc32_cmd {
	/* Address where data is written after the command is sent */
	uint8_t *addr;
	/* Length of data written after sending the command */
	uint32_t len;
	/* Command/instruction send to QSPI device */
	uint8_t instr;
};

/* Data format to read out or write in QSPI device */
struct qspi_nor_data {
	/* Address where data is written in or read out to/from */
	uint8_t *addr;
	/* Length of data written in or read out to/from */
	uint32_t len;
	/* true: write data into outside device, false: read data from */
	bool write_flag;
};

static struct flash_hc32_qspi_nor_data flash_hc32_qspi_nor_data;

static const struct flash_parameters flash_hc32_qspi_nor_parameters = {
	.write_block_size = HC32_QSPI_NOR_PRG_SIZE,
	.erase_value = 0xff,
};

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_hc32_qspi_nor_layout[] = {
	{
	.pages_size = HC32_QSPI_NOR_PRG_SIZE,
	.pages_count = HC32_QSPI_NOR_SIZE / HC32_QSPI_NOR_PRG_SIZE
	}
};

void flash_hc32_qspi_nor_pages_layout(const struct device *dev,
				 const struct flash_pages_layout **layout,
				 size_t *layout_size)
{
	ARG_UNUSED(dev);

	*layout = flash_hc32_qspi_nor_layout;
	*layout_size = ARRAY_SIZE(flash_hc32_qspi_nor_layout);
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static void flash_hc32_qspi_nor_word2byte(uint32_t addr_word,
				uint8_t *pu_addr, uint32_t byte_len)
{
	while ((byte_len--) != 0UL) {
		*pu_addr++ = (uint8_t)(addr_word >> (byte_len * 8U)) & 0xFFU;
	}
}

int qspi_hc32_wr_cmd(const struct device *dev, const struct qspi_hc32_cmd *cmd,
		const struct qspi_nor_data *p_inout)
{
	uint32_t count;

	qspi_hc32_command_mode(dev, true);
	(void)qspi_hc32_write_data(dev, cmd->instr);

	if ((NULL != cmd->addr) && (0UL != cmd->len)) {
		for (count = 0UL; count < cmd->len; count++) {
			(void)qspi_hc32_write_data(dev, cmd->addr[count]);
		}
	}

	if ((NULL != p_inout) && (NULL != p_inout->addr) &&
		(0UL != p_inout->len)) {
		if (p_inout->write_flag) {
			for (count = 0UL; count < p_inout->len; count++) {
				(void)qspi_hc32_write_data(dev, p_inout->addr[count]);
			}
		} else {
			for (count = 0UL; count < p_inout->len; count++) {
				(void)qspi_hc32_read_data(dev, p_inout->addr + count);
			}
		}
	}
	qspi_hc32_command_mode(dev, false);

	return 0;
}

static int32_t flash_hc32_qspi_nor_is_process_done(const struct device *dev,
					int32_t wait_ms)
{
	int err = -EINVAL;
	uint8_t status;

	qspi_hc32_command_mode(dev, true);
	(void)qspi_hc32_write_data(dev, W25Q64_RD_STATUS_REG1);
	while ((wait_ms--) > 0) {
		(void)qspi_hc32_read_data(dev, &status);
		if (0U == (status & W25Q64_FLAG_BUSY)) {
			err = 0;
			break;
		}
		k_msleep(1U);
	}
	qspi_hc32_command_mode(dev, false);

	return err;
}


static bool flash_hc32_qspi_nor_valid_range(off_t offset, uint32_t len, bool write)
{
	if ((offset > HC32_QSPI_NOR_SIZE) ||
	 	((offset + len) > HC32_QSPI_NOR_SIZE)) {
		return false;
	}

	return true;
}

int flash_hc32_qspi_nor_erase_sector(const struct device *dev, off_t offset)
{
	int err = 0;
	uint8_t addr[HC32_QSPI_NOR_WIDTH_BYTE];
	struct qspi_hc32_cmd cmd = {
		.instr = W25Q64_WR_ENABLE,
		.addr = NULL,
		.len = 0
	};

	(void)qspi_hc32_wr_cmd(dev, &cmd, NULL);
	/* Convert 32/24/16 address to big indian */
	flash_hc32_qspi_nor_word2byte(offset, addr, HC32_QSPI_NOR_WIDTH_BYTE);
	cmd.instr = W25Q64_SECTOR_ERASE;
	cmd.addr = addr;
	cmd.len = HC32_QSPI_NOR_WIDTH_BYTE;

	(void)qspi_hc32_wr_cmd(dev, &cmd, NULL);
	err = flash_hc32_qspi_nor_is_process_done(dev, 5000U);

	return err;
}

int flash_hc32_qspi_nor_erase_chip(const struct device *dev)
{
	int err = 0;
	struct qspi_hc32_cmd cmd = {
		.instr = W25Q64_WR_ENABLE,
		.addr = NULL,
		.len = 0
	};

	(void)qspi_hc32_wr_cmd(dev, &cmd, NULL);
	cmd.instr = W25Q64_CHIP_ERASE;
	(void)qspi_hc32_wr_cmd(dev, &cmd, NULL);
	err = flash_hc32_qspi_nor_is_process_done(dev, 5000U);

	return err;
}

static int flash_hc32_qspi_nor_read(const struct device *dev, off_t offset,
				void *data, size_t len)
{
	uint8_t *p_nor, *p_read;
	uint16_t idx = 0;
	uint32_t nor_address;
	struct flash_hc32_qspi_nor_data *dev_data = dev->data;

	if (!flash_hc32_qspi_nor_valid_range(offset, len, false)) {
		return -EINVAL;
	}

	p_read = (uint8_t *)data;
	nor_address = offset + HC32_QSPI_NOR_ADDR;
	p_nor = (uint8_t *)nor_address;

	k_sem_take(&dev_data->mutex, K_FOREVER);
	while (idx < len) {
		p_read[idx++] = *p_nor++;
	}
	k_sem_give(&dev_data->mutex);

	return 0;
}

static int flash_hc32_qspi_nor_write(const struct device *dev, off_t offset,
				const void *data, size_t len)
{
	int err = 0;
	uint8_t addr[HC32_QSPI_NOR_WIDTH_BYTE];
	uint32_t program_size;
	struct qspi_hc32_cmd cmd = {0};
	struct qspi_nor_data data_in = {0};
	struct flash_hc32_qspi_nor_data *dev_data = dev->data;

	if (!flash_hc32_qspi_nor_valid_range(offset, len, true)) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->mutex, K_FOREVER);
	while (len != 0UL) {
		if (len >= HC32_QSPI_NOR_PRG_SIZE) {
			program_size = HC32_QSPI_NOR_PRG_SIZE;
		} else {
			program_size = len;
		}

		memset(&cmd, 0, sizeof(struct qspi_hc32_cmd));
		cmd.instr = W25Q64_WR_ENABLE;
		(void)qspi_hc32_wr_cmd(dev, &cmd, NULL);

		flash_hc32_qspi_nor_word2byte(offset, addr, HC32_QSPI_NOR_WIDTH_BYTE);
		cmd.instr = W25Q64_PAGE_PROGRAM;
		cmd.len = HC32_QSPI_NOR_WIDTH_BYTE;
		cmd.addr = addr;
		data_in.write_flag = true;
		data_in.addr = (uint8_t *)data;
		data_in.len = program_size;
		(void)qspi_hc32_wr_cmd(dev, &cmd, &data_in);

		err = flash_hc32_qspi_nor_is_process_done(dev, 500U);
		if (err != LL_OK) {
			break;
		}
		offset += program_size;
		len -= program_size;
	}
	k_sem_give(&dev_data->mutex);

	return err;
}

static int flash_hc32_qspi_nor_erase(const struct device *dev,
			off_t offset, size_t size)
{
	int err = 0;
	int remain_size = size;
	struct flash_hc32_qspi_nor_data *dev_data = dev->data;

	if (!flash_hc32_qspi_nor_valid_range(offset, size, true)) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->mutex, K_FOREVER);
	/* Erase sector or chip */
	if(offset == 0 && size == HC32_QSPI_NOR_SIZE) {
		err = flash_hc32_qspi_nor_erase_chip(dev);
	} else {
		do {
			err = flash_hc32_qspi_nor_erase_sector(dev, offset);
			if (err != 0) {
				LOG_ERR("Failed to erase qspi nor flash");
				break;
			}
			remain_size -= HC32_QSPI_NOR_SECTOR_BYTE;
			offset += HC32_QSPI_NOR_SECTOR_BYTE;
		} while (remain_size > 0);
	}
	k_sem_give(&dev_data->mutex);

	return err;
}

static const struct flash_parameters*
flash_hc32_qspi_nor_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_hc32_qspi_nor_parameters;
}

static const struct flash_driver_api flash_hc32_qspi_nor_driver_api = {
	.read = flash_hc32_qspi_nor_read,
	.write = flash_hc32_qspi_nor_write,
	.erase = flash_hc32_qspi_nor_erase,
	.get_parameters = flash_hc32_qspi_nor_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_hc32_qspi_nor_pages_layout,
#endif
};


static int flash_hc32_qspi_nor_init(const struct device *dev)
{
	int err, i;
	struct flash_hc32_qspi_nor_data *dev_data = dev->data;
	uint8_t dummy_data[4U], nor_uid[W25Q64_UNIQUE_ID_SIZE];
	struct qspi_hc32_cmd cmd = {
		.instr = W25Q64_RD_UNIQUE_ID,
		.addr = dummy_data,
		.len = 4
	};
	struct qspi_nor_data read_out = {
		.write_flag = false,
		.addr = nor_uid,
		.len = W25Q64_UNIQUE_ID_SIZE
	};

	k_sem_init(&dev_data->mutex, 1, 1);

	/* Fill the dummy values */
	for (i = 0UL; i < 4UL; i++) {
		dummy_data[i] = 0xFFU;
	}

	/* Read nor flash UID */
	k_sem_take(&dev_data->mutex, K_FOREVER);
	err = qspi_hc32_wr_cmd(dev, &cmd, &read_out);
	k_sem_give(&dev_data->mutex);
	if(err != 0) {
		LOG_ERR("Failed to get flash ID, initialize error");
		return -1;
	}

	LOG_INF("Flash ID: %02x %02x %02x %02x",
		nor_uid[0], nor_uid[1], nor_uid[2], nor_uid[3]);

	return 0;
}

DEVICE_DT_INST_DEFINE(0,
			flash_hc32_qspi_nor_init, NULL,
			&flash_hc32_qspi_nor_data, NULL, POST_KERNEL,
			CONFIG_FLASH_INIT_PRIORITY, &flash_hc32_qspi_nor_driver_api);
