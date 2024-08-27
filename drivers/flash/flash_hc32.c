/*
 * Copyright (c) 2022 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT xhsc_hc32_flash_controller

#include "flash_hc32.h"

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(flash_hc32, CONFIG_FLASH_LOG_LEVEL);

struct flash_hc32_data {
	struct k_sem mutex;
};

static struct flash_hc32_data flash_data;

static const struct flash_parameters flash_hc32_parameters = {
	.write_block_size = HC32_SOC_NV_FLASH_PRG_SIZE,
	.erase_value = 0xff,
};

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout hc32_efm_layout[] = {
	{
	.pages_size = HC32_SOC_NV_FLASH_PRG_SIZE,
	.pages_count = HC32_SOC_NV_FLASH_SIZE / HC32_SOC_NV_FLASH_PRG_SIZE
	}
};
#endif

bool flash_hc32_valid_range(off_t offset, uint32_t len, bool write)
{
	if ((offset > HC32_SOC_NV_FLASH_SIZE) ||
	 	((offset + len) > HC32_SOC_NV_FLASH_SIZE)) {
		return false;
	}

	if (write) {
		/* Check offset and len is flash_prg_t aligned */
		if ((offset % sizeof(flash_prg_t)) || (len % sizeof(flash_prg_t))) {
			return false;
		}
	}

	return true;
}

int flash_hc32_write_range(off_t offset, const void *data, size_t len)
{
	int ret = -1;

	/* Program reserved area is invalid operation */
	if ((offset >= HC32_SOC_NV_FLASH_RSVD_ADDR) ||
		((offset + len) >= HC32_SOC_NV_FLASH_RSVD_ADDR)) {
		return ret;
	}

	EFM_REG_Unlock();
	EFM_FWMC_Cmd(ENABLE);
	ret = EFM_SequenceProgram(offset, data, len);
	EFM_FWMC_Cmd(DISABLE);
	EFM_REG_Lock();
	return ret;
}

int flash_hc32_erase_block(off_t offset, size_t size)
{
	size_t erased_size= 0;
	int ret = 0;

	EFM_REG_Unlock();
	EFM_FWMC_Cmd(ENABLE);

	while (erased_size < size) {
		ret = EFM_SectorErase(offset);
		if (ret < 0) {
			EFM_FWMC_Cmd(DISABLE);
			EFM_REG_Lock();
			return ret;
		}
		erased_size += HC32_SOC_NV_FLASH_PRG_SIZE;
		offset += HC32_SOC_NV_FLASH_PRG_SIZE;
	}

	EFM_FWMC_Cmd(DISABLE);
	EFM_REG_Lock();

	return ret;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
void flash_hc32_pages_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	ARG_UNUSED(dev);

	*layout = hc32_efm_layout;
	*layout_size = ARRAY_SIZE(hc32_efm_layout);
}
#endif

static int flash_hc32_read(const struct device *dev, off_t offset,
			   void *data, size_t len)
{
	if ((offset > HC32_SOC_NV_FLASH_SIZE) ||
	    ((offset + len) > HC32_SOC_NV_FLASH_SIZE)) {
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

	memcpy(data, (uint8_t *)HC32_SOC_NV_FLASH_ADDR + offset, len);

	return 0;
}

static int flash_hc32_write(const struct device *dev, off_t offset,
			    const void *data, size_t len)
{
	struct flash_hc32_data *dev_data = dev->data;
	int ret = 0;

	if (!flash_hc32_valid_range(offset, len, true)) {
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

	k_sem_take(&dev_data->mutex, K_FOREVER);

	ret = flash_hc32_write_range(offset, data, len);

	k_sem_give(&dev_data->mutex);

	return ret;
}

static int flash_hc32_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_hc32_data *data = dev->data;
	int ret = 0;

	if (size == 0U) {
		return 0;
	}

	if (!flash_hc32_valid_range(offset, size, false)) {
		return -EINVAL;
	}

	k_sem_take(&data->mutex, K_FOREVER);

	ret = flash_hc32_erase_block(offset, size);

	k_sem_give(&data->mutex);

	return ret;
}

static const struct flash_parameters*
flash_hc32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_hc32_parameters;
}

static const struct flash_driver_api flash_hc32_driver_api = {
	.read = flash_hc32_read,
	.write = flash_hc32_write,
	.erase = flash_hc32_erase,
	.get_parameters = flash_hc32_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_hc32_pages_layout,
#endif
};

static int flash_hc32_init(const struct device *dev)
{
	struct flash_hc32_data *data = dev->data;

	k_sem_init(&data->mutex, 1, 1);

	return 0;
}

DEVICE_DT_INST_DEFINE(0,
			flash_hc32_init, NULL,
			&flash_data, NULL, POST_KERNEL,
			CONFIG_FLASH_INIT_PRIORITY, &flash_hc32_driver_api);
