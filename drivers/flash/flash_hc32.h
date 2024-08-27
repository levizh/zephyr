/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_FLASH_FLASH_HC32_H_
#define ZEPHYR_DRIVERS_FLASH_FLASH_HC32_H_

#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>

#define HC32_SOC_NV_FLASH_NODE		DT_INST(0, soc_nv_flash)
#define HC32_SOC_NV_FLASH_SIZE		DT_REG_SIZE(HC32_SOC_NV_FLASH_NODE)
#define HC32_SOC_NV_FLASH_ADDR		DT_REG_ADDR(HC32_SOC_NV_FLASH_NODE)
#define HC32_SOC_NV_FLASH_PRG_SIZE	\
	DT_PROP(HC32_SOC_NV_FLASH_NODE, write_block_size)

/* Reserved address in flash */
#define HC32_SOC_NV_FLASH_RSVD_ADDR	\
	DT_REG_ADDR_BY_IDX(DT_NODELABEL(reserved_partition), 0)
#define HC32_SOC_NV_FLASH_RSVD_SIZE	\
	DT_REG_SIZE_BY_IDX(DT_NODELABEL(reserved_partition), 1)

#if (4 == HC32_SOC_NV_FLASH_PRG_SIZE)
typedef uint32_t flash_prg_t;
#elif (2 == HC32_SOC_NV_FLASH_PRG_SIZE)
typedef uint16_t flash_prg_t;
#elif (1 == HC32_SOC_NV_FLASH_PRG_SIZE)
typedef uint8_t flash_prg_t;
#else
#error "Invalid write-unit-size value"
#endif

/* Helper for conditional compilation directives, KB cannot be used because it has type casting. */
#define PRE_KB(x) ((x) << 10)

bool flash_hc32_valid_range(off_t offset, uint32_t len, bool write);

int flash_hc32_write_range(off_t offset, const void *data, size_t len);

int flash_hc32_erase_block(off_t offset, size_t size);

#ifdef CONFIG_FLASH_PAGE_LAYOUT
void flash_hc32_pages_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size);
#endif
#endif /* ZEPHYR_DRIVERS_FLASH_FLASH_HC32_H_ */
