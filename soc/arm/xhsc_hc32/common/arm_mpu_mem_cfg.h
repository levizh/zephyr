/*
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _ARM_MPU_MEM_CFG_H_
#define _ARM_MPU_MEM_CFG_H_

#include <soc.h>
#include <arch/arm/cortex_m/mpu/arm_mpu.h>

#define REGION_28K      (REGION_SIZE(32KB)-REGION_SIZE(8KB))
#define REGION_156K     (REGION_28K+REGION_SIZE(128KB))

/* Flash Region Definitions */
#if CONFIG_FLASH_SIZE == 64
#define REGION_FLASH_SIZE REGION_64K
#elif CONFIG_FLASH_SIZE == 128
#define REGION_FLASH_SIZE REGION_128K
#elif CONFIG_FLASH_SIZE == 256
#define REGION_FLASH_SIZE REGION_256K
#elif CONFIG_FLASH_SIZE == 512
#define REGION_FLASH_SIZE REGION_512K
#elif CONFIG_FLASH_SIZE == 1024
#define REGION_FLASH_SIZE REGION_1M
#elif CONFIG_FLASH_SIZE == 1536
#define REGION_FLASH_SIZE REGION_2M	/* last 512kB are not mapped */
#elif CONFIG_FLASH_SIZE == 2048
#define REGION_FLASH_SIZE REGION_2M
#else
#error "Unsupported configuration"
#endif

/* SRAM Region Definitions */
#if CONFIG_SRAM_SIZE == 32
#define REGION_SRAM_0_SIZE REGION_16K
#define REGION_SRAM_1_START 0x2000
#define REGION_SRAM_1_SIZE REGION_8K
#define REGION_SRAM_2_START 0x2000
#define REGION_SRAM_2_SIZE REGION_8K
#elif CONFIG_SRAM_SIZE == 156
#define REGION_SRAM_0_SIZE REGION_64K
#define REGION_SRAM_1_START 0x10000
#define REGION_SRAM_1_SIZE REGION_64K
#define REGION_SRAM_2_START 0x20000
#define REGION_SRAM_2_SIZE REGION_28K
#elif CONFIG_SRAM_SIZE == 188
#define REGION_SRAM_0_SIZE REGION_32K
#define REGION_SRAM_1_START 0x4000
#define REGION_SRAM_1_SIZE REGION_128K
#define REGION_SRAM_2_START 0x24000
#define REGION_SRAM_2_SIZE REGION_28K
#elif CONFIG_SRAM_SIZE == 192
#define REGION_SRAM_0_SIZE REGION_32K
#define REGION_SRAM_1_START 0x4000
#define REGION_SRAM_1_SIZE REGION_156K
#define REGION_SRAM_2_START 0xF8000
#define REGION_SRAM_2_SIZE REGION_4K
#else
#error "Unsupported configuration"
#endif

#endif /* _ARM_MPU_MEM_CFG_H_ */
