/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Linker command/script file
 *
 * This is the linker script for both standard images and XIP images.
 */

#include <autoconf.h>

#define SKIP_TO_KINETIS_FLASH_CONFIG . = 0x400;
#define kinetis_flash_config         icg_sec

#include <arch/arm/cortex_m/scripts/linker.ld>
