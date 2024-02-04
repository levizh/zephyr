#
# Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
#
# SPDX-License-Identifier: Apache-2.0
#

add_subdirectory(${SOC_SERIES})
#add_subdirectory(common)

zephyr_linker_sources_ifdef(CONFIG_HC32_LL_ICG
  ROM_START
  SORT_KEY 0x400
  flash_config.ld
  )