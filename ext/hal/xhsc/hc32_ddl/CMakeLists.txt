# Makefile - HC32_DDL
#
# Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
#
# SPDX-License-Identifier: Apache-2.0

string(TOUPPER ${CONFIG_SOC} _HC32_CPU)

# HC32 ddl uses the CPU name to expose SOC-specific attributes of a specific
# peripheral.
# It also requires USE_DDL_DRIVER
zephyr_compile_definitions(
  -D${_HC32_CPU}
  -DUSE_DDL_DRIVER
  )

if(CONFIG_HC32_DDL_DEBUG)
  zephyr_compile_definitions(
    -D__DEBUG
    )
endif()

set(hc32_socs
  hc32f460
  hc32f4a0
  )

foreach(hc32_soc ${hc32_socs})
  string(TOUPPER ${hc32_soc} soc_to_upper)
  if(CONFIG_SOC_${soc_to_upper})
    zephyr_include_directories(
	  ${hc32_soc}/soc
	  ${hc32_soc}/drivers/inc
	  )

    add_subdirectory(${hc32_soc})
  endif()
endforeach()
