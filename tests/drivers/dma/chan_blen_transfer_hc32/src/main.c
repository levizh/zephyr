/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup t_driver_dma
 * @{
 * @defgroup t_dma_mem_to_mem test_dma_mem_to_mem
 * @}
 */

#include <zephyr.h>
#include <ztest.h>

#include <dma.h>
#include <dma/dma_hc32.h>

extern void test_dma_m2m_chan0_burst8(void);
extern void test_dma_m2m_chan1_burst8(void);
extern void test_dma_m2m_chan0_burst16(void);
extern void test_dma_m2m_chan1_burst16(void);

extern void test_dma_m2m_chan0_burst8_long(void);
extern void test_dma_m2m_chan1_burst8_long(void);
extern void test_dma_m2m_chan0_burst16_long(void);
extern void test_dma_m2m_chan1_burst16_long(void);

#ifdef CONFIG_SHELL
TC_CMD_DEFINE(test_dma_m2m_chan0_burst8)
TC_CMD_DEFINE(test_dma_m2m_chan1_burst8)
TC_CMD_DEFINE(test_dma_m2m_chan0_burst16)
TC_CMD_DEFINE(test_dma_m2m_chan1_burst16)

SHELL_CMD_REGISTER(test_dma_m2m_chan0_burst8, NULL, NULL,
			TC_CMD_ITEM(test_dma_m2m_chan0_burst8));
SHELL_CMD_REGISTER(test_dma_m2m_chan1_burst8, NULL, NULL,
			TC_CMD_ITEM(test_dma_m2m_chan1_burst8));
SHELL_CMD_REGISTER(test_dma_m2m_chan0_burst16, NULL, NULL,
			TC_CMD_ITEM(test_dma_m2m_chan0_burst16));
SHELL_CMD_REGISTER(test_dma_m2m_chan1_burst16, NULL, NULL,
			TC_CMD_ITEM(test_dma_m2m_chan1_burst16));
#endif

void test_main(void)
{
	struct device *dma_dev = device_get_binding(dma_hc32_get_device_name(1U));
	ARG_UNUSED(dma_dev);

#ifndef CONFIG_SHELL
	ztest_test_suite(dma_m2m_test,
			 ztest_unit_test(test_dma_m2m_chan0_burst8),
			 ztest_unit_test(test_dma_m2m_chan1_burst8),
			 ztest_unit_test(test_dma_m2m_chan0_burst16),
			 ztest_unit_test(test_dma_m2m_chan1_burst16),
			 ztest_unit_test(test_dma_m2m_chan0_burst8_long),
			 ztest_unit_test(test_dma_m2m_chan1_burst8_long),
			 ztest_unit_test(test_dma_m2m_chan0_burst16_long),
			 ztest_unit_test(test_dma_m2m_chan1_burst16_long));
	ztest_run_test_suite(dma_m2m_test);
#endif
}
