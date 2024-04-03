/*
* Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
*/
#include <soc.h>
#include <clock_control.h>
#include <misc/util.h>
#include "hc32f4_clock.h"
#include <drivers/clock_control/hc32_clock_control.h>

#define IN_RANGE(val, min, max) (val >= min && val <= max)
/**
 * @defgroup EV_HC32F460_LQFP100_V2_XTAL_CONFIG EV_HC32F460_LQFP100_V2 XTAL Configure definition
 * @{
 */
#define BSP_XTAL_PORT                   (GPIO_PORT_H)
#define BSP_XTAL_IN_PIN                 (GPIO_PIN_01)
#define BSP_XTAL_OUT_PIN                (GPIO_PIN_00)

/**
 * @defgroup EV_HC32F460_LQFP100_V2_XTAL32_CONFIG EV_HC32F460_LQFP100_V2 XTAL32 Configure definition
 * @{
 */
#define BSP_XTAL32_PORT                 (GPIO_PORT_C)
#define BSP_XTAL32_IN_PIN               (GPIO_PIN_15)
#define BSP_XTAL32_OUT_PIN              (GPIO_PIN_14)

static void hc32_clock_stale(u32_t flag)
{
	u32_t stable_time = 0;

	while (RESET == CLK_GetStableStatus(flag)) {
		if (stable_time ++ >= 20000) {
			break;
		}
	}
}

#if CONFIG_HC32_CLK_XTAL
static void hc32_clock_xtal_init(void)
{
	stc_clock_xtal_init_t     stcXtalInit;

	GPIO_AnalogCmd(BSP_XTAL_PORT, BSP_XTAL_IN_PIN | BSP_XTAL_OUT_PIN, ENABLE);

	(void)CLK_XtalStructInit(&stcXtalInit);
	stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
	stcXtalInit.u8Drv = XTAL_DRV;
	stcXtalInit.u8State = CLK_XTAL_ON;
	stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
	(void)CLK_XtalInit(&stcXtalInit);
	hc32_clock_stale(CLK_STB_FLAG_XTAL);
}
#endif

#if CONFIG_HC32_CLK_XTAL32
static void hc32_clock_xtal32_init(void)
{
	stc_clock_xtal32_init_t stcXtal32Init;

	(void)CLK_Xtal32StructInit(&stcXtal32Init);
	stcXtal32Init.u8State = CLK_XTAL32_ON;
	stcXtal32Init.u8Drv = CONFIG_HC32_XTAL32_DRV;
	stcXtal32Init.u8Filter = CLK_XTAL32_FILTER_ALL_MD;
	GPIO_AnalogCmd(BSP_XTAL32_PORT, BSP_XTAL32_IN_PIN | BSP_XTAL32_OUT_PIN, ENABLE);
	(void)CLK_Xtal32Init(&stcXtal32Init);
}
#endif

static void hc32_clock_hrc_init(void)
{
	CLK_HrcCmd(ENABLE);
	hc32_clock_stale(CLK_STB_FLAG_HRC);
}

static void hc32_clock_mrc_init(void)
{
	CLK_MrcCmd(ENABLE);
}

static void hc32_clock_lrc_init(void)
{
	CLK_LrcCmd(ENABLE);
}

#if CONFIG_HC32_SYSCLK_SRC_PLL
static void hc32_clock_pll_init(void)
{
	stc_clock_pll_init_t      stcMPLLInit;

	(void)CLK_PLLStructInit(&stcMPLLInit);
	stcMPLLInit.PLLCFGR = 0UL;
	stcMPLLInit.PLLCFGR_f.PLLM = (CONFIG_HC32_PLL_DIV_M - 1UL);
	stcMPLLInit.PLLCFGR_f.PLLN = (CONFIG_HC32_PLL_MUL_N - 1UL);
	stcMPLLInit.PLLCFGR_f.PLLP = (CONFIG_HC32_PLL_DIV_P - 1UL);
	stcMPLLInit.PLLCFGR_f.PLLQ = (CONFIG_HC32_PLL_DIV_Q - 1UL);
	stcMPLLInit.PLLCFGR_f.PLLR = (CONFIG_HC32_PLL_DIV_R - 1UL);
#if CONFIG_HC32_PLL_SRC_XTAL
	hc32_clock_xtal_init();
	stcMPLLInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
#elif CONFIG_HC32_PLL_SRC_HRC
	hc32_clock_hrc_init();
	stcMPLLInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_HRC;	
#endif
	stcMPLLInit.u8PLLState = CLK_PLL_ON;
	(void)CLK_PLLInit(&stcMPLLInit);
	hc32_clock_stale(CLK_STB_FLAG_PLL);
}
#endif

static void hc32_clk_conf(void)
{
#if CONFIG_HC32_SYSCLK_SRC_PLL
	hc32_clock_pll_init();
#endif
#if CONFIG_HC32_CLK_XTAL
	hc32_clock_xtal_init();
#endif
#if CONFIG_HC32_CLK_HRC
	hc32_clock_hrc_init();
#endif
#if CONFIG_HC32_CLK_MRC
	hc32_clock_mrc_init();
#endif
#if CONFIG_HC32_CLK_LRC
	hc32_clock_lrc_init();
#endif
#if CONFIG_HC32_CLK_XTAL32
	hc32_clock_xtal32_init();
#endif
}

static void hc32_run_mode_switch(uint32_t old_freq, uint32_t new_freq)
{
	uint8_t old_run_mode;
	uint8_t new_run_mode;

	new_run_mode = (new_freq >= 168000000) ? \
			2 : (new_freq >= 8000000) ? 1 : 0;
	old_run_mode = (old_freq >= 168000000) ? \
			2 : (old_freq >= 8000000) ? 1 : 0;
	if (old_run_mode == 0) {
		if (new_run_mode == 1) {
			PWC_LowSpeedToHighSpeed();
		} else if (new_run_mode == 2) {
			PWC_LowSpeedToHighPerformance();
		}
	} else if (old_run_mode == 1) {
		if (new_run_mode == 0) {
			PWC_HighSpeedToLowSpeed();
		} else if (new_run_mode == 2) {
			PWC_HighSpeedToHighPerformance();
		}
	} else if (old_run_mode == 2) {
		if (new_run_mode == 0) {
			PWC_HighPerformanceToLowSpeed();
		} else if (new_run_mode == 1) {
			PWC_HighPerformanceToHighSpeed();
		}
	} 
}

static int hc32_clock_control_init(struct device *dev)
{
	uint32_t old_core_freq;
	uint32_t new_core_freq;
	stc_clock_freq_t stcClockFreq;

	CLK_GetClockFreq(&stcClockFreq);
	old_core_freq = stcClockFreq.u32SysclkFreq;
	/* Set bus clk div. */
	CLK_SetClockDiv(CLK_BUS_CLK_ALL, (HC32_HCLK_DIV(CONFIG_HC32_BUS_HCLK_DIV) | \
									  HC32_EXCLK_DIV(CONFIG_HC32_BUS_EXCLK_DIV) | \
									  HC32_PCLK(0, CONFIG_HC32_BUS_PCLK0_DIV) | \
									  HC32_PCLK(1, CONFIG_HC32_BUS_PCLK1_DIV) | \
									  HC32_PCLK(2, CONFIG_HC32_BUS_PCLK2_DIV) | \
									  HC32_PCLK(3, CONFIG_HC32_BUS_PCLK3_DIV) | \
									  HC32_PCLK(4, CONFIG_HC32_BUS_PCLK4_DIV)));

	/* sram init include read/write wait cycle setting */
	SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
	SRAM_SetWaitCycle((SRAM_SRAM12 | SRAM_SRAM3 | SRAM_SRAMR), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

	/* flash read wait cycle setting */
	(void)EFM_SetWaitCycle(EFM_WAIT_CYCLE);
	/* 3 cycles for 126MHz ~ 200MHz */
	GPIO_SetReadWaitCycle(GPIO_RD_WAIT);

	hc32_clk_conf();

#if CONFIG_HC32_SYSCLK_SRC_PLL
	CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
#elif CONFIG_HC32_SYSCLK_SRC_XTAL
	CLK_SetSysClockSrc(CLK_SYSCLK_SRC_XTAL);
#elif CONFIG_HC32_SYSCLK_SRC_HRC
	CLK_SetSysClockSrc(CLK_SYSCLK_SRC_HRC);
#elif CONFIG_HC32_SYSCLK_SRC_MRC
	CLK_SetSysClockSrc(CLK_SYSCLK_SRC_MRC);
#elif CONFIG_HC32_SYSCLK_SRC_LRC
	CLK_SetSysClockSrc(CLK_SYSCLK_SRC_LRC);
#elif CONFIG_HC32_SYSCLK_SRC_XTAL32
	CLK_SetSysClockSrc(CLK_SYSCLK_SRC_XTAL32);
#endif

	CLK_GetClockFreq(&stcClockFreq);
	new_core_freq = stcClockFreq.u32SysclkFreq;
	hc32_run_mode_switch(old_core_freq, new_core_freq);

	return 0;
}

static inline int hc32_clock_control_on(struct device *dev,
					 clock_control_subsys_t sub_system)
{
	struct hc32_modules_clock_sys *clk_sys = \
		(struct hc32_modules_clock_sys *)sub_system;
	struct hc32_modules_clock_config *mod_conf = \
		(struct hc32_modules_clock_config *)dev->config->config_info;

	if (IN_RANGE(clk_sys->fcg, HC32_CLK_FCG0, HC32_CLK_FCG3) == 0) {
		return -ENOTSUP;
	}

	WRITE_BIT(*(uint32_t *)(mod_conf->addr + HC32_CLK_MODULES_OFFSET(clk_sys->fcg)), \
				clk_sys->bits, 0);
	return 0;
}

static inline int hc32_clock_control_off(struct device *dev,
					  clock_control_subsys_t sub_system)
{
	struct hc32_modules_clock_sys *clk_sys = \
		(struct hc32_modules_clock_sys *)sub_system;
	struct hc32_modules_clock_config *mod_conf = \
		(struct hc32_modules_clock_config *)dev->config->config_info;

	if (IN_RANGE(clk_sys->fcg, HC32_CLK_FCG0, HC32_CLK_FCG3) == 0) {
		return -ENOTSUP;
	}

	WRITE_BIT(*(uint32_t *)(mod_conf->addr + HC32_CLK_MODULES_OFFSET(clk_sys->fcg)), \
				clk_sys->bits, 1);
	return 0;
}

static int hc32_clock_control_get_subsys_rate(struct device *clock,
						clock_control_subsys_t sub_system,
						u32_t *rate)
{
	struct hc32_modules_clock_sys *clk_sys = \
		(struct hc32_modules_clock_sys *)sub_system;

	switch (clk_sys->bus)
	{
#if CONFIG_HC32_CLK_HRC
	case HC32_CLK_SRC_HRC:
		*rate = CONFIG_HC32_HRC_FREQ;
		break;
#endif
#if CONFIG_HC32_CLK_MRC
	case HC32_CLK_SRC_MRC:
		*rate = CONFIG_HC32_MRC_FREQ;
		break;
#endif
#if CONFIG_HC32_CLK_XTAL
	case HC32_CLK_SRC_XTAL:
		*rate = CONFIG_HC32_XTAL_FREQ;
		break;
#endif
#if CONFIG_HC32_SYSCLK_SRC_PLL
	case HC32_CLK_SRC_PLL:
		*rate = SYS_CLK_FREQ;
		break;
#endif
	case HC32_CLK_BUS_HCLK:
		*rate = CORE_CLK_FREQ;
		break;
	case HC32_CLK_BUS_PCLK0:
		*rate = PCLK0_FREQ;
		break;
	case HC32_CLK_BUS_PCLK1:
		*rate = PCLK1_FREQ;
		break;
	case HC32_CLK_BUS_PCLK2:
		*rate = PCLK2_FREQ;
		break;
	case HC32_CLK_BUS_PCLK3:
		*rate = PCLK3_FREQ;
		break;
	case HC32_CLK_BUS_PCLK4:
		*rate = PCLK4_FREQ;
		break;
	case HC32_SYS_CLK:
		*rate = SYS_CLK_FREQ;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static struct clock_control_driver_api hc32_clock_control_api = {
	.on = hc32_clock_control_on,
	.off = hc32_clock_control_off,
	.get_rate = hc32_clock_control_get_subsys_rate,
};

static const struct hc32_modules_clock_config hc32_modules_clk= {
	.addr = DT_XHSC_HC32_BUS_FCG_0_BASE_ADDRESS,
};

DEVICE_AND_API_INIT(cc_hc32, HC32_CLOCK_CONTROL_NAME,
		    &hc32_clock_control_init,
		    NULL, &hc32_modules_clk,
		    PRE_KERNEL_1,
		    1,
		    &hc32_clock_control_api);