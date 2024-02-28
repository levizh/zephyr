#ifndef _HC32F4_CLOCK_
#define _HC32F4_CLOCK_

#include <zephyr/devicetree.h>
#include <autoconf.h>


#if (CORE_CLK_FREQ > 168000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE5
#elif (CORE_CLK_FREQ > 132000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE4
#elif (CORE_CLK_FREQ > 99000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE3
#elif (CORE_CLK_FREQ > 66000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE2
#elif (CORE_CLK_FREQ > 33000000)
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE1
#else
#define EFM_WAIT_CYCLE EFM_WAIT_CYCLE0
#endif

#if (CORE_CLK_FREQ > 126000000)
#define GPIO_RD_WAIT GPIO_RD_WAIT3
#elif (CORE_CLK_FREQ > 84000000)
#define GPIO_RD_WAIT GPIO_RD_WAIT2
#elif (CORE_CLK_FREQ > 42000000)
#define GPIO_RD_WAIT GPIO_RD_WAIT1
#else
#define GPIO_RD_WAIT GPIO_RD_WAIT0
#endif

#define HC32_CLOCK_CONTROL_NAME "hc32-cc"

struct hc32_modules_clock_config {
	uint32_t addr;
};
#endif // !_HC32F4_CLOCK_
