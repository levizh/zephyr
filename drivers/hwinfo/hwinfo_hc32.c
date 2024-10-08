/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include <hc32_ll.h>
#include <zephyr/drivers/hwinfo.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>


ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint8_t i, id_len;
	stc_efm_unique_id_t stc_uid = {0};

	EFM_GetUID(&stc_uid);

	id_len = (length < sizeof(stc_uid) / sizeof(uint8_t)) ? length :
		(sizeof(stc_uid) / sizeof(uint8_t));
	for (i = 0; i < id_len; i++) {
		buffer[i] = *((uint8_t *)&stc_uid + i);
	}

	return id_len;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t flags = 0;

	if (RMU_GetStatus(RMU_FLAG_PIN)) {
		flags |= RESET_PIN;
	}

	if (RMU_GetStatus(RMU_FLAG_SW)) {
		flags |= RESET_SOFTWARE;
	}

	if (RMU_GetStatus(RMU_FLAG_BROWN_OUT)) {
		flags |= RESET_BROWNOUT;
	}

	if (RMU_GetStatus(RMU_FLAG_PWR_ON)) {
		flags |= RESET_POR;
	}

	if (RMU_GetStatus(RMU_FLAG_WDT | RMU_FLAG_SWDT)) {
		flags |= RESET_WATCHDOG;
	}

	if (RMU_GetStatus(RMU_FLAG_PWR_DOWN)) {
		flags |= RESET_LOW_POWER_WAKE;
	}

	if (RMU_GetStatus(RMU_FLAG_RAM_PARITY_ERR)) {
		flags |= RESET_PARITY;
	}

	if (RMU_GetStatus(RMU_FLAG_CLK_ERR)) {
		flags |= RESET_CLOCK;
	}

	if (RMU_GetStatus(RMU_FLAG_PVD1 | RMU_FLAG_PVD2)) {
		flags |= RESET_HARDWARE;
	}

	*cause = flags;
	return 0;
}


int z_impl_hwinfo_clear_reset_cause(void)
{
	RMU_ClearStatus();
	return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = (
		RESET_PIN 			/* External pin reset, NRST */
		| RESET_SOFTWARE 	/* Reset caused by a writing AIRCR.SYSRESETREQ */
		| RESET_BROWNOUT 	/* VCC brown out */
		| RESET_POR 		/* Power supply reset */
		| RESET_WATCHDOG 	/* Reset caused by WDT or SWDT */
		| RESET_LOW_POWER_WAKE /* Reset from power down mode */
		| RESET_PARITY 		/* RAM parity error */
		| RESET_CLOCK 		/* Reset caused by FCM */
		| RESET_HARDWARE 	/* LVD related reset */
	);

	return 0;
}
