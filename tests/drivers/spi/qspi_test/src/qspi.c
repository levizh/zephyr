/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr.h>
#include <misc/printk.h>
#include <string.h>
#include <stdio.h>
#include <ztest.h>

#include <spi.h>
#include <qspi_hc32.h>

#define QSPI_DRV_NAME	CONFIG_QSPI_TEST_DRV_NAME
#define DEFAULT_FREQ	CONFIG_QSPI_TEST_FREQ
#define TEST_ADDR       0x600008u


#define CS_CTRL_GPIO_DRV_NAME  CONFIG_QSPI_CS_CTRL_GPIO_DRV_NAME
struct spi_cs_control spi_cs = {
	.gpio_pin = CONFIG_QSPI_CS_CTRL_GPIO_PIN,
	.delay = 0,
};
#define SPI_CS (&spi_cs)

struct qspi_instruction qspi_instr = {
	.content = 0x90u,
	.lines = 1u,
};

struct qspi_address qspi_add = {
	.content = 0u,
	.bytes = 3u,
	.lines = 1,
};

struct qspi_dummy qspi_dum = {
	.content = 0u,
	.dummy_cycles = 3u,
	.XIP_Enable = 0u,
};

struct qspi_hc32_config qspi_cfg = {
	.config.frequency = DEFAULT_FREQ,
	.config.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL,
	.config.cs = SPI_CS,
	.instruction = &qspi_instr,
	.address = &qspi_add,
	.data_lines = 1u,
};

#define BUF_SIZE   128
u8_t buffer_tx[BUF_SIZE] = {0};
u8_t buffer_rx[BUF_SIZE] = {};

/*
 * We need 5x(buffer size) + 1 to print a comma-separated list of each
 * byte in hex, plus a null.
 */
u8_t buffer_print_tx[BUF_SIZE * 5 + 1];
u8_t buffer_print_rx[BUF_SIZE * 5 + 1];

static void to_display_format(const u8_t *src, size_t size, char *dst)
{
	size_t i;

	for (i = 0; i < size; i++) {
		sprintf(dst + 5 * i, "0x%02x,", src[i]);
	}
}

static int cs_ctrl_gpio_config(void)
{
	spi_cs.gpio_dev = device_get_binding(CS_CTRL_GPIO_DRV_NAME);
	if (!spi_cs.gpio_dev) {
		LOG_ERR("Cannot find %s!", CS_CTRL_GPIO_DRV_NAME);
		zassert_not_null(spi_cs.gpio_dev, "Invalid gpio device");
		return -1;
	}

	return 0;
}

static int qspi_flash_readID_single(struct device *dev,
				    struct spi_config *spi_conf)
{
	/* 2 lines */
	qspi_instr.content = 0x90u;
	qspi_instr.lines = 1u;
	qspi_cfg.address = &qspi_add;
	qspi_add.content = 0u;
	qspi_add.lines = 1u;
	qspi_add.bytes = 3u;
	qspi_cfg.data_lines = 1u;
	const struct spi_buf rx_bufs[] = {
		{
			.buf = buffer_rx,
			.len = 2u,
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs)
	};

	int ret;

	LOG_INF("Start");

	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return ret;
	}

	if ((buffer_rx[0] != 0xefu) || ((buffer_rx[1] != 0x16u)
					&& (buffer_rx[1] != 0x17u))) {
		to_display_format(buffer_rx, 2u, buffer_print_rx);
		LOG_ERR("                           vs: %s",
			buffer_print_rx);
		zassert_false(1, "wrong data");
		return -1;
	}

	LOG_INF("Passed");

	return 0;
}

static int qspi_flash_readID_dual(struct device *dev,
				  struct spi_config *spi_conf)
{
	/* 2 lines */
	qspi_instr.content = 0x92u;
	qspi_instr.lines = 1u;
	qspi_add.content = 0u;
	qspi_add.lines = 2u;
	qspi_cfg.data_lines = 2u;

	const struct spi_buf rx_bufs[] = {
		{
			.buf = buffer_rx,
			.len = 3u,
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs)
	};

	int ret;

	LOG_INF("Start");

	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return ret;
	}

	if ((buffer_rx[1] != 0xefu) || ((buffer_rx[2] != 0x16u)
					&& (buffer_rx[2] != 0x17u))) {
		to_display_format(buffer_rx, 3u, buffer_print_rx);
		LOG_ERR("                           vs: %s",
			buffer_print_rx);
		zassert_false(1, "wrong data");
		return -1;
	}

	LOG_INF("Passed");

	return 0;
}

static int qspi_flash_readID_quad(struct device *dev,
				  struct spi_config *spi_conf)
{
	/* 2 lines */
	qspi_instr.content = 0x94u;
	qspi_instr.lines = 1u;
	qspi_add.content = 0u;
	qspi_add.lines = 4u;
	qspi_cfg.data_lines = 4u;

	const struct spi_buf rx_bufs[] = {
		{
			.buf = buffer_rx,
			.len = 5u,
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs)
	};

	int ret;

	LOG_INF("Start");

	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return ret;
	}

	if ((buffer_rx[3] != 0xefu) || ((buffer_rx[4] != 0x16u)
					&& (buffer_rx[4] != 0x17u))) {
		to_display_format(buffer_rx, 3u, buffer_print_rx);
		LOG_ERR("                           vs: %s",
			buffer_print_rx);
		zassert_false(1, "wrong data");
		return -1;
	}

	LOG_INF("Passed");

	return 0;
}

static int qspi_flash_section_erase(struct device *dev,
				    struct spi_config *spi_conf)
{
	struct spi_buf rx_bufs[] = {
		{
			.buf = buffer_rx,
			.len = 1u,
		},
	};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs)
	};
	int ret;

	LOG_INF("Start");

	(void)memset(buffer_rx, 0, BUF_SIZE);
	/* write enable */
	qspi_instr.content = 0x06u;
	qspi_cfg.address = NULL;
	ret = spi_transceive(dev, spi_conf, NULL, NULL);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	/* send erase instr */
	qspi_instr.content = 0x20u;
	qspi_instr.lines = 1u;
	qspi_cfg.address = &qspi_add;
	qspi_add.content = TEST_ADDR;
	qspi_add.lines = 1u;
	ret = spi_transceive(dev, spi_conf, NULL, NULL);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	/* read status-reg1 busy bit */
	qspi_instr.content = 0x05u;
	qspi_cfg.address = NULL;
	qspi_cfg.data_lines = 1u;

	do {
		k_sleep(50u);
		ret = spi_transceive(dev, spi_conf, NULL, &rx);
		if (ret) {
			LOG_ERR("Code %d", ret);
			zassert_false(ret, "SPI transceive failed");
			return -1;
		}
	} while (buffer_rx[0] & 1u);

	/* read data, check if erase success */
	qspi_instr.content = 0x3bu;
	qspi_cfg.address = &qspi_add;
	qspi_cfg.data_lines = 2u;
	rx_bufs[0].len = BUF_SIZE;
	qspi_cfg.dummy = &qspi_dum;
	qspi_dum.dummy_cycles = 8u;

	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	for (uint8_t i = 0u; i < BUF_SIZE; i++) {
		if (0xff != buffer_rx[i]) {
			zassert_false(1, "Buffer contents are not ff");
			return -1;
		}
	}

	LOG_INF("Passed");

	return 0;
}

static int qspi_flash_program(struct device *dev, struct spi_config *spi_conf)
{
	const struct spi_buf tx_bufs[] = {
		{
			.buf = buffer_tx,
			.len = BUF_SIZE,
		},
	};
	struct spi_buf rx_bufs[] = {
		{
			.buf = buffer_rx,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs)
	};
	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs)
	};
	int ret;

	LOG_INF("Start");

	(void)memset(buffer_rx, 0, BUF_SIZE);
	/* write enable */
	qspi_instr.content = 0x06u;
	qspi_cfg.address = NULL;
	qspi_cfg.dummy = NULL;
	ret = spi_transceive(dev, spi_conf, NULL, NULL);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	/* write data */
	qspi_instr.content = 0x32u;
	qspi_cfg.address = &qspi_add;
	qspi_cfg.data_lines = 4u;
	ret = spi_transceive(dev, spi_conf, &tx, NULL);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	/* read status-reg1 busy bit */
	qspi_instr.content = 0x05u;
	qspi_cfg.address = NULL;
	qspi_cfg.data_lines = 1u;

	do {
		k_sleep(50u);
		ret = spi_transceive(dev, spi_conf, NULL, &rx);
		if (ret) {
			LOG_ERR("Code %d", ret);
			zassert_false(ret, "SPI transceive failed");
			return -1;
		}
	} while (buffer_rx[0] & 1u);

	/* qual fast read data */
	qspi_instr.content = 0xEBu;
	qspi_cfg.address = &qspi_add;
	qspi_add.lines = 4u;
	qspi_add.bytes = 3u;
	qspi_add.content = TEST_ADDR;
	qspi_cfg.data_lines = 4u;
	qspi_dum.dummy_cycles = 6u;
	qspi_dum.content = 0xf0u;
	qspi_dum.XIP_Enable = 0u;
	qspi_cfg.dummy = &qspi_dum;

	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	if (memcmp(buffer_tx, buffer_rx, BUF_SIZE)) {
		to_display_format(buffer_tx, BUF_SIZE, buffer_print_tx);
		to_display_format(buffer_rx, BUF_SIZE, buffer_print_rx);
		LOG_ERR("Buffer contents are different: %s",
			buffer_print_tx);
		LOG_ERR("                           vs: %s",
			buffer_print_rx);
		zassert_false(1, "Buffer contents are different");
		return -1;
	}

	LOG_INF("Passed");

	return 0;
}

static int qspi_flash_write_status(struct device *dev,
				   struct spi_config *spi_conf)
{
	int ret;
	qspi_instr.content = 0x01u;
	qspi_cfg.address = NULL;
	qspi_cfg.data_lines = 1u;
	qspi_cfg.dummy = NULL;

	buffer_tx[0] = 0x40u;
	const struct spi_buf tx_bufs[] = {
		{
			.buf = buffer_tx,
			.len = 1u,
		},
	};

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs)
	};

	ret = spi_transceive(dev, spi_conf, &tx, NULL);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}
	return 0;
}

static int spi_resource_lock_test(struct device *lock_dev,
				  struct spi_config *spi_conf_lock)
{
	spi_conf_lock->operation |= SPI_LOCK_ON;

	qspi_flash_readID_single(lock_dev, spi_conf_lock);

	if (spi_release(lock_dev, spi_conf_lock)) {
		LOG_ERR("Deadlock now?");
		zassert_false(1, "SPI release failed");
		return -1;
	}

	return 0;
}

static int qspi_flash_XIP_quad_read(struct device *dev,
				    struct spi_config *spi_conf)
{
	struct spi_buf rx_bufs[] = {
		{
			.buf = buffer_rx,
			.len = BUF_SIZE / 2,
		},
	};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs)
	};
	int ret;
	buffer_tx[0] = 3u;
	LOG_INF("Start");

	(void)memset(buffer_rx, 0, BUF_SIZE);
	/* XIP mode read : half data */
	qspi_cfg.instruction = &qspi_instr;
	qspi_instr.content = 0xebu;
	qspi_cfg.address = &qspi_add;
	qspi_add.lines = 4u;
	qspi_add.bytes = 3u;
	qspi_add.content = TEST_ADDR;

	qspi_cfg.data_lines = 4u;

	qspi_cfg.dummy = &qspi_dum;
	qspi_dum.dummy_cycles = 6u;
	qspi_dum.content = 0x20u;
	qspi_dum.XIP_Enable = 1u;

	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	qspi_add.content = TEST_ADDR + BUF_SIZE / 2;

	rx_bufs[0].buf = buffer_rx + BUF_SIZE / 2;
	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	if (memcmp(buffer_tx, buffer_rx, BUF_SIZE)) {
		to_display_format(buffer_tx, BUF_SIZE, buffer_print_tx);
		to_display_format(buffer_rx, BUF_SIZE, buffer_print_rx);
		LOG_ERR("Buffer contents are different: %s",
			buffer_print_tx);
		LOG_ERR("                           vs: %s",
			buffer_print_rx);
		zassert_false(1, "Buffer contents are different");
		return -1;
	}
	LOG_INF("Passed");

	/* Exit xip mode */
	qspi_add.content = TEST_ADDR;
	qspi_dum.content = 0xffu;
	qspi_dum.XIP_Enable = 0u;
	(void)memset(buffer_rx, 0, BUF_SIZE);
	rx_bufs[0].buf = buffer_rx;
	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}
	return 0;
}

static int qspi_flash_XIP_dual_read(struct device *dev,
				    struct spi_config *spi_conf)
{
	struct spi_buf rx_bufs[] = {
		{
			.buf = buffer_rx,
			.len = BUF_SIZE / 2,
		},
	};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs)
	};
	int ret;

	LOG_INF("Start");

	(void)memset(buffer_rx, 0, BUF_SIZE);
	/* XIP mode read : half data */
	qspi_cfg.instruction = &qspi_instr;
	qspi_instr.content = 0xbbu;
	qspi_cfg.address = &qspi_add;
	qspi_add.lines = 2u;
	qspi_add.bytes = 3u;
	qspi_add.content = TEST_ADDR;

	qspi_cfg.data_lines = 2u;

	qspi_cfg.dummy = &qspi_dum;
	qspi_dum.dummy_cycles = 4u;
	qspi_dum.content = 0x20u;
	qspi_dum.XIP_Enable = 1u;

	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	qspi_add.content = TEST_ADDR + BUF_SIZE / 2;

	rx_bufs[0].buf = buffer_rx + BUF_SIZE / 2;
	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	if (memcmp(buffer_tx, buffer_rx, BUF_SIZE)) {
		to_display_format(buffer_tx, BUF_SIZE, buffer_print_tx);
		to_display_format(buffer_rx, BUF_SIZE, buffer_print_rx);
		LOG_ERR("Buffer contents are different: %s",
			buffer_print_tx);
		LOG_ERR("                           vs: %s",
			buffer_print_rx);
		zassert_false(1, "Buffer contents are different");
		return -1;
	}
	LOG_INF("Passed");

	/* Exit xip mode */
	qspi_add.content = TEST_ADDR;
	qspi_dum.content = 0xffu;
	qspi_dum.XIP_Enable = 0u;
	(void)memset(buffer_rx, 0, BUF_SIZE);
	rx_bufs[0].buf = buffer_rx;
	ret = spi_transceive(dev, spi_conf, NULL, &rx);
	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}
	return 0;
}


void testing_qspi(void)
{
	struct device *qspi_dev;

	LOG_INF("QSPI test on buffers TX/RX %p/%p", buffer_tx, buffer_rx);

	if (cs_ctrl_gpio_config()) {
		return;
	}

	qspi_dev = device_get_binding(QSPI_DRV_NAME);
	if (!qspi_dev) {
		LOG_ERR("Cannot find %s!\n", QSPI_DRV_NAME);
		zassert_not_null(qspi_dev, "Invalid QSPI device");
		return;
	}

	for (uint8_t i = 0u; i < BUF_SIZE; i++) {
		buffer_tx[i] = i + 3u;
	}

	if ((qspi_flash_readID_single(qspi_dev, (struct spi_config *)&qspi_cfg))
	    || (qspi_flash_readID_dual(qspi_dev, (struct spi_config *)&qspi_cfg))
	    || (qspi_flash_readID_quad(qspi_dev, (struct spi_config *)&qspi_cfg))
	    || qspi_flash_section_erase(qspi_dev, (struct spi_config *)&qspi_cfg)
	    || qspi_flash_program(qspi_dev, (struct spi_config *)&qspi_cfg)
	    || qspi_flash_write_status(qspi_dev, (struct spi_config *)&qspi_cfg)
	    || spi_resource_lock_test(qspi_dev, (struct spi_config *)&qspi_cfg)
	    || qspi_flash_XIP_quad_read(qspi_dev, (struct spi_config *)&qspi_cfg)) {
		goto end;
	}

	LOG_INF("All tx/rx passed");
end:
	LOG_INF("Test failed");
}

/*test case main entry*/
void test_main(void)
{
	ztest_test_suite(test_qspi, ztest_unit_test(testing_qspi));
	ztest_run_test_suite(test_qspi);
}
