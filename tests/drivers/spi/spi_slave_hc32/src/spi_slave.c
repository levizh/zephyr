/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_loopback);

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <zephyr/ztest.h>

#include <zephyr/drivers/spi.h>

#define SPI_FAST_DEV	DT_COMPAT_GET_ANY_STATUS_OKAY(test_spi_loopback_fast)
#define SPI_SLOW_DEV	DT_COMPAT_GET_ANY_STATUS_OKAY(test_spi_loopback_slow)

#if CONFIG_SPI_LOOPBACK_MODE_LOOP
#define MODE_LOOP SPI_MODE_LOOP
#else
#define MODE_LOOP 0
#endif

#ifdef CONFIG_SPI_LOOPBACK_16BITS_FRAMES
#define FRAME_SIZE (16)
#define FRAME_SIZE_STR ", frame size = 16"
#else
#define FRAME_SIZE (8)
#define FRAME_SIZE_STR ", frame size = 8"
#endif /* CONFIG_SPI_LOOPBACK_16BITS_FRAMES */

#ifdef CONFIG_DMA

#ifdef CONFIG_NOCACHE_MEMORY
#define DMA_ENABLED_STR ", DMA enabled"
#else /* CONFIG_NOCACHE_MEMORY */
#define DMA_ENABLED_STR ", DMA enabled (without CONFIG_NOCACHE_MEMORY)"
#endif

#else /* CONFIG_DMA */

#define DMA_ENABLED_STR
#endif /* CONFIG_DMA */

#define SPI_OP(frame_size) SPI_OP_MODE_SLAVE | SPI_MODE_CPOL | MODE_LOOP | \
	       SPI_MODE_CPHA | SPI_WORD_SET(frame_size) | SPI_LINES_SINGLE

static struct spi_dt_spec spi_slow = SPI_DT_SPEC_GET(SPI_SLOW_DEV, SPI_OP(8), 0);

/* to run this test, connect MOSI pin to the MISO of the SPI */

#define STACK_SIZE 1024
#define BUF_SIZE 36

#if CONFIG_NOCACHE_MEMORY
#define __NOCACHE	__attribute__((__section__(".nocache")))
#elif defined(CONFIG_DT_DEFINED_NOCACHE)
#define __NOCACHE	__attribute__((__section__(CONFIG_DT_DEFINED_NOCACHE_NAME)))
#else /* CONFIG_NOCACHE_MEMORY */
#define __NOCACHE
#endif /* CONFIG_NOCACHE_MEMORY */

static const char tx_data[BUF_SIZE] = "SPI Master/Slave example:01234567895";
static __aligned(32) char buffer_tx[BUF_SIZE] __used __NOCACHE;
static __aligned(32) char buffer_rx[BUF_SIZE] __used __NOCACHE;

/*
 * We need 5x(buffer size) + 1 to print a comma-separated list of each
 * byte in hex, plus a null.
 */
static uint8_t buffer_print_tx[BUF_SIZE * 5 + 1];
static uint8_t buffer_print_rx[BUF_SIZE * 5 + 1];

static void to_display_format(const uint8_t *src, size_t size, char *dst)
{
	size_t i;

	for (i = 0; i < size; i++) {
		sprintf(dst + 5 * i, "0x%02x,", src[i]);
	}
}

static int spi_complete_loop(struct spi_dt_spec *spec)
{
	const struct spi_buf tx_bufs[] = {
		{
			.buf = buffer_tx,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf rx_bufs[] = {
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

	LOG_INF("Start complete loop");
	memset(buffer_rx, 0, sizeof(buffer_rx));
	ret = spi_transceive_dt(spec, &tx, &rx);
	if (ret != BUF_SIZE) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return ret;
	}

	if (memcmp(buffer_tx, buffer_rx, BUF_SIZE)) {
		to_display_format(buffer_tx, BUF_SIZE, buffer_print_tx);
		to_display_format(buffer_rx, BUF_SIZE, buffer_print_rx);
		LOG_ERR("Buffer contents are different: %s", buffer_print_tx);
		LOG_ERR("                           vs: %s", buffer_print_rx);
		zassert_false(1, "Buffer contents are different");
		return -1;
	}

	LOG_INF("Passed");

	return 0;
}

#if (CONFIG_SPI_ASYNC)
static struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
static struct k_poll_event async_evt =
	K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
				 K_POLL_MODE_NOTIFY_ONLY,
				 &async_sig);
static K_SEM_DEFINE(caller, 0, 1);
K_THREAD_STACK_DEFINE(spi_async_stack, STACK_SIZE);
static int result = 1;

static void spi_async_call_cb(struct k_poll_event *evt,
			      struct k_sem *caller_sem,
			      void *unused)
{
	int ret;

	LOG_DBG("Polling...");

	while (1) {
		ret = k_poll(evt, 1, K_FOREVER);
		zassert_false(ret, "one or more events are not ready");

		result = evt->signal->result;
		k_sem_give(caller_sem);

		/* Reinitializing for next call */
		evt->signal->signaled = 0U;
		evt->state = K_POLL_STATE_NOT_READY;
	}
}

static int spi_async_call(struct spi_dt_spec *spec)
{
	const struct spi_buf tx_bufs[] = {
		{
			.buf = buffer_tx,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf rx_bufs[] = {
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

	LOG_INF("Start async call");

	ret = spi_transceive_signal(spec->bus, &spec->config, &tx, &rx, &async_sig);
	if (ret == -ENOTSUP) {
		LOG_DBG("Not supported");
		return 0;
	}

	if (ret) {
		LOG_ERR("Code %d", ret);
		zassert_false(ret, "SPI transceive failed");
		return -1;
	}

	k_sem_take(&caller, K_FOREVER);

	if (result != BUF_SIZE) {
		LOG_ERR("Call code %d", ret);
		zassert_false(result, "SPI transceive failed");
		return -1;
	}

	LOG_INF("Passed");

	return 0;
}
#endif

static int spi_resource_lock_test(struct spi_dt_spec *lock_spec)
{
	lock_spec->config.operation |= SPI_LOCK_ON;

	if (spi_complete_loop(lock_spec)) {
		return -1;
	}

	if (spi_release_dt(lock_spec)) {
		LOG_ERR("Deadlock now?");
		zassert_false(1, "SPI release failed");
		return -1;
	}

	return 0;
}

ZTEST(spi_loopback, test_spi_loopback)
{
#if (CONFIG_SPI_ASYNC)
	struct k_thread async_thread;
	k_tid_t async_thread_id;
#endif

	LOG_INF("SPI test on buffers TX/RX %p/%p" FRAME_SIZE_STR DMA_ENABLED_STR,
			buffer_tx,
			buffer_rx);
	
	zassert_true(spi_is_ready_dt(&spi_slow), "Fast spi lookback device is not ready");

	LOG_INF("SPI test fast config");

	if (spi_complete_loop(&spi_slow)) {
		goto end;
	}

	if (spi_resource_lock_test(&spi_slow)) {
		goto end;
	}

#if ((CONFIG_SPI_ASYNC)&&(!CONFIG_SPI_HC32_DMA))
	async_thread_id = k_thread_create(&async_thread,
					  spi_async_stack,  K_THREAD_STACK_SIZEOF(spi_async_stack),
					  (k_thread_entry_t)spi_async_call_cb,
					  &async_evt, &caller, NULL,
					  K_HIGHEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);

	spi_async_call(&spi_slow);
#endif

	LOG_INF("All tx/rx passed");
end:

#if ((CONFIG_SPI_ASYNC)&&(!CONFIG_SPI_HC32_DMA))
	k_thread_abort(async_thread_id);
#else
	;
#endif
}

static void *spi_loopback_setup(void)
{
	memset(buffer_tx, 0, sizeof(buffer_tx));
	memcpy(buffer_tx, tx_data, sizeof(tx_data));
	return NULL;
}

ZTEST_SUITE(spi_loopback, NULL, spi_loopback_setup, NULL, NULL, NULL);
