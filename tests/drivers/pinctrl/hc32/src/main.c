/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/ztest.h>

/* pin configuration for test device */
#define TEST_DEVICE DT_NODELABEL(my_device)
PINCTRL_DT_DEV_CONFIG_DECLARE(TEST_DEVICE);
static struct pinctrl_dev_config *pcfg = PINCTRL_DT_DEV_CONFIG_GET(TEST_DEVICE);

ZTEST(pinctrl_hc32, test_dt_extract)
{
	const struct pinctrl_state *scfg;
	pinctrl_soc_pin_t pin;

	zassert_equal(pcfg->state_cnt, 1U);

	scfg = &pcfg->states[0];

	zassert_equal(scfg->id, PINCTRL_STATE_DEFAULT);
	zassert_equal(scfg->pin_cnt, 9U);

	pin = scfg->pins[1];
	zassert_equal(HC32_PORT(pin), 1);
	zassert_equal(HC32_PIN(pin), 8);
	zassert_equal(HC32_MODE(pin), HC32_FUNC);
	zassert_equal(HC32_FUNC_NUM(pin), 37);
	zassert_equal(HC32_PIN_BIAS(pin), HC32_NO_PULL);

	pin = scfg->pins[2];
	zassert_equal(HC32_PORT(pin), 0);
	zassert_equal(HC32_PIN(pin), 1);
	zassert_equal(HC32_MODE(pin), HC32_ANALOG);

	pin = scfg->pins[4];
	zassert_equal(HC32_PIN_DRIVER_STRENGTH(pin), HC32_DRIVER_STRENGTH_HIGH);

	pin = scfg->pins[6];
	zassert_equal(HC32_CINSEL(pin), HC32_CINSEL_CMOS);

}

ZTEST(pinctrl_hc32, test_lookup_state)
{
	int ret;
	const struct pinctrl_state *scfg;

	ret = pinctrl_lookup_state(pcfg, PINCTRL_STATE_DEFAULT, &scfg);
	zassert_equal(ret, 0);
	zassert_equal_ptr(scfg, &pcfg->states[0], NULL);
}

/**
 * @brief Test that pinctrl_apply_state() works as expected.
 */
ZTEST(pinctrl_hc32, test_apply_state)
{
	zassert_ok(pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT));
}

PINCTRL_DT_STATE_PINS_DEFINE(DT_PATH(zephyr_user), test_device1_default);
PINCTRL_DT_STATE_PINS_DEFINE(DT_PATH(zephyr_user), test_device1_sleep);
/** Test device alternative states. */
static const struct pinctrl_state test_device_alt[] = {
	PINCTRL_DT_STATE_INIT(test_device1_default, PINCTRL_STATE_DEFAULT),
};


static const struct pinctrl_state test_device_alt_invalid[] = {
	PINCTRL_DT_STATE_INIT(test_device1_default, PINCTRL_STATE_DEFAULT),
	/* sleep state is skipped (no CONFIG_PM_DEVICE), so it is invalid */
	PINCTRL_DT_STATE_INIT(test_device1_sleep, PINCTRL_STATE_SLEEP),
};

ZTEST(pinctrl_hc32, test_update_states)
{
	int ret;
	const struct pinctrl_state *scfg;
	pinctrl_soc_pin_t pin;

	ret = pinctrl_update_states(pcfg, test_device_alt, ARRAY_SIZE(test_device_alt));
	zassert_equal(ret, 0);

	scfg = &pcfg->states[0];
	pin = scfg->pins[0];
	zassert_equal(HC32_PORT(pin), 2);
	zassert_equal(HC32_PIN(pin), 5);
	zassert_equal(HC32_MODE(pin), HC32_FUNC);
	zassert_equal(HC32_FUNC_NUM(pin), 63);
	zassert_equal(HC32_PIN_BIAS(pin), HC32_NO_PULL);

	ret = pinctrl_update_states(pcfg, test_device_alt_invalid, ARRAY_SIZE(test_device_alt_invalid));
	zassert_equal(ret, -EINVAL);
}

ZTEST_SUITE(pinctrl_hc32, NULL, NULL, NULL, NULL, NULL);
