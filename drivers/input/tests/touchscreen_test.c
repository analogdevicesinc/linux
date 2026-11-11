// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit test for touchscreen property parsing.
 */

#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/property.h>

#include <kunit/device.h>
#include <kunit/test.h>

static struct input_dev *
touchscreen_test_create_device(struct kunit *test,
			       const struct property_entry *props)
{
	struct input_dev *input_dev;
	struct device *dev;

	dev = kunit_device_register(test, "touchscreen-test-dev");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	if (props)
		KUNIT_ASSERT_EQ(test,
				device_create_managed_software_node(dev, props, NULL),
				0);

	input_dev = devm_input_allocate_device(dev);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, input_dev);

	return input_dev;
}

static void touchscreen_test_resolution_single(struct kunit *test)
{
	const struct property_entry props[] = {
		PROPERTY_ENTRY_U32("touchscreen-x-mm", 100),
		PROPERTY_ENTRY_U32("touchscreen-y-mm", 200),
		{ }
	};
	struct touchscreen_properties prop;
	struct input_dev *input_dev;

	input_dev = touchscreen_test_create_device(test, props);

	/* Driver-supplied ranges: 0..999 (1000 units), 0..1999 (2000 units) */
	input_set_abs_params(input_dev, ABS_X, 0, 999, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 1999, 0, 0);

	touchscreen_parse_properties(input_dev, false, &prop);

	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_X), 10);
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_Y), 10);
}

static void touchscreen_test_resolution_multitouch(struct kunit *test)
{
	const struct property_entry props[] = {
		PROPERTY_ENTRY_U32("touchscreen-min-x", 10),
		PROPERTY_ENTRY_U32("touchscreen-size-x", 1010),
		PROPERTY_ENTRY_U32("touchscreen-x-mm", 50),
		PROPERTY_ENTRY_U32("touchscreen-min-y", 20),
		PROPERTY_ENTRY_U32("touchscreen-size-y", 2020),
		PROPERTY_ENTRY_U32("touchscreen-y-mm", 100),
		{ }
	};
	struct touchscreen_properties prop;
	struct input_dev *input_dev;

	input_dev = touchscreen_test_create_device(test, props);

	/* Multitouch axes setup */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 1, 0, 0);

	touchscreen_parse_properties(input_dev, true, &prop);

	/* min=10, max=1009, count=1000, 1000 / 50 = 20 */
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_X), 20);
	/* min=20, max=2019, count=2000, 2000 / 100 = 20 */
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_Y), 20);
}

static void touchscreen_test_resolution_rounding(struct kunit *test)
{
	const struct property_entry props[] = {
		/* 1951 / 100 = 19.51 -> rounds to 20 */
		PROPERTY_ENTRY_U32("touchscreen-size-x", 1951),
		PROPERTY_ENTRY_U32("touchscreen-x-mm", 100),
		/* 1949 / 100 = 19.49 -> rounds to 19 */
		PROPERTY_ENTRY_U32("touchscreen-size-y", 1949),
		PROPERTY_ENTRY_U32("touchscreen-y-mm", 100),
		{ }
	};
	struct touchscreen_properties prop;
	struct input_dev *input_dev;

	input_dev = touchscreen_test_create_device(test, props);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 1, 0, 0);

	touchscreen_parse_properties(input_dev, true, &prop);

	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_X), 20);
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_Y), 19);
}

static void touchscreen_test_resolution_swap_invert(struct kunit *test)
{
	const struct property_entry props[] = {
		PROPERTY_ENTRY_U32("touchscreen-x-mm", 50),
		PROPERTY_ENTRY_U32("touchscreen-y-mm", 200),
		PROPERTY_ENTRY_BOOL("touchscreen-inverted-x"),
		PROPERTY_ENTRY_BOOL("touchscreen-swapped-x-y"),
		{ }
	};
	struct touchscreen_properties prop;
	struct input_dev *input_dev;

	input_dev = touchscreen_test_create_device(test, props);

	/* X count = 1000, Y count = 2000 */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 999, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 1999, 0, 0);

	touchscreen_parse_properties(input_dev, true, &prop);

	/*
	 * Before swap: X res = 1000/50 = 20, Y res = 2000/200 = 10.
	 * After swap: X absinfo receives Y absinfo, Y absinfo receives X absinfo.
	 */
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_X), 10);
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_Y), 20);
	KUNIT_EXPECT_TRUE(test, prop.invert_x);
	KUNIT_EXPECT_TRUE(test, prop.swap_x_y);
}

static void touchscreen_test_resolution_edge_cases(struct kunit *test)
{
	const struct property_entry props[] = {
		/* Zero x-mm */
		PROPERTY_ENTRY_U32("touchscreen-x-mm", 0),
		/* y-mm omitted */
		{ }
	};
	struct touchscreen_properties prop;
	struct input_dev *input_dev;

	input_dev = touchscreen_test_create_device(test, props);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 999, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 1999, 0, 0);

	/* Pre-configure resolutions */
	input_abs_set_res(input_dev, ABS_MT_POSITION_X, 15);
	input_abs_set_res(input_dev, ABS_MT_POSITION_Y, 25);

	touchscreen_parse_properties(input_dev, true, &prop);

	/* Zero size and omitted property must preserve pre-existing resolutions */
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_X), 15);
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_Y), 25);
}

static void touchscreen_test_resolution_invalid_range_and_null_prop(struct kunit *test)
{
	const struct property_entry props[] = {
		PROPERTY_ENTRY_U32("touchscreen-x-mm", 50),
		PROPERTY_ENTRY_U32("touchscreen-y-mm", 50),
		{ }
	};
	struct input_dev *input_dev;

	input_dev = touchscreen_test_create_device(test, props);

	/* X has valid range 0..999 (1000 units), Y has invalid range 500..500 */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 999, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 500, 500, 0, 0);
	input_abs_set_res(input_dev, ABS_MT_POSITION_Y, 42);

	/* NULL prop pointer */
	touchscreen_parse_properties(input_dev, true, NULL);

	/* Valid X gets resolution computed even with prop == NULL */
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_X), 20);
	/* Invalid Y range (max <= min) preserves pre-configured resolution */
	KUNIT_EXPECT_EQ(test, input_abs_get_res(input_dev, ABS_MT_POSITION_Y), 42);
}

static struct kunit_case touchscreen_tests[] = {
	KUNIT_CASE(touchscreen_test_resolution_single),
	KUNIT_CASE(touchscreen_test_resolution_multitouch),
	KUNIT_CASE(touchscreen_test_resolution_rounding),
	KUNIT_CASE(touchscreen_test_resolution_swap_invert),
	KUNIT_CASE(touchscreen_test_resolution_edge_cases),
	KUNIT_CASE(touchscreen_test_resolution_invalid_range_and_null_prop),
	{ /* sentinel */ }
};

static struct kunit_suite touchscreen_test_suite = {
	.name = "touchscreen",
	.test_cases = touchscreen_tests,
};

kunit_test_suite(touchscreen_test_suite);

MODULE_DESCRIPTION("KUnit test for touchscreen property parsing");
MODULE_LICENSE("GPL");
