// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Sasha Levin <sashal@kernel.org>
 *
 * KUnit tests for the Kernel API Specification Framework
 *
 * Tests registration, lookup, validation, and JSON export functionality.
 */

#include <kunit/test.h>
#include <linux/kernel_api_spec.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/capability.h>
#include <linux/mm.h>
#include <linux/signal.h>
#include <linux/uaccess.h>

static void init_test_spec(struct kernel_api_spec *spec, const char *name)
{
	memset(spec, 0, sizeof(*spec));
	spec->name = name;
	spec->version = 1;
	spec->description = "Test API";
}

/* kapi_register_spec with valid spec returns 0 */
static void test_register_valid(struct kunit *test)
{
	struct kernel_api_spec *spec;
	int ret;

	spec = kzalloc_obj(*spec, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	init_test_spec(spec, "test_register_valid");

	ret = kapi_register_spec(spec);
	KUNIT_EXPECT_EQ(test, ret, 0);

	kapi_unregister_spec("test_register_valid");
	kfree(spec);
}

/* kapi_get_spec returns registered spec */
static void test_lookup_registered(struct kunit *test)
{
	struct kernel_api_spec *spec;
	const struct kernel_api_spec *found;
	int ret;

	spec = kzalloc_obj(*spec, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	init_test_spec(spec, "test_lookup_func");

	ret = kapi_register_spec(spec);
	KUNIT_ASSERT_EQ(test, ret, 0);

	found = kapi_get_spec("test_lookup_func");
	KUNIT_EXPECT_PTR_EQ(test, found, (const struct kernel_api_spec *)spec);

	kapi_unregister_spec("test_lookup_func");
	kfree(spec);
}

/* Double registration returns -EEXIST */
static void test_double_register(struct kunit *test)
{
	struct kernel_api_spec *spec;
	int ret;

	spec = kzalloc_obj(*spec, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	init_test_spec(spec, "test_double_reg");

	ret = kapi_register_spec(spec);
	KUNIT_ASSERT_EQ(test, ret, 0);

	ret = kapi_register_spec(spec);
	KUNIT_EXPECT_EQ(test, ret, -EEXIST);

	kapi_unregister_spec("test_double_reg");
	kfree(spec);
}

/* Unregister makes spec unfindable */
static void test_unregister(struct kunit *test)
{
	struct kernel_api_spec *spec;
	const struct kernel_api_spec *found;
	int ret;

	spec = kzalloc_obj(*spec, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	init_test_spec(spec, "test_unreg_func");

	ret = kapi_register_spec(spec);
	KUNIT_ASSERT_EQ(test, ret, 0);

	kapi_unregister_spec("test_unreg_func");

	found = kapi_get_spec("test_unreg_func");
	KUNIT_EXPECT_NULL(test, found);

	kfree(spec);
}

/* kapi_get_spec(NULL) returns NULL */
static void test_get_spec_null(struct kunit *test)
{
	const struct kernel_api_spec *found;

	found = kapi_get_spec(NULL);
	KUNIT_EXPECT_NULL(test, found);
}

/* kapi_register_spec(NULL) returns -EINVAL */
static void test_register_null(struct kunit *test)
{
	int ret;

	ret = kapi_register_spec(NULL);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

/* Spec with a NULL name is rejected */
static void test_register_null_name(struct kunit *test)
{
	struct kernel_api_spec *spec;
	int ret;

	spec = kzalloc_obj(*spec, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	/* spec->name == NULL after zero-init; registration rejects it. */
	ret = kapi_register_spec(spec);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	kfree(spec);
}

#ifdef CONFIG_KAPI_RUNTIME_CHECKS

/* RANGE constraint - value in range is valid */
static void test_constraint_range_valid(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_param";
	param.constraint_type = KAPI_CONSTRAINT_RANGE;
	param.min_value = 0;
	param.max_value = 100;

	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 0));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 50));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 100));

	param.min_value = -10;
	param.max_value = -1;
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, -10));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, -1));

	/* An unsigned maximum such as U64_MAX reads as -1 */
	param.min_value = 0;
	param.max_value = (s64)U64_MAX;
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, S64_MAX));
}

/* RANGE constraint - value out of range is invalid */
static void test_constraint_range_invalid(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_param";
	param.constraint_type = KAPI_CONSTRAINT_RANGE;
	param.min_value = 0;
	param.max_value = 100;

	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, -1));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 101));

	param.min_value = -10;
	param.max_value = -1;
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, -11));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 0));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 5));
}

/* MASK constraint - valid bits pass */
static void test_constraint_mask_valid(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_flags";
	param.constraint_type = KAPI_CONSTRAINT_MASK;
	param.valid_mask = 0xFF;

	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 0x00));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 0x0F));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 0xFF));
}

/* MASK constraint - extra bits fail */
static void test_constraint_mask_invalid(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_flags";
	param.constraint_type = KAPI_CONSTRAINT_MASK;
	param.valid_mask = 0xFF;

	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 0x100));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 0x1FF));
}

/* POWER_OF_TWO constraint */
static void test_constraint_power_of_two(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_pot";
	param.constraint_type = KAPI_CONSTRAINT_POWER_OF_TWO;

	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 1));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 2));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 4));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 8));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 0));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 3));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 5));
}

/* PAGE_ALIGNED constraint */
static void test_constraint_page_aligned(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_page";
	param.constraint_type = KAPI_CONSTRAINT_PAGE_ALIGNED;

	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 0));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, PAGE_SIZE));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 2 * PAGE_SIZE));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 1));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, PAGE_SIZE - 1));
}

/* NONZERO constraint */
static void test_constraint_nonzero(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_nz";
	param.constraint_type = KAPI_CONSTRAINT_NONZERO;

	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 0));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 1));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, -1));
}

/* Return value validation - success */
static void test_return_validation(struct kunit *test)
{
	struct kernel_api_spec *spec;

	spec = kunit_kzalloc(test, sizeof(*spec), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	spec->name = "test_ret";
	spec->return_magic = KAPI_MAGIC_RETURN;
	spec->return_spec.check_type = KAPI_RETURN_EXACT;
	spec->return_spec.success_value = 0;

	KUNIT_EXPECT_TRUE(test, kapi_validate_return_value(spec, 0));
}

/* Return value validation - known error */
static void test_return_known_error(struct kunit *test)
{
	struct kernel_api_spec *spec;

	spec = kunit_kzalloc(test, sizeof(*spec), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	spec->name = "test_ret_err";
	spec->return_magic = KAPI_MAGIC_RETURN;
	spec->return_spec.check_type = KAPI_RETURN_FD;
	spec->error_count = 1;
	spec->errors[0].error_code = -ENOENT;
	spec->errors[0].name = "ENOENT";

	/* -ENOENT is in the error list, so it's valid */
	KUNIT_EXPECT_TRUE(test, kapi_validate_return_value(spec, -ENOENT));
}

/* Return value validation - unknown error */
static void test_return_unknown_error(struct kunit *test)
{
	struct kernel_api_spec *spec;

	spec = kunit_kzalloc(test, sizeof(*spec), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	spec->name = "test_ret_unk";
	spec->return_magic = KAPI_MAGIC_RETURN;
	spec->return_spec.check_type = KAPI_RETURN_FD;
	spec->error_count = 1;
	spec->errors[0].error_code = -ENOENT;
	spec->errors[0].name = "ENOENT";

	/* -EPERM is not in the error list, but unlisted errors are accepted
	 * since filesystem/device-specific errors may not be exhaustively listed
	 */
	KUNIT_EXPECT_TRUE(test, kapi_validate_return_value(spec, -EPERM));
}

/* ALIGNMENT constraint */
static void test_constraint_alignment(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_align";
	param.constraint_type = KAPI_CONSTRAINT_ALIGNMENT;
	param.alignment = 8;

	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 0));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 8));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 16));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 1));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 7));
}

/* FD validation rejects values > INT_MAX */
static void test_fd_int_overflow(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_fd";
	param.type = KAPI_TYPE_FD;
	param.constraint_type = KAPI_CONSTRAINT_NONE;

	/* Value that overflows int: 0x100000003 -> truncates to 3 */
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 0x100000003LL));
}

/* ENUM constraint */
static const s64 test_enum_vals[] = { 1, 5, 10 };

static void test_constraint_enum(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_enum";
	param.constraint_type = KAPI_CONSTRAINT_ENUM;
	param.enum_values = test_enum_vals;
	param.enum_count = ARRAY_SIZE(test_enum_vals);

	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 1));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 5));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 10));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 0));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 3));
	KUNIT_EXPECT_FALSE(test, kapi_validate_param(&param, 11));
}

/* BUFFER constraint always accepts (size checked at runtime) */
static void test_constraint_buffer(struct kunit *test)
{
	struct kapi_param_spec param = {};

	param.name = "test_buf";
	param.constraint_type = KAPI_CONSTRAINT_BUFFER;

	/* Buffer constraint doesn't validate the value itself */
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 0));
	KUNIT_EXPECT_TRUE(test, kapi_validate_param(&param, 4096));
}

/* RETURN_RANGE check type */
static void test_return_range(struct kunit *test)
{
	struct kernel_api_spec *spec;

	spec = kunit_kzalloc(test, sizeof(*spec), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	spec->name = "test_ret_range";
	spec->return_magic = KAPI_MAGIC_RETURN;
	spec->return_spec.check_type = KAPI_RETURN_RANGE;
	spec->return_spec.success_min = 0;
	spec->return_spec.success_max = 100;

	KUNIT_EXPECT_TRUE(test, kapi_validate_return_value(spec, 0));
	KUNIT_EXPECT_TRUE(test, kapi_validate_return_value(spec, 50));
	KUNIT_EXPECT_TRUE(test, kapi_validate_return_value(spec, 100));
}

static void init_dyn_buf_param(struct kapi_param_spec *param)
{
	param->name = "buf";
	param->type = KAPI_TYPE_USER_PTR;
	param->constraint_type = KAPI_CONSTRAINT_BUFFER;
	param->size_param_idx = 2;
}

static bool validate_dyn_buf(const struct kapi_param_spec *param,
			     const void __user *ptr, s64 count)
{
	s64 params[] = { (s64)(unsigned long)ptr, count };

	return kapi_validate_param_with_context(param, params[0], params,
						ARRAY_SIZE(params));
}

/* Dynamic buffer: any pointer is accepted when the size is 0 */
static void test_dyn_buf_zero_count(struct kunit *test)
{
	struct kapi_param_spec param = {};
	const void __user *kernel_addr = (void __user *)-(unsigned long)PAGE_SIZE;

	init_dyn_buf_param(&param);

	KUNIT_EXPECT_TRUE(test, validate_dyn_buf(&param, NULL, 0));
	KUNIT_EXPECT_TRUE(test, validate_dyn_buf(&param, (void __user *)PAGE_SIZE, 0));
	KUNIT_EXPECT_TRUE(test, validate_dyn_buf(&param, kernel_addr, 0));
}

/* Dynamic buffer: NULL is rejected when the size is non-zero */
static void test_dyn_buf_null_nonzero_count(struct kunit *test)
{
	struct kapi_param_spec param = {};

	init_dyn_buf_param(&param);

	KUNIT_EXPECT_FALSE(test, validate_dyn_buf(&param, NULL, 1));
	KUNIT_EXPECT_FALSE(test, validate_dyn_buf(&param, NULL, PAGE_SIZE));
}

/*
 * Some architectures (separate user address space, !MMU) accept any address
 * in access_ok(), so the rejection cases below only apply where it bounds
 * user ranges.
 */
static bool kapi_rejects_kernel_addr(const void __user *addr)
{
	return !access_ok(addr, 1);
}

static bool kapi_bounds_user_range(const void __user *base)
{
	return access_ok(base, PAGE_SIZE) && !access_ok(base, 3 * PAGE_SIZE);
}

/* Dynamic buffer: addresses outside user space are rejected when the size is non-zero */
static void test_dyn_buf_kernel_addr(struct kunit *test)
{
	struct kapi_param_spec param = {};
	const void __user *kernel_addr = (void __user *)-(unsigned long)PAGE_SIZE;
	const void __user *top = (void __user *)ULONG_MAX;

	init_dyn_buf_param(&param);

	if (!kapi_rejects_kernel_addr(kernel_addr) || !kapi_rejects_kernel_addr(top))
		kunit_skip(test, "access_ok() accepts kernel addresses on this architecture");

	KUNIT_EXPECT_FALSE(test, validate_dyn_buf(&param, kernel_addr, 1));
	KUNIT_EXPECT_FALSE(test, validate_dyn_buf(&param, top, 1));
}

/* Dynamic buffer: an unset size_multiplier means byte units */
static void test_dyn_buf_default_multiplier(struct kunit *test)
{
	struct kapi_param_spec param = {};
	const void __user *near_top = (void __user *)(TASK_SIZE_MAX - 2 * PAGE_SIZE);

	init_dyn_buf_param(&param);

	KUNIT_EXPECT_TRUE(test, validate_dyn_buf(&param, (void __user *)PAGE_SIZE, 16));
	KUNIT_EXPECT_TRUE(test, validate_dyn_buf(&param, near_top, 0));

	if (!kapi_bounds_user_range(near_top))
		kunit_skip(test, "access_ok() does not bound user ranges on this architecture");

	KUNIT_EXPECT_TRUE(test, validate_dyn_buf(&param, near_top, PAGE_SIZE));
	KUNIT_EXPECT_FALSE(test, validate_dyn_buf(&param, near_top, 3 * PAGE_SIZE));
}

/* Dynamic buffer: explicit multiplier scales the size, negative and overflowing counts fail */
static void test_dyn_buf_multiplier_and_bad_count(struct kunit *test)
{
	struct kapi_param_spec param = {};
	const void __user *near_top = (void __user *)(TASK_SIZE_MAX - 2 * PAGE_SIZE);

	init_dyn_buf_param(&param);
	param.size_multiplier = 8;

	KUNIT_EXPECT_FALSE(test, validate_dyn_buf(&param, (void __user *)PAGE_SIZE,
						  (s64)(SIZE_MAX / 8 + 1)));
	KUNIT_EXPECT_FALSE(test, validate_dyn_buf(&param, (void __user *)PAGE_SIZE, -1));

	if (!kapi_bounds_user_range(near_top))
		kunit_skip(test, "access_ok() does not bound user ranges on this architecture");

	KUNIT_EXPECT_TRUE(test, validate_dyn_buf(&param, near_top, PAGE_SIZE / 8));
	KUNIT_EXPECT_FALSE(test, validate_dyn_buf(&param, near_top, PAGE_SIZE));
}

#endif /* CONFIG_KAPI_RUNTIME_CHECKS */

static const struct kernel_api_spec kapi_test_macro_spec = {
	.name = "test_macro_spec",
	KAPI_SIGNAL_MASK_COUNT(1)
	KAPI_SIGNAL_MASK(0, "blocked", "Signals blocked while waiting")
		KAPI_SIGNAL_MASK_SIGNALS(SIGINT, SIGTERM, SIGQUIT)
	},
	KAPI_CAPABILITY_COUNT(1)
	KAPI_CAPABILITY(0, CAP_SYS_ADMIN, "CAP_SYS_ADMIN", KAPI_CAP_BYPASS_CHECK)
		KAPI_CAP_ALTERNATIVE(CAP_SYS_RESOURCE, CAP_NET_ADMIN)
	},
};

/* Signal mask and capability alternative macros fill in their counts */
static void test_signal_mask_and_cap_alternative(struct kunit *test)
{
	const struct kernel_api_spec *spec = &kapi_test_macro_spec;
	char *buf;

	KUNIT_EXPECT_EQ(test, spec->signal_mask_count, 1U);
	KUNIT_EXPECT_EQ(test, spec->signal_masks[0].signal_count, 3U);
	KUNIT_EXPECT_EQ(test, spec->signal_masks[0].signals[2], SIGQUIT);
	KUNIT_EXPECT_EQ(test, spec->capabilities[0].alternative_count, 2U);
	KUNIT_EXPECT_EQ(test, spec->capabilities[0].alternative[1], CAP_NET_ADMIN);

	buf = kunit_kzalloc(test, 8192, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, buf);
	KUNIT_ASSERT_GT(test, kapi_export_json(spec, buf, 8192), 0);
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"blocked\""));
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"alternatives\""));
}

/* Unregister non-existent spec is a no-op */
static void test_unregister_nonexistent(struct kunit *test)
{
	/* Should not crash or error */
	kapi_unregister_spec("nonexistent_spec_xyz");
}

/* Multiple specs can be registered and looked up */
static void test_multiple_specs(struct kunit *test)
{
	struct kernel_api_spec *spec1, *spec2;
	const struct kernel_api_spec *found;

	spec1 = kzalloc_obj(*spec1, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec1);
	spec2 = kzalloc_obj(*spec2, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec2);

	init_test_spec(spec1, "multi_spec_1");
	init_test_spec(spec2, "multi_spec_2");

	KUNIT_ASSERT_EQ(test, kapi_register_spec(spec1), 0);
	KUNIT_ASSERT_EQ(test, kapi_register_spec(spec2), 0);

	found = kapi_get_spec("multi_spec_1");
	KUNIT_EXPECT_PTR_EQ(test, found, (const struct kernel_api_spec *)spec1);

	found = kapi_get_spec("multi_spec_2");
	KUNIT_EXPECT_PTR_EQ(test, found, (const struct kernel_api_spec *)spec2);

	kapi_unregister_spec("multi_spec_1");
	kapi_unregister_spec("multi_spec_2");
	kfree(spec1);
	kfree(spec2);
}

/* JSON export produces valid output */
static void test_json_export(struct kunit *test)
{
	static const s64 enum_vals[] = { 3, -1 };
	static const s64 err_vals[] = { -EINVAL, -EFAULT };
	struct kernel_api_spec *spec;
	char *buf;
	int ret;

	spec = kzalloc_obj(*spec, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);

	buf = kzalloc(4096, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, buf);

	init_test_spec(spec, "test_json");
	spec->param_count = 1;
	spec->params[0].name = "arg0";
	spec->params[0].type_name = "int";
	spec->params[0].constraint_type = KAPI_CONSTRAINT_ENUM;
	spec->params[0].enum_values = enum_vals;
	spec->params[0].enum_count = ARRAY_SIZE(enum_vals);
	spec->params[0].valid_mask = 0xff;
	spec->params[0].size_param_idx = 2;
	spec->return_spec.check_type = KAPI_RETURN_ERROR_CHECK;
	spec->return_spec.error_values = err_vals;
	spec->return_spec.error_count = ARRAY_SIZE(err_vals);

	ret = kapi_export_json(spec, buf, 4096);
	KUNIT_EXPECT_GT(test, ret, 0);

	/* Verify it starts with '{' and ends with '}' */
	KUNIT_EXPECT_EQ(test, buf[0], '{');
	KUNIT_ASSERT_GT(test, ret, 1);
	/* Find last non-whitespace char */
	while (ret > 0 && (buf[ret - 1] == '\n' || buf[ret - 1] == ' '))
		ret--;
	KUNIT_EXPECT_EQ(test, buf[ret - 1], '}');

	/* Verify key fields are present */
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"name\""));
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"test_json\""));
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"parameters\""));

	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"constraint_type\": \"enum\""));
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"enum_values\": [3, -1]"));
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"valid_mask\": \"0xff\""));
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"size_param_idx\": 1"));
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"error_values\": [-22, -14]"));
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"state_transitions\""));
	KUNIT_EXPECT_NOT_NULL(test, strstr(buf, "\"struct_specs\""));

	kfree(buf);
	kfree(spec);
}

/* JSON export with NULL args returns -EINVAL */
static void test_json_export_null(struct kunit *test)
{
	struct kernel_api_spec *spec;
	char buf[64];
	int ret;

	ret = kapi_export_json(NULL, buf, sizeof(buf));
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	spec = kunit_kzalloc(test, sizeof(*spec), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);
	init_test_spec(spec, "test");

	ret = kapi_export_json(spec, NULL, 64);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	ret = kapi_export_json(spec, buf, 0);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

/* JSON export into a buffer that is too small reports -E2BIG */
static void test_json_export_small_buffer(struct kunit *test)
{
	struct kernel_api_spec *spec;
	char buf[64];
	int ret;

	spec = kunit_kzalloc(test, sizeof(*spec), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, spec);
	init_test_spec(spec, "test_small");

	ret = kapi_export_json(spec, buf, sizeof(buf));

	KUNIT_EXPECT_EQ(test, ret, -E2BIG);
}

static struct kunit_case kapi_test_cases[] = {
	KUNIT_CASE(test_register_valid),
	KUNIT_CASE(test_lookup_registered),
	KUNIT_CASE(test_double_register),
	KUNIT_CASE(test_unregister),
	KUNIT_CASE(test_get_spec_null),
	KUNIT_CASE(test_register_null),
	KUNIT_CASE(test_register_null_name),
#ifdef CONFIG_KAPI_RUNTIME_CHECKS
	KUNIT_CASE(test_constraint_range_valid),
	KUNIT_CASE(test_constraint_range_invalid),
	KUNIT_CASE(test_constraint_mask_valid),
	KUNIT_CASE(test_constraint_mask_invalid),
	KUNIT_CASE(test_constraint_power_of_two),
	KUNIT_CASE(test_constraint_page_aligned),
	KUNIT_CASE(test_constraint_nonzero),
	KUNIT_CASE(test_return_validation),
	KUNIT_CASE(test_return_known_error),
	KUNIT_CASE(test_return_unknown_error),
	KUNIT_CASE(test_constraint_alignment),
	KUNIT_CASE(test_fd_int_overflow),
	KUNIT_CASE(test_constraint_enum),
	KUNIT_CASE(test_constraint_buffer),
	KUNIT_CASE(test_return_range),
	KUNIT_CASE(test_dyn_buf_zero_count),
	KUNIT_CASE(test_dyn_buf_null_nonzero_count),
	KUNIT_CASE(test_dyn_buf_kernel_addr),
	KUNIT_CASE(test_dyn_buf_default_multiplier),
	KUNIT_CASE(test_dyn_buf_multiplier_and_bad_count),
#endif
	KUNIT_CASE(test_signal_mask_and_cap_alternative),
	KUNIT_CASE(test_unregister_nonexistent),
	KUNIT_CASE(test_multiple_specs),
	KUNIT_CASE(test_json_export),
	KUNIT_CASE(test_json_export_null),
	KUNIT_CASE(test_json_export_small_buffer),
	{}
};

static struct kunit_suite kapi_test_suite = {
	.name = "kapi",
	.test_cases = kapi_test_cases,
};

kunit_test_suite(kapi_test_suite);

MODULE_DESCRIPTION("KUnit tests for Kernel API Specification Framework");
MODULE_LICENSE("GPL");
