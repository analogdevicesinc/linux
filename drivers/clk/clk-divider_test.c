// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for clk divider
 */
#include <kunit/clk.h>
#include <kunit/test.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/units.h>

#define PARENT_RATE_1GHZ	GIGA
#define PARENT_RATE_2GHZ	(2 * GIGA)
#define PARENT_RATE_4GHZ	(4 * GIGA)
#define PARENT_RATE_6GHZ	(6 * GIGA)
#define PARENT_RATE_10GHZ	(10 * GIGA)

KUNIT_DEFINE_ACTION_WRAPPER(clk_hw_unregister_fixed_rate_wrapper,
			    clk_hw_unregister_fixed_rate, struct clk_hw *);
KUNIT_DEFINE_ACTION_WRAPPER(clk_hw_unregister_divider_wrapper,
			    clk_hw_unregister_divider, struct clk_hw *);
KUNIT_DEFINE_ACTION_WRAPPER(clk_hw_unregister_mux_wrapper,
			    clk_hw_unregister_mux, struct clk_hw *);

static const struct clk_div_table bestdiv_table[] = {
	{ .val = 0, .div = 2 },
	{ .val = 1, .div = 4 },
	{ .val = 2, .div = 8 },
	{ /* sentinel */ }
};

/*
 * Test that clk_round_rate(clk, ULONG_MAX) returns the maximum achievable
 * rate for a divider clock.
 */
static void clk_divider_bestdiv_ulong_max_returns_max_rate(struct kunit *test)
{
	struct clk_hw *parent_hw, *div_hw;
	unsigned long rate;
	u32 *fake_reg;

	fake_reg = kunit_kzalloc(test, sizeof(*fake_reg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg);

	parent_hw = clk_hw_register_fixed_rate(NULL, "bestdiv-parent",
					       NULL, 0, PARENT_RATE_1GHZ);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, parent_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_fixed_rate_wrapper,
						  parent_hw));

	div_hw = clk_hw_register_divider_table(NULL, "bestdiv-div",
					       "bestdiv-parent",
					       CLK_SET_RATE_PARENT,
					       (void __iomem __force *)fake_reg,
					       0, 2, 0, bestdiv_table, NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, div_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_divider_wrapper,
						  div_hw));

	/*
	 * ULONG_MAX is the canonical way to probe the maximum rate a clock
	 * can produce.
	 */
	rate = clk_hw_round_rate(div_hw, ULONG_MAX);
	KUNIT_EXPECT_EQ(test, rate, PARENT_RATE_1GHZ / 2);
}

/*
 * Test that clk_round_rate(clk, ULONG_MAX) returns the correct maximum rate
 * when a mux clock sits between a divider and its parent candidates.
 *
 * Topology:
 *
 *   [fixed 4 GHz] --\
 *                    +--> [mux CLK_SET_RATE_PARENT] --> [div {2,4,8} CLK_SET_RATE_PARENT]
 *   [fixed 2 GHz] --/
 *
 */
static void clk_divider_bestdiv_mux_ulong_max_returns_max_rate(struct kunit *test)
{
	static const char * const mux_parents[] = {
		"bestdiv-mux-parent-a",
		"bestdiv-mux-parent-b",
	};
	struct clk_hw *parent_a_hw, *parent_b_hw, *mux_hw, *div_hw;
	u32 *fake_reg_mux, *fake_reg_div;
	unsigned long rate;

	fake_reg_mux = kunit_kzalloc(test, sizeof(*fake_reg_mux), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg_mux);

	fake_reg_div = kunit_kzalloc(test, sizeof(*fake_reg_div), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg_div);

	/* Higher-rate parent: the mux should select this for ULONG_MAX. */
	parent_a_hw = clk_hw_register_fixed_rate(NULL, "bestdiv-mux-parent-a",
						 NULL, 0, PARENT_RATE_4GHZ);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, parent_a_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_fixed_rate_wrapper,
						  parent_a_hw));

	/* Lower-rate parent: should not be selected. */
	parent_b_hw = clk_hw_register_fixed_rate(NULL, "bestdiv-mux-parent-b",
						 NULL, 0, PARENT_RATE_2GHZ);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, parent_b_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_fixed_rate_wrapper,
						  parent_b_hw));

	/*
	 * 1-bit mux register selects between the two parents.
	 * CLK_SET_RATE_PARENT allows the divider's rate request to
	 * propagate into clk_mux_determine_rate().
	 */
	mux_hw = clk_hw_register_mux(NULL, "bestdiv-mux",
				     mux_parents, ARRAY_SIZE(mux_parents),
				     CLK_SET_RATE_PARENT,
				     (void __iomem __force *)fake_reg_mux,
				     0, 1, 0, NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, mux_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_mux_wrapper,
						  mux_hw));

	div_hw = clk_hw_register_divider_table(NULL, "bestdiv-mux-div",
					       "bestdiv-mux",
					       CLK_SET_RATE_PARENT,
					       (void __iomem __force *)fake_reg_div,
					       0, 2, 0, bestdiv_table, NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, div_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_divider_wrapper,
						  div_hw));

	rate = clk_hw_round_rate(div_hw, ULONG_MAX);
	KUNIT_EXPECT_EQ(test, rate, PARENT_RATE_4GHZ / 2);
}

static struct kunit_case clk_divider_bestdiv_test_cases[] = {
	KUNIT_CASE(clk_divider_bestdiv_ulong_max_returns_max_rate),
	KUNIT_CASE(clk_divider_bestdiv_mux_ulong_max_returns_max_rate),
	{}
};

static struct kunit_suite clk_divider_bestdiv_test_suite = {
	.name = "clk_divider_bestdiv",
	.test_cases = clk_divider_bestdiv_test_cases,
};

/*
 * Register a fixed-rate parent at @parent_rate and a 3-bit divider (/1 to
 * /8) using @fake_reg as its register, and return a consumer clk for the
 * divider.
 */
static struct clk *clk_divider_large_rate_init(struct kunit *test,
					       u32 *fake_reg,
					       unsigned long parent_rate,
					       unsigned long flags,
					       u8 div_flags)
{
	struct clk_hw *parent_hw, *div_hw;
	struct clk *clk;

	parent_hw = clk_hw_register_fixed_rate(NULL, "large-rate-parent",
					       NULL, 0, parent_rate);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, parent_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_fixed_rate_wrapper,
						  parent_hw));

	div_hw = clk_hw_register_divider(NULL, "large-rate-div",
					 "large-rate-parent", flags,
					 (void __iomem __force *)fake_reg,
					 0, 3, div_flags, NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, div_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_divider_wrapper,
						  div_hw));

	clk = clk_hw_get_clk_kunit(test, div_hw, NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, clk);

	return clk;
}

/*
 * Test that rounding a rate above 4.29 GHz computes the divisor from the
 * full rate: the parent rate needs /1.
 */
static void clk_divider_large_rate_round_up(struct kunit *test)
{
	struct clk *clk;
	u32 *fake_reg;

	if (BITS_PER_LONG < 64)
		kunit_skip(test, "rates above 4.29 GHz need a 64-bit unsigned long");

	fake_reg = kunit_kzalloc(test, sizeof(*fake_reg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg);

	clk = clk_divider_large_rate_init(test, fake_reg, PARENT_RATE_6GHZ, 0, 0);

	KUNIT_EXPECT_EQ(test, clk_round_rate(clk, PARENT_RATE_6GHZ),
			PARENT_RATE_6GHZ);
}

/*
 * Same with CLK_DIVIDER_ROUND_CLOSEST: 4.4 GHz is closer to 3 GHz (/2) than
 * to 6 GHz (/1).
 */
static void clk_divider_large_rate_round_closest(struct kunit *test)
{
	struct clk *clk;
	u32 *fake_reg;

	if (BITS_PER_LONG < 64)
		kunit_skip(test, "rates above 4.29 GHz need a 64-bit unsigned long");

	fake_reg = kunit_kzalloc(test, sizeof(*fake_reg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg);

	clk = clk_divider_large_rate_init(test, fake_reg, PARENT_RATE_6GHZ,
					  0, CLK_DIVIDER_ROUND_CLOSEST);

	KUNIT_EXPECT_EQ(test, clk_round_rate(clk, 4400 * MEGA),
			PARENT_RATE_6GHZ / 2);
}

/*
 * Test that setting a rate above 4.29 GHz programs the right divisor.
 * With CLK_SET_RATE_PARENT the rounding searches the parent rates and
 * finds /1, so this only checks the set_rate path.
 */
static void clk_divider_large_rate_set_rate(struct kunit *test)
{
	struct clk *clk;
	u32 *fake_reg;

	if (BITS_PER_LONG < 64)
		kunit_skip(test, "rates above 4.29 GHz need a 64-bit unsigned long");

	fake_reg = kunit_kzalloc(test, sizeof(*fake_reg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg);

	/* Start from /2 */
	writel(1, (void __iomem __force *)fake_reg);

	clk = clk_divider_large_rate_init(test, fake_reg, PARENT_RATE_6GHZ,
					  CLK_SET_RATE_PARENT, 0);

	KUNIT_ASSERT_EQ(test, clk_set_rate(clk, PARENT_RATE_6GHZ), 0);
	KUNIT_EXPECT_EQ(test, clk_get_rate(clk), PARENT_RATE_6GHZ);
	KUNIT_EXPECT_EQ(test, *fake_reg, 0);
}

/*
 * Round ULONG_MAX, the usual way to ask for the maximum rate, on a divider
 * with a @parent_rate parent: it must return the parent rate.
 */
static void clk_divider_large_rate_check_max(struct kunit *test,
					     unsigned long parent_rate,
					     u8 div_flags)
{
	struct clk *clk;
	u32 *fake_reg;

	fake_reg = kunit_kzalloc(test, sizeof(*fake_reg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg);

	clk = clk_divider_large_rate_init(test, fake_reg, parent_rate, 0,
					  div_flags);

	KUNIT_EXPECT_EQ(test, clk_round_rate(clk, ULONG_MAX), parent_rate);
}

static void clk_divider_large_rate_round_max(struct kunit *test)
{
	if (BITS_PER_LONG < 64)
		kunit_skip(test, "rates above 4.29 GHz need a 64-bit unsigned long");

	clk_divider_large_rate_check_max(test, PARENT_RATE_6GHZ, 0);
}

/* Same with a parent more than twice the 32-bit limit */
static void clk_divider_large_rate_round_max_10ghz(struct kunit *test)
{
	if (BITS_PER_LONG < 64)
		kunit_skip(test, "rates above 4.29 GHz need a 64-bit unsigned long");

	clk_divider_large_rate_check_max(test, PARENT_RATE_10GHZ, 0);
}

/* Same with CLK_DIVIDER_ROUND_CLOSEST */
static void clk_divider_large_rate_round_max_closest(struct kunit *test)
{
	if (BITS_PER_LONG < 64)
		kunit_skip(test, "rates above 4.29 GHz need a 64-bit unsigned long");

	clk_divider_large_rate_check_max(test, PARENT_RATE_6GHZ,
					 CLK_DIVIDER_ROUND_CLOSEST);
}

/* Same with CLK_DIVIDER_POWER_OF_TWO and a 1 GHz parent */
static void clk_divider_large_rate_round_max_pow2(struct kunit *test)
{
	clk_divider_large_rate_check_max(test, PARENT_RATE_1GHZ,
					 CLK_DIVIDER_POWER_OF_TWO);
}

static struct kunit_case clk_divider_large_rate_test_cases[] = {
	KUNIT_CASE(clk_divider_large_rate_round_up),
	KUNIT_CASE(clk_divider_large_rate_round_closest),
	KUNIT_CASE(clk_divider_large_rate_set_rate),
	KUNIT_CASE(clk_divider_large_rate_round_max),
	KUNIT_CASE(clk_divider_large_rate_round_max_10ghz),
	KUNIT_CASE(clk_divider_large_rate_round_max_closest),
	KUNIT_CASE(clk_divider_large_rate_round_max_pow2),
	{}
};

static struct kunit_suite clk_divider_large_rate_test_suite = {
	.name = "clk_divider_large_rate",
	.test_cases = clk_divider_large_rate_test_cases,
};

/*
 * Register a 1 GHz fixed-rate parent and a round-closest 3-bit divider using
 * @fake_reg as its register and @table, if not NULL, as its divider table,
 * and return a consumer clk for the divider.
 */
static struct clk *clk_divider_above_parent_init(struct kunit *test,
						 u32 *fake_reg,
						 const struct clk_div_table *table)
{
	struct clk_hw *parent_hw, *div_hw;
	struct clk *clk;

	parent_hw = clk_hw_register_fixed_rate(NULL, "above-parent-parent",
					       NULL, 0, PARENT_RATE_1GHZ);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, parent_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_fixed_rate_wrapper,
						  parent_hw));

	div_hw = clk_hw_register_divider_table(NULL, "above-parent-div",
					       "above-parent-parent", 0,
					       (void __iomem __force *)fake_reg,
					       0, 3, CLK_DIVIDER_ROUND_CLOSEST,
					       table, NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, div_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_divider_wrapper,
						  div_hw));

	clk = clk_hw_get_clk_kunit(test, div_hw, NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, clk);

	return clk;
}

/*
 * Test that a round-closest divider asked for more than its parent rate
 * picks its smallest divider: /1.
 */
static void clk_divider_above_parent_round_closest(struct kunit *test)
{
	struct clk *clk;
	u32 *fake_reg;

	fake_reg = kunit_kzalloc(test, sizeof(*fake_reg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg);

	clk = clk_divider_above_parent_init(test, fake_reg, NULL);

	KUNIT_EXPECT_EQ(test, clk_round_rate(clk, PARENT_RATE_2GHZ),
			PARENT_RATE_1GHZ);
}

/* Same with a divider table, whose smallest divider is /2 */
static void clk_divider_above_parent_round_closest_table(struct kunit *test)
{
	struct clk *clk;
	u32 *fake_reg;

	fake_reg = kunit_kzalloc(test, sizeof(*fake_reg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg);

	clk = clk_divider_above_parent_init(test, fake_reg, bestdiv_table);

	KUNIT_EXPECT_EQ(test, clk_round_rate(clk, PARENT_RATE_2GHZ),
			PARENT_RATE_1GHZ / 2);
}

static struct kunit_case clk_divider_above_parent_test_cases[] = {
	KUNIT_CASE(clk_divider_above_parent_round_closest),
	KUNIT_CASE(clk_divider_above_parent_round_closest_table),
	{}
};

static struct kunit_suite clk_divider_above_parent_test_suite = {
	.name = "clk_divider_above_parent",
	.test_cases = clk_divider_above_parent_test_cases,
};

static const struct clk_div_table large_div_table[] = {
	{ .val = 0, .div = 1U << 31 },
	{ .val = 1, .div = UINT_MAX },
	{ }
};

struct clk_divider_large_div_param {
	const char *name;
	u64 parent_rate;
	unsigned long rate;
	unsigned long flags;
	u8 div_flags;
	const struct clk_div_table *table;
	unsigned int expected_div;
};

static const struct clk_divider_large_div_param clk_divider_large_div_params[] = {
	{
		.name = "above_int_max_pow2",
		.parent_rate = 3 * GIGA,
		.rate = 1,
		.div_flags = CLK_DIVIDER_POWER_OF_TWO,
		.expected_div = 128,
	}, {
		.name = "above_uint_max",
		.parent_rate = (1ULL << 32) + 1,
		.rate = 1,
		.expected_div = 8,
	}, {
		.name = "above_uint_max_closest",
		.parent_rate = (1ULL << 32) + 1,
		.rate = 1,
		.div_flags = CLK_DIVIDER_ROUND_CLOSEST,
		.expected_div = 8,
	}, {
		.name = "near_ulong_max_pow2",
		.parent_rate = ULONG_MAX - 127,
		.rate = 1,
		.div_flags = CLK_DIVIDER_POWER_OF_TWO,
		.expected_div = 128,
	}, {
		.name = "large_table_div",
		.parent_rate = 1ULL << 33,
		.rate = 4,
		.table = large_div_table,
		.expected_div = 1U << 31,
	}, {
		.name = "large_table_div_closest",
		.parent_rate = 1ULL << 33,
		.rate = 4,
		.div_flags = CLK_DIVIDER_ROUND_CLOSEST,
		.table = large_div_table,
		.expected_div = 1U << 31,
	}, {
		.name = "last_table_div_set_rate_parent",
		.parent_rate = 3 * GIGA,
		.rate = 1,
		.flags = CLK_SET_RATE_PARENT,
		.table = large_div_table,
		.expected_div = UINT_MAX,
	},
};

KUNIT_ARRAY_PARAM_DESC(clk_divider_large_div, clk_divider_large_div_params, name);

static void clk_divider_large_div_round_rate(struct kunit *test)
{
	const struct clk_divider_large_div_param *param = test->param_value;
	struct clk_hw *parent_hw, *div_hw;
	struct clk *clk;
	u32 *fake_reg;

	if (param->parent_rate > ULONG_MAX)
		kunit_skip(test, "parent rate needs a 64-bit unsigned long");

	fake_reg = kunit_kzalloc(test, sizeof(*fake_reg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fake_reg);

	parent_hw = clk_hw_register_fixed_rate(NULL, "large-div-parent", NULL,
					       0, param->parent_rate);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, parent_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_fixed_rate_wrapper,
						  parent_hw));

	div_hw = clk_hw_register_divider_table(NULL, "large-div", "large-div-parent",
					       param->flags,
					       (void __iomem __force *)fake_reg,
					       0, 3, param->div_flags, param->table,
					       NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, div_hw);
	KUNIT_ASSERT_EQ(test, 0,
			kunit_add_action_or_reset(test, clk_hw_unregister_divider_wrapper,
						  div_hw));
	clk = clk_hw_get_clk_kunit(test, div_hw, NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, clk);

	KUNIT_EXPECT_EQ(test, clk_round_rate(clk, param->rate),
			DIV_ROUND_UP_ULL(param->parent_rate, param->expected_div));
}

static void clk_divider_large_div_get_val(struct kunit *test)
{
	unsigned long parent_rate = (unsigned long)UINT_MAX + 3;

	if (BITS_PER_LONG < 64)
		kunit_skip(test, "divisor above UINT_MAX needs a 64-bit unsigned long");

	/* An oversized divisor must not wrap to /2 before it is encoded. */
	KUNIT_EXPECT_EQ(test, divider_get_val(1, parent_rate, NULL, 3, 0), 7);
	KUNIT_EXPECT_EQ(test, divider_get_val(1, parent_rate, bestdiv_table, 3, 0),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, divider_get_val(1, parent_rate, NULL, 3,
					      CLK_DIVIDER_POWER_OF_TWO), -EINVAL);
}

static struct kunit_case clk_divider_large_div_test_cases[] = {
	KUNIT_CASE_PARAM(clk_divider_large_div_round_rate,
			 clk_divider_large_div_gen_params),
	KUNIT_CASE(clk_divider_large_div_get_val),
	{}
};

static struct kunit_suite clk_divider_large_div_test_suite = {
	.name = "clk_divider_large_div",
	.test_cases = clk_divider_large_div_test_cases,
};

kunit_test_suites(&clk_divider_bestdiv_test_suite,
		  &clk_divider_large_rate_test_suite,
		  &clk_divider_above_parent_test_suite,
		  &clk_divider_large_div_test_suite);

MODULE_DESCRIPTION("KUnit tests for clk divider");
MODULE_LICENSE("GPL");
