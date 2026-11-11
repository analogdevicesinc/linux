// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the scalable bitmap (sbitmap) implementation.
 *
 * sbitmap is the tag allocator used by blk-mq and the I/O schedulers built on
 * top of it, but it has no tests.  These tests pin down the observable
 * behaviour of the plain, synchronous parts of the API.
 *
 * Copyright (C) 2026 Wang Yibo <szwyb123@gmail.com>
 */

#include <linux/bitmap.h>
#include <linux/gfp.h>
#include <linux/sbitmap.h>

#include <kunit/test.h>

/*
 * Big enough to span several words for any plausible BITS_PER_LONG, and not a
 * power of two so that the last word is partially used.
 */
#define SBITMAP_TEST_DEPTH	(3 * BITS_PER_LONG + 5)

static void sbitmap_test_init(struct kunit *test)
{
	const unsigned int depth = 97;
	struct sbitmap sb;
	int ret;

	ret = sbitmap_init_node(&sb, depth, -1, GFP_KERNEL, NUMA_NO_NODE,
				false, true);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, sb.depth, depth);
	KUNIT_EXPECT_EQ(test, sb.shift,
			(unsigned int)sbitmap_calculate_shift(depth));
	KUNIT_EXPECT_EQ(test, sb.map_nr, DIV_ROUND_UP(depth, 1U << sb.shift));
	KUNIT_EXPECT_FALSE(test, sb.round_robin);
	/*
	 * alloc_hint is a __percpu pointer, which the generic pointer
	 * assertions cannot take: on x86, where percpu data lives in
	 * __seg_gs, passing it through a const void * does not compile.
	 * Test it as a plain boolean instead.
	 */
	KUNIT_EXPECT_TRUE(test, sb.alloc_hint);
	KUNIT_EXPECT_FALSE(test, sbitmap_any_bit_set(&sb));
	KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb), 0U);
	sbitmap_free(&sb);

	/* An explicitly requested shift is honoured. */
	ret = sbitmap_init_node(&sb, depth, 3, GFP_KERNEL, NUMA_NO_NODE,
				false, true);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, sb.shift, 3U);
	KUNIT_EXPECT_EQ(test, sb.map_nr, DIV_ROUND_UP(depth, 1U << 3));
	sbitmap_free(&sb);

	/* BITS_PER_LONG bits per word is the largest representable granularity. */
	ret = sbitmap_init_node(&sb, depth, ilog2(BITS_PER_LONG), GFP_KERNEL,
				NUMA_NO_NODE, false, true);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, sb.shift, (unsigned int)ilog2(BITS_PER_LONG));
	KUNIT_EXPECT_EQ(test, sb.map_nr, DIV_ROUND_UP(depth, BITS_PER_LONG));
	sbitmap_free(&sb);

	/* Anything above that cannot be represented and must be rejected. */
	ret = sbitmap_init_node(&sb, depth, ilog2(BITS_PER_LONG) + 1,
				GFP_KERNEL, NUMA_NO_NODE, false, true);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

static void sbitmap_test_calculate_shift(struct kunit *test)
{
	/* Maps too small to spread over several words keep the default. */
	KUNIT_EXPECT_EQ(test, sbitmap_calculate_shift(0), ilog2(BITS_PER_LONG));
	KUNIT_EXPECT_EQ(test, sbitmap_calculate_shift(1), ilog2(BITS_PER_LONG));
	KUNIT_EXPECT_EQ(test, sbitmap_calculate_shift(3), ilog2(BITS_PER_LONG));

	/* Four bits map to one bit per word. */
	KUNIT_EXPECT_EQ(test, sbitmap_calculate_shift(4), 0);

	/* The shift grows back as the depth increases. */
	KUNIT_EXPECT_EQ(test, sbitmap_calculate_shift(4 * BITS_PER_LONG - 1),
			ilog2(BITS_PER_LONG) - 1);
	KUNIT_EXPECT_EQ(test, sbitmap_calculate_shift(4 * BITS_PER_LONG),
			ilog2(BITS_PER_LONG));
}

static void sbitmap_test_get_exhaust(struct kunit *test)
{
	DECLARE_BITMAP(seen, SBITMAP_TEST_DEPTH) = { 0 };
	struct sbitmap sb;
	unsigned int i;
	int bit;

	KUNIT_ASSERT_EQ(test, sbitmap_init_node(&sb, SBITMAP_TEST_DEPTH, -1,
						GFP_KERNEL, NUMA_NO_NODE,
						false, true), 0);

	/* Every bit is handed out exactly once, then the map is empty. */
	for (i = 0; i < SBITMAP_TEST_DEPTH; i++) {
		bit = sbitmap_get(&sb);
		KUNIT_ASSERT_GE(test, bit, 0);
		KUNIT_ASSERT_LT(test, bit, (int)SBITMAP_TEST_DEPTH);
		KUNIT_ASSERT_FALSE(test, test_bit(bit, seen));
		__set_bit(bit, seen);
	}

	KUNIT_EXPECT_TRUE(test, sbitmap_any_bit_set(&sb));
	KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb),
			(unsigned int)SBITMAP_TEST_DEPTH);
	KUNIT_EXPECT_EQ(test, sbitmap_get(&sb), -1);

	sbitmap_free(&sb);
}

static void sbitmap_test_put_reuse(struct kunit *test)
{
	struct sbitmap sb;
	unsigned int i;
	int bit;

	KUNIT_ASSERT_EQ(test, sbitmap_init_node(&sb, BITS_PER_LONG, -1,
						GFP_KERNEL, NUMA_NO_NODE,
						true, true), 0);

	for (i = 0; i < BITS_PER_LONG; i++)
		KUNIT_ASSERT_GE(test, sbitmap_get(&sb), 0);

	KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb), (unsigned int)BITS_PER_LONG);
	KUNIT_EXPECT_EQ(test, sbitmap_get(&sb), -1);

	/*
	 * Freeing a single bit must make exactly that bit available again, no
	 * matter which bit it was.
	 */
	for (i = 0; i < BITS_PER_LONG; i++) {
		sbitmap_put(&sb, i);
		KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb),
				(unsigned int)BITS_PER_LONG - 1);
		bit = sbitmap_get(&sb);
		KUNIT_EXPECT_EQ(test, bit, (int)i);
		KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb),
				(unsigned int)BITS_PER_LONG);
		KUNIT_EXPECT_EQ(test, sbitmap_get(&sb), -1);
	}

	sbitmap_free(&sb);
}

static void sbitmap_test_round_robin(struct kunit *test)
{
	struct sbitmap sb;
	unsigned int i;
	int bit, prev = -1;

	KUNIT_ASSERT_EQ(test, sbitmap_init_node(&sb, SBITMAP_TEST_DEPTH, -1,
						GFP_KERNEL, NUMA_NO_NODE,
						true, true), 0);

	/*
	 * round_robin must hand out strictly increasing bit numbers while the
	 * map fills up from empty.
	 */
	for (i = 0; i < SBITMAP_TEST_DEPTH; i++) {
		bit = sbitmap_get(&sb);
		KUNIT_ASSERT_GE(test, bit, 0);
		KUNIT_ASSERT_LT(test, bit, (int)SBITMAP_TEST_DEPTH);
		if (prev != -1)
			KUNIT_ASSERT_GT(test, bit, prev);
		prev = bit;
	}

	KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb),
			(unsigned int)SBITMAP_TEST_DEPTH);
	KUNIT_EXPECT_EQ(test, sbitmap_get(&sb), -1);

	sbitmap_free(&sb);
}

static void sbitmap_test_resize(struct kunit *test)
{
	const unsigned int depth = 4 * BITS_PER_LONG;
	struct sbitmap sb;
	unsigned int i;

	KUNIT_ASSERT_EQ(test, sbitmap_init_node(&sb, depth, -1, GFP_KERNEL,
						NUMA_NO_NODE, true, true), 0);

	for (i = 0; i < depth; i++)
		KUNIT_ASSERT_GE(test, sbitmap_get(&sb), 0);
	KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb), depth);

	/* Shrinking hides the bits past the new depth. */
	sbitmap_resize(&sb, depth / 2);
	KUNIT_EXPECT_EQ(test, sb.depth, depth / 2);
	KUNIT_EXPECT_EQ(test, sb.map_nr,
			DIV_ROUND_UP(depth / 2, 1U << sb.shift));
	KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb), depth / 2);
	KUNIT_EXPECT_EQ(test, sbitmap_get(&sb), -1);

	/* Growing makes them visible again; they are still allocated. */
	sbitmap_resize(&sb, depth);
	KUNIT_EXPECT_EQ(test, sb.depth, depth);
	KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb), depth);
	KUNIT_EXPECT_EQ(test, sbitmap_get(&sb), -1);

	sbitmap_free(&sb);
}

static void sbitmap_test_set_clear_bit(struct kunit *test)
{
	static const unsigned int bits[] = {
		0, 1, BITS_PER_LONG - 1, BITS_PER_LONG,
		2 * BITS_PER_LONG + 3, SBITMAP_TEST_DEPTH - 1
	};
	struct sbitmap sb;
	unsigned int i;

	KUNIT_ASSERT_EQ(test, sbitmap_init_node(&sb, SBITMAP_TEST_DEPTH, -1,
						GFP_KERNEL, NUMA_NO_NODE,
						false, true), 0);

	KUNIT_EXPECT_FALSE(test, sbitmap_any_bit_set(&sb));

	for (i = 0; i < ARRAY_SIZE(bits); i++) {
		sbitmap_set_bit(&sb, bits[i]);
		KUNIT_EXPECT_TRUE(test, sbitmap_test_bit(&sb, bits[i]));
	}
	KUNIT_EXPECT_TRUE(test, sbitmap_any_bit_set(&sb));
	KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb), (unsigned int)ARRAY_SIZE(bits));

	for (i = 0; i < ARRAY_SIZE(bits); i++) {
		sbitmap_clear_bit(&sb, bits[i]);
		KUNIT_EXPECT_FALSE(test, sbitmap_test_bit(&sb, bits[i]));
	}
	KUNIT_EXPECT_FALSE(test, sbitmap_any_bit_set(&sb));
	KUNIT_EXPECT_EQ(test, sbitmap_weight(&sb), 0U);

	sbitmap_free(&sb);
}

struct sbitmap_iter_ctx {
	struct kunit *test;
	DECLARE_BITMAP(visited, SBITMAP_TEST_DEPTH);
	unsigned int nr_visited;
};

static bool sbitmap_iter_collect(struct sbitmap *sb, unsigned int bitnr,
				 void *data)
{
	struct sbitmap_iter_ctx *ctx = data;

	KUNIT_ASSERT_LT(ctx->test, bitnr, SBITMAP_TEST_DEPTH);
	__set_bit(bitnr, ctx->visited);
	ctx->nr_visited++;

	return true;
}

static void sbitmap_test_iterate_set(struct kunit *test)
{
	static const unsigned int bits[] = {
		0, 5, BITS_PER_LONG - 1, BITS_PER_LONG, 2 * BITS_PER_LONG + 3,
		SBITMAP_TEST_DEPTH - 1
	};
	struct sbitmap_iter_ctx ctx = { .test = test };
	struct sbitmap sb;
	unsigned int i;

	KUNIT_ASSERT_EQ(test, sbitmap_init_node(&sb, SBITMAP_TEST_DEPTH, -1,
						GFP_KERNEL, NUMA_NO_NODE,
						false, true), 0);

	for (i = 0; i < ARRAY_SIZE(bits); i++) {
		KUNIT_ASSERT_LT(test, bits[i], SBITMAP_TEST_DEPTH);
		sbitmap_set_bit(&sb, bits[i]);
	}

	sbitmap_for_each_set(&sb, sbitmap_iter_collect, &ctx);

	KUNIT_EXPECT_EQ(test, ctx.nr_visited, (unsigned int)ARRAY_SIZE(bits));
	for (i = 0; i < ARRAY_SIZE(bits); i++)
		KUNIT_EXPECT_TRUE(test, test_bit(bits[i], ctx.visited));

	sbitmap_free(&sb);
}

static struct kunit_case sbitmap_test_cases[] = {
	KUNIT_CASE(sbitmap_test_init),
	KUNIT_CASE(sbitmap_test_calculate_shift),
	KUNIT_CASE(sbitmap_test_get_exhaust),
	KUNIT_CASE(sbitmap_test_put_reuse),
	KUNIT_CASE(sbitmap_test_round_robin),
	KUNIT_CASE(sbitmap_test_resize),
	KUNIT_CASE(sbitmap_test_set_clear_bit),
	KUNIT_CASE(sbitmap_test_iterate_set),
	{}
};

static struct kunit_suite sbitmap_test_suite = {
	.name = "sbitmap",
	.test_cases = sbitmap_test_cases,
};

kunit_test_suite(sbitmap_test_suite);

MODULE_DESCRIPTION("KUnit test for sbitmap");
MODULE_LICENSE("GPL");
