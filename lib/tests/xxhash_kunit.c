// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the xxHash implementation in lib/xxhash.c
 *
 * The expected values come from the reference xxHash implementation. The
 * input is generated the same way as in xxHash's own sanity check, so the
 * entries for lengths 1, 14 and 222 match the values published there.
 */

#include <kunit/test.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/xxhash.h>

#define XXH_TEST_LEN	222

static const u32 xxh_prime32 = 2654435761U;
static const u64 xxh_prime64 = 11400714785074694797ULL;

struct xxh32_case {
	unsigned int len;
	u32 seed;
	u32 hash;
};

struct xxh64_case {
	unsigned int len;
	u64 seed;
	u64 hash;
};

static const struct xxh32_case xxh32_cases[] = {
	{   0, 0x00000000, 0x02cc5d05 },
	{   1, 0x00000000, 0xcf65b03e },
	{   3, 0x00000000, 0xc23884f5 },
	{   4, 0x00000000, 0xa9de7ce9 },
	{   5, 0x00000000, 0xeb1734bb },
	{   8, 0x00000000, 0xa3f6f44b },
	{  14, 0x00000000, 0x1208e7e2 },
	{  15, 0x00000000, 0x6b859e14 },
	{  16, 0x00000000, 0x93ba3759 },
	{  17, 0x00000000, 0x89fdc23e },
	{  31, 0x00000000, 0x5f40e562 },
	{  32, 0x00000000, 0xd89829ec },
	{  33, 0x00000000, 0x31a427e5 },
	{  63, 0x00000000, 0xf1d48fdb },
	{  64, 0x00000000, 0x02e95dbb },
	{  65, 0x00000000, 0x16992b3d },
	{ 100, 0x00000000, 0x96ad8143 },
	{ 222, 0x00000000, 0x5bd11dbd },
	{   0, 0x9e3779b1, 0x36b78ae7 },
	{   1, 0x9e3779b1, 0xb4545aa4 },
	{   3, 0x9e3779b1, 0x1a269947 },
	{   4, 0x9e3779b1, 0x2baafe83 },
	{   5, 0x9e3779b1, 0x5874dab0 },
	{   8, 0x9e3779b1, 0xc2a8e239 },
	{  14, 0x9e3779b1, 0x6af1d1fe },
	{  15, 0x9e3779b1, 0xad53090d },
	{  16, 0x9e3779b1, 0xa94fc1e1 },
	{  17, 0x9e3779b1, 0xc9910739 },
	{  31, 0x9e3779b1, 0x5c0c3350 },
	{  32, 0x9e3779b1, 0xa5c44467 },
	{  33, 0x9e3779b1, 0x0de5b1f9 },
	{  63, 0x9e3779b1, 0x956b3d77 },
	{  64, 0x9e3779b1, 0xcf82f830 },
	{  65, 0x9e3779b1, 0x428eec5f },
	{ 100, 0x9e3779b1, 0x83d48124 },
	{ 222, 0x9e3779b1, 0x58803c5f },
};

static const struct xxh64_case xxh64_cases[] = {
	{   0, 0x0000000000000000ULL, 0xef46db3751d8e999ULL },
	{   1, 0x0000000000000000ULL, 0xe934a84adb052768ULL },
	{   3, 0x0000000000000000ULL, 0xff7e1959cb50794aULL },
	{   4, 0x0000000000000000ULL, 0x9136a0dca57457eeULL },
	{   5, 0x0000000000000000ULL, 0x9b046fb1397f09a5ULL },
	{   8, 0x0000000000000000ULL, 0xcdbcf538e71d1348ULL },
	{  14, 0x0000000000000000ULL, 0x8282dcc4994e35c8ULL },
	{  15, 0x0000000000000000ULL, 0x180719316d622d84ULL },
	{  16, 0x0000000000000000ULL, 0x98c90b57fdfcb55cULL },
	{  17, 0x0000000000000000ULL, 0x0d39a2d051a30c2cULL },
	{  31, 0x0000000000000000ULL, 0x299b39a290e6d783ULL },
	{  32, 0x0000000000000000ULL, 0x18b216492bb44b70ULL },
	{  33, 0x0000000000000000ULL, 0x55c8dc3e578f5b59ULL },
	{  63, 0x0000000000000000ULL, 0xa9efbe0fa0f3f4e7ULL },
	{  64, 0x0000000000000000ULL, 0xef558f8acac2b5cdULL },
	{  65, 0x0000000000000000ULL, 0xde0f20dc2631af7aULL },
	{ 100, 0x0000000000000000ULL, 0x4bfe019cd91d9ea4ULL },
	{ 222, 0x0000000000000000ULL, 0xb641ae8cb691c174ULL },
	{   0, 0x000000009e3779b1ULL, 0xac75fda2929b17efULL },
	{   1, 0x000000009e3779b1ULL, 0x5014607643a9b4c3ULL },
	{   3, 0x000000009e3779b1ULL, 0xaa8584e83660f7d1ULL },
	{   4, 0x000000009e3779b1ULL, 0xcaab286bd8e9fdb5ULL },
	{   5, 0x000000009e3779b1ULL, 0x2af5249930f984ecULL },
	{   8, 0x000000009e3779b1ULL, 0xfe0c047a5353cdacULL },
	{  14, 0x000000009e3779b1ULL, 0xc3bd6bf63deb6df0ULL },
	{  15, 0x000000009e3779b1ULL, 0xd61105c20e91f99fULL },
	{  16, 0x000000009e3779b1ULL, 0xc900ad2d536b607eULL },
	{  17, 0x000000009e3779b1ULL, 0x495cd68a647c7a22ULL },
	{  31, 0x000000009e3779b1ULL, 0xda673d5feb5c1d79ULL },
	{  32, 0x000000009e3779b1ULL, 0xb3f33bdf93ade409ULL },
	{  33, 0x000000009e3779b1ULL, 0xe92c292f64bc3071ULL },
	{  63, 0x000000009e3779b1ULL, 0x6c911fadb05b6fc2ULL },
	{  64, 0x000000009e3779b1ULL, 0xb5eeba99264cc44fULL },
	{  65, 0x000000009e3779b1ULL, 0xd3f6ff3941e310caULL },
	{ 100, 0x000000009e3779b1ULL, 0x4853706dc9625caeULL },
	{ 222, 0x000000009e3779b1ULL, 0x20cb8ab7ae10c14aULL },
	{   0, 0x9e3779b97f4a7c15ULL, 0xc4349fc93c010000ULL },
	{   1, 0x9e3779b97f4a7c15ULL, 0x126bb57a12364aa5ULL },
	{   3, 0x9e3779b97f4a7c15ULL, 0xe23f055f9113033dULL },
	{   4, 0x9e3779b97f4a7c15ULL, 0xf6337e5fb526b92eULL },
	{   5, 0x9e3779b97f4a7c15ULL, 0x3fe27fb9d0eb8e4cULL },
	{   8, 0x9e3779b97f4a7c15ULL, 0x4fe88e05b3a5bfd8ULL },
	{  14, 0x9e3779b97f4a7c15ULL, 0xbdd0f167d252e0bbULL },
	{  15, 0x9e3779b97f4a7c15ULL, 0xb0298e00622d8613ULL },
	{  16, 0x9e3779b97f4a7c15ULL, 0xb37a43269a32566aULL },
	{  17, 0x9e3779b97f4a7c15ULL, 0xfd7e04ed256c9413ULL },
	{  31, 0x9e3779b97f4a7c15ULL, 0xb9fa34ecbf9b57ffULL },
	{  32, 0x9e3779b97f4a7c15ULL, 0xe65ea5f4eff5d5deULL },
	{  33, 0x9e3779b97f4a7c15ULL, 0xf166a3272cadc4abULL },
	{  63, 0x9e3779b97f4a7c15ULL, 0x0a36d884436efe01ULL },
	{  64, 0x9e3779b97f4a7c15ULL, 0xe7bd5052c0d2145eULL },
	{  65, 0x9e3779b97f4a7c15ULL, 0xfea9a8f0561339ddULL },
	{ 100, 0x9e3779b97f4a7c15ULL, 0x8dd80a6ef6296bf7ULL },
	{ 222, 0x9e3779b97f4a7c15ULL, 0x8cdf13ba7f241d67ULL },
};

static u8 *xxh_test_buf(struct kunit *test)
{
	u64 gen = xxh_prime32;
	u8 *buf;
	int i;

	buf = kunit_kmalloc(test, XXH_TEST_LEN, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, buf);

	for (i = 0; i < XXH_TEST_LEN; i++) {
		buf[i] = gen >> 56;
		gen *= xxh_prime64;
	}
	return buf;
}

static void xxh32_case_desc(const struct xxh32_case *t, char *desc)
{
	snprintf(desc, KUNIT_PARAM_DESC_SIZE, "len %u seed %#x",
		 t->len, t->seed);
}

static void xxh64_case_desc(const struct xxh64_case *t, char *desc)
{
	snprintf(desc, KUNIT_PARAM_DESC_SIZE, "len %u seed %#llx",
		 t->len, t->seed);
}

KUNIT_ARRAY_PARAM(xxh32, xxh32_cases, xxh32_case_desc);
KUNIT_ARRAY_PARAM(xxh64, xxh64_cases, xxh64_case_desc);

static void xxh32_test(struct kunit *test)
{
	const struct xxh32_case *t = test->param_value;
	u8 *buf = xxh_test_buf(test);

	KUNIT_EXPECT_EQ(test, xxh32(buf, t->len, t->seed), t->hash);
}

static void xxh64_test(struct kunit *test)
{
	const struct xxh64_case *t = test->param_value;
	u8 *buf = xxh_test_buf(test);

	KUNIT_EXPECT_EQ(test, xxh64(buf, t->len, t->seed), t->hash);
}

/*
 * The streaming interface must give the same result as xxh64() however the
 * input is split. Try every split into two updates, and one byte at a time.
 */
static void xxh64_streaming_test(struct kunit *test)
{
	const struct xxh64_case *t = test->param_value;
	u8 *buf = xxh_test_buf(test);
	struct xxh64_state state;
	unsigned int split, i;

	for (split = 0; split <= t->len; split++) {
		xxh64_reset(&state, t->seed);
		KUNIT_ASSERT_EQ(test, xxh64_update(&state, buf, split), 0);
		KUNIT_ASSERT_EQ(test, xxh64_update(&state, buf + split,
						   t->len - split), 0);
		KUNIT_EXPECT_EQ_MSG(test, xxh64_digest(&state), t->hash,
				    "split at %u", split);
	}

	xxh64_reset(&state, t->seed);
	for (i = 0; i < t->len; i++)
		KUNIT_ASSERT_EQ(test, xxh64_update(&state, buf + i, 1), 0);
	KUNIT_EXPECT_EQ(test, xxh64_digest(&state), t->hash);

	/* xxh64_digest() does not change the state, so it can be repeated. */
	KUNIT_EXPECT_EQ(test, xxh64_digest(&state), t->hash);
}

static void xxh64_update_null_test(struct kunit *test)
{
	struct xxh64_state state;

	xxh64_reset(&state, 0);
	KUNIT_EXPECT_EQ(test, xxh64_update(&state, NULL, 0), -EINVAL);
	/* A rejected update leaves the state as it was. */
	KUNIT_EXPECT_EQ(test, xxh64_digest(&state), xxh64(NULL, 0, 0));
}

/* The result does not depend on the alignment of the input. */
static void xxh_unaligned_test(struct kunit *test)
{
	u8 *buf = xxh_test_buf(test);
	u32 h32 = xxh32(buf, XXH_TEST_LEN, 0);
	u64 h64 = xxh64(buf, XXH_TEST_LEN, 0);
	u8 *copy;
	int off;

	copy = kunit_kmalloc(test, XXH_TEST_LEN + 8, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, copy);

	for (off = 1; off < 8; off++) {
		struct xxh64_state state;

		memcpy(copy + off, buf, XXH_TEST_LEN);
		KUNIT_EXPECT_EQ_MSG(test, xxh32(copy + off, XXH_TEST_LEN, 0),
				    h32, "offset %d", off);
		KUNIT_EXPECT_EQ_MSG(test, xxh64(copy + off, XXH_TEST_LEN, 0),
				    h64, "offset %d", off);

		xxh64_reset(&state, 0);
		xxh64_update(&state, copy + off, 5);
		xxh64_update(&state, copy + off + 5, XXH_TEST_LEN - 5);
		KUNIT_EXPECT_EQ_MSG(test, xxh64_digest(&state), h64,
				    "offset %d", off);
	}
}

static struct kunit_case xxhash_test_cases[] = {
	KUNIT_CASE_PARAM(xxh32_test, xxh32_gen_params),
	KUNIT_CASE_PARAM(xxh64_test, xxh64_gen_params),
	KUNIT_CASE_PARAM(xxh64_streaming_test, xxh64_gen_params),
	KUNIT_CASE(xxh64_update_null_test),
	KUNIT_CASE(xxh_unaligned_test),
	{}
};

static struct kunit_suite xxhash_test_suite = {
	.name = "xxhash",
	.test_cases = xxhash_test_cases,
};

kunit_test_suite(xxhash_test_suite);

MODULE_DESCRIPTION("KUnit tests for xxHash");
MODULE_LICENSE("GPL");
