// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the HDM decoder interleave encode/decode helpers
 * in drivers/cxl/cxl.h
 */
#include <kunit/test.h>
#include "cxl.h"

struct eiw_case {
	u8 eiw;
	int ways;
};

/* Encodings of the IW field in the HDM Decoder n Control Register */
static const struct eiw_case valid_eiw[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 }, { 4, 16 },
	{ 8, 3 }, { 9, 6 }, { 10, 12 },
};

static const u8 invalid_eiw[] = { 5, 6, 7, 11, 12, 13, 14, 15, 0xff };

static const unsigned int invalid_ways[] = {
	0, 5, 7, 9, 10, 11, 13, 14, 15, 17, 24, 32,
};

static void eiw_to_ways_valid(struct kunit *test)
{
	for (int i = 0; i < ARRAY_SIZE(valid_eiw); i++) {
		int ways = 0;

		KUNIT_EXPECT_EQ_MSG(test, eiw_to_ways(valid_eiw[i].eiw, &ways),
				    0, "eiw=%u", valid_eiw[i].eiw);
		KUNIT_EXPECT_EQ_MSG(test, ways, valid_eiw[i].ways, "eiw=%u",
				    valid_eiw[i].eiw);
	}
}

static void eiw_to_ways_invalid(struct kunit *test)
{
	for (int i = 0; i < ARRAY_SIZE(invalid_eiw); i++) {
		int ways;

		KUNIT_EXPECT_EQ_MSG(test, eiw_to_ways(invalid_eiw[i], &ways),
				    -EINVAL, "eiw=%u", invalid_eiw[i]);
	}
}

static void ways_to_eiw_valid(struct kunit *test)
{
	for (int i = 0; i < ARRAY_SIZE(valid_eiw); i++) {
		u8 eiw = 0xff;

		KUNIT_EXPECT_EQ_MSG(test, ways_to_eiw(valid_eiw[i].ways, &eiw),
				    0, "ways=%d", valid_eiw[i].ways);
		KUNIT_EXPECT_EQ_MSG(test, eiw, valid_eiw[i].eiw, "ways=%d",
				    valid_eiw[i].ways);
	}
}

static void ways_to_eiw_invalid(struct kunit *test)
{
	for (int i = 0; i < ARRAY_SIZE(invalid_ways); i++) {
		u8 eiw;

		KUNIT_EXPECT_EQ_MSG(test, ways_to_eiw(invalid_ways[i], &eiw),
				    -EINVAL, "ways=%u", invalid_ways[i]);
	}
}

static void eig_to_granularity_valid(struct kunit *test)
{
	for (u16 eig = 0; eig <= CXL_DECODER_MAX_ENCODED_IG; eig++) {
		int gran = 0;
		u16 back = 0xffff;

		KUNIT_EXPECT_EQ_MSG(test, eig_to_granularity(eig, &gran), 0,
				    "eig=%u", eig);
		KUNIT_EXPECT_EQ_MSG(test, gran, 256 << eig, "eig=%u", eig);

		/* Round trip back to the encoding */
		KUNIT_EXPECT_EQ_MSG(test, granularity_to_eig(gran, &back), 0,
				    "gran=%d", gran);
		KUNIT_EXPECT_EQ_MSG(test, back, eig, "gran=%d", gran);
	}
}

static void eig_to_granularity_invalid(struct kunit *test)
{
	static const u16 invalid_eig[] = { 7, 8, 15, 0xffff };

	for (int i = 0; i < ARRAY_SIZE(invalid_eig); i++) {
		int gran;

		KUNIT_EXPECT_EQ_MSG(test,
				    eig_to_granularity(invalid_eig[i], &gran),
				    -EINVAL, "eig=%u", invalid_eig[i]);
	}
}

static void granularity_to_eig_invalid(struct kunit *test)
{
	static const int invalid_gran[] = {
		0, 1, 128, 255, 257, 384, 768, SZ_32K, -256,
	};

	for (int i = 0; i < ARRAY_SIZE(invalid_gran); i++) {
		u16 eig;

		KUNIT_EXPECT_EQ_MSG(test,
				    granularity_to_eig(invalid_gran[i], &eig),
				    -EINVAL, "gran=%d", invalid_gran[i]);
	}
}

/* Decoder Count field of the CXL HDM Decoder Capability Register */
static void hdm_decoder_count(struct kunit *test)
{
	static const int expect[16] = {
		[0x0] = 1,
		[0x1] = 2, [0x2] = 4, [0x3] = 6, [0x4] = 8,
		[0x5] = 10, [0x6] = 12, [0x7] = 14, [0x8] = 16,
		[0x9] = 20, [0xA] = 24, [0xB] = 28, [0xC] = 32,
		[0xD ... 0xF] = -ENXIO,
	};

	for (u32 val = 0; val < ARRAY_SIZE(expect); val++) {
		u32 cap = FIELD_PREP(CXL_HDM_DECODER_COUNT_MASK, val);

		KUNIT_EXPECT_EQ_MSG(test, cxl_hdm_decoder_count(cap),
				    expect[val], "count field=%u", val);
	}

	/* Bits outside the count field are ignored */
	KUNIT_EXPECT_EQ(test,
			cxl_hdm_decoder_count((u32)~CXL_HDM_DECODER_COUNT_MASK),
			1);
}

static struct kunit_case cxl_hdm_encoding_cases[] = {
	KUNIT_CASE(eiw_to_ways_valid),
	KUNIT_CASE(eiw_to_ways_invalid),
	KUNIT_CASE(ways_to_eiw_valid),
	KUNIT_CASE(ways_to_eiw_invalid),
	KUNIT_CASE(eig_to_granularity_valid),
	KUNIT_CASE(eig_to_granularity_invalid),
	KUNIT_CASE(granularity_to_eig_invalid),
	KUNIT_CASE(hdm_decoder_count),
	{}
};

static struct kunit_suite cxl_hdm_encoding_suite = {
	.name = "cxl_hdm_encoding",
	.test_cases = cxl_hdm_encoding_cases,
};

kunit_test_suite(cxl_hdm_encoding_suite);

MODULE_DESCRIPTION("KUnit tests for CXL HDM decoder encoding helpers");
MODULE_LICENSE("GPL");
