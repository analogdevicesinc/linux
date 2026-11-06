// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the UCS-2 string helpers in lib/ucs2_string.c
 */

#include <kunit/test.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/ucs2_string.h>

/* "abc" */
static const ucs2_char_t abc[] = { 'a', 'b', 'c', 0 };
/* "a", U+00E9 (e acute), U+20AC (euro sign) */
static const ucs2_char_t mixed[] = { 'a', 0x00e9, 0x20ac, 0 };
static const ucs2_char_t empty[] = { 0 };

static void ucs2_strnlen_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ucs2_strnlen(empty, 10), 0);
	KUNIT_EXPECT_EQ(test, ucs2_strnlen(abc, 10), 3);
	KUNIT_EXPECT_EQ(test, ucs2_strnlen(abc, 3), 3);
	KUNIT_EXPECT_EQ(test, ucs2_strnlen(abc, 2), 2);
	KUNIT_EXPECT_EQ(test, ucs2_strnlen(abc, 0), 0);
	KUNIT_EXPECT_EQ(test, ucs2_strnlen(mixed, 10), 3);
}

static void ucs2_strlen_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ucs2_strlen(empty), 0);
	KUNIT_EXPECT_EQ(test, ucs2_strlen(abc), 3);
	KUNIT_EXPECT_EQ(test, ucs2_strlen(mixed), 3);
}

static void ucs2_strsize_test(struct kunit *test)
{
	/* The result is in bytes, and @maxlength is in bytes too. */
	KUNIT_EXPECT_EQ(test, ucs2_strsize(abc, 100), 6);
	KUNIT_EXPECT_EQ(test, ucs2_strsize(abc, 4), 4);
	/* An odd byte limit is rounded down to whole characters. */
	KUNIT_EXPECT_EQ(test, ucs2_strsize(abc, 5), 4);
	KUNIT_EXPECT_EQ(test, ucs2_strsize(abc, 1), 0);
	KUNIT_EXPECT_EQ(test, ucs2_strsize(empty, 100), 0);
}

static void ucs2_strscpy_test(struct kunit *test)
{
	ucs2_char_t dst[8];

	/* Plenty of room. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_strscpy(dst, abc, ARRAY_SIZE(dst)), 3);
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(dst, abc, ARRAY_SIZE(dst)), 0);
	/* Nothing is written past the terminator. */
	KUNIT_EXPECT_EQ(test, dst[4], 0xffff);

	/* Exact fit: three characters plus the terminator. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_strscpy(dst, abc, 4), 3);
	KUNIT_EXPECT_EQ(test, dst[3], 0);

	/* One short: truncated, still terminated, -E2BIG. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_strscpy(dst, abc, 3), -E2BIG);
	KUNIT_EXPECT_EQ(test, dst[0], 'a');
	KUNIT_EXPECT_EQ(test, dst[1], 'b');
	KUNIT_EXPECT_EQ(test, dst[2], 0);
	KUNIT_EXPECT_EQ(test, dst[3], 0xffff);

	/* Room for the terminator only. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_strscpy(dst, abc, 1), -E2BIG);
	KUNIT_EXPECT_EQ(test, dst[0], 0);
	KUNIT_EXPECT_EQ(test, dst[1], 0xffff);

	/* A zero-sized buffer is rejected and left untouched. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_strscpy(dst, abc, 0), -E2BIG);
	KUNIT_EXPECT_EQ(test, dst[0], 0xffff);

	/* So is a size that does not fit in the ssize_t return value. */
	KUNIT_EXPECT_EQ(test, ucs2_strscpy(dst, abc, (size_t)SSIZE_MAX + 1),
			-E2BIG);
	KUNIT_EXPECT_EQ(test, dst[0], 0xffff);

	/* Empty source. */
	KUNIT_EXPECT_EQ(test, ucs2_strscpy(dst, empty, ARRAY_SIZE(dst)), 0);
	KUNIT_EXPECT_EQ(test, dst[0], 0);
}

static void ucs2_strncmp_test(struct kunit *test)
{
	static const ucs2_char_t abd[] = { 'a', 'b', 'd', 0 };
	static const ucs2_char_t ab[] = { 'a', 'b', 0 };
	static const ucs2_char_t lo[] = { 0x0001, 0 };
	static const ucs2_char_t hi[] = { 0xffff, 0 };

	KUNIT_EXPECT_EQ(test, ucs2_strncmp(abc, abc, 10), 0);
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(abc, abd, 10), -1);
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(abd, abc, 10), 1);
	/* Only the first @len characters are compared. */
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(abc, abd, 2), 0);
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(abc, abd, 0), 0);
	/* A prefix sorts first, because its terminator is the smaller value. */
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(ab, abc, 10), -1);
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(abc, ab, 10), 1);
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(empty, empty, 10), 0);
	/* Characters compare as unsigned 16-bit values. */
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(lo, hi, 10), -1);
	KUNIT_EXPECT_EQ(test, ucs2_strncmp(hi, lo, 10), 1);
}

/* One UCS-2 character and its expected UTF-8 encoding. */
struct ucs2_utf8_case {
	ucs2_char_t c;
	u8 utf8[3];
	unsigned int len;
};

static const struct ucs2_utf8_case utf8_cases[] = {
	{ 0x0001, { 0x01 }, 1 },
	{ 0x0041, { 0x41 }, 1 },
	{ 0x007f, { 0x7f }, 1 },
	{ 0x0080, { 0xc2, 0x80 }, 2 },
	{ 0x00e9, { 0xc3, 0xa9 }, 2 },
	{ 0x07ff, { 0xdf, 0xbf }, 2 },
	{ 0x0800, { 0xe0, 0xa0, 0x80 }, 3 },
	{ 0x20ac, { 0xe2, 0x82, 0xac }, 3 },
	{ 0xfffd, { 0xef, 0xbf, 0xbd }, 3 },
	{ 0xffff, { 0xef, 0xbf, 0xbf }, 3 },
};

static void ucs2_utf8_case_desc(const struct ucs2_utf8_case *t, char *desc)
{
	snprintf(desc, KUNIT_PARAM_DESC_SIZE, "U+%04X", t->c);
}

KUNIT_ARRAY_PARAM(ucs2_utf8, utf8_cases, ucs2_utf8_case_desc);

static void ucs2_as_utf8_char_test(struct kunit *test)
{
	const struct ucs2_utf8_case *t = test->param_value;
	const ucs2_char_t src[] = { t->c, 0 };
	u8 dst[8];

	KUNIT_EXPECT_EQ(test, ucs2_utf8size(src), t->len);

	memset(dst, 0xff, sizeof(dst));
	KUNIT_ASSERT_EQ(test, ucs2_as_utf8(dst, src, sizeof(dst)), t->len);
	KUNIT_EXPECT_MEMEQ(test, dst, t->utf8, t->len);
	KUNIT_EXPECT_EQ(test, dst[t->len], 0);
}

static void ucs2_utf8size_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ucs2_utf8size(empty), 0);
	KUNIT_EXPECT_EQ(test, ucs2_utf8size(abc), 3);
	KUNIT_EXPECT_EQ(test, ucs2_utf8size(mixed), 1 + 2 + 3);
}

static void ucs2_as_utf8_string_test(struct kunit *test)
{
	static const u8 mixed_utf8[] = { 'a', 0xc3, 0xa9, 0xe2, 0x82, 0xac };
	u8 dst[16];

	memset(dst, 0xff, sizeof(dst));
	KUNIT_ASSERT_EQ(test, ucs2_as_utf8(dst, mixed, sizeof(dst)),
			sizeof(mixed_utf8));
	KUNIT_EXPECT_MEMEQ(test, dst, mixed_utf8, sizeof(mixed_utf8));
	KUNIT_EXPECT_EQ(test, dst[sizeof(mixed_utf8)], 0);

	/* The output length matches what ucs2_utf8size() predicts. */
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8(dst, mixed, sizeof(dst)),
			ucs2_utf8size(mixed));

	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8(dst, empty, sizeof(dst)), 0);
	KUNIT_EXPECT_EQ(test, dst[0], 0);
}

static void ucs2_as_utf8_truncate_test(struct kunit *test)
{
	u8 dst[16];

	/*
	 * Output that exactly fills @maxlength is not NUL terminated, and
	 * nothing is written past it.
	 */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8(dst, abc, 3), 3);
	KUNIT_EXPECT_MEMEQ(test, dst, "abc", 3);
	KUNIT_EXPECT_EQ(test, dst[3], 0xff);

	/* A shorter buffer truncates, with no terminator. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8(dst, abc, 2), 2);
	KUNIT_EXPECT_MEMEQ(test, dst, "ab", 2);
	KUNIT_EXPECT_EQ(test, dst[2], 0xff);

	/*
	 * A multi-byte character is never split: "a" + U+00E9 + U+20AC needs
	 * 6 bytes; with 4 the euro sign is dropped whole and the spare byte
	 * holds the terminator.
	 */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8(dst, mixed, 4), 3);
	KUNIT_EXPECT_MEMEQ(test, dst, "a\xc3\xa9", 3);
	KUNIT_EXPECT_EQ(test, dst[3], 0);
	KUNIT_EXPECT_EQ(test, dst[4], 0xff);

	/* Likewise when two bytes are left for the three-byte euro sign. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8(dst, mixed, 5), 3);
	KUNIT_EXPECT_MEMEQ(test, dst, "a\xc3\xa9", 3);
	KUNIT_EXPECT_EQ(test, dst[3], 0);
	KUNIT_EXPECT_EQ(test, dst[4], 0xff);

	/* No room for even the first two-byte character. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8(dst, &mixed[1], 1), 0);
	KUNIT_EXPECT_EQ(test, dst[0], 0);
	KUNIT_EXPECT_EQ(test, dst[1], 0xff);

	/* A zero-sized buffer is not written at all. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8(dst, abc, 0), 0);
	KUNIT_EXPECT_EQ(test, dst[0], 0xff);
}

static void ucs2_as_utf8_l_test(struct kunit *test)
{
	/* Deliberately not NUL terminated. */
	static const ucs2_char_t unterminated[] = { 'x', 0x00e9, 'y' };
	static const u8 expected[] = { 'x', 0xc3, 0xa9, 'y' };
	u8 dst[16];

	/* @limit bounds the source, so it may lack a terminator. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8_l(dst, unterminated,
					     ARRAY_SIZE(unterminated),
					     sizeof(dst)), sizeof(expected));
	KUNIT_EXPECT_MEMEQ(test, dst, expected, sizeof(expected));
	KUNIT_EXPECT_EQ(test, dst[sizeof(expected)], 0);

	/* A smaller @limit stops early, on a character boundary. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8_l(dst, unterminated, 2,
					     sizeof(dst)), 3);
	KUNIT_EXPECT_MEMEQ(test, dst, expected, 3);
	KUNIT_EXPECT_EQ(test, dst[3], 0);

	/* @maxlength still applies when @limit allows more. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8_l(dst, unterminated,
					     ARRAY_SIZE(unterminated), 2), 1);
	KUNIT_EXPECT_EQ(test, dst[0], 'x');
	KUNIT_EXPECT_EQ(test, dst[1], 0);
	KUNIT_EXPECT_EQ(test, dst[2], 0xff);

	/* A zero @limit converts nothing but still terminates. */
	memset(dst, 0xff, sizeof(dst));
	KUNIT_EXPECT_EQ(test, ucs2_as_utf8_l(dst, unterminated, 0,
					     sizeof(dst)), 0);
	KUNIT_EXPECT_EQ(test, dst[0], 0);
}

static struct kunit_case ucs2_string_test_cases[] = {
	KUNIT_CASE(ucs2_strnlen_test),
	KUNIT_CASE(ucs2_strlen_test),
	KUNIT_CASE(ucs2_strsize_test),
	KUNIT_CASE(ucs2_strscpy_test),
	KUNIT_CASE(ucs2_strncmp_test),
	KUNIT_CASE(ucs2_utf8size_test),
	KUNIT_CASE_PARAM(ucs2_as_utf8_char_test, ucs2_utf8_gen_params),
	KUNIT_CASE(ucs2_as_utf8_string_test),
	KUNIT_CASE(ucs2_as_utf8_truncate_test),
	KUNIT_CASE(ucs2_as_utf8_l_test),
	{}
};

static struct kunit_suite ucs2_string_test_suite = {
	.name = "ucs2_string",
	.test_cases = ucs2_string_test_cases,
};

kunit_test_suite(ucs2_string_test_suite);

MODULE_DESCRIPTION("KUnit tests for the UCS-2 string helpers");
MODULE_LICENSE("GPL");
