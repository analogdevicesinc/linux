// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * KUnit tests and benchmark for the ChaCha stream cipher library.
 *
 * The (possibly architecture-optimized) chacha_crypt() is compared against a
 * reference built on top of chacha_block_generic(), for all lengths in a
 * range covering the multi-block SIMD code paths, random buffer alignments,
 * random block counters (including counters that wrap around) and both 20 and
 * 12 rounds. Both are checked against the RFC 7539 test vector.
 */
#include <crypto/chacha.h>
#include <kunit/test.h>
#include <linux/timekeeping.h>
#include <linux/unaligned.h>
#include "test-utils.h"

/* three 4 KiB chunks, as processed by some SIMD code, plus a partial one */
#define CHACHA_TEST_MAX_LEN	(3 * SZ_4K + 200)
#define CHACHA_TEST_MAX_OFFSET	64

static void chacha_ref(struct chacha_state *state, u8 *dst, const u8 *src,
		       size_t len, int nrounds)
{
	u8 stream[CHACHA_BLOCK_SIZE];

	while (len) {
		size_t n = min_t(size_t, len, CHACHA_BLOCK_SIZE);

		chacha_block_generic(state, stream, nrounds);
		for (size_t i = 0; i < n; i++)
			dst[i] = src[i] ^ stream[i];
		len -= n;
		src += n;
		dst += n;
	}
}

static void rand_state(struct chacha_state *state)
{
	u32 key[CHACHA_KEY_WORDS];
	u8 iv[CHACHA_IV_SIZE];

	rand_bytes((u8 *)key, sizeof(key));
	rand_bytes(iv, sizeof(iv));
	chacha_init(state, key, iv);
	/* Exercise 32-bit counter wraparound from time to time. */
	if (rand32() % 8 == 0)
		state->x[12] = 0xffffffff - rand32() % 32;
}

struct chacha_test_bufs {
	u8 *src_guarded;	/* ends right before an unmapped page */
	u8 *dst_guarded;
	u8 *ref;
	u8 *orig;
};

#define CHACHA_TEST_BUF_LEN	(CHACHA_TEST_MAX_LEN + CHACHA_TEST_MAX_OFFSET)
#define CHACHA_TEST_POISON	0xa5

static void chacha_test_one(struct kunit *test, struct chacha_test_bufs *b,
			    size_t len, int nrounds, bool split, bool at_guard)
{
	struct chacha_state st, st_ref;
	bool inplace = rand_bool();
	size_t src_off, dst_off;
	u8 *src, *dst;

	if (at_guard) {
		src_off = CHACHA_TEST_BUF_LEN - len;
		dst_off = src_off;
	} else {
		src_off = rand_offset(CHACHA_TEST_MAX_OFFSET);
		dst_off = rand_offset(CHACHA_TEST_MAX_OFFSET);
	}
	src = b->src_guarded + src_off;
	dst = inplace ? src : b->dst_guarded + dst_off;

	rand_state(&st);
	st_ref = st;

	rand_bytes(b->orig, len);
	memcpy(src, b->orig, len);
	/* detect writes outside the destination */
	if (!inplace)
		memset(b->dst_guarded, CHACHA_TEST_POISON, CHACHA_TEST_BUF_LEN);

	chacha_ref(&st_ref, b->ref, b->orig, len, nrounds);

	if (split) {
		/* Continuing the stream only works at block boundaries. */
		size_t done = 0;

		while (done < len) {
			size_t n = len - done;

			/* zero-length calls included */
			if (rand_bool()) {
				size_t part = rand_length(len - done);

				n = min(n, round_up(part, CHACHA_BLOCK_SIZE));
			}
			chacha_crypt(&st, dst + done, src + done, n, nrounds);
			done += n;
		}
	} else {
		chacha_crypt(&st, dst, src, len, nrounds);
	}

	KUNIT_ASSERT_MEMEQ_MSG(test, dst, b->ref, len,
			       "len=%zu nrounds=%d inplace=%d split=%d src_off=%zu dst_off=%zu",
			       len, nrounds, inplace, split, src_off, dst_off);
	if (!inplace) {
		KUNIT_ASSERT_MEMEQ_MSG(test, src, b->orig, len,
				       "source modified, len=%zu", len);
		KUNIT_ASSERT_NULL_MSG(test,
				      memchr_inv(b->dst_guarded,
						 CHACHA_TEST_POISON, dst_off),
				      "write before dst, len=%zu", len);
		KUNIT_ASSERT_NULL_MSG(test,
				      memchr_inv(dst + len, CHACHA_TEST_POISON,
						 CHACHA_TEST_BUF_LEN - dst_off -
						 len),
				      "write after dst, len=%zu", len);
	}
	KUNIT_ASSERT_MEMEQ_MSG(test, &st, &st_ref, sizeof(st),
			       "state mismatch, len=%zu nrounds=%d", len,
			       nrounds);
}

static void chacha_alloc_bufs(struct kunit *test, struct chacha_test_bufs *b)
{
	b->src_guarded = alloc_guarded_buf(test, CHACHA_TEST_BUF_LEN);
	b->dst_guarded = alloc_guarded_buf(test, CHACHA_TEST_BUF_LEN);
	b->ref = alloc_buf(test, CHACHA_TEST_MAX_LEN);
	b->orig = alloc_buf(test, CHACHA_TEST_MAX_LEN);
}

/*
 * Every length from 0 to 4100, with 20 and 12 rounds, ending right before an
 * unmapped page and at a random offset.
 */
static void test_chacha_all_lengths(struct kunit *test)
{
	struct chacha_test_bufs b;

	chacha_alloc_bufs(test, &b);
	for (size_t len = 0; len <= 4100; len++) {
		chacha_test_one(test, &b, len, 20, false, true);
		chacha_test_one(test, &b, len, 12, false, false);
	}
}

/* Random lengths up to the maximum, including multi-call usage. */
static void test_chacha_random(struct kunit *test)
{
	struct chacha_test_bufs b;

	chacha_alloc_bufs(test, &b);
	for (int i = 0; i < 4000; i++) {
		size_t len = rand_length(CHACHA_TEST_MAX_LEN);

		chacha_test_one(test, &b, len, rand_bool() ? 20 : 12,
				rand_bool(), rand_bool());
	}
}

/* Known answer test: RFC 7539 section 2.4.2, also anchoring the reference. */
static void test_chacha_rfc7539(struct kunit *test)
{
	static const u8 key[CHACHA_KEY_SIZE] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
	};
	static const u8 iv[CHACHA_IV_SIZE] = {
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x00, 0x00,
	};
	static const u8 pt[] =
		"Ladies and Gentlemen of the class of '99: If I could offer you "
		"only one tip for the future, sunscreen would be it.";
	static const u8 ct[sizeof(pt) - 1] = {
		0x6e, 0x2e, 0x35, 0x9a, 0x25, 0x68, 0xf9, 0x80,
		0x41, 0xba, 0x07, 0x28, 0xdd, 0x0d, 0x69, 0x81,
		0xe9, 0x7e, 0x7a, 0xec, 0x1d, 0x43, 0x60, 0xc2,
		0x0a, 0x27, 0xaf, 0xcc, 0xfd, 0x9f, 0xae, 0x0b,
		0xf9, 0x1b, 0x65, 0xc5, 0x52, 0x47, 0x33, 0xab,
		0x8f, 0x59, 0x3d, 0xab, 0xcd, 0x62, 0xb3, 0x57,
		0x16, 0x39, 0xd6, 0x24, 0xe6, 0x51, 0x52, 0xab,
		0x8f, 0x53, 0x0c, 0x35, 0x9f, 0x08, 0x61, 0xd8,
		0x07, 0xca, 0x0d, 0xbf, 0x50, 0x0d, 0x6a, 0x61,
		0x56, 0xa3, 0x8e, 0x08, 0x8a, 0x22, 0xb6, 0x5e,
		0x52, 0xbc, 0x51, 0x4d, 0x16, 0xcc, 0xf8, 0x06,
		0x81, 0x8c, 0xe9, 0x1a, 0xb7, 0x79, 0x37, 0x36,
		0x5a, 0xf9, 0x0b, 0xbf, 0x74, 0xa3, 0x5b, 0xe6,
		0xb4, 0x0b, 0x8e, 0xed, 0xf2, 0x78, 0x5e, 0x42,
		0x87, 0x4d,
	};
	struct chacha_state st;
	u32 k[CHACHA_KEY_WORDS];
	u8 out[sizeof(ct)];

	for (int i = 0; i < CHACHA_KEY_WORDS; i++)
		k[i] = get_unaligned_le32(key + 4 * i);

	chacha_init(&st, k, iv);
	chacha20_crypt(&st, out, pt, sizeof(out));
	KUNIT_ASSERT_MEMEQ(test, out, ct, sizeof(ct));

	chacha_init(&st, k, iv);
	chacha_ref(&st, out, pt, sizeof(out), 20);
	KUNIT_ASSERT_MEMEQ(test, out, ct, sizeof(ct));
}

static void benchmark_chacha(struct kunit *test)
{
	static const size_t lens_to_test[] = {
		64, 256, 512, 1024, 1420, 2048, 4096, 8920, 16384,
	};
	const size_t max_len = 16384;
	u8 *data = alloc_buf(test, max_len);
	u32 key[CHACHA_KEY_WORDS] = {};
	u8 iv[CHACHA_IV_SIZE] = {};
	struct chacha_state st;

	if (!IS_ENABLED(CONFIG_CRYPTO_LIB_BENCHMARK))
		kunit_skip(test, "not enabled");

	/* Warm-up */
	chacha_init(&st, key, iv);
	memset(data, 0, max_len);
	for (size_t i = 0; i < 10000000; i += max_len)
		chacha20_crypt(&st, data, data, max_len);

	for (size_t i = 0; i < ARRAY_SIZE(lens_to_test); i++) {
		size_t len = lens_to_test[i];
		/* The '+ 64' tries to account for per-call overhead. */
		size_t num_iters = 10000000 / (len + 64);
		u64 t;

		preempt_disable();
		t = ktime_get_ns();
		for (size_t j = 0; j < num_iters; j++)
			chacha20_crypt(&st, data, data, len);
		t = ktime_get_ns() - t;
		preempt_enable();
		kunit_info(test, "len=%zu: %llu MB/s", len,
			   div64_u64((u64)len * num_iters * 1000, t ?: 1));
	}
}

static struct kunit_case chacha_test_cases[] = {
	KUNIT_CASE(test_chacha_rfc7539),
	KUNIT_CASE(test_chacha_all_lengths),
	KUNIT_CASE(test_chacha_random),
	KUNIT_CASE(benchmark_chacha),
	{},
};

static struct kunit_suite chacha_test_suite = {
	.name = "chacha",
	.test_cases = chacha_test_cases,
};
kunit_test_suite(chacha_test_suite);

MODULE_DESCRIPTION("KUnit tests and benchmark for ChaCha");
MODULE_LICENSE("GPL");
