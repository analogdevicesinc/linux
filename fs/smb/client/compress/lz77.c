// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2026, SUSE LLC
 *
 * Authors: Enzo Matsumiya <ematsumiya@suse.de>
 *
 * Implementation of the LZ77 "plain" compression algorithm, as per MS-XCA spec.
 */
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/count_zeros.h>
#include <linux/unaligned.h>

#include "lz77.h"

/*
 * Compression parameters.
 */
#define LZ77_MATCH_MAX_DIST	SZ_8K
#define LZ77_HASH_LOG		15
#define LZ77_HASH_SIZE		(1 << LZ77_HASH_LOG)
#define LZ77_RSTEP_SIZE		sizeof(u32)
#define LZ77_MSTEP_SIZE		sizeof(u64)
#define LZ77_SKIP_TRIGGER	4

#define LZ77_PREFETCH(ptr)	__builtin_prefetch((ptr), 0, 3)
#define LZ77_FLAG_MAX		32

static __always_inline u8 lz77_read8(const u8 *ptr)
{
	return get_unaligned(ptr);
}

static __always_inline u32 lz77_read32(const u32 *ptr)
{
	return get_unaligned(ptr);
}

static __always_inline u64 lz77_read64(const u64 *ptr)
{
	return get_unaligned(ptr);
}

static __always_inline void lz77_write8(u8 *ptr, u8 v)
{
	put_unaligned(v, ptr);
}

static __always_inline void lz77_write16(u16 *ptr, u16 v)
{
	put_unaligned_le16(v, ptr);
}

static __always_inline void lz77_write32(u32 *ptr, u32 v)
{
	put_unaligned_le32(v, ptr);
}

static __always_inline u32 lz77_match_len(const void *match, const void *cur, const void *end)
{
	const void *start = cur;

	/* Safe for a do/while because otherwise we wouldn't reach here from the main loop. */
	do {
		const u64 diff = lz77_read64(cur) ^ lz77_read64(match);

		if (!diff) {
			cur += LZ77_MSTEP_SIZE;
			match += LZ77_MSTEP_SIZE;

			continue;
		}

		/* This computes the number of common bytes in @diff. */
		cur += count_trailing_zeros(diff) >> 3;

		return (cur - start);
	} while (likely(cur + LZ77_MSTEP_SIZE <= end));

	/* Fallback to byte-by-byte comparison for last <8 bytes. */
	while (cur < end && lz77_read8(cur) == lz77_read8(match)) {
		cur++;
		match++;
	}

	return (cur - start);
}

static __always_inline void *lz77_encode_match(void *dst, void **nib, u16 dist, u32 len)
{
	len -= 3;
	dist--;
	dist <<= 3;

	if (len < 7) {
		lz77_write16(dst, dist + len);

		return dst + 2;
	}

	dist |= 7;
	lz77_write16(dst, dist);
	dst += 2;
	len -= 7;

	if (!*nib) {
		lz77_write8(dst, umin(len, 15));
		*nib = dst;
		dst++;
	} else {
		u8 *b = *nib;

		lz77_write8(b, *b | umin(len, 15) << 4);
		*nib = NULL;
	}

	if (len < 15)
		return dst;

	len -= 15;
	if (len < 255) {
		lz77_write8(dst, len);

		return dst + 1;
	}

	lz77_write8(dst, 0xff);
	dst++;
	len += 7 + 15;
	if (len <= 0xffff) {
		lz77_write16(dst, len);

		return dst + 2;
	}

	lz77_write16(dst, 0);
	dst += 2;
	lz77_write32(dst, len);

	return dst + 4;
}

static __always_inline void *lz77_encode_literals(const void *start, const void *end, void *dst,
						  long *f, u32 *fc, void **fp)
{
	if (start >= end)
		return dst;

	do {
		const u32 len = umin(end - start, LZ77_FLAG_MAX - *fc);

		memcpy(dst, start, len);

		dst += len;
		start += len;

		*f <<= len;
		*fc += len;
		if (*fc == LZ77_FLAG_MAX) {
			lz77_write32(*fp, *f);
			*fc = 0;
			*fp = dst;
			dst += 4;
		}
	} while (start < end);

	return dst;
}

static __always_inline u32 lz77_hash(const u32 v)
{
	return ((v ^ 0x9E3779B9) * 0x85EBCA6B) >> (32 - LZ77_HASH_LOG);
}

noinline int lz77_compress(const void *src, const u32 slen, void *dst, u32 *dlen)
{
	const void *srcp, *rlim, *end, *anchor;
	u32 *htable, hash, flag_count = 0;
	void *dstp, *nib, *flag_pos;
	long flag = 0;

	/* This is probably a bug, so throw a warning. */
	if (WARN_ON_ONCE(*dlen < lz77_compressed_alloc_size(slen)))
		return -EINVAL;

	srcp = anchor = src;
	end = srcp + slen; /* absolute end */
	rlim = end - LZ77_MSTEP_SIZE; /* read limit (for lz77_match_len()) */
	dstp = dst;
	flag_pos = dstp;
	dstp += 4;
	nib = NULL;

	htable = kvcalloc(LZ77_HASH_SIZE, sizeof(*htable), GFP_KERNEL);
	if (!htable)
		return -ENOMEM;

	LZ77_PREFETCH(srcp + LZ77_RSTEP_SIZE);

	hash = lz77_hash(lz77_read32(srcp++));
	htable[hash] = 0;
	hash = lz77_hash(lz77_read32(srcp));

	/*
	 * Main loop.
	 *
	 * @dlen is >= lz77_compressed_alloc_size(), so run without bound-checking @dstp.
	 *
	 * This code was crafted in a way to best utilise fetch-decode-execute CPU flow.
	 * Any attempt to optimize it, or even organize it, can lead to huge performance loss.
	 */
	do {
		const void *match, *next = srcp;
		u32 len, step = 1, skip = 1U << LZ77_SKIP_TRIGGER;

		/* Match finding (hot path -- don't change the read/check/write order). */
		do {
			const u32 cur_hash = hash;

			srcp = next;
			next += step;
			step = (skip++ >> LZ77_SKIP_TRIGGER);
			if (unlikely(next > rlim))
				goto out;

			hash = lz77_hash(lz77_read32(next));
			match = src + htable[cur_hash];
			htable[cur_hash] = srcp - src;
		} while (likely(match + LZ77_MATCH_MAX_DIST < srcp) ||
			 lz77_read32(match) != lz77_read32(srcp));

		dstp = lz77_encode_literals(anchor, srcp, dstp, &flag, &flag_count, &flag_pos);
		len = lz77_match_len(match, srcp, end);
		dstp = lz77_encode_match(dstp, &nib, srcp - match, len);
		srcp += len;
		anchor = srcp;

		LZ77_PREFETCH(srcp);

		flag = (flag << 1) | 1;
		flag_count++;
		if (flag_count == LZ77_FLAG_MAX) {
			lz77_write32(flag_pos, flag);
			flag_count = 0;
			flag_pos = dstp;
			dstp += 4;
		}

		if (unlikely(srcp > rlim))
			break;

		/* Prepare for next loop. */
		hash = lz77_hash(lz77_read32(srcp));
	} while (srcp < end);
out:
	dstp = lz77_encode_literals(anchor, end, dstp, &flag, &flag_count, &flag_pos);

	flag_count = LZ77_FLAG_MAX - flag_count;
	flag <<= flag_count;
	flag |= (1UL << flag_count) - 1;
	lz77_write32(flag_pos, flag);

	*dlen = dstp - dst;
	kvfree(htable);

	if (*dlen < slen)
		return 0;

	return -EMSGSIZE;
}
