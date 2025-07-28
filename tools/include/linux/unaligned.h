/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_UNALIGNED_H
#define __LINUX_UNALIGNED_H

/*
 * This is the most generic implementation of unaligned accesses
 * and should work almost anywhere.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
#include <vdso/unaligned.h>

#define get_unaligned(ptr)	__get_unaligned_t(typeof(*(ptr)), (ptr))
#define put_unaligned(val, ptr) __put_unaligned_t(typeof(*(ptr)), (val), (ptr))

static inline u16 get_unaligned_le16(const void *p)
{
	return le16_to_cpu(__get_unaligned_t(__le16, p));
}

static inline u32 get_unaligned_le32(const void *p)
{
	return le32_to_cpu(__get_unaligned_t(__le32, p));
}

static inline u64 get_unaligned_le64(const void *p)
{
	return le64_to_cpu(__get_unaligned_t(__le64, p));
}

static inline void put_unaligned_le16(u16 val, void *p)
{
	__put_unaligned_t(__le16, cpu_to_le16(val), p);
}

static inline void put_unaligned_le32(u32 val, void *p)
{
	__put_unaligned_t(__le32, cpu_to_le32(val), p);
}

static inline void put_unaligned_le64(u64 val, void *p)
{
	__put_unaligned_t(__le64, cpu_to_le64(val), p);
}

static inline u16 get_unaligned_be16(const void *p)
{
	return be16_to_cpu(__get_unaligned_t(__be16, p));
}

static inline u32 get_unaligned_be32(const void *p)
{
	return be32_to_cpu(__get_unaligned_t(__be32, p));
}

static inline u64 get_unaligned_be64(const void *p)
{
	return be64_to_cpu(__get_unaligned_t(__be64, p));
}

static inline void put_unaligned_be16(u16 val, void *p)
{
	__put_unaligned_t(__be16, cpu_to_be16(val), p);
}

static inline void put_unaligned_be32(u32 val, void *p)
{
	__put_unaligned_t(__be32, cpu_to_be32(val), p);
}

static inline void put_unaligned_be64(u64 val, void *p)
{
	__put_unaligned_t(__be64, cpu_to_be64(val), p);
}

static inline u32 __get_unaligned_be24(const u8 *p)
{
	return p[0] << 16 | p[1] << 8 | p[2];
}

static inline u32 get_unaligned_be24(const void *p)
{
	return __get_unaligned_be24(p);
}

static inline u32 __get_unaligned_le24(const u8 *p)
{
	return p[0] | p[1] << 8 | p[2] << 16;
}

static inline u32 get_unaligned_le24(const void *p)
{
	return __get_unaligned_le24(p);
}

static inline void __put_unaligned_be24(const u32 val, u8 *p)
{
	*p++ = (val >> 16) & 0xff;
	*p++ = (val >> 8) & 0xff;
	*p++ = val & 0xff;
}

static inline void put_unaligned_be24(const u32 val, void *p)
{
	__put_unaligned_be24(val, p);
}

static inline void __put_unaligned_le24(const u32 val, u8 *p)
{
	*p++ = val & 0xff;
	*p++ = (val >> 8) & 0xff;
	*p++ = (val >> 16) & 0xff;
}

static inline void put_unaligned_le24(const u32 val, void *p)
{
	__put_unaligned_le24(val, p);
}

static inline void __put_unaligned_be48(const u64 val, u8 *p)
{
	*p++ = (val >> 40) & 0xff;
	*p++ = (val >> 32) & 0xff;
	*p++ = (val >> 24) & 0xff;
	*p++ = (val >> 16) & 0xff;
	*p++ = (val >> 8) & 0xff;
	*p++ = val & 0xff;
}

static inline void put_unaligned_be48(const u64 val, void *p)
{
	__put_unaligned_be48(val, p);
}

static inline u64 __get_unaligned_be48(const u8 *p)
{
	return (u64)p[0] << 40 | (u64)p[1] << 32 | (u64)p[2] << 24 |
		p[3] << 16 | p[4] << 8 | p[5];
}

static inline u64 get_unaligned_be48(const void *p)
{
	return __get_unaligned_be48(p);
}
#pragma GCC diagnostic pop

#endif /* __LINUX_UNALIGNED_H */
