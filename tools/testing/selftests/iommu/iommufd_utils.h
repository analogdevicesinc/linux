/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2021-2022, NVIDIA CORPORATION & AFFILIATES */
#ifndef __SELFTEST_IOMMUFD_UTILS
#define __SELFTEST_IOMMUFD_UTILS

#include <unistd.h>
#include <stddef.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <assert.h>
#include <poll.h>

#include "../kselftest_harness.h"
#include "../../../../drivers/iommu/iommufd/iommufd_test.h"

/* Hack to make assertions more readable */
#define _IOMMU_TEST_CMD(x) IOMMU_TEST_CMD

/* Imported from include/asm-generic/bitops/generic-non-atomic.h */
#define BITS_PER_BYTE 8
#define BITS_PER_LONG __BITS_PER_LONG
#define BIT_MASK(nr) (1UL << ((nr) % __BITS_PER_LONG))
#define BIT_WORD(nr) ((nr) / __BITS_PER_LONG)

enum {
	IOPT_PAGES_ACCOUNT_NONE = 0,
	IOPT_PAGES_ACCOUNT_USER = 1,
	IOPT_PAGES_ACCOUNT_MM = 2,
};

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

static inline void set_bit(unsigned int nr, unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);

	*p |= mask;
}

static inline bool test_bit(unsigned int nr, unsigned long *addr)
{
	return 1UL & (addr[BIT_WORD(nr)] >> (nr & (BITS_PER_LONG - 1)));
}

static void *buffer;
static unsigned long BUFFER_SIZE;

static void *mfd_buffer;
static int mfd;

static unsigned long PAGE_SIZE;

#define sizeof_field(TYPE, MEMBER) sizeof((((TYPE *)0)->MEMBER))
#define offsetofend(TYPE, MEMBER) \
	(offsetof(TYPE, MEMBER) + sizeof_field(TYPE, MEMBER))

#define test_err_mmap(_errno, length, offset)                                 \
	EXPECT_ERRNO(_errno, (long)mmap(NULL, length, PROT_READ | PROT_WRITE, \
					MAP_SHARED, self->fd, offset))

static inline void *memfd_mmap(size_t length, int prot, int flags, int *mfd_p)
{
	int mfd_flags = (flags & MAP_HUGETLB) ? MFD_HUGETLB : 0;
	int mfd = memfd_create("buffer", mfd_flags);
	void *buf = MAP_FAILED;

	if (mfd <= 0)
		return MAP_FAILED;
	if (ftruncate(mfd, length))
		goto out;
	*mfd_p = mfd;
	buf = mmap(0, length, prot, flags, mfd, 0);
out:
	if (buf == MAP_FAILED)
		close(mfd);
	return buf;
}

/*
 * Have the kernel check the refcount on pages. I don't know why a freshly
 * mmap'd anon non-compound page starts out with a ref of 3
 */
#define check_refs(_ptr, _length, _refs)                                      \
	({                                                                    \
		struct iommu_test_cmd test_cmd = {                            \
			.size = sizeof(test_cmd),                             \
			.op = IOMMU_TEST_OP_MD_CHECK_REFS,                    \
			.check_refs = { .length = _length,                    \
					.uptr = (uintptr_t)(_ptr),            \
					.refs = _refs },                      \
		};                                                            \
		ASSERT_EQ(0,                                                  \
			  ioctl(self->fd,                                     \
				_IOMMU_TEST_CMD(IOMMU_TEST_OP_MD_CHECK_REFS), \
				&test_cmd));                                  \
	})

static int _test_cmd_mock_domain(int fd, unsigned int ioas_id, __u32 *stdev_id,
				 __u32 *hwpt_id, __u32 *idev_id)
{
	struct iommu_test_cmd cmd = {
		.size = sizeof(cmd),
		.op = IOMMU_TEST_OP_MOCK_DOMAIN,
		.id = ioas_id,
		.mock_domain = {},
	};
	int ret;

	ret = ioctl(fd, IOMMU_TEST_CMD, &cmd);
	if (ret)
		return ret;
	if (stdev_id)
		*stdev_id = cmd.mock_domain.out_stdev_id;
	assert(cmd.id != 0);
	if (hwpt_id)
		*hwpt_id = cmd.mock_domain.out_hwpt_id;
	if (idev_id)
		*idev_id = cmd.mock_domain.out_idev_id;
	return 0;
}
#define test_cmd_mock_domain(ioas_id, stdev_id, hwpt_id, idev_id)       \
	ASSERT_EQ(0, _test_cmd_mock_domain(self->fd, ioas_id, stdev_id, \
					   hwpt_id, idev_id))
#define test_err_mock_domain(_errno, ioas_id, stdev_id, hwpt_id)      \
	EXPECT_ERRNO(_errno, _test_cmd_mock_domain(self->fd, ioas_id, \
						   stdev_id, hwpt_id, NULL))

static int _test_cmd_mock_domain_flags(int fd, unsigned int ioas_id,
				       __u32 stdev_flags, __u32 *stdev_id,
				       __u32 *hwpt_id, __u32 *idev_id)
{
	struct iommu_test_cmd cmd = {
		.size = sizeof(cmd),
		.op = IOMMU_TEST_OP_MOCK_DOMAIN_FLAGS,
		.id = ioas_id,
		.mock_domain_flags = { .dev_flags = stdev_flags },
	};
	int ret;

	ret = ioctl(fd, IOMMU_TEST_CMD, &cmd);
	if (ret)
		return ret;
	if (stdev_id)
		*stdev_id = cmd.mock_domain_flags.out_stdev_id;
	assert(cmd.id != 0);
	if (hwpt_id)
		*hwpt_id = cmd.mock_domain_flags.out_hwpt_id;
	if (idev_id)
		*idev_id = cmd.mock_domain_flags.out_idev_id;
	return 0;
}
#define test_cmd_mock_domain_flags(ioas_id, flags, stdev_id, hwpt_id, idev_id) \
	ASSERT_EQ(0, _test_cmd_mock_domain_flags(self->fd, ioas_id, flags,     \
						 stdev_id, hwpt_id, idev_id))
#define test_err_mock_domain_flags(_errno, ioas_id, flags, stdev_id, hwpt_id) \
	EXPECT_ERRNO(_errno,                                                  \
		     _test_cmd_mock_domain_flags(self->fd, ioas_id, flags,    \
						 stdev_id, hwpt_id, NULL))

static int _test_cmd_mock_domain_replace(int fd, __u32 stdev_id, __u32 pt_id,
					 __u32 *hwpt_id)
{
	struct iommu_test_cmd cmd = {
		.size = sizeof(cmd),
		.op = IOMMU_TEST_OP_MOCK_DOMAIN_REPLACE,
		.id = stdev_id,
		.mock_domain_replace = {
			.pt_id = pt_id,
		},
	};
	int ret;

	ret = ioctl(fd, IOMMU_TEST_CMD, &cmd);
	if (ret)
		return ret;
	if (hwpt_id)
		*hwpt_id = cmd.mock_domain_replace.pt_id;
	return 0;
}

#define test_cmd_mock_domain_replace(stdev_id, pt_id)                         \
	ASSERT_EQ(0, _test_cmd_mock_domain_replace(self->fd, stdev_id, pt_id, \
						   NULL))
#define test_err_mock_domain_replace(_errno, stdev_id, pt_id)                  \
	EXPECT_ERRNO(_errno, _test_cmd_mock_domain_replace(self->fd, stdev_id, \
							   pt_id, NULL))

static int _test_cmd_hwpt_alloc(int fd, __u32 device_id, __u32 pt_id, __u32 ft_id,
				__u32 flags, __u32 *hwpt_id, __u32 data_type,
				void *data, size_t data_len)
{
	struct iommu_hwpt_alloc cmd = {
		.size = sizeof(cmd),
		.flags = flags,
		.dev_id = device_id,
		.pt_id = pt_id,
		.data_type = data_type,
		.data_len = data_len,
		.data_uptr = (uint64_t)data,
		.fault_id = ft_id,
	};
	int ret;

	ret = ioctl(fd, IOMMU_HWPT_ALLOC, &cmd);
	if (ret)
		return ret;
	if (hwpt_id)
		*hwpt_id = cmd.out_hwpt_id;
	return 0;
}

#define test_cmd_hwpt_alloc(device_id, pt_id, flags, hwpt_id)                  \
	ASSERT_EQ(0, _test_cmd_hwpt_alloc(self->fd, device_id, pt_id, 0, flags,   \
					  hwpt_id, IOMMU_HWPT_DATA_NONE, NULL, \
					  0))
#define test_err_hwpt_alloc(_errno, device_id, pt_id, flags, hwpt_id)   \
	EXPECT_ERRNO(_errno, _test_cmd_hwpt_alloc(                      \
				     self->fd, device_id, pt_id, 0, flags, \
				     hwpt_id, IOMMU_HWPT_DATA_NONE, NULL, 0))

#define test_cmd_hwpt_alloc_nested(device_id, pt_id, flags, hwpt_id,         \
				   data_type, data, data_len)                \
	ASSERT_EQ(0, _test_cmd_hwpt_alloc(self->fd, device_id, pt_id, 0, flags, \
					  hwpt_id, data_type, data, data_len))
#define test_err_hwpt_alloc_nested(_errno, device_id, pt_id, flags, hwpt_id, \
				   data_type, data, data_len)                \
	EXPECT_ERRNO(_errno,                                                 \
		     _test_cmd_hwpt_alloc(self->fd, device_id, pt_id, 0, flags, \
					  hwpt_id, data_type, data, data_len))

#define test_cmd_hwpt_alloc_iopf(device_id, pt_id, fault_id, flags, hwpt_id,    \
				   data_type, data, data_len)                   \
	ASSERT_EQ(0, _test_cmd_hwpt_alloc(self->fd, device_id, pt_id, fault_id, \
					  flags, hwpt_id, data_type, data,      \
					  data_len))
#define test_err_hwpt_alloc_iopf(_errno, device_id, pt_id, fault_id, flags,     \
				 hwpt_id, data_type, data, data_len)            \
	EXPECT_ERRNO(_errno,                                                    \
		     _test_cmd_hwpt_alloc(self->fd, device_id, pt_id, fault_id, \
					  flags, hwpt_id, data_type, data,      \
					  data_len))

#define test_cmd_hwpt_check_iotlb(hwpt_id, iotlb_id, expected)                 \
	({                                                                     \
		struct iommu_test_cmd test_cmd = {                             \
			.size = sizeof(test_cmd),                              \
			.op = IOMMU_TEST_OP_MD_CHECK_IOTLB,                    \
			.id = hwpt_id,                                         \
			.check_iotlb = {                                       \
				.id = iotlb_id,                                \
				.iotlb = expected,                             \
			},                                                     \
		};                                                             \
		ASSERT_EQ(0,                                                   \
			  ioctl(self->fd,                                      \
				_IOMMU_TEST_CMD(IOMMU_TEST_OP_MD_CHECK_IOTLB), \
				&test_cmd));                                   \
	})

#define test_cmd_hwpt_check_iotlb_all(hwpt_id, expected)                       \
	({                                                                     \
		int i;                                                         \
		for (i = 0; i < MOCK_NESTED_DOMAIN_IOTLB_NUM; i++)             \
			test_cmd_hwpt_check_iotlb(hwpt_id, i, expected);       \
	})

#define test_cmd_dev_check_cache(device_id, cache_id, expected)                \
	({                                                                     \
		struct iommu_test_cmd test_cmd = {                             \
			.size = sizeof(test_cmd),                              \
			.op = IOMMU_TEST_OP_DEV_CHECK_CACHE,                   \
			.id = device_id,                                       \
			.check_dev_cache = {                                   \
				.id = cache_id,                                \
				.cache = expected,                             \
			},                                                     \
		};                                                             \
		ASSERT_EQ(0, ioctl(self->fd,                                   \
				   _IOMMU_TEST_CMD(                            \
					   IOMMU_TEST_OP_DEV_CHECK_CACHE),     \
				   &test_cmd));                                \
	})

#define test_cmd_dev_check_cache_all(device_id, expected)                      \
	({                                                                     \
		int c;                                                         \
		for (c = 0; c < MOCK_DEV_CACHE_NUM; c++)                       \
			test_cmd_dev_check_cache(device_id, c, expected);      \
	})

static int _test_cmd_hwpt_invalidate(int fd, __u32 hwpt_id, void *reqs,
				     uint32_t data_type, uint32_t lreq,
				     uint32_t *nreqs)
{
	struct iommu_hwpt_invalidate cmd = {
		.size = sizeof(cmd),
		.hwpt_id = hwpt_id,
		.data_type = data_type,
		.data_uptr = (uint64_t)reqs,
		.entry_len = lreq,
		.entry_num = *nreqs,
	};
	int rc = ioctl(fd, IOMMU_HWPT_INVALIDATE, &cmd);
	*nreqs = cmd.entry_num;
	return rc;
}

#define test_cmd_hwpt_invalidate(hwpt_id, reqs, data_type, lreq, nreqs)       \
	({                                                                    \
		ASSERT_EQ(0,                                                  \
			  _test_cmd_hwpt_invalidate(self->fd, hwpt_id, reqs,  \
						    data_type, lreq, nreqs)); \
	})
#define test_err_hwpt_invalidate(_errno, hwpt_id, reqs, data_type, lreq, \
				 nreqs)                                  \
	({                                                               \
		EXPECT_ERRNO(_errno, _test_cmd_hwpt_invalidate(          \
					     self->fd, hwpt_id, reqs,    \
					     data_type, lreq, nreqs));   \
	})

static int _test_cmd_viommu_invalidate(int fd, __u32 viommu_id, void *reqs,
				       uint32_t data_type, uint32_t lreq,
				       uint32_t *nreqs)
{
	struct iommu_hwpt_invalidate cmd = {
		.size = sizeof(cmd),
		.hwpt_id = viommu_id,
		.data_type = data_type,
		.data_uptr = (uint64_t)reqs,
		.entry_len = lreq,
		.entry_num = *nreqs,
	};
	int rc = ioctl(fd, IOMMU_HWPT_INVALIDATE, &cmd);
	*nreqs = cmd.entry_num;
	return rc;
}

#define test_cmd_viommu_invalidate(viommu, reqs, lreq, nreqs)                  \
	({                                                                     \
		ASSERT_EQ(0,                                                   \
			  _test_cmd_viommu_invalidate(self->fd, viommu, reqs,  \
					IOMMU_VIOMMU_INVALIDATE_DATA_SELFTEST, \
					lreq, nreqs));                         \
	})
#define test_err_viommu_invalidate(_errno, viommu_id, reqs, data_type, lreq,   \
				 nreqs)                                        \
	({                                                                     \
		EXPECT_ERRNO(_errno, _test_cmd_viommu_invalidate(              \
					     self->fd, viommu_id, reqs,        \
					     data_type, lreq, nreqs));         \
	})

static int _test_cmd_access_replace_ioas(int fd, __u32 access_id,
					 unsigned int ioas_id)
{
	struct iommu_test_cmd cmd = {
		.size = sizeof(cmd),
		.op = IOMMU_TEST_OP_ACCESS_REPLACE_IOAS,
		.id = access_id,
		.access_replace_ioas = { .ioas_id = ioas_id },
	};
	int ret;

	ret = ioctl(fd, IOMMU_TEST_CMD, &cmd);
	if (ret)
		return ret;
	return 0;
}
#define test_cmd_access_replace_ioas(access_id, ioas_id) \
	ASSERT_EQ(0, _test_cmd_access_replace_ioas(self->fd, access_id, ioas_id))

static int _test_cmd_set_dirty_tracking(int fd, __u32 hwpt_id, bool enabled)
{
	struct iommu_hwpt_set_dirty_tracking cmd = {
		.size = sizeof(cmd),
		.flags = enabled ? IOMMU_HWPT_DIRTY_TRACKING_ENABLE : 0,
		.hwpt_id = hwpt_id,
	};
	int ret;

	ret = ioctl(fd, IOMMU_HWPT_SET_DIRTY_TRACKING, &cmd);
	if (ret)
		return -errno;
	return 0;
}
#define test_cmd_set_dirty_tracking(hwpt_id, enabled) \
	ASSERT_EQ(0, _test_cmd_set_dirty_tracking(self->fd, hwpt_id, enabled))

static int _test_cmd_get_dirty_bitmap(int fd, __u32 hwpt_id, size_t length,
				      __u64 iova, size_t page_size,
				      __u64 *bitmap, __u32 flags)
{
	struct iommu_hwpt_get_dirty_bitmap cmd = {
		.size = sizeof(cmd),
		.hwpt_id = hwpt_id,
		.flags = flags,
		.iova = iova,
		.length = length,
		.page_size = page_size,
		.data = (uintptr_t)bitmap,
	};
	int ret;

	ret = ioctl(fd, IOMMU_HWPT_GET_DIRTY_BITMAP, &cmd);
	if (ret)
		return ret;
	return 0;
}

#define test_cmd_get_dirty_bitmap(fd, hwpt_id, length, iova, page_size,    \
				  bitmap, flags)                           \
	ASSERT_EQ(0, _test_cmd_get_dirty_bitmap(fd, hwpt_id, length, iova, \
						page_size, bitmap, flags))

static int _test_cmd_mock_domain_set_dirty(int fd, __u32 hwpt_id, size_t length,
					   __u64 iova, size_t page_size,
					   __u64 *bitmap, __u64 *dirty)
{
	struct iommu_test_cmd cmd = {
		.size = sizeof(cmd),
		.op = IOMMU_TEST_OP_DIRTY,
		.id = hwpt_id,
		.dirty = {
			.iova = iova,
			.length = length,
			.page_size = page_size,
			.uptr = (uintptr_t)bitmap,
		}
	};
	int ret;

	ret = ioctl(fd, _IOMMU_TEST_CMD(IOMMU_TEST_OP_DIRTY), &cmd);
	if (ret)
		return -ret;
	if (dirty)
		*dirty = cmd.dirty.out_nr_dirty;
	return 0;
}

#define test_cmd_mock_domain_set_dirty(fd, hwpt_id, length, iova, page_size, \
				       bitmap, nr)                           \
	ASSERT_EQ(0,                                                         \
		  _test_cmd_mock_domain_set_dirty(fd, hwpt_id, length, iova, \
						  page_size, bitmap, nr))

static int _test_mock_dirty_bitmaps(int fd, __u32 hwpt_id, size_t length,
				    __u64 iova, size_t page_size,
				    size_t pte_page_size, __u64 *bitmap,
				    __u64 nbits, __u32 flags,
				    struct __test_metadata *_metadata)
{
	unsigned long npte = pte_page_size / page_size, pteset = 2 * npte;
	unsigned long j, i, nr = nbits / pteset ?: 1;
	unsigned long bitmap_size = DIV_ROUND_UP(nbits, BITS_PER_BYTE);
	__u64 out_dirty = 0;

	/* Mark all even bits as dirty in the mock domain */
	memset(bitmap, 0, bitmap_size);
	for (i = 0; i < nbits; i += pteset)
		set_bit(i, (unsigned long *)bitmap);

	test_cmd_mock_domain_set_dirty(fd, hwpt_id, length, iova, page_size,
				       bitmap, &out_dirty);
	ASSERT_EQ(nr, out_dirty);

	/* Expect all even bits as dirty in the user bitmap */
	memset(bitmap, 0, bitmap_size);
	test_cmd_get_dirty_bitmap(fd, hwpt_id, length, iova, page_size, bitmap,
				  flags);
	/* Beware ASSERT_EQ() is two statements -- braces are not redundant! */
	for (i = 0; i < nbits; i += pteset) {
		for (j = 0; j < pteset; j++) {
			ASSERT_EQ(j < npte,
				  test_bit(i + j, (unsigned long *)bitmap));
		}
		ASSERT_EQ(!(i % pteset), test_bit(i, (unsigned long *)bitmap));
	}

	memset(bitmap, 0, bitmap_size);
	test_cmd_get_dirty_bitmap(fd, hwpt_id, length, iova, page_size, bitmap,
				  flags);

	/* It as read already -- expect all zeroes */
	for (i = 0; i < nbits; i += pteset) {
		for (j = 0; j < pteset; j++) {
			ASSERT_EQ(
				(j < npte) &&
					(flags &
					 IOMMU_HWPT_GET_DIRTY_BITMAP_NO_CLEAR),
				test_bit(i + j, (unsigned long *)bitmap));
		}
	}

	return 0;
}
#define test_mock_dirty_bitmaps(hwpt_id, length, iova, page_size, pte_size,\
				bitmap, bitmap_size, flags, _metadata)     \
	ASSERT_EQ(0, _test_mock_dirty_bitmaps(self->fd, hwpt_id, length, iova, \
					      page_size, pte_size, bitmap,     \
					      bitmap_size, flags, _metadata))

static int _test_cmd_create_access(int fd, unsigned int ioas_id,
				   __u32 *access_id, unsigned int flags)
{
	struct iommu_test_cmd cmd = {
		.size = sizeof(cmd),
		.op = IOMMU_TEST_OP_CREATE_ACCESS,
		.id = ioas_id,
		.create_access = { .flags = flags },
	};
	int ret;

	ret = ioctl(fd, IOMMU_TEST_CMD, &cmd);
	if (ret)
		return ret;
	*access_id = cmd.create_access.out_access_fd;
	return 0;
}
#define test_cmd_create_access(ioas_id, access_id, flags)                  \
	ASSERT_EQ(0, _test_cmd_create_access(self->fd, ioas_id, access_id, \
					     flags))

static int _test_cmd_destroy_access(unsigned int access_id)
{
	return close(access_id);
}
#define test_cmd_destroy_access(access_id) \
	ASSERT_EQ(0, _test_cmd_destroy_access(access_id))

static int _test_cmd_destroy_access_pages(int fd, unsigned int access_id,
					  unsigned int access_pages_id)
{
	struct iommu_test_cmd cmd = {
		.size = sizeof(cmd),
		.op = IOMMU_TEST_OP_DESTROY_ACCESS_PAGES,
		.id = access_id,
		.destroy_access_pages = { .access_pages_id = access_pages_id },
	};
	return ioctl(fd, IOMMU_TEST_CMD, &cmd);
}
#define test_cmd_destroy_access_pages(access_id, access_pages_id)        \
	ASSERT_EQ(0, _test_cmd_destroy_access_pages(self->fd, access_id, \
						    access_pages_id))
#define test_err_destroy_access_pages(_errno, access_id, access_pages_id) \
	EXPECT_ERRNO(_errno, _test_cmd_destroy_access_pages(              \
				     self->fd, access_id, access_pages_id))

static int _test_ioctl_destroy(int fd, unsigned int id)
{
	struct iommu_destroy cmd = {
		.size = sizeof(cmd),
		.id = id,
	};
	return ioctl(fd, IOMMU_DESTROY, &cmd);
}
#define test_ioctl_destroy(id) ASSERT_EQ(0, _test_ioctl_destroy(self->fd, id))

static int _test_ioctl_ioas_alloc(int fd, __u32 *id)
{
	struct iommu_ioas_alloc cmd = {
		.size = sizeof(cmd),
	};
	int ret;

	ret = ioctl(fd, IOMMU_IOAS_ALLOC, &cmd);
	if (ret)
		return ret;
	*id = cmd.out_ioas_id;
	return 0;
}
#define test_ioctl_ioas_alloc(id)                                   \
	({                                                          \
		ASSERT_EQ(0, _test_ioctl_ioas_alloc(self->fd, id)); \
		ASSERT_NE(0, *(id));                                \
	})

static int _test_ioctl_ioas_map(int fd, unsigned int ioas_id, void *buffer,
				size_t length, __u64 *iova, unsigned int flags)
{
	struct iommu_ioas_map cmd = {
		.size = sizeof(cmd),
		.flags = flags,
		.ioas_id = ioas_id,
		.user_va = (uintptr_t)buffer,
		.length = length,
	};
	int ret;

	if (flags & IOMMU_IOAS_MAP_FIXED_IOVA)
		cmd.iova = *iova;

	ret = ioctl(fd, IOMMU_IOAS_MAP, &cmd);
	*iova = cmd.iova;
	return ret;
}
#define test_ioctl_ioas_map(buffer, length, iova_p)                        \
	ASSERT_EQ(0, _test_ioctl_ioas_map(self->fd, self->ioas_id, buffer, \
					  length, iova_p,                  \
					  IOMMU_IOAS_MAP_WRITEABLE |       \
						  IOMMU_IOAS_MAP_READABLE))

#define test_err_ioctl_ioas_map(_errno, buffer, length, iova_p)            \
	EXPECT_ERRNO(_errno,                                               \
		     _test_ioctl_ioas_map(self->fd, self->ioas_id, buffer, \
					  length, iova_p,                  \
					  IOMMU_IOAS_MAP_WRITEABLE |       \
						  IOMMU_IOAS_MAP_READABLE))

#define test_ioctl_ioas_map_id(ioas_id, buffer, length, iova_p)              \
	ASSERT_EQ(0, _test_ioctl_ioas_map(self->fd, ioas_id, buffer, length, \
					  iova_p,                            \
					  IOMMU_IOAS_MAP_WRITEABLE |         \
						  IOMMU_IOAS_MAP_READABLE))

#define test_ioctl_ioas_map_fixed(buffer, length, iova)                       \
	({                                                                    \
		__u64 __iova = iova;                                          \
		ASSERT_EQ(0, _test_ioctl_ioas_map(                            \
				     self->fd, self->ioas_id, buffer, length, \
				     &__iova,                                 \
				     IOMMU_IOAS_MAP_FIXED_IOVA |              \
					     IOMMU_IOAS_MAP_WRITEABLE |       \
					     IOMMU_IOAS_MAP_READABLE));       \
	})

#define test_ioctl_ioas_map_fixed_id(ioas_id, buffer, length, iova)           \
	({                                                                    \
		__u64 __iova = iova;                                          \
		ASSERT_EQ(0,                                                  \
			  _test_ioctl_ioas_map(                               \
				  self->fd, ioas_id, buffer, length, &__iova, \
				  IOMMU_IOAS_MAP_FIXED_IOVA |                 \
					  IOMMU_IOAS_MAP_WRITEABLE |          \
					  IOMMU_IOAS_MAP_READABLE));          \
	})

#define test_err_ioctl_ioas_map_fixed(_errno, buffer, length, iova)           \
	({                                                                    \
		__u64 __iova = iova;                                          \
		EXPECT_ERRNO(_errno,                                          \
			     _test_ioctl_ioas_map(                            \
				     self->fd, self->ioas_id, buffer, length, \
				     &__iova,                                 \
				     IOMMU_IOAS_MAP_FIXED_IOVA |              \
					     IOMMU_IOAS_MAP_WRITEABLE |       \
					     IOMMU_IOAS_MAP_READABLE));       \
	})

static int _test_ioctl_ioas_unmap(int fd, unsigned int ioas_id, uint64_t iova,
				  size_t length, uint64_t *out_len)
{
	struct iommu_ioas_unmap cmd = {
		.size = sizeof(cmd),
		.ioas_id = ioas_id,
		.iova = iova,
		.length = length,
	};
	int ret;

	ret = ioctl(fd, IOMMU_IOAS_UNMAP, &cmd);
	if (out_len)
		*out_len = cmd.length;
	return ret;
}
#define test_ioctl_ioas_unmap(iova, length)                                \
	ASSERT_EQ(0, _test_ioctl_ioas_unmap(self->fd, self->ioas_id, iova, \
					    length, NULL))

#define test_ioctl_ioas_unmap_id(ioas_id, iova, length)                      \
	ASSERT_EQ(0, _test_ioctl_ioas_unmap(self->fd, ioas_id, iova, length, \
					    NULL))

#define test_err_ioctl_ioas_unmap(_errno, iova, length)                      \
	EXPECT_ERRNO(_errno, _test_ioctl_ioas_unmap(self->fd, self->ioas_id, \
						    iova, length, NULL))

static int _test_ioctl_ioas_map_file(int fd, unsigned int ioas_id, int mfd,
				     size_t start, size_t length, __u64 *iova,
				     unsigned int flags)
{
	struct iommu_ioas_map_file cmd = {
		.size = sizeof(cmd),
		.flags = flags,
		.ioas_id = ioas_id,
		.fd = mfd,
		.start = start,
		.length = length,
	};
	int ret;

	if (flags & IOMMU_IOAS_MAP_FIXED_IOVA)
		cmd.iova = *iova;

	ret = ioctl(fd, IOMMU_IOAS_MAP_FILE, &cmd);
	*iova = cmd.iova;
	return ret;
}

#define test_ioctl_ioas_map_file(mfd, start, length, iova_p)                   \
	ASSERT_EQ(0,                                                           \
		  _test_ioctl_ioas_map_file(                                   \
			  self->fd, self->ioas_id, mfd, start, length, iova_p, \
			  IOMMU_IOAS_MAP_WRITEABLE | IOMMU_IOAS_MAP_READABLE))

#define test_err_ioctl_ioas_map_file(_errno, mfd, start, length, iova_p)     \
	EXPECT_ERRNO(                                                        \
		_errno,                                                      \
		_test_ioctl_ioas_map_file(                                   \
			self->fd, self->ioas_id, mfd, start, length, iova_p, \
			IOMMU_IOAS_MAP_WRITEABLE | IOMMU_IOAS_MAP_READABLE))

#define test_ioctl_ioas_map_id_file(ioas_id, mfd, start, length, iova_p)     \
	ASSERT_EQ(0,                                                         \
		  _test_ioctl_ioas_map_file(                                 \
			  self->fd, ioas_id, mfd, start, length, iova_p,     \
			  IOMMU_IOAS_MAP_WRITEABLE | IOMMU_IOAS_MAP_READABLE))

static int _test_ioctl_set_temp_memory_limit(int fd, unsigned int limit)
{
	struct iommu_test_cmd memlimit_cmd = {
		.size = sizeof(memlimit_cmd),
		.op = IOMMU_TEST_OP_SET_TEMP_MEMORY_LIMIT,
		.memory_limit = { .limit = limit },
	};

	return ioctl(fd, _IOMMU_TEST_CMD(IOMMU_TEST_OP_SET_TEMP_MEMORY_LIMIT),
		     &memlimit_cmd);
}

#define test_ioctl_set_temp_memory_limit(limit) \
	ASSERT_EQ(0, _test_ioctl_set_temp_memory_limit(self->fd, limit))

#define test_ioctl_set_default_memory_limit() \
	test_ioctl_set_temp_memory_limit(65536)

static void teardown_iommufd(int fd, struct __test_metadata *_metadata)
{
	struct iommu_test_cmd test_cmd = {
		.size = sizeof(test_cmd),
		.op = IOMMU_TEST_OP_MD_CHECK_REFS,
		.check_refs = { .length = BUFFER_SIZE,
				.uptr = (uintptr_t)buffer },
	};

	if (fd == -1)
		return;

	EXPECT_EQ(0, close(fd));

	fd = open("/dev/iommu", O_RDWR);
	EXPECT_NE(-1, fd);
	EXPECT_EQ(0, ioctl(fd, _IOMMU_TEST_CMD(IOMMU_TEST_OP_MD_CHECK_REFS),
			   &test_cmd));
	EXPECT_EQ(0, close(fd));
}

#define EXPECT_ERRNO(expected_errno, cmd)         \
	({                                        \
		ASSERT_EQ(-1, cmd);               \
		EXPECT_EQ(expected_errno, errno); \
	})

#endif

/* @data can be NULL */
static int _test_cmd_get_hw_info(int fd, __u32 device_id, __u32 data_type,
				 void *data, size_t data_len,
				 uint32_t *capabilities, uint8_t *max_pasid)
{
	struct iommu_test_hw_info *info = (struct iommu_test_hw_info *)data;
	struct iommu_hw_info cmd = {
		.size = sizeof(cmd),
		.dev_id = device_id,
		.data_len = data_len,
		.in_data_type = data_type,
		.data_uptr = (uint64_t)data,
		.out_capabilities = 0,
	};
	int ret;

	if (data_type != IOMMU_HW_INFO_TYPE_DEFAULT)
		cmd.flags |= IOMMU_HW_INFO_FLAG_INPUT_TYPE;

	ret = ioctl(fd, IOMMU_GET_HW_INFO, &cmd);
	if (ret)
		return ret;

	assert(cmd.out_data_type == IOMMU_HW_INFO_TYPE_SELFTEST);

	/*
	 * The struct iommu_test_hw_info should be the one defined
	 * by the current kernel.
	 */
	assert(cmd.data_len == sizeof(struct iommu_test_hw_info));

	/*
	 * Trailing bytes should be 0 if user buffer is larger than
	 * the data that kernel reports.
	 */
	if (data_len > cmd.data_len) {
		char *ptr = (char *)(data + cmd.data_len);
		int idx = 0;

		while (idx < data_len - cmd.data_len) {
			assert(!*(ptr + idx));
			idx++;
		}
	}

	if (info) {
		if (data_len >= offsetofend(struct iommu_test_hw_info, test_reg))
			assert(info->test_reg == IOMMU_HW_INFO_SELFTEST_REGVAL);
		if (data_len >= offsetofend(struct iommu_test_hw_info, flags))
			assert(!info->flags);
	}

	if (max_pasid)
		*max_pasid = cmd.out_max_pasid_log2;

	if (capabilities)
		*capabilities = cmd.out_capabilities;

	return 0;
}

#define test_cmd_get_hw_info(device_id, data_type, data, data_len)         \
	ASSERT_EQ(0, _test_cmd_get_hw_info(self->fd, device_id, data_type, \
					   data, data_len, NULL, NULL))

#define test_err_get_hw_info(_errno, device_id, data_type, data, data_len) \
	EXPECT_ERRNO(_errno,                                               \
		     _test_cmd_get_hw_info(self->fd, device_id, data_type, \
					   data, data_len, NULL, NULL))

#define test_cmd_get_hw_capabilities(device_id, caps)                        \
	ASSERT_EQ(0, _test_cmd_get_hw_info(self->fd, device_id,              \
					   IOMMU_HW_INFO_TYPE_DEFAULT, NULL, \
					   0, &caps, NULL))

#define test_cmd_get_hw_info_pasid(device_id, max_pasid)                     \
	ASSERT_EQ(0, _test_cmd_get_hw_info(self->fd, device_id,              \
					   IOMMU_HW_INFO_TYPE_DEFAULT, NULL, \
					   0, NULL, max_pasid))

static int _test_ioctl_fault_alloc(int fd, __u32 *fault_id, __u32 *fault_fd)
{
	struct iommu_fault_alloc cmd = {
		.size = sizeof(cmd),
	};
	int ret;

	ret = ioctl(fd, IOMMU_FAULT_QUEUE_ALLOC, &cmd);
	if (ret)
		return ret;
	*fault_id = cmd.out_fault_id;
	*fault_fd = cmd.out_fault_fd;
	return 0;
}

#define test_ioctl_fault_alloc(fault_id, fault_fd)                       \
	({                                                               \
		ASSERT_EQ(0, _test_ioctl_fault_alloc(self->fd, fault_id, \
						     fault_fd));         \
		ASSERT_NE(0, *(fault_id));                               \
		ASSERT_NE(0, *(fault_fd));                               \
	})

static int _test_cmd_trigger_iopf(int fd, __u32 device_id, __u32 pasid,
				  __u32 fault_fd)
{
	struct iommu_test_cmd trigger_iopf_cmd = {
		.size = sizeof(trigger_iopf_cmd),
		.op = IOMMU_TEST_OP_TRIGGER_IOPF,
		.trigger_iopf = {
			.dev_id = device_id,
			.pasid = pasid,
			.grpid = 0x2,
			.perm = IOMMU_PGFAULT_PERM_READ | IOMMU_PGFAULT_PERM_WRITE,
			.addr = 0xdeadbeaf,
		},
	};
	struct iommu_hwpt_page_response response = {
		.code = IOMMUFD_PAGE_RESP_SUCCESS,
	};
	struct iommu_hwpt_pgfault fault = {};
	ssize_t bytes;
	int ret;

	ret = ioctl(fd, _IOMMU_TEST_CMD(IOMMU_TEST_OP_TRIGGER_IOPF), &trigger_iopf_cmd);
	if (ret)
		return ret;

	bytes = read(fault_fd, &fault, sizeof(fault));
	if (bytes <= 0)
		return -EIO;

	response.cookie = fault.cookie;

	bytes = write(fault_fd, &response, sizeof(response));
	if (bytes <= 0)
		return -EIO;

	return 0;
}

#define test_cmd_trigger_iopf(device_id, fault_fd) \
	ASSERT_EQ(0, _test_cmd_trigger_iopf(self->fd, device_id, 0x1, fault_fd))
#define test_cmd_trigger_iopf_pasid(device_id, pasid, fault_fd) \
	ASSERT_EQ(0, _test_cmd_trigger_iopf(self->fd, device_id, \
					    pasid, fault_fd))

static int _test_cmd_viommu_alloc(int fd, __u32 device_id, __u32 hwpt_id,
				  __u32 flags, __u32 type, void *data,
				  __u32 data_len, __u32 *viommu_id)
{
	struct iommu_viommu_alloc cmd = {
		.size = sizeof(cmd),
		.flags = flags,
		.type = type,
		.dev_id = device_id,
		.hwpt_id = hwpt_id,
		.data_uptr = (uint64_t)data,
		.data_len = data_len,
	};
	int ret;

	ret = ioctl(fd, IOMMU_VIOMMU_ALLOC, &cmd);
	if (ret)
		return ret;
	if (viommu_id)
		*viommu_id = cmd.out_viommu_id;
	return 0;
}

#define test_cmd_viommu_alloc(device_id, hwpt_id, type, data, data_len,      \
			      viommu_id)                                     \
	ASSERT_EQ(0, _test_cmd_viommu_alloc(self->fd, device_id, hwpt_id, 0, \
					    type, data, data_len, viommu_id))
#define test_err_viommu_alloc(_errno, device_id, hwpt_id, type, data,        \
			      data_len, viommu_id)                           \
	EXPECT_ERRNO(_errno,                                                 \
		     _test_cmd_viommu_alloc(self->fd, device_id, hwpt_id, 0, \
					    type, data, data_len, viommu_id))

static int _test_cmd_vdevice_alloc(int fd, __u32 viommu_id, __u32 idev_id,
				   __u64 virt_id, __u32 *vdev_id)
{
	struct iommu_vdevice_alloc cmd = {
		.size = sizeof(cmd),
		.dev_id = idev_id,
		.viommu_id = viommu_id,
		.virt_id = virt_id,
	};
	int ret;

	ret = ioctl(fd, IOMMU_VDEVICE_ALLOC, &cmd);
	if (ret)
		return ret;
	if (vdev_id)
		*vdev_id = cmd.out_vdevice_id;
	return 0;
}

#define test_cmd_vdevice_alloc(viommu_id, idev_id, virt_id, vdev_id)       \
	ASSERT_EQ(0, _test_cmd_vdevice_alloc(self->fd, viommu_id, idev_id, \
					     virt_id, vdev_id))
#define test_err_vdevice_alloc(_errno, viommu_id, idev_id, virt_id, vdev_id) \
	EXPECT_ERRNO(_errno,                                                 \
		     _test_cmd_vdevice_alloc(self->fd, viommu_id, idev_id,   \
					     virt_id, vdev_id))

static int _test_cmd_hw_queue_alloc(int fd, __u32 viommu_id, __u32 type,
				    __u32 idx, __u64 base_addr, __u64 length,
				    __u32 *hw_queue_id)
{
	struct iommu_hw_queue_alloc cmd = {
		.size = sizeof(cmd),
		.viommu_id = viommu_id,
		.type = type,
		.index = idx,
		.nesting_parent_iova = base_addr,
		.length = length,
	};
	int ret;

	ret = ioctl(fd, IOMMU_HW_QUEUE_ALLOC, &cmd);
	if (ret)
		return ret;
	if (hw_queue_id)
		*hw_queue_id = cmd.out_hw_queue_id;
	return 0;
}

#define test_cmd_hw_queue_alloc(viommu_id, type, idx, base_addr, len, out_qid) \
	ASSERT_EQ(0, _test_cmd_hw_queue_alloc(self->fd, viommu_id, type, idx,  \
					      base_addr, len, out_qid))
#define test_err_hw_queue_alloc(_errno, viommu_id, type, idx, base_addr, len, \
				out_qid)                                      \
	EXPECT_ERRNO(_errno,                                                  \
		     _test_cmd_hw_queue_alloc(self->fd, viommu_id, type, idx, \
					      base_addr, len, out_qid))

static int _test_cmd_veventq_alloc(int fd, __u32 viommu_id, __u32 type,
				   __u32 *veventq_id, __u32 *veventq_fd)
{
	struct iommu_veventq_alloc cmd = {
		.size = sizeof(cmd),
		.type = type,
		.veventq_depth = 2,
		.viommu_id = viommu_id,
	};
	int ret;

	ret = ioctl(fd, IOMMU_VEVENTQ_ALLOC, &cmd);
	if (ret)
		return ret;
	if (veventq_id)
		*veventq_id = cmd.out_veventq_id;
	if (veventq_fd)
		*veventq_fd = cmd.out_veventq_fd;
	return 0;
}

#define test_cmd_veventq_alloc(viommu_id, type, veventq_id, veventq_fd) \
	ASSERT_EQ(0, _test_cmd_veventq_alloc(self->fd, viommu_id, type, \
					     veventq_id, veventq_fd))
#define test_err_veventq_alloc(_errno, viommu_id, type, veventq_id,     \
			       veventq_fd)                              \
	EXPECT_ERRNO(_errno,                                            \
		     _test_cmd_veventq_alloc(self->fd, viommu_id, type, \
					     veventq_id, veventq_fd))

static int _test_cmd_trigger_vevents(int fd, __u32 dev_id, __u32 nvevents)
{
	struct iommu_test_cmd trigger_vevent_cmd = {
		.size = sizeof(trigger_vevent_cmd),
		.op = IOMMU_TEST_OP_TRIGGER_VEVENT,
		.trigger_vevent = {
			.dev_id = dev_id,
		},
	};

	while (nvevents--) {
		if (ioctl(fd, _IOMMU_TEST_CMD(IOMMU_TEST_OP_TRIGGER_VEVENT),
			  &trigger_vevent_cmd))
			return -1;
	}
	return 0;
}

#define test_cmd_trigger_vevents(dev_id, nvevents) \
	ASSERT_EQ(0, _test_cmd_trigger_vevents(self->fd, dev_id, nvevents))

static int _test_cmd_read_vevents(int fd, __u32 event_fd, __u32 nvevents,
				  __u32 virt_id, int *prev_seq)
{
	struct pollfd pollfd = { .fd = event_fd, .events = POLLIN };
	struct iommu_viommu_event_selftest *event;
	struct iommufd_vevent_header *hdr;
	ssize_t bytes;
	void *data;
	int ret, i;

	ret = poll(&pollfd, 1, 1000);
	if (ret < 0)
		return -1;

	data = calloc(nvevents, sizeof(*hdr) + sizeof(*event));
	if (!data) {
		errno = ENOMEM;
		return -1;
	}

	bytes = read(event_fd, data,
		     nvevents * (sizeof(*hdr) + sizeof(*event)));
	if (bytes <= 0) {
		errno = EFAULT;
		ret = -1;
		goto out_free;
	}

	for (i = 0; i < nvevents; i++) {
		hdr = data + i * (sizeof(*hdr) + sizeof(*event));

		if (hdr->flags & IOMMU_VEVENTQ_FLAG_LOST_EVENTS ||
		    hdr->sequence - *prev_seq > 1) {
			*prev_seq = hdr->sequence;
			errno = EOVERFLOW;
			ret = -1;
			goto out_free;
		}
		*prev_seq = hdr->sequence;
		event = data + sizeof(*hdr);
		if (event->virt_id != virt_id) {
			errno = EINVAL;
			ret = -1;
			goto out_free;
		}
	}

	ret = 0;
out_free:
	free(data);
	return ret;
}

#define test_cmd_read_vevents(event_fd, nvevents, virt_id, prev_seq)      \
	ASSERT_EQ(0, _test_cmd_read_vevents(self->fd, event_fd, nvevents, \
					    virt_id, prev_seq))
#define test_err_read_vevents(_errno, event_fd, nvevents, virt_id, prev_seq) \
	EXPECT_ERRNO(_errno,                                                 \
		     _test_cmd_read_vevents(self->fd, event_fd, nvevents,    \
					    virt_id, prev_seq))

static int _test_cmd_pasid_attach(int fd, __u32 stdev_id, __u32 pasid,
				  __u32 pt_id)
{
	struct iommu_test_cmd test_attach = {
		.size = sizeof(test_attach),
		.op = IOMMU_TEST_OP_PASID_ATTACH,
		.id = stdev_id,
		.pasid_attach = {
			.pasid = pasid,
			.pt_id = pt_id,
		},
	};

	return ioctl(fd, _IOMMU_TEST_CMD(IOMMU_TEST_OP_PASID_ATTACH),
		     &test_attach);
}

#define test_cmd_pasid_attach(pasid, hwpt_id) \
	ASSERT_EQ(0, _test_cmd_pasid_attach(self->fd, self->stdev_id, \
					    pasid, hwpt_id))

#define test_err_pasid_attach(_errno, pasid, hwpt_id) \
	EXPECT_ERRNO(_errno, \
		     _test_cmd_pasid_attach(self->fd, self->stdev_id, \
					    pasid, hwpt_id))

static int _test_cmd_pasid_replace(int fd, __u32 stdev_id, __u32 pasid,
				   __u32 pt_id)
{
	struct iommu_test_cmd test_replace = {
		.size = sizeof(test_replace),
		.op = IOMMU_TEST_OP_PASID_REPLACE,
		.id = stdev_id,
		.pasid_replace = {
			.pasid = pasid,
			.pt_id = pt_id,
		},
	};

	return ioctl(fd, _IOMMU_TEST_CMD(IOMMU_TEST_OP_PASID_REPLACE),
		     &test_replace);
}

#define test_cmd_pasid_replace(pasid, hwpt_id) \
	ASSERT_EQ(0, _test_cmd_pasid_replace(self->fd, self->stdev_id, \
					     pasid, hwpt_id))

#define test_err_pasid_replace(_errno, pasid, hwpt_id) \
	EXPECT_ERRNO(_errno, \
		     _test_cmd_pasid_replace(self->fd, self->stdev_id, \
					     pasid, hwpt_id))

static int _test_cmd_pasid_detach(int fd, __u32 stdev_id, __u32 pasid)
{
	struct iommu_test_cmd test_detach = {
		.size = sizeof(test_detach),
		.op = IOMMU_TEST_OP_PASID_DETACH,
		.id = stdev_id,
		.pasid_detach = {
			.pasid = pasid,
		},
	};

	return ioctl(fd, _IOMMU_TEST_CMD(IOMMU_TEST_OP_PASID_DETACH),
		     &test_detach);
}

#define test_cmd_pasid_detach(pasid) \
	ASSERT_EQ(0, _test_cmd_pasid_detach(self->fd, self->stdev_id, pasid))

static int test_cmd_pasid_check_hwpt(int fd, __u32 stdev_id, __u32 pasid,
				     __u32 hwpt_id)
{
	struct iommu_test_cmd test_pasid_check = {
		.size = sizeof(test_pasid_check),
		.op = IOMMU_TEST_OP_PASID_CHECK_HWPT,
		.id = stdev_id,
		.pasid_check = {
			.pasid = pasid,
			.hwpt_id = hwpt_id,
		},
	};

	return ioctl(fd, _IOMMU_TEST_CMD(IOMMU_TEST_OP_PASID_CHECK_HWPT),
		     &test_pasid_check);
}
