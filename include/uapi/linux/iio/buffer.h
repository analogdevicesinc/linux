/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* industrial I/O buffer definitions needed both in and out of kernel
 */

#ifndef _UAPI_IIO_BUFFER_H_
#define _UAPI_IIO_BUFFER_H_

#include <linux/types.h>

/**
 * struct iio_dmabuf_alloc_req - Descriptor for allocating IIO DMABUFs
 * @size:	the size of a single DMABUF
 * @resv:	reserved
 */
struct iio_dmabuf_alloc_req {
	__u64 size;
	__u64 resv;
};

/**
 * struct iio_dmabuf - Descriptor for a single IIO DMABUF object
 * @fd:		file descriptor of the DMABUF object
 * @resv:	reserved
 * @bytes_used:	number of bytes used in this DMABUF for the data transfer.
 *		If zero, the full buffer is used.
 */
struct iio_dmabuf {
	__u32 fd;
	__u32 resv;
	__u64 bytes_used;
};

#define IIO_BUFFER_GET_FD_IOCTL			_IOWR('i', 0x91, int)
#define IIO_BUFFER_DMABUF_ALLOC_IOCTL		_IOW('i', 0x92, struct iio_dmabuf_alloc_req)
#define IIO_BUFFER_DMABUF_ENQUEUE_IOCTL		_IOW('i', 0x93, struct iio_dmabuf)

#endif /* _UAPI_IIO_BUFFER_H_ */
