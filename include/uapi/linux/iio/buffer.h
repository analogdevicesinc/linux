/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* industrial I/O buffer definitions needed both in and out of kernel
 */

#ifndef _UAPI_IIO_BUFFER_H_
#define _UAPI_IIO_BUFFER_H_

#include <linux/types.h>

/**
 * See for more details:
 *   Documentation/iio/iio_high_speed_buffers.rst
 */

/**
 * See for more details:
 *   Documentation/iio/iio_high_speed_buffers.rst
 */

/**
 * struct iio_buffer_block_alloc_req - Descriptor for allocating IIO buffer blocks
 * @type:	type of block(s) to allocate (currently unused, reserved)
 * @size:	the size of a single block
 * @count:	the number of blocks to allocate
 * @id:		returned by the request, the number of blocks allocated
 */
struct iio_buffer_block_alloc_req {
	__u32 type;
	__u32 size;
	__u32 count;
	__u32 id;
};

/* A function will be assigned later for BIT(0) */
#define IIO_BUFFER_BLOCK_FLAG_RESERVED		(1 << 0)
#define IIO_BUFFER_BLOCK_FLAG_CYCLIC		(1 << 1)

/**
 * struct iio_buffer_block - Descriptor for a single IIO block
 * @id:		identifier of the block
 * @size:	size of the block
 * @bytes_used:	number of bytes used in this block by a data transfer
 * @type:	type of this block (currently unused, reserved)
 * @flags:	flags for this buffer, set when enqueuing this block
 * @offset:	data offset of this block in a larger memory segment
 * @timestamp:	timestamp for this block
 */
struct iio_buffer_block {
	__u32 id;
	__u32 size;
	__u32 bytes_used;
	__u32 type;
	__u32 flags;
	union {
		__u32 offset;
	} data;
	__u64 timestamp;
};

#define IIO_BUFFER_GET_FD_IOCTL			_IOWR('i', 0x91, int)
#define IIO_BUFFER_BLOCK_ALLOC_IOCTL		_IOWR('i', 0x92, struct iio_buffer_block_alloc_req)
#define IIO_BUFFER_BLOCK_FREE_IOCTL		_IO('i',   0x93)
#define IIO_BUFFER_BLOCK_QUERY_IOCTL		_IOWR('i', 0x93, struct iio_buffer_block)
#define IIO_BUFFER_BLOCK_ENQUEUE_IOCTL		_IOWR('i', 0x94, struct iio_buffer_block)
#define IIO_BUFFER_BLOCK_DEQUEUE_IOCTL		_IOWR('i', 0x95, struct iio_buffer_block)

#endif /* _UAPI_IIO_BUFFER_H_ */
