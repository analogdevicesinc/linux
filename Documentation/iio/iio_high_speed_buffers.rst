===================================
Industrial IO High-Speed Buffer API
===================================

1. Overview
===========

Industrial IO supports access to buffers via an mmap interface. The
advantage of the mmap based interface compared to the read() based
interface is that it avoids an extra copy of the data between kernel and
userspace. This is particular useful for high-speed devices which produce
several megabytes or even gigabytes of data per second.

The data for the mmap interface is managed at the granularity of so called
blocks. A block is a contiguous region of memory (at the moment both
physically and virtually contiguous). Reducing the granularity from byte
level to block level is done to reduce the userspace-kernelspace
synchronization overhead since performing syscalls for each byte at a
data-rate of a few megabytes is not feasible.

This of course leads to a slightly increased latency. For this reason an
application can choose the size of the blocks as well as how many blocks it
allocates. E.g. two blocks would be a traditional double buffering scheme.
But using a higher number might be necessary to avoid underflow/overflow
situations in the presence of scheduling latencies.

A block can either be owned by kernel space or userspace. When owned by
userspace it is safe to access the data in the block and process it. When
owned by kernel space the block can be in one of 3 states:

* It can be in the incoming queue where all blocks submitted from userspace
  are placed and are waiting to be processed by the kernel driver.
* It can be currently being processed by the kernel driver, this means it is
  actively placing capturing data in it (usually using DMA).
* Or it can be in the outgoing queue where all blocks that have been
  processed by the kernel are placed. Userspace can dequeue the blocks as
  necessary.

2. Interface
============

As part of the interface 5 IOCTLs are used to manage the blocks and exchange
them between userspace and kernelspace. The IOCTLs can be accessed through
a open file descriptor to a IIO device.

* **IIO_BUFFER_BLOCK_ALLOC_IOCTL(struct iio_buffer_block_alloc_req *)**:
    Allocates new blocks. Can be called multiple times if necessary. A newly
    allocated block is initially owned by userspace.

* **IIO_BUFFER_BLOCK_FREE_IOCTL(void)**:
   Frees all previously allocated blocks. If the backing memory of a block is
   still in use by a kernel driver (i.e. active DMA transfer) it will be
   freed once the kernel driver has released it.

* **IIO_BUFFER_BLOCK_QUERY_IOCTL(struct iio_buffer_block *)**:
   Queries information about a block. The id of the block about which
   information is to be queried needs to be set by userspace.

* **IIO_BUFFER_BLOCK_ENQUEUE_IOCTL(struct iio_buffer_block *)**:
   Places a block on the incoming queue. This transfers ownership of the
   block from userspace to kernelspace. Userspace must populate the id field
   of the block to indicate which block to enqueue.

* **IIO_BUFFER_BLOCK_DEQUEUE_IOCTL(struct iio_buffer_block *)**:
   Removes the first block from the outgoing queue. This transfers ownership
   of the block from kernelspace to userspace. Kernelspace will populate all
   fields of the block. If the queue is empty and the file descriptor is set
   to blocking the IOCTL will block until a new block is available on the
   outgoing queue.

3. Usage
========

To access the data stored in a block by userspace the block must be mapped
to the process's memory. This is done by calling mmap() on the IIO device
file descriptor. Each block has a unique offset assigned to it which should
be passed to the mmap interface. E.g.

  mmap(0, block.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
       block.offset);

A typical workflow for the new interface is:

  BLOCK_ALLOC

  foreach block
     BLOCK_QUERY block
	 mmap block.data.offset
	 BLOCK_ENQUEUE block

  enable buffer

  while !done
	BLOCK_DEQUEUE block
	process data
	BLOCK_ENQUEUE block

  disable buffer

  BLOCK_FREE
