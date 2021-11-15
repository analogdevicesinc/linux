===================================
High-speed DMABUF interface for IIO
===================================

1. Overview
===========

The Industrial I/O subsystem supports access to buffers through a
file-based interface, with read() and write() access calls through the
IIO device's dev node.

It additionally supports a DMABUF based interface, where the userspace
application can append DMABUF objects to the buffer's queue.
This interface is however optional and is not available in all drivers.

The advantage of this DMABUF based interface vs. the read()
interface, is that it avoids an extra copy of the data between the
kernel and userspace. This is particularly useful for high-speed
devices which produce several megabytes or even gigabytes of data per
second.

The data in this DMABUF interface is managed at the granularity of
DMABUF objects. Reducing the granularity from byte level to block level
is done to reduce the userspace-kernelspace synchronization overhead
since performing syscalls for each byte at a few Mbps is just not
feasible.

This of course leads to a slightly increased latency. However, the
application is free to choose how big its DMABUFs will be, and how
many should be used.

2. User API
===========

``IIO_BUFFER_DMABUF_ATTACH_IOCTL(int)``
----------------------------------------------------------------

Attach the DMABUF object, identified by its file descriptor, to the IIO
buffer. Returns zero on success, and a negative errno value on error.

``IIO_BUFFER_DMABUF_DETACH_IOCTL(int)``
--------------------------------------------------------

Detach the given DMABUF object, identified by its file descriptor, from
the IIO buffer. Returns zero on success, and a negative errno value on
error.

Note that closing the IIO buffer's file descriptor will automatically
detach all previously attached DMABUF objects.

``IIO_BUFFER_DMABUF_ENQUEUE_IOCTL(struct iio_dmabuf *iio_dmabuf)``
--------------------------------------------------------

Enqueue a previously attached DMABUF object to the buffer queue.
Enqueued DMABUFs will be read from (if output buffer) or written to
(if input buffer) as long as the buffer is enabled.

3. Usage
========

To access the data stored in a block by userspace the block must be
mapped to the process's memory. This is done by calling mmap() on the
DMABUF's file descriptor.

Before accessing the data through the map, you must use the
DMA_BUF_IOCTL_SYNC(struct dma_buf_sync *) ioctl, with the
DMA_BUF_SYNC_START flag, to make sure that the data is available.
This call may block until the hardware is done with this block. Once
you are done reading or writing the data, you must use this ioctl again
with the DMA_BUF_SYNC_END flag, before enqueueing the DMABUF to the
kernel's queue.

If you need to know when the hardware is done with a DMABUF, you can
poll its file descriptor for the POLLOUT event.

Finally, to destroy a DMABUF object, simply call close() on its file
descriptor.

For more information about manipulating DMABUF objects, see: :ref:`dma-buf`.

A typical workflow for the new interface is:

    enable buffer

    while !done
      for dmabuf in dmabufs:
        DMABUF_ENQUEUE dmabuf

        DMABUF_SYNC_START dmabuf
        process data
        DMABUF_SYNC_END dmabuf

    disable buffer

    for dmabuf in dmabufs:
      close dmabuf
