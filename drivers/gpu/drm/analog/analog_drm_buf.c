#include <drm/drmP.h>
#include <drm/drm.h>

#include "analog_drm_drv.h"
#include "analog_drm_gem.h"
#include "analog_drm_buf.h"

static int lowlevel_buffer_allocate(struct drm_device *dev,
		struct analog_drm_gem_buf *buffer)
{

	buffer->kvaddr = dma_alloc_writecombine(dev->dev, buffer->size,
			&buffer->dma_addr, GFP_KERNEL);
	if (!buffer->kvaddr) {
		DRM_ERROR("failed to allocate buffer. %ld\n", buffer->size);
		return -ENOMEM;
	}

	DRM_DEBUG_KMS("vaddr(0x%lx), dma_addr(0x%lx), size(0x%lx)\n",
			(unsigned long)buffer->kvaddr,
			(unsigned long)buffer->dma_addr,
			buffer->size);

	return 0;
}

static void lowlevel_buffer_deallocate(struct drm_device *dev,
		struct analog_drm_gem_buf *buffer)
{

	if (buffer->dma_addr && buffer->size)
		dma_free_coherent(dev->dev, buffer->size, buffer->kvaddr,
				(dma_addr_t)buffer->dma_addr);
	else
		DRM_DEBUG_KMS("buffer data are invalid.\n");
}

struct analog_drm_gem_buf *analog_drm_buf_create(struct drm_device *dev,
		unsigned int size)
{
	struct analog_drm_gem_buf *buffer;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		DRM_ERROR("failed to allocate analog_drm_gem_buf.\n");
		return ERR_PTR(-ENOMEM);
	}

	buffer->size = size;

	/*
	 * allocate memory region with size and set the memory information
	 * to vaddr and dma_addr of a buffer object.
	 */
	if (lowlevel_buffer_allocate(dev, buffer) < 0) {
		kfree(buffer);
		buffer = NULL;
		return ERR_PTR(-ENOMEM);
	}

	return buffer;
}

void analog_drm_buf_destroy(struct drm_device *dev,
		struct analog_drm_gem_buf *buffer)
{
	if (!buffer) {
		DRM_DEBUG_KMS("buffer is null.\n");
		return;
	}

	lowlevel_buffer_deallocate(dev, buffer);

	kfree(buffer);
	buffer = NULL;
}
