#ifndef _ANALOG_DRM_GEM_H_
#define _ANALOG_DRM_GEM_H_

#define to_analog_gem_obj(x)	container_of(x,\
			struct analog_drm_gem_obj, base)

/*
 * analog drm buffer structure.
 *
 * @base: a gem object.
 *	- a new handle to this gem object would be created
 *	by drm_gem_handle_create().
 * @addr: kernel virtual address to allocated memory region.
 * @dma_addr: bus address(accessed by dma) to allocated memory region.
 *	- this address could be physical address without IOMMU and
 *	device address with IOMMU.
 *
 */
struct analog_drm_gem_obj {
	struct drm_gem_object	base;
	void			*addr;
	dma_addr_t		dma_addr;
};

struct analog_drm_gem_obj *analog_drm_gem_alloc(struct drm_device *dev,
	size_t size);

/* free gem object. */
void analog_drm_gem_free_object(struct drm_gem_object *gem_obj);

/* create memory region for drm framebuffer. */
int analog_drm_gem_dumb_create(struct drm_file *file_priv,
		struct drm_device *dev, struct drm_mode_create_dumb *args);

/* map memory region for drm framebuffer to user space. */
int analog_drm_gem_dumb_map_offset(struct drm_file *file_priv,
		struct drm_device *dev, uint32_t handle, uint64_t *offset);

/* page fault handler and mmap fault address(virtual) to physical memory. */
int analog_drm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf);

/*
 * destroy memory region allocated.
 *	- a gem handle and physical memory region pointed by a gem object
 *	would be released by drm_gem_handle_delete().
 */
int analog_drm_gem_dumb_destroy(struct drm_file *file_priv,
		struct drm_device *dev, unsigned int handle);

int analog_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma);

#endif
