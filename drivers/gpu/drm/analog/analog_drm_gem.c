#include <drm/drmP.h>
#include <drm/drm.h>

#include <linux/module.h>

#include "analog_drm_drv.h"
#include "analog_drm_gem.h"

static unsigned int convert_to_vm_err_msg(int msg)
{
	unsigned int out_msg;

	switch (msg) {
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
		out_msg = VM_FAULT_NOPAGE;
		break;

	case -ENOMEM:
		out_msg = VM_FAULT_OOM;
		break;

	default:
		out_msg = VM_FAULT_SIGBUS;
		break;
	}

	return out_msg;
}

static unsigned int get_gem_mmap_offset(struct drm_gem_object *obj)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	return (unsigned int)obj->map_list.hash.key << PAGE_SHIFT;
}

struct analog_drm_gem_obj *analog_drm_gem_alloc(struct drm_device *dev,
	size_t size)
{
	struct analog_drm_gem_obj *analog_gem_obj;
	struct drm_gem_object *obj;
	dma_addr_t dma_addr;
	void *addr;
	int ret;

	size = roundup(size, PAGE_SIZE);

	addr = dma_alloc_writecombine(dev->dev, size, &dma_addr, GFP_KERNEL);
	if (!addr)
	    return ERR_PTR(-ENOMEM);

	analog_gem_obj = kzalloc(sizeof(*analog_gem_obj), GFP_KERNEL);
	if (!analog_gem_obj) {
		DRM_ERROR("failed to allocate analog gem object.\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	analog_gem_obj->addr = addr;
	analog_gem_obj->dma_addr = dma_addr;

	obj = &analog_gem_obj->base;

	ret = drm_gem_object_init(dev, obj, size);
	if (ret < 0) {
		DRM_ERROR("failed to initialize gem object.\n");
		ret = -EINVAL;
		goto err_object_init;
	}

	return analog_gem_obj;

err_object_init:
	kfree(analog_gem_obj);
err_alloc:
	dma_free_coherent(dev->dev, size, addr, dma_addr);

	return ERR_PTR(ret);
}

static struct analog_drm_gem_obj *analog_drm_gem_create(struct drm_device *dev,
	struct drm_file *file, unsigned int *handle, unsigned long size)
{
	struct analog_drm_gem_obj *obj;
	int ret;

	obj = analog_drm_gem_alloc(dev, size);
	if (IS_ERR(obj))
		return obj;

	ret = drm_gem_handle_create(file, &obj->base, handle);
	if (ret) {
		drm_gem_object_release(&obj->base);
		kfree(obj);
		return ERR_PTR(ret);
	}

	drm_gem_object_unreference(&obj->base);
	return 0;
}

void analog_drm_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct analog_drm_gem_obj *analog_gem_obj = to_analog_gem_obj(gem_obj);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	DRM_DEBUG_KMS("handle count = %d\n",
			atomic_read(&gem_obj->handle_count));

	if (gem_obj->map_list.map)
		drm_gem_free_mmap_offset(gem_obj);

	/* release file pointer to gem object. */
	drm_gem_object_release(gem_obj);


	dma_free_coherent(gem_obj->dev->dev, analog_gem_obj->base.size,
	    analog_gem_obj->addr, analog_gem_obj->dma_addr);

	kfree(analog_gem_obj);
}

int analog_drm_gem_dumb_create(struct drm_file *file_priv,
		struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	struct analog_drm_gem_obj *analog_gem_obj;

	args->pitch = args->width * args->bpp >> 3;
	args->size = args->pitch * args->height;

	analog_gem_obj = analog_drm_gem_create(dev, file_priv, &args->handle,
							args->size);
	if (IS_ERR(analog_gem_obj))
		return PTR_ERR(analog_gem_obj);

	return 0;
}

int analog_drm_gem_dumb_map_offset(struct drm_file *file_priv,
		struct drm_device *dev, uint32_t handle, uint64_t *offset)
{
	struct analog_drm_gem_obj *analog_gem_obj;
	struct drm_gem_object *obj;
	int ret = 0;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	mutex_lock(&dev->struct_mutex);

	obj = drm_gem_object_lookup(dev, file_priv, handle);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	if (!obj->map_list.map) {
		ret = drm_gem_create_mmap_offset(obj);
		if (ret)
			goto out;
	}

	analog_gem_obj = to_analog_gem_obj(obj);

	*offset = get_gem_mmap_offset(&analog_gem_obj->base);
	DRM_DEBUG_KMS("offset = 0x%lx\n", (unsigned long)*offset);

out:
	drm_gem_object_unreference(obj);


	mutex_unlock(&dev->struct_mutex);

	return ret;
}

int analog_drm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct drm_gem_object *obj = vma->vm_private_data;
	struct analog_drm_gem_obj *analog_gem_obj = to_analog_gem_obj(obj);
	struct drm_device *dev = obj->dev;
	unsigned int i, page_num;
	unsigned long address;
	unsigned long pfn;
	pgoff_t page_offset;
	int ret = 0;

	page_num = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	page_offset = ((unsigned long)vmf->virtual_address -
			vma->vm_start) >> PAGE_SHIFT;

	mutex_lock(&dev->struct_mutex);

	pfn = (((unsigned long)analog_gem_obj->dma_addr) >>
			PAGE_SHIFT) + page_offset;

	address = (unsigned long)vmf->virtual_address;

	for (i = 0; i < page_num; i++) {
		ret = vm_insert_mixed(vma, address, pfn);
		if (ret == -EBUSY || (ret != 0 && i > 0))
			break;
		else if (ret)
			return (ret == -ENOMEM) ? VM_FAULT_OOM : VM_FAULT_SIGBUS;
		address += PAGE_SIZE;
		pfn++;
	}

	mutex_unlock(&dev->struct_mutex);

	return convert_to_vm_err_msg(ret);
}

int analog_drm_gem_dumb_destroy(struct drm_file *file_priv,
		struct drm_device *dev, unsigned int handle)
{
	return drm_gem_handle_delete(file_priv, handle);
}

int analog_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* set vm_area_struct. */
	ret = drm_gem_mmap(filp, vma);
	if (ret < 0) {
		DRM_ERROR("failed to mmap.\n");
		return ret;
	}

	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_flags |= VM_MIXEDMAP;

	return ret;
}

MODULE_LICENSE("GPL");
