/*
 * Copyright 2012 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Ben Skeggs
 */
#include "priv.h"

#include <subdev/bar.h>

/******************************************************************************
 * instmem object base implementation
 *****************************************************************************/
void
nvkm_instobj_load(struct nvkm_instobj *iobj)
{
	struct nvkm_memory *memory = &iobj->memory;
	const u64 size = nvkm_memory_size(memory);
	void __iomem *map;
	int i;

	if (!(map = nvkm_kmap(memory))) {
		for (i = 0; i < size; i += 4)
			nvkm_wo32(memory, i, iobj->suspend[i / 4]);
	} else {
		memcpy_toio(map, iobj->suspend, size);
	}
	nvkm_done(memory);

	kvfree(iobj->suspend);
	iobj->suspend = NULL;
}

int
nvkm_instobj_save(struct nvkm_instobj *iobj)
{
	struct nvkm_memory *memory = &iobj->memory;
	const u64 size = nvkm_memory_size(memory);
	void __iomem *map;
	int i;

	iobj->suspend = kvmalloc(size, GFP_KERNEL);
	if (!iobj->suspend)
		return -ENOMEM;

	if (!(map = nvkm_kmap(memory))) {
		for (i = 0; i < size; i += 4)
			iobj->suspend[i / 4] = nvkm_ro32(memory, i);
	} else {
		memcpy_fromio(iobj->suspend, map, size);
	}
	nvkm_done(memory);
	return 0;
}

void
nvkm_instobj_dtor(struct nvkm_instmem *imem, struct nvkm_instobj *iobj)
{
	spin_lock(&imem->lock);
	list_del(&iobj->head);
	spin_unlock(&imem->lock);
}

void
nvkm_instobj_ctor(const struct nvkm_memory_func *func,
		  struct nvkm_instmem *imem, struct nvkm_instobj *iobj)
{
	nvkm_memory_ctor(func, &iobj->memory);
	iobj->suspend = NULL;
	spin_lock(&imem->lock);
	list_add_tail(&iobj->head, &imem->list);
	spin_unlock(&imem->lock);
}

int
nvkm_instobj_wrap(struct nvkm_device *device,
		  struct nvkm_memory *memory, struct nvkm_memory **pmemory)
{
	struct nvkm_instmem *imem = device->imem;
	int ret;

	if (!imem->func->memory_wrap)
		return -ENOSYS;

	ret = imem->func->memory_wrap(imem, memory, pmemory);
	if (ret)
		return ret;

	container_of(*pmemory, struct nvkm_instobj, memory)->preserve = true;
	return 0;
}

int
nvkm_instobj_new(struct nvkm_instmem *imem, u32 size, u32 align, bool zero, bool preserve,
		 struct nvkm_memory **pmemory)
{
	struct nvkm_subdev *subdev = &imem->subdev;
	struct nvkm_memory *memory = NULL;
	u32 offset;
	int ret;

	ret = imem->func->memory_new(imem, size, align, zero, &memory);
	if (ret) {
		nvkm_error(subdev, "OOM: %08x %08x %d\n", size, align, ret);
		goto done;
	}

	nvkm_trace(subdev, "new %08x %08x %d: %010llx %010llx\n", size, align,
		   zero, nvkm_memory_addr(memory), nvkm_memory_size(memory));

	if (!imem->func->zero && zero) {
		void __iomem *map = nvkm_kmap(memory);
		if (unlikely(!map)) {
			for (offset = 0; offset < size; offset += 4)
				nvkm_wo32(memory, offset, 0x00000000);
		} else {
			memset_io(map, 0x00, size);
		}
		nvkm_done(memory);
	}

	container_of(memory, struct nvkm_instobj, memory)->preserve = preserve;
done:
	if (ret)
		nvkm_memory_unref(&memory);
	*pmemory = memory;
	return ret;
}

/******************************************************************************
 * instmem subdev base implementation
 *****************************************************************************/

u32
nvkm_instmem_rd32(struct nvkm_instmem *imem, u32 addr)
{
	return imem->func->rd32(imem, addr);
}

void
nvkm_instmem_wr32(struct nvkm_instmem *imem, u32 addr, u32 data)
{
	return imem->func->wr32(imem, addr, data);
}

void
nvkm_instmem_boot(struct nvkm_instmem *imem)
{
	/* Separate bootstrapped objects from normal list, as we need
	 * to make sure they're accessed with the slowpath on suspend
	 * and resume.
	 */
	struct nvkm_instobj *iobj, *itmp;
	spin_lock(&imem->lock);
	list_for_each_entry_safe(iobj, itmp, &imem->list, head) {
		list_move_tail(&iobj->head, &imem->boot);
	}
	spin_unlock(&imem->lock);
}

static int
nvkm_instmem_fini(struct nvkm_subdev *subdev, bool suspend)
{
	struct nvkm_instmem *imem = nvkm_instmem(subdev);
	int ret;

	if (suspend) {
		ret = imem->func->suspend(imem);
		if (ret)
			return ret;

		imem->suspend = true;
	}

	if (imem->func->fini)
		imem->func->fini(imem);

	return 0;
}

static int
nvkm_instmem_init(struct nvkm_subdev *subdev)
{
	struct nvkm_instmem *imem = nvkm_instmem(subdev);

	if (imem->suspend) {
		if (imem->func->resume)
			imem->func->resume(imem);

		imem->suspend = false;
		return 0;
	}

	nvkm_bar_bar2_init(subdev->device);
	return 0;
}

static int
nvkm_instmem_oneinit(struct nvkm_subdev *subdev)
{
	struct nvkm_instmem *imem = nvkm_instmem(subdev);
	if (imem->func->oneinit)
		return imem->func->oneinit(imem);
	return 0;
}

static void *
nvkm_instmem_dtor(struct nvkm_subdev *subdev)
{
	struct nvkm_instmem *imem = nvkm_instmem(subdev);
	void *data = imem;
	if (imem->func->dtor)
		data = imem->func->dtor(imem);
	mutex_destroy(&imem->mutex);
	return data;
}

static const struct nvkm_subdev_func
nvkm_instmem = {
	.dtor = nvkm_instmem_dtor,
	.oneinit = nvkm_instmem_oneinit,
	.init = nvkm_instmem_init,
	.fini = nvkm_instmem_fini,
};

void
nvkm_instmem_ctor(const struct nvkm_instmem_func *func, struct nvkm_device *device,
		  enum nvkm_subdev_type type, int inst, struct nvkm_instmem *imem)
{
	nvkm_subdev_ctor(&nvkm_instmem, device, type, inst, &imem->subdev);
	imem->func = func;
	spin_lock_init(&imem->lock);
	INIT_LIST_HEAD(&imem->list);
	INIT_LIST_HEAD(&imem->boot);
	mutex_init(&imem->mutex);
}
