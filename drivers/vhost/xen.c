// SPDX-License-Identifier: GPL-2.0-only
/*
 * A specific module for accessing descriptors in virtio rings which contain
 * either usual guest pseudo-physical addresses or guest grant based addresses.
 * Depending on the descriptor's nature we use either Xen foreign mappings or
 * Xen grant mappings to map/unmap an underlying guest page.
 * Please see Xen grant DMA-mapping layer at drivers/xen/grant-dma-ops.c
 * which is the origin of Xen grant mappings scheme.
 *
 * Copyright (C) 2023 EPAM Systems Inc.
 * Copyright 2024 NXP
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vhost.h>
#include <linux/vmalloc.h>
#include <xen/grant_table.h>
#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/interface/memory.h>
#include <asm/xen/hypercall.h>

#include "vhost.h"

static domid_t guest_domid = DOMID_INVALID;

static bool nogrant = true;
module_param(nogrant, bool, 0444);
MODULE_PARM_DESC(nogrant, "Disable Xen grant mappings");

struct vhost_xen_map {
	struct list_head next;
	int count;
	grant_handle_t *handles;
	domid_t domid;
	struct page **pages;
	unsigned long vaddr;
};

struct vhost_xen_ops {
	struct vhost_xen_map *(*alloc_map)(int count);
	void (*free_map)(struct vhost_xen_map *map);
	int (*map_pages)(struct vhost_xen_map *map, u64 gpaddr, bool readonly);
	int (*unmap_pages)(struct vhost_xen_map *map);
};

static const struct vhost_xen_ops *vhost_xen_ops;

static void vhost_xen_grant_free_map(struct vhost_xen_map *map)
{
	if (!map)
		return;

	if (map->pages)
		gnttab_free_pages(map->count, map->pages);

	kvfree(map->pages);
	kvfree(map->handles);
	kfree(map);
}

static struct vhost_xen_map *vhost_xen_grant_alloc_map(int count)
{
	struct vhost_xen_map *map;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return NULL;

	map->handles = kvcalloc(count, sizeof(map->handles[0]), GFP_KERNEL);
	map->pages = kvcalloc(count, sizeof(map->pages[0]), GFP_KERNEL);
	if (!map->handles || !map->pages)
		goto err;

	if (gnttab_alloc_pages(count, map->pages))
		goto err;

	map->count = count;

	return map;

err:
	vhost_xen_grant_free_map(map);

	return NULL;
}

static int vhost_xen_grant_map_pages(struct vhost_xen_map *map, u64 gpaddr,
		bool readonly)
{
	struct gnttab_map_grant_ref *map_ops;
	int i, ret;
	uint32_t flags = GNTMAP_host_map;

	map_ops = kvcalloc(map->count, sizeof(map_ops[0]), GFP_KERNEL);
	if (!map_ops)
		return -ENOMEM;

	if (readonly)
		flags |= GNTMAP_readonly;

	for (i = 0; i < map->count; i++) {
		unsigned long vaddr = (unsigned long)
			pfn_to_kaddr(page_to_xen_pfn(map->pages[i]));
		grant_ref_t ref = XEN_PFN_DOWN(gpaddr & ~XEN_GRANT_DMA_ADDR_OFF) + i;

		gnttab_set_map_op(&map_ops[i], vaddr, flags, ref, map->domid);
		map->handles[i] = -1;
	}

	ret = gnttab_map_refs(map_ops, NULL, map->pages, map->count);
	for (i = 0; i < map->count; i++) {
		if (map_ops[i].status == GNTST_okay)
			map->handles[i] = map_ops[i].handle;
		else if (!ret)
			ret = -EINVAL;
	}

	kvfree(map_ops);

	return ret;
}

static int vhost_xen_grant_unmap_pages(struct vhost_xen_map *map)
{
	struct gnttab_unmap_grant_ref *unmap_ops;
	int i, ret;

	unmap_ops = kvcalloc(map->count, sizeof(unmap_ops[0]), GFP_KERNEL);
	if (!unmap_ops)
		return -ENOMEM;

	for (i = 0; i < map->count; i++) {
		unsigned long vaddr = (unsigned long)
			pfn_to_kaddr(page_to_xen_pfn(map->pages[i]));

		gnttab_set_unmap_op(&unmap_ops[i], vaddr, GNTMAP_host_map,
				map->handles[i]);
	}

	ret = gnttab_unmap_refs(unmap_ops, NULL, map->pages, map->count);
	if (ret) {
		kvfree(unmap_ops);
		return ret;
	}

	for (i = 0; i < map->count; i++) {
		if (unmap_ops[i].status != GNTST_okay)
			ret = -EINVAL;
		map->handles[i] = -1;
	}

	kvfree(unmap_ops);

	return ret;
}

static void vhost_xen_foreign_free_map(struct vhost_xen_map *map)
{
	if (!map)
		return;

	if (map->pages)
		xen_free_unpopulated_pages(map->count, map->pages);

	kvfree(map->pages);
	kfree(map);
}

static struct vhost_xen_map *vhost_xen_foreign_alloc_map(int count)
{
	struct vhost_xen_map *map;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return NULL;

	map->pages = kvcalloc(count, sizeof(map->pages[0]), GFP_KERNEL);
	if (!map->pages)
		goto err;

	if (xen_alloc_unpopulated_pages(count, map->pages))
		goto err;

	map->count = count;

	return map;

err:
	vhost_xen_foreign_free_map(map);

	return NULL;
}

static int vhost_xen_foreign_map_pages(struct vhost_xen_map *map, u64 gpaddr,
		bool readonly)
{
	xen_pfn_t *gpfns;
	xen_ulong_t *idxs;
	int *errs;
	int i, ret;

	struct xen_add_to_physmap_range xatp = {
		.domid = DOMID_SELF,
		.foreign_domid = map->domid,
		.space = XENMAPSPACE_gmfn_foreign,
	};

	gpfns = kvcalloc(map->count, sizeof(xen_pfn_t), GFP_KERNEL);
	idxs = kvcalloc(map->count, sizeof(xen_ulong_t), GFP_KERNEL);
	errs = kvcalloc(map->count, sizeof(int), GFP_KERNEL);
	if (!gpfns || !idxs || !errs) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < map->count; i++) {
		xen_pfn_t pfn = page_to_xen_pfn(map->pages[i / XEN_PFN_PER_PAGE]);
		xen_ulong_t idx = XEN_PFN_DOWN(gpaddr) + i;

		gpfns[i] = pfn + (i % XEN_PFN_PER_PAGE);
		idxs[i] = idx;
		errs[i] = 0;
	}

	xatp.size = map->count;
	set_xen_guest_handle(xatp.gpfns, gpfns);
	set_xen_guest_handle(xatp.idxs, idxs);
	set_xen_guest_handle(xatp.errs, errs);

	ret = HYPERVISOR_memory_op(XENMEM_add_to_physmap_range, &xatp);
	for (i = 0; i < map->count; i++) {
		if (errs[i] && !ret)
			ret = errs[i];
	}

out:
	kvfree(gpfns);
	kvfree(idxs);
	kvfree(errs);

	return ret;
}

static int vhost_xen_foreign_unmap_pages(struct vhost_xen_map *map)
{
	int i, ret = 0;
	struct xen_remove_from_physmap xrp;

	for (i = 0; i < map->count; i++) {
		xen_pfn_t pfn = page_to_xen_pfn(map->pages[i / XEN_PFN_PER_PAGE]);

		xrp.domid = DOMID_SELF;
		xrp.gpfn = pfn + (i % XEN_PFN_PER_PAGE);

		ret = HYPERVISOR_memory_op(XENMEM_remove_from_physmap, &xrp);
		if (ret)
			return ret;
	}

	return ret;
}

static void vhost_xen_put_map(struct vhost_xen_map *map)
{
	if (!map)
		return;

	if (map->vaddr) {
		if (map->count > 1)
			vunmap((void *)map->vaddr);
		map->vaddr = 0;
	}

	if (map->pages) {
		int ret;

		ret = vhost_xen_ops->unmap_pages(map);
		if (ret)
			pr_err("%s: Failed to unmap pages from dom%d (ret=%d)\n",
					__func__, map->domid, ret);
	}
	vhost_xen_ops->free_map(map);
}

static struct vhost_xen_map *vhost_xen_find_map(struct vhost_virtqueue *vq,
		unsigned long vaddr, int count)
{
	struct vhost_xen_map *map;

	list_for_each_entry(map, &vq->desc_maps, next) {
		if (map->vaddr != vaddr)
			continue;
		if (count && map->count != count)
			continue;
		return map;
	}

	return NULL;
}

void vhost_xen_unmap_desc_all(struct vhost_virtqueue *vq)
{
	struct vhost_xen_map *map;

	if (!xen_domain())
		return;

	while (!list_empty(&vq->desc_maps)) {
		map = list_entry(vq->desc_maps.next, struct vhost_xen_map, next);
		list_del(&map->next);

		pr_debug("%s: dom%d: vaddr 0x%lx count %u\n",
				__func__, map->domid, map->vaddr, map->count);
		vhost_xen_put_map(map);
	}
}

void *vhost_xen_map_desc(struct vhost_virtqueue *vq, u64 addr, u32 size,
		int access)
{
	struct vhost_xen_map *map;
	unsigned long offset = xen_offset_in_page(addr);
	int count = XEN_PFN_UP(offset + size);
	int ret;

	if (!xen_domain() || guest_domid == DOMID_INVALID)
		return ERR_PTR(-ENODEV);

	if ((nogrant && (addr & XEN_GRANT_DMA_ADDR_OFF)) ||
			(!nogrant && !(addr & XEN_GRANT_DMA_ADDR_OFF))) {
		pr_err("%s: Descriptor from dom%d cannot be mapped via Xen %s mappings (addr 0x%llx)\n",
				__func__, guest_domid, nogrant ? "foreign" : "grant", addr);
		return ERR_PTR(-EINVAL);
	}

	map = vhost_xen_ops->alloc_map(count);
	if (!map)
		return ERR_PTR(-ENOMEM);

	map->domid = guest_domid;
	ret = vhost_xen_ops->map_pages(map, addr, access == VHOST_ACCESS_RO);
	if (ret) {
		pr_err("%s: Failed to map pages from dom%d (ret=%d)\n",
				__func__, map->domid, ret);
		vhost_xen_ops->free_map(map);
		return ERR_PTR(ret);
	}

	/*
	 * Consider allocating xen_alloc_unpopulated_contiguous_pages() instead of
	 * xen_alloc_unpopulated_pages() to avoid mapping as with the later
	 * map->pages are not guaranteed to be contiguous.
	 */
	if (map->count > 1) {
		map->vaddr = (unsigned long)vmap(map->pages, map->count, VM_MAP,
				PAGE_KERNEL);
		if (!map->vaddr) {
			pr_err("%s: Failed to create virtual mappings\n", __func__);
			vhost_xen_put_map(map);
			return ERR_PTR(-ENOMEM);
		}
	} else
		map->vaddr = (unsigned long)pfn_to_kaddr(page_to_xen_pfn(map->pages[0]));

	list_add_tail(&map->next, &vq->desc_maps);

	pr_debug("%s: dom%d: addr 0x%llx size 0x%x (access 0x%x) -> vaddr 0x%lx count %u (paddr 0x%llx)\n",
			__func__, map->domid, addr, size, access, map->vaddr, map->count,
			page_to_phys(map->pages[0]));

	return (void *)(map->vaddr + offset);
}

void vhost_xen_unmap_desc(struct vhost_virtqueue *vq, void *ptr, u32 size)
{
	struct vhost_xen_map *map;
	unsigned long offset = xen_offset_in_page(ptr);
	int count = XEN_PFN_UP(offset + size);

	if (!xen_domain())
		return;

	map = vhost_xen_find_map(vq, (unsigned long)ptr & XEN_PAGE_MASK, count);
	if (map) {
		list_del(&map->next);

		pr_debug("%s: dom%d: vaddr 0x%lx count %u\n",
				__func__, map->domid, map->vaddr, map->count);
		vhost_xen_put_map(map);
	}
}

static const struct vhost_xen_ops vhost_xen_grant_ops = {
	.alloc_map = vhost_xen_grant_alloc_map,
	.free_map = vhost_xen_grant_free_map,
	.map_pages = vhost_xen_grant_map_pages,
	.unmap_pages = vhost_xen_grant_unmap_pages,
};

static const struct vhost_xen_ops vhost_xen_foreign_ops = {
	.alloc_map = vhost_xen_foreign_alloc_map,
	.free_map = vhost_xen_foreign_free_map,
	.map_pages = vhost_xen_foreign_map_pages,
	.unmap_pages = vhost_xen_foreign_unmap_pages,
};

static void vhost_xen_get_guest_domid(struct xenbus_watch *watch,
		const char *path, const char *token)
{
	char **dm_dir;
	unsigned int n = 0;

	dm_dir = xenbus_directory(XBT_NIL, "device-model", "", &n);
	if (IS_ERR(dm_dir))
		return;

	if (n == 0) {
		if (guest_domid != DOMID_INVALID) {
			guest_domid = DOMID_INVALID;
			pr_info("%s: Reset current domid\n", __func__);
		}
	} else if (n == 1) {
		char *dm_str, *dm_path;
		domid_t domid;

		domid = simple_strtoul(dm_dir[0], NULL, 0);
		dm_path = kasprintf(GFP_KERNEL, "device-model/%d", domid);
		if (!dm_path)
			goto out;

		dm_str = (char *)xenbus_read(XBT_NIL, dm_path, "state", NULL);
		if (IS_ERR(dm_str)) {
			kfree(dm_path);
			goto out;
		}

		if (!strcmp(dm_str, "running")) {
			guest_domid = domid;
			pr_info("%s: Set new domid: %u\n", __func__, guest_domid);
		}

		kfree(dm_path);
		kfree(dm_str);
	}

out:
	kfree(dm_dir);
}

static struct xenbus_watch vhost_xen_qemu_args = {
	.node = "device-model",
	.callback = vhost_xen_get_guest_domid,
};

static int vhost_xen_watcher(struct notifier_block *notifier,
		unsigned long event, void *data)
{
	int ret;

	ret = register_xenbus_watch(&vhost_xen_qemu_args);
	if (ret)
		pr_err("%s: Failed to set watcher (ret=%d)\n", __func__, ret);

	return NOTIFY_DONE;
}

static struct notifier_block vhost_xen_notifier = {
	.notifier_call = vhost_xen_watcher,
};

static int __init vhost_xen_init(void)
{
	if (!xen_domain())
		return -ENODEV;

	register_xenstore_notifier(&vhost_xen_notifier);

	vhost_xen_ops = nogrant ? &vhost_xen_foreign_ops : &vhost_xen_grant_ops;

	pr_info("%s: Initialize module for Xen %s mappings\n", __func__,
			nogrant ? "foreign" : "grant");

	return 0;
}

static void __exit vhost_xen_exit(void)
{

}

module_init(vhost_xen_init);
module_exit(vhost_xen_exit);

MODULE_DESCRIPTION("Xen specific mappings for vhost");
MODULE_AUTHOR("Oleksandr Tyshchenko <oleksandr_tyshchenko@epam.com>");
MODULE_LICENSE("GPL");
