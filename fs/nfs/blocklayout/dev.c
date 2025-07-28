// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2014-2016 Christoph Hellwig.
 */
#include <linux/sunrpc/svc.h>
#include <linux/blkdev.h>
#include <linux/nfs4.h>
#include <linux/nfs_fs.h>
#include <linux/nfs_xdr.h>
#include <linux/pr.h>

#include "blocklayout.h"
#include "../nfs4trace.h"

#define NFSDBG_FACILITY		NFSDBG_PNFS_LD

static void bl_unregister_scsi(struct pnfs_block_dev *dev)
{
	struct block_device *bdev = file_bdev(dev->bdev_file);
	const struct pr_ops *ops = bdev->bd_disk->fops->pr_ops;
	int status;

	if (!test_and_clear_bit(PNFS_BDEV_REGISTERED, &dev->flags))
		return;

	status = ops->pr_register(bdev, dev->pr_key, 0, false);
	if (status)
		trace_bl_pr_key_unreg_err(bdev, dev->pr_key, status);
	else
		trace_bl_pr_key_unreg(bdev, dev->pr_key);
}

static bool bl_register_scsi(struct pnfs_block_dev *dev)
{
	struct block_device *bdev = file_bdev(dev->bdev_file);
	const struct pr_ops *ops = bdev->bd_disk->fops->pr_ops;
	int status;

	if (test_and_set_bit(PNFS_BDEV_REGISTERED, &dev->flags))
		return true;

	status = ops->pr_register(bdev, 0, dev->pr_key, true);
	if (status) {
		trace_bl_pr_key_reg_err(bdev, dev->pr_key, status);
		return false;
	}
	trace_bl_pr_key_reg(bdev, dev->pr_key);
	return true;
}

static void bl_unregister_dev(struct pnfs_block_dev *dev)
{
	u32 i;

	if (dev->nr_children) {
		for (i = 0; i < dev->nr_children; i++)
			bl_unregister_dev(&dev->children[i]);
		return;
	}

	if (dev->type == PNFS_BLOCK_VOLUME_SCSI)
		bl_unregister_scsi(dev);
}

bool bl_register_dev(struct pnfs_block_dev *dev)
{
	u32 i;

	if (dev->nr_children) {
		for (i = 0; i < dev->nr_children; i++) {
			if (!bl_register_dev(&dev->children[i])) {
				while (i > 0)
					bl_unregister_dev(&dev->children[--i]);
				return false;
			}
		}
		return true;
	}

	if (dev->type == PNFS_BLOCK_VOLUME_SCSI)
		return bl_register_scsi(dev);
	return true;
}

static void
bl_free_device(struct pnfs_block_dev *dev)
{
	bl_unregister_dev(dev);

	if (dev->nr_children) {
		int i;

		for (i = 0; i < dev->nr_children; i++)
			bl_free_device(&dev->children[i]);
		kfree(dev->children);
	} else {
		if (dev->bdev_file)
			fput(dev->bdev_file);
	}
}

void
bl_free_deviceid_node(struct nfs4_deviceid_node *d)
{
	struct pnfs_block_dev *dev =
		container_of(d, struct pnfs_block_dev, node);

	bl_free_device(dev);
	kfree_rcu(dev, node.rcu);
}

static int
nfs4_block_decode_volume(struct xdr_stream *xdr, struct pnfs_block_volume *b)
{
	__be32 *p;
	int i;

	p = xdr_inline_decode(xdr, 4);
	if (!p)
		return -EIO;
	b->type = be32_to_cpup(p++);

	switch (b->type) {
	case PNFS_BLOCK_VOLUME_SIMPLE:
		p = xdr_inline_decode(xdr, 4);
		if (!p)
			return -EIO;
		b->simple.nr_sigs = be32_to_cpup(p++);
		if (!b->simple.nr_sigs || b->simple.nr_sigs > PNFS_BLOCK_MAX_UUIDS) {
			dprintk("Bad signature count: %d\n", b->simple.nr_sigs);
			return -EIO;
		}

		b->simple.len = 4 + 4;
		for (i = 0; i < b->simple.nr_sigs; i++) {
			p = xdr_inline_decode(xdr, 8 + 4);
			if (!p)
				return -EIO;
			p = xdr_decode_hyper(p, &b->simple.sigs[i].offset);
			b->simple.sigs[i].sig_len = be32_to_cpup(p++);
			if (b->simple.sigs[i].sig_len > PNFS_BLOCK_UUID_LEN) {
				pr_info("signature too long: %d\n",
					b->simple.sigs[i].sig_len);
				return -EIO;
			}

			p = xdr_inline_decode(xdr, b->simple.sigs[i].sig_len);
			if (!p)
				return -EIO;
			memcpy(&b->simple.sigs[i].sig, p,
				b->simple.sigs[i].sig_len);

			b->simple.len += 8 + 4 + \
				(XDR_QUADLEN(b->simple.sigs[i].sig_len) << 2);
		}
		break;
	case PNFS_BLOCK_VOLUME_SLICE:
		p = xdr_inline_decode(xdr, 8 + 8 + 4);
		if (!p)
			return -EIO;
		p = xdr_decode_hyper(p, &b->slice.start);
		p = xdr_decode_hyper(p, &b->slice.len);
		b->slice.volume = be32_to_cpup(p++);
		break;
	case PNFS_BLOCK_VOLUME_CONCAT:
		p = xdr_inline_decode(xdr, 4);
		if (!p)
			return -EIO;

		b->concat.volumes_count = be32_to_cpup(p++);
		if (b->concat.volumes_count > PNFS_BLOCK_MAX_DEVICES) {
			dprintk("Too many volumes: %d\n", b->concat.volumes_count);
			return -EIO;
		}

		p = xdr_inline_decode(xdr, b->concat.volumes_count * 4);
		if (!p)
			return -EIO;
		for (i = 0; i < b->concat.volumes_count; i++)
			b->concat.volumes[i] = be32_to_cpup(p++);
		break;
	case PNFS_BLOCK_VOLUME_STRIPE:
		p = xdr_inline_decode(xdr, 8 + 4);
		if (!p)
			return -EIO;

		p = xdr_decode_hyper(p, &b->stripe.chunk_size);
		b->stripe.volumes_count = be32_to_cpup(p++);
		if (b->stripe.volumes_count > PNFS_BLOCK_MAX_DEVICES) {
			dprintk("Too many volumes: %d\n", b->stripe.volumes_count);
			return -EIO;
		}

		p = xdr_inline_decode(xdr, b->stripe.volumes_count * 4);
		if (!p)
			return -EIO;
		for (i = 0; i < b->stripe.volumes_count; i++)
			b->stripe.volumes[i] = be32_to_cpup(p++);
		break;
	case PNFS_BLOCK_VOLUME_SCSI:
		p = xdr_inline_decode(xdr, 4 + 4 + 4);
		if (!p)
			return -EIO;
		b->scsi.code_set = be32_to_cpup(p++);
		b->scsi.designator_type = be32_to_cpup(p++);
		b->scsi.designator_len = be32_to_cpup(p++);
		p = xdr_inline_decode(xdr, b->scsi.designator_len);
		if (!p)
			return -EIO;
		if (b->scsi.designator_len > 256)
			return -EIO;
		memcpy(&b->scsi.designator, p, b->scsi.designator_len);
		p = xdr_inline_decode(xdr, 8);
		if (!p)
			return -EIO;
		p = xdr_decode_hyper(p, &b->scsi.pr_key);
		break;
	default:
		dprintk("unknown volume type!\n");
		return -EIO;
	}

	return 0;
}

static bool bl_map_simple(struct pnfs_block_dev *dev, u64 offset,
		struct pnfs_block_dev_map *map)
{
	map->start = dev->start;
	map->len = dev->len;
	map->disk_offset = dev->disk_offset;
	map->bdev = file_bdev(dev->bdev_file);
	return true;
}

static bool bl_map_concat(struct pnfs_block_dev *dev, u64 offset,
		struct pnfs_block_dev_map *map)
{
	int i;

	for (i = 0; i < dev->nr_children; i++) {
		struct pnfs_block_dev *child = &dev->children[i];

		if (child->start > offset ||
		    child->start + child->len <= offset)
			continue;

		child->map(child, offset - child->start, map);
		return true;
	}

	dprintk("%s: ran off loop!\n", __func__);
	return false;
}

static bool bl_map_stripe(struct pnfs_block_dev *dev, u64 offset,
		struct pnfs_block_dev_map *map)
{
	struct pnfs_block_dev *child;
	u64 chunk;
	u32 chunk_idx;
	u64 disk_offset;

	chunk = div_u64(offset, dev->chunk_size);
	div_u64_rem(chunk, dev->nr_children, &chunk_idx);

	if (chunk_idx >= dev->nr_children) {
		dprintk("%s: invalid chunk idx %d (%lld/%lld)\n",
			__func__, chunk_idx, offset, dev->chunk_size);
		/* error, should not happen */
		return false;
	}

	/* truncate offset to the beginning of the stripe */
	offset = chunk * dev->chunk_size;

	/* disk offset of the stripe */
	disk_offset = div_u64(offset, dev->nr_children);

	child = &dev->children[chunk_idx];
	child->map(child, disk_offset, map);

	map->start += offset;
	map->disk_offset += disk_offset;
	map->len = dev->chunk_size;
	return true;
}

static int
bl_parse_deviceid(struct nfs_server *server, struct pnfs_block_dev *d,
		struct pnfs_block_volume *volumes, int idx, gfp_t gfp_mask);


static int
bl_parse_simple(struct nfs_server *server, struct pnfs_block_dev *d,
		struct pnfs_block_volume *volumes, int idx, gfp_t gfp_mask)
{
	struct pnfs_block_volume *v = &volumes[idx];
	struct file *bdev_file;
	dev_t dev;

	dev = bl_resolve_deviceid(server, v, gfp_mask);
	if (!dev)
		return -EIO;

	bdev_file = bdev_file_open_by_dev(dev, BLK_OPEN_READ | BLK_OPEN_WRITE,
				       NULL, NULL);
	if (IS_ERR(bdev_file)) {
		printk(KERN_WARNING "pNFS: failed to open device %d:%d (%ld)\n",
			MAJOR(dev), MINOR(dev), PTR_ERR(bdev_file));
		return PTR_ERR(bdev_file);
	}
	d->bdev_file = bdev_file;
	d->len = bdev_nr_bytes(file_bdev(bdev_file));
	d->map = bl_map_simple;

	printk(KERN_INFO "pNFS: using block device %s\n",
		file_bdev(bdev_file)->bd_disk->disk_name);
	return 0;
}

static bool
bl_validate_designator(struct pnfs_block_volume *v)
{
	switch (v->scsi.designator_type) {
	case PS_DESIGNATOR_EUI64:
		if (v->scsi.code_set != PS_CODE_SET_BINARY)
			return false;

		if (v->scsi.designator_len != 8 &&
		    v->scsi.designator_len != 10 &&
		    v->scsi.designator_len != 16)
			return false;

		return true;
	case PS_DESIGNATOR_NAA:
		if (v->scsi.code_set != PS_CODE_SET_BINARY)
			return false;

		if (v->scsi.designator_len != 8 &&
		    v->scsi.designator_len != 16)
			return false;

		return true;
	case PS_DESIGNATOR_T10:
	case PS_DESIGNATOR_NAME:
		pr_err("pNFS: unsupported designator "
			"(code set %d, type %d, len %d.\n",
			v->scsi.code_set,
			v->scsi.designator_type,
			v->scsi.designator_len);
		return false;
	default:
		pr_err("pNFS: invalid designator "
			"(code set %d, type %d, len %d.\n",
			v->scsi.code_set,
			v->scsi.designator_type,
			v->scsi.designator_len);
		return false;
	}
}

static struct file *
bl_open_path(struct pnfs_block_volume *v, const char *prefix)
{
	struct file *bdev_file;
	const char *devname;

	devname = kasprintf(GFP_KERNEL, "/dev/disk/by-id/%s%*phN",
			prefix, v->scsi.designator_len, v->scsi.designator);
	if (!devname)
		return ERR_PTR(-ENOMEM);

	bdev_file = bdev_file_open_by_path(devname, BLK_OPEN_READ | BLK_OPEN_WRITE,
					NULL, NULL);
	if (IS_ERR(bdev_file)) {
		dprintk("failed to open device %s (%ld)\n",
			devname, PTR_ERR(bdev_file));
	}

	kfree(devname);
	return bdev_file;
}

static int
bl_parse_scsi(struct nfs_server *server, struct pnfs_block_dev *d,
		struct pnfs_block_volume *volumes, int idx, gfp_t gfp_mask)
{
	struct pnfs_block_volume *v = &volumes[idx];
	struct block_device *bdev;
	const struct pr_ops *ops;
	struct file *bdev_file;
	int error;

	if (!bl_validate_designator(v))
		return -EINVAL;

	/*
	 * Try to open the RH/Fedora specific dm-mpath udev path first, as the
	 * wwn- links will only point to the first discovered SCSI device there.
	 * On other distributions like Debian, the default SCSI by-id path will
	 * point to the dm-multipath device if one exists.
	 */
	bdev_file = bl_open_path(v, "dm-uuid-mpath-0x");
	if (IS_ERR(bdev_file))
		bdev_file = bl_open_path(v, "wwn-0x");
	if (IS_ERR(bdev_file))
		bdev_file = bl_open_path(v, "nvme-eui.");
	if (IS_ERR(bdev_file)) {
		pr_warn("pNFS: no device found for volume %*phN\n",
			v->scsi.designator_len, v->scsi.designator);
		return PTR_ERR(bdev_file);
	}
	d->bdev_file = bdev_file;
	bdev = file_bdev(bdev_file);

	d->len = bdev_nr_bytes(bdev);
	d->map = bl_map_simple;
	d->pr_key = v->scsi.pr_key;

	if (d->len == 0)
		return -ENODEV;

	ops = bdev->bd_disk->fops->pr_ops;
	if (!ops) {
		pr_err("pNFS: block device %s does not support reservations.",
				bdev->bd_disk->disk_name);
		error = -EINVAL;
		goto out_blkdev_put;
	}

	return 0;

out_blkdev_put:
	fput(d->bdev_file);
	return error;
}

static int
bl_parse_slice(struct nfs_server *server, struct pnfs_block_dev *d,
		struct pnfs_block_volume *volumes, int idx, gfp_t gfp_mask)
{
	struct pnfs_block_volume *v = &volumes[idx];
	int ret;

	ret = bl_parse_deviceid(server, d, volumes, v->slice.volume, gfp_mask);
	if (ret)
		return ret;

	d->disk_offset = v->slice.start;
	d->len = v->slice.len;
	return 0;
}

static int
bl_parse_concat(struct nfs_server *server, struct pnfs_block_dev *d,
		struct pnfs_block_volume *volumes, int idx, gfp_t gfp_mask)
{
	struct pnfs_block_volume *v = &volumes[idx];
	u64 len = 0;
	int ret, i;

	d->children = kcalloc(v->concat.volumes_count,
			sizeof(struct pnfs_block_dev), gfp_mask);
	if (!d->children)
		return -ENOMEM;

	for (i = 0; i < v->concat.volumes_count; i++) {
		ret = bl_parse_deviceid(server, &d->children[i],
				volumes, v->concat.volumes[i], gfp_mask);
		if (ret)
			return ret;

		d->nr_children++;
		d->children[i].start += len;
		len += d->children[i].len;
	}

	d->len = len;
	d->map = bl_map_concat;
	return 0;
}

static int
bl_parse_stripe(struct nfs_server *server, struct pnfs_block_dev *d,
		struct pnfs_block_volume *volumes, int idx, gfp_t gfp_mask)
{
	struct pnfs_block_volume *v = &volumes[idx];
	u64 len = 0;
	int ret, i;

	d->children = kcalloc(v->stripe.volumes_count,
			sizeof(struct pnfs_block_dev), gfp_mask);
	if (!d->children)
		return -ENOMEM;

	for (i = 0; i < v->stripe.volumes_count; i++) {
		ret = bl_parse_deviceid(server, &d->children[i],
				volumes, v->stripe.volumes[i], gfp_mask);
		if (ret)
			return ret;

		d->nr_children++;
		len += d->children[i].len;
	}

	d->len = len;
	d->chunk_size = v->stripe.chunk_size;
	d->map = bl_map_stripe;
	return 0;
}

static int
bl_parse_deviceid(struct nfs_server *server, struct pnfs_block_dev *d,
		struct pnfs_block_volume *volumes, int idx, gfp_t gfp_mask)
{
	d->type = volumes[idx].type;

	switch (d->type) {
	case PNFS_BLOCK_VOLUME_SIMPLE:
		return bl_parse_simple(server, d, volumes, idx, gfp_mask);
	case PNFS_BLOCK_VOLUME_SLICE:
		return bl_parse_slice(server, d, volumes, idx, gfp_mask);
	case PNFS_BLOCK_VOLUME_CONCAT:
		return bl_parse_concat(server, d, volumes, idx, gfp_mask);
	case PNFS_BLOCK_VOLUME_STRIPE:
		return bl_parse_stripe(server, d, volumes, idx, gfp_mask);
	case PNFS_BLOCK_VOLUME_SCSI:
		return bl_parse_scsi(server, d, volumes, idx, gfp_mask);
	default:
		dprintk("unsupported volume type: %d\n", d->type);
		return -EIO;
	}
}

struct nfs4_deviceid_node *
bl_alloc_deviceid_node(struct nfs_server *server, struct pnfs_device *pdev,
		gfp_t gfp_mask)
{
	struct nfs4_deviceid_node *node = NULL;
	struct pnfs_block_volume *volumes;
	struct pnfs_block_dev *top;
	struct xdr_stream xdr;
	struct xdr_buf buf;
	struct page *scratch;
	int nr_volumes, ret, i;
	__be32 *p;

	scratch = alloc_page(gfp_mask);
	if (!scratch)
		goto out;

	xdr_init_decode_pages(&xdr, &buf, pdev->pages, pdev->pglen);
	xdr_set_scratch_page(&xdr, scratch);

	p = xdr_inline_decode(&xdr, sizeof(__be32));
	if (!p)
		goto out_free_scratch;
	nr_volumes = be32_to_cpup(p++);

	volumes = kcalloc(nr_volumes, sizeof(struct pnfs_block_volume),
			  gfp_mask);
	if (!volumes)
		goto out_free_scratch;

	for (i = 0; i < nr_volumes; i++) {
		ret = nfs4_block_decode_volume(&xdr, &volumes[i]);
		if (ret < 0)
			goto out_free_volumes;
	}

	top = kzalloc(sizeof(*top), gfp_mask);
	if (!top)
		goto out_free_volumes;

	ret = bl_parse_deviceid(server, top, volumes, nr_volumes - 1, gfp_mask);

	node = &top->node;
	nfs4_init_deviceid_node(node, server, &pdev->dev_id);
	if (ret)
		nfs4_mark_deviceid_unavailable(node);

out_free_volumes:
	kfree(volumes);
out_free_scratch:
	__free_page(scratch);
out:
	return node;
}
