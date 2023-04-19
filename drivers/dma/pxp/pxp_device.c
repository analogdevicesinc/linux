/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2010-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/pxp_device.h>
#include <linux/atomic.h>
#include <linux/dma-buf.h>
#include <linux/dma/imx-dma.h>

#define BUFFER_HASH_ORDER 4

static struct pxp_buffer_hash bufhash;
static struct pxp_irq_info irq_info[NR_PXP_VIRT_CHANNEL];
static int major;
static struct class *pxp_class;
static struct device *pxp_dev;

static int pxp_ht_create(struct pxp_buffer_hash *hash, int order)
{
	unsigned long i;
	unsigned long table_size;

	table_size = 1U << order;

	hash->order = order;
	hash->hash_table = kmalloc(sizeof(*hash->hash_table) * table_size, GFP_KERNEL);

	if (!hash->hash_table) {
		pr_err("%s: Out of memory for hash table\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < table_size; i++)
		INIT_HLIST_HEAD(&hash->hash_table[i]);

	return 0;
}

static int pxp_ht_insert_item(struct pxp_buffer_hash *hash,
			      struct pxp_buf_obj *new)
{
	unsigned long hashkey;
	struct hlist_head *h_list;

	hashkey = hash_long(new->offset >> PAGE_SHIFT, hash->order);
	h_list = &hash->hash_table[hashkey];

	spin_lock(&hash->hash_lock);
	hlist_add_head_rcu(&new->item, h_list);
	spin_unlock(&hash->hash_lock);

	return 0;
}

static int pxp_ht_remove_item(struct pxp_buffer_hash *hash,
			      struct pxp_buf_obj *obj)
{
	spin_lock(&hash->hash_lock);
	hlist_del_init_rcu(&obj->item);
	spin_unlock(&hash->hash_lock);
	return 0;
}

static struct hlist_node *pxp_ht_find_key(struct pxp_buffer_hash *hash,
					  unsigned long key)
{
	struct pxp_buf_obj *entry;
	struct hlist_head *h_list;
	unsigned long hashkey;

	hashkey = hash_long(key, hash->order);
	h_list = &hash->hash_table[hashkey];

	hlist_for_each_entry_rcu(entry, h_list, item) {
		if (entry->offset >> PAGE_SHIFT == key)
			return &entry->item;
	}

	return NULL;
}

static void pxp_ht_destroy(struct pxp_buffer_hash *hash)
{
	kfree(hash->hash_table);
	hash->hash_table = NULL;
}

static int pxp_buffer_handle_create(struct pxp_file *file_priv,
				    struct pxp_buf_obj *obj,
				    uint32_t *handlep)
{
	int ret;

	idr_preload(GFP_KERNEL);
	spin_lock(&file_priv->buffer_lock);

	ret = idr_alloc(&file_priv->buffer_idr, obj, 1, 0, GFP_NOWAIT);

	spin_unlock(&file_priv->buffer_lock);
	idr_preload_end();

	if (ret < 0)
		return ret;

	*handlep = ret;

	return 0;
}

static struct pxp_buf_obj *
pxp_buffer_object_lookup(struct pxp_file *file_priv,
			 uint32_t handle)
{
	struct pxp_buf_obj *obj;

	spin_lock(&file_priv->buffer_lock);

	obj = idr_find(&file_priv->buffer_idr, handle);
	if (!obj) {
		spin_unlock(&file_priv->buffer_lock);
		return NULL;
	}

	spin_unlock(&file_priv->buffer_lock);

	return obj;
}

static int pxp_buffer_handle_delete(struct pxp_file *file_priv,
				    uint32_t handle)
{
	struct pxp_buf_obj *obj;

	spin_lock(&file_priv->buffer_lock);

	obj = idr_find(&file_priv->buffer_idr, handle);
	if (!obj) {
		spin_unlock(&file_priv->buffer_lock);
		return -EINVAL;
	}

	idr_remove(&file_priv->buffer_idr, handle);
	spin_unlock(&file_priv->buffer_lock);

	return 0;
}

static int pxp_channel_handle_create(struct pxp_file *file_priv,
				     struct pxp_chan_obj *obj,
				     uint32_t *handlep)
{
	int ret;

	idr_preload(GFP_KERNEL);
	spin_lock(&file_priv->channel_lock);

	ret = idr_alloc(&file_priv->channel_idr, obj, 0, 0, GFP_NOWAIT);

	spin_unlock(&file_priv->channel_lock);
	idr_preload_end();

	if (ret < 0)
		return ret;

	*handlep = ret;

	return 0;
}

static struct pxp_chan_obj *
pxp_channel_object_lookup(struct pxp_file *file_priv,
			  uint32_t handle)
{
	struct pxp_chan_obj *obj;

	spin_lock(&file_priv->channel_lock);

	obj = idr_find(&file_priv->channel_idr, handle);
	if (!obj) {
		spin_unlock(&file_priv->channel_lock);
		return NULL;
	}

	spin_unlock(&file_priv->channel_lock);

	return obj;
}

static int pxp_channel_handle_delete(struct pxp_file *file_priv,
				     uint32_t handle)
{
	struct pxp_chan_obj *obj;

	spin_lock(&file_priv->channel_lock);

	obj = idr_find(&file_priv->channel_idr, handle);
	if (!obj) {
		spin_unlock(&file_priv->channel_lock);
		return -EINVAL;
	}

	idr_remove(&file_priv->channel_idr, handle);
	spin_unlock(&file_priv->channel_lock);

	return 0;
}

static int pxp_alloc_dma_buffer(struct pxp_buf_obj *obj)
{
	obj->virtual = dma_alloc_coherent(pxp_dev, PAGE_ALIGN(obj->size),
			       (dma_addr_t *) (&obj->offset),
			       GFP_DMA | GFP_KERNEL);
	pr_debug("[ALLOC] mem alloc phys_addr = 0x%lx\n", obj->offset);

	if (obj->virtual == NULL) {
		printk(KERN_ERR "Physical memory allocation error!\n");
		return -1;
	}

	return 0;
}

static void pxp_free_dma_buffer(struct pxp_buf_obj *obj)
{
	if (obj->virtual != NULL) {
		dma_free_coherent(pxp_dev, PAGE_ALIGN(obj->size),
				  obj->virtual, (dma_addr_t)obj->offset);
	}
}

static int
pxp_buffer_object_free(int id, void *ptr, void *data)
{
	struct pxp_file *file_priv = data;
	struct pxp_buf_obj *obj = ptr;
	int ret;

	ret = pxp_buffer_handle_delete(file_priv, obj->handle);
	if (ret < 0)
		return ret;

	pxp_ht_remove_item(&bufhash, obj);
	pxp_free_dma_buffer(obj);
	kfree(obj);

	return 0;
}

static int
pxp_channel_object_free(int id, void *ptr, void *data)
{
	struct pxp_file *file_priv = data;
	struct pxp_chan_obj *obj = ptr;
	int chan_id;

	chan_id = obj->chan->chan_id;
	wait_event(irq_info[chan_id].waitq,
		atomic_read(&irq_info[chan_id].irq_pending) == 0);

	pxp_channel_handle_delete(file_priv, obj->handle);
	dma_release_channel(obj->chan);
	kfree(obj);

	return 0;
}

static void pxp_free_buffers(struct pxp_file *file_priv)
{
	idr_for_each(&file_priv->buffer_idr,
			&pxp_buffer_object_free, file_priv);
	idr_destroy(&file_priv->buffer_idr);
}

static void pxp_free_channels(struct pxp_file *file_priv)
{
	idr_for_each(&file_priv->channel_idr,
			&pxp_channel_object_free, file_priv);
	idr_destroy(&file_priv->channel_idr);
}

/* Callback function triggered after PxP receives an EOF interrupt */
static void pxp_dma_done(void *arg)
{
	struct pxp_tx_desc *tx_desc = to_tx_desc(arg);
	struct dma_chan *chan = tx_desc->txd.chan;
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	int chan_id = pxp_chan->dma_chan.chan_id;

	pr_debug("DMA Done ISR, chan_id %d\n", chan_id);

	atomic_dec(&irq_info[chan_id].irq_pending);
	irq_info[chan_id].hist_status = tx_desc->hist_status;

	wake_up(&(irq_info[chan_id].waitq));
}

static int pxp_ioc_config_chan(struct pxp_file *priv, unsigned long arg)
{
	struct scatterlist *sg;
	struct pxp_tx_desc *desc;
	struct dma_async_tx_descriptor *txd;
	struct pxp_config_data *pxp_conf;
	dma_cookie_t cookie;
	int handle, chan_id;
	struct dma_chan *chan;
	struct pxp_chan_obj *obj;
	int i = 0, j = 0, k = 0, m = 0, length, ret, sg_len;

	pxp_conf = kzalloc(sizeof(*pxp_conf), GFP_KERNEL);
	if (!pxp_conf)
		return -ENOMEM;

	ret = copy_from_user(pxp_conf,
			     (struct pxp_config_data *)arg,
			     sizeof(struct pxp_config_data));
	if (ret) {
		kfree(pxp_conf);
		return -EFAULT;
	}

	handle = pxp_conf->handle;
	obj = pxp_channel_object_lookup(priv, handle);
	if (!obj) {
		kfree(pxp_conf);
		return -EINVAL;
	}
	chan = obj->chan;
	chan_id = chan->chan_id;

	sg_len = 3;
	if (pxp_conf->proc_data.engine_enable & PXP_ENABLE_WFE_A)
		sg_len += 4;
	if (pxp_conf->proc_data.engine_enable & PXP_ENABLE_WFE_B)
		sg_len += 4;
	if (pxp_conf->proc_data.engine_enable & PXP_ENABLE_DITHER)
		sg_len += 4;

	sg = kmalloc(sizeof(*sg) * sg_len, GFP_KERNEL);
	if (!sg) {
		kfree(pxp_conf);
		return -ENOMEM;
	}

	sg_init_table(sg, sg_len);

	txd = chan->device->device_prep_slave_sg(chan,
						 sg, sg_len,
						 DMA_MEM_TO_DEV,
						 DMA_PREP_INTERRUPT,
						 NULL);
	if (!txd) {
		pr_err("Error preparing a DMA transaction descriptor.\n");
		kfree(pxp_conf);
		kfree(sg);
		return -EIO;
	}

	txd->callback_param = txd;
	txd->callback = pxp_dma_done;

	desc = to_tx_desc(txd);

	length = desc->len;
	for (i = 0; i < length; i++) {
		if (i == 0) {	/* S0 */
			memcpy(&desc->proc_data,
			       &pxp_conf->proc_data,
			       sizeof(struct pxp_proc_data));
			memcpy(&desc->layer_param.s0_param,
			       &pxp_conf->s0_param,
			       sizeof(struct pxp_layer_param));
			desc = desc->next;
		} else if (i == 1) {	/* Output */
			memcpy(&desc->layer_param.out_param,
			       &pxp_conf->out_param,
			       sizeof(struct pxp_layer_param));
			desc = desc->next;
		} else if (i == 2) {
			/* OverLay */
			memcpy(&desc->layer_param.ol_param,
			       &pxp_conf->ol_param,
			       sizeof(struct pxp_layer_param));
			desc = desc->next;
		} else if ((pxp_conf->proc_data.engine_enable & PXP_ENABLE_WFE_A) && (j < 4)) {
			for (j = 0; j < 4; j++) {
				if (j == 0) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->wfe_a_fetch_param[0],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_WFE_A_FETCH0;
				} else if (j == 1) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->wfe_a_fetch_param[1],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_WFE_A_FETCH1;
				} else if (j == 2) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->wfe_a_store_param[0],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_WFE_A_STORE0;
				} else if (j == 3) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->wfe_a_store_param[1],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_WFE_A_STORE1;
				}

				desc = desc->next;
			}

			i += 4;

		} else if ((pxp_conf->proc_data.engine_enable & PXP_ENABLE_WFE_B) && (m < 4)) {
			for (m = 0; m < 4; m++) {
				if (m == 0) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->wfe_b_fetch_param[0],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_WFE_B_FETCH0;
				} else if (m == 1) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->wfe_b_fetch_param[1],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_WFE_B_FETCH1;
				} else if (m == 2) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->wfe_b_store_param[0],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_WFE_B_STORE0;
				} else if (m == 3) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->wfe_b_store_param[1],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_WFE_B_STORE1;
				}

				desc = desc->next;
			}

			i += 4;

		} else if ((pxp_conf->proc_data.engine_enable & PXP_ENABLE_DITHER) && (k < 4)) {
			for (k = 0; k < 4; k++) {
				if (k == 0) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->dither_fetch_param[0],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_DITHER_FETCH0;
				} else if (k == 1) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->dither_fetch_param[1],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_DITHER_FETCH1;
				} else if (k == 2) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->dither_store_param[0],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_DITHER_STORE0;
				} else if (k == 3) {
					memcpy(&desc->layer_param.processing_param,
					       &pxp_conf->dither_store_param[1],
					       sizeof(struct pxp_layer_param));
					desc->layer_param.processing_param.flag = PXP_BUF_FLAG_DITHER_STORE1;
				}

				desc = desc->next;
			}

			i += 4;
		}
	}

	cookie = txd->tx_submit(txd);
	if (cookie < 0) {
		pr_err("Error tx_submit\n");
		kfree(pxp_conf);
		kfree(sg);
		return -EIO;
	}

	atomic_inc(&irq_info[chan_id].irq_pending);

	kfree(pxp_conf);
	kfree(sg);

	return 0;
}

static int pxp_device_open(struct inode *inode, struct file *filp)
{
	struct pxp_file *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);

	if (!priv)
		return -ENOMEM;

	filp->private_data = priv;
	priv->filp = filp;

	idr_init(&priv->buffer_idr);
	spin_lock_init(&priv->buffer_lock);

	idr_init(&priv->channel_idr);
	spin_lock_init(&priv->channel_lock);

	return 0;
}

static int pxp_device_release(struct inode *inode, struct file *filp)
{
	struct pxp_file *priv = filp->private_data;

	if (priv) {
		pxp_free_channels(priv);
		pxp_free_buffers(priv);
		kfree(priv);
		filp->private_data = NULL;
	}

	return 0;
}

static int pxp_device_mmap(struct file *file, struct vm_area_struct *vma)
{
	int request_size;
	struct hlist_node *node;
	struct pxp_buf_obj *obj;

	request_size = vma->vm_end - vma->vm_start;

	pr_debug("start=0x%x, pgoff=0x%x, size=0x%x\n",
		 (unsigned int)(vma->vm_start), (unsigned int)(vma->vm_pgoff),
		 request_size);

	node = pxp_ht_find_key(&bufhash, vma->vm_pgoff);
	if (!node)
		return -EINVAL;

	obj = list_entry(node, struct pxp_buf_obj, item);
	if (obj->offset + (obj->size >> PAGE_SHIFT) <
		(vma->vm_pgoff + vma_pages(vma)))
		return -ENOMEM;

	switch (obj->mem_type) {
	case MEMORY_TYPE_UNCACHED:
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		break;
	case MEMORY_TYPE_WC:
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
		break;
	case MEMORY_TYPE_CACHED:
		break;
	default:
		pr_err("%s: invalid memory type!\n", __func__);
		return -EINVAL;
	}

	return remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			       request_size, vma->vm_page_prot) ? -EAGAIN : 0;
}

static bool chan_filter(struct dma_chan *chan, void *arg)
{
	if (imx_dma_is_pxp(chan))
		return true;
	else
		return false;
}

/*********************************************/
/*         DMABUF ops for exporters          */
/*********************************************/

static struct sg_table *pxp_get_base_sgt(struct pxp_buf_obj *obj)
{
	int ret;
	struct sg_table *sgt;

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		dev_err(pxp_dev, "failed to alloc sg table\n");
		return NULL;
	}

	ret = dma_get_sgtable_attrs(pxp_dev, sgt, obj->virtual,
				   (dma_addr_t)obj->offset,
				   PAGE_ALIGN(obj->size), obj->attrs);
	if (ret < 0) {
		dev_err(pxp_dev, "failed to get scatterlist from DMA API\n");
		kfree(sgt);
		return NULL;
	}

	return sgt;
}


static int pxp_dmabuf_ops_attach(struct dma_buf *dbuf,
	struct dma_buf_attachment *dbuf_attach)
{
	struct pxp_attachment *attach;
	struct sg_table *sgt;
	struct scatterlist *rd, *wr;
	struct pxp_buf_obj *obj = dbuf->priv;
	int ret, i;

	attach = kzalloc(sizeof(*attach), GFP_KERNEL);
	if (!attach)
		return -ENOMEM;

	sgt = &attach->sgt;
	ret = sg_alloc_table(sgt, obj->sgt_base->orig_nents, GFP_KERNEL);
	if (ret) {
		kfree(attach);
		return -ENOMEM;
	}

	rd = obj->sgt_base->sgl;
	wr = sgt->sgl;
	for (i = 0; i < sgt->orig_nents; ++i) {
		sg_set_page(wr, sg_page(rd), rd->length, rd->offset);
		rd = sg_next(rd);
		wr = sg_next(wr);
	}

	attach->dma_dir = DMA_NONE;
	dbuf_attach->priv = attach;

	return 0;
}

static void pxp_dmabuf_ops_detach(struct dma_buf *dbuf,
	struct dma_buf_attachment *db_attach)
{
	struct pxp_attachment *attach = db_attach->priv;
	struct sg_table *sgt;

	if (!attach)
		return;

	sgt = &attach->sgt;

	if (attach->dma_dir != DMA_NONE)
		dma_unmap_sgtable(db_attach->dev, sgt, attach->dma_dir,
				  DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(attach);
	db_attach->priv = NULL;
}

static struct sg_table *pxp_dmabuf_ops_map(
	struct dma_buf_attachment *db_attach, enum dma_data_direction dma_dir)
{
	struct pxp_attachment *attach = db_attach->priv;
	struct sg_table *sgt;

	sgt = &attach->sgt;
	/* return previously mapped sg table */
	if (attach->dma_dir == dma_dir) {
		return sgt;
	}

	/* release any previous cache */
	if (attach->dma_dir != DMA_NONE) {
		dma_unmap_sgtable(db_attach->dev, sgt, attach->dma_dir,
				  DMA_ATTR_SKIP_CPU_SYNC);
		attach->dma_dir = DMA_NONE;
	}

	/*
	 * mapping to the client with new direction, no cache sync
	 * required see comment in vb2_dc_dmabuf_ops_detach()
	 */
	if (dma_map_sgtable(db_attach->dev, sgt, dma_dir,
			    DMA_ATTR_SKIP_CPU_SYNC)) {
		pr_err("failed to map scatterlist\n");
		return ERR_PTR(-EIO);
	}

	attach->dma_dir = dma_dir;

	return sgt;
}

static void pxp_dmabuf_ops_unmap(struct dma_buf_attachment *db_attach,
				 struct sg_table *sgt,
				 enum dma_data_direction dma_dir)
{
	/* nothing to be done here */
}

static void pxp_dmabuf_ops_release(struct dma_buf *dbuf)
{
	/* nothing to be done here */
}

static int
pxp_dmabuf_ops_begin_cpu_access(struct dma_buf *dbuf,
				enum dma_data_direction direction)
{
	return 0;
}

static int
pxp_dmabuf_ops_end_cpu_access(struct dma_buf *dbuf,
			      enum dma_data_direction direction)
{
	return 0;
}

static int pxp_dmabuf_ops_vmap(struct dma_buf *dbuf, struct iosys_map *map)
{
	struct pxp_buf_obj *obj = dbuf->priv;

	iosys_map_set_vaddr(map, obj->virtual);

	return 0;
}

static int pxp_dmabuf_ops_mmap(struct dma_buf *dbuf,
	struct vm_area_struct *vma)
{
	/* Don't support so far */
	return 0;
}

static const struct dma_buf_ops pxp_dmabuf_ops = {
	.attach = pxp_dmabuf_ops_attach,
	.detach = pxp_dmabuf_ops_detach,
	.map_dma_buf = pxp_dmabuf_ops_map,
	.unmap_dma_buf = pxp_dmabuf_ops_unmap,
	.begin_cpu_access = pxp_dmabuf_ops_begin_cpu_access,
	.end_cpu_access = pxp_dmabuf_ops_end_cpu_access,
	.vmap = pxp_dmabuf_ops_vmap,
	.mmap = pxp_dmabuf_ops_mmap,
	.release = pxp_dmabuf_ops_release,
};

static long pxp_device_ioctl(struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct pxp_file *file_priv = filp->private_data;

	switch (cmd) {
	case PXP_IOC_GET_CHAN:
		{
			int ret;
			struct dma_chan *chan = NULL;
			dma_cap_mask_t mask;
			struct pxp_chan_obj *obj = NULL;

			pr_debug("drv: PXP_IOC_GET_CHAN Line %d\n", __LINE__);

			dma_cap_zero(mask);
			dma_cap_set(DMA_SLAVE, mask);
			dma_cap_set(DMA_PRIVATE, mask);

			chan = dma_request_channel(mask, chan_filter, NULL);
			if (!chan) {
				pr_err("Unsccessfully received channel!\n");
				return -EBUSY;
			}

			pr_debug("Successfully received channel."
				 "chan_id %d\n", chan->chan_id);

			obj = kzalloc(sizeof(*obj), GFP_KERNEL);
			if (!obj) {
				dma_release_channel(chan);
				return -ENOMEM;
			}
			obj->chan = chan;

			ret = pxp_channel_handle_create(file_priv, obj,
							&obj->handle);
			if (ret) {
				dma_release_channel(chan);
				kfree(obj);
				return ret;
			}

			init_waitqueue_head(&(irq_info[chan->chan_id].waitq));
			if (put_user(obj->handle, (u32 __user *) arg)) {
				pxp_channel_handle_delete(file_priv, obj->handle);
				dma_release_channel(chan);
				kfree(obj);
				return -EFAULT;
			}

			break;
		}
	case PXP_IOC_PUT_CHAN:
		{
			int handle;
			struct pxp_chan_obj *obj;

			if (get_user(handle, (u32 __user *) arg))
				return -EFAULT;

			pr_debug("%d release handle %d\n", __LINE__, handle);

			obj = pxp_channel_object_lookup(file_priv, handle);
			if (!obj)
				return -EINVAL;

			pxp_channel_handle_delete(file_priv, obj->handle);
			dma_release_channel(obj->chan);
			kfree(obj);

			break;
		}
	case PXP_IOC_CONFIG_CHAN:
		{
			int ret;

			ret = pxp_ioc_config_chan(file_priv, arg);
			if (ret)
				return ret;

			break;
		}
	case PXP_IOC_START_CHAN:
		{
			int handle;
			struct pxp_chan_obj *obj = NULL;

			if (get_user(handle, (u32 __user *) arg))
				return -EFAULT;

			obj = pxp_channel_object_lookup(file_priv, handle);
			if (!obj)
				return -EINVAL;

			dma_async_issue_pending(obj->chan);

			break;
		}
	case PXP_IOC_GET_PHYMEM:
		{
			struct pxp_mem_desc buffer;
			struct pxp_buf_obj *obj;

			ret = copy_from_user(&buffer,
					     (struct pxp_mem_desc *)arg,
					     sizeof(struct pxp_mem_desc));
			if (ret)
				return -EFAULT;

			pr_debug("[ALLOC] mem alloc size = 0x%x\n",
				 buffer.size);

			obj = kzalloc(sizeof(*obj), GFP_KERNEL);
			if (!obj)
				return -ENOMEM;
			obj->size = buffer.size;
			obj->mem_type = buffer.mtype;

			ret = pxp_alloc_dma_buffer(obj);
			if (ret == -1) {
				printk(KERN_ERR
				       "Physical memory allocation error!\n");
				kfree(obj);
				return ret;
			}

			ret = pxp_buffer_handle_create(file_priv, obj, &obj->handle);
			if (ret) {
				pxp_free_dma_buffer(obj);
				kfree(obj);
				return ret;
			}
			buffer.handle = obj->handle;
			buffer.phys_addr = obj->offset;

			ret = copy_to_user((void __user *)arg, &buffer,
					   sizeof(struct pxp_mem_desc));
			if (ret) {
				pxp_buffer_handle_delete(file_priv, buffer.handle);
				pxp_free_dma_buffer(obj);
				kfree(obj);
				return -EFAULT;
			}

			pxp_ht_insert_item(&bufhash, obj);

			break;
		}
	case PXP_IOC_PUT_PHYMEM:
		{
			struct pxp_mem_desc pxp_mem;
			struct pxp_buf_obj *obj;

			ret = copy_from_user(&pxp_mem,
					     (struct pxp_mem_desc *)arg,
					     sizeof(struct pxp_mem_desc));
			if (ret)
				return -EACCES;

			obj = pxp_buffer_object_lookup(file_priv, pxp_mem.handle);
			if (!obj)
				return -EINVAL;

			if (obj->sgt_base) {
				sg_free_table(obj->sgt_base);
				kfree(obj->sgt_base);
			}

			ret = pxp_buffer_handle_delete(file_priv, obj->handle);
			if (ret)
				return ret;

			pxp_ht_remove_item(&bufhash, obj);
			pxp_free_dma_buffer(obj);
			kfree(obj);

			break;
		}
	case PXP_IOC_FLUSH_PHYMEM:
		{
			int ret;
			struct pxp_mem_flush flush;
			struct pxp_buf_obj *obj;

			ret = copy_from_user(&flush,
					     (struct pxp_mem_flush *)arg,
					     sizeof(struct pxp_mem_flush));
			if (ret)
				return -EACCES;

			if (flush.handle) {
				obj = pxp_buffer_object_lookup(file_priv, flush.handle);
				if (!obj)
					return -EINVAL;

				switch (flush.type) {
				case CACHE_CLEAN:
					dma_sync_single_for_device(pxp_dev, obj->offset,
								   obj->size, DMA_TO_DEVICE);
					break;
				case CACHE_INVALIDATE:
					dma_sync_single_for_device(pxp_dev, obj->offset,
								   obj->size, DMA_FROM_DEVICE);
					break;
				case CACHE_FLUSH:
					dma_sync_single_for_device(pxp_dev, obj->offset,
								   obj->size, DMA_TO_DEVICE);
					dma_sync_single_for_device(pxp_dev, obj->offset,
								   obj->size, DMA_FROM_DEVICE);
					break;
				default:
					pr_err("%s: invalid cache flush type\n", __func__);
					return -EINVAL;
				}
			} else {
				struct dma_buf *dmabuf;
				struct dma_buf_attachment *attachment;
				struct sg_table *sgt;
				int direction;

				if (flush.type == CACHE_CLEAN)
					direction = DMA_TO_DEVICE;
				else if (flush.type == CACHE_INVALIDATE)
					direction = DMA_FROM_DEVICE;
				else if (flush.type == CACHE_FLUSH)
					direction = DMA_BIDIRECTIONAL;
				else
					direction = DMA_NONE;

				ret = 0;
				dmabuf = dma_buf_get(flush.dmabuf_fd);
				if (IS_ERR(dmabuf)) {
					dev_err(pxp_dev, "failed to get dmabuf\n");
					return PTR_ERR(dmabuf);
				}
				attachment = dma_buf_attach(dmabuf, pxp_dev);
				if (IS_ERR(attachment)) {
					ret = PTR_ERR(attachment);
					dev_err(pxp_dev, "failed to attach dmabuf\n");
					goto err_put;
				}
				sgt = dma_buf_map_attachment(attachment, direction);
				if (IS_ERR(sgt)) {
					ret = PTR_ERR(sgt);
					dev_err(pxp_dev, "failed to get dmabuf sg_table\n");
					goto err_detach;
				}
				dma_buf_unmap_attachment(attachment, sgt, direction);
err_detach:
				dma_buf_detach(dmabuf, attachment);
err_put:
				dma_buf_put(dmabuf);
				if (ret)
					return ret;
			}

			break;
		}
	case PXP_IOC_WAIT4CMPLT:
		{
			struct pxp_chan_handle chan_handle;
			int ret, chan_id, handle;
			struct pxp_chan_obj *obj = NULL;

			ret = copy_from_user(&chan_handle,
					     (struct pxp_chan_handle *)arg,
					     sizeof(struct pxp_chan_handle));
			if (ret)
				return -EFAULT;

			handle = chan_handle.handle;
			obj = pxp_channel_object_lookup(file_priv, handle);
			if (!obj)
				return -EINVAL;
			chan_id = obj->chan->chan_id;

			ret = wait_event_interruptible
			    (irq_info[chan_id].waitq,
			     (atomic_read(&irq_info[chan_id].irq_pending) == 0));
			if (ret < 0)
				return -ERESTARTSYS;

			chan_handle.hist_status = irq_info[chan_id].hist_status;
			ret = copy_to_user((struct pxp_chan_handle *)arg,
					   &chan_handle,
					   sizeof(struct pxp_chan_handle));
			if (ret)
				return -EFAULT;
			break;
		}
	case PXP_IOC_EXPBUF:
		{
			struct pxp_mem_desc buffer;
			struct dma_buf *dbuf;
			struct pxp_buf_obj *obj;
			DEFINE_DMA_BUF_EXPORT_INFO(pxp_exp_info);

			ret = copy_from_user(&buffer, (struct pxp_mem_desc *)arg,
					     sizeof(struct pxp_mem_desc));
			if (ret)
				return -EFAULT;

			obj = pxp_buffer_object_lookup(file_priv, buffer.handle);
			if (!obj)
				return -EINVAL;

			if (!obj->sgt_base)
				obj->sgt_base = pxp_get_base_sgt(obj);

			if (WARN_ON(!obj->sgt_base))
				return -EINVAL;

			pxp_exp_info.ops   = &pxp_dmabuf_ops;
			pxp_exp_info.flags = buffer.flags;
			pxp_exp_info.size  = obj->size;
			pxp_exp_info.priv  = obj;

			dbuf = dma_buf_export(&pxp_exp_info);
			if (IS_ERR(dbuf))
				return PTR_ERR(dbuf);

			/* need to check buffer.flags */
			ret = dma_buf_fd(dbuf, buffer.flags & ~O_ACCMODE);
			if (ret < 0)
				return ret;
			buffer.fd = ret;

			ret = copy_to_user((void __user *)arg, &buffer,
				     sizeof(struct pxp_mem_desc));
			if (ret)
				return -EFAULT;
			break;
		}
	default:
		break;
	}

	return 0;
}

static const struct file_operations pxp_device_fops = {
	.open = pxp_device_open,
	.release = pxp_device_release,
	.unlocked_ioctl = pxp_device_ioctl,
	.mmap = pxp_device_mmap,
};

int register_pxp_device(void)
{
	int ret;

	if (!major) {
		major = register_chrdev(0, "pxp_device", &pxp_device_fops);
		if (major < 0) {
			printk(KERN_ERR "Unable to register pxp device\n");
			ret = major;
			goto register_cdev_fail;
		}

		pxp_class = class_create("pxp_device");
		if (IS_ERR(pxp_class)) {
			ret = PTR_ERR(pxp_class);
			goto pxp_class_fail;
		}

		pxp_dev = device_create(pxp_class, NULL, MKDEV(major, 0),
				NULL, "pxp_device");
		if (IS_ERR(pxp_dev)) {
			ret = PTR_ERR(pxp_dev);
			goto dev_create_fail;
		}
		pxp_dev->dma_mask = kmalloc(sizeof(*pxp_dev->dma_mask),
						GFP_KERNEL);
		*pxp_dev->dma_mask = DMA_BIT_MASK(32);
		pxp_dev->coherent_dma_mask = DMA_BIT_MASK(32);
	}

	ret = pxp_ht_create(&bufhash, BUFFER_HASH_ORDER);
	if (ret) {
		goto ht_create_fail;
	}
	spin_lock_init(&(bufhash.hash_lock));

	pr_debug("PxP_Device registered Successfully\n");
	return 0;

ht_create_fail:
	device_destroy(pxp_class, MKDEV(major, 0));
dev_create_fail:
	class_destroy(pxp_class);
pxp_class_fail:
	unregister_chrdev(major, "pxp_device");
register_cdev_fail:
	return ret;
}
EXPORT_SYMBOL_GPL(register_pxp_device);

void unregister_pxp_device(void)
{
	pxp_ht_destroy(&bufhash);
	if (major) {
		device_destroy(pxp_class, MKDEV(major, 0));
		class_destroy(pxp_class);
		unregister_chrdev(major, "pxp_device");
		major = 0;
	}
}
EXPORT_SYMBOL_GPL(unregister_pxp_device);
MODULE_LICENSE("GPL");
