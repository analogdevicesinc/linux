// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtio over ivshmem front-end device driver
 *
 * Copyright (c) Siemens AG, 2019
 */

#include <linux/delay.h>
#include <linux/dma-map-ops.h>
#include <linux/ivshmem.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ring.h>

#define DRV_NAME "virtio-ivshmem"

#define VIRTIO_IVSHMEM_PREFERRED_ALLOC_CHUNKS	4096

#define VIRTIO_STATE_READY	cpu_to_le32(1)

struct virtio_ivshmem_header {
	__le32 revision;
	__le32 size;

	__le32 write_transaction;

	__le32 device_features;
	__le32 device_features_sel;
	__le32 driver_features;
	__le32 driver_features_sel;

	__le32 queue_sel;

	__le16 queue_size;
	__le16 queue_device_vector;
	__le16 queue_driver_vector;
	__le16 queue_enable;
	__le64 queue_desc;
	__le64 queue_driver;
	__le64 queue_device;

	__u8 config_event;
	__u8 queue_event;
	__u8 __reserved[2];
	__le32 device_status;

	__le32 config_generation;
	__u8 config[];
};

#define VI_REG_OFFSET(reg)	offsetof(struct virtio_ivshmem_header, reg)

struct virtio_ivshmem_device {
	struct virtio_device vdev;
	struct pci_dev *pci_dev;

	struct ivshm_regs __iomem *ivshm_regs;

	unsigned int num_vectors;
	bool per_vq_vector;
	char *config_irq_name;
	char *queues_irq_name;

	u32 peer_id;
	u32 *peer_state;

	void *shmem;
	resource_size_t shmem_sz;
	struct virtio_ivshmem_header *virtio_header;

	spinlock_t alloc_lock;
	unsigned long *alloc_bitmap;
	unsigned int alloc_shift;
	void **map_src_addr;

	/* a list of queues so we can dispatch IRQs */
	spinlock_t virtqueues_lock;
	struct list_head virtqueues;
};

struct virtio_ivshmem_vq_info {
	/* the actual virtqueue */
	struct virtqueue *vq;

	/* vector to use for signaling the device */
	unsigned int device_vector;
	/* vector used by the device for signaling the driver */
	unsigned int driver_vector;

	char *irq_name;

	/* the list node for the virtqueues list */
	struct list_head node;
};

static inline unsigned int get_custom_order(unsigned long size,
					    unsigned int shift)
{
	size--;
	size >>= shift;
#if BITS_PER_LONG == 32
	return fls(size);
#else
	return fls64(size);
#endif
}

static inline struct virtio_ivshmem_device *
to_virtio_ivshmem_device(struct virtio_device *vdev)
{
	return container_of(vdev, struct virtio_ivshmem_device, vdev);
}

static bool vi_synchronize_reg_write(struct virtio_ivshmem_device *vi_dev)
{
	while (READ_ONCE(vi_dev->virtio_header->write_transaction)) {
		if (READ_ONCE(*vi_dev->peer_state) != VIRTIO_STATE_READY) {
			dev_err_ratelimited(&vi_dev->pci_dev->dev,
					    "backend failed!");
			return false;
		}
		cpu_relax();
	}
	return true;
}

static bool vi_reg_write(struct virtio_ivshmem_device *vi_dev, unsigned int reg,
			 u64 value, unsigned int size)
{
	u8 *reg_area = (u8 *)vi_dev->virtio_header;

	if (!vi_synchronize_reg_write(vi_dev))
		return false;

	if (size == 1)
		*(u8 *)(reg_area + reg) = (u8)value;
	else if (size == 2)
		*(u16 *)(reg_area + reg) = cpu_to_le16((u16)value);
	else if (size == 4)
		*(u32 *)(reg_area + reg) = cpu_to_le32((u32)value);
	else if (size == 8)
		*(u64 *)(reg_area + reg) = cpu_to_le64(value);
	else
		BUG();
	virt_wmb();

	vi_dev->virtio_header->write_transaction = cpu_to_le32(reg);
	virt_wmb();

	writel((vi_dev->peer_id << 16), &vi_dev->ivshm_regs->doorbell);

	return true;
}

static bool vi_reg_write16(struct virtio_ivshmem_device *vi_dev,
			   unsigned int reg, u32 value)
{
	return vi_reg_write(vi_dev, reg, value, 2);
}

static bool vi_reg_write32(struct virtio_ivshmem_device *vi_dev,
			   unsigned int reg, u32 value)
{
	return vi_reg_write(vi_dev, reg, value, 4);
}

static bool vi_reg_write64(struct virtio_ivshmem_device *vi_dev,
			   unsigned int reg, u64 value)
{
	return vi_reg_write(vi_dev, reg, value, 8);
}

static void vi_get(struct virtio_device *vdev, unsigned offset,
		   void *buf, unsigned len)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);
	__le16 w;
	__le32 l;
	__le64 q;

	switch (len) {
	case 1:
		*(u8 *)buf = *(u8 *)(vi_dev->virtio_header->config + offset);
		break;
	case 2:
		w = *(u16 *)(vi_dev->virtio_header->config + offset);
		*(u16 *)buf = le16_to_cpu(w);
		break;
	case 4:
		l = *(u32 *)(vi_dev->virtio_header->config + offset);
		*(u32 *)buf = le32_to_cpu(l);
		break;
	case 8:
		q = *(u64 *)(vi_dev->virtio_header->config + offset);
		*(u64 *)buf = le64_to_cpu(q);
		break;
	default:
		BUG();
	}
}

static void vi_set(struct virtio_device *vdev, unsigned offset,
		   const void *buf, unsigned len)
{
	u64 value;

	switch (len) {
	case 1:
		value = *(u8 *)buf;
		break;
	case 2:
		value = *(u16 *)buf;
		break;
	case 4:
		value = *(u32 *)buf;
		break;
	case 8:
		value = *(u64 *)buf;
		break;
	default:
		BUG();
	}
	vi_reg_write(to_virtio_ivshmem_device(vdev),
		     offsetof(struct virtio_ivshmem_header, config) + offset,
		     value, len);
}

static u32 vi_generation(struct virtio_device *vdev)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);
	u32 gen  = READ_ONCE(vi_dev->virtio_header->config_generation);

	while (gen & 1) {
		if (READ_ONCE(*vi_dev->peer_state) != VIRTIO_STATE_READY) {
			dev_err_ratelimited(&vi_dev->pci_dev->dev,
					    "backend failed!");
			return 0;
		}
		cpu_relax();

		gen = READ_ONCE(vi_dev->virtio_header->config_generation);
	}
	return gen;
}

static u8 vi_get_status(struct virtio_device *vdev)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);

	return le32_to_cpu(vi_dev->virtio_header->device_status) & 0xff;
}

static void vi_set_status(struct virtio_device *vdev, u8 status)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);

	/* We should never be setting status to 0. */
	BUG_ON(status == 0);

	vi_reg_write32(vi_dev, VI_REG_OFFSET(device_status), status);
}

static void vi_reset(struct virtio_device *vdev)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);

	/* 0 status means a reset. */
	vi_reg_write32(vi_dev, VI_REG_OFFSET(device_status), 0);
}

static u64 vi_get_features(struct virtio_device *vdev)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);
	u64 features;

	if (!vi_reg_write32(vi_dev, VI_REG_OFFSET(device_features_sel), 1) ||
	    !vi_synchronize_reg_write(vi_dev))
		return 0;
	features = le32_to_cpu(vi_dev->virtio_header->device_features);
	features <<= 32;

	if (!vi_reg_write32(vi_dev, VI_REG_OFFSET(device_features_sel), 0) ||
	    !vi_synchronize_reg_write(vi_dev))
		return 0;
	features |= le32_to_cpu(vi_dev->virtio_header->device_features);

	return features;
}

static int vi_finalize_features(struct virtio_device *vdev)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);

	/* Give virtio_ring a chance to accept features. */
	vring_transport_features(vdev);

	if (!__virtio_test_bit(vdev, VIRTIO_F_VERSION_1)) {
		dev_err(&vdev->dev,
			"virtio: device does not have VIRTIO_F_VERSION_1\n");
		return -EINVAL;
	}

	if (!vi_reg_write32(vi_dev, VI_REG_OFFSET(driver_features_sel), 1) ||
	    !vi_reg_write32(vi_dev, VI_REG_OFFSET(driver_features),
			    (u32)(vdev->features >> 32)))
		return -ENODEV;

	if (!vi_reg_write32(vi_dev, VI_REG_OFFSET(driver_features_sel), 0) ||
	    !vi_reg_write32(vi_dev, VI_REG_OFFSET(driver_features),
			    (u32)vdev->features))
		return -ENODEV;

	return 0;
}

/* the notify function used when creating a virt queue */
static bool vi_notify(struct virtqueue *vq)
{
	struct virtio_ivshmem_vq_info *info = vq->priv;
	struct virtio_ivshmem_device *vi_dev =
		to_virtio_ivshmem_device(vq->vdev);

	virt_wmb();
	writel((vi_dev->peer_id << 16) | info->device_vector,
	       &vi_dev->ivshm_regs->doorbell);

	return true;
}

static irqreturn_t vi_config_interrupt(int irq, void *opaque)
{
	struct virtio_ivshmem_device *vi_dev = opaque;

	if (unlikely(READ_ONCE(*vi_dev->peer_state) != VIRTIO_STATE_READY)) {
		virtio_break_device(&vi_dev->vdev);
		vi_dev->virtio_header->config_event = 0;
		vi_dev->virtio_header->queue_event = 0;
		dev_err(&vi_dev->pci_dev->dev, "backend failed!");
		return IRQ_HANDLED;
	}

	if (unlikely(READ_ONCE(vi_dev->virtio_header->config_event) & 1)) {
		vi_dev->virtio_header->config_event = 0;
		virt_wmb();
		virtio_config_changed(&vi_dev->vdev);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t vi_queues_interrupt(int irq, void *opaque)
{
	struct virtio_ivshmem_device *vi_dev = opaque;
	struct virtio_ivshmem_vq_info *info;
	irqreturn_t ret = IRQ_NONE;

	if (likely(READ_ONCE(vi_dev->virtio_header->queue_event) & 1)) {
		vi_dev->virtio_header->queue_event = 0;
		virt_wmb();
		spin_lock(&vi_dev->virtqueues_lock);
		list_for_each_entry(info, &vi_dev->virtqueues, node)
			ret |= vring_interrupt(irq, info->vq);
		spin_unlock(&vi_dev->virtqueues_lock);
	}

	return ret;
}

static irqreturn_t vi_interrupt(int irq, void *opaque)
{
	return vi_config_interrupt(irq, opaque) |
		vi_queues_interrupt(irq, opaque);
}

static struct virtqueue *vi_setup_vq(struct virtio_device *vdev,
				     unsigned int index,
				     void (*callback)(struct virtqueue *vq),
				     const char *name, bool ctx,
				     unsigned int irq_vector)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);
	struct virtio_ivshmem_vq_info *info;
	struct virtqueue *vq;
	unsigned long flags;
	unsigned int size;
	int irq, err;

	/* Select the queue we're interested in */
	if (!vi_reg_write32(vi_dev, VI_REG_OFFSET(queue_sel), index) ||
	    !vi_synchronize_reg_write(vi_dev))
		return ERR_PTR(-ENODEV);

	/* Queue shouldn't already be set up. */
	if (vi_dev->virtio_header->queue_enable)
		return ERR_PTR(-ENOENT);

	/* Allocate and fill out our active queue description */
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return ERR_PTR(-ENOMEM);

	size = vi_dev->virtio_header->queue_size;
	if (size == 0) {
		err = -ENOENT;
		goto error_new_virtqueue;
	}

	info->device_vector = vi_dev->virtio_header->queue_device_vector;
	info->driver_vector = irq_vector;

	/* Create the vring */
	vq = vring_create_virtqueue(index, size, SMP_CACHE_BYTES, vdev, true,
				    true, ctx, vi_notify, callback, name);
	if (!vq) {
		err = -ENOMEM;
		goto error_new_virtqueue;
	}

	if (callback && vi_dev->per_vq_vector) {
		irq = pci_irq_vector(vi_dev->pci_dev, info->driver_vector);
		info->irq_name = kasprintf(GFP_KERNEL, "%s-%s",
					   dev_name(&vdev->dev), name);
		if (!info->irq_name) {
			err = -ENOMEM;
			goto error_setup_virtqueue;
		}

		err = request_irq(irq, vring_interrupt, 0, info->irq_name, vq);
		if (err)
			goto error_setup_virtqueue;
	}

	/* Activate the queue */
	if (!vi_reg_write16(vi_dev, VI_REG_OFFSET(queue_size),
			    virtqueue_get_vring_size(vq)) ||
	    !vi_reg_write16(vi_dev, VI_REG_OFFSET(queue_driver_vector),
			    info->driver_vector) ||
	    !vi_reg_write64(vi_dev, VI_REG_OFFSET(queue_desc),
			    virtqueue_get_desc_addr(vq)) ||
	    !vi_reg_write64(vi_dev, VI_REG_OFFSET(queue_driver),
			    virtqueue_get_avail_addr(vq)) ||
	    !vi_reg_write64(vi_dev, VI_REG_OFFSET(queue_device),
			    virtqueue_get_used_addr(vq)) ||
	    !vi_reg_write16(vi_dev, VI_REG_OFFSET(queue_enable), 1)) {
		err = -ENODEV;
		goto error_setup_virtqueue;
	}

	vq->priv = info;
	info->vq = vq;

	spin_lock_irqsave(&vi_dev->virtqueues_lock, flags);
	list_add(&info->node, &vi_dev->virtqueues);
	spin_unlock_irqrestore(&vi_dev->virtqueues_lock, flags);

	return vq;

error_setup_virtqueue:
	vring_del_virtqueue(vq);

error_new_virtqueue:
	vi_reg_write32(vi_dev, VI_REG_OFFSET(queue_enable), 0);
	kfree(info);
	return ERR_PTR(err);
}

static void vi_del_vq(struct virtqueue *vq)
{
	struct virtio_ivshmem_device *vi_dev =
		to_virtio_ivshmem_device(vq->vdev);
	struct virtio_ivshmem_vq_info *info = vq->priv;
	unsigned long flags;

	spin_lock_irqsave(&vi_dev->virtqueues_lock, flags);
	list_del(&info->node);
	spin_unlock_irqrestore(&vi_dev->virtqueues_lock, flags);

	/* Select and deactivate the queue */
	vi_reg_write32(vi_dev, VI_REG_OFFSET(queue_sel), vq->index);
	vi_reg_write32(vi_dev, VI_REG_OFFSET(queue_enable), 0);

	vring_del_virtqueue(vq);

	if (info->driver_vector) {
		free_irq(pci_irq_vector(vi_dev->pci_dev, info->driver_vector),
			 vq);
		kfree(info->irq_name);
	}

	kfree(info);
}

static void vi_del_vqs(struct virtio_device *vdev)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);
	struct virtqueue *vq, *n;

	list_for_each_entry_safe(vq, n, &vdev->vqs, list)
		vi_del_vq(vq);

	free_irq(pci_irq_vector(vi_dev->pci_dev, 0), vi_dev);
	if (!vi_dev->per_vq_vector && vi_dev->num_vectors > 1)
		free_irq(pci_irq_vector(vi_dev->pci_dev, 1), vi_dev);
	pci_free_irq_vectors(vi_dev->pci_dev);

	kfree(vi_dev->config_irq_name);
	vi_dev->config_irq_name = NULL;
	kfree(vi_dev->queues_irq_name);
	vi_dev->queues_irq_name = NULL;
}

static int vi_find_vqs(struct virtio_device *vdev, unsigned nvqs,
		       struct virtqueue *vqs[],
		       struct virtqueue_info vqs_info[],
		       struct irq_affinity *desc)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);
	unsigned int vq_vector, desired_vectors;
	int err, vectors, i, queue_idx = 0;
	struct virtqueue_info *vqi;

	desired_vectors = 1; /* one for config events */
	for (i = 0; i < nvqs; i++) {
		vqi = &vqs_info[i];
		if (vqi->callback)
			desired_vectors++;
	}

	vectors = pci_alloc_irq_vectors(vi_dev->pci_dev, desired_vectors,
					desired_vectors, PCI_IRQ_MSIX);
	if (vectors != desired_vectors) {
		vectors = pci_alloc_irq_vectors(vi_dev->pci_dev, 1, 2,
						PCI_IRQ_INTX | PCI_IRQ_MSIX);
		if (vectors < 0)
			return vectors;
	}

	vi_dev->num_vectors = vectors;
	vi_dev->per_vq_vector = vectors == desired_vectors;

	if (vectors == 1) {
		vq_vector = 0;
		err = request_irq(pci_irq_vector(vi_dev->pci_dev, 0),
				  vi_interrupt, IRQF_SHARED,
				  dev_name(&vdev->dev), vi_dev);
		if (err)
			goto error_common_irq;
	} else {
		vq_vector = 1;
		vi_dev->config_irq_name = kasprintf(GFP_KERNEL, "%s-config",
						    dev_name(&vdev->dev));
		if (!vi_dev->config_irq_name) {
			err = -ENOMEM;
			goto error_common_irq;
		}

		err = request_irq(pci_irq_vector(vi_dev->pci_dev, 0),
				  vi_config_interrupt, 0,
				  vi_dev->config_irq_name, vi_dev);
		if (err)
			goto error_common_irq;
	}

	if (!vi_dev->per_vq_vector && vectors > 1) {
		vi_dev->queues_irq_name = kasprintf(GFP_KERNEL, "%s-virtqueues",
						    dev_name(&vdev->dev));
		if (!vi_dev->queues_irq_name) {
			err = -ENOMEM;
			goto error_queues_irq;
		}

		err = request_irq(pci_irq_vector(vi_dev->pci_dev, 1),
				  vi_queues_interrupt, 0,
				  vi_dev->queues_irq_name, vi_dev);
		if (err)
			goto error_queues_irq;
	}

	for (i = 0; i < nvqs; ++i) {
		vqi = &vqs_info[i];

		if (!vqi->name) {
			vqs[i] = NULL;
			continue;
		}

		vqs[i] = vi_setup_vq(vdev, queue_idx++, vqi->callback,
				    vqi->name, vqi->ctx, vq_vector);
		if (IS_ERR(vqs[i])) {
			vi_del_vqs(vdev);
			return PTR_ERR(vqs[i]);
		}

		if (vi_dev->per_vq_vector)
			vq_vector++;
	}

	writel(IVSHM_INT_ENABLE, &vi_dev->ivshm_regs->int_control);

	return 0;

error_queues_irq:
	free_irq(pci_irq_vector(vi_dev->pci_dev, 0), vi_dev);
	kfree(vi_dev->config_irq_name);
	vi_dev->config_irq_name = NULL;

error_common_irq:
	kfree(vi_dev->queues_irq_name);
	vi_dev->queues_irq_name = NULL;
	pci_free_irq_vectors(vi_dev->pci_dev);
	return err;
}

static const char *vi_bus_name(struct virtio_device *vdev)
{
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);

	return pci_name(vi_dev->pci_dev);
}

static const struct virtio_config_ops virtio_ivshmem_config_ops = {
	.get			= vi_get,
	.set			= vi_set,
	.generation		= vi_generation,
	.get_status		= vi_get_status,
	.set_status		= vi_set_status,
	.reset			= vi_reset,
	.find_vqs		= vi_find_vqs,
	.del_vqs		= vi_del_vqs,
	.get_features		= vi_get_features,
	.finalize_features	= vi_finalize_features,
	.bus_name		= vi_bus_name,
};

static void virtio_ivshmem_release_dev(struct device *_d)
{
	struct virtio_device *vdev = dev_to_virtio(_d);
	struct virtio_ivshmem_device *vi_dev = to_virtio_ivshmem_device(vdev);

	devm_kfree(&vi_dev->pci_dev->dev, vi_dev);
}

static u64 get_config_qword(struct pci_dev *pci_dev, unsigned int pos)
{
	u32 lo, hi;

	pci_read_config_dword(pci_dev, pos, &lo);
	pci_read_config_dword(pci_dev, pos + 4, &hi);
	return lo | ((u64)hi << 32);
}

static void *vi_dma_alloc(struct device *dev, size_t size,
			  dma_addr_t *dma_handle, gfp_t flag,
			  unsigned long attrs)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct virtio_ivshmem_device *vi_dev = pci_get_drvdata(pci_dev);
	int order = get_custom_order(size, vi_dev->alloc_shift);
	int chunk = -ENOMEM;
	unsigned long flags;
	void *addr;

	spin_lock_irqsave(&vi_dev->alloc_lock, flags);
	chunk = bitmap_find_free_region(vi_dev->alloc_bitmap,
					vi_dev->shmem_sz >> vi_dev->alloc_shift,
					order);
	spin_unlock_irqrestore(&vi_dev->alloc_lock, flags);

	if (chunk < 0) {
		if (!(attrs & DMA_ATTR_NO_WARN) && printk_ratelimit())
			dev_warn(dev,
				 "shared memory is full (size: %zd bytes)\n",
				 size);
		return NULL;
	}

	*dma_handle = chunk << vi_dev->alloc_shift;
	addr = vi_dev->shmem + *dma_handle;
	memset(addr, 0, size);

	return addr;
}

static void vi_dma_free(struct device *dev, size_t size, void *vaddr,
			dma_addr_t dma_handle, unsigned long attrs)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct virtio_ivshmem_device *vi_dev = pci_get_drvdata(pci_dev);
	int order = get_custom_order(size, vi_dev->alloc_shift);
	int chunk = (int)(dma_handle >> vi_dev->alloc_shift);
	unsigned long flags;

	spin_lock_irqsave(&vi_dev->alloc_lock, flags);
	bitmap_release_region(vi_dev->alloc_bitmap, chunk, order);
	spin_unlock_irqrestore(&vi_dev->alloc_lock, flags);
}

static dma_addr_t vi_dma_map_page(struct device *dev, struct page *page,
				  unsigned long offset, size_t size,
				  enum dma_data_direction dir,
				  unsigned long attrs)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct virtio_ivshmem_device *vi_dev = pci_get_drvdata(pci_dev);
	void *buffer, *orig_addr;
	dma_addr_t dma_addr;

	buffer = vi_dma_alloc(dev, size, &dma_addr, 0, attrs);
	if (!buffer)
		return DMA_MAPPING_ERROR;

	orig_addr = page_address(page) + offset;
	vi_dev->map_src_addr[dma_addr >> vi_dev->alloc_shift] = orig_addr;

	if (!(attrs & DMA_ATTR_SKIP_CPU_SYNC) &&
	    (dir == DMA_TO_DEVICE || dir == DMA_BIDIRECTIONAL))
		memcpy(buffer, orig_addr, size);

	return dma_addr;
}

static void vi_dma_unmap_page(struct device *dev, dma_addr_t dma_addr,
			      size_t size, enum dma_data_direction dir,
			      unsigned long attrs)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct virtio_ivshmem_device *vi_dev = pci_get_drvdata(pci_dev);
	void *orig_addr = vi_dev->map_src_addr[dma_addr >> vi_dev->alloc_shift];
	void *buffer = vi_dev->shmem + dma_addr;

	if (!(attrs & DMA_ATTR_SKIP_CPU_SYNC) &&
            ((dir == DMA_FROM_DEVICE) || (dir == DMA_BIDIRECTIONAL)))
		memcpy(orig_addr, buffer, size);

	vi_dma_free(dev, size, buffer, dma_addr, attrs);
}

static void
vi_dma_sync_single_for_cpu(struct device *dev, dma_addr_t dma_addr,
			   size_t size, enum dma_data_direction dir)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct virtio_ivshmem_device *vi_dev = pci_get_drvdata(pci_dev);
	void *orig_addr = vi_dev->map_src_addr[dma_addr >> vi_dev->alloc_shift];
	void *buffer = vi_dev->shmem + dma_addr;

	memcpy(orig_addr, buffer, size);
}

static void
vi_dma_sync_single_for_device(struct device *dev, dma_addr_t dma_addr,
			      size_t size, enum dma_data_direction dir)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct virtio_ivshmem_device *vi_dev = pci_get_drvdata(pci_dev);
	void *orig_addr = vi_dev->map_src_addr[dma_addr >> vi_dev->alloc_shift];
	void *buffer = vi_dev->shmem + dma_addr;

	memcpy(buffer, orig_addr, size);
}

static const struct dma_map_ops virtio_ivshmem_dma_ops = {
	.alloc = vi_dma_alloc,
	.free = vi_dma_free,
	.map_page = vi_dma_map_page,
	.unmap_page = vi_dma_unmap_page,
	.sync_single_for_cpu = vi_dma_sync_single_for_cpu,
	.sync_single_for_device = vi_dma_sync_single_for_device,
};

static int virtio_ivshmem_probe(struct pci_dev *pci_dev,
				const struct pci_device_id *pci_id)
{
	unsigned int chunks, chunk_size, bitmap_size;
	struct virtio_ivshmem_device *vi_dev;
	resource_size_t section_sz;
	phys_addr_t section_addr;
	unsigned int cap_pos;
	u32 *state_table;
	int vendor_cap;
	u32 id, dword;
	int ret;

	vi_dev = devm_kzalloc(&pci_dev->dev, sizeof(*vi_dev), GFP_KERNEL);
	if (!vi_dev)
		return -ENOMEM;

	pci_set_drvdata(pci_dev, vi_dev);
	vi_dev->vdev.dev.parent = &pci_dev->dev;
	vi_dev->vdev.dev.release = virtio_ivshmem_release_dev;
	vi_dev->vdev.config = &virtio_ivshmem_config_ops;
	vi_dev->vdev.id.device = pci_dev->class & IVSHM_PROTO_VIRTIO_DEVID_MASK;
	vi_dev->vdev.id.vendor = pci_dev->subsystem_vendor;
	vi_dev->pci_dev = pci_dev;

	spin_lock_init(&vi_dev->virtqueues_lock);
	INIT_LIST_HEAD(&vi_dev->virtqueues);

	ret = pcim_enable_device(pci_dev);
	if (ret)
		return ret;

	ret = pcim_iomap_regions(pci_dev, BIT(0), DRV_NAME);
	if (ret)
		return ret;

	vi_dev->ivshm_regs = pcim_iomap_table(pci_dev)[0];

	id = readl(&vi_dev->ivshm_regs->id);
	if (id > 1) {
		dev_err(&pci_dev->dev, "invalid ID %d\n", id);
		return -EINVAL;
	}
	if (readl(&vi_dev->ivshm_regs->max_peers) != 2) {
		dev_err(&pci_dev->dev, "number of peers must be 2\n");
		return -EINVAL;
	}

	vi_dev->peer_id = !id;

	vendor_cap = pci_find_capability(pci_dev, PCI_CAP_ID_VNDR);
	if (vendor_cap < 0) {
		dev_err(&pci_dev->dev, "missing vendor capability\n");
		return -EINVAL;
	}

	if (pci_resource_len(pci_dev, 2) > 0) {
		section_addr = pci_resource_start(pci_dev, 2);
	} else {
		cap_pos = vendor_cap + IVSHM_CFG_ADDRESS;
		section_addr = get_config_qword(pci_dev, cap_pos);
	}

	cap_pos = vendor_cap + IVSHM_CFG_STATE_TAB_SZ;
	pci_read_config_dword(pci_dev, cap_pos, &dword);
	section_sz = dword;

	if (!devm_request_mem_region(&pci_dev->dev, section_addr, section_sz,
				     DRV_NAME))
		return -EBUSY;

	state_table = devm_memremap(&pci_dev->dev, section_addr, section_sz,
				    MEMREMAP_WB);
	if (!state_table)
		return -ENOMEM;

	vi_dev->peer_state = &state_table[vi_dev->peer_id];
	if (*vi_dev->peer_state != VIRTIO_STATE_READY) {
		dev_err(&pci_dev->dev, "backend not ready\n");
		return -ENODEV;
	}

	section_addr += section_sz;

	cap_pos = vendor_cap + IVSHM_CFG_RW_SECTION_SZ;
	section_sz = get_config_qword(pci_dev, cap_pos);
	if (section_sz < 2 * PAGE_SIZE) {
		dev_err(&pci_dev->dev, "R/W section too small\n");
		return -EINVAL;
	}

	vi_dev->shmem_sz = section_sz;
	vi_dev->shmem = devm_memremap(&pci_dev->dev, section_addr, section_sz,
				      MEMREMAP_WB);
	if (!vi_dev->shmem)
		return -ENOMEM;

	vi_dev->virtio_header = vi_dev->shmem;
	if (vi_dev->virtio_header->revision < 1) {
		dev_err(&pci_dev->dev, "invalid virtio-ivshmem revision\n");
		return -EINVAL;
	}

	spin_lock_init(&vi_dev->alloc_lock);

	chunk_size = vi_dev->shmem_sz / VIRTIO_IVSHMEM_PREFERRED_ALLOC_CHUNKS;
	if (chunk_size < SMP_CACHE_BYTES)
		chunk_size = SMP_CACHE_BYTES;
	if (chunk_size > PAGE_SIZE)
		chunk_size = PAGE_SIZE;
	vi_dev->alloc_shift = get_custom_order(chunk_size, 0);

	chunks = vi_dev->shmem_sz >> vi_dev->alloc_shift;
	bitmap_size = BITS_TO_LONGS(chunks) * sizeof(long);
	vi_dev->alloc_bitmap = devm_kzalloc(&pci_dev->dev,
					    bitmap_size,
					    GFP_KERNEL);
	if (!vi_dev->alloc_bitmap)
		return -ENOMEM;

	/* mark the header chunks used */
	bitmap_set(vi_dev->alloc_bitmap, 0,
		   1 << get_custom_order(vi_dev->virtio_header->size,
					 vi_dev->alloc_shift));

	vi_dev->map_src_addr = devm_kzalloc(&pci_dev->dev,
					    chunks * sizeof(void *),
					    GFP_KERNEL);
	if (!vi_dev->map_src_addr)
		return -ENOMEM;

	set_dma_ops(&pci_dev->dev, &virtio_ivshmem_dma_ops);

	pci_set_master(pci_dev);
	pci_write_config_byte(pci_dev, vendor_cap + IVSHM_CFG_PRIV_CNTL, 0);

	writel(VIRTIO_STATE_READY, &vi_dev->ivshm_regs->state);

	ret = register_virtio_device(&vi_dev->vdev);
	if (ret) {
		dev_err(&pci_dev->dev, "failed to register device\n");
		writel(0, &vi_dev->ivshm_regs->state);
		put_device(&vi_dev->vdev.dev);
	}

	return ret;
}

static void virtio_ivshmem_remove(struct pci_dev *pci_dev)
{
	struct virtio_ivshmem_device *vi_dev = pci_get_drvdata(pci_dev);

	writel(0, &vi_dev->ivshm_regs->state);
	writel(0, &vi_dev->ivshm_regs->int_control);

	unregister_virtio_device(&vi_dev->vdev);
}

static const struct pci_device_id virtio_ivshmem_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_SIEMENS, PCI_DEVICE_ID_IVSHMEM),
	  (PCI_CLASS_OTHERS << 16) | IVSHM_PROTO_VIRTIO_FRONT, 0xffff00 },
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, virtio_ivshmem_id_table);

static struct pci_driver virtio_ivshmem_driver = {
	.name		= DRV_NAME,
	.id_table	= virtio_ivshmem_id_table,
	.probe		= virtio_ivshmem_probe,
	.remove		= virtio_ivshmem_remove,
};

module_pci_driver(virtio_ivshmem_driver);

MODULE_AUTHOR("Jan Kiszka <jan.kiszka@siemens.com>");
MODULE_DESCRIPTION("Driver for ivshmem-based virtio front-end devices");
MODULE_LICENSE("GPL v2");
