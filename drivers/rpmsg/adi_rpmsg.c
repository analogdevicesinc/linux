// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Device RPMSG driver for SC5XX processors
 *
 * Copyright 2022 Analog Devices
 *
 * Author:
 *   Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/delay.h>

#include <linux/soc/adi/rcu.h>
#include <linux/soc/adi/icc.h>

#define ADI_INIT_TIMEOUT_MS (0)
#define __ADI_DELAY_MS (20)

#define ADI_RPMSG_FLAG_VRING_IN_DMA (1<<0)
#define ADI_RPMSG_FLAG_HAS_MEMORY_REGION (1<<1)

#define VRING_ALIGN 0x1000
#define VRING_DEFAULT_SIZE 0x800

enum adi_rpmsg_soc {
	SC598,
};

enum adi_rpmsg_state {
	ADI_RP_RPMSG_SYNCED = 0,
	ADI_RP_RPMSG_WAITING = 1,
	ADI_RP_RPMSG_TIMED_OUT = 2,
};

struct adi_sharc_resource_table {
	struct adi_resource_table_hdr adi_table_hdr;
	struct resource_table rsc_table;
} __packed;

struct adi_rpmsg_vring {
	struct virtqueue *vq;
	u32 da;
	u32 align;
	void *va;
	unsigned int size;
	unsigned int num;
	unsigned int id;
	unsigned int notify_id;
	struct adi_rpmsg_channel *rpchan;
};

struct adi_rpmsg_channel {
	struct platform_device *pdev;
	struct device *dev;
	enum adi_rpmsg_soc soc;
	struct adi_sharc_resource_table *adi_rsc_table;
	struct adi_rcu *rcu;
	struct adi_tru *tru;
	int icc_irq;
	int icc_irq_flags;
	int core_id;
	int flags;
	enum adi_rpmsg_state rpmsg_state;

	struct virtio_device vdev;
	struct adi_rpmsg_vring vring[2];

	/* Resource table for remote core*/
	struct fw_rsc_vdev *rsc_vdev;
	struct fw_rsc_vdev_vring *rsc_vring[2];
};

static u64 adi_rpmsg_get_features(struct virtio_device *vdev)
{
	/* 1<<0 is VIRTIO_RPMSG_F_NS bit defined in virtio_rpmsg_bus.c */
	return 1 << 0;
}

static int adi_rpmsg_finalize_features(struct virtio_device *vdev)
{
	vring_transport_features(vdev);
	return 0;
}

static bool adi_rpmsg_notify(struct virtqueue *vq)
{
	struct adi_rpmsg_vring *vring = vq->priv;
	struct adi_rpmsg_channel *rpchan = vring->rpchan;
	int wait_time, step;

	/* Delay a little the first notify and check if remote core has done its initialization */
	if (rpchan->rpmsg_state == ADI_RP_RPMSG_WAITING) {
		msleep(__ADI_DELAY_MS);
		if (rpchan->adi_rsc_table->adi_table_hdr.initialized == ADI_RSC_TABLE_INIT_MAGIC) {
			rpchan->rpmsg_state = ADI_RP_RPMSG_SYNCED;
		} else {
			dev_info(rpchan->dev,
				"Core%d resource table not initialized, delay first notify\n",
				rpchan->core_id);
			if (ADI_INIT_TIMEOUT_MS == 0) {
				/* Wait forever */
				step = 0;
				dev_info(rpchan->dev,
					 "Wait forever for Core%d resource table init\n",
					 rpchan->core_id);
			} else {
				step = __ADI_DELAY_MS;
			}

			for (wait_time = 0; wait_time <= ADI_INIT_TIMEOUT_MS; wait_time += step) {
				if (rpchan->adi_rsc_table->adi_table_hdr.initialized ==
				    ADI_RSC_TABLE_INIT_MAGIC) {
					dev_info(rpchan->dev,
						 "Core%d resource table initialized\n",
						 rpchan->core_id);
					rpchan->rpmsg_state = ADI_RP_RPMSG_SYNCED;
					break;
				}
				msleep(__ADI_DELAY_MS);
			}
			if (rpchan->rpmsg_state != ADI_RP_RPMSG_SYNCED) {
				rpchan->rpmsg_state = ADI_RP_RPMSG_TIMED_OUT;
				dev_info(rpchan->dev,
					 "Core%d rpmsg init timeout, probably not supported.\n",
					 rpchan->core_id);
			}
		}
	}

	if (rpchan->rpmsg_state == ADI_RP_RPMSG_SYNCED)
		adi_tru_trigger_device(rpchan->tru, rpchan->dev);

	return true;
}

static struct virtqueue *adi_find_vq(struct virtio_device *vdev,
				    unsigned int id,
				    void (*callback)(struct virtqueue *vq),
				    const char *name, bool ctx)
{
	struct adi_rpmsg_channel *rpchan = container_of(vdev, struct adi_rpmsg_channel, vdev);
	struct device *dev = rpchan->dev;
	void *addr;
	unsigned int num, size, align;
	struct virtqueue *vq;

	if (!name)
		return NULL;

	addr = rpchan->vring[id].va;
	size = rpchan->vring[id].size;
	num = rpchan->vring[id].num;
	align = rpchan->vring[id].align;

	memset(addr, 0, size);

	vq = vring_new_virtqueue(id, num, align, vdev, false, ctx,
				 addr, adi_rpmsg_notify, callback, name);
	if (!vq) {
		dev_err(dev, "vring_new_virtqueue %s failed\n", name);
		return ERR_PTR(-ENOMEM);
	}

	rpchan->vring[id].id = id;
	rpchan->vring[id].rpchan = rpchan;

	rpchan->vring[id].vq = vq;
	rpchan->vring[id].vq->priv = &rpchan->vring[id];

	return vq;
}

static void adi_rproc_virtio_del_vqs(struct virtio_device *vdev)
{
	struct virtqueue *vq, *n;
	struct adi_rpmsg_vring *vring;

	list_for_each_entry_safe(vq, n, &vdev->vqs, list) {
		vring = vq->priv;
		vring->vq = NULL;
		vring_del_virtqueue(vq);
	}
}

static int adi_rpmsg_virtio_find_vqs(struct virtio_device *vdev, unsigned int nvqs,
				 struct virtqueue *vqs[],
				 vq_callback_t *callbacks[],
				 const char * const names[],
				 const bool *ctx,
				 struct irq_affinity *desc)
{

	int i, ret;

	if (nvqs != 2)
		return -EINVAL;

	for (i = 0; i < nvqs; ++i) {
		if (!names[i]) {
			vqs[i] = NULL;
			continue;
		}

		vqs[i] = adi_find_vq(vdev, i, callbacks[i], names[i],
				    ctx ? ctx[i] : false);
		if (IS_ERR(vqs[i])) {
			ret = PTR_ERR(vqs[i]);
			goto error;
		}
	}

	return 0;

error:
	adi_rproc_virtio_del_vqs(vdev);
	return ret;
}

static u8 adi_virtio_get_status(struct virtio_device *vdev)
{
	struct adi_rpmsg_channel *rpchan = container_of(vdev, struct adi_rpmsg_channel, vdev);
	struct fw_rsc_vdev *rsc_vdev = rpchan->rsc_vdev;

	return rsc_vdev->status;
}

static void adi_virtio_set_status(struct virtio_device *vdev, u8 status)
{
	struct adi_rpmsg_channel *rpchan = container_of(vdev, struct adi_rpmsg_channel, vdev);
	struct fw_rsc_vdev *rsc_vdev = rpchan->rsc_vdev;

	rsc_vdev->status = status;
}

static void adi_virtio_reset(struct virtio_device *vdev)
{
	struct adi_rpmsg_channel *rpchan = container_of(vdev, struct adi_rpmsg_channel, vdev);
	struct fw_rsc_vdev *rsc_vdev = rpchan->rsc_vdev;

	rsc_vdev->status = 0;
}

static struct virtio_config_ops adi_rpmsg_config_ops = {
	.get_features	= adi_rpmsg_get_features,
	.finalize_features = adi_rpmsg_finalize_features,
	.find_vqs	= adi_rpmsg_virtio_find_vqs,
	.del_vqs	= adi_rproc_virtio_del_vqs,
	.reset		= adi_virtio_reset,
	.set_status	= adi_virtio_set_status,
	.get_status	= adi_virtio_get_status,
};

static int adi_rpmsg_parse_resource_table(struct adi_rpmsg_channel *rpchan)
{
	struct adi_sharc_resource_table *adi_rsc_table = rpchan->adi_rsc_table;
	struct device *dev = rpchan->dev;
	struct fw_rsc_hdr *hdr;
	int i, offset;

	if (strcmp(adi_rsc_table->adi_table_hdr.tag, ADI_RESOURCE_TABLE_TAG)) {
		dev_err(dev, "Corrupted resource table\n");
		return -ENODEV;
	}
	if (adi_rsc_table->adi_table_hdr.version != ADI_RESOURCE_TABLE_VERSION) {
		dev_err(dev, "Invalid resource table version\n");
		return -ENODEV;
	}

	/* Look for a VDEV entry */
	for (i = 0; i < adi_rsc_table->rsc_table.num; i++) {
		offset = adi_rsc_table->rsc_table.offset[i];
		hdr = (void *)&adi_rsc_table->rsc_table + offset;
		if (hdr->type == RSC_VDEV) {
			rpchan->rsc_vdev = (struct fw_rsc_vdev *)hdr->data;
			if (rpchan->rsc_vdev->id != VIRTIO_ID_RPMSG)
				continue;
			rpchan->rsc_vring[0] = &rpchan->rsc_vdev->vring[0];
			rpchan->rsc_vring[1] = &rpchan->rsc_vdev->vring[1];
			return 0;
		}
	}

	dev_err(dev, "Resource table doesn't have RSC_VDEV entry with VIRTIO_ID_RPMSG id.\n");
	return -ENODEV;
}

static void adi_rpmsg_vproc_release(struct device *dev)
{
}

static irqreturn_t adi_rpmsg_virtio_irq_threaded_handler(int irq, void *p)
{
	struct adi_rpmsg_channel *rpchan = (struct adi_rpmsg_channel *)p;

	vring_interrupt(0, rpchan->vring[0].vq);
	vring_interrupt(0, rpchan->vring[1].vq);

	return IRQ_HANDLED;
}

static int adi_rpmsg_probe(struct platform_device *pdev)
{
	struct adi_rpmsg_channel *rpchan;
	struct device *dev = &pdev->dev;
	struct adi_rcu *adi_rcu;
	struct adi_tru *adi_tru;
	struct device_node *dev_node = dev_of_node(&pdev->dev);
	struct device_node *node;
	struct resource *res;
	struct reserved_mem *rmem;
	dma_addr_t dma;
	void *va;
	int ret, size, num;

	rpchan = devm_kzalloc(dev, sizeof(struct adi_rpmsg_channel), GFP_KERNEL);
	if (!rpchan)
		return -ENOMEM;

	adi_tru = get_adi_tru_from_node(dev);
	if (IS_ERR(adi_tru))
		return PTR_ERR(adi_tru);

	adi_rcu = get_adi_rcu_from_node(dev);
	if (IS_ERR(adi_rcu)) {
		ret = PTR_ERR(adi_rcu);
		goto free_adi_tru;
	}

	platform_set_drvdata(pdev, rpchan);
	rpchan->pdev = pdev;
	rpchan->dev = dev;
	rpchan->soc = (enum adi_rpmsg_soc)of_device_get_match_data(dev);
	rpchan->tru = adi_tru;
	rpchan->rcu = adi_rcu;
	rpchan->rpmsg_state = ADI_RP_RPMSG_WAITING;

	ret = of_property_read_u32(dev_node, "core-id", &rpchan->core_id);
	if (ret) {
		dev_err(dev, "Unable to get core-id property\n");
		goto free_adi_rcu;
	}

	rpchan->icc_irq = platform_get_irq(pdev, 0);
	if (rpchan->icc_irq <= 0) {
		dev_err(dev, "No ICC IRQ specified\n");
		ret = -ENOENT;
		goto free_adi_rcu;
	}
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	rpchan->icc_irq_flags = (res->flags & IORESOURCE_BITS) |
				IRQF_PERCPU | IRQF_SHARED | IRQF_ONESHOT;

	ret = devm_request_threaded_irq(rpchan->dev, rpchan->icc_irq, NULL,
		adi_rpmsg_virtio_irq_threaded_handler, rpchan->icc_irq_flags,
		"ICC virtio IRQ", rpchan);
	if (ret) {
		dev_err(rpchan->dev, "Fail to request ICC IRQ\n");
		ret = -ENOENT;
		goto free_adi_rcu;
	}

	if (of_property_read_bool(dev_node, "adi,check-idle")) {
		ret = adi_rcu_is_core_idle(rpchan->rcu, rpchan->core_id);
		if (ret < 0) {
			dev_err(dev, "Invalid core-id\n");
			goto free_adi_rcu;
		} else if (ret > 0) {
			dev_err(dev, "Error: Core%d idle\n", rpchan->core_id);
			ret = -ENODEV;
			goto free_adi_rcu;
		}
	}

	/* Get ADI resource table address */
	node = of_parse_phandle(dev_node, "adi,rsc-table", 0);
	if (!node) {
		dev_err(&pdev->dev, "Can't find adi,rsc-table\n");
		ret = -EINVAL;
		goto free_adi_rcu;
	}
	rmem = of_reserved_mem_lookup(node);
	of_node_put(node);
	if (!rmem) {
		dev_err(&pdev->dev, "Translating adi,rsc-table failed\n");
		ret = -ENOMEM;
		goto free_adi_rcu;
	}

	rpchan->adi_rsc_table = devm_ioremap_wc(dev, rmem->base, rmem->size);
	if (IS_ERR(rpchan->adi_rsc_table)) {
		dev_err(dev, "Can't map adi,rsc-table\n");
		ret = PTR_ERR(rpchan->adi_rsc_table);
		goto free_adi_rcu;
	}

	ret = adi_rpmsg_parse_resource_table(rpchan);
	if (ret)
		goto free_adi_rcu;

	/* Check reserved memory for vrings */
	node = of_parse_phandle(dev_node, "vdev-vring", 0);
	if (node) {
		/* Vrings at specific reserved address */
		rmem = of_reserved_mem_lookup(node);
		of_node_put(node);
		if (!rmem) {
			dev_err(dev, "Failed to acquire vdev-vring\n");
			ret = -EINVAL;
			goto free_adi_rcu;
		}

		if (rmem->size < 0x4000) {
			dev_err(dev, "Insufficient space in vdev-vring, min space req is 16kB\n");
			ret = -EINVAL;
			goto free_adi_rcu;
		}

		/* Split the range for two vrings, vring0 -rx and vring1 - tx*/
		size = rmem->size / 2;

		/*
		 * Calc how many buffers we can fit in the vring region,
		 * number of buffers must be power of 2
		 */
		for (num = 2; num < 0x00400000; num <<= 1) {
			if (PAGE_ALIGN(vring_size(num, VRING_ALIGN)) > size) {
				num >>= 1; // It's too much, restore previous value and break
				break;
			}
		}

		va = devm_ioremap_wc(dev, rmem->base, rmem->size);
		if (!(va)) {
			dev_err(dev, "Unable to map vdev-vring\n");
			ret = -ENOMEM;
			goto free_adi_rcu;
		}

		dev_info(dev, "vrings in vdev-vring reserved-memory.\n");

		rpchan->vring[0].num = num;
		rpchan->vring[1].num = num;

		rpchan->vring[0].align = VRING_ALIGN;
		rpchan->vring[1].align = VRING_ALIGN;

		rpchan->vring[0].size = size;
		rpchan->vring[1].size = size;

		rpchan->vring[0].da = rmem->base;
		rpchan->vring[1].da = rpchan->vring[0].da + rpchan->vring[0].size;

		rpchan->vring[0].va = va;
		rpchan->vring[1].va = rpchan->vring[0].va + rpchan->vring[0].size;

	} else {
		/* Vrings from DMA pool */
		dev_info(dev, "vrings in generic DMA pool.\n");
		rpchan->flags |= ADI_RPMSG_FLAG_VRING_IN_DMA;

		rpchan->vring[0].num = rpchan->rsc_vring[0]->num;
		rpchan->vring[1].num = rpchan->rsc_vring[1]->num;

		rpchan->vring[0].align = rpchan->rsc_vring[0]->align;
		rpchan->vring[1].align = rpchan->rsc_vring[1]->align;

		rpchan->vring[0].size = PAGE_ALIGN(vring_size(rpchan->rsc_vring[0]->num,
							      rpchan->rsc_vring[0]->align));
		rpchan->vring[1].size = PAGE_ALIGN(vring_size(rpchan->rsc_vring[1]->num,
							      rpchan->rsc_vring[1]->align));
		va = dma_alloc_coherent(dev, rpchan->vring[0].size + rpchan->vring[1].size,
					&dma, GFP_KERNEL);
		if (!(va)) {
			dev_err(dev, "Unable to map vdev-vring\n");
			ret = -ENOMEM;
			goto free_adi_rcu;
		}

		rpchan->vring[0].da = dma;
		rpchan->vring[1].da = rpchan->vring[0].da + rpchan->vring[0].size;
		rpchan->vring[0].va = va;
		rpchan->vring[1].va = rpchan->vring[0].va + rpchan->vring[0].size;
	}

	/* Update resource table */
	rpchan->rsc_vring[0]->da = rpchan->vring[0].da;
	rpchan->rsc_vring[0]->align = rpchan->vring[0].align;
	rpchan->rsc_vring[0]->num = rpchan->vring[0].num;
	rpchan->rsc_vring[0]->notifyid = rpchan->core_id;

	rpchan->rsc_vring[1]->da = rpchan->vring[1].da;
	rpchan->rsc_vring[1]->align = rpchan->vring[1].align;
	rpchan->rsc_vring[1]->num = rpchan->vring[1].num;
	rpchan->rsc_vring[1]->notifyid = rpchan->core_id;

	if (of_reserved_mem_device_init(dev)) {
		dev_info(dev, "msg buffers in generic DMA pool.\n");
	} else {
		dev_info(dev, "msg buffers in memory-region.\n");
		rpchan->flags |= ADI_RPMSG_FLAG_HAS_MEMORY_REGION;
	}


	rpchan->vring[0].notify_id = rpchan->core_id;
	rpchan->vring[1].notify_id = rpchan->core_id;

	rpchan->vdev.id.device = VIRTIO_ID_RPMSG;
	rpchan->vdev.config = &adi_rpmsg_config_ops;
	rpchan->vdev.dev.parent = &rpchan->pdev->dev;
	rpchan->vdev.dev.release = adi_rpmsg_vproc_release;

	ret = register_virtio_device(&rpchan->vdev);
	if (ret) {
		dev_err(dev, "failed to register vdev\n");
		goto free_adi_rcu;
	}

	return 0;

free_adi_rcu:
	put_adi_rcu(adi_rcu);
free_adi_tru:
	put_adi_tru(adi_tru);

	return ret;
}

static int adi_rpmsg_remove(struct platform_device *pdev)
{
	struct adi_rpmsg_channel *rpchan = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int size;

	put_adi_rcu(rpchan->rcu);

	if (rpchan->flags & ADI_RPMSG_FLAG_HAS_MEMORY_REGION)
		of_reserved_mem_device_release(dev);

	if (rpchan->flags & ADI_RPMSG_FLAG_VRING_IN_DMA) {
		size = rpchan->vring[0].size + rpchan->vring[1].size;
		dma_free_coherent(dev, size, rpchan->vring[0].va, rpchan->vring[0].da);
	}
	return 0;
}

static const struct of_device_id adi_rpmsg_dt_ids[] = {
	{ .compatible = "adi,rpmsg-SC598", .data = (void *)SC598, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adi_rpmsg_dt_ids);

static struct platform_driver adi_rpmsg_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "adi-rpmsg",
		   .of_match_table = adi_rpmsg_dt_ids,
		   },
	.probe = adi_rpmsg_probe,
	.remove = adi_rpmsg_remove,
};
module_platform_driver(adi_rpmsg_driver);

MODULE_DESCRIPTION("Analog Devices rpmsg driver");
MODULE_AUTHOR("Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>");
MODULE_LICENSE("GPL v2");
