// SPDX-License-Identifier: GPL-2.0
/*
 * adi-pcie-of-overlay: driver for PCI endpoints whose internals are described by
 * a firmware-provided device-tree overlay.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iommu-dma.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time64.h>

#define ADI_PCIE_INTC_BAR		0
#define ADI_PCIE_INTC_OFFSET		0x10000
#define ADI_PCIE_INTC_SIZE		0x10000

#define ADI_PCIE_INTC_MAGIC		0x00c
#define ADI_PCIE_INTC_MAGIC_VAL		0x494e5443	/* "INTC" */
#define ADI_PCIE_INTC_CONFIG		0x010
#define ADI_PCIE_INTC_CONFIG_NVEC	GENMASK(7, 0)
#define ADI_PCIE_INTC_CONFIG_NSRC	GENMASK(15, 8)
#define ADI_PCIE_INTC_CONFIG_TYPE	GENMASK(23, 16)

#define ADI_PCIE_INTC_TYPE_USR_IRQ	0
#define ADI_PCIE_INTC_TYPE_MSI		1

#define ADI_PCIE_INTC_VEC(v)			(0x200 + (v) * 0x20)
#define ADI_PCIE_INTC_VEC_ENABLE(v)		(ADI_PCIE_INTC_VEC(v) + 0x0)
#define ADI_PCIE_INTC_VEC_PENDING(v)		(ADI_PCIE_INTC_VEC(v) + 0x4)

/*
 * How long to wait for BAR0 to answer, and how often to look. An endpoint that
 * programs its own inbound translation cannot have done so before the host
 * assigned the BAR, so there is a window after enumeration in which BAR0 is
 * unreachable; a driver binds long after it, but nothing guarantees that.
 */
#define ADI_PCIE_INTC_READY_US		(100 * USEC_PER_MSEC)
#define ADI_PCIE_INTC_POLL_US		(1 * USEC_PER_MSEC)

#define ADI_PCIE_INTC_MAX_NVEC		16
#define ADI_PCIE_INTC_MAX_NSRC		32

struct adi_pcie_vector {
	struct adi_pcie_overlay *apo;
	unsigned int index;
	int parent_irq;
	u32 enable;
};

/**
 * struct adi_pcie_overlay - an endpoint, its interrupt controller and its overlay
 * @irq_domain:		domain over @nvec * @nsrc hwirqs.
 * @pdev:		the endpoint being driven.
 * @intc:		intc regs; NULL if none.
 * @iommu_nb:		notifier joining children to the endpoint's IOMMU group.
 * @lock:		guards @vec[].enable against the register it shadows.
 * @ovcs_id:		overlay changeset id, kept to remove what was applied.
 * @type:		type of intc interface.
 * @nsrc:		sources per irq vector.
 * @nvec:		irq vectors the controller implements.
 * @nirq:		PCI interrupts granted, which may be fewer than @nvec.
 * @stride:		vector stride when serving irq.
 * @vec:		@nvec per-vector entries.
 */
struct adi_pcie_overlay {
	struct irq_domain *irq_domain;
	struct pci_dev *pdev;
	void __iomem *intc;
	struct notifier_block iommu_nb;
	raw_spinlock_t lock;

	int ovcs_id;
	unsigned int type;
	unsigned int nsrc;
	unsigned int nvec;
	unsigned int nirq;
	unsigned int stride;
	struct adi_pcie_vector *vec;
};

static void adi_pcie_irq_enable_write(struct irq_data *d, bool on)
{
	struct adi_pcie_overlay *apo = irq_data_get_irq_chip_data(d);
	struct adi_pcie_vector *vec = &apo->vec[d->hwirq / apo->nsrc];
	u32 bit = BIT(d->hwirq % apo->nsrc);
	unsigned long flags;

	raw_spin_lock_irqsave(&apo->lock, flags);
	if (on)
		vec->enable |= bit;
	else
		vec->enable &= ~bit;
	writel(vec->enable, apo->intc + ADI_PCIE_INTC_VEC_ENABLE(vec->index));
	raw_spin_unlock_irqrestore(&apo->lock, flags);
}

static void adi_pcie_irq_mask(struct irq_data *d)
{
	adi_pcie_irq_enable_write(d, false);
}

static void adi_pcie_irq_unmask(struct irq_data *d)
{
	adi_pcie_irq_enable_write(d, true);
}

static void adi_pcie_irq_eoi(struct irq_data *d)
{
	struct adi_pcie_overlay *apo = irq_data_get_irq_chip_data(d);
	unsigned int v = d->hwirq / apo->nsrc;

	writel(BIT(d->hwirq % apo->nsrc), apo->intc + ADI_PCIE_INTC_VEC_PENDING(v));
}

static struct irq_chip adi_pcie_irq_chip = {
	.name		= "adi-pcie-of-overlay",
	.irq_mask	= adi_pcie_irq_mask,
	.irq_unmask	= adi_pcie_irq_unmask,
	.irq_eoi	= adi_pcie_irq_eoi,
};

static int adi_pcie_irq_map(struct irq_domain *d, unsigned int virq,
			    irq_hw_number_t hw)
{
	struct adi_pcie_overlay *apo = d->host_data;

	if (hw >= apo->nvec * apo->nsrc)
		return -EINVAL;

	if (apo->intc)
		irq_set_chip_and_handler(virq, &adi_pcie_irq_chip,
					 handle_fasteoi_irq);
	else
		irq_set_chip_and_handler(virq, &dummy_irq_chip,
					 handle_simple_irq);
	irq_set_chip_data(virq, apo);

	return 0;
}

static const struct irq_domain_ops adi_pcie_irq_domain_ops = {
	.map	= adi_pcie_irq_map,
	.xlate	= irq_domain_xlate_onecell,
};

static bool adi_pcie_vec_dispatch(struct adi_pcie_overlay *apo, unsigned int v)
{
	unsigned long pending = readl(apo->intc + ADI_PCIE_INTC_VEC_PENDING(v));
	unsigned int bit;

	if (!pending)
		return false;

	for_each_set_bit(bit, &pending, apo->nsrc) {
		if (!generic_handle_domain_irq(apo->irq_domain,
					       v * apo->nsrc + bit))
			continue;

		/*
		 * No flow handler ran, so nothing will eoi this bit and
		 * PENDING can never reach zero -- the vector would stop
		 * delivering for every source sharing it. Clear it here so the
		 * cost is a log line rather than a dead vector. Unreachable
		 * while ENABLE is only ever set from a mapped virq's unmask.
		 */
		writel(BIT(bit), apo->intc + ADI_PCIE_INTC_VEC_PENDING(v));
		dev_warn_ratelimited(&apo->pdev->dev,
				     "no handler for source %u (vector %u bit %u)\n",
				     v * apo->nsrc + bit, v, bit);
	}

	return true;
}

static void adi_pcie_intc_reset(struct adi_pcie_overlay *apo)
{
	unsigned int v;

	if (!apo->intc)
		return;

	for (v = 0; v < apo->nvec; v++) {
		apo->vec[v].enable = 0;
		writel(0, apo->intc + ADI_PCIE_INTC_VEC_ENABLE(v));
		writel(~0U, apo->intc + ADI_PCIE_INTC_VEC_PENDING(v));
	}
}

static unsigned int adi_pcie_msi_stride(struct pci_dev *pdev, unsigned int nirq)
{
	if (pdev->msix_enabled)
		return nirq;

	return rounddown_pow_of_two(nirq);
}

static void adi_pcie_msi_chained_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct adi_pcie_vector *vec = irq_desc_get_handler_data(desc);
	struct adi_pcie_overlay *apo = vec->apo;
	unsigned int v;

	chained_irq_enter(chip, desc);
	for (v = vec->index; v < apo->nvec; v += apo->stride)
		adi_pcie_vec_dispatch(apo, v);
	chained_irq_exit(chip, desc);
}

static void adi_pcie_msi_unchain(void *data)
{
	struct adi_pcie_overlay *apo = data;
	unsigned int k;

	adi_pcie_intc_reset(apo);
	for (k = 0; k < apo->nirq; k++)
		irq_set_chained_handler_and_data(apo->vec[k].parent_irq, NULL,
						 NULL);
}

static irqreturn_t adi_pcie_intx_handler(int irq, void *data)
{
	struct adi_pcie_overlay *apo = data;
	bool handled = false;
	unsigned int v;

	if (!apo->intc)
		return generic_handle_domain_irq(apo->irq_domain, 0) ?
			IRQ_NONE : IRQ_HANDLED;

	for (v = 0; v < apo->nvec; v++)
		handled |= adi_pcie_vec_dispatch(apo, v);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static void adi_pcie_intx_free(void *data)
{
	struct adi_pcie_overlay *apo = data;

	adi_pcie_intc_reset(apo);
	free_irq(apo->pdev->irq, apo);
}

static int adi_pcie_intc_probe(struct adi_pcie_overlay *apo)
{
	struct pci_dev *pdev = apo->pdev;
	struct device *dev = &pdev->dev;
	void __iomem *intc;
	u32 magic, config;
	int ret;

	apo->nvec = 1;
	apo->nsrc = 1;

	if (pci_resource_len(pdev, ADI_PCIE_INTC_BAR) <
	    ADI_PCIE_INTC_OFFSET + ADI_PCIE_INTC_SIZE)
		return 0;

	intc = pcim_iomap_range(pdev, ADI_PCIE_INTC_BAR, ADI_PCIE_INTC_OFFSET,
				ADI_PCIE_INTC_SIZE);
	if (IS_ERR(intc))
		return dev_err_probe(dev, PTR_ERR(intc),
				     "cannot map BAR%d\n", ADI_PCIE_INTC_BAR);

	/*
	 * All-ones is what an unanswered read completes as, and on an endpoint that
	 * programs its own inbound translation that is what the whole BAR reads
	 * until it has: enumeration assigns the BAR, and only then can anything on
	 * the far side act on it. So wait, rather than take the first read as the
	 * answer. Any other value means something is there and it is not this
	 * controller, which is a design without one -- not a device to wait for.
	 */
	ret = readl_poll_timeout(intc + ADI_PCIE_INTC_MAGIC, magic, magic != ~0U,
				 ADI_PCIE_INTC_POLL_US, ADI_PCIE_INTC_READY_US);
	if (ret) {
		pcim_iounmap(pdev, intc);
		return dev_err_probe(dev, ret,
				     "BAR%d still reads all-ones: the endpoint never made its own window reachable\n",
				     ADI_PCIE_INTC_BAR);
	}

	if (magic != ADI_PCIE_INTC_MAGIC_VAL) {
		pcim_iounmap(pdev, intc);
		return 0;
	}

	config = readl(intc + ADI_PCIE_INTC_CONFIG);
	apo->nvec = FIELD_GET(ADI_PCIE_INTC_CONFIG_NVEC, config);
	apo->nsrc = FIELD_GET(ADI_PCIE_INTC_CONFIG_NSRC, config);
	apo->type = FIELD_GET(ADI_PCIE_INTC_CONFIG_TYPE, config);

	if (!apo->nvec || apo->nvec > ADI_PCIE_INTC_MAX_NVEC ||
	    !apo->nsrc || apo->nsrc > ADI_PCIE_INTC_MAX_NSRC) {
		pcim_iounmap(pdev, intc);
		return dev_err_probe(dev, -EINVAL,
				     "Invalid config: %u nvec: %u nsrc\n",
				     apo->nvec, apo->nsrc);
	}

	apo->intc = intc;

	return 0;
}

static void adi_pcie_remove_irq_domain(void *data)
{
	irq_domain_remove(data);
}

static void adi_pcie_free_irq_vectors(void *data)
{
	pci_free_irq_vectors(data);
}

static int adi_pcie_alloc_irq_vectors(struct adi_pcie_overlay *apo)
{
	struct device *dev = &apo->pdev->dev;
	int ret = -ENOSPC;

	if (apo->type == ADI_PCIE_INTC_TYPE_MSI) {
		ret = pci_alloc_irq_vectors(apo->pdev, 1, apo->nvec,
					    PCI_IRQ_MSIX | PCI_IRQ_MSI);
	} else {
		if (apo->intc)
			ret = pci_alloc_irq_vectors(apo->pdev, apo->nvec,
						    apo->nvec,
						    PCI_IRQ_MSIX | PCI_IRQ_MSI);

		if (ret < 0) /* fallback to INTX */
			ret = pci_alloc_irq_vectors(apo->pdev, 1, 1,
						    PCI_IRQ_INTX);
	}
	if (ret < 0)
		return dev_err_probe(dev, ret, "IRQ vector allocation failed\n");

	apo->nirq = ret;

	return devm_add_action_or_reset(dev, adi_pcie_free_irq_vectors, apo->pdev);
}

static int adi_pcie_irq_domain_setup(struct adi_pcie_overlay *apo)
{
	struct device *dev = &apo->pdev->dev;
	struct fwnode_handle *fwnode;
	unsigned int v;
	int irq, ret;

	fwnode = of_fwnode_handle(dev_of_node(dev));
	if (!fwnode)
		return dev_err_probe(dev, -ENODEV,
				     "no of_node fwnode for irqdomain\n");

	apo->irq_domain = irq_domain_create_linear(fwnode, apo->nvec * apo->nsrc,
						   &adi_pcie_irq_domain_ops, apo);
	if (!apo->irq_domain)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to create irqdomain\n");

	ret = devm_add_action_or_reset(dev, adi_pcie_remove_irq_domain,
				       apo->irq_domain);
	if (ret)
		return ret;

	if (pci_dev_msi_enabled(apo->pdev)) {
		apo->stride = adi_pcie_msi_stride(apo->pdev, apo->nirq);

		for (v = 0; v < apo->nirq; v++) {
			ret = pci_irq_vector(apo->pdev, v);
			if (ret < 0)
				return dev_err_probe(dev, ret,
						     "pci_irq_vector(%u) failed\n",
						     v);
			apo->vec[v].parent_irq = ret;
		}

		for (v = 0; v < apo->nirq; v++)
			irq_set_chained_handler_and_data(apo->vec[v].parent_irq,
							 adi_pcie_msi_chained_handler,
							 &apo->vec[v]);

		return devm_add_action_or_reset(dev, adi_pcie_msi_unchain, apo);
	}

	apo->stride = 1;
	irq = pci_irq_vector(apo->pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "pci_irq_vector(0) failed\n");

	ret = request_irq(irq, adi_pcie_intx_handler, IRQF_SHARED, dev_name(dev),
			  apo);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to request INTx irq %d\n", irq);

	return devm_add_action_or_reset(dev, adi_pcie_intx_free, apo);
}

static int adi_pcie_iommu_join(struct adi_pcie_overlay *apo, struct device *dev)
{
	struct device *ep = &apo->pdev->dev;
	struct iommu_group *group;
	int ret;

	/* NULL only if the IOMMU driver went away since setup */
	group = iommu_group_get(ep);
	if (!group)
		return -ENODEV;

	ret = iommu_group_add_device(group, dev);
	iommu_group_put(group);
	if (ret)
		return ret;

	dev->dma_iommu = true;

	return 0;
}

static int adi_pcie_iommu_notify(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct adi_pcie_overlay *apo = container_of(nb, struct adi_pcie_overlay,
						    iommu_nb);
	struct device *ep = &apo->pdev->dev;
	struct device *dev = data;
	struct device *d;
	int ret;

	if (action != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	for (d = dev->parent; d && d != ep; d = d->parent)
		;
	if (!d)
		return NOTIFY_DONE;

	ret = adi_pcie_iommu_join(apo, dev);
	if (ret)
		dev_err(dev, "failed to join the endpoint IOMMU group: %d\n",
			ret);

	return NOTIFY_DONE;
}

static void adi_pcie_iommu_unregister(void *data)
{
	struct adi_pcie_overlay *apo = data;

	bus_unregister_notifier(&platform_bus_type, &apo->iommu_nb);
}

static int adi_pcie_iommu_setup(struct adi_pcie_overlay *apo)
{
	struct device *dev = &apo->pdev->dev;
	int ret;

	if (!use_dma_iommu(dev))
		return 0;

	apo->iommu_nb.notifier_call = adi_pcie_iommu_notify;
	ret = bus_register_notifier(&platform_bus_type, &apo->iommu_nb);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, adi_pcie_iommu_unregister, apo);
}

static void adi_pcie_overlay_remove(void *data)
{
	of_overlay_remove(data);
}

static void adi_pcie_overlay_depopulate(void *data)
{
	of_platform_depopulate(data);
}

static int adi_pcie_overlay_setup(struct adi_pcie_overlay *apo, const void *fdt,
				   size_t size)
{
	struct pci_dev *pdev = apo->pdev;
	struct device *dev = &pdev->dev;
	unsigned int v;
	int ret;

	if (!fdt || !size)
		return -EINVAL;

	/*
	 * PCI core only auto-creates an of_node for bridges and for a handful
	 * of quirked endpoint VID:DIDs; a generic FPGA endpoint won't have one
	 * yet. If it already exists (DT system, or an upstream quirk fired),
	 * the helper is a no-op and its creator retains ownership.
	 */
	ret = devm_of_pci_make_dev_node(pdev);
	if (ret)
		return ret;

	ret = of_overlay_fdt_apply(fdt, size, &apo->ovcs_id, dev_of_node(dev));
	if (ret)
		return dev_err_probe(dev, ret, "failed to apply overlay\n");

	ret = devm_add_action_or_reset(dev, adi_pcie_overlay_remove,
				       &apo->ovcs_id);
	if (ret)
		return ret;

	ret = adi_pcie_intc_probe(apo);
	if (ret)
		return ret;

	apo->vec = devm_kcalloc(dev, apo->nvec, sizeof(*apo->vec), GFP_KERNEL);
	if (!apo->vec)
		return -ENOMEM;

	for (v = 0; v < apo->nvec; v++) {
		apo->vec[v].apo = apo;
		apo->vec[v].index = v;
	}

	adi_pcie_intc_reset(apo);

	ret = adi_pcie_alloc_irq_vectors(apo);
	if (ret)
		return ret;

	ret = adi_pcie_iommu_setup(apo);
	if (ret)
		return ret;

	ret = adi_pcie_irq_domain_setup(apo);
	if (ret)
		return ret;

	ret = of_platform_default_populate(dev_of_node(dev), NULL, dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to populate platform devs\n");

	return devm_add_action_or_reset(dev, adi_pcie_overlay_depopulate, dev);
}

static int adi_pcie_overlay_firmware_load(struct adi_pcie_overlay *apo,
					  const char *fw_name)
{
	struct device *dev = &apo->pdev->dev;
	const struct firmware *fw;
	int ret;

	if (!fw_name)
		return -EINVAL;

	ret = request_firmware(&fw, fw_name, dev);
	if (ret)
		return dev_err_probe(dev, ret, "missing overlay %s\n", fw_name);

	ret = adi_pcie_overlay_setup(apo, fw->data, fw->size);
	release_firmware(fw);
	return ret;
}

static char *overlay;
module_param(overlay, charp, 0644);
MODULE_PARM_DESC(overlay, "DTB overlay firmware name to apply on next bind");

static int adi_pcie_overlay_probe(struct pci_dev *pdev,
				  const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct adi_pcie_overlay *apo;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);

	apo = devm_kzalloc(dev, sizeof(*apo), GFP_KERNEL);
	if (!apo)
		return -ENOMEM;

	apo->pdev = pdev;
	raw_spin_lock_init(&apo->lock);

	return adi_pcie_overlay_firmware_load(apo, overlay);
}

static struct pci_driver adi_pcie_driver = {
	.name		= "adi-pcie-of-overlay",
	.probe		= adi_pcie_overlay_probe,
};
module_pci_driver(adi_pcie_driver);

MODULE_DESCRIPTION("ADI PCIe endpoint device-tree overlay");
MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_LICENSE("GPL");
