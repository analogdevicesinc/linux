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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/msi.h>
#include <linux/mutex.h>
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

#define ADI_PCIE_INTC_VERSION		0x000
#define ADI_PCIE_INTC_VERSION_MAJOR	GENMASK(31, 16)
#define ADI_PCIE_INTC_VERSION_MAJOR_VAL	1
#define ADI_PCIE_INTC_MAGIC		0x00c
#define ADI_PCIE_INTC_MAGIC_VAL		0x494e5443	/* "INTC" */
#define ADI_PCIE_INTC_CONFIG		0x010
#define ADI_PCIE_INTC_CONFIG_NVEC	GENMASK(7, 0)
#define ADI_PCIE_INTC_CONFIG_NSRC	GENMASK(15, 8)
#define ADI_PCIE_INTC_CONFIG_TYPE	GENMASK(23, 16)

#define ADI_PCIE_INTC_TYPE_USR		0
#define ADI_PCIE_INTC_TYPE_MSI		1

#define ADI_PCIE_INTC_SRC_ENABLE	0x040
#define ADI_PCIE_INTC_SRC_PENDING	0x044
#define ADI_PCIE_INTC_VEC_PENDING(v)	(0x200 + (v) * 0x20 + 0x14)
#define ADI_PCIE_INTC_SRC_ROUTE(s)	(0x400 + (s) * 4)
#define ADI_PCIE_INTC_MSIX_TABLE	0x8000

#define ADI_PCIE_INTC_READY_US		(100 * USEC_PER_MSEC)
#define ADI_PCIE_INTC_POLL_US		(1 * USEC_PER_MSEC)

#define ADI_PCIE_INTC_MAX_NVEC		16
#define ADI_PCIE_INTC_MAX_NSRC		32

struct adi_pcie_vector {
	struct adi_pcie_overlay *apo;
	unsigned int index;
	int parent_irq;
};

/**
 * struct adi_pcie_overlay - an endpoint, its interrupt controller and its overlay
 * @irq_domain:		domain over @nsrc hwirqs; hwirq is the flat source index.
 * @pdev:		the endpoint being driven.
 * @intc:		intc regs.
 * @vec:		@nvec per-vector entries.
 * @iommu_nb:		notifier joining children to the endpoint's IOMMU group.
 * @lock:		guards @irq_mask and the MSI-X table against their writers.
 * @ovcs_id:		overlay changeset id, kept to remove what was applied.
 * @nsrc:		interrupt sources the controller samples.
 * @nvec:		irq vectors the controller implements.
 * @nirq:		PCI interrupts granted, which may be fewer than @nvec.
 * @type:		endpoint the controller drives, CONFIG_TYPE.
 * @irq_mask:		SRC_ENABLE shadow.
 */
struct adi_pcie_overlay {
	struct irq_domain	*irq_domain;
	struct pci_dev		*pdev;
	void __iomem		*intc;
	struct adi_pcie_vector	*vec;
	struct notifier_block	iommu_nb;
	raw_spinlock_t		lock;

	int			ovcs_id;
	u32			nsrc;
	u32			nvec;
	u32			nirq;
	u32			type;
	u32			irq_mask;
};

static bool adi_pcie_has_msix_table(struct adi_pcie_overlay *apo)
{
	return apo->type == ADI_PCIE_INTC_TYPE_MSI;
}

static void __iomem *adi_pcie_msix_entry(struct adi_pcie_overlay *apo,
					 unsigned int v)
{
	return apo->intc + ADI_PCIE_INTC_MSIX_TABLE +
	       v * PCI_MSIX_ENTRY_SIZE;
}

/*
 * Called by the PCI core after every message write, so the table follows an
 * activate, an affinity move, a CPU going offline and a resume with nothing
 * here to drive it. This function reconciles the endpoint with the interrupt
 * controller when in plain MSI mode.
 */
static void adi_pcie_msi_write_msg(struct msi_desc *desc, void *data)
{
	struct adi_pcie_overlay *apo = data;
	unsigned int v, end;
	void __iomem *entry;
	unsigned long flags;

	/* the core skipped its own write too, and repeats it at resume */
	if (apo->pdev->current_state != PCI_D0)
		return;

	end = min(desc->msi_index + desc->nvec_used, apo->nirq);

	raw_spin_lock_irqsave(&apo->lock, flags);
	for (v = desc->msi_index; v < end; v++) {
		entry = adi_pcie_msix_entry(apo, v);

		writel(PCI_MSIX_ENTRY_CTRL_MASKBIT,
		       entry + PCI_MSIX_ENTRY_VECTOR_CTRL);
		writel(desc->msg.address_lo, entry + PCI_MSIX_ENTRY_LOWER_ADDR);
		writel(desc->msg.address_hi, entry + PCI_MSIX_ENTRY_UPPER_ADDR);
		writel(desc->msg.data + v - desc->msi_index,
		       entry + PCI_MSIX_ENTRY_DATA);

		if (desc->msg.address_lo || desc->msg.address_hi)
			writel(0, entry + PCI_MSIX_ENTRY_VECTOR_CTRL);
	}
	raw_spin_unlock_irqrestore(&apo->lock, flags);
}

static void adi_pcie_irq_enable_write(struct irq_data *d, bool on)
{
	struct adi_pcie_overlay *apo = irq_data_get_irq_chip_data(d);
	u32 bit = BIT(d->hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&apo->lock, flags);
	if (on)
		apo->irq_mask |= bit;
	else
		apo->irq_mask &= ~bit;
	writel(apo->irq_mask, apo->intc + ADI_PCIE_INTC_SRC_ENABLE);
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

	writel(BIT(d->hwirq), apo->intc + ADI_PCIE_INTC_SRC_PENDING);
}

static int adi_pcie_irq_set_affinity(struct irq_data *d,
				     const struct cpumask *mask, bool force)
{
	struct adi_pcie_overlay *apo = irq_data_get_irq_chip_data(d);
	unsigned int cpu, i, v, current_v;
	const struct cpumask *eff;

	if (!pci_dev_msi_enabled(apo->pdev))
		return -EINVAL;

	current_v = readl(apo->intc + ADI_PCIE_INTC_SRC_ROUTE(d->hwirq));
	if (current_v >= apo->nirq)
		current_v = 0;

	/* the vectors keep the cpus they were given, so route to one that fits */
	for (i = 0; i < apo->nirq; i++) {
		v = (current_v + i) % apo->nirq;
		eff = irq_get_effective_affinity_mask(apo->vec[v].parent_irq);
		if (eff && cpumask_intersects(eff, mask))
			break;
	}

	/* no vector serves this mask */
	if (i == apo->nirq)
		return -EINVAL;

	if (v != current_v)
		writel(v, apo->intc + ADI_PCIE_INTC_SRC_ROUTE(d->hwirq));

	cpu = cpumask_first_and(eff, mask);
	irq_data_update_effective_affinity(d, cpumask_of(cpu));

	return IRQ_SET_MASK_OK;
}

static struct irq_chip adi_pcie_irq_chip = {
	.name			= "ADI-PCI-OVERLAY",
	.irq_mask		= adi_pcie_irq_mask,
	.irq_unmask		= adi_pcie_irq_unmask,
	.irq_eoi		= adi_pcie_irq_eoi,
	.irq_set_affinity	= adi_pcie_irq_set_affinity,
};

static int adi_pcie_irq_map(struct irq_domain *d, unsigned int virq,
			    irq_hw_number_t hw)
{
	struct adi_pcie_overlay *apo = d->host_data;

	if (hw >= apo->nsrc)
		return -EINVAL;

	/* no flow handler completes a pending move, so set_affinity must run here */
	irq_set_status_flags(virq, IRQ_MOVE_PCNTXT);

	irq_set_chip_and_handler(virq, &adi_pcie_irq_chip, handle_fasteoi_irq);
	irq_set_chip_data(virq, apo);

	return 0;
}

static const struct irq_domain_ops adi_pcie_irq_domain_ops = {
	.map	= adi_pcie_irq_map,
	.xlate	= irq_domain_xlate_onecell,
};

static irqreturn_t adi_pcie_irq_dispatch(struct adi_pcie_overlay *apo, u32 set)
{
	unsigned long pending = set;
	unsigned int s;

	if (!pending)
		return IRQ_NONE;

	for_each_set_bit(s, &pending, apo->nsrc) {
		if (!generic_handle_domain_irq(apo->irq_domain, s))
			continue;

		/*
		 * No flow handler ran, so nothing will eoi this bit and
		 * PENDING can never reach zero -- the vector would stop
		 * delivering for every source sharing it. Clear it here so the
		 * cost is a log line rather than a dead vector.
		 */
		writel(BIT(s), apo->intc + ADI_PCIE_INTC_SRC_PENDING);
		dev_warn_ratelimited(&apo->pdev->dev,
				     "no handler for source %u\n", s);
	}

	return IRQ_HANDLED;
}

static void adi_pcie_intc_reset(struct adi_pcie_overlay *apo)
{
	unsigned long flags;
	unsigned int v;

	raw_spin_lock_irqsave(&apo->lock, flags);
	apo->irq_mask = 0;
	writel(0, apo->intc + ADI_PCIE_INTC_SRC_ENABLE);
	raw_spin_unlock_irqrestore(&apo->lock, flags);
	writel(~0U, apo->intc + ADI_PCIE_INTC_SRC_PENDING);

	if (!adi_pcie_has_msix_table(apo))
		return;

	/* whatever the last host left here is aimed at memory now reused */
	for (v = 0; v < apo->nvec; v++)
		writel(PCI_MSIX_ENTRY_CTRL_MASKBIT,
		       adi_pcie_msix_entry(apo, v) + PCI_MSIX_ENTRY_VECTOR_CTRL);
}

static void adi_pcie_msi_chained_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct adi_pcie_vector *vec = irq_desc_get_handler_data(desc);
	struct adi_pcie_overlay *apo = vec->apo;

	chained_irq_enter(chip, desc);
	adi_pcie_irq_dispatch(apo, readl(apo->intc + ADI_PCIE_INTC_VEC_PENDING(vec->index)));
	chained_irq_exit(chip, desc);
}

static void adi_pcie_msi_unchain(void *data)
{
	struct adi_pcie_overlay *apo = data;
	unsigned int k;

	adi_pcie_intc_reset(apo);
	for (k = 0; k < apo->nirq; k++)
		irq_set_chained_handler_and_data(apo->vec[k].parent_irq, NULL, NULL);
}

static irqreturn_t adi_pcie_intx_handler(int irq, void *data)
{
	struct adi_pcie_overlay *apo = data;

	return adi_pcie_irq_dispatch(apo, readl(apo->intc + ADI_PCIE_INTC_SRC_PENDING));
}

static void adi_pcie_intx_free(void *data)
{
	struct adi_pcie_overlay *apo = data;

	adi_pcie_intc_reset(apo);
	free_irq(apo->pdev->irq, apo);
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
	unsigned int i;
	int ret;

	apo->vec = devm_kcalloc(dev, apo->nvec, sizeof(*apo->vec), GFP_KERNEL);
	if (!apo->vec)
		return -ENOMEM;

	for (i = 0; i < apo->nvec; i++) {
		apo->vec[i].apo = apo;
		apo->vec[i].index = i;
	}

	adi_pcie_intc_reset(apo);

	ret = pci_alloc_irq_vectors(apo->pdev, 1, apo->nvec, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		return dev_err_probe(dev, ret, "no interrupt vectors\n");

	apo->nirq = ret;

	/* init source-vector mapping */
	for (i = 0; i < apo->nsrc; i++)
		writel(i % apo->nirq, apo->intc + ADI_PCIE_INTC_SRC_ROUTE(i));

	return devm_add_action_or_reset(dev, adi_pcie_free_irq_vectors, apo->pdev);
}

static void adi_pcie_msi_hook_install(struct adi_pcie_overlay *apo)
{
	struct device *dev = &apo->pdev->dev;
	struct msi_desc *desc;

	msi_lock_descs(dev);
	msi_for_each_desc(desc, dev, MSI_DESC_ASSOCIATED) {
		desc->write_msi_msg = adi_pcie_msi_write_msg;
		desc->write_msi_msg_data = apo;
		adi_pcie_msi_write_msg(desc, apo);
	}
	msi_unlock_descs(dev);
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

	apo->irq_domain = irq_domain_create_linear(fwnode, apo->nsrc,
						   &adi_pcie_irq_domain_ops, apo);
	if (!apo->irq_domain)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to create irqdomain\n");

	ret = devm_add_action_or_reset(dev, adi_pcie_remove_irq_domain,
				       apo->irq_domain);
	if (ret)
		return ret;

	if (pci_dev_msi_enabled(apo->pdev)) {
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

		ret = devm_add_action_or_reset(dev, adi_pcie_msi_unchain, apo);
		if (ret)
			return ret;

		if (apo->pdev->msi_enabled && adi_pcie_has_msix_table(apo))
			adi_pcie_msi_hook_install(apo);

		return 0;
	}

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

static int adi_pcie_intc_probe(struct adi_pcie_overlay *apo)
{
	struct pci_dev *pdev = apo->pdev;
	struct device *dev = &pdev->dev;
	u32 magic, version, config;
	int ret;

	if (pci_resource_len(pdev, ADI_PCIE_INTC_BAR) <
	    ADI_PCIE_INTC_OFFSET + ADI_PCIE_INTC_SIZE)
		return dev_err_probe(dev, -ENODEV, "BAR%d is too small\n",
				     ADI_PCIE_INTC_BAR);

	/* devres unmaps this, on a failure below as much as on unbind */
	apo->intc = pcim_iomap_range(pdev, ADI_PCIE_INTC_BAR,
				     ADI_PCIE_INTC_OFFSET, ADI_PCIE_INTC_SIZE);
	if (IS_ERR(apo->intc))
		return dev_err_probe(dev, PTR_ERR(apo->intc),
				     "cannot map BAR%d\n", ADI_PCIE_INTC_BAR);

	/* wait the device to be ready */
	ret = readl_poll_timeout(apo->intc + ADI_PCIE_INTC_MAGIC, magic,
				 magic != ~0U, ADI_PCIE_INTC_POLL_US,
				 ADI_PCIE_INTC_READY_US);
	if (ret)
		return dev_err_probe(dev, ret,
				     "the EP hasn't made BAR%d available\n",
				     ADI_PCIE_INTC_BAR);

	if (magic != ADI_PCIE_INTC_MAGIC_VAL)
		return dev_err_probe(dev, -ENODEV,
				     "no controller at BAR%d + %#x (magic %#x)\n",
				     ADI_PCIE_INTC_BAR, ADI_PCIE_INTC_OFFSET,
				     magic);

	version = readl(apo->intc + ADI_PCIE_INTC_VERSION);
	if (FIELD_GET(ADI_PCIE_INTC_VERSION_MAJOR, version) !=
	    ADI_PCIE_INTC_VERSION_MAJOR_VAL)
		return dev_err_probe(dev, -ENODEV,
				     "controller version %#x, expected major %u\n",
				     version, ADI_PCIE_INTC_VERSION_MAJOR_VAL);

	config = readl(apo->intc + ADI_PCIE_INTC_CONFIG);
	apo->nvec = FIELD_GET(ADI_PCIE_INTC_CONFIG_NVEC, config);
	apo->nsrc = FIELD_GET(ADI_PCIE_INTC_CONFIG_NSRC, config);
	apo->type = FIELD_GET(ADI_PCIE_INTC_CONFIG_TYPE, config);

	if (!apo->nvec || apo->nvec > ADI_PCIE_INTC_MAX_NVEC ||
	    !apo->nsrc || apo->nsrc > ADI_PCIE_INTC_MAX_NSRC)
		return dev_err_probe(dev, -EINVAL,
				     "invalid config: nvec %u nsrc %u\n",
				     apo->nvec, apo->nsrc);

	ret = adi_pcie_alloc_irq_vectors(apo);
	if (ret)
		return ret;

	ret = adi_pcie_iommu_setup(apo);
	if (ret)
		return ret;

	return adi_pcie_irq_domain_setup(apo);
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
	int ret;

	if (!fdt || !size)
		return -EINVAL;

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

	ret = of_platform_default_populate(dev_of_node(dev), NULL, dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to populate platform devs\n");

	return devm_add_action_or_reset(dev, adi_pcie_overlay_depopulate, dev);
}

static char *overlay;
module_param(overlay, charp, 0644);
MODULE_PARM_DESC(overlay, "DTB overlay firmware name to apply on next bind");

static int adi_pcie_overlay_probe(struct pci_dev *pdev,
				  const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct adi_pcie_overlay *apo;
	const struct firmware *fw;
	int ret;

	if (!overlay)
		return -EINVAL;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);

	apo = devm_kzalloc(dev, sizeof(*apo), GFP_KERNEL);
	if (!apo)
		return -ENOMEM;

	apo->pdev = pdev;
	raw_spin_lock_init(&apo->lock);

	ret = request_firmware(&fw, overlay, dev);
	if (ret)
		return dev_err_probe(dev, ret, "missing overlay %s\n", overlay);

	ret = adi_pcie_overlay_setup(apo, fw->data, fw->size);
	release_firmware(fw);

	return ret;
}

static struct pci_driver adi_pcie_driver = {
	.name		= "adi-pcie-of-overlay",
	.probe		= adi_pcie_overlay_probe,
};

module_pci_driver(adi_pcie_driver);

MODULE_DESCRIPTION("ADI PCIe endpoint device-tree overlay");
MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_LICENSE("GPL");
