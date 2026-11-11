// SPDX-License-Identifier: GPL-2.0
/*
 * Volume Management Device driver
 * Copyright (c) 2015, Intel Corporation.
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/irq-msi-lib.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/pci.h>
#include <linux/pci-acpi.h>
#include <linux/pci-ecam.h>
#include <linux/srcu.h>
#include <linux/rculist.h>
#include <linux/rcupdate.h>

#include <xen/xen.h>

#include <asm/irqdomain.h>

#define VMD_CFGBAR	0
#define VMD_MEMBAR1	2
#define VMD_MEMBAR2	4

/* VMD restriction value determines secondary start bus number */
#define VMD_RESTRICT_0_BUS_START 0x0
#define VMD_RESTRICT_1_BUS_START 0x80
#define VMD_RESTRICT_2_BUS_START 0xE0
#define VMD_RESTRICT_3_BUS_START 0xE1

#define PCI_REG_VMCAP		0x40
#define BUS_RESTRICT_CAP(vmcap)	(vmcap & 0x1)
#define PCI_REG_VMCONFIG	0x44
#define BUS_RESTRICT_CFG(vmcfg)	((vmcfg >> 8) & 0x3)
#define VMCONFIG_MSI_REMAP	0x2
#define PCI_REG_VMLOCK		0x70
#define MB2_SHADOW_EN(vmlock)	(vmlock & 0x2)

#define MB2_SHADOW_OFFSET	0x2000
#define MB2_SHADOW_SIZE		16

/* DMR BAR4 register offsets */
#define SHADOW_MEMBAR1_28C1		0x2818 /* MEMBAR1 physical address */
#define SHADOW_MEMBAR2_28C1		0x2820 /* MEMBAR2 physical address */
#define BASE_ID_REG_28C1		0x2840
#define MEMBAR2_OFFSET_28C1		0x30d0

/* Primary Bus Number for VMD devices on root bus 0 */
#define VMD_PRIMARY_BUS0    0x00
/* Primary Bus Number for VMD devices on root bus 1 */
#define VMD_PRIMARY_BUS1    0x80

#define VMD_BUSRANGE0       0xc8
#define VMD_BUSRANGE1       0xcc
#define VMD_MEMBAR1_OFFSET  0xd0
#define VMD_MEMBAR2_OFFSET1 0xd8
#define VMD_MEMBAR2_OFFSET2 0xdc
#define VMD_BUS_END(busr) FIELD_GET(GENMASK(15, 8), busr)
#define VMD_BUS_START(busr) FIELD_GET(GENMASK(7, 0), busr)

/*
 * Add VMD resources for BUS1, it will share the same MMIO space with
 * previous VMD resources.
 */
enum vmd_resource {
	VMD_RES_CFGBAR = 0,  /* VMD Bus0 Config BAR */
	VMD_RES_MBAR_1,      /* VMD Bus0 Resource MemBAR 1 */
	VMD_RES_MBAR_2,      /* VMD Bus0 Resource MemBAR 2 */
	VMD_RES_BUS1_CFGBAR, /* VMD Bus1 Config BAR */
	VMD_RES_BUS1_MBAR_1, /* VMD Bus1 Resource MemBAR 1 */
	VMD_RES_BUS1_MBAR_2, /* VMD Bus1 Resource MemBAR 2 */
	VMD_RES_COUNT
};

enum vmd_rootbus {
	VMD_BUS_0 = 0,
	VMD_BUS_1,
	VMD_BUS_COUNT
};

enum vmd_features {
	/*
	 * Device may contain registers which hint the physical location of the
	 * membars, in order to allow proper address translation during
	 * resource assignment to enable guest virtualization
	 */
	VMD_FEAT_HAS_MEMBAR_SHADOW		= (1 << 0),

	/*
	 * Device may provide root port configuration information which limits
	 * bus numbering
	 */
	VMD_FEAT_HAS_BUS_RESTRICTIONS		= (1 << 1),

	/*
	 * Device contains physical location shadow registers in
	 * vendor-specific capability space
	 */
	VMD_FEAT_HAS_MEMBAR_SHADOW_VSCAP	= (1 << 2),

	/*
	 * Device may use MSI-X vector 0 for software triggering and will not
	 * be used for MSI remapping
	 */
	VMD_FEAT_OFFSET_FIRST_VECTOR		= (1 << 3),

	/*
	 * Device can bypass remapping MSI-X transactions into its MSI-X table,
	 * avoiding the requirement of a VMD MSI domain for child device
	 * interrupt handling.
	 */
	VMD_FEAT_CAN_BYPASS_MSI_REMAP		= (1 << 4),

	/*
	 * Enable ASPM on the PCIE root ports and set the default LTR of the
	 * storage devices on platforms where these values are not configured by
	 * BIOS. This is needed for laptops, which require these settings for
	 * proper power management of the SoC.
	 */
	VMD_FEAT_BIOS_PM_QUIRK		= (1 << 5),

	/*
	 * Newer VMD with device ID 0x28C1 has unique settings compared to its
	 * predecessor where BIOS enumerates the entire VMD device tree and
	 * stores respective configurations including bus start range and
	 * shadow registers in VMD MMIO space in VMD BAR4/BAR5, otherwise
	 * referred to as MEMBAR2 or MSI-X BAR.
	 */
	VMD_FEAT_USE_BIOS_INFO		= (1 << 6),

	/*
	 * Some Intel VMD devices are affected by interrupt ordering errata,
	 * including MTL016 (0x7d0b) and ARL004 (0xad0b):
	 * the VMD may signal its MSI before the posted writes that carry the
	 * child device's DMA data have landed in memory.  The demuxed handler
	 * then runs against a not-yet-coherent completion queue and misses the
	 * completion entirely, so the I/O is only recovered when the block
	 * layer timeout fires and polls the queue ("timeout, completion
	 * polled").  Intel's documented workaround is to issue a dummy read
	 * to the MSI initiator (the child device) before handling the
	 * interrupt: the read completion cannot pass the device's earlier
	 * posted writes, so it pulls them into memory per PCIe ordering
	 * rules.  A read that terminates at the VMD itself does not order
	 * against the child's writes and is not sufficient (measured).
	 */
	VMD_FEAT_INTERRUPT_QUIRK	= (1 << 7),

	/*
	 * Starting from Intel Arrow Lake, VMD devices have their VMD root ports
	 * on the additional BUS1 root bus.
	 */
	VMD_FEAT_HAS_BUS1_ROOTBUS	= (1 << 8),
};

#define VMD_BIOS_PM_QUIRK_LTR	0x1003	/* 3145728 ns */

#define VMD_FEATS_CLIENT	(VMD_FEAT_HAS_MEMBAR_SHADOW_VSCAP |	\
				 VMD_FEAT_HAS_BUS_RESTRICTIONS |	\
				 VMD_FEAT_OFFSET_FIRST_VECTOR |		\
				 VMD_FEAT_BIOS_PM_QUIRK |		\
				 VMD_FEAT_HAS_BUS1_ROOTBUS)

static DEFINE_IDA(vmd_instance_ida);

/*
 * Lock for manipulating VMD IRQ lists.
 */
static DEFINE_RAW_SPINLOCK(list_lock);

/**
 * struct vmd_irq - private data to map driver IRQ to the VMD shared vector
 * @node:	list item for parent traversal.
 * @irq:	back pointer to parent.
 * @enabled:	true if driver enabled IRQ
 * @virq:	the virtual IRQ value provided to the requesting driver.
 * @flush_addr:	config space address of the initiating device, read before
 *		demuxing to flush its posted writes; NULL if the
 *		VMD is not affected.
 *
 * Every MSI/MSI-X IRQ requested for a device in a VMD domain will be mapped to
 * a VMD IRQ using this structure.
 */
struct vmd_irq {
	struct list_head	node;
	struct vmd_irq_list	*irq;
	bool			enabled;
	unsigned int		virq;
	void __iomem		*flush_addr;
};

struct vmd_dev;

static void __iomem *vmd_cfg_addr(struct vmd_dev *vmd, struct pci_bus *bus,
				  unsigned int devfn, int reg, int len);

/**
 * struct vmd_irq_list - list of driver requested IRQs mapping to a VMD vector
 * @irq_list:	the list of irq's the VMD one demuxes to.
 * @srcu:	SRCU struct for local synchronization.
 * @count:	number of child IRQs assigned to this vector; used to track
 *		sharing.
 * @virq:	The underlying VMD Linux interrupt number
 * @vmd:	back pointer to the owning VMD device; serializes the MTL016
 *		flush read against other config space access.
 */
struct vmd_irq_list {
	struct list_head	irq_list;
	struct srcu_struct	srcu;
	unsigned int		count;
	unsigned int		virq;
	struct vmd_dev		*vmd;
};

struct vmd_dev {
	struct pci_dev		*dev;

	raw_spinlock_t		cfg_lock;
	void __iomem		*cfgbar;

	int msix_count;
	struct vmd_irq_list	*irqs;

	struct pci_sysdata	sysdata[VMD_BUS_COUNT];
	struct resource		resources[VMD_RES_COUNT];
	struct irq_domain	*irq_domain;
	struct pci_bus		*bus[VMD_BUS_COUNT];
	u8			busn_start[VMD_BUS_COUNT];
	u8			first_vec;
	char			*name;
	int			instance;
	unsigned long		features;
	bool			bus1_rootbus;
};

static inline struct vmd_dev *vmd_from_bus(struct pci_bus *bus)
{
	struct pci_sysdata *sd = bus->sysdata;

	return pci_get_drvdata(sd->vmd_dev);

}

static inline unsigned int index_from_irqs(struct vmd_dev *vmd,
					   struct vmd_irq_list *irqs)
{
	return irqs - vmd->irqs;
}

/*
 * Drivers managing a device in a VMD domain allocate their own IRQs as before,
 * but the MSI entry for the hardware it's driving will be programmed with a
 * destination ID for the VMD MSI-X table.  The VMD muxes interrupts in its
 * domain into one of its own, and the VMD driver de-muxes these for the
 * handlers sharing that VMD IRQ.  The vmd irq_domain provides the operations
 * and irq_chip to set this up.
 */
static void vmd_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct vmd_irq *vmdirq = data->chip_data;
	struct vmd_irq_list *irq = vmdirq->irq;
	struct vmd_dev *vmd = irq_data_get_irq_handler_data(data);

	memset(msg, 0, sizeof(*msg));
	msg->address_hi = X86_MSI_BASE_ADDRESS_HIGH;
	msg->arch_addr_lo.base_address = X86_MSI_BASE_ADDRESS_LOW;
	msg->arch_addr_lo.destid_0_7 = index_from_irqs(vmd, irq);
}

static void vmd_irq_enable(struct irq_data *data)
{
	struct vmd_irq *vmdirq = data->chip_data;

	scoped_guard(raw_spinlock_irqsave, &list_lock) {
		WARN_ON(vmdirq->enabled);
		list_add_tail_rcu(&vmdirq->node, &vmdirq->irq->irq_list);
		vmdirq->enabled = true;
	}
}

static void vmd_pci_msi_enable(struct irq_data *data)
{
	vmd_irq_enable(data->parent_data);
	data->chip->irq_unmask(data);
}

static unsigned int vmd_pci_msi_startup(struct irq_data *data)
{
	vmd_pci_msi_enable(data);
	return 0;
}

static void vmd_irq_disable(struct irq_data *data)
{
	struct vmd_irq *vmdirq = data->chip_data;

	scoped_guard(raw_spinlock_irqsave, &list_lock) {
		if (vmdirq->enabled) {
			list_del_rcu(&vmdirq->node);
			vmdirq->enabled = false;
		}
	}
}

static void vmd_pci_msi_disable(struct irq_data *data)
{
	data->chip->irq_mask(data);
	vmd_irq_disable(data->parent_data);
}

static void vmd_pci_msi_shutdown(struct irq_data *data)
{
	vmd_pci_msi_disable(data);
}

static struct irq_chip vmd_msi_controller = {
	.name			= "VMD-MSI",
	.irq_compose_msi_msg	= vmd_compose_msi_msg,
};

/*
 * XXX: We can be even smarter selecting the best IRQ once we solve the
 * affinity problem.
 */
static struct vmd_irq_list *vmd_next_irq(struct vmd_dev *vmd, struct msi_desc *desc)
{
	int i, best;

	if (vmd->msix_count == 1 + vmd->first_vec)
		return &vmd->irqs[vmd->first_vec];

	/*
	 * White list for fast-interrupt handlers. All others will share the
	 * "slow" interrupt vector.
	 */
	switch (msi_desc_to_pci_dev(desc)->class) {
	case PCI_CLASS_STORAGE_EXPRESS:
		break;
	default:
		return &vmd->irqs[vmd->first_vec];
	}

	scoped_guard(raw_spinlock_irq, &list_lock) {
		best = vmd->first_vec + 1;
		for (i = best; i < vmd->msix_count; i++)
			if (vmd->irqs[i].count < vmd->irqs[best].count)
				best = i;
		vmd->irqs[best].count++;
	}

	return &vmd->irqs[best];
}

static void vmd_msi_free(struct irq_domain *domain, unsigned int virq,
			 unsigned int nr_irqs);

static int vmd_msi_alloc(struct irq_domain *domain, unsigned int virq,
			 unsigned int nr_irqs, void *arg)
{
	struct msi_desc *desc = ((msi_alloc_info_t *)arg)->desc;
	struct vmd_dev *vmd = domain->host_data;
	struct vmd_irq *vmdirq;

	for (int i = 0; i < nr_irqs; ++i) {
		vmdirq = kzalloc_obj(*vmdirq);
		if (!vmdirq) {
			vmd_msi_free(domain, virq, i);
			return -ENOMEM;
		}

		INIT_LIST_HEAD(&vmdirq->node);
		vmdirq->irq = vmd_next_irq(vmd, desc);
		vmdirq->virq = virq + i;
		if (vmd->features & VMD_FEAT_INTERRUPT_QUIRK) {
			struct pci_dev *pdev = msi_desc_to_pci_dev(desc);

			vmdirq->flush_addr = vmd_cfg_addr(vmd, pdev->bus,
							  pdev->devfn,
							  PCI_VENDOR_ID, 2);
		}

		irq_domain_set_info(domain, virq + i, vmdirq->irq->virq,
				    &vmd_msi_controller, vmdirq,
				    handle_untracked_irq, vmd, NULL);
	}

	return 0;
}

static void vmd_msi_free(struct irq_domain *domain, unsigned int virq,
			 unsigned int nr_irqs)
{
	struct irq_data *irq_data;
	struct vmd_irq *vmdirq;

	for (int i = 0; i < nr_irqs; ++i) {
		irq_data = irq_domain_get_irq_data(domain, virq + i);
		vmdirq = irq_data->chip_data;

		synchronize_srcu(&vmdirq->irq->srcu);

		/* XXX: Potential optimization to rebalance */
		scoped_guard(raw_spinlock_irq, &list_lock)
			vmdirq->irq->count--;

		kfree(vmdirq);
	}
}

static const struct irq_domain_ops vmd_msi_domain_ops = {
	.alloc		= vmd_msi_alloc,
	.free		= vmd_msi_free,
};

static bool vmd_init_dev_msi_info(struct device *dev, struct irq_domain *domain,
				  struct irq_domain *real_parent,
				  struct msi_domain_info *info)
{
	if (!msi_lib_init_dev_msi_info(dev, domain, real_parent, info))
		return false;

	info->chip->irq_startup		= vmd_pci_msi_startup;
	info->chip->irq_shutdown	= vmd_pci_msi_shutdown;
	info->chip->irq_enable		= vmd_pci_msi_enable;
	info->chip->irq_disable		= vmd_pci_msi_disable;
	return true;
}

#define VMD_MSI_FLAGS_SUPPORTED	(MSI_GENERIC_FLAGS_MASK | MSI_FLAG_PCI_MSIX)
#define VMD_MSI_FLAGS_REQUIRED	(MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_NO_AFFINITY)

static const struct msi_parent_ops vmd_msi_parent_ops = {
	.supported_flags	= VMD_MSI_FLAGS_SUPPORTED,
	.required_flags		= VMD_MSI_FLAGS_REQUIRED,
	.bus_select_token	= DOMAIN_BUS_VMD_MSI,
	.bus_select_mask	= MATCH_PCI_MSI,
	.prefix			= "VMD-",
	.init_dev_msi_info	= vmd_init_dev_msi_info,
};

static int vmd_create_irq_domain(struct vmd_dev *vmd)
{
	struct irq_domain_info info = {
		.size		= vmd->msix_count,
		.ops		= &vmd_msi_domain_ops,
		.host_data	= vmd,
	};

	info.fwnode = irq_domain_alloc_named_id_fwnode("VMD-MSI",
						       vmd->sysdata[VMD_BUS_0].domain);
	if (!info.fwnode)
		return -ENODEV;

	vmd->irq_domain = msi_create_parent_irq_domain(&info,
						       &vmd_msi_parent_ops);
	if (!vmd->irq_domain) {
		irq_domain_free_fwnode(info.fwnode);
		return -ENODEV;
	}

	return 0;
}

static void vmd_set_msi_remapping(struct vmd_dev *vmd, bool enable)
{
	u16 reg;

	if (!!(vmd->features & VMD_FEAT_USE_BIOS_INFO))
		return;

	pci_read_config_word(vmd->dev, PCI_REG_VMCONFIG, &reg);
	reg = enable ? (reg & ~VMCONFIG_MSI_REMAP) :
		       (reg | VMCONFIG_MSI_REMAP);
	pci_write_config_word(vmd->dev, PCI_REG_VMCONFIG, reg);
}

static void vmd_remove_irq_domain(struct vmd_dev *vmd)
{
	/*
	 * Some production BIOS won't enable remapping between soft reboots.
	 * Ensure remapping is restored before unloading the driver.
	 */
	if (!vmd->msix_count)
		vmd_set_msi_remapping(vmd, true);

	if (vmd->irq_domain) {
		struct fwnode_handle *fn = vmd->irq_domain->fwnode;

		irq_domain_remove(vmd->irq_domain);
		irq_domain_free_fwnode(fn);
	}
}

static unsigned int vmd_bus_to_ecam(struct vmd_dev *vmd, unsigned int busnr)
{
	if (!!(vmd->features & VMD_FEAT_USE_BIOS_INFO))
		return busnr;

	return busnr - vmd->busn_start[VMD_BUS_0];
}

static void __iomem *vmd_cfg_addr(struct vmd_dev *vmd, struct pci_bus *bus,
				  unsigned int devfn, int reg, int len)
{
	unsigned int busnr_ecam;
	u32 offset;

	busnr_ecam = vmd_bus_to_ecam(vmd, bus->number);
	offset = PCIE_ECAM_OFFSET(busnr_ecam, devfn, reg);

	if (offset + len >= resource_size(&vmd->dev->resource[VMD_CFGBAR]))
		return NULL;

	return vmd->cfgbar + offset;
}

static bool vmd_is_bus1_root_bridge(struct vmd_dev *vmd, struct pci_bus *bus,
				    unsigned int devfn)
{
	void __iomem *hdr_addr;
	u8 hdr_type;

	lockdep_assert_held(&vmd->cfg_lock);

	if (!vmd->bus1_rootbus)
		return false;

	/* Limit strictly to BUS1 */
	if (bus->sysdata != &vmd->sysdata[VMD_BUS_1])
		return false;

	if (!pci_is_root_bus(bus))
		return false;

	/*
	 * Offset 0x18 is PCI_PRIMARY_BUS only for Type 1 bridge headers.
	 * On Type 0 endpoints, 0x18 is BAR2. Only apply the quirk to bridges.
	 */
	hdr_addr = vmd_cfg_addr(vmd, bus, devfn, PCI_HEADER_TYPE, 1);
	if (!hdr_addr)
		return false;

	hdr_type = readb(hdr_addr) & PCI_HEADER_TYPE_MASK;
	return hdr_type == PCI_HEADER_TYPE_BRIDGE;
}

/*
 * CPU may deadlock if config space is not serialized on some versions of this
 * hardware, so all config space access is done under a spinlock.
 */
static int vmd_pci_read(struct pci_bus *bus, unsigned int devfn, int reg,
			int len, u32 *value)
{
	struct vmd_dev *vmd = vmd_from_bus(bus);
	void __iomem *addr = vmd_cfg_addr(vmd, bus, devfn, reg, len);
	int ret = 0;

	if (!addr)
		return -EFAULT;

	guard(raw_spinlock_irqsave)(&vmd->cfg_lock);
	switch (len) {
	case 1:
		*value = readb(addr);
		break;
	case 2:
		*value = readw(addr);
		break;
	case 4:
		*value = readl(addr);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	/*
	 * Quirk: Hardware PCIe root port bridges on BUS1 have
	 * PCI_PRIMARY_BUS hardwired to 0x80 (VMD_PRIMARY_BUS1).
	 *
	 * Emulate the primary bus number to match the actual allocated
	 * root bus number. This prevents the PCI core from detecting a
	 * bus mismatch without needing synthetic bus ranges.
	 */
	if (!ret && reg <= PCI_PRIMARY_BUS && reg + len > PCI_PRIMARY_BUS &&
	    vmd_is_bus1_root_bridge(vmd, bus, devfn)) {
		u8 *buf = (u8 *)value;

		buf[PCI_PRIMARY_BUS - reg] = bus->number;
	}

	return ret;
}

/*
 * VMD h/w converts non-posted config writes to posted memory writes. The
 * read-back in this function forces the completion so it returns only after
 * the config space was written, as expected.
 */
static int vmd_pci_write(struct pci_bus *bus, unsigned int devfn, int reg,
			 int len, u32 value)
{
	struct vmd_dev *vmd = vmd_from_bus(bus);
	void __iomem *addr = vmd_cfg_addr(vmd, bus, devfn, reg, len);

	if (!addr)
		return -EFAULT;

	guard(raw_spinlock_irqsave)(&vmd->cfg_lock);
	switch (len) {
	case 1:
		writeb(value, addr);
		readb(addr);
		return 0;
	case 2:
		writew(value, addr);
		readw(addr);
		return 0;
	case 4:
		writel(value, addr);
		readl(addr);
		return 0;
	default:
		return -EINVAL;
	}
}

static struct pci_ops vmd_ops = {
	.read		= vmd_pci_read,
	.write		= vmd_pci_write,
};

#ifdef CONFIG_ACPI
static struct acpi_device *vmd_acpi_find_companion(struct pci_dev *pci_dev)
{
	struct pci_host_bridge *bridge;
	u32 busnr, addr;

	if (pci_dev->bus->ops != &vmd_ops)
		return NULL;

	bridge = pci_find_host_bridge(pci_dev->bus);
	busnr = pci_dev->bus->number - bridge->bus->number;
	/*
	 * The address computation below is only applicable to relative bus
	 * numbers below 32.
	 */
	if (busnr > 31)
		return NULL;

	addr = (busnr << 24) | ((u32)pci_dev->devfn << 16) | 0x8000FFFFU;

	dev_dbg(&pci_dev->dev, "Looking for ACPI companion (address 0x%x)\n",
		addr);

	return acpi_find_child_device(ACPI_COMPANION(bridge->dev.parent), addr,
				      false);
}

static bool hook_installed;

static void vmd_acpi_begin(void)
{
	if (pci_acpi_set_companion_lookup_hook(vmd_acpi_find_companion))
		return;

	hook_installed = true;
}

static void vmd_acpi_end(void)
{
	if (!hook_installed)
		return;

	pci_acpi_clear_companion_lookup_hook();
	hook_installed = false;
}
#else
static inline void vmd_acpi_begin(void) { }
static inline void vmd_acpi_end(void) { }
#endif /* CONFIG_ACPI */

static resource_size_t vmd_cfgbar_ecam_space(struct vmd_dev *vmd)
{
	resource_size_t cfgbar_buses;
	unsigned int ecam_start;

	cfgbar_buses = resource_size(&vmd->dev->resource[VMD_CFGBAR]) >> 20;

	if (vmd->bus1_rootbus)
		/* BUS1 splits the aperture, so reset scans start from BUS0. */
		ecam_start = vmd_bus_to_ecam(vmd, vmd->busn_start[VMD_BUS_0]);
	else
		ecam_start = vmd_bus_to_ecam(vmd, vmd->resources[VMD_RES_CFGBAR].start);

	if (ecam_start >= cfgbar_buses)
		return 0;

	return cfgbar_buses - ecam_start;
}

static void vmd_domain_reset(struct vmd_dev *vmd)
{
	u16 bus, max_buses = resource_size(&vmd->resources[VMD_RES_CFGBAR]);
	u8 dev, functions, fn, hdr_type;
	unsigned int ecam_bus;
	char __iomem *base;

	if (vmd->bus1_rootbus)
		max_buses = resource_size(&vmd->dev->resource[VMD_CFGBAR]) >> 20;

	max_buses = min_t(u16, max_buses, vmd_cfgbar_ecam_space(vmd));
	for (bus = 0; bus < max_buses; bus++) {
		/* Calculate the ECAM bus nr starting from the first bus of VMD_BUS_0 */
		ecam_bus = vmd_bus_to_ecam(vmd, vmd->busn_start[VMD_BUS_0] + bus);
		for (dev = 0; dev < 32; dev++) {
			base = vmd->cfgbar + PCIE_ECAM_OFFSET(ecam_bus,
						PCI_DEVFN(dev, 0), 0);

			hdr_type = readb(base + PCI_HEADER_TYPE);

			functions = (hdr_type & PCI_HEADER_TYPE_MFD) ? 8 : 1;
			for (fn = 0; fn < functions; fn++) {
				base = vmd->cfgbar + PCIE_ECAM_OFFSET(ecam_bus,
						PCI_DEVFN(dev, fn), 0);

				hdr_type = readb(base + PCI_HEADER_TYPE) &
						PCI_HEADER_TYPE_MASK;

				if (hdr_type != PCI_HEADER_TYPE_BRIDGE ||
				    (readw(base + PCI_CLASS_DEVICE) !=
				     PCI_CLASS_BRIDGE_PCI))
					continue;

				/*
				 * Temporarily disable the I/O range before updating
				 * PCI_IO_BASE.
				 */
				writel(0x0000ffff, base + PCI_IO_BASE_UPPER16);
				/* Update lower 16 bits of I/O base/limit */
				writew(0x00f0, base + PCI_IO_BASE);
				/* Update upper 16 bits of I/O base/limit */
				writel(0, base + PCI_IO_BASE_UPPER16);

				/* MMIO Base/Limit */
				writel(0x0000fff0, base + PCI_MEMORY_BASE);

				/* Prefetchable MMIO Base/Limit */
				writel(0, base + PCI_PREF_LIMIT_UPPER32);
				writel(0x0000fff0, base + PCI_PREF_MEMORY_BASE);
				writel(0xffffffff, base + PCI_PREF_BASE_UPPER32);
			}
		}
	}
}

static int vmd_attach_resources(struct vmd_dev *vmd)
{
	int ret;

	ret = request_resource(&vmd->dev->resource[VMD_MEMBAR1],
			       &vmd->resources[VMD_RES_MBAR_1]);
	if (ret)
		return ret;

	ret = request_resource(&vmd->dev->resource[VMD_MEMBAR2],
			       &vmd->resources[VMD_RES_MBAR_2]);
	if (ret) {
		release_resource(&vmd->resources[VMD_RES_MBAR_1]);
		return ret;
	}

	if (vmd->bus1_rootbus && vmd->sysdata[VMD_BUS_1].domain >= 0) {
		ret = request_resource(&vmd->dev->resource[VMD_MEMBAR1],
				       &vmd->resources[VMD_RES_BUS1_MBAR_1]);
		if (ret) {
			release_resource(&vmd->resources[VMD_RES_MBAR_1]);
			release_resource(&vmd->resources[VMD_RES_MBAR_2]);
			return ret;
		}

		ret = request_resource(&vmd->dev->resource[VMD_MEMBAR2],
				       &vmd->resources[VMD_RES_BUS1_MBAR_2]);
		if (ret) {
			release_resource(&vmd->resources[VMD_RES_MBAR_1]);
			release_resource(&vmd->resources[VMD_RES_MBAR_2]);
			release_resource(&vmd->resources[VMD_RES_BUS1_MBAR_1]);
			return ret;
		}
	}

	return 0;
}

static void vmd_detach_resources(struct vmd_dev *vmd)
{
	if (vmd->bus1_rootbus && vmd->sysdata[VMD_BUS_1].domain >= 0) {
		release_resource(&vmd->resources[VMD_RES_BUS1_MBAR_2]);
		release_resource(&vmd->resources[VMD_RES_BUS1_MBAR_1]);
	}

	release_resource(&vmd->resources[VMD_RES_MBAR_2]);
	release_resource(&vmd->resources[VMD_RES_MBAR_1]);
}

static int vmd_get_phys_offsets(struct vmd_dev *vmd, bool native_hint,
				resource_size_t *offset1,
				resource_size_t *offset2)
{
	struct pci_dev *dev = vmd->dev;
	u64 phys1, phys2;

	if (native_hint) {
		u32 vmlock;
		int ret;

		ret = pci_read_config_dword(dev, PCI_REG_VMLOCK, &vmlock);
		if (ret || PCI_POSSIBLE_ERROR(vmlock))
			return -ENODEV;

		if (MB2_SHADOW_EN(vmlock)) {
			void __iomem *membar2;

			membar2 = pci_iomap(dev, VMD_MEMBAR2, 0);
			if (!membar2)
				return -ENOMEM;
			phys1 = readq(membar2 + MB2_SHADOW_OFFSET);
			phys2 = readq(membar2 + MB2_SHADOW_OFFSET + 8);
			pci_iounmap(dev, membar2);
		} else
			return 0;
	} else {
		/* Hypervisor-Emulated Vendor-Specific Capability */
		int pos = pci_find_capability(dev, PCI_CAP_ID_VNDR);
		u32 reg, regu;

		pci_read_config_dword(dev, pos + 4, &reg);

		/* "SHDW" */
		if (pos && reg == 0x53484457) {
			pci_read_config_dword(dev, pos + 8, &reg);
			pci_read_config_dword(dev, pos + 12, &regu);
			phys1 = (u64) regu << 32 | reg;

			pci_read_config_dword(dev, pos + 16, &reg);
			pci_read_config_dword(dev, pos + 20, &regu);
			phys2 = (u64) regu << 32 | reg;
		} else
			return 0;
	}

	*offset1 = dev->resource[VMD_MEMBAR1].start -
			(phys1 & PCI_BASE_ADDRESS_MEM_MASK);
	*offset2 = dev->resource[VMD_MEMBAR2].start -
			(phys2 & PCI_BASE_ADDRESS_MEM_MASK);

	return 0;
}

static int vmd_get_bus_number_start(struct vmd_dev *vmd, unsigned long features)
{
	struct pci_dev *dev = vmd->dev;
	u16 reg;

	pci_read_config_word(dev, PCI_REG_VMCAP, &reg);
	if (BUS_RESTRICT_CAP(reg)) {
		pci_read_config_word(dev, PCI_REG_VMCONFIG, &reg);
		if (PCI_POSSIBLE_ERROR(reg))
			return -ENODEV;

		switch (BUS_RESTRICT_CFG(reg)) {
		case 0:
			vmd->busn_start[VMD_BUS_0] = VMD_RESTRICT_0_BUS_START;
			break;
		case 1:
			vmd->busn_start[VMD_BUS_0] = VMD_RESTRICT_1_BUS_START;
			break;
		case 2:
			vmd->busn_start[VMD_BUS_0] = VMD_RESTRICT_2_BUS_START;
			break;
		case 3:
			if (!(features & VMD_FEAT_HAS_BUS1_ROOTBUS)) {
				pci_err(dev, "VMD Bus Restriction detected type %d, but BUS1 root bus is not supported, aborting.\n",
					BUS_RESTRICT_CFG(reg));
				return -ENODEV;
			}

			/*
			 * Per Intel VMD specification (e.g., Arrow Lake-S):
			 * - VMD (on root bus 0) secondary bus start number: 0xE0
			 *	(devices span 0xE2..0xF0 via VMD_BUSRANGE0).
			 * - VMD (on root bus 1) secondary bus start number: 0xE1
			 *	(devices span 0xF1..0xFF via VMD_BUSRANGE1).
			 */
			vmd->busn_start[VMD_BUS_0] = VMD_RESTRICT_2_BUS_START;
			vmd->busn_start[VMD_BUS_1] = VMD_RESTRICT_3_BUS_START;
			vmd->bus1_rootbus = true;
			break;
		default:
			pci_err(dev, "Unknown Bus Offset Setting (%d)\n",
				BUS_RESTRICT_CFG(reg));
			return -ENODEV;
		}
	}

	return 0;
}

static int vmd_get_bus_info_from_bar4(struct vmd_dev *vmd,
				       resource_size_t *offset1,
				       resource_size_t *offset2)
{
	u64 phys1, phys2, bar4_2840;
	void __iomem *bar4;
	u32 base_id;
	u8 base_bus;

	bar4 = pci_ioremap_bar(vmd->dev, 4);
	if (!bar4)
		return -ENOMEM;

	/* Read shadow registers for MEMBAR1 and MEMBAR2 physical addresses */
	phys1 = readq(bar4 + SHADOW_MEMBAR1_28C1);
	phys2 = readq(bar4 + SHADOW_MEMBAR2_28C1);

	/*
	 * Read and set bus start number from Base ID register. 24-bit Base ID
	 * register is part of 64-bit shadowed reqid hide range register and
	 * holds segment, bus, device and function.
	 */
	bar4_2840 = readq(bar4 + BASE_ID_REG_28C1);
	base_id = bar4_2840 & 0xFFFFFF;
	base_bus = base_id >> 8;
	vmd->busn_start[VMD_BUS_0] = base_bus;

	/* Calculate offsets like vmd_get_phys_offsets() does */
	if (phys1)
		*offset1 = vmd->dev->resource[VMD_MEMBAR1].start -
			(phys1 & PCI_BASE_ADDRESS_MEM_MASK);
	if (phys2)
		*offset2 = vmd->dev->resource[VMD_MEMBAR2].start -
			(phys2 & PCI_BASE_ADDRESS_MEM_MASK);

	pci_iounmap(vmd->dev, bar4);

	return 0;
}

static irqreturn_t vmd_irq(int irq, void *data)
{
	struct vmd_irq_list *irqs = data;
	struct vmd_irq *vmdirq;
	int idx;

	idx = srcu_read_lock(&irqs->srcu);
	list_for_each_entry_rcu(vmdirq, &irqs->irq_list, node) {
		/*
		 * MTL016: the MSI may have outrun the initiating device's
		 * posted writes (e.g. its NVMe completion entry).  A read
		 * that completes at the initiator flushes them, so the
		 * demuxed handler observes a coherent completion queue.
		 * The value is discarded; only the ordering matters.
		 */
		if (vmdirq->flush_addr) {
			guard(raw_spinlock)(&irqs->vmd->cfg_lock);
			readw(vmdirq->flush_addr);
		}
		generic_handle_irq(vmdirq->virq);
	}
	srcu_read_unlock(&irqs->srcu, idx);

	return IRQ_HANDLED;
}

static int vmd_alloc_irqs(struct vmd_dev *vmd)
{
	struct pci_dev *dev = vmd->dev;
	int i, err;

	vmd->msix_count = pci_msix_vec_count(dev);
	if (vmd->msix_count < 0)
		return -ENODEV;

	vmd->msix_count = pci_alloc_irq_vectors(dev, vmd->first_vec + 1,
						vmd->msix_count, PCI_IRQ_MSIX);
	if (vmd->msix_count < 0)
		return vmd->msix_count;

	vmd->irqs = devm_kzalloc_objs(&dev->dev, *vmd->irqs, vmd->msix_count);
	if (!vmd->irqs)
		return -ENOMEM;

	for (i = 0; i < vmd->msix_count; i++) {
		err = init_srcu_struct(&vmd->irqs[i].srcu);
		if (err)
			return err;

		INIT_LIST_HEAD(&vmd->irqs[i].irq_list);
		vmd->irqs[i].vmd = vmd;
		vmd->irqs[i].virq = pci_irq_vector(dev, i);
		err = devm_request_irq(&dev->dev, vmd->irqs[i].virq,
				       vmd_irq, IRQF_NO_THREAD,
				       vmd->name, &vmd->irqs[i]);
		if (err)
			return err;
	}

	return 0;
}

static int vmd_prepare_offsets_and_bus(struct vmd_dev *vmd,
					unsigned long features,
					resource_size_t *membar2_offset,
					resource_size_t *offset1,
					resource_size_t *offset2)
{
	int ret;

	/*
	 * Shadow registers may exist in certain VMD device IDs which allow
	 * guests to correctly assign host physical addresses to the root ports
	 * and child devices. These registers will either return the host value
	 * or 0, depending on an enable bit in the VMD device.
	 *
	 * For certain VMD devices (i.e. 0x28C1), BIOS places device info
	 * in BAR4 shadow registers to determine the base bus number and memory
	 * offsets.
	 */
	if (features & VMD_FEAT_USE_BIOS_INFO) {
		*membar2_offset = MEMBAR2_OFFSET_28C1;
		ret = vmd_get_bus_info_from_bar4(vmd, offset1, offset2);
		if (ret)
			return ret;
	} else if (features & VMD_FEAT_HAS_MEMBAR_SHADOW) {
		*membar2_offset = MB2_SHADOW_OFFSET + MB2_SHADOW_SIZE;
		ret = vmd_get_phys_offsets(vmd, true, offset1, offset2);
		if (ret)
			return ret;
	} else if (features & VMD_FEAT_HAS_MEMBAR_SHADOW_VSCAP) {
		ret = vmd_get_phys_offsets(vmd, false, offset1, offset2);
		if (ret)
			return ret;
	}

	/*
	 * Certain VMD devices may have a root port configuration option which
	 * limits the bus range to between 0-127, 128-255, or 224-255.
	 */
	if (features & VMD_FEAT_HAS_BUS_RESTRICTIONS) {
		ret = vmd_get_bus_number_start(vmd, features);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * Since VMD is an aperture to regular PCIe root ports, only allow it to
 * control features that the OS is allowed to control on the physical PCI bus.
 */
static void vmd_copy_host_bridge_flags(struct pci_host_bridge *root_bridge,
				       struct pci_host_bridge *vmd_bridge)
{
	vmd_bridge->native_pcie_hotplug = root_bridge->native_pcie_hotplug;
	vmd_bridge->native_shpc_hotplug = root_bridge->native_shpc_hotplug;
	vmd_bridge->native_aer = root_bridge->native_aer;
	vmd_bridge->native_pme = root_bridge->native_pme;
	vmd_bridge->native_ltr = root_bridge->native_ltr;
	vmd_bridge->native_dpc = root_bridge->native_dpc;
}

/*
 * Enable ASPM and LTR settings on devices that aren't configured by BIOS.
 */
static int vmd_pm_enable_quirk(struct pci_dev *pdev, void *userdata)
{
	unsigned long features = *(unsigned long *)userdata;
	u16 ltr = VMD_BIOS_PM_QUIRK_LTR;
	u32 ltr_reg;
	int pos;

	if (!(features & VMD_FEAT_BIOS_PM_QUIRK))
		return 0;

	pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_LTR);
	if (!pos)
		goto out_state_change;

	/*
	 * Skip if the max snoop LTR is non-zero, indicating BIOS has set it
	 * so the LTR quirk is not needed.
	 */
	pci_read_config_dword(pdev, pos + PCI_LTR_MAX_SNOOP_LAT, &ltr_reg);
	if (!!(ltr_reg & (PCI_LTR_VALUE_MASK | PCI_LTR_SCALE_MASK)))
		goto out_state_change;

	/*
	 * Set the default values to the maximum required by the platform to
	 * allow the deepest power management savings. Write as a DWORD where
	 * the lower word is the max snoop latency and the upper word is the
	 * max non-snoop latency.
	 */
	ltr_reg = (ltr << 16) | ltr;
	pci_write_config_dword(pdev, pos + PCI_LTR_MAX_SNOOP_LAT, ltr_reg);
	pci_info(pdev, "VMD: Default LTR value set by driver\n");

out_state_change:
	/*
	 * Ensure devices are in D0 before enabling PCI-PM L1 PM Substates, per
	 * PCIe r6.0, sec 5.5.4.
	 */
	pci_set_power_state_locked(pdev, PCI_D0);
	pci_enable_link_state_locked(pdev, PCIE_LINK_STATE_ALL);
	return 0;
}

static int vmd_configure_cfgbar(struct vmd_dev *vmd)
{
	struct resource *res = &vmd->dev->resource[VMD_CFGBAR];
	resource_size_t busn_end;

	/* Do not let resource[VMD_RES_CFGBAR] end go out of bound.*/
	busn_end = vmd->busn_start[VMD_BUS_0] + (resource_size(res) >> 20) - 1;
	busn_end = min_t(resource_size_t, busn_end, 0xff);
	vmd->resources[VMD_RES_CFGBAR] = (struct resource) {
		.name  = "VMD CFGBAR",
		.start = vmd->busn_start[VMD_BUS_0],
		.end   = busn_end,
		.flags = IORESOURCE_BUS | IORESOURCE_PCI_FIXED,
	};

	if (vmd->bus1_rootbus) {
		int ret;
		u16 bus0_range = 0;
		u16 bus1_range = 0;
		u8 bus0_start, bus0_end;
		u8 bus1_start, bus1_end;

		ret = pci_read_config_word(vmd->dev, VMD_BUSRANGE0, &bus0_range);
		if (ret) {
			pci_err(vmd->dev, "Failed to read VMD_BUSRANGE0: %d\n", ret);
			return -EIO;
		}

		ret = pci_read_config_word(vmd->dev, VMD_BUSRANGE1, &bus1_range);
		if (ret) {
			pci_err(vmd->dev, "Failed to read VMD_BUSRANGE1: %d\n", ret);
			return -EIO;
		}

		bus0_start = VMD_BUS_START(bus0_range);
		bus0_end = VMD_BUS_END(bus0_range);
		bus1_start = VMD_BUS_START(bus1_range);
		bus1_end = VMD_BUS_END(bus1_range);

		if (bus0_start > bus0_end || bus1_start > bus1_end) {
			pci_err(vmd->dev,
				"Invalid bus range(s): BUS0 [%02x-%02x], BUS1 [%02x-%02x]\n",
				bus0_start, bus0_end, bus1_start, bus1_end);
			return -EINVAL;
		}

		if (!(bus0_end < bus1_start || bus1_end < bus0_start)) {
			pci_err(vmd->dev,
				"Overlapping bus ranges: BUS0 [%02x-%02x], BUS1 [%02x-%02x]\n",
				bus0_start, bus0_end, bus1_start, bus1_end);
			return -EINVAL;
		}

		/*
		 * Resize BUS0 CFGBAR range to make space for BUS1
		 * owned devices by adjusting range end with value stored in
		 * VMD_BUSRANGE0 register.
		 */
		vmd->resources[VMD_RES_CFGBAR].start = bus0_start;
		vmd->resources[VMD_RES_CFGBAR].end = bus0_end;

		vmd->resources[VMD_RES_BUS1_CFGBAR] = (struct resource){
			.name = "VMD CFGBAR BUS1",
			.start = bus1_start,
			.end = bus1_end,
			.flags = IORESOURCE_BUS | IORESOURCE_PCI_FIXED,
		};
	}

	return 0;
}

/*
 * vmd_configure_membar - Configure VMD MemBAR register, which points
 * to MMIO address assigned by the OS or BIOS.
 * @vmd: the VMD device
 * @resource_number: resource buffer number to be filled in
 * @membar_number: number of the MemBAR
 * @start_offset: 4K aligned offset applied to start of VMD’s MEMBAR MMIO space
 * @end_offset: 4K aligned offset applied to end of VMD’s MEMBAR MMIO space
 *
 * Function fills resource buffer inside the VMD structure.
 *
 * Return: 0 on success, -ENOMEM on allocation failure.
 */
static int vmd_configure_membar(struct vmd_dev *vmd, u8 resource_number,
				u8 membar_number, resource_size_t start_offset,
				resource_size_t end_offset)
{
	char *name;
	u32 upper_bits;
	unsigned long flags;

	struct resource *res = &vmd->dev->resource[membar_number];

	upper_bits = upper_32_bits(res->end);
	flags = res->flags & ~IORESOURCE_SIZEALIGN;
	if (!upper_bits)
		flags &= ~IORESOURCE_MEM_64;

	name = devm_kasprintf(&vmd->dev->dev, GFP_KERNEL, "VMD MEMBAR%d %s",
			      membar_number / 2,
			      resource_number > VMD_RES_MBAR_2 ? "BUS1" : "");
	if (!name)
		return -ENOMEM;

	vmd->resources[resource_number] = (struct resource){
		.name = name,
		.start = res->start + start_offset,
		.end = res->end - end_offset,
		.flags = flags,
		.parent = res,
	};

	return 0;
}

static int vmd_configure_membar1_membar2(struct vmd_dev *vmd,
					 resource_size_t mbar2_ofs)
{
	int ret;

	if (vmd->bus1_rootbus) {
		u32 reg = 0;
		u32 bus1_mbar1_ofs = 0;
		u64 bus1_mbar2_ofs = 0;
		resource_size_t mbar1_sz, mbar2_sz;

		mbar1_sz = resource_size(&vmd->dev->resource[VMD_MEMBAR1]);
		mbar2_sz = resource_size(&vmd->dev->resource[VMD_MEMBAR2]);

		ret = pci_read_config_dword(vmd->dev, VMD_MEMBAR1_OFFSET,
					    &bus1_mbar1_ofs);
		if (ret)
			return -EIO;

		ret = pci_read_config_dword(vmd->dev, VMD_MEMBAR2_OFFSET1, &reg);
		if (ret)
			return -EIO;
		bus1_mbar2_ofs = reg;

		ret = pci_read_config_dword(vmd->dev, VMD_MEMBAR2_OFFSET2, &reg);
		if (ret)
			return -EIO;
		bus1_mbar2_ofs |= (u64)reg << 32;

		if (!bus1_mbar1_ofs || bus1_mbar1_ofs >= mbar1_sz) {
			pci_err(vmd->dev, "Invalid MEMBAR1 offset %#llx (BAR size %#llx)\n",
				(unsigned long long)bus1_mbar1_ofs,
				(unsigned long long)mbar1_sz);
			return -EINVAL;
		}

		if (!bus1_mbar2_ofs || mbar2_ofs >= mbar2_sz ||
		    bus1_mbar2_ofs <= mbar2_ofs || bus1_mbar2_ofs >= mbar2_sz) {
			pci_err(vmd->dev,
				"Invalid MEMBAR2 split %#llx (shadow offset %#llx, BAR size %#llx)\n",
				(unsigned long long)bus1_mbar2_ofs,
				(unsigned long long)mbar2_ofs,
				(unsigned long long)mbar2_sz);
			return -EINVAL;
		}

		/*
		 * Resize BUS MEMBAR1 and MEMBAR2 ranges to make space
		 * for BUS1 owned devices by adjusting range end with values
		 * stored in VMD_MEMBAR1_OFFSET and VMD_MEMBAR2_OFFSET registers
		 */
		ret = vmd_configure_membar(vmd, VMD_RES_MBAR_1, VMD_MEMBAR1, 0,
					   mbar1_sz - bus1_mbar1_ofs);
		if (ret)
			return ret;

		ret = vmd_configure_membar(vmd, VMD_RES_MBAR_2, VMD_MEMBAR2,
					   mbar2_ofs, mbar2_sz - bus1_mbar2_ofs);
		if (ret)
			return ret;

		ret = vmd_configure_membar(vmd, VMD_RES_BUS1_MBAR_1, VMD_MEMBAR1,
					   bus1_mbar1_ofs, 0);
		if (ret)
			return ret;

		ret = vmd_configure_membar(vmd, VMD_RES_BUS1_MBAR_2, VMD_MEMBAR2,
					   bus1_mbar2_ofs, 0);
		if (ret)
			return ret;
	} else {
		ret = vmd_configure_membar(vmd, VMD_RES_MBAR_1, VMD_MEMBAR1, 0, 0);
		if (ret)
			return ret;

		ret = vmd_configure_membar(vmd, VMD_RES_MBAR_2, VMD_MEMBAR2,
					   mbar2_ofs, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static int vmd_create_bus(struct vmd_dev *vmd, enum vmd_rootbus bus_number,
			  struct pci_sysdata *sd, resource_size_t *offset,
			  bool vmd_in_guest, u8 primary)
{
	u8 cfgbar = bus_number * 3;
	u8 membar1 = cfgbar + 1;
	u8 membar2 = cfgbar + 2;
	struct pci_bus *vmd_bus;
	LIST_HEAD(resources);

	pci_add_resource(&resources, &vmd->resources[cfgbar]);
	pci_add_resource_offset(&resources, &vmd->resources[membar1],
				offset[0]);
	pci_add_resource_offset(&resources, &vmd->resources[membar2],
				offset[1]);

	vmd_bus = pci_create_root_bus(&vmd->dev->dev,
				      vmd->busn_start[bus_number], &vmd_ops, sd,
				      &resources);

	if (!vmd_bus) {
		pci_free_resource_list(&resources);
		return -ENODEV;
	}

	/*
	 * pci_create_root_bus() does not initialise bus->primary.
	 * Set it here before any scanning so bridge traversal logic
	 * sees the correct upstream bus number from the start.
	 */
	vmd_bus->primary = primary;

	/*
	 * Don't copy _OSC control flags from root bridge if running in a VM, as
	 * they don't reflect the physical root bridge capabilities.
	 */
	if (!vmd_in_guest)
		vmd_copy_host_bridge_flags(pci_find_host_bridge(vmd->dev->bus),
					   to_pci_host_bridge(vmd_bus->bridge));
	if (vmd->irq_domain)
		dev_set_msi_domain(&vmd_bus->dev, vmd->irq_domain);
	else
		dev_set_msi_domain(&vmd_bus->dev,
				   dev_get_msi_domain(&vmd->dev->dev));

	vmd->bus[bus_number] = vmd_bus;

	return 0;
}

static void vmd_bus_enumeration(struct pci_bus *bus, unsigned long features)
{
	struct pci_bus *child;
	struct pci_dev *dev;
	int ret;

	vmd_acpi_begin();

	pci_scan_child_bus(bus);

	/*
	 * vmd_domain_reset() walks the full VMD CFGBAR aperture, so a single
	 * invocation from BUS0 resets bridge windows for the whole VMD domain,
	 * including BUS1. Running it again during BUS1 enumeration would
	 * re-clobber windows already assigned for BUS0.
	 */
	if (bus->primary == VMD_PRIMARY_BUS0)
		vmd_domain_reset(vmd_from_bus(bus));

	/*
	 * When Intel VMD is enabled, the OS does not discover the Root Ports
	 * owned by Intel VMD within the MMCFG space. pci_reset_bus() applies
	 * a reset to the parent of the PCI device supplied as argument. This
	 * is why we pass a child device, so the reset can be triggered at
	 * the Intel bridge level and propagated to all the children in the
	 * hierarchy.
	 */
	list_for_each_entry(child, &bus->children, node) {
		if (!list_empty(&child->devices)) {
			dev = list_first_entry(&child->devices, struct pci_dev,
					       bus_list);
			ret = pci_reset_bus(dev);
			if (ret)
				pci_warn(dev, "can't reset device: %d\n", ret);

			break;
		}
	}

	pci_assign_unassigned_bus_resources(bus);

	pci_walk_bus(bus, vmd_pm_enable_quirk, &features);

	/*
	 * VMD root buses are virtual and don't return true on pci_is_pcie()
	 * and will fail pcie_bus_configure_settings() early. It can instead be
	 * run on each of the real root ports.
	 */
	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);

	pci_bus_add_devices(bus);

	vmd_acpi_end();
}

/* Drop BUS1 configuration and continue with BUS0 only. */
static void vmd_disable_bus1(struct vmd_dev *vmd)
{
	vmd->resources[VMD_RES_BUS1_CFGBAR] = (struct resource){};
	vmd->resources[VMD_RES_BUS1_MBAR_1] = (struct resource){};
	vmd->resources[VMD_RES_BUS1_MBAR_2] = (struct resource){};
	vmd->sysdata[VMD_BUS_1].domain = PCI_DOMAIN_NR_NOT_SET;
	vmd->bus[VMD_BUS_1] = NULL;
}

static int vmd_enable_domain(struct vmd_dev *vmd, unsigned long features)
{
	struct pci_sysdata *sd = &vmd->sysdata[VMD_BUS_0];
	resource_size_t offset[2] = {0};
	resource_size_t membar2_offset = 0x2000;
	bool vmd_in_guest;
	int ret;

	ret = vmd_prepare_offsets_and_bus(vmd, features, &membar2_offset,
					  &offset[0], &offset[1]);
	if (ret)
		return ret;

	ret = vmd_configure_cfgbar(vmd);
	if (ret)
		return ret;

	/*
	 * If the window is below 4GB, clear IORESOURCE_MEM_64 so we can
	 * put 32-bit resources in the window.
	 *
	 * There's no hardware reason why a 64-bit window *couldn't*
	 * contain a 32-bit resource, but pbus_size_mem() computes the
	 * bridge window size assuming a 64-bit window will contain no
	 * 32-bit resources.  __pci_assign_resource() enforces that
	 * artificial restriction to make sure everything will fit.
	 *
	 * The only way we could use a 64-bit non-prefetchable MEMBAR is
	 * if its address is <4GB so that we can convert it to a 32-bit
	 * resource. To be visible to the host OS, all VMD endpoints must
	 * be initially configured by platform BIOS, which includes setting
	 * up these resources. We can assume the device is configured
	 * according to the platform needs.
	 */
	ret = vmd_configure_membar1_membar2(vmd, membar2_offset);
	if (ret)
		return ret;

	/* Non-zero offset means guest/direct assign view. */
	vmd_in_guest = offset[0] || offset[1];

	/*
	 * Currently MSI remapping must be enabled in guest passthrough mode
	 * due to some missing interrupt remapping plumbing. This is probably
	 * acceptable because the guest is usually CPU-limited and MSI
	 * remapping doesn't become a performance bottleneck.
	 */
	if (!(features & VMD_FEAT_CAN_BYPASS_MSI_REMAP) || vmd_in_guest) {
		ret = vmd_alloc_irqs(vmd);
		if (ret)
			return ret;

		vmd_set_msi_remapping(vmd, true);

		ret = vmd_create_irq_domain(vmd);
		if (ret)
			return ret;
	} else {
		vmd_set_msi_remapping(vmd, false);
	}

	sd->vmd_dev = vmd->dev;

	/*
	 * Emulated domains start at 0x10000 to not clash with ACPI _SEG
	 * domains.  Per ACPI r6.0, sec 6.5.6, _SEG returns an integer, of
	 * which the lower 16 bits are the PCI Segment Group (domain) number.
	 * Other bits are currently reserved.
	 */
	sd->domain = pci_bus_find_emul_domain_nr(0, 0x10000, INT_MAX);
	if (sd->domain < 0)
		return sd->domain;

	sd->node = pcibus_to_node(vmd->dev->bus);

	if (vmd->bus1_rootbus) {
		vmd->sysdata[VMD_BUS_1].vmd_dev = vmd->dev;
		vmd->sysdata[VMD_BUS_1].node = pcibus_to_node(vmd->dev->bus);
		vmd->sysdata[VMD_BUS_1].domain = pci_bus_find_emul_domain_nr(0, 0x10000, INT_MAX);
		if (vmd->sysdata[VMD_BUS_1].domain < 0) {
			pci_warn(vmd->dev,
				 "Can't allocate BUS1 domain: %d, continuing with BUS0 only\n",
				 vmd->sysdata[VMD_BUS_1].domain);
			vmd_disable_bus1(vmd);
		}
	}

	ret = vmd_attach_resources(vmd);
	if (ret) {
		pci_bus_release_emul_domain_nr(sd->domain);
		if (vmd->bus1_rootbus && vmd->sysdata[VMD_BUS_1].domain >= 0)
			pci_bus_release_emul_domain_nr(vmd->sysdata[VMD_BUS_1].domain);

		vmd_remove_irq_domain(vmd);
		return ret;
	}

	ret = vmd_create_bus(vmd, VMD_BUS_0, sd, offset, vmd_in_guest,
			     VMD_PRIMARY_BUS0);
	if (ret) {
		pci_err(vmd->dev, "Can't create bus: %d\n", ret);
		pci_bus_release_emul_domain_nr(sd->domain);
		if (vmd->bus1_rootbus && vmd->sysdata[VMD_BUS_1].domain >= 0)
			pci_bus_release_emul_domain_nr(vmd->sysdata[VMD_BUS_1].domain);

		vmd_remove_irq_domain(vmd);
		vmd_detach_resources(vmd);
		return ret;
	}

	WARN(sysfs_create_link(&vmd->dev->dev.kobj,
			       &vmd->bus[VMD_BUS_0]->dev.kobj, "domain"),
	     "Can't create symlink to domain\n");

	if (vmd->bus1_rootbus && vmd->sysdata[VMD_BUS_1].domain >= 0) {
		ret = vmd_create_bus(vmd, VMD_BUS_1, &vmd->sysdata[VMD_BUS_1], offset,
				     vmd_in_guest, VMD_PRIMARY_BUS1);
		if (ret) {
			pci_warn(vmd->dev,
				 "Can't create BUS1: %d, continuing with BUS0 only\n",
				 ret);

			release_resource(&vmd->resources[VMD_RES_BUS1_MBAR_2]);
			release_resource(&vmd->resources[VMD_RES_BUS1_MBAR_1]);
			pci_bus_release_emul_domain_nr(vmd->sysdata[VMD_BUS_1].domain);
			vmd_disable_bus1(vmd);
		}

		if (vmd->bus1_rootbus && vmd->sysdata[VMD_BUS_1].domain >= 0) {
			WARN(sysfs_create_link(&vmd->dev->dev.kobj,
					       &vmd->bus[VMD_BUS_1]->dev.kobj,
					       "domain1"),
			     "Can't create symlink to domain1\n");
		}
	}

	vmd_bus_enumeration(vmd->bus[VMD_BUS_0], features);

	if (vmd->bus1_rootbus && vmd->sysdata[VMD_BUS_1].domain >= 0)
		vmd_bus_enumeration(vmd->bus[VMD_BUS_1], features);

	return 0;
}

static int vmd_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	unsigned long features = (unsigned long) id->driver_data;
	struct vmd_dev *vmd;
	int err;

	if (xen_domain()) {
		/*
		 * Xen doesn't have knowledge about devices in the VMD bus
		 * because the config space of devices behind the VMD bridge is
		 * not known to Xen, and hence Xen cannot discover or configure
		 * them in any way.
		 *
		 * Bypass of MSI remapping won't work in that case as direct
		 * write by Linux to the MSI entries won't result in functional
		 * interrupts, as Xen is the entity that manages the host
		 * interrupt controller and must configure interrupts.  However
		 * multiplexing of interrupts by the VMD bridge will work under
		 * Xen, so force the usage of that mode which must always be
		 * supported by VMD bridges.
		 */
		features &= ~VMD_FEAT_CAN_BYPASS_MSI_REMAP;
	}

	if (resource_size(&dev->resource[VMD_CFGBAR]) < (1 << 20))
		return -ENOMEM;

	vmd = devm_kzalloc_obj(&dev->dev, *vmd);
	if (!vmd)
		return -ENOMEM;

	vmd->dev = dev;
	vmd->sysdata[VMD_BUS_0].domain = PCI_DOMAIN_NR_NOT_SET;
	vmd->sysdata[VMD_BUS_1].domain = PCI_DOMAIN_NR_NOT_SET;
	vmd->features = features;
	vmd->instance = ida_alloc(&vmd_instance_ida, GFP_KERNEL);
	if (vmd->instance < 0)
		return vmd->instance;

	vmd->name = devm_kasprintf(&dev->dev, GFP_KERNEL, "vmd%d",
				   vmd->instance);
	if (!vmd->name) {
		err = -ENOMEM;
		goto out_release_instance;
	}

	err = pcim_enable_device(dev);
	if (err < 0)
		goto out_release_instance;

	vmd->cfgbar = pcim_iomap(dev, VMD_CFGBAR, 0);
	if (!vmd->cfgbar) {
		err = -ENOMEM;
		goto out_release_instance;
	}

	pci_set_master(dev);
	if (dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(64)) &&
	    dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(32))) {
		err = -ENODEV;
		goto out_release_instance;
	}

	if (features & VMD_FEAT_OFFSET_FIRST_VECTOR)
		vmd->first_vec = 1;

	raw_spin_lock_init(&vmd->cfg_lock);
	pci_set_drvdata(dev, vmd);
	err = vmd_enable_domain(vmd, features);
	if (err)
		goto out_release_instance;

	if (vmd->bus1_rootbus && vmd->sysdata[VMD_BUS_1].domain >= 0)
		dev_info(&vmd->dev->dev, "Bound to PCI domains %04x and %04x\n",
			 vmd->sysdata[VMD_BUS_0].domain,
			 vmd->sysdata[VMD_BUS_1].domain);
	else
		dev_info(&vmd->dev->dev, "Bound to PCI domain %04x\n",
			 vmd->sysdata[VMD_BUS_0].domain);

	return 0;

 out_release_instance:
	ida_free(&vmd_instance_ida, vmd->instance);
	return err;
}

static void vmd_cleanup_srcu(struct vmd_dev *vmd)
{
	int i;

	for (i = 0; i < vmd->msix_count; i++)
		cleanup_srcu_struct(&vmd->irqs[i].srcu);
}

static void vmd_remove(struct pci_dev *dev)
{
	struct vmd_dev *vmd = pci_get_drvdata(dev);

	pci_stop_root_bus(vmd->bus[VMD_BUS_0]);
	sysfs_remove_link(&vmd->dev->dev.kobj, "domain");
	pci_remove_root_bus(vmd->bus[VMD_BUS_0]);

	if (vmd->bus1_rootbus && vmd->bus[VMD_BUS_1]) {
		pci_stop_root_bus(vmd->bus[VMD_BUS_1]);
		sysfs_remove_link(&vmd->dev->dev.kobj, "domain1");
		pci_remove_root_bus(vmd->bus[VMD_BUS_1]);
		pci_bus_release_emul_domain_nr(vmd->sysdata[VMD_BUS_1].domain);
	}

	vmd_cleanup_srcu(vmd);
	vmd_detach_resources(vmd);
	vmd_remove_irq_domain(vmd);
	ida_free(&vmd_instance_ida, vmd->instance);
	pci_bus_release_emul_domain_nr(vmd->sysdata[VMD_BUS_0].domain);
}

static void vmd_shutdown(struct pci_dev *dev)
{
	struct vmd_dev *vmd = pci_get_drvdata(dev);

	vmd_remove_irq_domain(vmd);
}

#ifdef CONFIG_PM_SLEEP
static int vmd_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct vmd_dev *vmd = pci_get_drvdata(pdev);
	int i;

	for (i = 0; i < vmd->msix_count; i++)
		devm_free_irq(dev, vmd->irqs[i].virq, &vmd->irqs[i]);

	return 0;
}

static int vmd_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct vmd_dev *vmd = pci_get_drvdata(pdev);
	int err, i;

	vmd_set_msi_remapping(vmd, !!vmd->irq_domain);

	for (i = 0; i < vmd->msix_count; i++) {
		err = devm_request_irq(dev, vmd->irqs[i].virq,
				       vmd_irq, IRQF_NO_THREAD,
				       vmd->name, &vmd->irqs[i]);
		if (err)
			return err;
	}

	return 0;
}
#endif
static SIMPLE_DEV_PM_OPS(vmd_dev_pm_ops, vmd_suspend, vmd_resume);

static const struct pci_device_id vmd_ids[] = {
	{PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_VMD_201D),
		.driver_data = VMD_FEAT_HAS_MEMBAR_SHADOW_VSCAP,},
	{PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_VMD_28C0),
		.driver_data = VMD_FEAT_HAS_MEMBAR_SHADOW |
				VMD_FEAT_HAS_BUS_RESTRICTIONS |
				VMD_FEAT_CAN_BYPASS_MSI_REMAP,},
	{PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_VMD_28C1),
		.driver_data = VMD_FEAT_HAS_MEMBAR_SHADOW |
				VMD_FEAT_CAN_BYPASS_MSI_REMAP |
				VMD_FEAT_USE_BIOS_INFO,},
	{PCI_VDEVICE(INTEL, 0x467f),
		.driver_data = VMD_FEATS_CLIENT,},
	{PCI_VDEVICE(INTEL, 0x4c3d),
		.driver_data = VMD_FEATS_CLIENT,},
	{PCI_VDEVICE(INTEL, 0xa77f),
		.driver_data = VMD_FEATS_CLIENT,},
	{PCI_VDEVICE(INTEL, 0x7d0b),
		.driver_data = VMD_FEATS_CLIENT | VMD_FEAT_INTERRUPT_QUIRK,},
	{PCI_VDEVICE(INTEL, 0xad0b),
		.driver_data = VMD_FEATS_CLIENT | VMD_FEAT_INTERRUPT_QUIRK,},
	{PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_VMD_9A0B),
		.driver_data = VMD_FEATS_CLIENT,},
	{PCI_VDEVICE(INTEL, 0xb60b),
                .driver_data = VMD_FEATS_CLIENT,},
	{PCI_VDEVICE(INTEL, 0xb06f),
                .driver_data = VMD_FEATS_CLIENT,},
	{PCI_VDEVICE(INTEL, 0xb07f),
                .driver_data = VMD_FEATS_CLIENT,},
	{PCI_VDEVICE(INTEL, 0xd70b),
		.driver_data = VMD_FEATS_CLIENT,},
	{PCI_VDEVICE(INTEL, 0xd73b),
		.driver_data = VMD_FEATS_CLIENT,},
	{0,}
};
MODULE_DEVICE_TABLE(pci, vmd_ids);

static struct pci_driver vmd_drv = {
	.name		= "vmd",
	.id_table	= vmd_ids,
	.probe		= vmd_probe,
	.remove		= vmd_remove,
	.shutdown	= vmd_shutdown,
	.driver		= {
		.pm	= &vmd_dev_pm_ops,
	},
};
module_pci_driver(vmd_drv);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Volume Management Device driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.7");
