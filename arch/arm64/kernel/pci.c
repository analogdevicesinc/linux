// SPDX-License-Identifier: GPL-2.0-only
/*
 * Code borrowed from powerpc/kernel/pci-common.c
 *
 * Copyright (C) 2003 Anton Blanchard <anton@au.ibm.com>, IBM
 * Copyright (C) 2014 ARM Ltd.
 */

#include <linux/pci.h>
#include <linux/of_irq.h>

#include "../../../drivers/pci/pcie/portdrv.h"

/*
 * Check device tree if the service interrupts are there
 */
int pcibios_check_service_irqs(struct pci_dev *dev, int *irqs, int mask)
{
	int ret, count = 0;
	struct device_node *np = NULL;

	if (dev->bus->dev.of_node)
		np = dev->bus->dev.of_node;

	if (np == NULL)
		return 0;

	if (!IS_ENABLED(CONFIG_OF_IRQ))
		return 0;

	/* If root port doesn't support MSI/MSI-X/INTx in RC mode,
	 * request irq for aer
	 */
	if (mask & PCIE_PORT_SERVICE_AER) {
		ret = of_irq_get_byname(np, "aer");
		if (ret > 0) {
			irqs[PCIE_PORT_SERVICE_AER_SHIFT] = ret;
			count++;
		}
	}

	if (mask & PCIE_PORT_SERVICE_PME) {
		ret = of_irq_get_byname(np, "pme");
		if (ret > 0) {
			irqs[PCIE_PORT_SERVICE_PME_SHIFT] = ret;
			count++;
		}
	}

	/* TODO: add more service interrupts if there it is in the device tree*/

	return count;
}

/*
 * raw_pci_read/write - Platform-specific PCI config space access.
 */
int raw_pci_read(unsigned int domain, unsigned int bus,
		  unsigned int devfn, int reg, int len, u32 *val)
{
	struct pci_bus *b = pci_find_bus(domain, bus);

	if (!b)
		return PCIBIOS_DEVICE_NOT_FOUND;
	return b->ops->read(b, devfn, reg, len, val);
}

int raw_pci_write(unsigned int domain, unsigned int bus,
		unsigned int devfn, int reg, int len, u32 val)
{
	struct pci_bus *b = pci_find_bus(domain, bus);

	if (!b)
		return PCIBIOS_DEVICE_NOT_FOUND;
	return b->ops->write(b, devfn, reg, len, val);
}

#ifdef CONFIG_NUMA

int pcibus_to_node(struct pci_bus *bus)
{
	return dev_to_node(&bus->dev);
}
EXPORT_SYMBOL(pcibus_to_node);

#endif
