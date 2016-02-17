/*
 * mwgeneric_pci_of.h
 *
 *  Created on: Jan 6, 2016
 *      Author: mfornero
 */

#ifndef __PCIE_AXI_BRIDGE_H__
#define __PCIE_AXI_BRIDGE_H__

#include <linux/pci.h>
#include <linux/irqdomain.h>
#include <linux/fpga/fpga-overlay.h>

struct fdt_blob_info {
	const uint8_t		*start;
	size_t				size;
};

struct of_pcie_axi_bridge {
	struct pci_dev				*pdev;
	int							pci_overlay_id;
	struct device_node			*of_self;
	int							of_added_self;
	struct irq_domain 			*irq_domain;
	struct irq_domain			*parent_irq_domain;
	int							nvec;
	int							irq_base;
	struct fpga_overlay_dev		*overlay_dev;
	const char					*compat;
	void 						*priv;
	struct list_head			list;
};

extern int pab_update_overlay_fragment (struct fpga_overlay_dev *overlay_dev, struct device_node *fragment);

extern int pcie_axi_bridge_register(struct pci_dev *pdev, const char *compat, const struct firmware *fdt_self, struct fpga_overlay_ops *overlay_ops, void *priv);
extern void pcie_axi_bridge_unregister(struct pci_dev *pdev);

#endif /* __PCIE_AXI_BRIDGE_H__ */
