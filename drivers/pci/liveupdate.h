/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCI Live Update support (core API)
 *
 * Copyright (c) 2026, Google LLC.
 * David Matlack <dmatlack@google.com>
 */
#ifndef DRIVERS_PCI_LIVEUPDATE_H
#define DRIVERS_PCI_LIVEUPDATE_H

#include <linux/pci.h>

#ifdef CONFIG_PCI_LIVEUPDATE
void pci_liveupdate_setup_device(struct pci_dev *dev);
void pci_liveupdate_cleanup_device(struct pci_dev *dev);
void pci_liveupdate_freeze(struct pci_dev *dev);
bool pci_liveupdate_preserve_bus_numbers(struct pci_bus *bus,
					 struct pci_dev *dev);
void pci_liveupdate_scan_bridge_end(struct pci_dev *dev);
void pci_liveupdate_cache_adopted_acs_controls(struct pci_dev *dev);
int pci_liveupdate_enable_adopted_acs_controls(struct pci_dev *dev);
int pci_liveupdate_adopt_ari(struct pci_dev *dev);
bool pci_liveupdate_is_outgoing(struct pci_dev *dev);
#else
static inline void pci_liveupdate_setup_device(struct pci_dev *dev)
{
}

static inline void pci_liveupdate_cleanup_device(struct pci_dev *dev)
{
}

static inline void pci_liveupdate_freeze(struct pci_dev *dev)
{
}
static inline bool pci_liveupdate_preserve_bus_numbers(struct pci_bus *bus,
						       struct pci_dev *dev)
{
	return false;
}

static inline void pci_liveupdate_scan_bridge_end(struct pci_dev *dev)
{
}

static inline void pci_liveupdate_cache_adopted_acs_controls(struct pci_dev *dev)
{
}

static inline int pci_liveupdate_enable_adopted_acs_controls(struct pci_dev *dev)
{
	return -EINVAL;
}

static inline int pci_liveupdate_adopt_ari(struct pci_dev *dev)
{
	return -EINVAL;
}

static inline bool pci_liveupdate_is_outgoing(struct pci_dev *dev)
{
	return false;
}
#endif

#endif /* DRIVERS_PCI_LIVEUPDATE_H */
