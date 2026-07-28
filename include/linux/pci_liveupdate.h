/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCI Live Update support (Public/Driver API)
 *
 * Copyright (c) 2026, Google LLC.
 * David Matlack <dmatlack@google.com>
 */
#ifndef LINUX_PCI_LIVEUPDATE_H
#define LINUX_PCI_LIVEUPDATE_H

#include <linux/liveupdate.h>
#include <linux/types.h>

struct pci_dev;

#ifdef CONFIG_PCI_LIVEUPDATE
int pci_liveupdate_register_flb(struct liveupdate_file_handler *fh);
void pci_liveupdate_unregister_flb(struct liveupdate_file_handler *fh);
#else
static inline int pci_liveupdate_register_flb(struct liveupdate_file_handler *fh)
{
	return -EOPNOTSUPP;
}

static inline void pci_liveupdate_unregister_flb(struct liveupdate_file_handler *fh)
{
}
#endif

#endif /* LINUX_PCI_LIVEUPDATE_H */
