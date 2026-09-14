// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 Nebula Matrix Limited.
 */

#include <linux/device.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/bits.h>
#include "nbl_include/nbl_include.h"
#include "nbl_core.h"

static int nbl_probe(struct pci_dev *pdev,
		     const struct pci_device_id *id)
{
	return -ENODEV;
}

static void nbl_remove(struct pci_dev *pdev)
{
}

/*
 * PCI Device IDs for Leonis/NBL Network Controllers
 *
 * Vendor ID: 0x1F0F
 * SNIC v3r1 product Device IDs range: 0x3403-0x3412
 */
#define NBL_VENDOR_ID				0x1F0F

#define NBL_DEVICE_ID_M18110			0x3403
#define NBL_DEVICE_ID_M18110_LX			0x3404
#define NBL_DEVICE_ID_M18110_BASE_T		0x3405
#define NBL_DEVICE_ID_M18110_LX_BASE_T		0x3406
#define NBL_DEVICE_ID_M18110_OCP		0x3407
#define NBL_DEVICE_ID_M18110_LX_OCP		0x3408
#define NBL_DEVICE_ID_M18110_BASE_T_OCP		0x3409
#define NBL_DEVICE_ID_M18110_LX_BASE_T_OCP	0x340a
#define NBL_DEVICE_ID_M18000			0x340b
#define NBL_DEVICE_ID_M18000_LX			0x340c
#define NBL_DEVICE_ID_M18000_BASE_T		0x340d
#define NBL_DEVICE_ID_M18000_LX_BASE_T		0x340e
#define NBL_DEVICE_ID_M18000_OCP		0x340f
#define NBL_DEVICE_ID_M18000_LX_OCP		0x3410
#define NBL_DEVICE_ID_M18000_BASE_T_OCP		0x3411
#define NBL_DEVICE_ID_M18000_LX_BASE_T_OCP	0x3412

/* All below IDs belong to Leonis ASIC family, different form-factor variants,
 * share the same hardware initialization flow without differentiated ops.
 */
static const struct pci_device_id nbl_id_table[] = {
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18110),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18110_LX),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18110_BASE_T),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18110_LX_BASE_T),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18110_OCP),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18110_LX_OCP),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18110_BASE_T_OCP),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18110_LX_BASE_T_OCP),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18000),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18000_LX),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18000_BASE_T),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18000_LX_BASE_T),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18000_OCP),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18000_LX_OCP),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18000_BASE_T_OCP),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	{ PCI_DEVICE(NBL_VENDOR_ID, NBL_DEVICE_ID_M18000_LX_BASE_T_OCP),
	  .driver_data = BIT(NBL_CAP_HAS_NET_BIT) },
	/* required as sentinel */
	{ }
};
MODULE_DEVICE_TABLE(pci, nbl_id_table);

static struct pci_driver nbl_driver = {
	.name = NBL_DRIVER_NAME,
	.id_table = nbl_id_table,
	.probe = nbl_probe,
	.remove = nbl_remove,
};

module_pci_driver(nbl_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nebula Matrix Network Driver");
MODULE_AUTHOR("Illusion Wang <illusion.wang@nebula-matrix.com>");
