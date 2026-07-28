/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2026, Google LLC.
 * David Matlack <dmatlack@google.com>
 */

#ifndef _LINUX_KHO_ABI_PCI_H
#define _LINUX_KHO_ABI_PCI_H

#include <linux/bug.h>
#include <linux/compiler.h>
#include <linux/types.h>

/**
 * DOC: PCI File-Lifecycle Bound (FLB) Live Update ABI
 *
 * This header defines the ABI for preserving core PCI state across kexec using
 * Live Update File-Lifecycle Bound (FLB) data.
 *
 * This interface is a contract. Any modification to any of the serialization
 * structs defined here constitutes a breaking change. Such changes require
 * incrementing the version number in the PCI_LUO_FLB_VERSION number.
 */

#define PCI_LUO_FLB_COMPATIBLE "pci"
#define PCI_LUO_FLB_VERSION 1

/**
 * struct pci_dev_ser - Serialized state about a single PCI device.
 *
 * @domain: The device's PCI domain number (segment).
 * @bdf: The device's PCI bus, device, and function number.
 * @refcount: Reference count used by the PCI core to keep track of whether it
 *            is done using a device's struct pci_dev_ser. The value of the
 *            refcount is equal to 1 when the struct pci_dev_ser is in use, and
 *            0 otherwise.
 */
struct pci_dev_ser {
	u32 domain;
	u16 bdf;
	u16 refcount;
} __packed;

/**
 * struct pci_ser - PCI Subsystem Live Update State
 *
 * This struct tracks state about all devices that are being preserved across
 * a Live Update for the next kernel.
 *
 * @version: The version of the "pci" FLB struct. This field must never be
 *           deleted, moved, or resized, as the kernel depends on always being
 *           able to check the struct pci_ser version number.
 * @nr_devices: The number of devices that were preserved.
 * @devices: Physical address of the first KHO block containing pci_dev_ser.
 */
struct pci_ser {
	u32 version;
	u32 nr_devices;
	u64 devices;
} __packed;

#endif /* _LINUX_KHO_ABI_PCI_H */
