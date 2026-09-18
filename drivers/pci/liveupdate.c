// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2026, Google LLC.
 * David Matlack <dmatlack@google.com>
 */

/**
 * DOC: PCI Live Update
 *
 * The PCI subsystem participates in the Live Update process to enable drivers
 * to preserve their PCI devices across kexec.
 *
 * Preserving a device requires preserving two independent sets of state: the
 * driver's own state, which the driver preserves with no involvement from the
 * PCI core, and the PCI core's state about the device, which the next kernel
 * needs so that enumeration does not disturb a device that is still running.
 * This file implements the latter.
 *
 * :ref:`FLB <flb>` Data
 * =====================
 *
 * Userspace decides which devices are preserved, using :ref:`LUO <luo>` file
 * preservation: a driver exposes a file that represents a single PCI device,
 * and userspace preserves the device with
 * ``ioctl(LIVEUPDATE_SESSION_PRESERVE_FD)`` on that file. Binding preservation
 * to a file gives it proper lifecycle management, e.g. the preservation is
 * undone if userspace cancels it or goes away. How a driver exposes that file
 * is up to the driver and invisible to the PCI core (vfio-pci variant drivers,
 * the first intended use-case, use their per-device cdev).
 *
 * LUO only knows that a file was preserved; it does not know that the file
 * represents a PCI device. Drivers therefore register their
 * struct liveupdate_file_handler with the PCI core:
 *
 *  * ``pci_liveupdate_register_flb(driver_file_handler)``
 *  * ``pci_liveupdate_unregister_flb(driver_file_handler)``
 *
 * LUO then refcounts the PCI core's FLB against the files preserved by that
 * handler, and that refcount drives the lifetime of struct pci_ser:
 * pci_flb_preserve() allocates and preserves it when the first file is
 * preserved, and pci_flb_unpreserve() frees it when the last file is
 * unpreserved. In the next kernel, pci_flb_retrieve() hands the PCI core the
 * struct pci_ser built by the previous kernel, whenever the PCI core asks for
 * it (e.g. during enumeration), and pci_flb_finish() frees it once the PCI
 * core is done with it.
 *
 * Call Flow
 * ---------
 *
 * ::
 *
 *   # Driver initialization
 *   pci_liveupdate_register_flb(fh)
 *
 *   # Userspace: ioctl(LIVEUPDATE_SESSION_PRESERVE_FD, devfd)
 *   luo_preserve_file()
 *     luo_flb_file_preserve()
 *       luo_flb_file_preserve_one()      # first preserved file only
 *         pci_flb_preserve()             # alloc and preserve struct pci_ser
 *     fh->ops->preserve()                # driver callback
 *
 *   # Userspace: preservation cancelled or session torn down
 *   luo_file_unpreserve_files()
 *     luo_flb_file_unpreserve()
 *       liveupdate_flb_put_outgoing()    # last unpreserved file only
 *         pci_flb_unpreserve()           # free struct pci_ser
 *
 *   # ---------------- kexec ----------------
 *
 *   # New kernel: the PCI core asks for the previous kernel's state
 *   liveupdate_flb_get_incoming()
 *     luo_flb_retrieve_one()             # first request only
 *       pci_flb_retrieve()               # previous kernel's struct pci_ser
 *
 *   # Userspace: ioctl(LIVEUPDATE_SESSION_FINISH)
 *   luo_file_finish_one()
 *     fh->ops->finish()                  # driver callback
 *     luo_flb_file_finish()
 *       liveupdate_flb_put_incoming()    # last incoming file only
 *         pci_flb_finish()               # free struct pci_ser
 */

#define pr_fmt(fmt) "PCI: liveupdate: " fmt

#include <linux/io.h>
#include <linux/kexec_handover.h>
#include <linux/kho/abi/pci.h>
#include <linux/kho_block.h>
#include <linux/liveupdate.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/slab.h>

/**
 * struct pci_flb_outgoing - Outgoing PCI FLB object
 * @ser: Pointer to the preserved struct pci_ser.
 * @block_set: The KHO block set holding the outgoing devices.
 *
 * This structure holds the runtime state for the outgoing PCI Live Update
 * state. It wraps the serialized pci_ser and the block_set used to manage
 * the serialized entries.
 */
struct pci_flb_outgoing {
	struct pci_ser *ser;
	struct kho_block_set block_set;
};

static int pci_flb_preserve(struct liveupdate_flb_op_args *args)
{
	struct pci_flb_outgoing *outgoing __free(kfree) = NULL;
	struct pci_ser *ser;

	outgoing = kzalloc_obj(*outgoing);
	if (!outgoing)
		return -ENOMEM;

	ser = kho_alloc_preserve(sizeof(*ser));
	if (IS_ERR(ser))
		return PTR_ERR(ser);

	ser->version = PCI_LUO_FLB_VERSION;
	ser->nr_devices = 0;
	ser->devices = 0;

	outgoing->ser = ser;
	kho_block_set_init(&outgoing->block_set, sizeof(struct pci_dev_ser));

	args->obj = no_free_ptr(outgoing);
	args->data = virt_to_phys(ser);
	pr_debug("Preserved struct pci_ser (0x%llx)\n", args->data);
	return 0;
}

static void pci_flb_unpreserve(struct liveupdate_flb_op_args *args)
{
	struct pci_flb_outgoing *outgoing = args->obj;

	pr_debug("Unpreserving struct pci_ser (0x%llx)\n", args->data);

	WARN_ON(outgoing->ser->nr_devices);
	kho_block_set_destroy(&outgoing->block_set);
	kho_unpreserve_free(outgoing->ser);
	kfree(outgoing);
}

static int pci_flb_retrieve(struct liveupdate_flb_op_args *args)
{
	pr_debug("Retrieving struct pci_ser (0x%llx)\n", args->data);
	args->obj = phys_to_virt(args->data);
	return 0;
}

static void pci_flb_finish(struct liveupdate_flb_op_args *args)
{
	pr_debug("Finished struct pci_ser (0x%llx)\n", args->data);
	kho_restore_free(args->obj);
}

static struct liveupdate_flb_ops pci_liveupdate_flb_ops = {
	.preserve = pci_flb_preserve,
	.unpreserve = pci_flb_unpreserve,
	.retrieve = pci_flb_retrieve,
	.finish = pci_flb_finish,
	.owner = THIS_MODULE,
};

static struct liveupdate_flb pci_liveupdate_flb = {
	.ops = &pci_liveupdate_flb_ops,
	.compatible = PCI_LUO_FLB_COMPATIBLE,
};

/**
 * pci_liveupdate_register_flb() - Register a file handler with the PCI core
 * @fh: The file handler to register.
 *
 * Drivers that support preserving PCI devices across Live Update must call
 * pci_liveupdate_register_flb() to register their
 * struct liveupdate_file_handler with the PCI core, typically at module init,
 * and always before any file managed by @fh can be preserved.
 *
 * Registering links the PCI core's FLB to @fh, so that LUO allocates the PCI
 * core's outgoing struct pci_ser (via pci_flb_preserve()) when the first file
 * managed by any registered handler is preserved, and frees it (via
 * pci_flb_unpreserve()) when the last such file is unpreserved.
 *
 * Return: 0 on success, <0 on failure.
 */
int pci_liveupdate_register_flb(struct liveupdate_file_handler *fh)
{
	pr_debug("Registering file handler \"%s\"\n", fh->compatible);
	return liveupdate_register_flb(fh, &pci_liveupdate_flb);
}
EXPORT_SYMBOL_GPL(pci_liveupdate_register_flb);

/**
 * pci_liveupdate_unregister_flb() - Unregister a file handler with the PCI core
 * @fh: The file handler to unregister.
 */
void pci_liveupdate_unregister_flb(struct liveupdate_file_handler *fh)
{
	pr_debug("Unregistering file handler \"%s\"\n", fh->compatible);
	liveupdate_unregister_flb(fh, &pci_liveupdate_flb);
}
EXPORT_SYMBOL_GPL(pci_liveupdate_unregister_flb);
