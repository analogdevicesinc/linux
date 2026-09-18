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
 *       pci_liveupdate_preserve(dev)     # record this device in struct pci_ser
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
 *
 * Device Tracking
 * ===============
 *
 * Drivers must notify the PCI core when specific devices are preserved or
 * unpreserved with the following APIs:
 *
 *  * ``pci_liveupdate_preserve(pci_dev)``
 *  * ``pci_liveupdate_unpreserve(pci_dev)``
 *
 * This allows the PCI core to keep its FLB data (struct pci_ser) up to date
 * with the list of **outgoing** preserved devices for the next kernel.
 *
 * Restrictions
 * ============
 *
 * The PCI core enforces the following restrictions on which devices can be
 * preserved. These may be relaxed in the future:
 *
 *  * The device cannot be a Virtual Function (VF).
 *  * The device cannot be behind a PCI-to-PCI bridge.
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

#include "liveupdate.h"

/**
 * struct pci_liveupdate_global - Global state for PCI Live Update support
 * @rwsem: Reader/writer semaphore used to protect the incoming and outgoing
 *         FLBs, and the references to them in struct pci_dev.
 */
struct pci_liveupdate_global {
	struct rw_semaphore rwsem;
};

static struct pci_liveupdate_global pci_liveupdate = {
	.rwsem = __RWSEM_INITIALIZER(pci_liveupdate.rwsem),
};

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

static void pci_liveupdate_flb_put_outgoing(void)
{
	liveupdate_flb_put_outgoing(&pci_liveupdate_flb);
}

static struct pci_flb_outgoing *pci_liveupdate_flb_get_outgoing(void)
{
	struct pci_flb_outgoing *outgoing = NULL;
	int ret;

	ret = liveupdate_flb_get_outgoing(&pci_liveupdate_flb, (void **)&outgoing);
	if (ret)
		return ERR_PTR(ret);

	if (!outgoing)
		return ERR_PTR(-ENOENT);

	return outgoing;
}

static struct pci_dev_ser *pci_flb_alloc_dev_ser(struct pci_flb_outgoing *outgoing)
{
	struct pci_dev_ser *dev_ser;
	struct kho_block_set_it it;
	u64 count = 0;
	int err;

	kho_block_set_it_init(&it, &outgoing->block_set);

	/* Try to find an existing, previously unpreserved, entry. */
	while ((dev_ser = kho_block_set_it_read_entry(&it))) {
		if (!dev_ser->refcount)
			return dev_ser;

		count++;
	}

	/* Otherwise grow the block set and reserve a new entry. */
	err = kho_block_set_grow(&outgoing->block_set, count + 1);
	if (err)
		return ERR_PTR(err);

	if (!count)
		kho_block_set_it_init(&it, &outgoing->block_set);

	/* This should always succeed since kho_block_set_grow() succeeded. */
	dev_ser = kho_block_set_it_reserve_entry(&it);
	if (WARN_ON_ONCE(!dev_ser))
		return ERR_PTR(-ENOSPC);

	return dev_ser;
}

static void pci_liveupdate_unpreserve_device(struct pci_flb_outgoing *outgoing,
					     struct pci_dev *dev)
{
	struct pci_dev_ser *dev_ser = dev->liveupdate.outgoing;

	if (!dev_ser) {
		pci_warn(dev, "Cannot unpreserve device that is not preserved\n");
		return;
	}

	pci_info(dev, "Device will no longer be preserved across next Live Update\n");
	outgoing->ser->nr_devices--;
	memset(dev_ser, 0, sizeof(*dev_ser));
	dev->liveupdate.outgoing = NULL;
}

static int pci_liveupdate_preserve_device(struct pci_flb_outgoing *outgoing,
					  struct pci_dev *dev)
{
	struct pci_dev_ser *dev_ser;

	if (dev->is_virtfn) {
		pci_warn(dev, "Cannot preserve Virtual Functions\n");
		return -EINVAL;
	}

	if (dev->liveupdate.outgoing) {
		pci_warn(dev, "Device is already preserved\n");
		return -EBUSY;
	}

	if (!pci_is_root_bus(dev->bus)) {
		pci_warn(dev, "Cannot preserve devices behind bridges\n");
		return -EINVAL;
	}

	dev_ser = pci_flb_alloc_dev_ser(outgoing);
	if (IS_ERR(dev_ser))
		return PTR_ERR(dev_ser);

	pci_info(dev, "Device will be preserved across next Live Update\n");
	outgoing->ser->nr_devices++;
	outgoing->ser->devices = kho_block_set_head_pa(&outgoing->block_set);

	dev_ser->domain = pci_domain_nr(dev->bus);
	dev_ser->bdf = pci_dev_id(dev);
	dev_ser->refcount++;

	dev->liveupdate.outgoing = dev_ser;
	return 0;
}

/**
 * pci_liveupdate_preserve() - Preserve a PCI device across Live Update
 * @dev: The PCI device to preserve.
 *
 * pci_liveupdate_preserve() notifies the PCI core that a PCI device should be
 * preserved across the next Live Update. Drivers are expected to call
 * pci_liveupdate_preserve() from their struct liveupdate_file_handler
 * preserve() callback to ensure the outgoing struct pci_ser is already set up.
 *
 * Returns: 0 on success, <0 on failure.
 */
int pci_liveupdate_preserve(struct pci_dev *dev)
{
	struct pci_flb_outgoing *outgoing = NULL;
	int ret;

	guard(rwsem_write)(&pci_liveupdate.rwsem);

	outgoing = pci_liveupdate_flb_get_outgoing();
	if (IS_ERR(outgoing))
		return PTR_ERR(outgoing);

	ret = pci_liveupdate_preserve_device(outgoing, dev);

	pci_liveupdate_flb_put_outgoing();
	return ret;
}
EXPORT_SYMBOL_GPL(pci_liveupdate_preserve);

/**
 * pci_liveupdate_unpreserve() - Cancel preservation of a PCI device
 * @dev: The PCI device to unpreserve.
 *
 * pci_liveupdate_unpreserve() notifies the PCI core that a PCI device should no
 * longer be preserved across the next Live Update. Drivers are expected to call
 * pci_liveupdate_unpreserve() from their struct liveupdate_file_handler
 * unpreserve() callback to ensure the outgoing struct pci_ser is already set
 * up.
 */
void pci_liveupdate_unpreserve(struct pci_dev *dev)
{
	struct pci_flb_outgoing *outgoing = NULL;

	guard(rwsem_write)(&pci_liveupdate.rwsem);

	outgoing = pci_liveupdate_flb_get_outgoing();
	if (IS_ERR(outgoing)) {
		pci_warn(dev, "Cannot unpreserve device without outgoing Live Update state\n");
		return;
	}

	pci_liveupdate_unpreserve_device(outgoing, dev);
	pci_liveupdate_flb_put_outgoing();
}
EXPORT_SYMBOL_GPL(pci_liveupdate_unpreserve);

void pci_liveupdate_cleanup_device(struct pci_dev *dev)
{
	/*
	 * It should be safe to READ_ONCE() outside of the rwsem during cleanup
	 * since there should no longer be any references to @dev on the system.
	 *
	 * This should never happen in practice. Drivers should block removal
	 * while a device is preserved.
	 */
	if (READ_ONCE(dev->liveupdate.outgoing))
		pci_WARN(dev, 1, "Destroying outgoing-preserved device!\n");
}

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
