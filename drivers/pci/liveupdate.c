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
 * State handed over by the previous kernel is trusted. The PCI core validates
 * it only far enough to detect an incompatible or corrupt hand over, and makes
 * no attempt to defend against deliberate modification, since a previous kernel
 * able to corrupt preserved state is able to corrupt arbitrary memory anyway.
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
 *   # New kernel: PCI enumeration
 *   pci_setup_device()
 *     pci_liveupdate_setup_device()
 *       liveupdate_flb_get_incoming()
 *         luo_flb_retrieve_one()         # first request only
 *           pci_flb_retrieve()           # previous kernel's struct pci_ser
 *
 *   # Userspace: ioctl(LIVEUPDATE_SESSION_FINISH)
 *   luo_file_finish_one()
 *     fh->ops->finish()                  # driver callback
 *       pci_liveupdate_finish(dev)       # release this device's pci_dev_ser
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
 * After kexec, whenever a device is enumerated, the PCI core will check if it
 * is an **incoming** preserved device (i.e. preserved by the previous kernel)
 * by checking the incoming FLB data (struct pci_ser).
 *
 * Drivers must notify the PCI core when an **incoming** device is done
 * participating in the incoming Live Update with the following API:
 *
 *  * ``pci_liveupdate_finish(pci_dev)``
 *
 * The PCI core does not enforce any ordering of ``pci_liveupdate_finish()`` and
 * ``pci_liveupdate_preserve()``, i.e., a PCI device can be **outgoing**
 * (preserved for next kernel) and **incoming** (preserved by previous kernel)
 * at the same time.
 *
 * Restrictions
 * ============
 *
 * The PCI core enforces the following restrictions on which devices can be
 * preserved. These may be relaxed in the future:
 *
 *  * The device cannot be a Virtual Function (VF).
 *
 * Driver Binding
 * ==============
 *
 * In the outgoing kernel, the driver must ensure that it does not release a
 * device between pci_liveupdate_preserve() and pci_liveupdate_unpreserve().
 *
 * In the incoming kernel, the driver must ensure that it does not release a
 * preserved device between probe() and pci_liveupdate_finish().
 *
 * It is the user's responsibility to ensure that incoming preserved devices are
 * bound to the correct driver. The PCI core does not protect against a device
 * getting preserved by driver A in the outgoing kernel and then getting bound
 * to driver B in the incoming kernel.
 *
 * PCI-to-PCI Bridges
 * ==================
 *
 * Any PCI-to-PCI bridges upstream of a preserved device are automatically
 * preserved when the device is preserved. The PCI core keeps track of the
 * number of downstream devices that are preserved under a bridge so that the
 * bridge is only unpreserved once all downstream devices are unpreserved.
 *
 * This enables the PCI core and any drivers bound to the bridge to participate
 * in the Live Update so that preserved endpoints can continue issuing memory
 * transactions during the Live Update.
 *
 * BDF Stability
 * =============
 *
 * The PCI core guarantees that preserved devices can be identified by the same
 * bus, device, and function numbers for as long as they are preserved
 * (including across kexec). To accomplish this, the PCI core keeps the
 * secondary and subordinate bus numbers that the previous kernel programmed
 * into bridges, if the previous kernel preserved any device. This is true even
 * on architectures that always assign new bus numbers during scanning. The
 * kernel assumes the previous kernel established a sane bus topology across
 * kexec.
 *
 * Bridges that do not have bus numbers are assigned new ones as usual, so
 * hot-adding a bridge keeps working, both during and after a Live Update. The
 * two-pass bridge scan ensures such bridges are only assigned bus numbers above
 * those already claimed by preserved bridges.
 *
 * If a preserved bridge comes up without a valid bus number configuration, e.g.
 * because it was reset during kexec, the PCI core refuses to assign it new bus
 * numbers and does not enumerate anything below it. Assigning new bus numbers
 * would silently change the BDF of every preserved device in its hierarchy. The
 * PCI core also stops assigning bus numbers to the other bridges on the same
 * bus, since the bus numbers of the failed bridge can no longer be read from
 * hardware and handing them to another bridge would let an unrelated device
 * inherit the BDF of a preserved device.
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
 * @had_incoming: True if the previous kernel preserved at least one PCI device.
 *                Set when the incoming FLB is retrieved and never cleared, so
 *                it stays true after Live Update finishes.
 */
struct pci_liveupdate_global {
	struct rw_semaphore rwsem;
	bool had_incoming;
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

/**
 * struct pci_flb_incoming - Incoming PCI FLB object
 * @ser: The incoming struct pci_ser from the previous kernel.
 * @xa: Xarray used to quickly lookup devices in @ser.
 * @block_set: The KHO block set holding the incoming devices.
 *
 * This structure holds the runtime state for the incoming PCI Live Update
 * state. It wraps the serialized pci_ser, the block_set used to restore
 * the serialized entries, and an xarray for fast lookups.
 */
struct pci_flb_incoming {
	struct pci_ser *ser;
	struct xarray xa;
	struct kho_block_set block_set;
};

static unsigned long pci_ser_xa_key(u32 domain, u16 bdf)
{
	return (unsigned long)domain << 16 | bdf;
}

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

/*
 * Any failure here is fatal. The previous kernel handed over devices that are
 * still performing DMA, and this kernel cannot identify them without this
 * state. Continuing would let the PCI core reassign bus numbers and rebind
 * drivers underneath live devices, so fail loudly instead of unwinding.
 */
static int pci_flb_retrieve(struct liveupdate_flb_op_args *args)
{
	struct pci_ser *ser = phys_to_virt(args->data);
	struct pci_flb_incoming *incoming;
	struct pci_dev_ser *dev_ser;
	struct kho_block_set_it it;
	int ret;

	pr_debug("Retrieving struct pci_ser (0x%llx)\n", args->data);

	if (ser->version != PCI_LUO_FLB_VERSION)
		panic("Incoming PCI FLB version (v%d) is incompatible with this kernel (v%d)\n",
		      ser->version, PCI_LUO_FLB_VERSION);

	incoming = kzalloc_obj(*incoming);
	if (!incoming)
		panic("Failed to allocate struct pci_flb_incoming\n");

	incoming->ser = ser;
	xa_init(&incoming->xa);

	kho_block_set_init(&incoming->block_set, sizeof(struct pci_dev_ser));
	ret = kho_block_set_restore(&incoming->block_set, ser->devices);
	if (ret)
		panic("Failed to restore devices KHO block set (%d)\n", ret);

	kho_block_set_it_init(&it, &incoming->block_set);
	while ((dev_ser = kho_block_set_it_read_entry(&it))) {
		unsigned long key;

		if (!dev_ser->refcount)
			continue;

		key = pci_ser_xa_key(dev_ser->domain, dev_ser->bdf);
		ret = xa_insert(&incoming->xa, key, dev_ser, GFP_KERNEL);
		if (ret)
			panic("Failed to insert PCI device %04x:%02x:%02x.%d into xarray (%d)\n",
			      dev_ser->domain, PCI_BUS_NUM(dev_ser->bdf),
			      PCI_SLOT(dev_ser->bdf), PCI_FUNC(dev_ser->bdf),
			      ret);
	}

	/*
	 * Remember that the previous kernel preserved devices for the lifetime
	 * of this kernel, even after Live Update finishes and the incoming FLB
	 * is freed. See pci_liveupdate_preserve_bus_numbers().
	 */
	if (!xa_empty(&incoming->xa))
		pci_liveupdate.had_incoming = true;

	args->obj = incoming;
	return 0;
}

static void pci_check_all_devices_finished(struct pci_flb_incoming *incoming)
{
	struct pci_dev_ser *dev_ser;
	unsigned long index;
	u32 nr_devices;

	/*
	 * nr_devices is only decremented by pci_liveupdate_finish_device().
	 * This runs once the last reference to the incoming FLB is dropped, so
	 * there are no finishers left in flight.
	 */
	nr_devices = incoming->ser->nr_devices;
	if (nr_devices == 0)
		return;

	/*
	 * Report the unfinished devices from the incoming FLB rather than by
	 * walking struct pci_dev, so that devices that never showed up after
	 * kexec, or that were destroyed before they finished, are identified
	 * as well.
	 */
	xa_for_each(&incoming->xa, index, dev_ser) {
		if (!dev_ser->refcount)
			continue;

		pr_emerg("%04x:%02x:%02x.%d was never finished!\n",
			 dev_ser->domain, PCI_BUS_NUM(dev_ser->bdf),
			 PCI_SLOT(dev_ser->bdf), PCI_FUNC(dev_ser->bdf));
	}

	/*
	 * This should only happen if a driver violated the contract to call
	 * pci_liveupdate_finish() (something is extremely broken).
	 */
	panic("%u preserved device(s) were never finished!\n", nr_devices);
}

static void pci_flb_finish(struct liveupdate_flb_op_args *args)
{
	struct pci_flb_incoming *incoming = args->obj;

	pr_debug("Finished struct pci_ser (0x%llx)\n", args->data);
	pci_check_all_devices_finished(incoming);

	xa_destroy(&incoming->xa);
	kho_block_set_destroy(&incoming->block_set);
	kho_restore_free(incoming->ser);
	kfree(incoming);
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

static int pci_liveupdate_unpreserve_device(struct pci_flb_outgoing *outgoing,
					    struct pci_dev *dev)
{
	struct pci_dev_ser *dev_ser = dev->liveupdate.outgoing;

	if (!dev_ser) {
		pci_warn(dev, "Cannot unpreserve device that is not preserved\n");
		return -EINVAL;
	}

	if (!dev_ser->refcount) {
		pci_WARN(dev, 1, "Preserved device has a 0 refcount!\n");
		return -EINVAL;
	}

	if (--dev_ser->refcount)
		return 0;

	pci_info(dev, "Device will no longer be preserved across next Live Update\n");
	outgoing->ser->nr_devices--;
	memset(dev_ser, 0, sizeof(*dev_ser));
	dev->liveupdate.outgoing = NULL;
	return 0;
}

static void pci_liveupdate_unpreserve_path(struct pci_flb_outgoing *outgoing,
					   struct pci_dev *dev,
					   struct pci_dev *end)
{
	for_each_pci_dev_in_path(dev) {
		if (dev == end)
			break;

		if (pci_liveupdate_unpreserve_device(outgoing, dev))
			return;
	}
}

static int pci_liveupdate_preserve_device(struct pci_flb_outgoing *outgoing,
					  struct pci_dev *dev)
{
	if (dev->is_virtfn) {
		pci_warn(dev, "Cannot preserve Virtual Functions\n");
		return -EINVAL;
	}

	/*
	 * Endpoint devices should not be preserved more than once.
	 * Bridges are preserved once for every downstream device that
	 * is preserved.
	 */
	if (dev->liveupdate.outgoing && !dev->subordinate) {
		pci_warn(dev, "Device is already preserved\n");
		return -EBUSY;
	}

	if (dev->liveupdate.outgoing && !dev->liveupdate.outgoing->refcount) {
		pci_WARN(dev, 1, "Preserved device with 0 refcount!\n");
		return -EINVAL;
	}

	if (!dev->liveupdate.outgoing) {
		struct pci_dev_ser *dev_ser;

		dev_ser = pci_flb_alloc_dev_ser(outgoing);
		if (IS_ERR(dev_ser))
			return PTR_ERR(dev_ser);

		pci_info(dev, "Device will be preserved across next Live Update\n");
		outgoing->ser->nr_devices++;
		outgoing->ser->devices = kho_block_set_head_pa(&outgoing->block_set);

		dev_ser->domain = pci_domain_nr(dev->bus);
		dev_ser->bdf = pci_dev_id(dev);
		dev->liveupdate.outgoing = dev_ser;
	}

	dev->liveupdate.outgoing->refcount++;
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
 * pci_liveupdate_preserve() automatically preserves all bridges upstream of
 * @dev.
 *
 * Returns: 0 on success, <0 on failure.
 */
int pci_liveupdate_preserve(struct pci_dev *dev)
{
	struct pci_flb_outgoing *outgoing = NULL;
	struct pci_dev *start = dev;
	int ret = -ENODEV;

	guard(rwsem_write)(&pci_liveupdate.rwsem);

	outgoing = pci_liveupdate_flb_get_outgoing();
	if (IS_ERR(outgoing))
		return PTR_ERR(outgoing);

	for_each_pci_dev_in_path(dev) {
		ret = pci_liveupdate_preserve_device(outgoing, dev);
		if (ret) {
			pci_liveupdate_unpreserve_path(outgoing, start, dev);
			break;
		}
	}

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
 *
 * pci_liveupdate_unpreserve() automatically unpreserves all bridges upstream of
 * @dev.
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

	pci_liveupdate_unpreserve_path(outgoing, dev, /*end=*/NULL);
	pci_liveupdate_flb_put_outgoing();
}
EXPORT_SYMBOL_GPL(pci_liveupdate_unpreserve);

static struct pci_flb_incoming *pci_liveupdate_flb_get_incoming(void)
{
	struct pci_flb_incoming *incoming = NULL;
	int ret;

	ret = liveupdate_flb_get_incoming(&pci_liveupdate_flb, (void **)&incoming);

	/* Live Update is not enabled. */
	if (ret == -EOPNOTSUPP)
		return NULL;

	/* Live Update is enabled, but there is no incoming FLB data. */
	if (ret == -ENODATA)
		return NULL;

	/*
	 * Live Update is enabled and there is incoming FLB data, but none of it
	 * matches pci_liveupdate_flb.compatible.
	 */
	if (ret == -ENOENT)
		return NULL;

	/*
	 * There is incoming FLB data that matches pci_liveupdate_flb.compatible
	 * but retrieve failed (pci_flb_retrieve() returned an error or LUO
	 * failed to acquire a reference to pci_liveupdate_flb_ops.owner).
	 */
	if (ret)
		panic("Failed to retrieve incoming FLB data (%d)\n", ret);

	return incoming;
}

static void pci_liveupdate_flb_put_incoming(void)
{
	liveupdate_flb_put_incoming(&pci_liveupdate_flb);
}

/**
 * pci_liveupdate_preserve_bus_numbers() - Determine if the PCI core should
 *                                         preserve bus numbers when scanning
 *                                         bridges.
 *
 * This function is called by the PCI core when it is scanning a bridge. It
 * determines whether the PCI core should preserve the secondary and subordinate
 * bus numbers that the previous kernel programmed into that bridge, rather than
 * assigning new ones. This is necessary to keep RequesterIDs constant for
 * preserved devices issuing memory transactions.
 *
 * Bus numbers are preserved everywhere, and for the lifetime of the kernel, if
 * the previous kernel preserved any device. Bus numbers have to be preserved
 * above a preserved device anyway, since an upstream bridge cannot expand its
 * window. Applying the same policy everywhere matches the scope of
 * pcibios_assign_all_busses(), and gives an answer that cannot change part way
 * through the two passes of a bridge scan.
 *
 * The incoming FLB is retrieved while setting up the first device, which always
 * happens before any bridge is scanned, so this returns the same answer for the
 * entire enumeration.
 *
 * Note that this does not prevent the PCI core from assigning bus numbers to
 * bridges that do not have any, e.g. bridges that are hot-added after the
 * Live Update. See pci_liveupdate_refuse_bus_numbers() for the one case where
 * the PCI core must refuse to do so.
 *
 * Return: True if bus numbers should be preserved, false otherwise.
 */
bool pci_liveupdate_preserve_bus_numbers(void)
{
	return pci_liveupdate.had_incoming;
}

/**
 * pci_liveupdate_refuse_bus_numbers() - Determine if the PCI core must refuse
 *                                       to assign bus numbers to the provided
 *                                       bridge.
 * @bus: The PCI bus the bus numbers would be assigned from.
 * @dev: The PCI bridge device the bus numbers would be assigned to.
 *
 * This function is called by the PCI core before it assigns bus numbers to a
 * bridge that does not have any.
 *
 * A bridge that was preserved by the previous kernel but came up without a
 * valid bus number configuration, e.g. because it was reset during kexec, is
 * left alone by the PCI core and therefore has no child bus once the first pass
 * of the bridge scan is done.
 *
 * The PCI core must not assign bus numbers from @bus while such a bridge is on
 * it, including to the failed bridge itself. Assigning new bus numbers to the
 * failed bridge would silently change the BDF of every preserved device in its
 * hierarchy. Its bus numbers cannot be read from hardware anymore either, so
 * they cannot be excluded from assignment, and handing them to another bridge
 * would let an unrelated device inherit the BDF of a preserved device.
 *
 * Return: True if @dev must not be assigned bus numbers, false otherwise.
 */
bool pci_liveupdate_refuse_bus_numbers(struct pci_bus *bus, struct pci_dev *dev)
{
	struct pci_dev *bridge;

	for_each_pci_bridge(bridge, bus) {
		if (!bridge->liveupdate.was_incoming || bridge->subordinate)
			continue;

		pci_err(dev, "Not assigning bus numbers, preserved bridge %s lost its bus number configuration\n",
			pci_name(bridge));
		return true;
	}

	return false;
}

void pci_liveupdate_setup_device(struct pci_dev *dev)
{
	struct pci_flb_incoming *incoming;
	struct pci_dev_ser *dev_ser;
	unsigned long key;

	guard(rwsem_write)(&pci_liveupdate.rwsem);

	incoming = pci_liveupdate_flb_get_incoming();
	if (!incoming)
		return;

	key = pci_ser_xa_key(pci_domain_nr(dev->bus), pci_dev_id(dev));
	dev_ser = xa_load(&incoming->xa, key);

	/*
	 * This device was not preserved across Live Update, or it was preserved
	 * but has already been probed and gone through pci_liveupdate_finish(),
	 * e.g. due to removing and re-adding the device. Either way, it's not
	 * treated as incoming-preserved.
	 */
	if (!dev_ser || !dev_ser->refcount) {
		pci_liveupdate_flb_put_incoming();
		return;
	}

	pci_info(dev, "Device was preserved by previous kernel across Live Update\n");
	dev->liveupdate.incoming = dev_ser;
	dev->liveupdate.was_incoming = true;

	pci_liveupdate_flb_put_incoming();
}

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

	if (READ_ONCE(dev->liveupdate.incoming))
		pci_WARN(dev, 1, "Destroying incoming-preserved device!\n");
}

static int pci_liveupdate_finish_device(struct pci_ser *ser, struct pci_dev *dev)
{
	if (!dev->liveupdate.incoming) {
		pci_warn(dev, "Cannot finish preserving an unpreserved device\n");
		return -EINVAL;
	}

	if (!dev->liveupdate.incoming->refcount) {
		pci_WARN(dev, 1, "Preserved device has a 0 refcount!\n");
		return -EINVAL;
	}

	/*
	 * Decrement the refcount so this device does not get treated as an
	 * incoming device again, e.g. in case pci_liveupdate_setup_device()
	 * gets called again because the device is hot-plugged.
	 */
	if (--dev->liveupdate.incoming->refcount)
		return 0;

	pci_info(dev, "Device is finished participating in Live Update\n");
	dev->liveupdate.incoming = NULL;
	ser->nr_devices--;
	return 0;
}

/**
 * pci_liveupdate_finish() - Finish the preservation of a PCI device
 * @dev: The PCI device
 *
 * pci_liveupdate_finish() notifies the PCI core that a PCI device that was
 * preserved across the previous Live Update has finished participating in Live
 * Update. Drivers must call pci_liveupdate_finish() from their struct
 * liveupdate_file_handler finish() callback to ensure the incoming struct
 * pci_ser is allocated.
 *
 * pci_liveupdate_finish() automatically finishes all bridges upstream of @dev.
 */
void pci_liveupdate_finish(struct pci_dev *dev)
{
	struct pci_flb_incoming *incoming;

	guard(rwsem_write)(&pci_liveupdate.rwsem);

	incoming = pci_liveupdate_flb_get_incoming();
	if (!incoming) {
		pci_warn(dev, "Cannot finish preserving device without incoming FLB\n");
		return;
	}

	for_each_pci_dev_in_path(dev) {
		if (pci_liveupdate_finish_device(incoming->ser, dev))
			break;
	}

	pci_liveupdate_flb_put_incoming();
}
EXPORT_SYMBOL_GPL(pci_liveupdate_finish);

/**
 * pci_liveupdate_is_incoming() - Check if a device is incoming-preserved
 * @dev: The PCI device to check
 *
 * Check if a device was preserved across Live Update by the previous kernel,
 * i.e. the device is incoming-preserved. Note that a device is only considered
 * incoming-preserved prior to pci_liveupdate_finish(). It is up to drivers to
 * synchronize usage of pci_liveupdate_is_incoming() with their own call to
 * pci_liveupdate_finish() to avoid acting on stale data.
 *
 * Returns: True if the device is incoming-preserved, false otherwise.
 */
bool pci_liveupdate_is_incoming(struct pci_dev *dev)
{
	guard(rwsem_read)(&pci_liveupdate.rwsem);
	return dev->liveupdate.incoming;
}
EXPORT_SYMBOL_GPL(pci_liveupdate_is_incoming);

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
