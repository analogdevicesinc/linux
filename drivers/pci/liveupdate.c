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
 * :ref:`FLB <flb>` Data
 * =====================
 *
 * PCI device preservation across Live Update is built on top of the
 * :ref:`LUO <luo>` support for file preservation across kexec. Drivers are
 * expected to expose a file to represent a single PCI device and support
 * preservation of that file with ``ioctl(LIVEUPDATE_SESSION_PRESERVE_FD)``.
 * This allows userspace to control the preservation of devices and ensure
 * proper lifecycle management while a device is preserved. The first intended
 * use-case is preserving vfio-pci device files.
 *
 * The PCI core maintains its own state about what devices are being preserved
 * across Live Update using FLB data in LUO. Essentially, this allows the PCI
 * core to allocate struct pci_ser when the first device (file) is preserved
 * and free it when the last device (file) is unpreserved. After kexec, the
 * PCI core can fetch the struct pci_ser (which was constructed by the previous
 * kernel) from LUO at any time (e.g. during enumeration) so that it knows
 * which devices were preserved.
 *
 * To enable the PCI core to be notified whenever a file representing a device
 * is preserved, drivers must register their struct liveupdate_file_handler with
 * the PCI core by using the following APIs:
 *
 *  * ``pci_liveupdate_register_flb(driver_file_handler)``
 *  * ``pci_liveupdate_unregister_flb(driver_file_handler)``
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
 * ``pci_liveupdate_preserve()``. i.e. A PCI device can be **outgoing**
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
 *  * The device cannot be behind a PCI-to-PCI bridge.
 *
 * Driver Binding
 * ==============
 *
 * In the outgoing kernel, it is the driver's responsibility to ensure that it
 * does not release a device between pci_liveupdate_preserve() and
 * pci_liveupdate_unpreserve().
 *
 * In the incoming kernel, it is the driver's responsibility to ensure that it
 * does not release a preserved device between probe() and
 * pci_liveupdate_finish().
 *
 * It is the user's responsibility to ensure that incoming preserved devices are
 * bound to the correct driver. i.e. The PCI core does not protect against a
 * device getting preserved by driver A in the outgoing kernel and then getting
 * bound to driver B in the incoming kernel. This may change in the future.
 *
 * BDF Stability
 * =============
 *
 * The PCI core guarantees that preserved devices can be identified by the same
 * bus, device, and function numbers for as long as they are preserved
 * (including across kexec). To accomplish this, the PCI core always preserves
 * the secondary and subordinate bus numbers assigned to bridges during scanning
 * if any device is preserved. This is true even on architectures that always
 * assign new bus numbers during scanning. The kernel assumes the previous
 * kernel established a sane bus topology across kexec.
 *
 * If a misconfigured or unconfigured bridge is encountered during enumeration
 * while there are preserved devices, its secondary and subordinate bus numbers
 * will be cleared and devices below it will not be enumerated.
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

static int pci_flb_retrieve(struct liveupdate_flb_op_args *args)
{
	struct pci_ser *ser = phys_to_virt(args->data);
	struct pci_flb_incoming *incoming;
	struct pci_dev_ser *dev_ser;
	struct kho_block_set_it it;
	int ret;

	pr_debug("Retrieving struct pci_ser (0x%llx)\n", args->data);

	if (ser->version != PCI_LUO_FLB_VERSION) {
		pr_err("Incoming PCI FLB version (v%d) is incompatible with this kernel (v%d)\n",
		       ser->version, PCI_LUO_FLB_VERSION);
		ret = -EINVAL;
		goto err_restore_free;
	}

	incoming = kzalloc_obj(*incoming);
	if (!incoming) {
		ret = -ENOMEM;
		goto err_restore_free;
	}

	incoming->ser = ser;
	xa_init(&incoming->xa);

	kho_block_set_init(&incoming->block_set, sizeof(struct pci_dev_ser));
	ret = kho_block_set_restore(&incoming->block_set, ser->devices);
	if (ret)
		goto err_free_incoming;

	kho_block_set_it_init(&it, &incoming->block_set);
	while ((dev_ser = kho_block_set_it_read_entry(&it))) {
		unsigned long key;

		if (!dev_ser->refcount)
			continue;

		key = pci_ser_xa_key(dev_ser->domain, dev_ser->bdf);
		ret = xa_insert(&incoming->xa, key, dev_ser, GFP_KERNEL);
		if (ret)
			goto err_block_set_destroy;
	}

	args->obj = incoming;
	return 0;

err_block_set_destroy:
	kho_block_set_destroy(&incoming->block_set);
err_free_incoming:
	xa_destroy(&incoming->xa);
	kfree(incoming);
err_restore_free:
	kho_restore_free(ser);
	return ret;
}

static void pci_check_all_devices_finished(struct pci_flb_incoming *incoming)
{
	struct pci_dev *dev = NULL;

	if (READ_ONCE(incoming->ser->nr_devices) == 0)
		return;

	for_each_pci_dev(dev) {
		if (READ_ONCE(dev->liveupdate.incoming))
			pci_emerg(dev, "Preserved device was never finished!\n");
	}

	/*
	 * This should only happen if a driver violated the contract to call
	 * pci_liveupdate_finish() (something is extremely broken).
	 */
	panic("Some preserved devices were never finished!\n");
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
	dev_ser->refcount = 1;

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

static bool pci_has_incoming_preserved_devices(void)
{
	struct pci_flb_incoming *incoming;
	u32 nr_devices;

	guard(rwsem_read)(&pci_liveupdate.rwsem);

	incoming = pci_liveupdate_flb_get_incoming();
	if (!incoming)
		return false;

	nr_devices = incoming->ser->nr_devices;
	pci_liveupdate_flb_put_incoming();

	return nr_devices > 0;
}

/**
 * pci_liveupdate_preserve_bus_numbers() - Determine if the PCI core should
 *                                         preserve bus numbers when scanning
 *                                         the provided bridge.
 * @bus: The parent bus of the bridge.
 * @dev: The PCI bridge device.
 *
 * This function is called by the PCI core when it is scanning a bridge.  It
 * determines whether the PCI core should preserve the secondary and subordinate
 * bus numbers assigned to @dev by the previous kernel. This is necessary to
 * keep RequesterIDs constant for preserved devices issuing memory transactions.
 *
 * Return: True if bus numbers should be preserved, false otherwise.
 */
bool pci_liveupdate_preserve_bus_numbers(struct pci_bus *bus, struct pci_dev *dev)
{
	struct pci_dev *parent = bus->self;

	if (dev->liveupdate.preserve_bus_numbers)
		return true;

	if (parent && parent->liveupdate.preserve_bus_numbers) {
		/*
		 * Preserve bus numbers if the parent bridge is required to
		 * preserve bus numbers. Otherwise the PCI core could expand
		 * this bridge's reservation beyond its parent (which cannot
		 * expand).
		 */
		dev->liveupdate.preserve_bus_numbers = true;
	} else {
		/*
		 * Otherwise preserve bus numbers if there are any incoming
		 * preserved devices. This ensures that the PCI core does not
		 * allocate a bus number to a non-preserved device that
		 * conflicts with the bus number already assigned to a preserved
		 * device.
		 *
		 * This is slightly more restrictive than it needs to be. For
		 * example, each host bridges have their own range of bus
		 * numbers that won't conflict with other host bridges. But the
		 * previous kernel should have assigned a sane bus topology and
		 * it is simpler to just adopt that entire topology.
		 */
		dev->liveupdate.preserve_bus_numbers =
			pci_has_incoming_preserved_devices();
	}

	return dev->liveupdate.preserve_bus_numbers;
}

/**
 * pci_liveupdate_scan_bridge_end() - Finish scanning a PCI bridge
 * @dev: The PCI bridge device.
 *
 * This function is called by the PCI core when it finishes scanning a bridge.
 * It clears the bus number preservation status of the bridge so it can be
 * re-evaluated on future scans.
 */
void pci_liveupdate_scan_bridge_end(struct pci_dev *dev)
{
	dev->liveupdate.preserve_bus_numbers = false;
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

static void pci_liveupdate_finish_device(struct pci_ser *ser, struct pci_dev *dev)
{
	if (!dev->liveupdate.incoming) {
		pci_warn(dev, "Cannot finish preserving an unpreserved device\n");
		return;
	}

	if (dev->liveupdate.incoming->refcount != 1) {
		pci_WARN(dev, 1, "Preserved device has a corrupted refcount!\n");
		return;
	}

	/*
	 * Drop the refcount so this device does not get treated as an incoming
	 * device again, e.g. in case pci_liveupdate_setup_device() gets called
	 * again because the device is hot-plugged.
	 */
	dev->liveupdate.incoming->refcount = 0;

	pci_info(dev, "Device is finished participating in Live Update\n");
	dev->liveupdate.incoming = NULL;
	ser->nr_devices--;
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

	pci_liveupdate_finish_device(incoming->ser, dev);
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
 * Drivers should call pci_liveupdate_register_flb() to register their
 * struct liveupdate_file_handler with the PCI core. This enables the PCI core
 * to allocate its outgoing struct pci_ser whenever the first device is
 * preserved, and free it when the last device is unpreserved.
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
