// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/base/power/common.c - Common device power management code.
 *
 * Copyright (C) 2011 Rafael J. Wysocki <rjw@sisk.pl>, Renesas Electronics Corp.
 */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/pm_clock.h>
#include <linux/acpi.h>
#include <linux/pm_domain.h>

#include "power.h"

/**
 * dev_pm_get_subsys_data - Create or refcount power.subsys_data for device.
 * @dev: Device to handle.
 *
 * If power.subsys_data is NULL, point it to a new object, otherwise increment
 * its reference counter.  Return 0 if new object has been created or refcount
 * increased, otherwise negative error code.
 */
int dev_pm_get_subsys_data(struct device *dev)
{
	struct pm_subsys_data *psd;

	psd = kzalloc(sizeof(*psd), GFP_KERNEL);
	if (!psd)
		return -ENOMEM;

	spin_lock_irq(&dev->power.lock);

	if (dev->power.subsys_data) {
		dev->power.subsys_data->refcount++;
	} else {
		spin_lock_init(&psd->lock);
		psd->refcount = 1;
		dev->power.subsys_data = psd;
		pm_clk_init(dev);
		psd = NULL;
	}

	spin_unlock_irq(&dev->power.lock);

	/* kfree() verifies that its argument is nonzero. */
	kfree(psd);

	return 0;
}
EXPORT_SYMBOL_GPL(dev_pm_get_subsys_data);

/**
 * dev_pm_put_subsys_data - Drop reference to power.subsys_data.
 * @dev: Device to handle.
 *
 * If the reference counter of power.subsys_data is zero after dropping the
 * reference, power.subsys_data is removed.
 */
void dev_pm_put_subsys_data(struct device *dev)
{
	struct pm_subsys_data *psd;

	spin_lock_irq(&dev->power.lock);

	psd = dev_to_psd(dev);
	if (!psd)
		goto out;

	if (--psd->refcount == 0)
		dev->power.subsys_data = NULL;
	else
		psd = NULL;

 out:
	spin_unlock_irq(&dev->power.lock);
	kfree(psd);
}
EXPORT_SYMBOL_GPL(dev_pm_put_subsys_data);

/**
 * dev_pm_domain_attach - Attach a device to its PM domain.
 * @dev: Device to attach.
 * @power_on: Used to indicate whether we should power on the device.
 *
 * The @dev may only be attached to a single PM domain. By iterating through
 * the available alternatives we try to find a valid PM domain for the device.
 * As attachment succeeds, the ->detach() callback in the struct dev_pm_domain
 * should be assigned by the corresponding attach function.
 *
 * This function should typically be invoked from subsystem level code during
 * the probe phase. Especially for those that holds devices which requires
 * power management through PM domains.
 *
 * Callers must ensure proper synchronization of this function with power
 * management callbacks.
 *
 * Returns 0 on successfully attached PM domain, or when it is found that the
 * device doesn't need a PM domain, else a negative error code.
 */
int dev_pm_domain_attach(struct device *dev, bool power_on)
{
	int ret;

	if (dev->pm_domain)
		return 0;

	ret = acpi_dev_pm_attach(dev, power_on);
	if (!ret)
		ret = genpd_dev_pm_attach(dev);

	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(dev_pm_domain_attach);

/**
 * dev_pm_domain_attach_by_id - Associate a device with one of its PM domains.
 * @dev: The device used to lookup the PM domain.
 * @index: The index of the PM domain.
 *
 * As @dev may only be attached to a single PM domain, the backend PM domain
 * provider creates a virtual device to attach instead. If attachment succeeds,
 * the ->detach() callback in the struct dev_pm_domain are assigned by the
 * corresponding backend attach function, as to deal with detaching of the
 * created virtual device.
 *
 * This function should typically be invoked by a driver during the probe phase,
 * in case its device requires power management through multiple PM domains. The
 * driver may benefit from using the received device, to configure device-links
 * towards its original device. Depending on the use-case and if needed, the
 * links may be dynamically changed by the driver, which allows it to control
 * the power to the PM domains independently from each other.
 *
 * Callers must ensure proper synchronization of this function with power
 * management callbacks.
 *
 * Returns the virtual created device when successfully attached to its PM
 * domain, NULL in case @dev don't need a PM domain, else an ERR_PTR().
 * Note that, to detach the returned virtual device, the driver shall call
 * dev_pm_domain_detach() on it, typically during the remove phase.
 */
struct device *dev_pm_domain_attach_by_id(struct device *dev,
					  unsigned int index)
{
	if (dev->pm_domain)
		return ERR_PTR(-EEXIST);

	return genpd_dev_pm_attach_by_id(dev, index);
}
EXPORT_SYMBOL_GPL(dev_pm_domain_attach_by_id);

/**
 * dev_pm_domain_attach_by_name - Associate a device with one of its PM domains.
 * @dev: The device used to lookup the PM domain.
 * @name: The name of the PM domain.
 *
 * For a detailed function description, see dev_pm_domain_attach_by_id().
 */
struct device *dev_pm_domain_attach_by_name(struct device *dev,
					    const char *name)
{
	if (dev->pm_domain)
		return ERR_PTR(-EEXIST);

	return genpd_dev_pm_attach_by_name(dev, name);
}
EXPORT_SYMBOL_GPL(dev_pm_domain_attach_by_name);

/**
 * dev_pm_domain_attach_list - Associate a device with its PM domains.
 * @dev: The device used to lookup the PM domains for.
 * @data: The data used for attaching to the PM domains.
 * @list: An out-parameter with an allocated list of attached PM domains.
 *
 * This function helps to attach a device to its multiple PM domains. The
 * caller, which is typically a driver's probe function, may provide a list of
 * names for the PM domains that we should try to attach the device to, but it
 * may also provide an empty list, in case the attach should be done for all of
 * the available PM domains.
 *
 * Callers must ensure proper synchronization of this function with power
 * management callbacks.
 *
 * Returns the number of attached PM domains or a negative error code in case of
 * a failure. Note that, to detach the list of PM domains, the driver shall call
 * dev_pm_domain_detach_list(), typically during the remove phase.
 */
int dev_pm_domain_attach_list(struct device *dev,
			      const struct dev_pm_domain_attach_data *data,
			      struct dev_pm_domain_list **list)
{
	struct device_node *np = dev->of_node;
	struct dev_pm_domain_list *pds;
	struct device *pd_dev = NULL;
	int ret, i, num_pds = 0;
	bool by_id = true;
	size_t size;
	u32 pd_flags = data ? data->pd_flags : 0;
	u32 link_flags = pd_flags & PD_FLAG_NO_DEV_LINK ? 0 :
			DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME;

	if (dev->pm_domain)
		return -EEXIST;

	/* For now this is limited to OF based platforms. */
	if (!np)
		return 0;

	if (data && data->pd_names) {
		num_pds = data->num_pd_names;
		by_id = false;
	} else {
		num_pds = of_count_phandle_with_args(np, "power-domains",
						     "#power-domain-cells");
	}

	if (num_pds <= 0)
		return 0;

	pds = kzalloc(sizeof(*pds), GFP_KERNEL);
	if (!pds)
		return -ENOMEM;

	size = sizeof(*pds->pd_devs) + sizeof(*pds->pd_links);
	pds->pd_devs = kcalloc(num_pds, size, GFP_KERNEL);
	if (!pds->pd_devs) {
		ret = -ENOMEM;
		goto free_pds;
	}
	pds->pd_links = (void *)(pds->pd_devs + num_pds);

	if (link_flags && pd_flags & PD_FLAG_DEV_LINK_ON)
		link_flags |= DL_FLAG_RPM_ACTIVE;

	for (i = 0; i < num_pds; i++) {
		if (by_id)
			pd_dev = dev_pm_domain_attach_by_id(dev, i);
		else
			pd_dev = dev_pm_domain_attach_by_name(dev,
							data->pd_names[i]);
		if (IS_ERR_OR_NULL(pd_dev)) {
			ret = pd_dev ? PTR_ERR(pd_dev) : -ENODEV;
			goto err_attach;
		}

		if (link_flags) {
			struct device_link *link;

			link = device_link_add(dev, pd_dev, link_flags);
			if (!link) {
				ret = -ENODEV;
				goto err_link;
			}

			pds->pd_links[i] = link;
		}

		pds->pd_devs[i] = pd_dev;
	}

	pds->num_pds = num_pds;
	*list = pds;
	return num_pds;

err_link:
	dev_pm_domain_detach(pd_dev, true);
err_attach:
	while (--i >= 0) {
		if (pds->pd_links[i])
			device_link_del(pds->pd_links[i]);
		dev_pm_domain_detach(pds->pd_devs[i], true);
	}
	kfree(pds->pd_devs);
free_pds:
	kfree(pds);
	return ret;
}
EXPORT_SYMBOL_GPL(dev_pm_domain_attach_list);

/**
 * devm_pm_domain_detach_list - devres-enabled version of dev_pm_domain_detach_list.
 * @_list: The list of PM domains to detach.
 *
 * This function reverse the actions from devm_pm_domain_attach_list().
 * it will be invoked during the remove phase from drivers implicitly if driver
 * uses devm_pm_domain_attach_list() to attach the PM domains.
 */
static void devm_pm_domain_detach_list(void *_list)
{
	struct dev_pm_domain_list *list = _list;

	dev_pm_domain_detach_list(list);
}

/**
 * devm_pm_domain_attach_list - devres-enabled version of dev_pm_domain_attach_list
 * @dev: The device used to lookup the PM domains for.
 * @data: The data used for attaching to the PM domains.
 * @list: An out-parameter with an allocated list of attached PM domains.
 *
 * NOTE: this will also handle calling devm_pm_domain_detach_list() for
 * you during remove phase.
 *
 * Returns the number of attached PM domains or a negative error code in case of
 * a failure.
 */
int devm_pm_domain_attach_list(struct device *dev,
			       const struct dev_pm_domain_attach_data *data,
			       struct dev_pm_domain_list **list)
{
	int ret, num_pds;

	num_pds = dev_pm_domain_attach_list(dev, data, list);
	if (num_pds <= 0)
		return num_pds;

	ret = devm_add_action_or_reset(dev, devm_pm_domain_detach_list, *list);
	if (ret)
		return ret;

	return num_pds;
}
EXPORT_SYMBOL_GPL(devm_pm_domain_attach_list);

/**
 * dev_pm_domain_detach - Detach a device from its PM domain.
 * @dev: Device to detach.
 * @power_off: Used to indicate whether we should power off the device.
 *
 * This functions will reverse the actions from dev_pm_domain_attach(),
 * dev_pm_domain_attach_by_id() and dev_pm_domain_attach_by_name(), thus it
 * detaches @dev from its PM domain.  Typically it should be invoked during the
 * remove phase, either from subsystem level code or from drivers.
 *
 * Callers must ensure proper synchronization of this function with power
 * management callbacks.
 */
void dev_pm_domain_detach(struct device *dev, bool power_off)
{
	if (dev->pm_domain && dev->pm_domain->detach)
		dev->pm_domain->detach(dev, power_off);
}
EXPORT_SYMBOL_GPL(dev_pm_domain_detach);

/**
 * dev_pm_domain_detach_list - Detach a list of PM domains.
 * @list: The list of PM domains to detach.
 *
 * This function reverse the actions from dev_pm_domain_attach_list().
 * Typically it should be invoked during the remove phase from drivers.
 *
 * Callers must ensure proper synchronization of this function with power
 * management callbacks.
 */
void dev_pm_domain_detach_list(struct dev_pm_domain_list *list)
{
	int i;

	if (!list)
		return;

	for (i = 0; i < list->num_pds; i++) {
		if (list->pd_links[i])
			device_link_del(list->pd_links[i]);
		dev_pm_domain_detach(list->pd_devs[i], true);
	}

	kfree(list->pd_devs);
	kfree(list);
}
EXPORT_SYMBOL_GPL(dev_pm_domain_detach_list);

/**
 * dev_pm_domain_start - Start the device through its PM domain.
 * @dev: Device to start.
 *
 * This function should typically be called during probe by a subsystem/driver,
 * when it needs to start its device from the PM domain's perspective. Note
 * that, it's assumed that the PM domain is already powered on when this
 * function is called.
 *
 * Returns 0 on success and negative error values on failures.
 */
int dev_pm_domain_start(struct device *dev)
{
	if (dev->pm_domain && dev->pm_domain->start)
		return dev->pm_domain->start(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(dev_pm_domain_start);

/**
 * dev_pm_domain_set - Set PM domain of a device.
 * @dev: Device whose PM domain is to be set.
 * @pd: PM domain to be set, or NULL.
 *
 * Sets the PM domain the device belongs to. The PM domain of a device needs
 * to be set before its probe finishes (it's bound to a driver).
 *
 * This function must be called with the device lock held.
 */
void dev_pm_domain_set(struct device *dev, struct dev_pm_domain *pd)
{
	if (dev->pm_domain == pd)
		return;

	WARN(pd && device_is_bound(dev),
	     "PM domains can only be changed for unbound devices\n");
	dev->pm_domain = pd;
	device_pm_check_callbacks(dev);
}
EXPORT_SYMBOL_GPL(dev_pm_domain_set);

/**
 * dev_pm_domain_set_performance_state - Request a new performance state.
 * @dev: The device to make the request for.
 * @state: Target performance state for the device.
 *
 * This function should be called when a new performance state needs to be
 * requested for a device that is attached to a PM domain. Note that, the
 * support for performance scaling for PM domains is optional.
 *
 * Returns 0 on success and when performance scaling isn't supported, negative
 * error code on failure.
 */
int dev_pm_domain_set_performance_state(struct device *dev, unsigned int state)
{
	if (dev->pm_domain && dev->pm_domain->set_performance_state)
		return dev->pm_domain->set_performance_state(dev, state);

	return 0;
}
EXPORT_SYMBOL_GPL(dev_pm_domain_set_performance_state);
