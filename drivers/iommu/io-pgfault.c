// SPDX-License-Identifier: GPL-2.0
/*
 * Handle device page faults
 *
 * Copyright (C) 2020 ARM Ltd.
 */

#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/sched/mm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "iommu-priv.h"

/*
 * Return the fault parameter of a device if it exists. Otherwise, return NULL.
 * On a successful return, the caller takes a reference of this parameter and
 * should put it after use by calling iopf_put_dev_fault_param().
 */
static struct iommu_fault_param *iopf_get_dev_fault_param(struct device *dev)
{
	struct dev_iommu *param = dev->iommu;
	struct iommu_fault_param *fault_param;

	rcu_read_lock();
	fault_param = rcu_dereference(param->fault_param);
	if (fault_param && !refcount_inc_not_zero(&fault_param->users))
		fault_param = NULL;
	rcu_read_unlock();

	return fault_param;
}

/* Caller must hold a reference of the fault parameter. */
static void iopf_put_dev_fault_param(struct iommu_fault_param *fault_param)
{
	if (refcount_dec_and_test(&fault_param->users))
		kfree_rcu(fault_param, rcu);
}

static void __iopf_free_group(struct iopf_group *group)
{
	struct iopf_fault *iopf, *next;

	list_for_each_entry_safe(iopf, next, &group->faults, list) {
		if (!(iopf->fault.prm.flags & IOMMU_FAULT_PAGE_REQUEST_LAST_PAGE))
			kfree(iopf);
	}

	/* Pair with iommu_report_device_fault(). */
	iopf_put_dev_fault_param(group->fault_param);
}

void iopf_free_group(struct iopf_group *group)
{
	__iopf_free_group(group);
	kfree(group);
}
EXPORT_SYMBOL_GPL(iopf_free_group);

/* Non-last request of a group. Postpone until the last one. */
static int report_partial_fault(struct iommu_fault_param *fault_param,
				struct iommu_fault *fault)
{
	struct iopf_fault *iopf;

	iopf = kzalloc(sizeof(*iopf), GFP_KERNEL);
	if (!iopf)
		return -ENOMEM;

	iopf->fault = *fault;

	mutex_lock(&fault_param->lock);
	list_add(&iopf->list, &fault_param->partial);
	mutex_unlock(&fault_param->lock);

	return 0;
}

static struct iopf_group *iopf_group_alloc(struct iommu_fault_param *iopf_param,
					   struct iopf_fault *evt,
					   struct iopf_group *abort_group)
{
	struct iopf_fault *iopf, *next;
	struct iopf_group *group;

	group = kzalloc(sizeof(*group), GFP_KERNEL);
	if (!group) {
		/*
		 * We always need to construct the group as we need it to abort
		 * the request at the driver if it can't be handled.
		 */
		group = abort_group;
	}

	group->fault_param = iopf_param;
	group->last_fault.fault = evt->fault;
	INIT_LIST_HEAD(&group->faults);
	INIT_LIST_HEAD(&group->pending_node);
	list_add(&group->last_fault.list, &group->faults);

	/* See if we have partial faults for this group */
	mutex_lock(&iopf_param->lock);
	list_for_each_entry_safe(iopf, next, &iopf_param->partial, list) {
		if (iopf->fault.prm.grpid == evt->fault.prm.grpid)
			/* Insert *before* the last fault */
			list_move(&iopf->list, &group->faults);
	}
	list_add(&group->pending_node, &iopf_param->faults);
	mutex_unlock(&iopf_param->lock);

	group->fault_count = list_count_nodes(&group->faults);

	return group;
}

static struct iommu_attach_handle *find_fault_handler(struct device *dev,
						     struct iopf_fault *evt)
{
	struct iommu_fault *fault = &evt->fault;
	struct iommu_attach_handle *attach_handle;

	if (fault->prm.flags & IOMMU_FAULT_PAGE_REQUEST_PASID_VALID) {
		attach_handle = iommu_attach_handle_get(dev->iommu_group,
				fault->prm.pasid, 0);
		if (IS_ERR(attach_handle)) {
			const struct iommu_ops *ops = dev_iommu_ops(dev);

			if (!ops->user_pasid_table)
				return NULL;
			/*
			 * The iommu driver for this device supports user-
			 * managed PASID table. Therefore page faults for
			 * any PASID should go through the NESTING domain
			 * attached to the device RID.
			 */
			attach_handle = iommu_attach_handle_get(
					dev->iommu_group, IOMMU_NO_PASID,
					IOMMU_DOMAIN_NESTED);
			if (IS_ERR(attach_handle))
				return NULL;
		}
	} else {
		attach_handle = iommu_attach_handle_get(dev->iommu_group,
				IOMMU_NO_PASID, 0);

		if (IS_ERR(attach_handle))
			return NULL;
	}

	if (!attach_handle->domain->iopf_handler)
		return NULL;

	return attach_handle;
}

static void iopf_error_response(struct device *dev, struct iopf_fault *evt)
{
	const struct iommu_ops *ops = dev_iommu_ops(dev);
	struct iommu_fault *fault = &evt->fault;
	struct iommu_page_response resp = {
		.pasid = fault->prm.pasid,
		.grpid = fault->prm.grpid,
		.code = IOMMU_PAGE_RESP_INVALID
	};

	ops->page_response(dev, evt, &resp);
}

/**
 * iommu_report_device_fault() - Report fault event to device driver
 * @dev: the device
 * @evt: fault event data
 *
 * Called by IOMMU drivers when a fault is detected, typically in a threaded IRQ
 * handler. If this function fails then ops->page_response() was called to
 * complete evt if required.
 *
 * This module doesn't handle PCI PASID Stop Marker; IOMMU drivers must discard
 * them before reporting faults. A PASID Stop Marker (LRW = 0b100) doesn't
 * expect a response. It may be generated when disabling a PASID (issuing a
 * PASID stop request) by some PCI devices.
 *
 * The PASID stop request is issued by the device driver before unbind(). Once
 * it completes, no page request is generated for this PASID anymore and
 * outstanding ones have been pushed to the IOMMU (as per PCIe 4.0r1.0 - 6.20.1
 * and 10.4.1.2 - Managing PASID TLP Prefix Usage). Some PCI devices will wait
 * for all outstanding page requests to come back with a response before
 * completing the PASID stop request. Others do not wait for page responses, and
 * instead issue this Stop Marker that tells us when the PASID can be
 * reallocated.
 *
 * It is safe to discard the Stop Marker because it is an optimization.
 * a. Page requests, which are posted requests, have been flushed to the IOMMU
 *    when the stop request completes.
 * b. The IOMMU driver flushes all fault queues on unbind() before freeing the
 *    PASID.
 *
 * So even though the Stop Marker might be issued by the device *after* the stop
 * request completes, outstanding faults will have been dealt with by the time
 * the PASID is freed.
 *
 * Any valid page fault will be eventually routed to an iommu domain and the
 * page fault handler installed there will get called. The users of this
 * handling framework should guarantee that the iommu domain could only be
 * freed after the device has stopped generating page faults (or the iommu
 * hardware has been set to block the page faults) and the pending page faults
 * have been flushed. In case no page fault handler is attached or no iopf params
 * are setup, then the ops->page_response() is called to complete the evt.
 *
 * Returns 0 on success, or an error in case of a bad/failed iopf setup.
 */
int iommu_report_device_fault(struct device *dev, struct iopf_fault *evt)
{
	struct iommu_attach_handle *attach_handle;
	struct iommu_fault *fault = &evt->fault;
	struct iommu_fault_param *iopf_param;
	struct iopf_group abort_group = {};
	struct iopf_group *group;

	attach_handle = find_fault_handler(dev, evt);
	if (!attach_handle)
		goto err_bad_iopf;

	/*
	 * Something has gone wrong if a fault capable domain is attached but no
	 * iopf_param is setup
	 */
	iopf_param = iopf_get_dev_fault_param(dev);
	if (WARN_ON(!iopf_param))
		goto err_bad_iopf;

	if (!(fault->prm.flags & IOMMU_FAULT_PAGE_REQUEST_LAST_PAGE)) {
		int ret;

		ret = report_partial_fault(iopf_param, fault);
		iopf_put_dev_fault_param(iopf_param);
		/* A request that is not the last does not need to be ack'd */

		return ret;
	}

	/*
	 * This is the last page fault of a group. Allocate an iopf group and
	 * pass it to domain's page fault handler. The group holds a reference
	 * count of the fault parameter. It will be released after response or
	 * error path of this function. If an error is returned, the caller
	 * will send a response to the hardware. We need to clean up before
	 * leaving, otherwise partial faults will be stuck.
	 */
	group = iopf_group_alloc(iopf_param, evt, &abort_group);
	if (group == &abort_group)
		goto err_abort;

	group->attach_handle = attach_handle;

	/*
	 * On success iopf_handler must call iopf_group_response() and
	 * iopf_free_group()
	 */
	if (group->attach_handle->domain->iopf_handler(group))
		goto err_abort;

	return 0;

err_abort:
	dev_warn_ratelimited(dev, "iopf with pasid %d aborted\n",
			     fault->prm.pasid);
	iopf_group_response(group, IOMMU_PAGE_RESP_FAILURE);
	if (group == &abort_group)
		__iopf_free_group(group);
	else
		iopf_free_group(group);

	return 0;

err_bad_iopf:
	if (fault->type == IOMMU_FAULT_PAGE_REQ)
		iopf_error_response(dev, evt);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(iommu_report_device_fault);

/**
 * iopf_queue_flush_dev - Ensure that all queued faults have been processed
 * @dev: the endpoint whose faults need to be flushed.
 *
 * The IOMMU driver calls this before releasing a PASID, to ensure that all
 * pending faults for this PASID have been handled, and won't hit the address
 * space of the next process that uses this PASID. The driver must make sure
 * that no new fault is added to the queue. In particular it must flush its
 * low-level queue before calling this function.
 *
 * Return: 0 on success and <0 on error.
 */
int iopf_queue_flush_dev(struct device *dev)
{
	struct iommu_fault_param *iopf_param;

	/*
	 * It's a driver bug to be here after iopf_queue_remove_device().
	 * Therefore, it's safe to dereference the fault parameter without
	 * holding the lock.
	 */
	iopf_param = rcu_dereference_check(dev->iommu->fault_param, true);
	if (WARN_ON(!iopf_param))
		return -ENODEV;

	flush_workqueue(iopf_param->queue->wq);

	return 0;
}
EXPORT_SYMBOL_GPL(iopf_queue_flush_dev);

/**
 * iopf_group_response - Respond a group of page faults
 * @group: the group of faults with the same group id
 * @status: the response code
 */
void iopf_group_response(struct iopf_group *group,
			 enum iommu_page_response_code status)
{
	struct iommu_fault_param *fault_param = group->fault_param;
	struct iopf_fault *iopf = &group->last_fault;
	struct device *dev = group->fault_param->dev;
	const struct iommu_ops *ops = dev_iommu_ops(dev);
	struct iommu_page_response resp = {
		.pasid = iopf->fault.prm.pasid,
		.grpid = iopf->fault.prm.grpid,
		.code = status,
	};

	/* Only send response if there is a fault report pending */
	mutex_lock(&fault_param->lock);
	if (!list_empty(&group->pending_node)) {
		ops->page_response(dev, &group->last_fault, &resp);
		list_del_init(&group->pending_node);
	}
	mutex_unlock(&fault_param->lock);
}
EXPORT_SYMBOL_GPL(iopf_group_response);

/**
 * iopf_queue_discard_partial - Remove all pending partial fault
 * @queue: the queue whose partial faults need to be discarded
 *
 * When the hardware queue overflows, last page faults in a group may have been
 * lost and the IOMMU driver calls this to discard all partial faults. The
 * driver shouldn't be adding new faults to this queue concurrently.
 *
 * Return: 0 on success and <0 on error.
 */
int iopf_queue_discard_partial(struct iopf_queue *queue)
{
	struct iopf_fault *iopf, *next;
	struct iommu_fault_param *iopf_param;

	if (!queue)
		return -EINVAL;

	mutex_lock(&queue->lock);
	list_for_each_entry(iopf_param, &queue->devices, queue_list) {
		mutex_lock(&iopf_param->lock);
		list_for_each_entry_safe(iopf, next, &iopf_param->partial,
					 list) {
			list_del(&iopf->list);
			kfree(iopf);
		}
		mutex_unlock(&iopf_param->lock);
	}
	mutex_unlock(&queue->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(iopf_queue_discard_partial);

/**
 * iopf_queue_add_device - Add producer to the fault queue
 * @queue: IOPF queue
 * @dev: device to add
 *
 * Return: 0 on success and <0 on error.
 */
int iopf_queue_add_device(struct iopf_queue *queue, struct device *dev)
{
	int ret = 0;
	struct dev_iommu *param = dev->iommu;
	struct iommu_fault_param *fault_param;
	const struct iommu_ops *ops = dev_iommu_ops(dev);

	if (!ops->page_response)
		return -ENODEV;

	mutex_lock(&queue->lock);
	mutex_lock(&param->lock);
	if (rcu_dereference_check(param->fault_param,
				  lockdep_is_held(&param->lock))) {
		ret = -EBUSY;
		goto done_unlock;
	}

	fault_param = kzalloc(sizeof(*fault_param), GFP_KERNEL);
	if (!fault_param) {
		ret = -ENOMEM;
		goto done_unlock;
	}

	mutex_init(&fault_param->lock);
	INIT_LIST_HEAD(&fault_param->faults);
	INIT_LIST_HEAD(&fault_param->partial);
	fault_param->dev = dev;
	refcount_set(&fault_param->users, 1);
	list_add(&fault_param->queue_list, &queue->devices);
	fault_param->queue = queue;

	rcu_assign_pointer(param->fault_param, fault_param);

done_unlock:
	mutex_unlock(&param->lock);
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iopf_queue_add_device);

/**
 * iopf_queue_remove_device - Remove producer from fault queue
 * @queue: IOPF queue
 * @dev: device to remove
 *
 * Removing a device from an iopf_queue. It's recommended to follow these
 * steps when removing a device:
 *
 * - Disable new PRI reception: Turn off PRI generation in the IOMMU hardware
 *   and flush any hardware page request queues. This should be done before
 *   calling into this helper.
 * - Acknowledge all outstanding PRQs to the device: Respond to all outstanding
 *   page requests with IOMMU_PAGE_RESP_INVALID, indicating the device should
 *   not retry. This helper function handles this.
 * - Disable PRI on the device: After calling this helper, the caller could
 *   then disable PRI on the device.
 *
 * Calling iopf_queue_remove_device() essentially disassociates the device.
 * The fault_param might still exist, but iommu_page_response() will do
 * nothing. The device fault parameter reference count has been properly
 * passed from iommu_report_device_fault() to the fault handling work, and
 * will eventually be released after iommu_page_response().
 */
void iopf_queue_remove_device(struct iopf_queue *queue, struct device *dev)
{
	struct iopf_fault *partial_iopf;
	struct iopf_fault *next;
	struct iopf_group *group, *temp;
	struct dev_iommu *param = dev->iommu;
	struct iommu_fault_param *fault_param;
	const struct iommu_ops *ops = dev_iommu_ops(dev);

	mutex_lock(&queue->lock);
	mutex_lock(&param->lock);
	fault_param = rcu_dereference_check(param->fault_param,
					    lockdep_is_held(&param->lock));

	if (WARN_ON(!fault_param || fault_param->queue != queue))
		goto unlock;

	mutex_lock(&fault_param->lock);
	list_for_each_entry_safe(partial_iopf, next, &fault_param->partial, list)
		kfree(partial_iopf);

	list_for_each_entry_safe(group, temp, &fault_param->faults, pending_node) {
		struct iopf_fault *iopf = &group->last_fault;
		struct iommu_page_response resp = {
			.pasid = iopf->fault.prm.pasid,
			.grpid = iopf->fault.prm.grpid,
			.code = IOMMU_PAGE_RESP_INVALID
		};

		ops->page_response(dev, iopf, &resp);
		list_del_init(&group->pending_node);
	}
	mutex_unlock(&fault_param->lock);

	list_del(&fault_param->queue_list);

	/* dec the ref owned by iopf_queue_add_device() */
	rcu_assign_pointer(param->fault_param, NULL);
	iopf_put_dev_fault_param(fault_param);
unlock:
	mutex_unlock(&param->lock);
	mutex_unlock(&queue->lock);
}
EXPORT_SYMBOL_GPL(iopf_queue_remove_device);

/**
 * iopf_queue_alloc - Allocate and initialize a fault queue
 * @name: a unique string identifying the queue (for workqueue)
 *
 * Return: the queue on success and NULL on error.
 */
struct iopf_queue *iopf_queue_alloc(const char *name)
{
	struct iopf_queue *queue;

	queue = kzalloc(sizeof(*queue), GFP_KERNEL);
	if (!queue)
		return NULL;

	/*
	 * The WQ is unordered because the low-level handler enqueues faults by
	 * group. PRI requests within a group have to be ordered, but once
	 * that's dealt with, the high-level function can handle groups out of
	 * order.
	 */
	queue->wq = alloc_workqueue("iopf_queue/%s", WQ_UNBOUND, 0, name);
	if (!queue->wq) {
		kfree(queue);
		return NULL;
	}

	INIT_LIST_HEAD(&queue->devices);
	mutex_init(&queue->lock);

	return queue;
}
EXPORT_SYMBOL_GPL(iopf_queue_alloc);

/**
 * iopf_queue_free - Free IOPF queue
 * @queue: queue to free
 *
 * Counterpart to iopf_queue_alloc(). The driver must not be queuing faults or
 * adding/removing devices on this queue anymore.
 */
void iopf_queue_free(struct iopf_queue *queue)
{
	struct iommu_fault_param *iopf_param, *next;

	if (!queue)
		return;

	list_for_each_entry_safe(iopf_param, next, &queue->devices, queue_list)
		iopf_queue_remove_device(queue, iopf_param->dev);

	destroy_workqueue(queue->wq);
	kfree(queue);
}
EXPORT_SYMBOL_GPL(iopf_queue_free);
