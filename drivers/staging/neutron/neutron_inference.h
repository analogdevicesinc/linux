/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2023 NXP
 */

#ifndef NEUTRON_INFERENCE_H
#define NEUTRON_INFERENCE_H

/****************************************************************************
 * Includes
 ****************************************************************************/

#include "uapi/neutron.h"

#include <linux/kref.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/spinlock.h>

/****************************************************************************
 * Types
 ****************************************************************************/

struct neutron_device;
struct neutron_uapi_inference_args;
struct file;

enum neutron_cmd_type {
	NEUTRON_CMD_RESET_STATUS,
	NEUTRON_CMD_LOAD_KERNEL,
	NEUTRON_CMD_RUN_INFERENCE,
	NEUTRON_CMD_CLEAR_LOG,
	NEUTRON_CMD_TYPE_MAX,
};

/**
 * struct neutron_inference - Inference struct
 * @ndev:			Neutron device
 * @file:			File handle
 * @kref:			Reference counter
 * @waitq:			Wait queue, to wakeup poll thread in userspace
 * @status:			Inference status
 * @node:			Inference node in queue
 * @inf_arg:			Inference arguments
 * @poll_mode:			Whether use polling mode to read inference result
 * @poll_timer:			Poll timer to check inference status
 */
struct neutron_inference {
	struct neutron_device    *ndev;
	struct file              *file;
	struct kref              kref;
	wait_queue_head_t        waitq;
	enum neutron_uapi_status status;
	struct list_head         node;
	bool                     poll_mode;
	struct hrtimer           poll_timer;
	unsigned int             poll_count;
	enum   neutron_cmd_type  cmd_type;
	struct neutron_uapi_inference_args  args;
};

/**
 * struct neutron_inference_queue - Inference queue
 * @ndev:			Neutron device
 * @head:			List header
 * @queue_count:		Inference queue element count
 * @cur_inf:			Point to current inference instance
 * @wq:				Inference singlethread workqueue
 * @lock:			A spin_lock to protect list data
 */

struct neutron_inference_queue {
	struct neutron_device    *ndev;
	/* inference queue element count */
	unsigned int             queue_count;
	/* inference list head */
	struct list_head         head;
	/* current inference job */
	struct neutron_inference *cur_inf;
	/* singlethread_workqueue */
	struct workqueue_struct  *wq;
	struct work_struct       work;
	/* protects the list add/modify/delete */
	spinlock_t               lock;
};

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
 * neutron_inference_queue() - Create and initialize inference queue list
 *
 * Return: Pointer on success, else ERR_PTR.
 */
struct neutron_inference_queue *neutron_queue_create(struct neutron_device *ndev);

/**
 * neutron_queue_destroy() - Destroy neutron queue list
 */
void neutron_queue_destroy(struct neutron_inference_queue *queue);

/**
 * neutron_inference_create() - Create inference job
 *
 * This function must be called in the context of a user space process.
 *
 * Return: fd on success, else error code.
 */
int neutron_inference_create(struct neutron_device *edev, enum neutron_cmd_type type,
			     struct neutron_uapi_inference_args *uapi);

/**
 * neutron_inference_done() - Call inference done callback function.
 *
 * Queue workqueue to call inference done callback.
 *
 * Return: 0 on success, else ERR.
 */
int neutron_inference_done(struct neutron_device *ndev);

#endif /* NEUTRON_INFERENCE_H */
