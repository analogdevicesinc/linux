// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */

/****************************************************************************
 * Includes
 ****************************************************************************/

#include <linux/dma-mapping.h>
#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>

#include "neutron_inference.h"
#include "neutron_buffer.h"
#include "neutron_device.h"
#include "neutron_mailbox.h"
#include "uapi/neutron.h"

/****************************************************************************
 * Variables
 ****************************************************************************/
static void inference_done_callback(struct work_struct *work);

static void neutron_inference_get(struct neutron_inference *inf);

static int neutron_inference_put(struct neutron_inference *inf);

static int neutron_inference_release(struct inode *inode,
				     struct file *file);

static unsigned int neutron_inference_poll(struct file *file,
					   poll_table *wait);

static long neutron_inference_ioctl(struct file *file,
				    unsigned int cmd,
				    unsigned long arg);

static const struct file_operations neutron_inference_fops = {
	.release        = &neutron_inference_release,
	.poll           = &neutron_inference_poll,
	.unlocked_ioctl	= &neutron_inference_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= &neutron_inference_ioctl,
#endif
};

/****************************************************************************
 * Functions
 ****************************************************************************/

static inline void neutron_inference_del_list(struct neutron_inference_queue *queue,
					      struct neutron_inference *inf)
{
	queue->queue_count--;
	list_del(&inf->node);
}

static enum hrtimer_restart poll_result_callback(struct hrtimer *poll_timer)
{
	struct neutron_inference *inf =
		container_of(poll_timer, typeof(*inf), poll_timer);
	struct neutron_mbox *mbox;
	u32 val;

	if (!inf || IS_ERR(inf) || !kref_read(&inf->kref))
		return HRTIMER_NORESTART;

	mbox = inf->ndev->mbox;

	val = mbox->ops->read_ret(mbox);
	if (val == DONE) {
		neutron_inference_done(inf->ndev);
		return HRTIMER_NORESTART;
	}
	/* Reload timer */
	hrtimer_forward_now(poll_timer, ns_to_ktime(100 * 1000));
	return HRTIMER_RESTART;
}

static int neutron_inference_run(struct neutron_inference *inf)
{
	struct neutron_device *ndev;
	struct neutron_mbox_tx_msg msg;
	u32 i, val;
	int ret = 0;

	if (!inf || IS_ERR(inf))
		return PTR_ERR(inf);

	if (!inf->ndev || IS_ERR(inf->ndev))
		return PTR_ERR(inf);

	/* The job have been done */
	if (inf->status == NEUTRON_UAPI_STATUS_DONE)
		return 0;

	ndev = inf->ndev;

	/* Sync the input data for device before running inference job */
	neutron_memory_sync(ndev, inf->buf->dma_addr + inf->args.input_offset,
			    inf->args.input_size, DMA_TO_DEVICE);

	// reload only when firmware was changed
	if (ndev->firmw_id  != inf->args.firmw_id) {
		mutex_lock(&ndev->mutex);
		ndev->firmw_id = inf->args.firmw_id;
		neutron_firmw_reload(ndev, inf->buf);
		mutex_unlock(&ndev->mutex);
		dev_dbg(ndev->dev, "Inference firmw_reload: %x\n", inf->args.firmw_id);
	}

	spin_lock_bh(&ndev->queue->lock);
	ndev->queue->cur_inf = inf;
	inf->status = NEUTRON_UAPI_STATUS_RUNNING;
	spin_unlock_bh(&ndev->queue->lock);

	ndev = inf->ndev;

	neu_dbg("job %x is started, base_ddr %llx\n",
		inf->args.tensor_offset, (__u64)inf->args.base_ddr_h << 32 | inf->args.base_ddr_l);

	/* Do reset when neutron is stuck.
	 * If the previous inference job is done, the ACK register will be set to RESET_VAL.
	 * Otherwise it means neutron is stuck.
	 */
	val = ndev->mbox->ops->read_ret(ndev->mbox);
	if (unlikely(val != RESET_VAL)) {
		dev_dbg(ndev->dev, "reset neutron: 0x%x\n", val);
		mutex_lock(&ndev->mutex);
		neutron_hw_reset(ndev);
		neutron_firmw_reload(ndev, inf->buf);
		mutex_unlock(&ndev->mutex);
	}

	/* set BASEDDR address */
	writel(inf->args.base_ddr_l, inf->ndev->reg_base + BASEDDRL);
	writel(inf->args.base_ddr_l, inf->ndev->reg_base + BASEINOUTL);
	writel(inf->args.base_ddr_l, inf->ndev->reg_base + BASESPILLL);

	writel(inf->args.base_ddr_h, inf->ndev->reg_base + BASEDDRH);
	writel(inf->args.base_ddr_h, inf->ndev->reg_base + BASEINOUTH);
	writel(inf->args.base_ddr_h, inf->ndev->reg_base + BASESPILLH);

	/* Run neutron inference */
	if (inf->cmd_type == NEUTRON_CMD_RUN_INFERENCE) {
		neu_dbg("run inference\n");
		msg.command = RUN;
		msg.args[0] = inf->args.tensor_offset;
		msg.args[1] = inf->args.microcode_offset;
		msg.args[2] = inf->args.tensor_count;
		msg.argc = 3;

	/* Load neutron kernel binary */
	} else if (inf->cmd_type == NEUTRON_CMD_LOAD_KERNEL) {
		neu_dbg("load kernel\n");
		msg.command = KERNELS;
		msg.args[0] = inf->args.kernel_offset;
		msg.argc = 1;
	/* Clear log */
	} else if (inf->cmd_type == NEUTRON_CMD_CLEAR_LOG) {
		msg.command = CLEAR_FW_LOG;
		msg.argc = 0;
	} else {
		inf->status = NEUTRON_UAPI_STATUS_ERROR;
		ret = -EINVAL;
		dev_err(ndev->dev, "unkonw inference type: %d\n", inf->cmd_type);
		goto inf_stop_early;
	}

	mutex_lock(&ndev->mutex);
	for (int i = 0; i < 3; i++) {
		ret = ndev->mbox->ops->send_data(ndev->mbox, &msg);
		if (ret == 0)
			break;
		neutron_hw_reset(ndev);
		neutron_firmw_reload(ndev, inf->buf);
		msleep(5);
	}
	mutex_unlock(&ndev->mutex);

	if (ret < 0) {
		inf->status = NEUTRON_UAPI_STATUS_ERROR;
		dev_err(ndev->dev, "failed to send mbox_message\n");
		goto inf_stop_early;
	}

	if (inf->poll_mode) {
		/* Loop state before setting timer to get result early */
		for (i = 0; i < 100; i++) {
			val = ndev->mbox->ops->read_ret(ndev->mbox);
			/* Call inference_done_callback if it is done status */
			if (val == DONE)
				goto inf_stop_early;
			usleep_range(10, 20);
		}
		/* Start timer to continue poll status if it's not done yet */
		hrtimer_start(&inf->poll_timer, ns_to_ktime(100 * 1000), HRTIMER_MODE_REL);
	};

	return 0;

inf_stop_early:
	neutron_inference_done(ndev);
	return ret;
}

int neutron_inference_done(struct neutron_device *ndev)
{
	if (ndev->queue->wq)
		queue_work(ndev->queue->wq, &ndev->queue->work);

	return 0;
}

static void inference_inqueue(struct neutron_inference_queue *queue,
			      struct neutron_inference *inf)
{
	int queue_count;

	spin_lock_bh(&queue->lock);
	queue_count = queue->queue_count++;
	list_add_tail(&inf->node, &queue->head);
	inf->status = NEUTRON_UAPI_STATUS_PENDING;
	spin_unlock_bh(&queue->lock);

	/* Start inference directly if there are no jobs in queue */
	if (queue_count == 0)
		neutron_inference_run(inf);
}

static struct neutron_inference *inference_dequeue(struct neutron_inference_queue *queue,
						   struct neutron_inference *inf)
{
	struct neutron_inference *next_inf;
	int queue_count;

	spin_lock_bh(&queue->lock);
	/* Get next element before delete it */
	next_inf = list_next_entry(inf, node);
	neutron_inference_del_list(queue, inf);
	queue->cur_inf = NULL;
	queue_count = queue->queue_count;
	spin_unlock_bh(&queue->lock);

	/* There are jobs left in queue list */
	if (queue_count > 0) {
		neu_dbg("next %x\n", next_inf->args.tensor_offset);
		return next_inf;
	}
	return NULL;
}

static void inference_done_callback(struct work_struct *work)
{
	struct neutron_inference_queue *queue =
		container_of(work, struct neutron_inference_queue, work);

	struct neutron_inference *inf, *next_inf;
	struct neutron_device *ndev;
	struct neutron_mbox *mbox;

	spin_lock_bh(&queue->lock);
	inf = queue->cur_inf;
	spin_unlock_bh(&queue->lock);

	if (!inf || IS_ERR(inf))
		return;

	neutron_inference_get(inf);

	/* Update inference job from running to done */
	if (inf->status == NEUTRON_UAPI_STATUS_RUNNING)
		inf->status = NEUTRON_UAPI_STATUS_DONE;

	ndev = inf->ndev;
	mbox = ndev->mbox;

	/* Sync the output data for cpu after inference is done */
	neutron_memory_sync(ndev, inf->buf->dma_addr + inf->args.output_offset,
			    inf->args.output_size, DMA_FROM_DEVICE);

	/* Wake up the waiting process */
	wake_up_interruptible(&inf->waitq);

	/* Reset neutron */
	if (mbox->ops->send_reset(ndev->mbox))
		dev_warn(ndev->dev, "failed to reset neutron state\n");

	dev_dbg(ndev->dev, "inf %x is done\n", inf->args.tensor_offset);

	/* Only dequeue a new job when the previous one is completed,
	 * allowing only 1 job to run at a time.
	 */
	next_inf = inference_dequeue(ndev->queue, inf);
	neutron_inference_put(inf);

	neutron_inference_run(next_inf);
}

static void neutron_inference_kref_destroy(struct kref *kref)
{
	struct neutron_inference *inf =
		container_of(kref, struct neutron_inference, kref);

	struct neutron_inference *del_inf, *_del_inf;

	dev_dbg(inf->ndev->dev,
		"inference %x destroy status: %x\n",
		inf->args.tensor_offset, inf->status);

	/* Delete entry if inference is not scheduled */
	spin_lock_bh(&inf->ndev->queue->lock);
	list_for_each_entry_safe(del_inf, _del_inf, &inf->ndev->queue->head, node) {
		if (del_inf == inf) {
			neutron_inference_del_list(inf->ndev->queue, inf);
			break;
		}
	}
	spin_unlock_bh(&inf->ndev->queue->lock);

	if (inf->poll_mode)
		hrtimer_cancel(&inf->poll_timer);

	devm_kfree(inf->ndev->dev, inf);
}

static int neutron_inference_release(struct inode *inode,
				     struct file *file)
{
	struct neutron_inference *inf = file->private_data;

	dev_dbg(inf->ndev->dev,
		"Inference release. file=0x%pK, inf=0x%pK",
		file, inf);

	neutron_inference_put(inf);

	return 0;
}

static unsigned int neutron_inference_poll(struct file *file,
					   poll_table *wait)
{
	struct neutron_inference *inf = file->private_data;
	int ret = 0;

	poll_wait(file, &inf->waitq, wait);
	if (inf->status == NEUTRON_UAPI_STATUS_DONE)
		ret |= POLLIN;
	else if (inf->status == NEUTRON_UAPI_STATUS_ERROR)
		ret |= POLLERR;

	return ret;
}

static long neutron_inference_ioctl(struct file *file,
				    unsigned int cmd,
				    unsigned long arg)
{
	struct neutron_inference *inf = file->private_data;
	void __user *udata = (void __user *)arg;
	struct neutron_device *ndev;
	int ret = -EINVAL;
	struct neutron_uapi_result_status uapi;
	struct neutron_mbox_rx_msg rx_msg;

	if (!inf || IS_ERR(inf))
		return ret;

	ndev = inf->ndev;

	dev_dbg(ndev->dev, "Device ioctl. file=0x%pK, cmd=0x%x, arg=0x%lx\n",
		file, cmd, arg);

	switch (cmd) {
	case NEUTRON_IOCTL_INFERENCE_STATE:
		uapi.status = inf->status;
		uapi.error_code = 0;

		/* Read firmware return code if the job is done */
		if (inf->status == NEUTRON_UAPI_STATUS_DONE) {
			ndev->mbox->ops->recv_data(ndev->mbox, &rx_msg);
			uapi.error_code = rx_msg.args[0];
		}

		if (copy_to_user(udata, &uapi, sizeof(uapi)))
			break;
		ret = 0;
		break;

	default:
		dev_err(ndev->dev, "Invalid ioctl. cmd=%u, arg=%lu",
			cmd, arg);
		break;
	}
	return ret;
}

int neutron_inference_create(struct neutron_device *ndev, enum neutron_cmd_type type,
			     struct neutron_uapi_inference_args *uapi)
{
	struct neutron_inference *inf;
	int ret = -ENOMEM;

	inf = devm_kzalloc(ndev->dev, sizeof(*inf), GFP_KERNEL);
	if (!inf)
		return -ENOMEM;

	inf->ndev = ndev;
	inf->cmd_type = type;
	/* use polling mode */
	inf->poll_mode = true;

	kref_init(&inf->kref);
	init_waitqueue_head(&inf->waitq);

	if (inf->poll_mode) {
		hrtimer_init(&inf->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		inf->poll_timer.function = &poll_result_callback;
	}

	memcpy(&inf->args, uapi, sizeof(struct neutron_uapi_inference_args));

	/* Create file descriptor */
	ret = anon_inode_getfd("neutron-inference", &neutron_inference_fops,
			       inf, O_RDWR | O_CLOEXEC);
	if (ret < 0)
		goto kfree_inference;

	inf->buf = neutron_buffer_get_from_fd(inf->args.buf_id);

	inference_inqueue(ndev->queue, inf);

	/* Store pointer to file structure */
	inf->file = fget(ret);
	fput(inf->file);

	dev_dbg(ndev->dev, "inference %x created, type 0x%x\n",
		uapi->tensor_offset, type);

	return ret;

kfree_inference:
	devm_kfree(ndev->dev, inf);

	return ret;
}

static void neutron_inference_get(struct neutron_inference *inf)
{
	kref_get(&inf->kref);
}

static int neutron_inference_put(struct neutron_inference *inf)
{
	return kref_put(&inf->kref, &neutron_inference_kref_destroy);
}

struct neutron_inference_queue *neutron_queue_create(struct neutron_device *ndev)
{
	struct neutron_inference_queue *queue;

	queue = devm_kzalloc(ndev->dev, sizeof(*queue), GFP_KERNEL);
	if (!queue)
		return NULL;

	queue->queue_count = 0;
	queue->ndev = ndev;
	spin_lock_init(&queue->lock);

	queue->wq = create_singlethread_workqueue("neutron_workqueue");
	if (!queue->wq) {
		dev_err(ndev->dev, "Failed to create work queue\n");
		goto kfree;
	}

	INIT_LIST_HEAD(&queue->head);
	INIT_WORK(&queue->work, inference_done_callback);

	return queue;

kfree:
	devm_kfree(ndev->dev, queue);

	dev_err(ndev->dev, "Failed to create neutron queue\n");
	return NULL;
}

void neutron_queue_destroy(struct neutron_inference_queue *queue)
{
	if (queue) {
		destroy_workqueue(queue->wq);
		devm_kfree(queue->ndev->dev, queue);
	}
}

