/* Copyright (c) 2013 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* define a device that allows USPDAA processes to open a file
   descriptor and specify which IRQ it wants to montior using an ioctl()
   When an IRQ is received, the device becomes readable so that a process
   can use read() or select() type calls to monitor for IRQs */

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/fsl_usdpaa.h>
#include <linux/module.h>
#include <linux/fdtable.h>
#include <linux/file.h>

#include "qman_low.h"
#include "bman_low.h"

struct usdpaa_irq_ctx {
	int irq_set; /* Set to true once the irq is set via ioctl */
	unsigned int irq_num;
	u32 last_irq_count; /* Last value returned from read */
	u32 irq_count; /* Number of irqs since last read */
	wait_queue_head_t wait_queue; /* Waiting processes */
	spinlock_t lock;
	void *inhibit_addr; /* inhibit register address */
	struct file *usdpaa_filp;
	char irq_name[128];
};

static int usdpaa_irq_open(struct inode *inode, struct file *filp)
{
	struct usdpaa_irq_ctx *ctx = kmalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->irq_set = 0;
	ctx->irq_count = 0;
	ctx->last_irq_count = 0;
	init_waitqueue_head(&ctx->wait_queue);
	spin_lock_init(&ctx->lock);
	filp->private_data = ctx;
	return 0;
}

static int usdpaa_irq_release(struct inode *inode, struct file *filp)
{
	struct usdpaa_irq_ctx *ctx = filp->private_data;
	if (ctx->irq_set) {
		/* Inhibit the IRQ */
		out_be32(ctx->inhibit_addr, 0x1);
		irq_set_affinity_hint(ctx->irq_num, NULL);
		free_irq(ctx->irq_num, ctx);
		ctx->irq_set = 0;
		fput(ctx->usdpaa_filp);
	}
	kfree(filp->private_data);
	return 0;
}

static irqreturn_t usdpaa_irq_handler(int irq, void *_ctx)
{
	unsigned long flags;
	struct usdpaa_irq_ctx *ctx = _ctx;
	spin_lock_irqsave(&ctx->lock, flags);
	++ctx->irq_count;
	spin_unlock_irqrestore(&ctx->lock, flags);
	wake_up_all(&ctx->wait_queue);
	/* Set the inhibit register.  This will be reenabled
	   once the USDPAA code handles the IRQ */
	out_be32(ctx->inhibit_addr, 0x1);
	pr_debug("Inhibit at %p count %d", ctx->inhibit_addr, ctx->irq_count);
	return IRQ_HANDLED;
}

static int map_irq(struct file *fp, struct usdpaa_ioctl_irq_map *irq_map)
{
	struct usdpaa_irq_ctx *ctx = fp->private_data;
	int ret;

	if (ctx->irq_set) {
		pr_debug("Setting USDPAA IRQ when it was already set!\n");
		return -EBUSY;
	}

	ctx->usdpaa_filp = fget(irq_map->fd);
	if (!ctx->usdpaa_filp) {
		pr_debug("USDPAA fget(%d) returned NULL\n", irq_map->fd);
		return -EINVAL;
	}

	ret = usdpaa_get_portal_config(ctx->usdpaa_filp, irq_map->portal_cinh,
				       irq_map->type, &ctx->irq_num,
				       &ctx->inhibit_addr);
	if (ret) {
		pr_debug("USDPAA IRQ couldn't identify portal\n");
		fput(ctx->usdpaa_filp);
		return ret;
	}

	ctx->irq_set = 1;

	snprintf(ctx->irq_name, sizeof(ctx->irq_name),
		 "usdpaa_irq %d", ctx->irq_num);

	ret = request_irq(ctx->irq_num, usdpaa_irq_handler, 0,
			  ctx->irq_name, ctx);
	if (ret) {
		pr_err("USDPAA request_irq(%d) failed, ret= %d\n",
		       ctx->irq_num, ret);
		ctx->irq_set = 0;
		fput(ctx->usdpaa_filp);
		return ret;
	}
	ret = irq_set_affinity(ctx->irq_num, &current->cpus_allowed);
	if (ret)
		pr_err("USDPAA irq_set_affinity() failed, ret= %d\n", ret);

	ret = irq_set_affinity_hint(ctx->irq_num, &current->cpus_allowed);
	if (ret)
		pr_err("USDPAA irq_set_affinity_hint() failed, ret= %d\n", ret);

	return 0;
}

static long usdpaa_irq_ioctl(struct file *fp, unsigned int cmd,
			     unsigned long arg)
{
	int ret;
	struct usdpaa_ioctl_irq_map irq_map;

	if (cmd != USDPAA_IOCTL_PORTAL_IRQ_MAP) {
		pr_debug("USDPAA IRQ unknown command 0x%x\n", cmd);
		return -EINVAL;
	}

	ret = copy_from_user(&irq_map, (void __user *)arg,
			     sizeof(irq_map));
	if (ret)
		return ret;
	return map_irq(fp, &irq_map);
}

static ssize_t usdpaa_irq_read(struct file *filp, char __user *buff,
			       size_t count, loff_t *offp)
{
	struct usdpaa_irq_ctx *ctx = filp->private_data;
	int ret;

	if (!ctx->irq_set) {
		pr_debug("Reading USDPAA IRQ before it was set\n");
		return -EINVAL;
	}

	if (count < sizeof(ctx->irq_count)) {
		pr_debug("USDPAA IRQ Read too small\n");
		return -EINVAL;
	}
	if (ctx->irq_count == ctx->last_irq_count) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(ctx->wait_queue,
				       ctx->irq_count != ctx->last_irq_count);
		if (ret == -ERESTARTSYS)
			return ret;
	}

	ctx->last_irq_count = ctx->irq_count;

	if (copy_to_user(buff, &ctx->last_irq_count,
			 sizeof(ctx->last_irq_count)))
		return -EFAULT;
	return sizeof(ctx->irq_count);
}

static unsigned int usdpaa_irq_poll(struct file *filp, poll_table *wait)
{
	struct usdpaa_irq_ctx *ctx = filp->private_data;
	unsigned int ret = 0;
	unsigned long flags;

	if (!ctx->irq_set)
		return POLLHUP;

	poll_wait(filp, &ctx->wait_queue, wait);

	spin_lock_irqsave(&ctx->lock, flags);
	if (ctx->irq_count != ctx->last_irq_count)
		ret |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&ctx->lock, flags);
	return ret;
}

static long usdpaa_irq_ioctl_compat(struct file *fp, unsigned int cmd,
				unsigned long arg)
{
#ifdef CONFIG_COMPAT
	void __user *a = (void __user *)arg;
#endif
	switch (cmd) {
#ifdef CONFIG_COMPAT
	case  USDPAA_IOCTL_PORTAL_IRQ_MAP_COMPAT:
	{
		struct compat_ioctl_irq_map input;
		struct usdpaa_ioctl_irq_map converted;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		converted.type = input.type;
		converted.fd = input.fd;
		converted.portal_cinh = compat_ptr(input.portal_cinh);
		return map_irq(fp, &converted);
	}
#endif
	default:
		return usdpaa_irq_ioctl(fp, cmd, arg);
	}
}

static const struct file_operations usdpaa_irq_fops = {
	.open		   = usdpaa_irq_open,
	.release	   = usdpaa_irq_release,
	.unlocked_ioctl	   = usdpaa_irq_ioctl,
	.compat_ioctl	   = usdpaa_irq_ioctl_compat,
	.read              = usdpaa_irq_read,
	.poll              = usdpaa_irq_poll
};

static struct miscdevice usdpaa_miscdev = {
	.name = "fsl-usdpaa-irq",
	.fops = &usdpaa_irq_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

static int __init usdpaa_irq_init(void)
{
	int ret;

	pr_info("Freescale USDPAA process IRQ driver\n");
	ret = misc_register(&usdpaa_miscdev);
	if (ret)
		pr_err("fsl-usdpaa-irq: failed to register misc device\n");
	return ret;
}

static void __exit usdpaa_irq_exit(void)
{
	misc_deregister(&usdpaa_miscdev);
}

module_init(usdpaa_irq_init);
module_exit(usdpaa_irq_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Freescale USDPAA process IRQ driver");
