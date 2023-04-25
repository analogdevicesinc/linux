// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */

/* @pfe_cdev.c.
 *  Dummy device representing the PFE US in userspace.
 *  - used for interacting with the kernel layer for link status
 */

#include <linux/eventfd.h>
#include <linux/irqreturn.h>
#include <linux/io.h>
#include <asm/irq.h>

#include "pfe_cdev.h"
#include "pfe_mod.h"

static int pfe_majno;
static struct class *pfe_char_class;
static struct device *pfe_char_dev;
struct eventfd_ctx *g_trigger;

struct pfe_shared_info link_states[PFE_CDEV_ETH_COUNT];

static int pfe_cdev_open(struct inode *inp, struct file *fp)
{
	pr_debug("PFE CDEV device opened.\n");
	return 0;
}

static ssize_t pfe_cdev_read(struct file *fp, char *buf,
			     size_t len, loff_t *off)
{
	int ret = 0;

	pr_info("PFE CDEV attempt copying (%lu) size of user.\n",
		sizeof(link_states));

	pr_debug("Dump link_state on screen before copy_to_user\n");
	for (; ret < PFE_CDEV_ETH_COUNT; ret++) {
		pr_debug("%u  %u", link_states[ret].phy_id,
			 link_states[ret].state);
		pr_debug("\n");
	}

	/* Copy to user the value in buffer sized len */
	ret = copy_to_user(buf, &link_states, sizeof(link_states));
	if (ret != 0) {
		pr_err("Failed to send (%d)bytes of (%lu) requested.\n",
		       ret, len);
		return -EFAULT;
	}

	/* offset set back to 0 as there is contextual reading offset */
	*off = 0;
	pr_debug("Read of (%lu) bytes performed.\n", sizeof(link_states));

	return sizeof(link_states);
}

/**
 * This function is for getting some commands from user through non-IOCTL
 * channel. It can used to configure the device.
 * TODO: To be filled in future, if require duplex communication with user
 * space.
 */
static ssize_t pfe_cdev_write(struct file *fp, const char *buf,
			      size_t len, loff_t *off)
{
	pr_info("PFE CDEV Write operation not supported!\n");

	return -EFAULT;
}

static int pfe_cdev_release(struct inode *inp, struct file *fp)
{
	if (g_trigger) {
		free_irq(pfe->hif_irq, g_trigger);
		eventfd_ctx_put(g_trigger);
		g_trigger = NULL;
	}

	pr_info("PFE_CDEV: Device successfully closed\n");
	return 0;
}

/*
 * hif_us_isr-
 * This ISR routine processes Rx/Tx done interrupts from the HIF hardware block
 */
static irqreturn_t hif_us_isr(int irq, void *arg)
{
	struct eventfd_ctx *trigger = (struct eventfd_ctx *)arg;
	int int_status;
	int int_enable_mask;

	/*Read hif interrupt source register */
	int_status = readl_relaxed(HIF_INT_SRC);
	int_enable_mask = readl_relaxed(HIF_INT_ENABLE);

	if ((int_status & HIF_INT) == 0)
		return IRQ_NONE;

	if (int_status & HIF_RXPKT_INT) {
		int_enable_mask &= ~(HIF_RXPKT_INT);
		/* Disable interrupts, they will be enabled after
		 * they are serviced
		 */
		writel_relaxed(int_enable_mask, HIF_INT_ENABLE);

		eventfd_signal(trigger, 1);
	}

	return IRQ_HANDLED;
}

#define PFE_INTR_COAL_USECS	100
static long pfe_cdev_ioctl(struct file *fp, unsigned int cmd,
			   unsigned long arg)
{
	int ret = -EFAULT;
	int __user *argp = (int __user *)arg;

	pr_debug("PFE CDEV IOCTL Called with cmd=(%u)\n", cmd);

	switch (cmd) {
	case PFE_CDEV_ETH0_STATE_GET:
		/* Return an unsigned int (link state) for ETH0 */
		*argp = link_states[0].state;
		pr_debug("Returning state=%d for ETH0\n", *argp);
		ret = 0;
		break;
	case PFE_CDEV_ETH1_STATE_GET:
		/* Return an unsigned int (link state) for ETH0 */
		*argp = link_states[1].state;
		pr_debug("Returning state=%d for ETH1\n", *argp);
		ret = 0;
		break;
	case PFE_CDEV_HIF_INTR_EN:
		/* Return success/failure */
		g_trigger = eventfd_ctx_fdget(*argp);
		if (IS_ERR(g_trigger))
			return PTR_ERR(g_trigger);
		ret = request_irq(pfe->hif_irq, hif_us_isr, 0, "pfe_hif",
				  g_trigger);
		if (ret) {
			pr_err("%s: failed to get the hif IRQ = %d\n",
			       __func__, pfe->hif_irq);
			eventfd_ctx_put(g_trigger);
			g_trigger = NULL;
		}
		writel((PFE_INTR_COAL_USECS * (pfe->ctrl.sys_clk / 1000)) |
			HIF_INT_COAL_ENABLE, HIF_INT_COAL);

		pr_debug("request_irq for hif interrupt: %d\n", pfe->hif_irq);
		ret = 0;
		break;
	default:
		pr_info("Unsupport cmd (%d) for PFE CDEV.\n", cmd);
		break;
	};

	return ret;
}

static unsigned int pfe_cdev_poll(struct file *fp,
				  struct poll_table_struct *wait)
{
	pr_info("PFE CDEV poll method not supported\n");
	return 0;
}

static const struct file_operations pfe_cdev_fops = {
	.open = pfe_cdev_open,
	.read = pfe_cdev_read,
	.write = pfe_cdev_write,
	.release = pfe_cdev_release,
	.unlocked_ioctl = pfe_cdev_ioctl,
	.poll = pfe_cdev_poll,
};

int pfe_cdev_init(void)
{
	int ret;

	pr_debug("PFE CDEV initialization begin\n");

	/* Register the major number for the device */
	pfe_majno = register_chrdev(0, PFE_CDEV_NAME, &pfe_cdev_fops);
	if (pfe_majno < 0) {
		pr_err("Unable to register PFE CDEV. PFE CDEV not available\n");
		ret = pfe_majno;
		goto cleanup;
	}

	pr_debug("PFE CDEV assigned major number: %d\n", pfe_majno);

	/* Register the class for the device */
	pfe_char_class = class_create(PFE_CLASS_NAME);
	if (IS_ERR(pfe_char_class)) {
		pr_err(
		"Failed to init class for PFE CDEV. PFE CDEV not available.\n");
		ret = PTR_ERR(pfe_char_class);
		goto cleanup;
	}

	pr_debug("PFE CDEV Class created successfully.\n");

	/* Create the device without any parent and without any callback data */
	    pfe_char_dev = device_create(pfe_char_class, NULL,
					 MKDEV(pfe_majno, 0), NULL,
					 PFE_CDEV_NAME);
	if (IS_ERR(pfe_char_dev)) {
		pr_err("Unable to PFE CDEV device. PFE CDEV not available.\n");
		ret = PTR_ERR(pfe_char_dev);
		goto cleanup;
	}

	/* Information structure being shared with the userspace */
	memset(link_states, 0, sizeof(struct pfe_shared_info) *
			PFE_CDEV_ETH_COUNT);

	pr_info("PFE CDEV created: %s\n", PFE_CDEV_NAME);

	ret = 0;
	return ret;

cleanup:
	if (!IS_ERR(pfe_char_class))
		class_destroy(pfe_char_class);

	if (pfe_majno > 0)
		unregister_chrdev(pfe_majno, PFE_CDEV_NAME);

	return ret;
}

void pfe_cdev_exit(void)
{
	if (!IS_ERR(pfe_char_dev))
		device_destroy(pfe_char_class, MKDEV(pfe_majno, 0));

	if (!IS_ERR(pfe_char_class)) {
		class_unregister(pfe_char_class);
		class_destroy(pfe_char_class);
	}

	if (pfe_majno > 0)
		unregister_chrdev(pfe_majno, PFE_CDEV_NAME);

	/* reset the variables */
	pfe_majno = 0;
	pfe_char_class = NULL;
	pfe_char_dev = NULL;

	pr_info("PFE CDEV Removed.\n");
}
