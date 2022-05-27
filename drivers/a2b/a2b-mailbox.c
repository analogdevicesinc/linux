// SPDX-License-Identifier: GPL-2.0-only

#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

static dev_t a2b_mailbox_devt;

struct a2b_mailbox {
	struct a2b_node *node;
	unsigned int word_size;

	struct delayed_work status_poll_work;

	unsigned long flags;

	struct device dev;
	struct cdev chrdev;
};

#define A2B_MAILBOX_BUSY BIT(0)

static ssize_t a2b_mailbox_chardev_read(struct file *filp, char __user *buf,
					size_t n, loff_t *f_ps)
{
	struct a2b_mailbox *mbox = filp->private_data;
	struct regmap *regmap = mbox->node->regmap;
	DEFINE_WAIT_FUNC(wait, woken_wake_function);
	unsigned int stat, val;
	int read = 0;
	int ret = 0;
	int i;

	n = rounddown(n, mbox->word_size);

	add_wait_queue(&mbox->node->mbox_pollq, &wait);
	do {
		if (!mbox->node) {
			ret = -ENODEV;
			break;
		}

		ret = regmap_read(regmap, A2B_MBOXSTAT(1), &stat);
		if (ret)
			break;

		if (!(stat & A2B_MBOXSTAT_FULL)) {
			if (filp->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				break;
			}

			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}

			wait_woken(&wait, TASK_INTERRUPTIBLE,
				   MAX_SCHEDULE_TIMEOUT);
			continue;
		}

		for (i = 0; i < mbox->word_size; i++) {
			ret = regmap_read(regmap, A2B_MBOXDATA(1, i), &val);
			if (ret)
				break;
			buf[read] = val;
			read++;
		}

	} while (read < n && ret == 0);
	remove_wait_queue(&mbox->node->mbox_pollq, &wait);

	return read ? read : ret;
}

ssize_t a2b_mailbox_chardev_write(struct file *filp, const char __user *buf,
				  size_t n, loff_t *f_ps)
{
	struct a2b_mailbox *mbox = filp->private_data;
	struct regmap *regmap = mbox->node->regmap;
	DEFINE_WAIT_FUNC(wait, woken_wake_function);
	unsigned int stat;
	int written = 0;
	int ret = 0;
	int i;

	n = rounddown(n, mbox->word_size);

	add_wait_queue(&mbox->node->mbox_pollq, &wait);
	do {
		if (!mbox->node) {
			written = -ENODEV;
			break;
		}

		ret = regmap_read(regmap, A2B_MBOXSTAT(0), &stat);
		if (ret)
			break;

		if (!(stat & A2B_MBOXSTAT_EMPTY)) {
			if (filp->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}

			wait_woken(&wait, TASK_INTERRUPTIBLE,
				   MAX_SCHEDULE_TIMEOUT);
			continue;
		}

		for (i = 0; i < mbox->word_size; i++) {
			ret = regmap_write(regmap, A2B_MBOXDATA(0, i),
					   buf[written]);
			if (ret)
				break;
			written++;
		}

	} while (written < n && ret == 0);
	remove_wait_queue(&mbox->node->mbox_pollq, &wait);

	return written ? written : ret;
}

static __poll_t a2b_mailbox_get_poll_flags(struct a2b_mailbox *mbox)
{
	struct regmap *regmap = mbox->node->regmap;
	unsigned int stat[2];
	__poll_t flags = 0;
	int ret;
	int i;

	for (i = 0; i < 2; i++) {
		ret = regmap_read(regmap, A2B_MBOXSTAT(i), &stat[i]);
		if (ret)
			return 0;
	}

	if (stat[0] & A2B_MBOXSTAT_EMPTY)
		flags |= EPOLLOUT | EPOLLWRNORM;
	if (stat[1] & A2B_MBOXSTAT_FULL)
		flags |= EPOLLIN | EPOLLRDNORM;

	return flags;
}

/**
 * a2b_mailbox_chardev_poll() - poll the buffer to find out if it has data
 * @filp:	File structure pointer for device access
 * @wait:	Poll table structure pointer for which the driver adds
 *		a wait queue
 *
 * Return: (EPOLLIN | EPOLLRDNORM) if data is available for reading,
           (EPOLLOUT | EPOLLWRNORM) if room is available for writing
 *	   or 0 for other cases.
 */
static __poll_t a2b_mailbox_chardev_poll(struct file *filp,
					 struct poll_table_struct *wait)
{
	struct a2b_mailbox *mbox = filp->private_data;

	if (!mbox->node)
		return 0;

	poll_wait(filp, &mbox->node->mbox_pollq, wait);

	return a2b_mailbox_get_poll_flags(mbox);
}

#define A2B_MAILBOX_IRQ_POLL_DELAY_MS 1

static void a2b_mailbox_poll_status(struct work_struct *work)
{
	struct a2b_mailbox *mbox =
		container_of(work, struct a2b_mailbox, status_poll_work.work);
	__poll_t flags = 0;

	if (!mbox->node)
		return;

	flags = a2b_mailbox_get_poll_flags(mbox);
	if (flags != 0) {
		wake_up_interruptible_poll(&mbox->node->mbox_pollq,
					   (uintptr_t)flags);
	}

	if (test_bit(A2B_MAILBOX_BUSY, &mbox->flags)) {
		schedule_delayed_work(
			&mbox->status_poll_work,
			msecs_to_jiffies(A2B_MAILBOX_IRQ_POLL_DELAY_MS));
	}
}

/**
 * a2b_mailbox_chardev_open() - chrdev file open for buffer access and ioctls
 * @inode:	Inode structure for identifying the device in the file system
 * @filp:	File structure for a2b_mailbox device used to keep and later
 *              access private data
 *
 * Return: 0 on success or -EBUSY if the device is already opened
 **/
static int a2b_mailbox_chardev_open(struct inode *inode, struct file *filp)
{
	struct a2b_mailbox *mbox =
		container_of(inode->i_cdev, struct a2b_mailbox, chrdev);

	if (test_and_set_bit(A2B_MAILBOX_BUSY, &mbox->flags))
		return -EBUSY;

	filp->private_data = mbox;

	if (!a2b_node_has_irq(mbox->node)) {
		schedule_delayed_work(
			&mbox->status_poll_work,
			msecs_to_jiffies(A2B_MAILBOX_IRQ_POLL_DELAY_MS));
	}

	return 0;
}

/**
 * a2b_mailbox_release() - chrdev file close buffer access and ioctls
 * @inode:	Inode structure pointer for the char device
 * @filp:	File structure pointer for the char device
 *
 * Return: 0 for successful release
 */
static int a2b_mailbox_chardev_release(struct inode *inode, struct file *filp)
{
	struct a2b_mailbox *mbox =
		container_of(inode->i_cdev, struct a2b_mailbox, chrdev);

	clear_bit(A2B_MAILBOX_BUSY, &mbox->flags);
	if (!a2b_node_has_irq(mbox->node))
		cancel_delayed_work_sync(&mbox->status_poll_work);

	return 0;
}

static const struct file_operations a2b_mailbox_fops = {
	.read = a2b_mailbox_chardev_read,
	.write = a2b_mailbox_chardev_write,
	.open = a2b_mailbox_chardev_open,
	.release = a2b_mailbox_chardev_release,
	.poll = a2b_mailbox_chardev_poll,
	.owner = THIS_MODULE,
	.llseek = noop_llseek,
};

static void a2b_mailbox_device_release(struct device *dev)
{
	struct a2b_mailbox *mbox = dev_get_drvdata(dev);

	kfree(mbox);
}

static char *a2b_mailbox_device_devnode(struct device *dev, umode_t *mode,
					kuid_t *uid, kgid_t *gid)
{
	struct a2b_mailbox *mbox = dev_get_drvdata(dev);

	return kasprintf(GFP_KERNEL, "a2b/mbox%d-%d",
			 a2b_node_get_bus_id(mbox->node), mbox->node->id);
}

static struct device_type a2b_mailbox_device_type = {
	.release = a2b_mailbox_device_release,
	.devnode = a2b_mailbox_device_devnode,
};

static int a2b_mailbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct a2b_node *node;
	struct a2b_mailbox *mbox;
	unsigned int ctrl[2];
	unsigned int bus_id;
	u32 word_size;
	int ret;
	int i;

	if (!dev->of_node)
		return -ENODEV;

	node = dev_to_a2b_node(dev->parent);

	/* No mailbox on the main node */
	if (a2b_node_is_main(node))
		return -ENODEV;

	bus_id = a2b_node_get_bus_id(node);

	word_size = 1;
	of_property_read_u32(dev->of_node, "adi,word-size", &word_size);
	if (word_size < 1 || word_size > 4) {
		dev_err(dev, "Invalid word size %d. Must be between 1 and 4.\n",
			word_size);
		return -EINVAL;
	}

	/*
	 * Can't be managed allocation because the char dev can stay open after
	 * the platform_device is removed.
	 */
	mbox = kzalloc(sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->node = node;
	mbox->word_size = word_size;
	INIT_DELAYED_WORK(&mbox->status_poll_work, a2b_mailbox_poll_status);

	device_initialize(&mbox->dev);
	dev_set_drvdata(&mbox->dev, mbox);
	dev_set_name(&mbox->dev, "a2b-mbox%d-%d", bus_id, node->id);
	mbox->dev.parent = &pdev->dev;
	mbox->dev.type = &a2b_mailbox_device_type;
	mbox->dev.devt =
		MKDEV(MAJOR(a2b_mailbox_devt),
		      MINOR(a2b_mailbox_devt) + ((bus_id << 4) | node->id));

	cdev_init(&mbox->chrdev, &a2b_mailbox_fops);
	mbox->chrdev.owner = THIS_MODULE;

	platform_set_drvdata(pdev, mbox);

	ctrl[0] = A2B_MBOXCTL_EN | A2B_MBOXCTL_LEN(mbox->word_size);
	ctrl[1] = A2B_MBOXCTL_EN | A2B_MBOXCTL_LEN(mbox->word_size) |
		  A2B_MBOXCTL_TX;
	if (a2b_node_has_irq(node)) {
		ctrl[0] |= A2B_MBOXCTL_EIEN;
		ctrl[1] |= A2B_MBOXCTL_FIEN;
	}

	for (i = 0; i < 2; i++) {
		ret = regmap_write(node->regmap, A2B_MBOXCTL(i), ctrl[i]);
		if (ret) {
			dev_err(&pdev->dev, "Failed to write ctrl[%d]: %d\n", i,
				ret);
			return ret;
		}
	}

	return cdev_device_add(&mbox->chrdev, &mbox->dev);
}

static int a2b_mailbox_remove(struct platform_device *pdev)
{
	struct a2b_mailbox *mbox = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&mbox->status_poll_work);

	cdev_device_del(&mbox->chrdev, &mbox->dev);
	// TODO(ES-26693): locking
	mbox->node = NULL;

	return 0;
}

static const struct of_device_id a2b_mailbox_of_match[] = {
	{
		.compatible = "adi,ad2428w-mailbox",
	},
	{}
};
MODULE_DEVICE_TABLE(of, a2b_mailbox_of_match);

static struct platform_driver a2b_mailbox_driver = {
	.driver = {
		.name = "a2b-mailbox",
		.of_match_table = a2b_mailbox_of_match,
	},
	.probe = a2b_mailbox_probe,
	.remove = a2b_mailbox_remove,
};

static int a2b_mailbox_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&a2b_mailbox_devt, 0, 256, "iio");
	if (ret)
		return ret;

	ret = platform_driver_register(&a2b_mailbox_driver);
	if (ret)
		goto err_unregister_chrdev_region;

	return 0;

err_unregister_chrdev_region:
	unregister_chrdev_region(a2b_mailbox_devt, 256);
	return ret;
}
module_init(a2b_mailbox_init);

static void __exit a2b_mailbox_exit(void)
{
	platform_driver_unregister(&a2b_mailbox_driver);

	if (a2b_mailbox_devt)
		unregister_chrdev_region(a2b_mailbox_devt, 256);
}
module_exit(a2b_mailbox_exit);

MODULE_DESCRIPTION("AD2428 mailbox driver");
MODULE_LICENSE("GPL v2");
