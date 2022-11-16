// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADI Linkport driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/scatterlist.h>
#include <linux/major.h>
#include <linux/pinctrl/consumer.h>
#include <asm/irq.h>
#include <mach/dma.h>
#include <linux/soc/adi/cpu.h>
#include <mach/irqs.h>
#include <linux/soc/adi/hardware.h>
#ifdef ARCH_SC58X
#include <mach/sc58x.h>
#elif defined(ARCH_SC57X)
#include <mach/sc57x.h>
#endif

/* Kfifo size in elements (in byte) */
#define FIFO_SIZE					4096

#define LINKPORT_DRVNAME			"adi-linkport"
#define LINKPORT_DEVNUM				2
#define LINKPORT_READ_TIMEOUT		2100   /* Receiver timeout in 2.1 seconds */
#define LINKPORT_TRANSFER_TIMEOUT	5100   /* Transfer timeout in 5.1 seconds */
#define LINKPORT_MAGIC_NUM			0xf3ad4b5c        /* Message magic number */
#define LP_CTL_EN					0x1
#define LP_CTL_TRAN					0x8
#define LP_CTL_TRQMSK				0x100
#define LP_CTL_RRQMSK				0x200
#define LP_CTL_ITMSK				0x800

#define LP_STAT_DONE				0x1000
#define LP_STAT_LTRQ				0x1
#define LP_STAT_LRRQ				0x2
#define LP_STAT_LPIT				0x8
#define LP_STAT_FFST				0x70
#define LP_STAT_LERR				0x80
#define LP_STAT_LPBS				0x100

#define LP_CTL_OFF					0x0
#define LP_STAT_OFF					0x4
#define LP_DIV_OFF					0x8
#define LP_CNT_OFF					0xC
#define LP_TX_OFF					0x10
#define LP_RX_OFF					0x14
#define LP_TX_SHADOW_OFF			0x18
#define LP_RX_SHADOW_OFF			0x1C

struct adi_linkport {
	struct list_head lp_dev;
	struct class *class;
	int major;
	spinlock_t lp_dev_lock;
};

/* Linkport message header */
struct adi_lp_msg_header {
	unsigned int magic_num;
	size_t data_size;		/* Message data size in byte */
	u16 reserved[2];		/* Header reserved in one word */
};

struct adi_lp_dev {
	struct list_head list;
	struct device *device;
	phys_addr_t preg_base;
	void __iomem *reg_base;
	wait_queue_head_t rx_waitq;
	spinlock_t lock;
	struct workqueue_struct *workqueue;
	struct work_struct transfer_work;
	int linkport_num;
	int dma_chan;
	int status;
	int lp_div;
	int data_irq;
	int data_irq_disabled;
	int status_irq;
	int status_irq_disabled;
	int count;
	DECLARE_KFIFO_PTR(lpfifo, char);	/* Element in byte */
	struct adi_lp_msg_header msg_header;
	size_t msg_header_count;
	size_t msg_data_count;
	struct completion complete;
};

struct adi_linkport *linkport_dev;

struct adi_lp_dev lp_dev_info[LINKPORT_DEVNUM] = {
	{
		.preg_base = LP0_CTL,
		.data_irq = IRQ_LP0,
		.status_irq = IRQ_LP0_STAT,
		.dma_chan = CH_LP0,
	},
	{
		.preg_base = LP1_CTL,
		.data_irq = IRQ_LP1,
		.status_irq = IRQ_LP1_STAT,
		.dma_chan = CH_LP1,
	},
};

static void adi_lp_enable_irq(struct adi_lp_dev *lpdev, unsigned int irq)
{
	if (lpdev == NULL)
		return;

	if (irq == lpdev->data_irq && lpdev->data_irq_disabled) {
		lpdev->data_irq_disabled = 0;
		enable_irq(irq);
	} else if (irq == lpdev->status_irq && lpdev->status_irq_disabled) {
		lpdev->status_irq_disabled = 0;
		enable_irq(irq);
	}
}

static void adi_lp_disable_irq(struct adi_lp_dev *lpdev, unsigned int irq)
{
	if (lpdev == NULL)
		return;

	if (irq == lpdev->data_irq && !lpdev->data_irq_disabled) {
		disable_irq(irq);
		lpdev->data_irq_disabled = 1;
	} else if (irq == lpdev->status_irq && !lpdev->status_irq_disabled) {
		disable_irq(irq);
		lpdev->status_irq_disabled = 1;
	}
}

int adi_lp_config_channel(struct adi_lp_dev *lpdev, int direction)
{
	uint32_t reg;
	uint32_t ctl;

	if (direction)
		reg = LP_CTL_TRAN | LP_CTL_TRQMSK;
	else
		reg =  LP_CTL_RRQMSK;

	ctl = readl(lpdev->reg_base + LP_CTL_OFF);
	writel(ctl | reg, lpdev->reg_base + LP_CTL_OFF);
	return 0;
}

void adi_lp_enable(struct adi_lp_dev *lpdev)
{
	uint32_t ctl;

	ctl = readl(lpdev->reg_base + LP_CTL_OFF);
	writel(ctl | LP_CTL_EN, lpdev->reg_base + LP_CTL_OFF);
}

void adi_lp_disable(struct adi_lp_dev *lpdev)
{
	writel(0xFF, lpdev->reg_base + LP_STAT_OFF);
	writel(!LP_CTL_EN, lpdev->reg_base + LP_CTL_OFF);
}

int adi_lp_get_rx_fifo(struct adi_lp_dev *lpdev)
{
	uint32_t stat = readl(lpdev->reg_base + LP_STAT_OFF);

	stat = (stat & LP_STAT_FFST) >> 4;
	if (stat <= 4)
		return stat;
	else
		return 0;
}

int adi_lp_get_tx_fifo(struct adi_lp_dev *lpdev)
{
	uint32_t stat = readl(lpdev->reg_base + LP_STAT_OFF);

	stat = (stat & (LP_STAT_FFST)) >> 4;
	if (stat == 0)
		return 2;
	else if (stat == 4)
		return 1;
	else
		return 0;
}

static void lp_kfifo_in_drain(struct adi_lp_dev *dev, unsigned int *buf,
			size_t count)
{
	int drain_count;
	int ret;

	if (dev == NULL || buf == NULL) {
		panic("%s, invalid pointer", __func__);
		return;
	}

	drain_count = count;
	do {
		ret = kfifo_in(&dev->lpfifo, (char *)buf, drain_count);
		drain_count -= ret;
		if (drain_count > 0) {
			wake_up_interruptible(&dev->rx_waitq);
			msleep(20);
			*buf >>= (ret * 8);
		}
	} while (drain_count > 0);
}

static void lp_rx_fifo(struct adi_lp_dev *dev)
{
	int ready_cnt;
	int copy_size;
	int offset;

	ready_cnt = adi_lp_get_rx_fifo(dev);
	while (ready_cnt) {
		unsigned int data;

		data = readl(dev->reg_base + LP_RX_OFF);
		/* Step 1:	Parse the message header */
		if (dev->msg_header_count < sizeof(dev->msg_header)) {
			copy_size = min(sizeof(data),
						sizeof(dev->msg_header) - dev->msg_header_count);
			memcpy((char *)&dev->msg_header + dev->msg_header_count,
						(char *)&data, copy_size);
			dev->msg_header_count += copy_size;
			/* Check the magic number of message */
			if (dev->msg_header_count == sizeof(dev->msg_header)) {
				if (dev->msg_header.magic_num != LINKPORT_MAGIC_NUM) {
					pr_err("%s: Bad config message\n", __func__);
					dev->msg_header_count = 0;
					ready_cnt--;
					break;
				}
			}
			/* Copy the remaining bytes of one word to RX kfifo */
			if (copy_size < sizeof(data)) {
				offset = sizeof(data) - copy_size;
				data >>= (copy_size * 8);
				lp_kfifo_in_drain(dev, &data, offset);
				dev->msg_data_count = offset;
			} else if (copy_size == sizeof(data))
				dev->msg_data_count = 0;

			ready_cnt--;
			continue;
		}
		/* Step 2: If the message header has been read done, then copy message
		 * data to RX kfifo until available message data size has been reached
		 */
		if (dev->msg_header_count == sizeof(dev->msg_header) &&
					dev->msg_data_count < dev->msg_header.data_size) {
			copy_size = min(sizeof(data),
						dev->msg_header.data_size - dev->msg_data_count);
			lp_kfifo_in_drain(dev, &data, copy_size);
			dev->msg_data_count += copy_size;

			/* If the remaining bytes data match the corresponding part of the
			 * magic number, there will be next available message package, copy
			 * it to header receiver.
			 */
			if (copy_size < sizeof(data)) {
				offset = sizeof(data) - copy_size;
				data >>= (copy_size * 8);
				if (data & LINKPORT_MAGIC_NUM) {
					memcpy((char *)&dev->msg_header, &data, offset);
					dev->msg_header_count = offset;
				}
			}
		}
		/* Step 3: If the message header and data both have been read done, it
		 * means one message package is transferred done, initialize the message
		 * header receiver and clear the message counter.
		 */
		if ((dev->msg_header_count == sizeof(dev->msg_header)) &&
					(dev->msg_data_count == dev->msg_header.data_size)) {
			memset(&dev->msg_header, 0, sizeof(dev->msg_header));
			dev->msg_data_count = 0;
			dev->msg_header_count = 0;
			adi_lp_disable_irq(dev, dev->data_irq);
		}

		ready_cnt--;
	}

	adi_lp_enable_irq(dev, dev->data_irq);
	/* Wake up read/write block */
	wake_up_interruptible(&dev->rx_waitq);
}

static void lp_tx_fifo(struct adi_lp_dev *dev)
{
	unsigned int data = 0;
	int ready_cnt;
	int ret;
	int copy_size;

	ready_cnt = adi_lp_get_tx_fifo(dev);
	if (!ready_cnt) {
		pr_debug("Wait for TX-FIFO available\n");
		return;
	}

	/* Get data from TX kfifo */
	copy_size = min(sizeof(data), kfifo_len(&dev->lpfifo));
	ret = kfifo_out(&dev->lpfifo, (char *)&data, copy_size);
	if (ret != copy_size) {
		pr_err("%s: Data transferred failed %d\n", __func__, ret);
		return;
	}
	writel(data, dev->reg_base + LP_TX_OFF);
}

static void transfer_fn(struct work_struct *work)
{
	struct adi_lp_dev *dev = container_of(work,
				struct adi_lp_dev, transfer_work);

	if (dev->status == LP_STAT_LTRQ) {
		while (kfifo_len(&dev->lpfifo)) {
			/* Check if linkport transfer bus is busy */
			if (readl(dev->reg_base + LP_STAT_OFF) & (LP_STAT_LPBS)) {
				adi_lp_enable_irq(dev, dev->data_irq);
				break;
			}
			lp_tx_fifo(dev);
		}
		if (kfifo_len(&dev->lpfifo) == 0) {
			dev->status = LP_STAT_DONE;
			kfifo_reset(&dev->lpfifo);
			adi_lp_enable_irq(dev, dev->data_irq);
		}
	} else if (dev->status == LP_STAT_DONE) {
		complete(&dev->complete);
		adi_lp_disable_irq(dev, dev->data_irq);
		pr_debug("transfer complete\n");
	} else {
		/* Check if linkport receiver bus is busy */
		while (readl(dev->reg_base + LP_STAT_OFF) & LP_STAT_LPBS)
			;
		lp_rx_fifo(dev);
	}
}

/*
 * adi_linkport driver interrupt handler function
 *
 */
static irqreturn_t adi_lp_irq(int irq, void *dev_id)
{
	struct adi_lp_dev *dev = (struct adi_lp_dev *)dev_id;
	uint32_t stat = readl(dev->reg_base + LP_STAT_OFF);

	pr_debug("%s, irq:%d, stat:0x%x, dev:%p, status:%d\n",
				__func__, irq, stat, dev, dev->status);

	if (dev->workqueue == NULL)
		return IRQ_NONE;

	if (irq == dev->data_irq && !dev->data_irq_disabled) {
		disable_irq_nosync(irq);
		dev->data_irq_disabled = 1;
	} else if (irq == dev->status_irq && !dev->status_irq_disabled) {
		disable_irq_nosync(irq);
		dev->status_irq_disabled = 1;
	}

	if (stat & LP_STAT_LTRQ) {
		if (kfifo_len(&dev->lpfifo))
			dev->status = LP_STAT_LTRQ;
		else
			dev->status = LP_STAT_DONE;

		adi_lp_enable_irq(dev, dev->data_irq);
		adi_lp_enable(dev);
		goto out;
	} else if (stat & LP_STAT_LRRQ) {
		dev->status = LP_STAT_LRRQ;
		goto out;
	}

	queue_work(dev->workqueue, &dev->transfer_work);

out:
	writel(stat, dev->reg_base + LP_STAT_OFF);
	dev->count++;
	return IRQ_HANDLED;
}

static int adi_lp_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	struct adi_lp_dev *dev;
	unsigned int index = iminor(inode);
	int ret = -EBUSY;

	dev = &lp_dev_info[index];
	if (!dev) {
		pr_err("Device: minor %d unknown", index);
		return ret;
	}

	spin_lock_irqsave(&dev->lock, flags);
	filp->private_data = dev;
	spin_unlock_irqrestore(&dev->lock, flags);

	writel(dev->lp_div, dev->reg_base + LP_DIV_OFF);
	/* Note: As the interrupts are asserted only when linkport (receiver or
	 * transmitter) is disabled according to the HRM, so disable the
	 * linkport device first.
	 */
	adi_lp_disable(dev);

	pr_debug("adi lp open device: %d\n", index);
	return 0;
}

static int adi_lp_release(struct inode *inode, struct file *filp)
{
	struct adi_lp_dev *dev = filp->private_data;
	unsigned int index = iminor(inode);
	int ret;

	pr_debug("adi lp relese device: %d\n", index);
	if (readl(dev->reg_base + LP_CTL_OFF) & LP_CTL_TRAN)
		ret = wait_for_completion_interruptible_timeout(&dev->complete,
					msecs_to_jiffies(LINKPORT_TRANSFER_TIMEOUT));

	adi_lp_disable_irq(dev, dev->data_irq);
	adi_lp_disable_irq(dev, dev->status_irq);
	adi_lp_disable(dev);

	dev->status = 0;
	dev->msg_header_count = 0;
	dev->msg_data_count = 0;
	kfifo_reset(&dev->lpfifo);
	flush_workqueue(dev->workqueue);

	if (ret == 0)
		return -ETIMEDOUT;

	if (ret == ERESTARTSYS)
		return -ERESTARTSYS;

	return 0;
}

static ssize_t adi_lp_read(struct file *filp, char *buf, size_t count,
			loff_t *pos)
{
	struct adi_lp_dev *dev = filp->private_data;
	int fifo_cnt = 0;
	unsigned int copied = 0;
	unsigned int copied_total = 0;
	int n;
	int ret;

	n = count;

	adi_lp_enable_irq(dev, dev->status_irq);
	adi_lp_enable_irq(dev, dev->data_irq);
	adi_lp_config_channel(dev, 0);
	adi_lp_enable(dev);

	while (n > 0) {
		fifo_cnt = kfifo_len(&dev->lpfifo);
		if (!fifo_cnt) {
			pr_debug("No data, wait event\n");
			ret = wait_event_interruptible_timeout(dev->rx_waitq,
						kfifo_len(&dev->lpfifo) != 0,
						msecs_to_jiffies(LINKPORT_READ_TIMEOUT));
			if (ret == 0) {
				pr_debug("Linkport read timeout: %d\n ", ret);
				goto out;
			} else
				continue;
		}
		ret = kfifo_to_user(&dev->lpfifo, buf + copied_total,
					count - copied_total, &copied);
		copied_total += copied;
		n -= copied;
	}

out:
	return copied_total;
}

static ssize_t adi_lp_write(struct file *filp, const char *buf, size_t count,
			loff_t *pos)
{
	struct adi_lp_dev *dev = filp->private_data;
	unsigned int copied = 0;
	int ret;

	dev->msg_header.magic_num = LINKPORT_MAGIC_NUM;
	dev->msg_header.data_size = count;

	ret = kfifo_in(&dev->lpfifo, (char *)&dev->msg_header,
				sizeof(dev->msg_header));
	if (ret != sizeof(dev->msg_header))
		return copied;

	ret = kfifo_from_user(&dev->lpfifo, buf, count, &copied);
	if (ret)
		goto abort;
	/* Enable the status interrupt, and disable the transmitter linkport, then
	 * the transmit service request interrupt is allowed
	 */
	adi_lp_enable_irq(dev, dev->status_irq);
	adi_lp_disable(dev);

	writel(dev->lp_div, dev->reg_base + LP_DIV_OFF);
	adi_lp_config_channel(dev, 1);

	return copied;

abort:
	pr_err("Write to TX kfifo failed: 0x%x\n", ret);
	kfifo_reset(&dev->lpfifo);
	return -EAGAIN;
}

static long adi_lp_ioctl(struct file *filp, uint cmd, unsigned long arg)
{
	/* TODO: add the ioctl handler function */
	return 0;
}

static const struct file_operations linkport_fops = {
	.owner = THIS_MODULE,
	.read = adi_lp_read,
	.write = adi_lp_write,
	.unlocked_ioctl = adi_lp_ioctl,
	.open = adi_lp_open,
	.release = adi_lp_release,
};

static ssize_t
linkport_status_show(struct class *class, struct class_attribute *attr,
			char *buf)
{
	char *p = buf;
	struct adi_lp_dev *dev;

	p += sprintf(p, "Linkport status\n");
	list_for_each_entry(dev, &linkport_dev->lp_dev, list) {
		p += sprintf(p, "linkport num %d\n", dev->linkport_num);
	}
	return (p - buf);
}

static ssize_t
linkport_reg_show(struct class *class, struct class_attribute *attr, char *buf)
{
	char *p = buf;
	struct adi_lp_dev *dev;

	p += sprintf(p, "Linkport status\n");
	list_for_each_entry(dev, &linkport_dev->lp_dev, list) {
		p += sprintf(p, "linkport num %d\n", dev->linkport_num);
		p += sprintf(p, "\t clt %d\n", readl(dev->reg_base + LP_CTL_OFF));
		p += sprintf(p, "\t stat %d\n", readl(dev->reg_base + LP_STAT_OFF));
	}
	return (p - buf);
}

static ssize_t
linkport_reg_store(struct class *class, struct class_attribute *attr,
			const char *buf, size_t count)
{
	char *p;
	int rw = 0;
	uint32_t value = 0;
	char buffer[64];
	uint32_t res;
	unsigned long temp;
	ssize_t ret;
	void __iomem *addr;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	p = buffer;

	while (*p == ' ')
		p++;

	if (p[0] == 'r')
		rw = 0;
	else if (p[0] == 'w')
		rw = 1;
	else
		pr_debug("-EINVAL\n");

	if (p[1] < '0' && p[1] > '9')
		pr_debug("-EINVAL2\n");

	ret = kstrtouint(&p[1], 10, &res);
	if (ret != 0)
		return ret;

	if (res == 8)
		p += 2;
	else if (res == 16)
		p += 3;
	else if (res == 32)
		p += 3;
	else
		pr_debug("-EINVAL3\n");

	while (*p == ' ')
		p++;

	ret = kstrtouint(p, 16, &temp);
	if (ret != 0)
		return ret;

	addr = (void __iomem *)temp;

	if (rw) {
		p = endp;
		while (*p == ' ')
			p++;

		ret = kstrtouint(p, 16, &value);
		if (ret != 0)
			return ret;

		switch (res) {
		case 8:
			writeb((uint8_t)value, addr);
			value = readb(addr);
			break;
		case 16:
			writew((uint16_t)value, addr);
			value = readw(addr);
			break;
		case 32:
			writel((uint32_t)value, addr);
			value = readl(addr);
			break;
		}
		pr_debug("Write addr %p reg %08x\n", addr, value);
	} else {
		switch (res) {
		case 8:
			value = readb(addr);
			break;
		case 16:
			value = readw(addr);
			break;
		case 32:
			value = readl(addr);
			break;
		}
		pr_debug("Read addr %p reg %08x\n", addr, value);
	}
	return count;
}

static CLASS_ATTR_RO(linkport_status);
static CLASS_ATTR_RW(linkport_reg);

/*
 * adi_linkport_init - Driver module loading
 *
 */
static int __init adi_linkport_init(void)
{
	struct device_node *np;
	int err;
	dev_t lp_dev;
	int i;
	int ret;
	struct adi_lp_dev *lpdev;

	/* Allocate glue struct */
	linkport_dev = kzalloc(sizeof(*linkport_dev), GFP_KERNEL);
	if (!linkport_dev)
		return -ENOMEM;

	/* Register linkport char device */
	linkport_dev->major = register_chrdev(UNNAMED_MAJOR, "adi-linkport",
				&linkport_fops);
	if (linkport_dev->major < 0) {
		err = linkport_dev->major;
		pr_err("Error %d registering chrdev for device\n", err);
		goto free;
	}

	lp_dev = MKDEV(linkport_dev->major, 0);

	/* Create linkport class */
	linkport_dev->class = class_create(THIS_MODULE, "linkport");
	err = class_create_file(linkport_dev->class, &class_attr_linkport_status);
	if (err) {
		pr_err("Error %d registering class device\n", err);
		goto free_chrdev;
	}

	err = class_create_file(linkport_dev->class, &class_attr_linkport_reg);
	if (err) {
		pr_err("Error %d registering class device\n", err);
		class_remove_file(linkport_dev->class, &class_attr_linkport_status);
		goto free_chrdev;
	}

	INIT_LIST_HEAD(&linkport_dev->lp_dev);
	spin_lock_init(&linkport_dev->lp_dev_lock);

	/* Create linkport devices */
	for (i = 0; i < LINKPORT_DEVNUM; i++) {
		struct device *dev;

		dev = device_create(linkport_dev->class, NULL, lp_dev + i,
					&lp_dev_info[i], "linkport%d", i);
		if (!dev)
			goto free_chrdev;

		/* Find our device node */
		np = of_find_compatible_node(NULL, NULL, dev_name(dev));
		if (np) {
			pr_err("Find dt node %s%d\n", np->name, i);
			dev->of_node = of_node_get(np);

			/* Find linkport clock divider value */
			ret = of_property_read_u32(dev->of_node, "clock-div",
						&lp_dev_info[i].lp_div);
			if (ret)
				panic("Can't parse LP DIV");
			pr_err("of parse lp_div: %d\n", lp_dev_info[i].lp_div);

			/* Find and map linkport interrupts */
			lp_dev_info[i].data_irq = irq_of_parse_and_map(dev->of_node, 0);
			if (lp_dev_info[i].data_irq <= 0)
				panic("Can't parse IRQ");

			lp_dev_info[i].status_irq = irq_of_parse_and_map(dev->of_node, 1);
			if (lp_dev_info[i].status_irq <= 0)
				panic("Can't parse IRQ");

			pr_err("of parse data irq: %d, status irq: %d\n",
						lp_dev_info[i].data_irq, lp_dev_info[i].status_irq);

			set_spu_securep_msec(5, true);
			set_spu_securep_msec(6, true);

		} else
			pr_err("Not found dt node %s %s\n", __func__, dev_name(dev));

		/* Init the linkport device's info */
		lp_dev_info[i].reg_base = ioremap(lp_dev_info[i].preg_base, 0x20);
		lp_dev_info[i].device = dev;
		lp_dev_info[i].linkport_num = i;
		spin_lock_init(&lp_dev_info[i].lock);
		init_waitqueue_head(&lp_dev_info[i].rx_waitq);
		INIT_WORK(&lp_dev_info[i].transfer_work, transfer_fn);
		INIT_LIST_HEAD(&lp_dev_info[i].list);
		init_completion(&lp_dev_info[i].complete);
		list_add(&lp_dev_info[i].list, &linkport_dev->lp_dev);

		lpdev = &lp_dev_info[i];

		/* Allocate linkport kfifo with size of 4096 bytes */
		ret = kfifo_alloc(&lpdev->lpfifo, FIFO_SIZE, GFP_KERNEL);
		if (ret) {
			pr_err("Error kfifo_alloc\n");
			err = ret;
		}

		/* Request the linkport interrupts */
		if (request_irq(lpdev->data_irq, adi_lp_irq, 0, LINKPORT_DRVNAME,
						lpdev)) {
			pr_err("Requesting data irq %d failed\n",
						lpdev->data_irq);
			err = -ENODEV;
		}

		if (request_irq(lpdev->status_irq, adi_lp_irq, 0, LINKPORT_DRVNAME,
						lpdev)) {
			pr_err("Requesting status irq  %d failed\n",
						lpdev->status_irq);
			ret = -ENODEV;
		}

		/* Init the linkport message transfer work queue */
		lpdev->workqueue = create_singlethread_workqueue("linkport_work");
		if (!lpdev->workqueue) {
			pr_err("Create workqueue failed\n");
			err = -ENOMEM;
		}

		if (IS_ERR(devm_pinctrl_get_select_default(dev))) {
			pr_err("Requesting Peripheral for %s failed.\n",
						dev_name(dev));
			device_destroy(linkport_dev->class, lp_dev + i);
			err = -EINVAL;
		}

		if (err) {
			destroy_workqueue(lpdev->workqueue);
			free_irq(lpdev->data_irq, lpdev);
			free_irq(lpdev->status_irq, lpdev);
			kfifo_free(&lpdev->lpfifo);
		}
	}
	return 0;

free_chrdev:
	unregister_chrdev(linkport_dev->major, "adi-linkport");
free:
	kfree(linkport_dev);

	return err;
}

/*
 * adi_linkport_exit - Driver module unloading
 *
 */
static void __exit adi_linkport_exit(void)
{
	struct adi_lp_dev *lpdev;
	int i;

	for (i = 0; i < LINKPORT_DEVNUM; i++) {
		lpdev = &lp_dev_info[i];

		/* Cleanup linkport worke queue  */
		if (lpdev->workqueue != NULL) {
			flush_workqueue(lpdev->workqueue);
			destroy_workqueue(lpdev->workqueue);
		}

		/* Unmap the linkport register from the kernel space */
		if (lpdev->reg_base != NULL)
			iounmap(lpdev->reg_base);

		/* Free the linkport interrupts */
		if (lpdev->data_irq)
			free_irq(lpdev->data_irq, lpdev);

		if (lpdev->status_irq)
			free_irq(lpdev->status_irq, lpdev);

		/* Free the linkport kfifo */
		kfifo_free(&lpdev->lpfifo);

		/* Destroy the linkport devices */
		device_destroy(linkport_dev->class, MKDEV(linkport_dev->major, 0) + i);
	}

	/* Destroy the linkport class */
	if (linkport_dev->class != NULL) {
		class_remove_file(linkport_dev->class, &class_attr_linkport_reg);
		class_remove_file(linkport_dev->class, &class_attr_linkport_status);
		class_destroy(linkport_dev->class);
		linkport_dev->class = NULL;
	}

	/* Unregister linkport char device and free the glue struct */
	unregister_chrdev(linkport_dev->major, "adi-linkport");
	kfree(linkport_dev);
}

module_init(adi_linkport_init);
module_exit(adi_linkport_exit);

MODULE_DESCRIPTION("ADI Linkport Glue Layer");
MODULE_LICENSE("GPL v2");
