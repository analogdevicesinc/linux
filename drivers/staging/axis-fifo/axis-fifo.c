// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx AXIS FIFO: interface to the Xilinx AXI-Stream FIFO IP core
 *
 * Copyright (C) 2018 Jacob Feder
 *
 * Authors:  Jacob Feder <jacobsfeder@gmail.com>
 *
 * See Xilinx PG080 document for IP details
 */

/* ----------------------------
 *           includes
 * ----------------------------
 */

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/param.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>

/* ----------------------------
 *       driver parameters
 * ----------------------------
 */

#define DRIVER_NAME "axis_fifo"

#define READ_BUF_SIZE 128U /* read buffer length in words */

#define AXIS_FIFO_DEBUG_REG_NAME_MAX_LEN	4

/* ----------------------------
 *     IP register offsets
 * ----------------------------
 */

#define XLLF_ISR_OFFSET  0x00000000  /* Interrupt Status */
#define XLLF_IER_OFFSET  0x00000004  /* Interrupt Enable */

#define XLLF_TDFR_OFFSET 0x00000008  /* Transmit Reset */
#define XLLF_TDFV_OFFSET 0x0000000c  /* Transmit Vacancy */
#define XLLF_TDFD_OFFSET 0x00000010  /* Transmit Data */
#define XLLF_TLR_OFFSET  0x00000014  /* Transmit Length */

#define XLLF_RDFR_OFFSET 0x00000018  /* Receive Reset */
#define XLLF_RDFO_OFFSET 0x0000001c  /* Receive Occupancy */
#define XLLF_RDFD_OFFSET 0x00000020  /* Receive Data */
#define XLLF_RLR_OFFSET  0x00000024  /* Receive Length */
#define XLLF_SRR_OFFSET  0x00000028  /* Local Link Reset */
#define XLLF_TDR_OFFSET  0x0000002C  /* Transmit Destination */
#define XLLF_RDR_OFFSET  0x00000030  /* Receive Destination */

/* ----------------------------
 *     reset register masks
 * ----------------------------
 */

#define XLLF_RDFR_RESET_MASK        0x000000a5 /* receive reset value */
#define XLLF_TDFR_RESET_MASK        0x000000a5 /* Transmit reset value */
#define XLLF_SRR_RESET_MASK         0x000000a5 /* Local Link reset value */

/* ----------------------------
 *       interrupt masks
 * ----------------------------
 */

#define XLLF_INT_RPURE_MASK       0x80000000 /* Receive under-read */
#define XLLF_INT_RPORE_MASK       0x40000000 /* Receive over-read */
#define XLLF_INT_RPUE_MASK        0x20000000 /* Receive underrun (empty) */
#define XLLF_INT_TPOE_MASK        0x10000000 /* Transmit overrun */
#define XLLF_INT_TC_MASK          0x08000000 /* Transmit complete */
#define XLLF_INT_RC_MASK          0x04000000 /* Receive complete */
#define XLLF_INT_TSE_MASK         0x02000000 /* Transmit length mismatch */

#define XLLF_INT_CLEAR_ALL	GENMASK(31, 0)

/* ----------------------------
 *           globals
 * ----------------------------
 */
static long read_timeout = 1000; /* ms to wait before read() times out */
static long write_timeout = 1000; /* ms to wait before write() times out */

static DEFINE_IDA(axis_fifo_ida);

/* ----------------------------
 * module command-line arguments
 * ----------------------------
 */

module_param(read_timeout, long, 0444);
MODULE_PARM_DESC(read_timeout, "ms to wait before blocking read() timing out; set to -1 for no timeout");
module_param(write_timeout, long, 0444);
MODULE_PARM_DESC(write_timeout, "ms to wait before blocking write() timing out; set to -1 for no timeout");

/* ----------------------------
 *            types
 * ----------------------------
 */

struct axis_fifo {
	int id;
	void __iomem *base_addr; /* kernel space memory */

	unsigned int rx_fifo_depth; /* max words in the receive fifo */
	unsigned int tx_fifo_depth; /* max words in the transmit fifo */
	int has_rx_fifo; /* whether the IP has the rx fifo enabled */
	int has_tx_fifo; /* whether the IP has the tx fifo enabled */

	wait_queue_head_t read_queue; /* wait queue for asynchronos read */
	struct mutex read_lock; /* lock for reading */
	wait_queue_head_t write_queue; /* wait queue for asynchronos write */
	struct mutex write_lock; /* lock for writing */

	struct device *dt_device; /* device created from the device tree */
	struct miscdevice miscdev;

	struct dentry *debugfs_dir;
};

struct axis_fifo_debug_reg {
	const char * const name;
	unsigned int offset;
};

/* ----------------------------
 *        implementation
 * ----------------------------
 */

static void reset_ip_core(struct axis_fifo *fifo)
{
	iowrite32(XLLF_SRR_RESET_MASK, fifo->base_addr + XLLF_SRR_OFFSET);
	iowrite32(XLLF_TDFR_RESET_MASK, fifo->base_addr + XLLF_TDFR_OFFSET);
	iowrite32(XLLF_RDFR_RESET_MASK, fifo->base_addr + XLLF_RDFR_OFFSET);
	iowrite32(XLLF_INT_TC_MASK | XLLF_INT_RC_MASK | XLLF_INT_RPURE_MASK |
		  XLLF_INT_RPORE_MASK | XLLF_INT_RPUE_MASK |
		  XLLF_INT_TPOE_MASK | XLLF_INT_TSE_MASK,
		  fifo->base_addr + XLLF_IER_OFFSET);
	iowrite32(XLLF_INT_CLEAR_ALL, fifo->base_addr + XLLF_ISR_OFFSET);
}

/**
 * axis_fifo_read() - Read a packet from AXIS-FIFO character device.
 * @f: Open file.
 * @buf: User space buffer to read to.
 * @len: User space buffer length.
 * @off: Buffer offset.
 *
 * As defined by the device's documentation, we need to check the device's
 * occupancy before reading the length register and then the data. All these
 * operations must be executed atomically, in order and one after the other
 * without missing any.
 *
 * Returns the number of bytes read from the device or negative error code
 *	on failure.
 */
static ssize_t axis_fifo_read(struct file *f, char __user *buf,
			      size_t len, loff_t *off)
{
	struct axis_fifo *fifo = (struct axis_fifo *)f->private_data;
	size_t bytes_available;
	unsigned int words_available;
	unsigned int copied;
	unsigned int copy;
	unsigned int i;
	int ret;
	u32 tmp_buf[READ_BUF_SIZE];

	if (f->f_flags & O_NONBLOCK) {
		/*
		 * Device opened in non-blocking mode. Try to lock it and then
		 * check if any packet is available.
		 */
		if (!mutex_trylock(&fifo->read_lock))
			return -EAGAIN;

		if (!ioread32(fifo->base_addr + XLLF_RDFO_OFFSET)) {
			ret = -EAGAIN;
			goto end_unlock;
		}
	} else {
		/* opened in blocking mode
		 * wait for a packet available interrupt (or timeout)
		 * if nothing is currently available
		 */
		mutex_lock(&fifo->read_lock);
		ret = wait_event_interruptible_timeout(fifo->read_queue,
						       ioread32(fifo->base_addr + XLLF_RDFO_OFFSET),
						       read_timeout);

		if (ret <= 0) {
			if (ret == 0) {
				ret = -EAGAIN;
			} else if (ret != -ERESTARTSYS) {
				dev_err(fifo->dt_device, "wait_event_interruptible_timeout() error in read (ret=%i)\n",
					ret);
			}

			goto end_unlock;
		}
	}

	bytes_available = ioread32(fifo->base_addr + XLLF_RLR_OFFSET);
	words_available = bytes_available / sizeof(u32);
	if (!bytes_available) {
		dev_err(fifo->dt_device, "received a packet of length 0\n");
		ret = -EIO;
		goto end_unlock;
	}

	if (bytes_available > len) {
		dev_err(fifo->dt_device, "user read buffer too small (available bytes=%zu user buffer bytes=%zu)\n",
			bytes_available, len);
		ret = -EINVAL;
		goto err_flush_rx;
	}

	if (bytes_available % sizeof(u32)) {
		/* this probably can't happen unless IP
		 * registers were previously mishandled
		 */
		dev_err(fifo->dt_device, "received a packet that isn't word-aligned\n");
		ret = -EIO;
		goto err_flush_rx;
	}

	/* read data into an intermediate buffer, copying the contents
	 * to userspace when the buffer is full
	 */
	copied = 0;
	while (words_available > 0) {
		copy = min(words_available, READ_BUF_SIZE);

		for (i = 0; i < copy; i++) {
			tmp_buf[i] = ioread32(fifo->base_addr +
					      XLLF_RDFD_OFFSET);
		}
		words_available -= copy;

		if (copy_to_user(buf + copied * sizeof(u32), tmp_buf,
				 copy * sizeof(u32))) {
			ret = -EFAULT;
			goto err_flush_rx;
		}

		copied += copy;
	}
	mutex_unlock(&fifo->read_lock);

	return bytes_available;

err_flush_rx:
	while (words_available--)
		ioread32(fifo->base_addr + XLLF_RDFD_OFFSET);

end_unlock:
	mutex_unlock(&fifo->read_lock);

	return ret;
}

/**
 * axis_fifo_write() - Write buffer to AXIS-FIFO character device.
 * @f: Open file.
 * @buf: User space buffer to write to the device.
 * @len: User space buffer length.
 * @off: Buffer offset.
 *
 * As defined by the device's documentation, we need to write to the device's
 * data buffer then to the device's packet length register atomically. Also,
 * we need to lock before checking if the device has available space to avoid
 * any concurrency issue.
 *
 * Returns the number of bytes written to the device or negative error code
 *	on failure.
 */
static ssize_t axis_fifo_write(struct file *f, const char __user *buf,
			       size_t len, loff_t *off)
{
	struct axis_fifo *fifo = (struct axis_fifo *)f->private_data;
	unsigned int words_to_write;
	u32 *txbuf;
	int ret;

	if (len % sizeof(u32)) {
		dev_err(fifo->dt_device,
			"tried to send a packet that isn't word-aligned\n");
		return -EINVAL;
	}

	words_to_write = len / sizeof(u32);

	if (!words_to_write) {
		dev_err(fifo->dt_device,
			"tried to send a packet of length 0\n");
		return -EINVAL;
	}

	/*
	 * In 'Store-and-Forward' mode, the maximum packet that can be
	 * transmitted is limited by the size of the FIFO, which is
	 * (C_TX_FIFO_DEPTH–4)*(data interface width/8) bytes.
	 *
	 * Do not attempt to send a packet larger than 'tx_fifo_depth - 4',
	 * otherwise a 'Transmit Packet Overrun Error' interrupt will be
	 * raised, which requires a reset of the TX circuit to recover.
	 */
	if (words_to_write > (fifo->tx_fifo_depth - 4))
		return -EINVAL;

	if (f->f_flags & O_NONBLOCK) {
		/*
		 * Device opened in non-blocking mode. Try to lock it and then
		 * check if there is any room to write the given buffer.
		 */
		if (!mutex_trylock(&fifo->write_lock))
			return -EAGAIN;

		if (words_to_write > ioread32(fifo->base_addr +
					      XLLF_TDFV_OFFSET)) {
			ret = -EAGAIN;
			goto end_unlock;
		}
	} else {
		/* opened in blocking mode */

		/* wait for an interrupt (or timeout) if there isn't
		 * currently enough room in the fifo
		 */
		mutex_lock(&fifo->write_lock);
		ret = wait_event_interruptible_timeout(fifo->write_queue,
						       ioread32(fifo->base_addr + XLLF_TDFV_OFFSET)
								>= words_to_write,
						       write_timeout);

		if (ret <= 0) {
			if (ret == 0) {
				ret = -EAGAIN;
			} else if (ret != -ERESTARTSYS) {
				dev_err(fifo->dt_device, "wait_event_interruptible_timeout() error in write (ret=%i)\n",
					ret);
			}

			goto end_unlock;
		}
	}

	txbuf = vmemdup_user(buf, len);
	if (IS_ERR(txbuf)) {
		ret = PTR_ERR(txbuf);
		goto end_unlock;
	}

	for (int i = 0; i < words_to_write; ++i)
		iowrite32(txbuf[i], fifo->base_addr + XLLF_TDFD_OFFSET);

	/* write packet size to fifo */
	iowrite32(len, fifo->base_addr + XLLF_TLR_OFFSET);

	ret = len;
	kvfree(txbuf);
end_unlock:
	mutex_unlock(&fifo->write_lock);

	return ret;
}

static irqreturn_t axis_fifo_irq(int irq, void *dw)
{
	struct axis_fifo *fifo = dw;
	u32 isr, ier, intr;

	ier = ioread32(fifo->base_addr + XLLF_IER_OFFSET);
	isr = ioread32(fifo->base_addr + XLLF_ISR_OFFSET);
	intr = ier & isr;

	if (intr & XLLF_INT_RC_MASK)
		wake_up(&fifo->read_queue);

	if (intr & XLLF_INT_TC_MASK)
		wake_up(&fifo->write_queue);

	if (intr & XLLF_INT_RPURE_MASK)
		dev_err(fifo->dt_device, "receive under-read interrupt\n");

	if (intr & XLLF_INT_RPORE_MASK)
		dev_err(fifo->dt_device, "receive over-read interrupt\n");

	if (intr & XLLF_INT_RPUE_MASK)
		dev_err(fifo->dt_device, "receive underrun error interrupt\n");

	if (intr & XLLF_INT_TPOE_MASK)
		dev_err(fifo->dt_device, "transmit overrun error interrupt\n");

	if (intr & XLLF_INT_TSE_MASK)
		dev_err(fifo->dt_device,
			"transmit length mismatch error interrupt\n");

	iowrite32(XLLF_INT_CLEAR_ALL, fifo->base_addr + XLLF_ISR_OFFSET);

	return IRQ_HANDLED;
}

static int axis_fifo_open(struct inode *inod, struct file *f)
{
	struct axis_fifo *fifo = container_of(f->private_data,
					      struct axis_fifo, miscdev);
	unsigned int flags = f->f_flags & O_ACCMODE;

	f->private_data = fifo;

	if ((flags == O_WRONLY || flags == O_RDWR) && !fifo->has_tx_fifo)
		return -EPERM;

	if ((flags == O_RDONLY || flags == O_RDWR) && !fifo->has_rx_fifo)
		return -EPERM;

	return 0;
}

static int axis_fifo_close(struct inode *inod, struct file *f)
{
	f->private_data = NULL;

	return 0;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = axis_fifo_open,
	.release = axis_fifo_close,
	.read = axis_fifo_read,
	.write = axis_fifo_write
};

static int axis_fifo_debugfs_regs_show(struct seq_file *m, void *p)
{
	static const struct axis_fifo_debug_reg regs[] = {
		{"isr", XLLF_ISR_OFFSET},
		{"ier", XLLF_IER_OFFSET},
		{"tdfv", XLLF_TDFV_OFFSET},
		{"rdfo", XLLF_RDFO_OFFSET},
		{ /* Sentinel */ },
	};
	const struct axis_fifo_debug_reg *reg;
	struct axis_fifo *fifo = m->private;

	for (reg = regs; reg->name; ++reg) {
		u32 val = ioread32(fifo->base_addr + reg->offset);

		seq_printf(m, "%*s: 0x%08x\n", AXIS_FIFO_DEBUG_REG_NAME_MAX_LEN,
			   reg->name, val);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(axis_fifo_debugfs_regs);

static void axis_fifo_debugfs_init(struct axis_fifo *fifo)
{
	fifo->debugfs_dir = debugfs_create_dir(dev_name(fifo->dt_device), NULL);

	debugfs_create_file("regs", 0444, fifo->debugfs_dir, fifo,
			    &axis_fifo_debugfs_regs_fops);
}

static int axis_fifo_parse_dt(struct axis_fifo *fifo)
{
	int ret;
	unsigned int value;
	struct device_node *node = fifo->dt_device->of_node;

	ret = of_property_read_u32(node, "xlnx,axi-str-rxd-tdata-width",
				   &value);
	if (ret) {
		dev_err(fifo->dt_device, "missing xlnx,axi-str-rxd-tdata-width property\n");
		goto end;
	} else if (value != 32) {
		dev_err(fifo->dt_device, "xlnx,axi-str-rxd-tdata-width only supports 32 bits\n");
		ret = -EIO;
		goto end;
	}

	ret = of_property_read_u32(node, "xlnx,axi-str-txd-tdata-width",
				   &value);
	if (ret) {
		dev_err(fifo->dt_device, "missing xlnx,axi-str-txd-tdata-width property\n");
		goto end;
	} else if (value != 32) {
		dev_err(fifo->dt_device, "xlnx,axi-str-txd-tdata-width only supports 32 bits\n");
		ret = -EIO;
		goto end;
	}

	ret = of_property_read_u32(node, "xlnx,rx-fifo-depth",
				   &fifo->rx_fifo_depth);
	if (ret) {
		dev_err(fifo->dt_device, "missing xlnx,rx-fifo-depth property\n");
		ret = -EIO;
		goto end;
	}

	ret = of_property_read_u32(node, "xlnx,tx-fifo-depth",
				   &fifo->tx_fifo_depth);
	if (ret) {
		dev_err(fifo->dt_device, "missing xlnx,tx-fifo-depth property\n");
		ret = -EIO;
		goto end;
	}

	ret = of_property_read_u32(node, "xlnx,use-rx-data",
				   &fifo->has_rx_fifo);
	if (ret) {
		dev_err(fifo->dt_device, "missing xlnx,use-rx-data property\n");
		ret = -EIO;
		goto end;
	}

	ret = of_property_read_u32(node, "xlnx,use-tx-data",
				   &fifo->has_tx_fifo);
	if (ret) {
		dev_err(fifo->dt_device, "missing xlnx,use-tx-data property\n");
		ret = -EIO;
		goto end;
	}

end:
	return ret;
}

static int axis_fifo_probe(struct platform_device *pdev)
{
	struct resource *r_mem; /* IO mem resources */
	struct device *dev = &pdev->dev; /* OS device (from device tree) */
	struct axis_fifo *fifo = NULL;
	char *device_name;
	int rc = 0; /* error return value */
	int irq;

	/* ----------------------------
	 *     init wrapper device
	 * ----------------------------
	 */

	device_name = devm_kzalloc(dev, 32, GFP_KERNEL);
	if (!device_name)
		return -ENOMEM;

	/* allocate device wrapper memory */
	fifo = devm_kzalloc(dev, sizeof(*fifo), GFP_KERNEL);
	if (!fifo)
		return -ENOMEM;

	dev_set_drvdata(dev, fifo);
	fifo->dt_device = dev;

	init_waitqueue_head(&fifo->read_queue);
	init_waitqueue_head(&fifo->write_queue);

	mutex_init(&fifo->read_lock);
	mutex_init(&fifo->write_lock);

	/* ----------------------------
	 *   init device memory space
	 * ----------------------------
	 */

	/* get iospace for the device and request physical memory */
	fifo->base_addr = devm_platform_get_and_ioremap_resource(pdev, 0, &r_mem);
	if (IS_ERR(fifo->base_addr))
		return PTR_ERR(fifo->base_addr);

	/* ----------------------------
	 *          init IP
	 * ----------------------------
	 */

	rc = axis_fifo_parse_dt(fifo);
	if (rc)
		return rc;

	reset_ip_core(fifo);

	/* ----------------------------
	 *    init device interrupts
	 * ----------------------------
	 */

	/* get IRQ resource */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* request IRQ */
	rc = devm_request_irq(fifo->dt_device, irq, &axis_fifo_irq, 0,
			      DRIVER_NAME, fifo);
	if (rc) {
		dev_err(fifo->dt_device, "couldn't allocate interrupt %i\n",
			irq);
		return rc;
	}

	/* ----------------------------
	 *      init char device
	 * ----------------------------
	 */
	fifo->id = ida_alloc(&axis_fifo_ida, GFP_KERNEL);
	if (fifo->id < 0)
		return fifo->id;

	snprintf(device_name, 32, "%s%d", DRIVER_NAME, fifo->id);

	/* create character device */
	fifo->miscdev.fops = &fops;
	fifo->miscdev.minor = MISC_DYNAMIC_MINOR;
	fifo->miscdev.name = device_name;
	fifo->miscdev.parent = dev;
	rc = misc_register(&fifo->miscdev);
	if (rc < 0) {
		ida_free(&axis_fifo_ida, fifo->id);
		return rc;
	}

	axis_fifo_debugfs_init(fifo);

	return 0;
}

static void axis_fifo_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct axis_fifo *fifo = dev_get_drvdata(dev);

	debugfs_remove(fifo->debugfs_dir);
	misc_deregister(&fifo->miscdev);
	ida_free(&axis_fifo_ida, fifo->id);
}

static const struct of_device_id axis_fifo_of_match[] = {
	{ .compatible = "xlnx,axi-fifo-mm-s-4.1", },
	{ .compatible = "xlnx,axi-fifo-mm-s-4.2", },
	{ .compatible = "xlnx,axi-fifo-mm-s-4.3", },
	{},
};
MODULE_DEVICE_TABLE(of, axis_fifo_of_match);

static struct platform_driver axis_fifo_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table	= axis_fifo_of_match,
	},
	.probe		= axis_fifo_probe,
	.remove		= axis_fifo_remove,
};

static int __init axis_fifo_init(void)
{
	if (read_timeout >= 0)
		read_timeout = msecs_to_jiffies(read_timeout);
	else
		read_timeout = MAX_SCHEDULE_TIMEOUT;

	if (write_timeout >= 0)
		write_timeout = msecs_to_jiffies(write_timeout);
	else
		write_timeout = MAX_SCHEDULE_TIMEOUT;

	pr_info("axis-fifo driver loaded with parameters read_timeout = %li, write_timeout = %li\n",
		read_timeout, write_timeout);
	return platform_driver_register(&axis_fifo_driver);
}

module_init(axis_fifo_init);

static void __exit axis_fifo_exit(void)
{
	platform_driver_unregister(&axis_fifo_driver);
	ida_destroy(&axis_fifo_ida);
}

module_exit(axis_fifo_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jacob Feder <jacobsfeder@gmail.com>");
MODULE_DESCRIPTION("Xilinx AXI-Stream FIFO IP core driver");
