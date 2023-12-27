// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */
/****************************************************************************/

#include <linux/dma-mapping.h>
#include <linux/bitmap.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/remoteproc.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#include "uapi/neutron.h"
#include "neutron_buffer.h"
#include "neutron_device.h"
#include "neutron_mailbox.h"
#include "neutron_inference.h"

/****************************************************************************/

#define NEUTRON_FIRMW_NAME     "NeutronFirmware.elf"
#define NEUTRON_LOG_SIZE    (0x1000)

#define APPSTATUS_MASK_INFDONE      BIT(0)
#define APPSTATUS_MASK_INFHALTED    BIT(1)
#define APPSTATUS_MASK_INFBUFFHALF  BIT(2)
#define APPSTATUS_MASK_INFOUTHALF   BIT(3)
#define APPSTATUS_MASK_MBOX         BIT(4)
#define APPSTATUS_MASK_FAULTCAUSE   (0x3f << 16)

#define APPCTRL_MBWR_UPDATED        (0xF807)
#define appctrl_get_mbwr(val)       (((val) & 0xFFFF0000) >> 16)

/****************************************************************************/

/**
 * wait_until_neutron_ready - Wait until neutron is ready.
 *
 * @time_ms: time to wait before timing out in milliseconds.
 *
 * Returns:
 * 0 is not ready after the @timeout elapsed.
 * else the remaining time_ms if neutron is ready.
 */
static bool wait_until_neutron_ready(struct neutron_device *ndev, int time_ms)
{
	int i, val;
	/* APPCTRL[31:16] register will be set to 0xF807 after neutron is startup */
	for (i = 0; i < time_ms; i++) {
		val = readl(ndev->reg_base + APPCTRL);
		if (appctrl_get_mbwr(val) == APPCTRL_MBWR_UPDATED)
			return time_ms - i;
		usleep_range(1000, 2000);
	}

	/* time out and not ready */
	return 0;
}

struct rproc *neutron_get_rproc(struct neutron_device *ndev)
{
	phandle rproc_phandle;
	struct rproc *rproc;

	if (!ndev || !ndev->dev)
		return NULL;

	if (!ndev->rproc) {
		if (of_property_read_u32(ndev->dev->of_node, "fsl,neutron-rproc",
					 &rproc_phandle)) {
			dev_err(ndev->dev, "could not get rproc phandle\n");
			return NULL;
		}

		rproc = rproc_get_by_phandle(rproc_phandle);
		if (!rproc) {
			dev_err(ndev->dev, "could not get rproc handle\n");
			return NULL;
		}
		ndev->rproc = rproc;

		return rproc;
	}
	return ndev->rproc;
}

int neutron_rproc_boot(struct neutron_device *ndev, const char *fw_name)
{
	struct rproc *rproc;
	int ret = 0;

	rproc = neutron_get_rproc(ndev);
	if (IS_ERR(rproc))
		return -ENODEV;

	if (atomic_read(&rproc->power) == 0) {
		ret = rproc_set_firmware(rproc, fw_name ? fw_name : NEUTRON_FIRMW_NAME);
		if (ret) {
			dev_err(ndev->dev, "could not set firmware: %s\n", fw_name);
			return ret;
		}
		ret = rproc_boot(rproc);
		if (ret) {
			dev_err(ndev->dev, "could not boot a remote processor\n");
			return ret;
		}
		/* Continue and assume boot neutron manually */
		if (!wait_until_neutron_ready(ndev, 100))
			dev_err(ndev->dev, "failed: neutron is not ready, timeout\n");
	}
	/* Update power state */
	if (ndev->power_state == NEUTRON_POWER_OFF)
		ndev->power_state = NEUTRON_POWER_ON;

	return ret;
}

int neutron_rproc_shutdown(struct neutron_device *ndev)
{
	struct rproc *rproc;

	rproc = neutron_get_rproc(ndev);
	if (IS_ERR(rproc))
		return -ENODEV;

	return rproc_shutdown(rproc);
}

void neutron_rproc_put(struct neutron_device *ndev)
{
	if (!ndev->rproc)
		return;

	/* release rproc reference */
	rproc_put(ndev->rproc);
}

/* neutron hardware reset used when firmware gets stuck */
int neutron_hw_reset(struct neutron_device *ndev)
{
	int ret = -ENODEV;

	/* Before reset ctrl is ready for neutron, we use the pm runtime interface
	 * powerOff and powerOn to do reset,
	 * shutdown the neutron core before powering off.
	 */
	neutron_rproc_shutdown(ndev);
	ret = pm_runtime_put_sync(ndev->dev);
	if (ret) {
		dev_err(ndev->dev, "hw_reset: failed to power off\n");
		goto rproc_boot;
	}

	msleep(20);
	ret = pm_runtime_get_sync(ndev->dev);
	if (ret) {
		dev_err(ndev->dev, "hw_reset: failed to power on\n");
		goto rproc_boot;
	}

rproc_boot:

	if (ndev->power_state == NEUTRON_POWER_ON)
		ret = neutron_rproc_boot(ndev, NEUTRON_FIRMW_NAME);

	return ret;
}

static int neutron_open(struct inode *inode,
			struct file *file)
{
	struct neutron_device *ndev =
		container_of(inode->i_cdev, struct neutron_device, cdev);
	struct rproc *rproc;
	int head, ret = 0;
	bool is_iomem = true;

	mutex_lock(&ndev->mutex);
	ret = neutron_rproc_boot(ndev, NEUTRON_FIRMW_NAME);
	mutex_unlock(&ndev->mutex);
	if (ret)
		return ret;

	rproc = ndev->rproc;
	head = readl(ndev->reg_base + HEAD);

	file->private_data = ndev;
	dev_dbg(ndev->dev, "Device open. file=0x%pK\n", file);

	/* 0x44000 is the LOG buffer address for neutron */
	if (!ndev->logger.start_addr)
		ndev->logger.start_addr = rproc_da_to_va(rproc, 0x44000, 0x1000, &is_iomem);

	if (!ndev->logger.end_addr)
		ndev->logger.end_addr = ndev->logger.start_addr + NEUTRON_LOG_SIZE;

	ndev->logger.end_of_data = ndev->logger.start_addr;
	ndev->logger.last_to_console  = ndev->logger.start_addr + head;

	return nonseekable_open(inode, file);
}

/* function to read neutron log */
static ssize_t neutron_read(struct file *file, char __user *buf,
			    size_t count, loff_t *f_pos)
{
	struct neutron_device *ndev = file->private_data;
	struct neutron_log_buffer *data = &ndev->logger;
	size_t bytes = 0;
	char c_data;
	int head, tail;

	/* Read logPtr and calculate log size*/
	tail = readl(ndev->reg_base + TAIL);
	head = readl(ndev->reg_base + HEAD);

	data->end_of_data = data->start_addr + tail;

	while (count != bytes && (data->end_of_data != data->last_to_console)) {
		/* Console is end */
		if (data->last_to_console == data->end_addr)
			data->last_to_console = data->start_addr;
		/* Read char data */
		c_data =  *data->last_to_console;
		if (copy_to_user(&buf[bytes], &c_data, 1))
			return -EFAULT;
		if (++bytes > NEUTRON_LOG_SIZE)
			break;
		++data->last_to_console;
	}

	return bytes;
}

static long neutron_ioctl(struct file *file,
			  unsigned int cmd,
			 unsigned long arg)
{
	struct neutron_device *ndev = file->private_data;
	void __user *udata = (void __user *)arg;
	int ret = -EINVAL;

	dev_dbg(ndev->dev, "Device ioctl. file=0x%pK, cmd=0x%x, arg=0x%lx\n",
		file, cmd, arg);

	switch (cmd) {
	case NEUTRON_IOCTL_BUFFER_CREATE: {
		struct neutron_uapi_buffer_create uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(ndev->dev,
			"Ioctl: Buffer create. size=%u\n",
			uapi.size);

		ret = neutron_buffer_create(ndev, uapi.size, &uapi.addr);
		if (copy_to_user(udata, &uapi, sizeof(uapi)))
			break;

		break;
	}
	case NEUTRON_IOCTL_KERNEL_LOAD: {
		struct neutron_uapi_inference_args uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(ndev->dev,
			"Ioctl: Inference run. dram_base=%u, kernel_offset=%u\n",
			uapi.dram_base, uapi.kernel_offset);
		ret = neutron_inference_create(ndev, NEUTRON_CMD_LOAD_KERNEL, &uapi);

		break;
	}
	case NEUTRON_IOCTL_INFERENCE_CREATE: {
		struct neutron_uapi_inference_args uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(ndev->dev,
			"Ioctl: Inference run. dram_base=%u, tensor_offset=%u\n",
			uapi.dram_base, uapi.tensor_offset);
		ret = neutron_inference_create(ndev, NEUTRON_CMD_RUN_INFERENCE, &uapi);

		break;
	}

	default: {
		dev_err(ndev->dev, "Invalid ioctl. cmd=%u, arg=%lu",
			cmd, arg);
		break;
	}
	}

	return ret;
}

static const struct file_operations ndev_fops = {
	.owner		= THIS_MODULE,
	.open		= &neutron_open,
	.read		= &neutron_read,
	.unlocked_ioctl	= &neutron_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= &neutron_ioctl,
#endif
};

static void neutron_mbox_rx_callback(struct neutron_device *ndev, void *data)
{
	struct neutron_mbox_rx_msg *msg = data;

	if (msg->retcode == DONE)
		neutron_inference_done(ndev);
}

int neutron_dev_clk_get(struct neutron_device *ndev)
{
	int ret = -ENODEV;

	if (!ndev || !ndev->dev)
		return ret;

	ret = devm_clk_bulk_get_all(ndev->dev, &ndev->clks);
	if (ret < 0) {
		dev_warn(ndev->dev, "unable to get clocks: %d\n", ret);
		ret = 0;
	}
	ndev->num_clks = ret;

	ret = clk_bulk_prepare_enable(ndev->num_clks, ndev->clks);
	if (ret)
		dev_err(ndev->dev, "failed to enable clock\n");

	return ret;
}

int neutron_dev_init(struct neutron_device *ndev,
		     struct device *dev, int irq,
		    struct class *class, dev_t devt)
{
	struct device *sysdev;
	int ret = -1;

	ndev->dev = dev;
	ndev->class = class;
	ndev->devt = devt;
	ndev->irq = irq;

	mutex_init(&ndev->mutex);

	if (ndev->irq < 0)
		goto destroy_mutex;

	ret = neutron_dev_clk_get(ndev);
	if (ret)
		goto destroy_mutex;

	ndev->mbox = neutron_mbox_create(ndev, irq, neutron_mbox_rx_callback);
	if (!ndev->mbox) {
		dev_err(ndev->dev, "Failed to init mailbox\n");
		goto put_clk;
	}

	ndev->queue = neutron_queue_create(ndev);
	if (!ndev->queue) {
		dev_err(ndev->dev, "Failed to create inference queue.\n");
		goto destroy_mbox;
	}

	dma_set_mask_and_coherent(ndev->dev, DMA_BIT_MASK(32));

	/* Init power state */
	ndev->power_state = NEUTRON_POWER_OFF;

	cdev_init(&ndev->cdev, &ndev_fops);
	ndev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&ndev->cdev, ndev->devt, 1);
	if (ret) {
		dev_err(ndev->dev, "Failed to add character device.\n");
		goto destroy_queue;
	}

	sysdev = device_create(ndev->class, NULL, ndev->devt, ndev,
			       "neutron%d", MINOR(ndev->devt));
	if (IS_ERR(sysdev)) {
		dev_err(ndev->dev, "Failed to create device.\n");
		ret = PTR_ERR(sysdev);
		goto del_cdev;
	}

	dev_info(ndev->dev,
		 "created neutron device, name=%s\n", dev_name(sysdev));

	return 0;

del_cdev:
	cdev_del(&ndev->cdev);
destroy_queue:
	neutron_queue_destroy(ndev->queue);
destroy_mbox:
	neutron_mbox_destroy(ndev->mbox);
put_clk:
	clk_bulk_disable_unprepare(ndev->num_clks, ndev->clks);
destroy_mutex:
	mutex_destroy(&ndev->mutex);

	dev_err(dev, "Failed to init neutron device.\n");
	return ret;
}

void neutron_dev_deinit(struct neutron_device *ndev)
{
	neutron_queue_destroy(ndev->queue);
	clk_bulk_disable_unprepare(ndev->num_clks, ndev->clks);
	neutron_mbox_destroy(ndev->mbox);
	neutron_rproc_put(ndev);
	mutex_destroy(&ndev->mutex);
	device_destroy(ndev->class, ndev->cdev.dev);
	cdev_del(&ndev->cdev);
	pr_info("neutron device is removed\n");
}

