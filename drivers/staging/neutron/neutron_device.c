// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */
/****************************************************************************/

#include <linux/dma-mapping.h>
#include <linux/dma-map-ops.h>
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
#include <linux/firmware.h>
#include <linux/elf.h>

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

static struct rproc *neutron_get_rproc(struct neutron_device *ndev)
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

static int neutron_rproc_elf_load(struct rproc *rproc, const struct firmware *fw,
				  void *data_ddr, u8 skip_flag)
{
	struct device *dev = &rproc->dev;
	int i, ret = 0;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;

	const u8 *elf_data = fw->data;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u64 da = phdr->p_paddr;
		u64 memsz = phdr->p_memsz;
		u64 filesz = phdr->p_filesz;
		u64 offset = phdr->p_offset;
		u32 type = phdr->p_type;
		bool is_iomem = false;
		void *ptr;

		dev_dbg(dev, "da: %llx memsz: %llx filesz %llx offset %llx type %x\n",
			da, memsz, filesz, offset, type);

		if (type != PT_LOAD || !memsz)
			continue;

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%llx memsz 0x%llx\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%llx avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		if (da == 0x50000) {
			if ((skip_flag & 0x1) || !data_ddr)
				continue;
			ptr = data_ddr;
			neu_dbg("copy ddr_data to %pS size 0x%llx\n", data_ddr, memsz);
			is_iomem = true;
		} else {
			/* grab the kernel address for this device address */
			if (skip_flag & 0x2)
				continue;
			ptr = rproc_da_to_va(rproc, da, memsz, &is_iomem);
			if (!ptr) {
				dev_err(dev, "neutron bad phdr da 0x%llx mem 0x%llx\n", da,
					memsz);
				ret = -EINVAL;
				break;
			}
		}

		/* put the segment where the remote processor expects it */
		if (filesz) {
			if (is_iomem)
				memcpy_toio((void __iomem *)ptr, elf_data + offset, filesz);
			else
				memcpy(ptr, elf_data + offset, filesz);
		}

		if (memsz > filesz) {
			if (is_iomem)
				memset_io((void __iomem *)(ptr + filesz), 0, memsz - filesz);
			else
				memset(ptr + filesz, 0, memsz - filesz);
		}
	}

	return ret;
}

static int neutron_firmw_request(struct neutron_device *ndev, struct neutron_buffer *buf,
				 void *data_ddr, const char *fw_name)
{
	int ret = 0;
	struct device *dev;
	struct rproc *rproc = ndev->rproc;

	if (!buf) {
		dev_err(dev, "%s: invalid neutron bufffer\n", __func__);
		return PTR_ERR(buf);
	}

	dev = ndev->dev;

	/* firmware exists */
	if (buf->firmware_p)
		return ret;

	/* request firmware without cache with flag FW_OPT_NOCACHE */
	ret = request_firmware_into_buf(&buf->firmware_p, fw_name, dev, NULL, 0);
	if (ret < 0) {
		dev_err(dev, "request_firmware failed: %d\n", ret);
		return ret;
	}

	// remap ddr data address to kernel virt.
	neu_dbg("data_ddr: 0x%lx,  sz: 0x%x\n", data_ddr, buf->firmware_p->size);

	/* Only the ddr data needs to be loaded on prepartion, other data
	 * will be loaded on demand at runtime.
	 */
	ret = neutron_rproc_elf_load(rproc, buf->firmware_p, data_ddr, 2);
	if (ret) {
		dev_err(dev, "neutron_elf_load failed\n");
		return ret;
	}

	/* Sync the data for device */
	neutron_memory_sync(ndev, buf->dma_addr, buf->size, DMA_TO_DEVICE);

	/* Firmware is changed, it should be reloaded on next job */
	ndev->firmw_id = 0;

	return ret;
}

int neutron_firmw_reload(struct neutron_device *ndev, struct neutron_buffer *buf)
{
	int ret = -1;
	void *data_ddr = NULL;
	struct device *dev = ndev->dev;
	struct rproc *rproc = ndev->rproc;

	if (!buf->firmware_p) {
		dev_err(dev, "firmware is not ready\n");
		return ret;
	}

	ret = rproc->ops->stop(rproc);
	if (ret)
		dev_err(dev, "could not stop neutron\n");

	ret = neutron_rproc_elf_load(rproc, buf->firmware_p, data_ddr, 0x1);
	if (ret)
		dev_err(dev, "neutron_rproc_elf_load failed\n");

	rproc->ops->start(rproc);

	return ret;
}

void neutron_memory_sync(struct neutron_device *ndev, dma_addr_t addr,
			 size_t size, enum dma_data_direction dir)
{
	/* unset dma_coherent to ensure arch_sync_dma_for_device() is executed */
	ndev->dev->dma_coherent = false;

	switch (dir) {
	case DMA_TO_DEVICE:
		dma_sync_single_for_device(ndev->dev, addr, size, DMA_TO_DEVICE);
		break;
	case DMA_FROM_DEVICE:
		dma_sync_single_for_cpu(ndev->dev, addr, size, DMA_FROM_DEVICE);
		break;
	default:
		break;
	}

	/* recovery dma_coherent */
	ndev->dev->dma_coherent = true;
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

static void neutron_rproc_put(struct neutron_device *ndev)
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
	ret = pm_runtime_force_suspend(ndev->dev);
	if (ret) {
		dev_err(ndev->dev, "hw_reset: failed to power off\n");
		goto rproc_boot;
	}

	msleep(20);
	ret = pm_runtime_force_resume(ndev->dev);
	if (ret) {
		dev_err(ndev->dev, "hw_reset: failed to power on\n");
		goto rproc_boot;
	}

rproc_boot:

	pm_runtime_resume_and_get(ndev->dev);
	if (ndev->power_state == NEUTRON_POWER_ON)
		ret = neutron_rproc_boot(ndev, NEUTRON_FIRMW_NAME);
	pm_runtime_put_sync(ndev->dev);

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

	pm_runtime_resume_and_get(ndev->dev);

	mutex_lock(&ndev->mutex);
	ret = neutron_rproc_boot(ndev, NEUTRON_FIRMW_NAME);
	mutex_unlock(&ndev->mutex);
	if (ret) {
		pm_runtime_put_autosuspend(ndev->dev);
		return ret;
	}

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

	pm_runtime_mark_last_busy(ndev->dev);

	return nonseekable_open(inode, file);
}

static int neutron_release(struct inode *inode, struct file *file)
{
	struct neutron_device *ndev =
		container_of(inode->i_cdev, struct neutron_device, cdev);

	pm_runtime_mark_last_busy(ndev->dev);
	pm_runtime_put_autosuspend(ndev->dev);

	return 0;
}

/* function to read neutron log */
static ssize_t neutron_read(struct file *file, char __user *buf,
			    size_t count, loff_t *f_pos)
{
	struct neutron_device *ndev = file->private_data;
	struct neutron_log_buffer *data = &ndev->logger;
	size_t bytes = 0;
	char c_data;
	int head, tail, next_head;

	pm_runtime_resume_and_get(ndev->dev);

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
			break;
		if (++bytes > NEUTRON_LOG_SIZE)
			break;
		++data->last_to_console;
	}

	if (bytes != 0) {
		next_head = (head + bytes) % NEUTRON_LOG_SIZE;
		writel(next_head, ndev->reg_base + HEAD);
	}

	pm_runtime_mark_last_busy(ndev->dev);
	pm_runtime_put_autosuspend(ndev->dev);

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

	pm_runtime_resume_and_get(ndev->dev);

	switch (cmd) {
	case NEUTRON_IOCTL_LOG_GET: {
		struct neutron_uapi_log_get uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(ndev->dev,
			"Ioctl: try to read size=%u from log buffer\n",
			uapi.size);

		ret = neutron_read(file, (char *)uapi.buf, uapi.size, NULL);
		if (copy_to_user(udata, &uapi, sizeof(uapi)))
			break;

		break;
	}
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
			"Ioctl: Inference run. base_ddr=%llx, kernel_offset=%x\n",
			(__u64)uapi.base_ddr_h << 32 | uapi.base_ddr_l, uapi.kernel_offset);
		ret = neutron_inference_create(ndev, NEUTRON_CMD_LOAD_KERNEL, &uapi);

		break;
	}
	case NEUTRON_IOCTL_INFERENCE_CREATE: {
		struct neutron_uapi_inference_args uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(ndev->dev,
			"Ioctl: Inference run. base_ddr=%llx, tensor_offset=%x\n",
			(__u64)uapi.base_ddr_h << 32 | uapi.base_ddr_l, uapi.tensor_offset);
		ret = neutron_inference_create(ndev, NEUTRON_CMD_RUN_INFERENCE, &uapi);

		break;
	}
	case NEUTRON_IOCTL_CACHE_SYNC: {
		struct neutron_uapi_cache_sync uapi;
		struct neutron_buffer *buf;

		ret = copy_from_user(&uapi, udata, sizeof(uapi));
		if (ret)
			break;

		dev_dbg(ndev->dev,
			"Ioctl: Sync cache offset:0x%x, size:0x%x, direction %d\n",
			uapi.offset, uapi.size, uapi.direction);

		buf = neutron_buffer_get_from_fd(uapi.fd);
		if (!buf || IS_ERR(buf)) {
			dev_err(ndev->dev, "IOCTL_CACHE_SYNC: Invalid buf. fd: %d\n", uapi.fd);
			ret = -EINVAL;
			break;
		}

		if (uapi.offset + uapi.size > buf->size) {
			dev_err(ndev->dev,
				"CACHE_SYNC: Out of range: fd %d, 0x%x + 0x%x > 0x%lx\n",
				uapi.fd, uapi.offset, uapi.size, buf->size);
			ret = -EINVAL;
		}

		if (uapi.direction)
			neutron_memory_sync(ndev, buf->dma_addr + uapi.offset,
					    uapi.size, DMA_FROM_DEVICE);
		else
			neutron_memory_sync(ndev, buf->dma_addr + uapi.offset,
					    uapi.size, DMA_TO_DEVICE);

		break;
	}
	case NEUTRON_IOCTL_FIRMWARE_LOAD: {
		struct neutron_uapi_firmware_load uapi;
		struct neutron_buffer *buf;
		char *fw_name;
		int buf_fd;
		u64 data_offset;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		fw_name = uapi.fw_name;
		buf_fd = uapi.buf_fd;
		data_offset = uapi.data_offset;

		if (*fw_name == '\0')
			strcpy(fw_name, NEUTRON_FIRMW_NAME);
		neu_dbg("fw_name: %s\n", fw_name);

		buf = neutron_buffer_get_from_fd(buf_fd);
		if (!buf || IS_ERR(buf)) {
			dev_err(ndev->dev, "IOCTL_FIRMWARE_LOAD: Invalid buf. fd: %d\n", buf_fd);
			break;
		}

		neu_dbg("buffer cpu addr 0x%pS, offset 0x%x\n", buf->cpu_addr, data_offset);
		mutex_lock(&ndev->mutex);
		ret = neutron_firmw_request(ndev, buf, buf->cpu_addr + data_offset, fw_name);
		mutex_unlock(&ndev->mutex);

		break;
	}

	default: {
		dev_err(ndev->dev, "Invalid ioctl. cmd=%u, arg=%lu",
			cmd, arg);
		break;
	}
	}

	pm_runtime_mark_last_busy(ndev->dev);
	pm_runtime_put_autosuspend(ndev->dev);

	return ret;
}

static const struct file_operations ndev_fops = {
	.owner		= THIS_MODULE,
	.open		= &neutron_open,
	.release	= &neutron_release,
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

static int neutron_dev_clk_get(struct neutron_device *ndev)
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

	dma_set_mask_and_coherent(ndev->dev, DMA_BIT_MASK(48));
	ndev->dev->dma_coherent = true;

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

