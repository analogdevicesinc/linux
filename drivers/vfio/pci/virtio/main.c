// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, NVIDIA CORPORATION & AFFILIATES. All rights reserved
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/vfio.h>
#include <linux/vfio_pci_core.h>
#include <linux/virtio_pci.h>
#include <linux/virtio_net.h>
#include <linux/virtio_pci_admin.h>

struct virtiovf_pci_core_device {
	struct vfio_pci_core_device core_device;
	u8 *bar0_virtual_buf;
	/* synchronize access to the virtual buf */
	struct mutex bar_mutex;
	void __iomem *notify_addr;
	u64 notify_offset;
	__le32 pci_base_addr_0;
	__le16 pci_cmd;
	u8 bar0_virtual_buf_size;
	u8 notify_bar;
};

static int
virtiovf_issue_legacy_rw_cmd(struct virtiovf_pci_core_device *virtvdev,
			     loff_t pos, char __user *buf,
			     size_t count, bool read)
{
	bool msix_enabled =
		(virtvdev->core_device.irq_type == VFIO_PCI_MSIX_IRQ_INDEX);
	struct pci_dev *pdev = virtvdev->core_device.pdev;
	u8 *bar0_buf = virtvdev->bar0_virtual_buf;
	bool common;
	u8 offset;
	int ret;

	common = pos < VIRTIO_PCI_CONFIG_OFF(msix_enabled);
	/* offset within the relevant configuration area */
	offset = common ? pos : pos - VIRTIO_PCI_CONFIG_OFF(msix_enabled);
	mutex_lock(&virtvdev->bar_mutex);
	if (read) {
		if (common)
			ret = virtio_pci_admin_legacy_common_io_read(pdev, offset,
					count, bar0_buf + pos);
		else
			ret = virtio_pci_admin_legacy_device_io_read(pdev, offset,
					count, bar0_buf + pos);
		if (ret)
			goto out;
		if (copy_to_user(buf, bar0_buf + pos, count))
			ret = -EFAULT;
	} else {
		if (copy_from_user(bar0_buf + pos, buf, count)) {
			ret = -EFAULT;
			goto out;
		}

		if (common)
			ret = virtio_pci_admin_legacy_common_io_write(pdev, offset,
					count, bar0_buf + pos);
		else
			ret = virtio_pci_admin_legacy_device_io_write(pdev, offset,
					count, bar0_buf + pos);
	}
out:
	mutex_unlock(&virtvdev->bar_mutex);
	return ret;
}

static int
virtiovf_pci_bar0_rw(struct virtiovf_pci_core_device *virtvdev,
		     loff_t pos, char __user *buf,
		     size_t count, bool read)
{
	struct vfio_pci_core_device *core_device = &virtvdev->core_device;
	struct pci_dev *pdev = core_device->pdev;
	u16 queue_notify;
	int ret;

	if (!(le16_to_cpu(virtvdev->pci_cmd) & PCI_COMMAND_IO))
		return -EIO;

	if (pos + count > virtvdev->bar0_virtual_buf_size)
		return -EINVAL;

	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret) {
		pci_info_ratelimited(pdev, "runtime resume failed %d\n", ret);
		return -EIO;
	}

	switch (pos) {
	case VIRTIO_PCI_QUEUE_NOTIFY:
		if (count != sizeof(queue_notify)) {
			ret = -EINVAL;
			goto end;
		}
		if (read) {
			ret = vfio_pci_core_ioread16(core_device, true, &queue_notify,
						     virtvdev->notify_addr);
			if (ret)
				goto end;
			if (copy_to_user(buf, &queue_notify,
					 sizeof(queue_notify))) {
				ret = -EFAULT;
				goto end;
			}
		} else {
			if (copy_from_user(&queue_notify, buf, count)) {
				ret = -EFAULT;
				goto end;
			}
			ret = vfio_pci_core_iowrite16(core_device, true, queue_notify,
						      virtvdev->notify_addr);
		}
		break;
	default:
		ret = virtiovf_issue_legacy_rw_cmd(virtvdev, pos, buf, count,
						   read);
	}

end:
	pm_runtime_put(&pdev->dev);
	return ret ? ret : count;
}

static ssize_t virtiovf_pci_read_config(struct vfio_device *core_vdev,
					char __user *buf, size_t count,
					loff_t *ppos)
{
	struct virtiovf_pci_core_device *virtvdev = container_of(
		core_vdev, struct virtiovf_pci_core_device, core_device.vdev);
	loff_t pos = *ppos & VFIO_PCI_OFFSET_MASK;
	size_t register_offset;
	loff_t copy_offset;
	size_t copy_count;
	__le32 val32;
	__le16 val16;
	u8 val8;
	int ret;

	ret = vfio_pci_core_read(core_vdev, buf, count, ppos);
	if (ret < 0)
		return ret;

	if (vfio_pci_core_range_intersect_range(pos, count, PCI_DEVICE_ID,
						sizeof(val16), &copy_offset,
						&copy_count, &register_offset)) {
		val16 = cpu_to_le16(VIRTIO_TRANS_ID_NET);
		if (copy_to_user(buf + copy_offset, (void *)&val16 + register_offset, copy_count))
			return -EFAULT;
	}

	if ((le16_to_cpu(virtvdev->pci_cmd) & PCI_COMMAND_IO) &&
	    vfio_pci_core_range_intersect_range(pos, count, PCI_COMMAND,
						sizeof(val16), &copy_offset,
						&copy_count, &register_offset)) {
		if (copy_from_user((void *)&val16 + register_offset, buf + copy_offset,
				   copy_count))
			return -EFAULT;
		val16 |= cpu_to_le16(PCI_COMMAND_IO);
		if (copy_to_user(buf + copy_offset, (void *)&val16 + register_offset,
				 copy_count))
			return -EFAULT;
	}

	if (vfio_pci_core_range_intersect_range(pos, count, PCI_REVISION_ID,
						sizeof(val8), &copy_offset,
						&copy_count, &register_offset)) {
		/* Transional needs to have revision 0 */
		val8 = 0;
		if (copy_to_user(buf + copy_offset, &val8, copy_count))
			return -EFAULT;
	}

	if (vfio_pci_core_range_intersect_range(pos, count, PCI_BASE_ADDRESS_0,
						sizeof(val32), &copy_offset,
						&copy_count, &register_offset)) {
		u32 bar_mask = ~(virtvdev->bar0_virtual_buf_size - 1);
		u32 pci_base_addr_0 = le32_to_cpu(virtvdev->pci_base_addr_0);

		val32 = cpu_to_le32((pci_base_addr_0 & bar_mask) | PCI_BASE_ADDRESS_SPACE_IO);
		if (copy_to_user(buf + copy_offset, (void *)&val32 + register_offset, copy_count))
			return -EFAULT;
	}

	if (vfio_pci_core_range_intersect_range(pos, count, PCI_SUBSYSTEM_ID,
						sizeof(val16), &copy_offset,
						&copy_count, &register_offset)) {
		/*
		 * Transitional devices use the PCI subsystem device id as
		 * virtio device id, same as legacy driver always did.
		 */
		val16 = cpu_to_le16(VIRTIO_ID_NET);
		if (copy_to_user(buf + copy_offset, (void *)&val16 + register_offset,
				 copy_count))
			return -EFAULT;
	}

	if (vfio_pci_core_range_intersect_range(pos, count, PCI_SUBSYSTEM_VENDOR_ID,
						sizeof(val16), &copy_offset,
						&copy_count, &register_offset)) {
		val16 = cpu_to_le16(PCI_VENDOR_ID_REDHAT_QUMRANET);
		if (copy_to_user(buf + copy_offset, (void *)&val16 + register_offset,
				 copy_count))
			return -EFAULT;
	}

	return count;
}

static ssize_t
virtiovf_pci_core_read(struct vfio_device *core_vdev, char __user *buf,
		       size_t count, loff_t *ppos)
{
	struct virtiovf_pci_core_device *virtvdev = container_of(
		core_vdev, struct virtiovf_pci_core_device, core_device.vdev);
	unsigned int index = VFIO_PCI_OFFSET_TO_INDEX(*ppos);
	loff_t pos = *ppos & VFIO_PCI_OFFSET_MASK;

	if (!count)
		return 0;

	if (index == VFIO_PCI_CONFIG_REGION_INDEX)
		return virtiovf_pci_read_config(core_vdev, buf, count, ppos);

	if (index == VFIO_PCI_BAR0_REGION_INDEX)
		return virtiovf_pci_bar0_rw(virtvdev, pos, buf, count, true);

	return vfio_pci_core_read(core_vdev, buf, count, ppos);
}

static ssize_t virtiovf_pci_write_config(struct vfio_device *core_vdev,
					 const char __user *buf, size_t count,
					 loff_t *ppos)
{
	struct virtiovf_pci_core_device *virtvdev = container_of(
		core_vdev, struct virtiovf_pci_core_device, core_device.vdev);
	loff_t pos = *ppos & VFIO_PCI_OFFSET_MASK;
	size_t register_offset;
	loff_t copy_offset;
	size_t copy_count;

	if (vfio_pci_core_range_intersect_range(pos, count, PCI_COMMAND,
						sizeof(virtvdev->pci_cmd),
						&copy_offset, &copy_count,
						&register_offset)) {
		if (copy_from_user((void *)&virtvdev->pci_cmd + register_offset,
				   buf + copy_offset,
				   copy_count))
			return -EFAULT;
	}

	if (vfio_pci_core_range_intersect_range(pos, count, PCI_BASE_ADDRESS_0,
						sizeof(virtvdev->pci_base_addr_0),
						&copy_offset, &copy_count,
						&register_offset)) {
		if (copy_from_user((void *)&virtvdev->pci_base_addr_0 + register_offset,
				   buf + copy_offset,
				   copy_count))
			return -EFAULT;
	}

	return vfio_pci_core_write(core_vdev, buf, count, ppos);
}

static ssize_t
virtiovf_pci_core_write(struct vfio_device *core_vdev, const char __user *buf,
			size_t count, loff_t *ppos)
{
	struct virtiovf_pci_core_device *virtvdev = container_of(
		core_vdev, struct virtiovf_pci_core_device, core_device.vdev);
	unsigned int index = VFIO_PCI_OFFSET_TO_INDEX(*ppos);
	loff_t pos = *ppos & VFIO_PCI_OFFSET_MASK;

	if (!count)
		return 0;

	if (index == VFIO_PCI_CONFIG_REGION_INDEX)
		return virtiovf_pci_write_config(core_vdev, buf, count, ppos);

	if (index == VFIO_PCI_BAR0_REGION_INDEX)
		return virtiovf_pci_bar0_rw(virtvdev, pos, (char __user *)buf, count, false);

	return vfio_pci_core_write(core_vdev, buf, count, ppos);
}

static int
virtiovf_pci_ioctl_get_region_info(struct vfio_device *core_vdev,
				   unsigned int cmd, unsigned long arg)
{
	struct virtiovf_pci_core_device *virtvdev = container_of(
		core_vdev, struct virtiovf_pci_core_device, core_device.vdev);
	unsigned long minsz = offsetofend(struct vfio_region_info, offset);
	void __user *uarg = (void __user *)arg;
	struct vfio_region_info info = {};

	if (copy_from_user(&info, uarg, minsz))
		return -EFAULT;

	if (info.argsz < minsz)
		return -EINVAL;

	switch (info.index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
		info.offset = VFIO_PCI_INDEX_TO_OFFSET(info.index);
		info.size = virtvdev->bar0_virtual_buf_size;
		info.flags = VFIO_REGION_INFO_FLAG_READ |
			     VFIO_REGION_INFO_FLAG_WRITE;
		return copy_to_user(uarg, &info, minsz) ? -EFAULT : 0;
	default:
		return vfio_pci_core_ioctl(core_vdev, cmd, arg);
	}
}

static long
virtiovf_vfio_pci_core_ioctl(struct vfio_device *core_vdev, unsigned int cmd,
			     unsigned long arg)
{
	switch (cmd) {
	case VFIO_DEVICE_GET_REGION_INFO:
		return virtiovf_pci_ioctl_get_region_info(core_vdev, cmd, arg);
	default:
		return vfio_pci_core_ioctl(core_vdev, cmd, arg);
	}
}

static int
virtiovf_set_notify_addr(struct virtiovf_pci_core_device *virtvdev)
{
	struct vfio_pci_core_device *core_device = &virtvdev->core_device;
	int ret;

	/*
	 * Setup the BAR where the 'notify' exists to be used by vfio as well
	 * This will let us mmap it only once and use it when needed.
	 */
	ret = vfio_pci_core_setup_barmap(core_device,
					 virtvdev->notify_bar);
	if (ret)
		return ret;

	virtvdev->notify_addr = core_device->barmap[virtvdev->notify_bar] +
			virtvdev->notify_offset;
	return 0;
}

static int virtiovf_pci_open_device(struct vfio_device *core_vdev)
{
	struct virtiovf_pci_core_device *virtvdev = container_of(
		core_vdev, struct virtiovf_pci_core_device, core_device.vdev);
	struct vfio_pci_core_device *vdev = &virtvdev->core_device;
	int ret;

	ret = vfio_pci_core_enable(vdev);
	if (ret)
		return ret;

	if (virtvdev->bar0_virtual_buf) {
		/*
		 * Upon close_device() the vfio_pci_core_disable() is called
		 * and will close all the previous mmaps, so it seems that the
		 * valid life cycle for the 'notify' addr is per open/close.
		 */
		ret = virtiovf_set_notify_addr(virtvdev);
		if (ret) {
			vfio_pci_core_disable(vdev);
			return ret;
		}
	}

	vfio_pci_core_finish_enable(vdev);
	return 0;
}

static int virtiovf_get_device_config_size(unsigned short device)
{
	/* Network card */
	return offsetofend(struct virtio_net_config, status);
}

static int virtiovf_read_notify_info(struct virtiovf_pci_core_device *virtvdev)
{
	u64 offset;
	int ret;
	u8 bar;

	ret = virtio_pci_admin_legacy_io_notify_info(virtvdev->core_device.pdev,
				VIRTIO_ADMIN_CMD_NOTIFY_INFO_FLAGS_OWNER_MEM,
				&bar, &offset);
	if (ret)
		return ret;

	virtvdev->notify_bar = bar;
	virtvdev->notify_offset = offset;
	return 0;
}

static int virtiovf_pci_init_device(struct vfio_device *core_vdev)
{
	struct virtiovf_pci_core_device *virtvdev = container_of(
		core_vdev, struct virtiovf_pci_core_device, core_device.vdev);
	struct pci_dev *pdev;
	int ret;

	ret = vfio_pci_core_init_dev(core_vdev);
	if (ret)
		return ret;

	pdev = virtvdev->core_device.pdev;
	ret = virtiovf_read_notify_info(virtvdev);
	if (ret)
		return ret;

	virtvdev->bar0_virtual_buf_size = VIRTIO_PCI_CONFIG_OFF(true) +
				virtiovf_get_device_config_size(pdev->device);
	BUILD_BUG_ON(!is_power_of_2(virtvdev->bar0_virtual_buf_size));
	virtvdev->bar0_virtual_buf = kzalloc(virtvdev->bar0_virtual_buf_size,
					     GFP_KERNEL);
	if (!virtvdev->bar0_virtual_buf)
		return -ENOMEM;
	mutex_init(&virtvdev->bar_mutex);
	return 0;
}

static void virtiovf_pci_core_release_dev(struct vfio_device *core_vdev)
{
	struct virtiovf_pci_core_device *virtvdev = container_of(
		core_vdev, struct virtiovf_pci_core_device, core_device.vdev);

	kfree(virtvdev->bar0_virtual_buf);
	vfio_pci_core_release_dev(core_vdev);
}

static const struct vfio_device_ops virtiovf_vfio_pci_tran_ops = {
	.name = "virtio-vfio-pci-trans",
	.init = virtiovf_pci_init_device,
	.release = virtiovf_pci_core_release_dev,
	.open_device = virtiovf_pci_open_device,
	.close_device = vfio_pci_core_close_device,
	.ioctl = virtiovf_vfio_pci_core_ioctl,
	.device_feature = vfio_pci_core_ioctl_feature,
	.read = virtiovf_pci_core_read,
	.write = virtiovf_pci_core_write,
	.mmap = vfio_pci_core_mmap,
	.request = vfio_pci_core_request,
	.match = vfio_pci_core_match,
	.bind_iommufd = vfio_iommufd_physical_bind,
	.unbind_iommufd = vfio_iommufd_physical_unbind,
	.attach_ioas = vfio_iommufd_physical_attach_ioas,
	.detach_ioas = vfio_iommufd_physical_detach_ioas,
};

static const struct vfio_device_ops virtiovf_vfio_pci_ops = {
	.name = "virtio-vfio-pci",
	.init = vfio_pci_core_init_dev,
	.release = vfio_pci_core_release_dev,
	.open_device = virtiovf_pci_open_device,
	.close_device = vfio_pci_core_close_device,
	.ioctl = vfio_pci_core_ioctl,
	.device_feature = vfio_pci_core_ioctl_feature,
	.read = vfio_pci_core_read,
	.write = vfio_pci_core_write,
	.mmap = vfio_pci_core_mmap,
	.request = vfio_pci_core_request,
	.match = vfio_pci_core_match,
	.bind_iommufd = vfio_iommufd_physical_bind,
	.unbind_iommufd = vfio_iommufd_physical_unbind,
	.attach_ioas = vfio_iommufd_physical_attach_ioas,
	.detach_ioas = vfio_iommufd_physical_detach_ioas,
};

static bool virtiovf_bar0_exists(struct pci_dev *pdev)
{
	struct resource *res = pdev->resource;

	return res->flags;
}

static int virtiovf_pci_probe(struct pci_dev *pdev,
			      const struct pci_device_id *id)
{
	const struct vfio_device_ops *ops = &virtiovf_vfio_pci_ops;
	struct virtiovf_pci_core_device *virtvdev;
	int ret;

	if (pdev->is_virtfn && virtio_pci_admin_has_legacy_io(pdev) &&
	    !virtiovf_bar0_exists(pdev))
		ops = &virtiovf_vfio_pci_tran_ops;

	virtvdev = vfio_alloc_device(virtiovf_pci_core_device, core_device.vdev,
				     &pdev->dev, ops);
	if (IS_ERR(virtvdev))
		return PTR_ERR(virtvdev);

	dev_set_drvdata(&pdev->dev, &virtvdev->core_device);
	ret = vfio_pci_core_register_device(&virtvdev->core_device);
	if (ret)
		goto out;
	return 0;
out:
	vfio_put_device(&virtvdev->core_device.vdev);
	return ret;
}

static void virtiovf_pci_remove(struct pci_dev *pdev)
{
	struct virtiovf_pci_core_device *virtvdev = dev_get_drvdata(&pdev->dev);

	vfio_pci_core_unregister_device(&virtvdev->core_device);
	vfio_put_device(&virtvdev->core_device.vdev);
}

static const struct pci_device_id virtiovf_pci_table[] = {
	/* Only virtio-net is supported/tested so far */
	{ PCI_DRIVER_OVERRIDE_DEVICE_VFIO(PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1041) },
	{}
};

MODULE_DEVICE_TABLE(pci, virtiovf_pci_table);

static void virtiovf_pci_aer_reset_done(struct pci_dev *pdev)
{
	struct virtiovf_pci_core_device *virtvdev = dev_get_drvdata(&pdev->dev);

	virtvdev->pci_cmd = 0;
}

static const struct pci_error_handlers virtiovf_err_handlers = {
	.reset_done = virtiovf_pci_aer_reset_done,
	.error_detected = vfio_pci_core_aer_err_detected,
};

static struct pci_driver virtiovf_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = virtiovf_pci_table,
	.probe = virtiovf_pci_probe,
	.remove = virtiovf_pci_remove,
	.err_handler = &virtiovf_err_handlers,
	.driver_managed_dma = true,
};

module_pci_driver(virtiovf_pci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yishai Hadas <yishaih@nvidia.com>");
MODULE_DESCRIPTION(
	"VIRTIO VFIO PCI - User Level meta-driver for VIRTIO NET devices");
