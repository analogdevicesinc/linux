// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Xilinx PUF device.
 *
 * Copyright (C) 2022 - 2023, Advanced Micro Devices, Inc.
 *
 * Description:
 * This driver is developed for PUF registration and regeneration support.
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/firmware/xlnx-zynqmp.h>
#include <uapi/misc/xilinx_puf.h>

static bool puf_clear;
module_param(puf_clear, bool, 0600);
MODULE_PARM_DESC(puf_clear, "Flag to enable clearing of PUF ID and key.");

/*
 * Below macro corresponds to Key source in firmware
 */
#define VERSAL_AES_PUF_KEY	11

/**
 * struct puf_params - parameters for PUF
 * @pufoperation: PUF registration or regeneration operation
 * @globalvarfilter: global variation filter
 * @readoption: option to read PUF data from efuse cache or ram address
 * @shuttervalue: shutter value for PUF registration/regeneration
 * @readsyndromeaddr: address to store the syndrome data during registration
 * @chashaddr: CHASH address
 * @auxaddr: AUX address
 * @pufidaddr: PUF ID address
 * @writesyndromeaddr: address where syndrome data is present and it is passed to the user
 * @trimsyndataaddr: trimmed syndrome data will be stored
 */
struct puf_params {
	u8 pufoperation;
	u8 globalvarfilter;
	u8 readoption;
	u32 shuttervalue;
	u64 readsyndromeaddr;
	u64 chashaddr;
	u64 auxaddr;
	u64 pufidaddr;
	u64 writesyndromeaddr;
	u64 trimsyndataaddr;
};

/**
 * struct xpuf_dev - Driver data for PUF
 * @dev: pointer to device struct
 * @miscdev: misc device handle
 */
struct xpuf_dev {
	struct device	*dev;
	struct miscdevice	miscdev;
};

static int xlnx_puf_regis(struct xpuf_dev *puf, struct puf_usrparams *pufreq)
{
	dma_addr_t dma_addr_data;
	struct puf_params *pufin;
	struct pufdata *pufdat;
	dma_addr_t dma_addr_in;
	u32 buflen;
	void *buf;
	int ret;

	if (pufreq->pufoperation != PUF_REGIS)
		return -EINVAL;
	buflen = sizeof(struct puf_params) + sizeof(struct pufdata);
	buf = kmalloc(buflen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	pufin = buf;
	dma_addr_in = dma_map_single(puf->dev, buf, buflen, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(puf->dev, dma_addr_in)) {
		ret = -ENOMEM;
		goto cleanup;
	}
	pufin->pufoperation = pufreq->pufoperation;
	pufin->globalvarfilter = pufreq->globalvarfilter;
	pufin->shuttervalue = pufreq->shuttervalue;

	pufdat = buf + sizeof(struct puf_params);
	dma_addr_data = dma_addr_in + sizeof(struct puf_params);
	pufin->readsyndromeaddr = (u64)(dma_addr_data);
	pufin->chashaddr = (u64)(pufin->readsyndromeaddr + sizeof(pufdat->pufhd.syndata));
	pufin->auxaddr = (u64)(pufin->chashaddr + sizeof(pufdat->pufhd.chash));
	pufin->pufidaddr = (u64)(pufin->auxaddr + sizeof(pufdat->pufhd.aux));
	pufin->trimsyndataaddr = (u64)(pufin->pufidaddr + sizeof(pufdat->pufid));
	dma_sync_single_for_device(puf->dev, dma_addr_in, buflen, DMA_BIDIRECTIONAL);
	ret = versal_pm_puf_registration(dma_addr_in);
	dma_unmap_single(puf->dev, dma_addr_in, buflen, DMA_BIDIRECTIONAL);
	if (ret != 0)
		goto cleanup;
	if (copy_to_user((void *)pufreq->pufdataaddr, pufdat, sizeof(struct pufdata))) {
		ret = -EFAULT;
		goto cleanup;
	}
cleanup:
	kfree(buf);
	return ret;
}

static int xlnx_puf_regen_id(struct xpuf_dev *puf,
			     struct puf_usrparams *pufreq)
{
	struct puf_helperdata *pufhd;
	struct puf_params *pufin;
	dma_addr_t dma_addr_data;
	dma_addr_t dma_addr_in;
	int ret = 0;
	u32 buflen;
	void *buf;

	if (pufreq->pufoperation != PUF_REGEN &&
	    pufreq->pufoperation != PUF_REGEN_ID)
		return -EINVAL;
	buflen = sizeof(struct puf_params) + (sizeof(struct puf_helperdata) +
		 PUF_ID_LEN_IN_BYTES);
	buf = kmalloc(buflen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	pufin = buf;
	dma_addr_in = dma_map_single(puf->dev, buf, buflen, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(puf->dev, dma_addr_in)) {
		ret = -ENOMEM;
		goto cleanup;
	}
	pufin->pufoperation = pufreq->pufoperation;
	pufin->globalvarfilter = pufreq->globalvarfilter;
	pufin->shuttervalue = pufreq->shuttervalue;
	pufin->readoption = pufreq->readoption;
	pufhd = buf + sizeof(struct puf_params);
	dma_addr_data = dma_addr_in + sizeof(struct puf_params);
	if (copy_from_user(pufhd, (void *)pufreq->pufdataaddr,
			   sizeof(struct puf_helperdata))) {
		ret = -EFAULT;
		dma_unmap_single(puf->dev, dma_addr_in, buflen, DMA_BIDIRECTIONAL);
		goto cleanup;
	}

	pufin->writesyndromeaddr = (u64)(dma_addr_data);
	pufin->chashaddr = (u64)(pufin->writesyndromeaddr + sizeof(pufhd->syndata));
	pufin->auxaddr = (u64)(pufin->chashaddr + sizeof(pufhd->chash));
	pufin->pufidaddr = (u64)(pufin->auxaddr + sizeof(pufhd->aux));
	dma_sync_single_for_device(puf->dev, dma_addr_in, buflen, DMA_BIDIRECTIONAL);
	ret = versal_pm_puf_regeneration(dma_addr_in);
	dma_unmap_single(puf->dev, dma_addr_in, buflen, DMA_BIDIRECTIONAL);
	if (ret != 0)
		goto cleanup;

	if (copy_to_user((void *)pufreq->pufidaddr, ((char *)pufhd +
			 sizeof(struct puf_helperdata)),
			 PUF_ID_LEN_IN_BYTES)) {
		ret = -EFAULT;
		goto cleanup;
	}
cleanup:
	kfree(buf);
	return ret;
}

static long xlnx_puf_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct puf_usrparams pufreq;
	struct xpuf_dev *puf = file->private_data;
	void __user *data = NULL;
	int ret;

	if (_IOC_TYPE(cmd) != PUF_IOC_MAGIC)
		return -ENOTTY;

	/* check if ioctl argument is present and valid */
	if (_IOC_DIR(cmd) != _IOC_NONE) {
		data = (void __user *)arg;
		if (!data)
			return -EINVAL;
	}
	if (copy_from_user(&pufreq, data,
			   sizeof(struct puf_usrparams)))
		return -EINVAL;
	switch (cmd) {
	case PUF_REGISTRATION:
		ret = xlnx_puf_regis(puf, &pufreq);
	break;
	case PUF_REGENERATION:
	case PUF_REGEN_ID_ONLY:
		ret = xlnx_puf_regen_id(puf, &pufreq);
		break;
	case PUF_CLEAR_ID:
		if (!puf_clear) {
			ret = -EOPNOTSUPP;
			break;
		}
		ret = versal_pm_puf_clear_id();
		break;
	case PUF_CLEAR_KEY:
		if (!puf_clear) {
			ret = -EOPNOTSUPP;
			break;
		}
		ret = versal_pm_aes_init();
		if (!ret)
			ret = versal_pm_aes_key_zero(VERSAL_AES_PUF_KEY);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

/**
 * xlnx_puf_open - open puf device
 * @inode:	inode object
 * @file:	file object
 *
 * Return:	0 if successful; otherwise -errno
 */
static int xlnx_puf_open(struct inode *inode, struct file *file)
{
	struct xpuf_dev *xpuf;

	xpuf = container_of(file->private_data, struct xpuf_dev, miscdev);
	file->private_data = xpuf;

	return 0;
}

/**
 * xlnx_puf_release - release puf resources
 * @inode:	inode object
 * @file:	file object
 *
 * Return:	0 if successful; otherwise -errno
 */
static int xlnx_puf_release(struct inode *inode, struct file *file)
{
	struct xpuf_dev *xpuf = file->private_data;

	dev_dbg(xpuf->dev, "%s: device /dev/xpuf unregistered\n", __func__);
	return 0;
}

static const struct file_operations dev_fops = {
	.owner          = THIS_MODULE,
	.open           = xlnx_puf_open,
	.release        = xlnx_puf_release,
	.unlocked_ioctl = xlnx_puf_ioctl,
};

/**
 * xlnx_puf_probe - probe puf device
 * @pdev: Pointer to puf platform device structure
 *
 * Return: 0 if successful; otherwise -errno
 */
static int xlnx_puf_probe(struct platform_device *pdev)
{
	int ret;
	struct xpuf_dev *xpuf;
	struct device *dev = &pdev->dev;

	xpuf = devm_kzalloc(dev, sizeof(*xpuf), GFP_KERNEL);
	if (!xpuf)
		return -ENOMEM;

	xpuf->dev = dev;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret < 0) {
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret < 0) {
			dev_err(dev, "no usable DMA configuration\n");
			return ret;
		}
	}

	xpuf->miscdev.minor = MISC_DYNAMIC_MINOR;
	xpuf->miscdev.name = "xpuf";
	xpuf->miscdev.fops = &dev_fops;
	xpuf->miscdev.parent = dev;

	if (misc_register(&xpuf->miscdev))
		return -ENODEV;

	platform_set_drvdata(pdev, xpuf);

	dev_dbg(dev, "puf registered as /dev/xpuf successfully");

	return 0;
}

/**
 * xlnx_puf_remove - clean up structures
 * @pdev:	The structure containing the device's details
 */
static void xlnx_puf_remove(struct platform_device *pdev)
{
	struct xpuf_dev *xpuf = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	misc_deregister(&xpuf->miscdev);

	dev_dbg(xpuf->dev, "device /dev/xpuf removed\n");
}

static struct platform_driver xlnx_puf_drv = {
	.probe = xlnx_puf_probe,
	.remove = xlnx_puf_remove,
	.driver = {
		.name = "xlnx-puf",
	},
};

static struct platform_device *platform_dev;

static int __init xlnx_puf_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&xlnx_puf_drv);
	if (ret)
		return ret;

	platform_dev = platform_device_register_simple(xlnx_puf_drv.driver.name,
						       0, NULL, 0);
	if (IS_ERR(platform_dev)) {
		ret = PTR_ERR(platform_dev);
		platform_driver_unregister(&xlnx_puf_drv);
	}

	return ret;
}

static void __exit xlnx_puf_driver_exit(void)
{
	platform_device_unregister(platform_dev);
	platform_driver_unregister(&xlnx_puf_drv);
}

module_init(xlnx_puf_driver_init);
module_exit(xlnx_puf_driver_exit);

MODULE_AUTHOR("Praveen Teja Kundanala <praveen.teja.kundanala@amd.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Xilinx Versal PUF driver");
