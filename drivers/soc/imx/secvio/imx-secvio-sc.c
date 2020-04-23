// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP
 *
 */

/*
 * The i.MX8QXP SoC contains the Secure Non-Volatile Storage (SNVS) block. This
 * block can detect specific hardware attacks. Due to the presence of the SECO,
 * this block can only be accessible using the SCFW API.
 *
 * This module interact with the SCU which relay request to/from the SNVS block
 * to detect if security violation occurred.
 *
 * The module exports an API to add processing when a SV is detected:
 *  - register_imx_secvio_sc_notifier
 *  - unregister_imx_secvio_sc_notifier
 *  - imx_secvio_sc_check_state
 *  - int_imx_secvio_sc_clear_state
 *  - imx_secvio_sc_enable_irq
 *  - imx_secvio_sc_disable_irq
 */

/* Includes */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/nvmem-consumer.h>
#include <linux/miscdevice.h>

#include <linux/firmware/imx/ipc.h>
#include <linux/firmware/imx/sci.h>
#include <linux/firmware/imx/svc/seco.h>
#include <linux/firmware/imx/svc/rm.h>
#include <dt-bindings/firmware/imx/rsrc.h>

#include <soc/imx/imx-secvio-sc.h>
#include "imx-secvio-sc-int.h"

/* Definitions */

/* Reference on the driver_device */
static struct device *gs_imx_secvio_sc_dev;

/* Register IDs for sc_seco_secvio_config API */
#define HPSVS_ID 0x18
#define LPS_ID 0x4c
#define LPTDS_ID 0xa4
#define HPVIDR_ID 0xf8

#define SECO_MINOR_VERSION_SUPPORT_SECVIO_TAMPER 0x53
#define SECO_VERSION_MINOR_MASK GENMASK(15, 0)

/* Notifier list for new CB */
static BLOCKING_NOTIFIER_HEAD(imx_secvio_sc_notifier_chain);

int register_imx_secvio_sc_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&imx_secvio_sc_notifier_chain,
						nb);
}
EXPORT_SYMBOL(register_imx_secvio_sc_notifier);

int unregister_imx_secvio_sc_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&imx_secvio_sc_notifier_chain,
						  nb);
}
EXPORT_SYMBOL(unregister_imx_secvio_sc_notifier);

static void if_imx_scu_irq_register_notifier(void *nb)
{
	imx_scu_irq_register_notifier(nb);
}

static void if_unregister_imx_secvio_sc_notifier(void *nb)
{
	unregister_imx_secvio_sc_notifier(nb);
}

static
int imx_secvio_sc_notifier_call_chain(struct secvio_sc_notifier_info *info)
{
	return blocking_notifier_call_chain(&imx_secvio_sc_notifier_chain, 0,
					    (void *)info);
}

int int_imx_secvio_sc_get_state(struct device *dev,
				struct secvio_sc_notifier_info *info)
{
	struct secvio_sc_notifier_info _info = {0};
	struct secvio_sc_notifier_info *p_info;
	int ret = 0, ret2 = 0;

	p_info = info ? info : &_info;

	/* Read secvio status */
	ret = call_secvio_config(dev, HPSVS_ID, SECVIO_CONFIG_READ,
				 &p_info->hpsvs, NULL, NULL, NULL, NULL, 1);
	if (ret) {
		ret2 = ret;
		dev_err(dev, "Cannot read secvio status: %d\n", ret);
	}
	p_info->hpsvs &= HPSVS__ALL_SV__MASK;

	/* Read tampers status */
	ret = call_secvio_config(dev, LPS_ID, SECVIO_CONFIG_READ,
				 &p_info->lps, NULL, NULL, NULL, NULL, 1);
	if (ret) {
		ret2 = ret;
		dev_err(dev, "Cannot read tamper 1 status: %d\n", ret);
	}
	p_info->lps &= LPS__ALL_TP__MASK;

	ret = call_secvio_config(dev, LPTDS_ID, SECVIO_CONFIG_READ,
				 &p_info->lptds, NULL, NULL, NULL, NULL, 1);
	if (ret) {
		ret2 = ret;
		dev_err(dev, "Cannot read  tamper 2 status: %d\n", ret);
	}
	p_info->lptds &= LPTDS__ALL_TP__MASK;

	dev_dbg(dev, "Status: %.8x, %.8x, %.8x\n", p_info->hpsvs,
		p_info->lps, p_info->lptds);

	return ret2;
}

inline int imx_secvio_sc_get_state(struct secvio_sc_notifier_info *info)
{
	return int_imx_secvio_sc_get_state(gs_imx_secvio_sc_dev, info);
}
EXPORT_SYMBOL(imx_secvio_sc_get_state);

int int_imx_secvio_sc_check_state(struct device *dev)
{
	struct secvio_sc_notifier_info info = {0};
	int ret = 0;

	ret = int_imx_secvio_sc_get_state(dev, &info);
	if (ret) {
		dev_err(dev, "Failed to get secvio state\n");
		goto exit;
	}

	/* Call chain of CB registered to this module if status detected */
	if (info.hpsvs || info.lps || info.lptds)
		if (imx_secvio_sc_notifier_call_chain(&info))
			dev_warn(dev,
				 "Issues when calling the notifier chain\n");

exit:
	return ret;
}

inline int imx_secvio_sc_check_state(void)
{
	return int_imx_secvio_sc_check_state(gs_imx_secvio_sc_dev);
}
EXPORT_SYMBOL(imx_secvio_sc_check_state);

static int imx_secvio_sc_notify(struct notifier_block *nb,
				unsigned long event, void *group)
{
	struct imx_secvio_sc_data *data =
				container_of(nb, struct imx_secvio_sc_data,
					     irq_nb);
	struct device *dev = data->dev;
	int ret = 0;

	/* Filter event for us */
	if (!((event & IMX_SC_IRQ_SECVIO) &&
	      (*(u8 *)group == IMX_SC_IRQ_GROUP_WAKE)))
		goto exit;

	dev_warn(dev, "secvio security violation detected\n");

	ret = int_imx_secvio_sc_check_state(dev);

	/* Re-enable interrupt */
	ret = int_imx_secvio_sc_enable_irq(dev);
	if (ret)
		dev_err(dev, "Failed to enable IRQ\n");

exit:
	return ret;
}

int int_imx_secvio_sc_clear_state(struct device *dev, u32 hpsvs, u32 lps,
				  u32 lptds)
{
	int ret = 0;

	if (!dev)
		return -EINVAL;

	ret = call_secvio_config(dev, HPSVS_ID, SECVIO_CONFIG_WRITE, &hpsvs,
				 NULL, NULL, NULL, NULL, 1);
	if (ret) {
		dev_err(dev, "Cannot clear secvio status: %d\n", ret);
		goto exit;
	}

	ret = call_secvio_config(dev, LPS_ID, SECVIO_CONFIG_WRITE, &lps, NULL,
				 NULL, NULL, NULL, 1);
	if (ret) {
		dev_err(dev, "Cannot clear tamper 1 status: %d\n", ret);
		goto exit;
	}

	ret = call_secvio_config(dev, LPTDS_ID, SECVIO_CONFIG_WRITE, &lptds,
				 NULL, NULL, NULL, NULL, 1);
	if (ret) {
		dev_err(dev, "Cannot clear tamper 2 status: %d\n", ret);
		goto exit;
	}

exit:
	return ret;
}

inline int imx_secvio_sc_clear_state(u32 hpsvs, u32 lps, u32 lptds)
{
	return int_imx_secvio_sc_clear_state(gs_imx_secvio_sc_dev, hpsvs, lps,
					     lptds);
}
EXPORT_SYMBOL(imx_secvio_sc_clear_state);

static int report_to_user_notify(struct notifier_block *nb,
				 unsigned long status, void *notif_info)
{
	struct secvio_sc_notifier_info *info = notif_info;
	struct imx_secvio_sc_data *data =
				container_of(nb, struct imx_secvio_sc_data,
					     report_nb);
	struct device *dev = data->dev;

	/* Information about the security violation */
	if (info->hpsvs & HPSVS__LP_SEC_VIO__MASK)
		dev_info(dev, "SNVS secvio: LPSV\n");
	if (info->hpsvs & HPSVS__SW_LPSV__MASK)
		dev_info(dev, "SNVS secvio: SW LPSV\n");
	if (info->hpsvs & HPSVS__SW_FSV__MASK)
		dev_info(dev, "SNVS secvio: SW FSV\n");
	if (info->hpsvs & HPSVS__SW_SV__MASK)
		dev_info(dev, "SNVS secvio: SW SV\n");
	if (info->hpsvs & HPSVS__SV5__MASK)
		dev_info(dev, "SNVS secvio: SV 5\n");
	if (info->hpsvs & HPSVS__SV4__MASK)
		dev_info(dev, "SNVS secvio: SV 4\n");
	if (info->hpsvs & HPSVS__SV3__MASK)
		dev_info(dev, "SNVS secvio: SV 3\n");
	if (info->hpsvs & HPSVS__SV2__MASK)
		dev_info(dev, "SNVS secvio: SV 2\n");
	if (info->hpsvs & HPSVS__SV1__MASK)
		dev_info(dev, "SNVS secvio: SV 1\n");
	if (info->hpsvs & HPSVS__SV0__MASK)
		dev_info(dev, "SNVS secvio: SV 0\n");

	/* Information about the tampers */
	if (info->lps & LPS__ESVD__MASK)
		dev_info(dev, "SNVS tamper: External SV\n");
	if (info->lps & LPS__ET2D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 2\n");
	if (info->lps & LPS__ET1D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 1\n");
	if (info->lps & LPS__WMT2D__MASK)
		dev_info(dev, "SNVS tamper: Wire Mesh 2\n");
	if (info->lps & LPS__WMT1D__MASK)
		dev_info(dev, "SNVS tamper: Wire Mesh 1\n");
	if (info->lps & LPS__VTD__MASK)
		dev_info(dev, "SNVS tamper: Voltage\n");
	if (info->lps & LPS__TTD__MASK)
		dev_info(dev, "SNVS tamper: Temperature\n");
	if (info->lps & LPS__CTD__MASK)
		dev_info(dev, "SNVS tamper: Clock\n");
	if (info->lps & LPS__PGD__MASK)
		dev_info(dev, "SNVS tamper: Power Glitch\n");
	if (info->lps & LPS__MCR__MASK)
		dev_info(dev, "SNVS tamper: Monotonic Counter rollover\n");
	if (info->lps & LPS__SRTCR__MASK)
		dev_info(dev, "SNVS tamper: Secure RTC rollover\n");
	if (info->lps & LPS__LPTA__MASK)
		dev_info(dev, "SNVS tamper: Time alarm\n");

	if (info->lptds & LPTDS__ET10D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 10\n");
	if (info->lptds & LPTDS__ET9D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 9\n");
	if (info->lptds & LPTDS__ET8D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 8\n");
	if (info->lptds & LPTDS__ET7D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 7\n");
	if (info->lptds & LPTDS__ET6D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 6\n");
	if (info->lptds & LPTDS__ET5D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 5\n");
	if (info->lptds & LPTDS__ET4D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 4\n");
	if (info->lptds & LPTDS__ET3D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 3\n");

	return 0;
}

int call_secvio_config(struct device *dev, u8 id, u8 access, u32 *data0,
		       u32 *data1, u32 *data2, u32 *data3, u32 *data4, u8 size)
{
	int ret = 0;
	struct imx_secvio_sc_data *data;

	if (!dev)
		return -EINVAL;

	data = dev_get_drvdata(dev);

	ret = imx_sc_seco_secvio_config(data->ipc_handle, id, access, data0,
					data1, data2, data3, data4, size);
	if (ret)
		dev_err(dev, "Fail %s secvio config %d",
			((access) ? "write" : "read"), ret);

	return ret;
}

int int_imx_secvio_sc_enable_irq(struct device *dev)
{
	int ret = 0, ret2;
	u32 irq_status;
	struct imx_secvio_sc_data *data;

	if (!dev)
		return -EINVAL;

	data = dev_get_drvdata(dev);

	/* Enable the IRQ */
	ret = imx_scu_irq_group_enable(IMX_SC_IRQ_GROUP_WAKE, IMX_SC_IRQ_SECVIO,
				       true);
	if (ret) {
		dev_err(dev, "Cannot enable SCU IRQ: %d\n", ret);
		goto exit;
	}

	/* Enable interrupt */
	ret = imx_sc_seco_secvio_enable(data->ipc_handle);
	if (ret) {
		dev_err(dev, "Cannot enable SNVS irq: %d\n", ret);
		goto exit;
	};

	/* Unmask interrupt */
	ret = imx_scu_irq_get_status(IMX_SC_IRQ_GROUP_WAKE, &irq_status);
	if (ret) {
		dev_err(dev, "Cannot unmask irq: %d\n", ret);
		goto exit;
	};

exit:
	if (ret) {
		ret2 = int_imx_secvio_sc_disable_irq(dev);
		if (ret2)
			dev_warn(dev, "Failed to disable the IRQ\n");
	}

	return ret;
}

int int_imx_secvio_sc_disable_irq(struct device *dev)
{
	int ret = 0;
	struct imx_secvio_sc_data *data;

	if (!dev)
		return -EINVAL;

	data = dev_get_drvdata(dev);

	/* Disable the IRQ */
	ret = imx_scu_irq_group_enable(IMX_SC_IRQ_GROUP_WAKE, IMX_SC_IRQ_SECVIO,
				       false);
	if (ret) {
		dev_err(dev, "Cannot disable SCU IRQ: %d\n", ret);
		goto exit;
	}

exit:
	return ret;
}

static void if_imx_secvio_sc_disable_irq(void *dev)
{
	int_imx_secvio_sc_disable_irq(dev);
}

static int imx_secvio_sc_open(struct inode *node, struct file *filp)
{
	filp->private_data = node->i_private;

	return 0;
}

static long imx_secvio_sc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct device *dev = file->private_data;
	struct secvio_sc_notifier_info info;
	int ret;

	switch (cmd) {
	case IMX_SECVIO_SC_GET_STATE:
		ret = int_imx_secvio_sc_get_state(dev, &info);
		if (ret) {
			dev_err(dev, "Fail to get state\n");
			goto exit;
		}

		ret = copy_to_user((void *)arg, &info, sizeof(info));
		if (ret) {
			dev_err(dev, "Fail to copy info to user\n");
			ret = -EFAULT;
			goto exit;
		}
		break;
	case IMX_SECVIO_SC_CHECK_STATE:
		ret = int_imx_secvio_sc_check_state(dev);
		if (ret) {
			dev_err(dev, "Fail to check state\n");
			goto exit;
		}
		break;
	case IMX_SECVIO_SC_CLEAR_STATE:
		ret = copy_from_user(&info, (void *)arg, sizeof(info));
		if (ret) {
			dev_err(dev, "Fail to copy info from user\n");
			ret = -EFAULT;
			goto exit;
		}

		ret = int_imx_secvio_sc_clear_state(dev, info.hpsvs, info.lps,
						    info.lptds);
		if (ret) {
			dev_err(dev, "Fail to clear state\n");
			goto exit;
		}
		break;
	default:
		ret = -ENOIOCTLCMD;
	}

exit:
	return ret;
}

const static struct file_operations imx_secvio_sc_fops = {
	.owner = THIS_MODULE,
	.open = imx_secvio_sc_open,
	.unlocked_ioctl = imx_secvio_sc_ioctl,
};

static void if_misc_deregister(void *miscdevice)
{
	misc_deregister(miscdevice);
}

static int imx_secvio_sc_setup(struct device *dev)
{
	struct imx_secvio_sc_data *data;
	u32 seco_version = 0;
	bool own_secvio;
	u32 irq_status;
	int ret = 0;

	if (!devres_open_group(dev, NULL, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto exit;
	}

	/* Allocate private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate mem for data\n");
		goto clean;
	}

	data->dev = dev;

	dev_set_drvdata(dev, data);

	data->nvmem = devm_nvmem_device_get(dev, NULL);
	if (IS_ERR(data->nvmem)) {
		ret = PTR_ERR(data->nvmem);

		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to retrieve nvmem\n");

		goto clean;
	}

	/* Get a handle */
	ret = imx_scu_get_handle(&data->ipc_handle);
	if (ret) {
		dev_err(dev, "cannot get handle to scu: %d\n", ret);
		goto clean;
	};

	/* Check the version of the SECO */
	ret = imx_sc_seco_build_info(data->ipc_handle, &seco_version, NULL);
	if (ret) {
		dev_err(dev, "Failed to get seco version\n");
		goto clean;
	}

	if ((seco_version & SECO_VERSION_MINOR_MASK) <
	     SECO_MINOR_VERSION_SUPPORT_SECVIO_TAMPER) {
		dev_err(dev, "SECO version %.8x doesn't support all secvio\n",
			seco_version);
		ret = -EOPNOTSUPP;
		goto clean;
	}

	/* Init debug FS */
	ret = imx_secvio_sc_debugfs(dev);
	if (ret) {
		dev_err(dev, "Failed to set debugfs\n");
		goto clean;
	}

	/* Check we own the SECVIO */
	ret = imx_sc_rm_is_resource_owned(data->ipc_handle, IMX_SC_R_SECVIO);
	if (ret < 0) {
		dev_err(dev, "Failed to retrieve secvio ownership\n");
		goto clean;
	}

	own_secvio = ret > 0;
	if (!own_secvio) {
		dev_err(dev, "Secvio resource is not owned\n");
		ret = -EPERM;
		goto clean;
	}

	/* Check IRQ exists and enable it */
	ret = imx_scu_irq_get_status(IMX_SC_IRQ_GROUP_WAKE, &irq_status);
	if (ret) {
		dev_err(dev, "Cannot get IRQ state: %d\n", ret);
		goto clean;
	}

	ret = int_imx_secvio_sc_enable_irq(dev);
	if (ret) {
		dev_err(dev, "Failed to enable IRQ\n");
		goto clean;
	}

	ret = devm_add_action_or_reset(dev, if_imx_secvio_sc_disable_irq, dev);
	if (ret) {
		dev_err(dev, "Failed to add managed action to disable IRQ\n");
		goto clean;
	}

	/* Register the notifier for IRQ from SNVS */
	data->irq_nb.notifier_call = imx_secvio_sc_notify;
	ret = imx_scu_irq_register_notifier(&data->irq_nb);
	if (ret) {
		dev_err(dev, "Failed to register IRQ notification handler\n");
		goto clean;
	}

	ret = devm_add_action_or_reset(dev, if_imx_scu_irq_register_notifier,
				       &data->irq_nb);
	if (ret) {
		dev_err(dev, "Failed to add action to remove irq notif\n");
		goto clean;
	}

	/* Register the notification for reporting to user */
	data->report_nb.notifier_call = report_to_user_notify;
	ret = register_imx_secvio_sc_notifier(&data->report_nb);
	if (ret) {
		dev_err(dev, "Failed to register report notif handler\n");
		goto clean;
	}

	ret = devm_add_action_or_reset(dev, if_unregister_imx_secvio_sc_notifier,
				       &data->report_nb);
	if (ret) {
		dev_err(dev, "Failed to add action to remove report notif\n");
		goto clean;
	}

	/* Register the notification to report to audit FW */
	data->audit_nb.notifier_call = report_to_audit_notify;
	ret = register_imx_secvio_sc_notifier(&data->audit_nb);
	if (ret) {
		dev_err(dev, "Failed to register report audit handler\n");
		goto clean;
	}

	ret = devm_add_action(dev, if_unregister_imx_secvio_sc_notifier,
			      &data->audit_nb);
	if (ret) {
		dev_err(dev, "Failed to add action to remove audit notif\n");
		goto clean;
	}

	/* Register misc device for IOCTL */
	data->miscdev.name = devm_kstrdup(dev, "secvio-sc", GFP_KERNEL);
	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.fops = &imx_secvio_sc_fops;
	data->miscdev.parent = dev;
	ret = misc_register(&data->miscdev);
	if (ret) {
		dev_err(dev, "failed to register misc device\n");
		goto exit;
	}

	ret = devm_add_action_or_reset(dev, if_misc_deregister, &data->miscdev);
	if (ret) {
		dev_err(dev, "Failed to add action to unregister miscdev\n");
		goto clean;
	}

	gs_imx_secvio_sc_dev = dev;

	/* Process current state of the secvio and tampers */
	int_imx_secvio_sc_check_state(dev);

	devres_remove_group(dev, NULL);

	goto exit;

clean:
	devres_release_group(dev, NULL);

exit:
	return ret;
}

static int imx_secvio_sc_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	ret = imx_secvio_sc_setup(dev);
	if (ret && ret != -EPROBE_DEFER)
		dev_err(dev, "Failed to setup\n");

	return ret;
}

static const struct of_device_id imx_secvio_sc_dt_ids[] = {
	{ .compatible = "fsl,imx-sc-secvio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_secvio_sc_dt_ids);

static struct platform_driver imx_secvio_sc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= "imx-secvio-sc",
		.of_match_table = imx_secvio_sc_dt_ids,
	},
	.probe		= imx_secvio_sc_probe,
};
module_platform_driver(imx_secvio_sc_driver);

MODULE_AUTHOR("Franck LENORMAND <franck.lenormand@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX driver to handle SNVS secvio irq sent by SCFW");
MODULE_LICENSE("GPL");
