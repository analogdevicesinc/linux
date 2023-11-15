// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022-2023 NXP
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/arm-smccc.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

#define IMX_SIP_GET_SOC_INFO	0xc2000006
#define SOC_ID(x)		(((x) & 0xFFFF) >> 8)
#define SOC_REV_MAJOR(x)	((((x) >> 28) & 0xF) - 0x9)
#define SOC_REV_MINOR(x)	(((x) >> 24) & 0xF)

static int imx9_soc_device_register(struct device *dev)
{
	struct soc_device_attribute *attr;
	struct arm_smccc_res res;
	struct soc_device *sdev;
	u32 soc_id, rev_major, rev_minor;
	u64 uid127_64, uid63_0;
	int err;

	attr = kzalloc(sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return -ENOMEM;

	err = of_property_read_string(of_root, "model", &attr->machine);
	if (err) {
		err = -EINVAL;
		goto attr;
	}

	attr->family = kasprintf(GFP_KERNEL, "Freescale i.MX");

	/*
	 * Retrieve the soc id, rev & uid info:
	 * res.a1[31:16]: soc revision;
	 * res.a1[15:0]: soc id;
	 * res.a2: uid[127:64];
	 * res.a3: uid[63:0];
	 */
	arm_smccc_smc(IMX_SIP_GET_SOC_INFO, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != SMCCC_RET_SUCCESS) {
		err = -EINVAL;
		goto family;
	}

	soc_id = SOC_ID(res.a1);
	rev_major = SOC_REV_MAJOR(res.a1);
	rev_minor = SOC_REV_MINOR(res.a1);

	attr->soc_id = kasprintf(GFP_KERNEL, "i.MX%2x", soc_id);
	attr->revision = kasprintf(GFP_KERNEL, "%d.%d", rev_major, rev_minor);

	uid127_64 = res.a2;
	uid63_0 = res.a3;
	attr->serial_number = kasprintf(GFP_KERNEL, "%016llx%016llx", uid127_64, uid63_0);

	if(of_machine_is_compatible("fsl,imx91p"))
		attr->soc_id = kasprintf(GFP_KERNEL, "i.MX91P");

	sdev = soc_device_register(attr);
	if (IS_ERR(sdev)) {
		err = -ENODEV;
		goto soc_id;
	}

	return 0;

soc_id:
	kfree(attr->soc_id);
	kfree(attr->serial_number);
	kfree(attr->revision);
family:
	kfree(attr->family);
attr:
	kfree(attr);
	return err;
}

static int imx9_init_soc_probe(struct platform_device *pdev)
{
        int ret;

	ret = imx9_soc_device_register(&pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to register SoC device\n");

        return ret;
}

static const struct of_device_id imx9_soc_of_match[] = {
	{ .compatible = "fsl,imx93-soc", },
	{ .compatible = "fsl,imx95-soc", },
	{ }
};
MODULE_DEVICE_TABLE(of, imx9_soc_of_match);

static struct platform_driver imx9_init_soc_driver = {
	.driver = {
		.name           = "imx9_init_soc",
		.of_match_table = of_match_ptr(imx9_soc_of_match),
	},
        .probe = imx9_init_soc_probe,
};
module_platform_driver(imx9_init_soc_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP i.MX9 SoC");
MODULE_LICENSE("GPL v2");
