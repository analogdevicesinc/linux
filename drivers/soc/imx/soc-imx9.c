// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

#define DEVICE_ID		0x800
#define DIGPROG_MAJOR_UPPER(x)	(((x) & 0x00f00000) >> 20)
#define DIGPROG_MAJOR_LOWER(x)	(((x) & 0x0000f000) >> 12)
#define BASE_LAYER_REV(x)	(((x) & 0x000000f0) >> 4)

static int imx9_soc_device_register(struct device *dev)
{
	struct soc_device_attribute *attr;
	struct device_node *anaosc_np;
	struct soc_device *sdev;
	void __iomem *anaosc;
	u32 device_id;
	u32 v[4];
	int err;
	struct nvmem_cell *cell;
	void *buf;
	size_t len;

	attr = kzalloc(sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return -ENOMEM;

	err = of_property_read_string(of_root, "model", &attr->machine);
	if (err) {
		err = -EINVAL;
		goto attr;
	}

	attr->family = kasprintf(GFP_KERNEL, "Freescale i.MX");

	anaosc_np = of_find_compatible_node(NULL, NULL, "fsl,imx93-anatop");
	if (!anaosc_np) {
		err = -ENOENT;
		goto family;
	}
	anaosc = of_iomap(anaosc_np, 0);
	WARN_ON(!anaosc);

	device_id = readl(anaosc + DEVICE_ID);

	iounmap(anaosc);
	of_node_put(anaosc_np);

	if (BASE_LAYER_REV(device_id) == 0x1) {
		attr->revision = kasprintf(GFP_KERNEL, "1.0");
	} else {
		attr->revision = kasprintf(GFP_KERNEL, "unknown" );
	}

	cell = nvmem_cell_get(dev, "soc_unique_id");
	if (IS_ERR(cell)) {
		err = PTR_ERR(cell);
		goto revision;
	}

	buf = nvmem_cell_read(cell, &len);
	if (IS_ERR(buf)) {
		nvmem_cell_put(cell);
		err = PTR_ERR(buf);
		goto revision;
	}
	nvmem_cell_put(cell);

	memcpy(v, buf, min(len, sizeof(v)));
	attr->serial_number = kasprintf(GFP_KERNEL, "%08x%08x%08x%08x", v[0], v[1], v[2], v[3]);

	if (DIGPROG_MAJOR_UPPER(device_id) == 0x9 && DIGPROG_MAJOR_LOWER(device_id) == 0x2) {
		attr->soc_id = kasprintf(GFP_KERNEL, "i.MX93");
	} else {
		attr->soc_id = kasprintf(GFP_KERNEL, "unknown");
	}

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
revision:
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
