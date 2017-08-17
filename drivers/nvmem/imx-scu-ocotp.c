/*
 * i.MX6 OCOTP fusebox driver
 *
 * Copyright (c) 2015 Pengutronix, Philipp Zabel <p.zabel@pengutronix.de>
 *
 * Based on the barebox ocotp driver,
 * Copyright (c) 2010 Baruch Siach <baruch@tkos.co.il>,
 *	Orex Computed Radiography
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <soc/imx8/sc/sci.h>

struct ocotp_priv {
	struct device *dev;
	unsigned int nregs;
	sc_ipc_t nvmem_ipc;
};

static int imx_scu_ocotp_read(void *context, unsigned int offset,
			      void *val, size_t bytes)
{
	struct ocotp_priv *priv = context;
	sc_err_t sciErr = SC_ERR_NONE;
	unsigned int count;
	u32 index;
	u32 num_bytes;
	int i;
	u8 *buf, *p;

	index = offset >> 2;
	num_bytes = round_up((offset % 4) + bytes, 4);
	count = num_bytes >> 2;

	if (count > (priv->nregs - index))
		count = priv->nregs - index;

	p = kzalloc(num_bytes, GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	buf = p;

	for (i = index; i < (index + count); i++) {
		sciErr = sc_misc_otp_fuse_read(priv->nvmem_ipc, i, (u32 *)buf);
		if (sciErr != SC_ERR_NONE) {
			kfree(p);
			return -EIO;
		}
		buf += 4;
	}

	index = offset % 4;
	memcpy(val, &p[index], bytes);

	kfree(p);

	return 0;
}

static struct nvmem_config imx_scu_ocotp_nvmem_config = {
	.name = "imx-ocotp",
	.read_only = true,
	.word_size = 4,
	.stride = 1,
	.owner = THIS_MODULE,
	.reg_read = imx_scu_ocotp_read,
};

static const struct of_device_id imx_scu_ocotp_dt_ids[] = {
	{ .compatible = "fsl,imx8qm-ocotp", (void *)800 },
	{ },
};
MODULE_DEVICE_TABLE(of, imx_scu_ocotp_dt_ids);

static int imx_scu_ocotp_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct device *dev = &pdev->dev;
	struct ocotp_priv *priv;
	struct nvmem_device *nvmem;
	uint32_t mu_id;
	sc_err_t sciErr = SC_ERR_NONE;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		pr_info("pinctrl: Cannot obtain MU ID\n");
		return -EIO;
	}

	sciErr = sc_ipc_open(&priv->nvmem_ipc, mu_id);

	if (sciErr != SC_ERR_NONE) {
		pr_info("pinctrl: Cannot open MU channel to SCU\n");
		return -EIO;
	};

	of_id = of_match_device(imx_scu_ocotp_dt_ids, dev);
	priv->nregs = (unsigned long)of_id->data;
	priv->dev = dev;
	imx_scu_ocotp_nvmem_config.size = 4 * priv->nregs;
	imx_scu_ocotp_nvmem_config.dev = dev;
	imx_scu_ocotp_nvmem_config.priv = priv;
	nvmem = nvmem_register(&imx_scu_ocotp_nvmem_config);
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	platform_set_drvdata(pdev, nvmem);

	return 0;
}

static int imx_scu_ocotp_remove(struct platform_device *pdev)
{
	struct nvmem_device *nvmem = platform_get_drvdata(pdev);

	return nvmem_unregister(nvmem);
}

static struct platform_driver imx_scu_ocotp_driver = {
	.probe	= imx_scu_ocotp_probe,
	.remove	= imx_scu_ocotp_remove,
	.driver = {
		.name	= "imx_scu_ocotp",
		.of_match_table = imx_scu_ocotp_dt_ids,
	},
};
module_platform_driver(imx_scu_ocotp_driver);

MODULE_AUTHOR("Peng Fan <peng.fan@nxp.com>");
MODULE_DESCRIPTION("i.MX8QM OCOTP fuse box driver");
MODULE_LICENSE("GPL v2");
