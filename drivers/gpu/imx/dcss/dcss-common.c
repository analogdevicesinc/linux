/*
 * Copyright (C) 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/busfreq-imx.h>
#include <video/imx-dcss.h>

#include <drm/drm_fourcc.h>

#include <video/imx-dcss.h>
#include "dcss-prv.h"

struct dcss_devtype {
	const char *name;
	u32 blkctl_ofs;
	u32 ctxld_ofs;
	u32 rdsrc_ofs;
	u32 wrscl_ofs;
	u32 dtg_ofs;
	u32 scaler_ofs;
	u32 ss_ofs;
	u32 dpr_ofs;
	u32 dtrc_ofs;
	u32 dec400d_ofs;
	u32 hdr10_ofs;
};

static struct dcss_devtype dcss_type_imx8m = {
	.name = "DCSS_imx8m",
	.blkctl_ofs = 0x2F000,
	.ctxld_ofs = 0x23000,
	.rdsrc_ofs = 0x22000,
	.wrscl_ofs = 0x21000,
	.dtg_ofs = 0x20000,
	.scaler_ofs = 0x1C000,
	.ss_ofs = 0x1B000,
	.dpr_ofs = 0x18000,
	.dtrc_ofs = 0x16000,
	.dec400d_ofs = 0x15000,
	.hdr10_ofs = 0x00000,
};

enum dcss_color_space dcss_drm_fourcc_to_colorspace(u32 drm_fourcc)
{
	switch (drm_fourcc) {
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_RGBA5551:
	case DRM_FORMAT_BGRA5551:
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
	case DRM_FORMAT_RGB888:
	case DRM_FORMAT_BGR888:
	case DRM_FORMAT_ARGB4444:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_XBGR2101010:
	case DRM_FORMAT_RGBX1010102:
	case DRM_FORMAT_BGRX1010102:
	case DRM_FORMAT_ARGB2101010:
	case DRM_FORMAT_ABGR2101010:
	case DRM_FORMAT_RGBA1010102:
	case DRM_FORMAT_BGRA1010102:
		return DCSS_COLORSPACE_RGB;
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_YVYU:
	case DRM_FORMAT_VYUY:
	case DRM_FORMAT_YUV420:
	case DRM_FORMAT_YVU420:
	case DRM_FORMAT_YUV422:
	case DRM_FORMAT_YVU422:
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
		return DCSS_COLORSPACE_YUV;
	default:
		return DCSS_COLORSPACE_UNKNOWN;
	}
}
EXPORT_SYMBOL_GPL(dcss_drm_fourcc_to_colorspace);

int dcss_vblank_irq_get(struct dcss_soc *dcss)
{
	struct platform_device *pdev = to_platform_device(dcss->dev);

	return platform_get_irq_byname(pdev, "dtg_prg1");
}
EXPORT_SYMBOL(dcss_vblank_irq_get);

void dcss_vblank_irq_enable(struct dcss_soc *dcss, bool en)
{
	dcss_dtg_vblank_irq_enable(dcss, en);
}
EXPORT_SYMBOL(dcss_vblank_irq_enable);

void dcss_vblank_irq_clear(struct dcss_soc *dcss)
{
	dcss_dtg_vblank_irq_clear(dcss);
}
EXPORT_SYMBOL(dcss_vblank_irq_clear);

static int dcss_submodules_init(struct dcss_soc *dcss)
{
	int ret;
	u32 dcss_base = dcss->start_addr;

	ret = dcss_blkctl_init(dcss, dcss_base + dcss->devtype->blkctl_ofs);
	if (ret)
		goto blkctl_err;

	ret = dcss_ctxld_init(dcss, dcss_base + dcss->devtype->ctxld_ofs);
	if (ret)
		goto ctxld_err;

	ret = dcss_dtrc_init(dcss, dcss_base + dcss->devtype->dtrc_ofs);
	if (ret)
		goto dtrc_err;

	ret = dcss_dec400d_init(dcss, dcss_base + dcss->devtype->dec400d_ofs);
	if (ret)
		goto dec400d_err;

	ret = dcss_dtg_init(dcss, dcss_base + dcss->devtype->dtg_ofs);
	if (ret)
		goto dtg_err;

	ret = dcss_ss_init(dcss, dcss_base + dcss->devtype->ss_ofs);
	if (ret)
		goto ss_err;

	ret = dcss_dpr_init(dcss, dcss_base + dcss->devtype->dpr_ofs);
	if (ret)
		goto dpr_err;

	ret = dcss_scaler_init(dcss, dcss_base + dcss->devtype->scaler_ofs);
	if (ret)
		goto scaler_err;

	ret = dcss_hdr10_init(dcss, dcss_base + dcss->devtype->hdr10_ofs);
	if (ret)
		goto hdr10_err;

	return 0;

hdr10_err:
	dcss_hdr10_exit(dcss);

scaler_err:
	dcss_scaler_exit(dcss);

dpr_err:
	dcss_dpr_exit(dcss);

ss_err:
	dcss_ss_exit(dcss);

dtg_err:
	dcss_dtg_exit(dcss);

dec400d_err:
	dcss_dec400d_exit(dcss);

dtrc_err:
	dcss_dtrc_exit(dcss);

ctxld_err:
	dcss_ctxld_exit(dcss);

blkctl_err:
	dcss_blkctl_exit(dcss);

	return ret;
}

struct dcss_platform_reg {
	struct dcss_client_platformdata pdata;
	const char *name;
};

static struct dcss_platform_reg client_reg = {
		.pdata = { },
		.name = "imx-dcss-crtc",
};

static int dcss_add_client_devices(struct dcss_soc *dcss)
{
	struct device *dev = dcss->dev;
	struct platform_device *pdev;
	struct device_node *of_node;
	int ret;

	of_node = of_graph_get_port_by_id(dev->of_node, 0);
	if (!of_node) {
		dev_err(dev, "no port@0 node in %s\n", dev->of_node->full_name);
		return -ENODEV;
	}

	pdev = platform_device_alloc(client_reg.name, 0);
	if (!pdev) {
		dev_err(dev, "cannot allocate platform device\n");
		return -ENOMEM;
	}

	pdev->dev.parent = dev;

	client_reg.pdata.of_node = of_node;
	ret = platform_device_add_data(pdev, &client_reg.pdata,
				       sizeof(client_reg.pdata));
	if (!ret)
		ret = platform_device_add(pdev);
	if (ret) {
		platform_device_put(pdev);
		goto err_register;
	}

	pdev->dev.of_node = of_node;

	return 0;

err_register:
	platform_device_unregister(pdev);
	return ret;
}

static int dcss_clks_init(struct dcss_soc *dcss)
{
	int ret, i, j;
	struct {
		const char *id;
		struct clk **clk;
	} clks[] = {
		{"apb",   &dcss->apb_clk},
		{"axi",   &dcss->axi_clk},
		{"pixel", &dcss->p_clk},
		{"rtrm",  &dcss->apb_clk},
		{"dtrc",  &dcss->dtrc_clk},
	};

	for (i = 0; i < ARRAY_SIZE(clks); i++) {
		*clks[i].clk = devm_clk_get(dcss->dev, clks[i].id);
		if (IS_ERR(*clks[i].clk)) {
			dev_err(dcss->dev, "failed to get %s clock\n",
				clks[i].id);
			ret = PTR_ERR(*clks[i].clk);
			goto err;
		}

		clk_prepare_enable(*clks[i].clk);
	}

	dcss->clks_on = true;

	return 0;

err:
	for (j = 0; j < i; j++)
		clk_disable_unprepare(*clks[j].clk);

	return ret;
}

static void dcss_clocks_enable(struct dcss_soc *dcss, bool en)
{
	if (en && !dcss->clks_on) {
		clk_prepare_enable(dcss->axi_clk);
		clk_prepare_enable(dcss->apb_clk);
		clk_prepare_enable(dcss->rtrm_clk);
		clk_prepare_enable(dcss->dtrc_clk);
		clk_prepare_enable(dcss->p_clk);
	}

	if (!en && dcss->clks_on) {
		clk_disable_unprepare(dcss->p_clk);
		clk_disable_unprepare(dcss->dtrc_clk);
		clk_disable_unprepare(dcss->rtrm_clk);
		clk_disable_unprepare(dcss->apb_clk);
		clk_disable_unprepare(dcss->axi_clk);
	}

	dcss->clks_on = en;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>

static int dcss_dump_regs_show(struct seq_file *s, void *data)
{
	dcss_blkctl_dump_regs(s, s->private);
	dcss_dtrc_dump_regs(s, s->private);
	dcss_dpr_dump_regs(s, s->private);
	dcss_scaler_dump_regs(s, s->private);
	dcss_dtg_dump_regs(s, s->private);
	dcss_ss_dump_regs(s, s->private);
	dcss_hdr10_dump_regs(s, s->private);
	dcss_ctxld_dump_regs(s, s->private);

	return 0;
}

static int dcss_dump_ctx_show(struct seq_file *s, void *data)
{
	dcss_ctxld_dump(s, s->private);

	return 0;
}

static int dcss_dump_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dcss_dump_regs_show, inode->i_private);
}

static int dcss_dump_ctx_open(struct inode *inode, struct file *file)
{
	return single_open(file, dcss_dump_ctx_show, inode->i_private);
}

static const struct file_operations dcss_dump_regs_fops = {
	.open		= dcss_dump_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations dcss_dump_ctx_fops = {
	.open		= dcss_dump_ctx_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void dcss_debugfs_init(struct dcss_soc *dcss)
{
	struct dentry *d, *root;

	root = debugfs_create_dir("imx-dcss", NULL);
	if (IS_ERR(root) || !root)
		goto err;

	d = debugfs_create_file("dump_registers", 0444, root, dcss,
				&dcss_dump_regs_fops);
	if (!d)
		goto err;

	d = debugfs_create_file("dump_context", 0444, root, dcss,
				&dcss_dump_ctx_fops);
	if (!d)
		goto err;

	return;

err:
	dev_err(dcss->dev, "Unable to create debugfs entries\n");
}
#else
static void dcss_debugfs_init(struct dcss_soc *dcss)
{
}
#endif

static void dcss_bus_freq(struct dcss_soc *dcss, bool en)
{
	if (en && !dcss->bus_freq_req)
		request_bus_freq(BUS_FREQ_HIGH);

	if (!en && dcss->bus_freq_req)
		release_bus_freq(BUS_FREQ_HIGH);

	dcss->bus_freq_req = en;
}

static int dcss_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	struct dcss_soc *dcss;
	const struct dcss_devtype *devtype;

	devtype = of_device_get_match_data(&pdev->dev);
	if (!devtype) {
		dev_err(&pdev->dev, "no device match found\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get memory resource\n");
		return -EINVAL;
	}

	dcss = devm_kzalloc(&pdev->dev, sizeof(struct dcss_soc), GFP_KERNEL);
	if (!dcss)
		return -ENOMEM;

	dcss->dev = &pdev->dev;
	dcss->devtype = devtype;

	platform_set_drvdata(pdev, dcss);

	ret = dcss_clks_init(dcss);
	if (ret) {
		dev_err(&pdev->dev, "clocks initialization failed\n");
		return ret;
	}

	dcss->start_addr = res->start;

	ret = dcss_submodules_init(dcss);
	if (ret) {
		dev_err(&pdev->dev, "submodules initialization failed\n");
		return ret;
	}

	dcss_debugfs_init(dcss);

	pm_runtime_set_autosuspend_delay(&pdev->dev, 3000);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	dcss_bus_freq(dcss, true);

	return dcss_add_client_devices(dcss);
}

static int dcss_remove(struct platform_device *pdev)
{
	struct dcss_soc *dcss = platform_get_drvdata(pdev);

	dcss_bus_freq(dcss, false);

	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dcss_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dcss_soc *dcss = platform_get_drvdata(pdev);
	int ret;

	if (pm_runtime_suspended(dev))
		return 0;

	ret = dcss_ctxld_suspend(dcss);
	if (ret)
		return ret;

	dcss_clocks_enable(dcss, false);

	dcss_bus_freq(dcss, false);

	return 0;
}

static int dcss_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dcss_soc *dcss = platform_get_drvdata(pdev);

	dcss_bus_freq(dcss, true);

	dcss_clocks_enable(dcss, true);

	dcss_blkctl_cfg(dcss);
	dcss_hdr10_cfg(dcss);

	dcss_ctxld_resume(dcss);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int dcss_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dcss_soc *dcss = platform_get_drvdata(pdev);
	int ret;

	ret = dcss_ctxld_suspend(dcss);
	if (ret)
		return ret;

	dcss_clocks_enable(dcss, false);

	dcss_bus_freq(dcss, false);

	return 0;
}

static int dcss_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dcss_soc *dcss = platform_get_drvdata(pdev);

	dcss_bus_freq(dcss, true);

	dcss_clocks_enable(dcss, true);

	dcss_blkctl_cfg(dcss);
	dcss_hdr10_cfg(dcss);

	dcss_ctxld_resume(dcss);

	return 0;
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops dcss_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(dcss_suspend, dcss_resume)
	SET_RUNTIME_PM_OPS(dcss_runtime_suspend,
			   dcss_runtime_resume, NULL)
};

static const struct of_device_id dcss_dt_ids[] = {
	{ .compatible = "nxp,imx8mq-dcss", .data = &dcss_type_imx8m, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dcss_dt_ids);

static struct platform_driver dcss_driver = {
	.driver = {
		.name = "dcss-core",
		.of_match_table = dcss_dt_ids,
		.pm = &dcss_pm,
	},
	.probe = dcss_probe,
	.remove = dcss_remove,
};

module_platform_driver(dcss_driver);

MODULE_DESCRIPTION("i.MX DCSS driver");
MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@nxp.com>");
MODULE_LICENSE("GPL");
