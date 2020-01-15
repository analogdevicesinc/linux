// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2020 NXP
 *
 */

#include "imx8-isi-hw.h"

static const struct soc_device_attribute imx8_soc[] = {
	{
		.soc_id   = "i.MX8QXP",
		.revision = "1.0",
	}, {
		.soc_id   = "i.MX8QXP",
		.revision = "1.1",
	}, {
		.soc_id   = "i.MX8QXP",
		.revision = "1.2",
	}, {
		.soc_id   = "i.MX8QM",
		.revision = "1.0",
	}, {
		.soc_id   = "i.MX8QM",
		.revision = "1.1",
	}, {
		.soc_id   = "i.MX8MN",
		.revision = "1.0",
	},
};

static const struct of_device_id mxc_isi_of_match[];

struct mxc_isi_dev *mxc_isi_get_hostdata(struct platform_device *pdev)
{
	struct mxc_isi_dev *mxc_isi;

	if (!pdev || !pdev->dev.parent)
		return NULL;

	device_lock(pdev->dev.parent);
	mxc_isi = (struct mxc_isi_dev *)dev_get_drvdata(pdev->dev.parent);
	if (!mxc_isi) {
		dev_err(&pdev->dev, "Cann't get host data\n");
		device_unlock(pdev->dev.parent);
		return NULL;
	}
	device_unlock(pdev->dev.parent);

	return mxc_isi;
}

struct device *mxc_isi_dev_get_parent(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *parent;
	struct platform_device *parent_pdev;

	if (!pdev)
		return NULL;

	/* Get parent for isi capture device */
	parent = of_get_parent(dev->of_node);
	parent_pdev = of_find_device_by_node(parent);
	if (!parent_pdev) {
		of_node_put(parent);
		return NULL;
	}
	of_node_put(parent);

	return &parent_pdev->dev;
}

static irqreturn_t mxc_isi_irq_handler(int irq, void *priv)
{
	struct mxc_isi_dev *mxc_isi = priv;
	struct device *dev = &mxc_isi->pdev->dev;
	struct mxc_isi_ier_reg *ier_reg = mxc_isi->pdata->ier_reg;
	u32 status;

	spin_lock(&mxc_isi->slock);

	status = mxc_isi_get_irq_status(mxc_isi);
	mxc_isi->status = status;
	mxc_isi_clean_irq_status(mxc_isi, status);

	if (status & CHNL_STS_FRM_STRD_MASK) {
		if (mxc_isi->m2m_enabled)
			mxc_isi_m2m_frame_write_done(mxc_isi);
		else
			mxc_isi_cap_frame_write_done(mxc_isi);
	}

	if (status & (CHNL_STS_AXI_WR_ERR_Y_MASK |
		      CHNL_STS_AXI_WR_ERR_U_MASK |
		      CHNL_STS_AXI_WR_ERR_V_MASK))
		dev_dbg(dev, "%s, IRQ AXI Error stat=0x%X\n", __func__, status);

	if (status & (ier_reg->panic_y_buf_en.mask |
		      ier_reg->panic_u_buf_en.mask |
		      ier_reg->panic_v_buf_en.mask))
		dev_dbg(dev, "%s, IRQ Panic OFLW Error stat=0x%X\n", __func__, status);

	if (status & (ier_reg->oflw_y_buf_en.mask |
		      ier_reg->oflw_u_buf_en.mask |
		      ier_reg->oflw_v_buf_en.mask))
		dev_dbg(dev, "%s, IRQ OFLW Error stat=0x%X\n", __func__, status);

	if (status & (ier_reg->excs_oflw_y_buf_en.mask |
		      ier_reg->excs_oflw_u_buf_en.mask |
		      ier_reg->excs_oflw_v_buf_en.mask))
		dev_dbg(dev, "%s, IRQ EXCS OFLW Error stat=0x%X\n", __func__, status);

	spin_unlock(&mxc_isi->slock);
	return IRQ_HANDLED;
}

static int disp_mix_sft_rstn(struct reset_control *reset, bool enable)
{
	int ret;

	if (!reset)
		return 0;

	ret = enable ? reset_control_assert(reset) :
			 reset_control_deassert(reset);
	return ret;
}

static int disp_mix_clks_enable(struct reset_control *reset, bool enable)
{
	int ret;

	if (!reset)
		return 0;

	ret = enable ? reset_control_assert(reset) :
			 reset_control_deassert(reset);
	return ret;
}

static int mxc_imx8_clk_get(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;

	mxc_isi->clk = devm_clk_get(dev, NULL);

	if (IS_ERR(mxc_isi->clk)) {
		dev_err(dev, "failed to get isi clk\n");
		return PTR_ERR(mxc_isi->clk);
	}

	return 0;
}

static int mxc_imx8_clk_enable(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;
	int ret;

	ret = clk_prepare_enable(mxc_isi->clk);
	if (ret < 0) {
		dev_err(dev, "%s, enable clk error\n", __func__);
		return ret;
	}

	return 0;
}

static void mxc_imx8_clk_disable(struct mxc_isi_dev *mxc_isi)
{
	clk_disable_unprepare(mxc_isi->clk);
}

static struct mxc_isi_dev_ops mxc_imx8_clk_ops = {
	.clk_get     = mxc_imx8_clk_get,
	.clk_enable  = mxc_imx8_clk_enable,
	.clk_disable = mxc_imx8_clk_disable,
};

static struct mxc_isi_chan_src mxc_imx8_chan_src = {
	.src_dc0   = 0,
	.src_dc1   = 1,
	.src_mipi0 = 2,
	.src_mipi1 = 3,
	.src_hdmi  = 4,
	.src_csi   = 4,
	.src_mem   = 5,
};

/* For i.MX8QM/QXP B0 ISI IER version */
static struct mxc_isi_ier_reg mxc_imx8_isi_ier_v0 = {
	.oflw_y_buf_en = { .offset = 16, .mask = 0x10000  },
	.oflw_u_buf_en = { .offset = 19, .mask = 0x80000  },
	.oflw_v_buf_en = { .offset = 22, .mask = 0x400000 },

	.excs_oflw_y_buf_en = { .offset = 17, .mask = 0x20000  },
	.excs_oflw_u_buf_en = { .offset = 20, .mask = 0x100000 },
	.excs_oflw_v_buf_en = { .offset = 23, .mask = 0x800000 },

	.panic_y_buf_en = {.offset = 18, .mask = 0x40000   },
	.panic_u_buf_en = {.offset = 21, .mask = 0x200000  },
	.panic_v_buf_en = {.offset = 24, .mask = 0x1000000 },
};

/* For i.MX8QXP C0 and i.MX8MN ISI IER version */
static struct mxc_isi_ier_reg mxc_imx8_isi_ier_v1 = {
	.oflw_y_buf_en = { .offset = 19, .mask = 0x80000  },
	.oflw_u_buf_en = { .offset = 21, .mask = 0x200000 },
	.oflw_v_buf_en = { .offset = 23, .mask = 0x800000 },

	.panic_y_buf_en = {.offset = 20, .mask = 0x100000  },
	.panic_u_buf_en = {.offset = 22, .mask = 0x400000  },
	.panic_v_buf_en = {.offset = 24, .mask = 0x1000000 },
};

static struct mxc_isi_plat_data mxc_imx8_data = {
	.ops      = &mxc_imx8_clk_ops,
	.chan_src = &mxc_imx8_chan_src,
	.ier_reg  = &mxc_imx8_isi_ier_v0,
};

static int mxc_imx8mn_clk_get(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;

	mxc_isi->clk_disp_axi = devm_clk_get(dev, "disp_axi");
	if (IS_ERR(mxc_isi->clk_disp_axi)) {
		dev_err(dev, "failed to get disp_axi clk\n");
		return PTR_ERR(mxc_isi->clk_disp_axi);
	}

	mxc_isi->clk_disp_apb = devm_clk_get(dev, "disp_apb");
	if (IS_ERR(mxc_isi->clk_disp_apb)) {
		dev_err(dev, "failed to get disp_apb clk\n");
		return PTR_ERR(mxc_isi->clk_disp_apb);
	}

	mxc_isi->clk_root_disp_axi = devm_clk_get(dev, "disp_axi_root");
	if (IS_ERR(mxc_isi->clk_root_disp_axi)) {
		dev_err(dev, "failed to get disp axi root clk\n");
		return PTR_ERR(mxc_isi->clk_root_disp_axi);
	}

	mxc_isi->clk_root_disp_apb = devm_clk_get(dev, "disp_apb_root");
	if (IS_ERR(mxc_isi->clk_root_disp_apb)) {
		dev_err(dev, "failed to get disp apb root clk\n");
		return PTR_ERR(mxc_isi->clk_root_disp_apb);
	}

	return 0;
}

static int mxc_imx8mn_clk_enable(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;
	int ret;

	ret = clk_prepare_enable(mxc_isi->clk_disp_axi);
	if (ret < 0) {
		dev_err(dev, "prepare and enable axi clk error\n");
		return ret;
	}

	ret = clk_prepare_enable(mxc_isi->clk_disp_apb);
	if (ret < 0) {
		dev_err(dev, "prepare and enable abp clk error\n");
		return ret;
	}

	ret = clk_prepare_enable(mxc_isi->clk_root_disp_axi);
	if (ret < 0) {
		dev_err(dev, "prepare and enable axi root clk error\n");
		return ret;
	}

	ret = clk_prepare_enable(mxc_isi->clk_root_disp_apb);
	if (ret < 0) {
		dev_err(dev, "prepare and enable apb root clk error\n");
		return ret;
	}

	return 0;
}

static void mxc_imx8mn_clk_disable(struct mxc_isi_dev *mxc_isi)
{
	clk_disable_unprepare(mxc_isi->clk_root_disp_axi);
	clk_disable_unprepare(mxc_isi->clk_root_disp_apb);
	clk_disable_unprepare(mxc_isi->clk_disp_axi);
	clk_disable_unprepare(mxc_isi->clk_disp_apb);
}

static struct mxc_isi_chan_src mxc_imx8mn_chan_src = {
	.src_mipi0 = 0,
	.src_mipi1 = 1,
};

static struct mxc_isi_dev_ops mxc_imx8mn_clk_ops = {
	.clk_get     = mxc_imx8mn_clk_get,
	.clk_enable  = mxc_imx8mn_clk_enable,
	.clk_disable = mxc_imx8mn_clk_disable,
};

static struct mxc_isi_plat_data mxc_imx8mn_data = {
	.ops      = &mxc_imx8mn_clk_ops,
	.chan_src = &mxc_imx8mn_chan_src,
	.ier_reg  = &mxc_imx8_isi_ier_v1,
};

static int mxc_isi_parse_dt(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;
	struct device_node *node = dev->of_node;
	int ret = 0;

	mxc_isi->id = of_alias_get_id(node, "isi");
	mxc_isi->chain_buf = of_property_read_bool(node, "fsl,chain_buf");

	ret = of_property_read_u32_array(node, "interface", mxc_isi->interface, 3);
	if (ret < 0)
		return ret;

	dev_dbg(dev, "%s, isi_%d,interface(%d, %d, %d)\n", __func__,
		mxc_isi->id,
		mxc_isi->interface[0],
		mxc_isi->interface[1],
		mxc_isi->interface[2]);
	return 0;
}

static int mxc_isi_clk_get(struct mxc_isi_dev *mxc_isi)
{
	const struct mxc_isi_dev_ops *ops = mxc_isi->pdata->ops;

	if (!ops || !ops->clk_get)
		return -EINVAL;

	return ops->clk_get(mxc_isi);
}

static int mxc_isi_clk_enable(struct mxc_isi_dev *mxc_isi)
{
	const struct mxc_isi_dev_ops *ops = mxc_isi->pdata->ops;

	if (!ops || !ops->clk_enable)
		return -EINVAL;

	return ops->clk_enable(mxc_isi);
}

static void mxc_isi_clk_disable(struct mxc_isi_dev *mxc_isi)
{
	const struct mxc_isi_dev_ops *ops = mxc_isi->pdata->ops;

	if (!ops || !ops->clk_disable)
		return;

	ops->clk_disable(mxc_isi);
}

static int mxc_isi_of_parse_resets(struct mxc_isi_dev *mxc_isi)
{
	int ret;
	struct device *dev = &mxc_isi->pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *parent, *child;
	struct of_phandle_args args;
	struct reset_control *rstc;
	const char *compat;
	uint32_t len, rstc_num = 0;

	ret = of_parse_phandle_with_args(np, "resets", "#reset-cells",
					 0, &args);
	if (ret)
		return ret;

	parent = args.np;
	for_each_child_of_node(parent, child) {
		compat = of_get_property(child, "compatible", NULL);
		if (!compat)
			continue;

		rstc = of_reset_control_array_get(child, false, false, true);
		if (IS_ERR(rstc))
			continue;

		len = strlen(compat);
		if (!of_compat_cmp("isi,soft-resetn", compat, len)) {
			mxc_isi->soft_resetn = rstc;
			rstc_num++;
		} else if (!of_compat_cmp("isi,clk-enable", compat, len)) {
			mxc_isi->clk_enable = rstc;
			rstc_num++;
		} else {
			dev_warn(dev, "invalid isi reset node: %s\n", compat);
		}
	}

	if (!rstc_num) {
		dev_err(dev, "no invalid reset control exists\n");
		return -EINVAL;
	}

	of_node_put(parent);
	return 0;
}

static int mxc_isi_soc_match(struct mxc_isi_dev *mxc_isi,
			     const struct soc_device_attribute *data)
{
	struct mxc_isi_ier_reg *ier_reg = mxc_isi->pdata->ier_reg;
	const struct soc_device_attribute *match;

	match = soc_device_match(data);
	if (!match)
		return -EINVAL;

	mxc_isi->buf_active_reverse = false;

	if (!strcmp(match->soc_id, "i.MX8QXP") ||
	    !strcmp(match->soc_id, "i.MX8QM")) {
		/* Chip C0 */
		if (strcmp(match->revision, "1.1") > 0) {
			memcpy(ier_reg, &mxc_imx8_isi_ier_v1, sizeof(*ier_reg));
			mxc_isi->buf_active_reverse = true;
		}
	}

	return 0;
}

static int mxc_isi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mxc_isi_dev *mxc_isi;
	struct resource *res;
	const struct of_device_id *of_id;
	int ret = 0;


	mxc_isi = devm_kzalloc(dev, sizeof(*mxc_isi), GFP_KERNEL);
	if (!mxc_isi)
		return -ENOMEM;

	mxc_isi->pdev = pdev;
	of_id = of_match_node(mxc_isi_of_match, dev->of_node);
	if (!of_id)
		return -EINVAL;

	mxc_isi->pdata = of_id->data;
	if (!mxc_isi->pdata) {
		dev_err(dev, "Can't get platform device data\n");
		return -EINVAL;
	}

	ret = mxc_isi_soc_match(mxc_isi, imx8_soc);
	if (ret < 0) {
		dev_err(dev, "Can't match soc version\n");
		return ret;
	}

	ret = mxc_isi_parse_dt(mxc_isi);
	if (ret < 0)
		return ret;

	if (mxc_isi->id >= MXC_ISI_MAX_DEVS || mxc_isi->id < 0) {
		dev_err(dev, "Invalid driver data or device id (%d)\n",
			mxc_isi->id);
		return -EINVAL;
	}

	spin_lock_init(&mxc_isi->slock);
	mutex_init(&mxc_isi->lock);
	atomic_set(&mxc_isi->usage_count, 0);

	if (of_device_is_compatible(dev->of_node, "nxp,imx8mn-isi")) {
		ret = mxc_isi_of_parse_resets(mxc_isi);
		if (ret) {
			dev_warn(dev, "Can not parse reset control\n");
			return ret;
		}
	}

	ret = mxc_isi_clk_get(mxc_isi);
	if (ret < 0) {
		dev_err(dev, "ISI_%d get clocks fail\n", mxc_isi->id);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mxc_isi->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(mxc_isi->regs)) {
		dev_err(dev, "Failed to get ISI register map\n");
		return PTR_ERR(mxc_isi->regs);
	}

	ret = mxc_isi_clk_enable(mxc_isi);
	if (ret < 0) {
		dev_err(dev, "ISI_%d enable clocks fail\n", mxc_isi->id);
		return ret;
	}
	disp_mix_sft_rstn(mxc_isi->soft_resetn, false);
	disp_mix_clks_enable(mxc_isi->clk_enable, true);

	mxc_isi_clean_registers(mxc_isi);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "Failed to get IRQ resource\n");
		goto err;
	}
	ret = devm_request_irq(dev, res->start, mxc_isi_irq_handler,
			       0, dev_name(dev), mxc_isi);
	if (ret < 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err;
	}

	mxc_isi_channel_set_chain_buf(mxc_isi);

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret < 0)
		dev_warn(dev, "Populate child platform device fail\n");

	mxc_isi_clk_disable(mxc_isi);

	platform_set_drvdata(pdev, mxc_isi);
	pm_runtime_enable(dev);

	dev_info(dev, "mxc_isi.%d registered successfully\n", mxc_isi->id);
	return 0;

err:
	disp_mix_clks_enable(mxc_isi->clk_enable, false);
	disp_mix_sft_rstn(mxc_isi->soft_resetn, true);
	mxc_isi_clk_disable(mxc_isi);
	return -ENXIO;
}

static int mxc_isi_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_disable(dev);

	return 0;
}

static int mxc_isi_pm_suspend(struct device *dev)
{
	struct mxc_isi_dev *mxc_isi = dev_get_drvdata(dev);

	if (mxc_isi->is_streaming) {
		dev_warn(dev, "running, prevent entering suspend.\n");
		return -EAGAIN;
	}

	return pm_runtime_force_suspend(dev);
}

static int mxc_isi_pm_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static int mxc_isi_runtime_suspend(struct device *dev)
{
	struct mxc_isi_dev *mxc_isi = dev_get_drvdata(dev);

	disp_mix_clks_enable(mxc_isi->clk_enable, false);
	mxc_isi_clk_disable(mxc_isi);

	return 0;
}

static int mxc_isi_runtime_resume(struct device *dev)
{
	struct mxc_isi_dev *mxc_isi = dev_get_drvdata(dev);
	int ret;

	ret = mxc_isi_clk_enable(mxc_isi);
	if (ret) {
		dev_err(dev, "%s clk enable fail\n", __func__);
		return ret;
	}
	disp_mix_sft_rstn(mxc_isi->soft_resetn, false);
	disp_mix_clks_enable(mxc_isi->clk_enable, true);

	return 0;
}

static const struct dev_pm_ops mxc_isi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mxc_isi_pm_suspend, mxc_isi_pm_resume)
	SET_RUNTIME_PM_OPS(mxc_isi_runtime_suspend, mxc_isi_runtime_resume, NULL)
};

static const struct of_device_id mxc_isi_of_match[] = {
	{.compatible = "fsl,imx8-isi", .data = &mxc_imx8_data },
	{.compatible = "nxp,imx8mn-isi", .data = &mxc_imx8mn_data },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mxc_isi_of_match);

static struct platform_driver mxc_isi_driver = {
	.probe		= mxc_isi_probe,
	.remove		= mxc_isi_remove,
	.driver = {
		.of_match_table = mxc_isi_of_match,
		.name		= MXC_ISI_DRIVER_NAME,
		.pm		= &mxc_isi_pm_ops,
	}
};
module_platform_driver(mxc_isi_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX8 Image Subsystem driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ISI");
MODULE_VERSION("1.0");
