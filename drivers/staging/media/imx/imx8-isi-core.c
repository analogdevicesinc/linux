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
	}, {
		.soc_id   = "i.MX8MP",
	}, {
		.soc_id   = "i.MX8ULP",
	}, {
		.soc_id   = "i.MX93",
	}, {
		.soc_id   = "i.MX91P",
	}, {
		.soc_id   = "i.MX91",
	}, {
		/* sentinel */
	},
};

static const struct of_device_id mxc_isi_of_match[];

static irqreturn_t mxc_isi_irq_handler(int irq, void *priv)
{
	struct mxc_isi_dev *mxc_isi = priv;
	struct device *dev = &mxc_isi->pdev->dev;
	struct mxc_isi_ier_reg *ier_reg = mxc_isi->pdata->ier_reg;
	unsigned long flags;
	u32 status;

	spin_lock_irqsave(&mxc_isi->slock, flags);

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

	spin_unlock_irqrestore(&mxc_isi->slock, flags);
	return IRQ_HANDLED;
}

static int disp_mix_sft_parse_resets(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_plat_data const *pdata = mxc_isi->pdata;

	if (mxc_isi->no_dispmix)
		return 0;

	if (!pdata->rst_ops || !pdata->rst_ops->parse)
		return -EINVAL;

	return pdata->rst_ops->parse(mxc_isi);
}

static int disp_mix_clks_get(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_plat_data  const *pdata = mxc_isi->pdata;

	if (mxc_isi->no_dispmix)
		return 0;

	if (!pdata->gclk_ops || !pdata->gclk_ops->gclk_get)
		return -EINVAL;

	return pdata->gclk_ops->gclk_get(mxc_isi);
}

/* For i.MX8QXP C0 and i.MX8MN ISI IER version */
static struct mxc_isi_ier_reg mxc_imx8_isi_ier_v1 = {
	.oflw_y_buf_en = { .offset = 19, .mask = 0x80000  },
	.oflw_u_buf_en = { .offset = 21, .mask = 0x200000 },
	.oflw_v_buf_en = { .offset = 23, .mask = 0x800000 },

	.panic_y_buf_en = {.offset = 20, .mask = 0x100000  },
	.panic_u_buf_en = {.offset = 22, .mask = 0x400000  },
	.panic_v_buf_en = {.offset = 24, .mask = 0x1000000 },
};

/* For i.MX8MP ISI IER version */
static struct mxc_isi_ier_reg mxc_imx8_isi_ier_v2 = {
	.oflw_y_buf_en = { .offset = 18, .mask = 0x40000  },
	.oflw_u_buf_en = { .offset = 20, .mask = 0x100000 },
	.oflw_v_buf_en = { .offset = 22, .mask = 0x400000 },

	.panic_y_buf_en = {.offset = 19, .mask = 0x80000  },
	.panic_u_buf_en = {.offset = 21, .mask = 0x200000 },
	.panic_v_buf_en = {.offset = 23, .mask = 0x800000 },
};

/* Panic will assert when the buffers are 50% full */
static struct mxc_isi_set_thd mxc_imx8_isi_thd_v1 = {
	.panic_set_thd_y = { .mask = 0x0000F, .offset = 0,  .threshold = 0x7 },
	.panic_set_thd_u = { .mask = 0x00F00, .offset = 8,  .threshold = 0x7 },
	.panic_set_thd_v = { .mask = 0xF0000, .offset = 16, .threshold = 0x7 },
};

static struct mxc_isi_chan_src mxc_imx8mp_chan_src = {
	.src_mipi0 = 0,
	.src_mipi1 = 1,
	.src_mem = 2,
};

static int mxc_imx8mp_clk_get(struct mxc_isi_dev *mxc_isi)
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

static int mxc_imx8mp_clk_enable(struct mxc_isi_dev *mxc_isi)
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

static void mxc_imx8mp_clk_disable(struct mxc_isi_dev *mxc_isi)
{
	clk_disable_unprepare(mxc_isi->clk_root_disp_axi);
	clk_disable_unprepare(mxc_isi->clk_root_disp_apb);
	clk_disable_unprepare(mxc_isi->clk_disp_axi);
	clk_disable_unprepare(mxc_isi->clk_disp_apb);
}

static struct mxc_isi_dev_ops mxc_imx8mp_clk_ops = {
	.clk_get     = mxc_imx8mp_clk_get,
	.clk_enable  = mxc_imx8mp_clk_enable,
	.clk_disable = mxc_imx8mp_clk_disable,
};

static int mxc_isi_imx8mp_parse_resets(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;
	struct reset_control *reset;

	reset = devm_reset_control_get_optional_shared(dev, "isi_rst_proc");
	if (IS_ERR(reset)) {
		if (PTR_ERR(reset) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get isi proc reset control\n");
		return PTR_ERR(reset);
	}
	mxc_isi->isi_rst_proc = reset;

	reset = devm_reset_control_get_optional_shared(dev, "isi_rst_apb");
	if (IS_ERR(reset)) {
		if (PTR_ERR(reset) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get isi apb reset control\n");
		return PTR_ERR(reset);
	}
	mxc_isi->isi_rst_apb = reset;

	return 0;
}

static int mxc_isi_imx8mp_resets_assert(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;
	int ret;

	if (!mxc_isi->isi_rst_proc || !mxc_isi->isi_rst_apb)
		return -EINVAL;

	ret = reset_control_assert(mxc_isi->isi_rst_proc);
	if (ret) {
		dev_err(dev, "Failed to assert isi proc reset control\n");
		return ret;
	}

	ret = reset_control_assert(mxc_isi->isi_rst_apb);
	if (ret) {
		dev_err(dev, "Failed to assert isi apb reset control\n");
		return ret;
	}

	ret = reset_control_assert(mxc_isi->isi_rst_bus);
	if (ret) {
		dev_err(dev, "Failed to assert isi bus reset control\n");
		return ret;
	}

	return ret;
}

static int mxc_isi_imx8mp_resets_deassert(struct mxc_isi_dev *mxc_isi)
{
	if (!mxc_isi->isi_rst_proc || !mxc_isi->isi_rst_apb)
		return -EINVAL;

	reset_control_deassert(mxc_isi->isi_rst_proc);
	reset_control_deassert(mxc_isi->isi_rst_apb);
	reset_control_deassert(mxc_isi->isi_rst_bus);

	return 0;
}

static struct mxc_isi_rst_ops mxc_imx8mp_isi_rst_ops = {
	.parse  = mxc_isi_imx8mp_parse_resets,
	.assert = mxc_isi_imx8mp_resets_assert,
	.deassert = mxc_isi_imx8mp_resets_deassert,
};

static int mxc_isi_imx8mp_gclk_get(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;

	mxc_isi->isi_proc = devm_clk_get_optional(dev, "media_blk_isi_proc");
	if (IS_ERR(mxc_isi->isi_proc)) {
		if (PTR_ERR(mxc_isi->isi_proc) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get media isi proc clock\n");
		return -ENODEV;
	}

	mxc_isi->isi_apb = devm_clk_get_optional(dev, "media_blk_isi_apb");
	if (IS_ERR(mxc_isi->isi_apb)) {
		if (PTR_ERR(mxc_isi->isi_apb) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get media isi apb clock\n");
		return -ENODEV;
	}

	mxc_isi->isi_bus = devm_clk_get_optional(dev, "media_blk_bus");
	if (IS_ERR(mxc_isi->isi_bus)) {
		if (PTR_ERR(mxc_isi->isi_bus) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get media bus clock\n");
		return -ENODEV;
	}

	return 0;
}

static int mxc_isi_imx8mp_gclk_enable(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;
	int ret;

	ret = clk_prepare_enable(mxc_isi->isi_proc);
	if (ret) {
		dev_err(dev, "enable isi proc clock failed!\n");
		return ret;
	}

	ret = clk_prepare_enable(mxc_isi->isi_apb);
	if (ret) {
		dev_err(dev, "enable isi apb clock failed!\n");
		return ret;
	}

	ret = clk_prepare_enable(mxc_isi->isi_bus);
	if (ret) {
		dev_err(dev, "enable bus clock failed!\n");
		return ret;
	}

	return ret;
}

static int mxc_isi_imx8mp_gclk_disable(struct mxc_isi_dev *mxc_isi)
{
	clk_disable_unprepare(mxc_isi->isi_proc);
	clk_disable_unprepare(mxc_isi->isi_apb);
	clk_disable_unprepare(mxc_isi->isi_bus);

	return 0;
}

static struct mxc_isi_gate_clk_ops mxc_imx8mp_isi_gclk_ops = {
	.gclk_get = mxc_isi_imx8mp_gclk_get,
	.gclk_enable  = mxc_isi_imx8mp_gclk_enable,
	.gclk_disable = mxc_isi_imx8mp_gclk_disable,
};

static struct mxc_isi_plat_data mxc_imx8mp_data = {
	.ops      = &mxc_imx8mp_clk_ops,
	.chan_src = &mxc_imx8mp_chan_src,
	.ier_reg  = &mxc_imx8_isi_ier_v1,
	.set_thd  = &mxc_imx8_isi_thd_v1,
	.rst_ops  = &mxc_imx8mp_isi_rst_ops,
	.gclk_ops = &mxc_imx8mp_isi_gclk_ops,
};

static int mxc_isi_parse_dt(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;
	struct device_node *node = dev->of_node;
	int ret = 0;

	mxc_isi->id = of_alias_get_id(node, "isi");

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

static int mxc_isi_soc_match(struct mxc_isi_dev *mxc_isi,
			     const struct soc_device_attribute *data)
{
	struct mxc_isi_ier_reg *ier_reg = mxc_isi->pdata->ier_reg;
	struct mxc_isi_set_thd *set_thd = mxc_isi->pdata->set_thd;
	const struct soc_device_attribute *match;

	match = soc_device_match(data);
	if (!match)
		return -EPROBE_DEFER;

	mxc_isi->buf_active_reverse = false;

	if (!strcmp(match->soc_id, "i.MX8QXP") ||
	    !strcmp(match->soc_id, "i.MX8QM")) {
		/* Chip C0 */
		if (strcmp(match->revision, "1.1") > 0) {
			memcpy(ier_reg, &mxc_imx8_isi_ier_v1, sizeof(*ier_reg));
			memcpy(set_thd, &mxc_imx8_isi_thd_v1, sizeof(*set_thd));
			mxc_isi->buf_active_reverse = true;
		}
	} else if (!strcmp(match->soc_id, "i.MX8MP") ||
		   !strcmp(match->soc_id, "i.MX8ULP") ||
		   !strcmp(match->soc_id, "i.MX93") ||
		   !strcmp(match->soc_id, "i.MX91")) {
		memcpy(ier_reg, &mxc_imx8_isi_ier_v2, sizeof(*ier_reg));
		mxc_isi->buf_active_reverse = true;
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
		if (ret != -EPROBE_DEFER)
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

	mxc_isi->chain = syscon_regmap_lookup_by_phandle(dev->of_node, "isi_chain");
	if (IS_ERR(mxc_isi->chain))
		mxc_isi->chain = NULL;

	spin_lock_init(&mxc_isi->slock);
	mutex_init(&mxc_isi->lock);
	atomic_set(&mxc_isi->usage_count, 0);

	mxc_isi->no_dispmix =
		of_property_read_bool(dev->of_node, "no-reset-control");

	ret = disp_mix_sft_parse_resets(mxc_isi);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Can not parse reset control for isi\n");
		return ret;
	}

	ret = mxc_isi_clk_get(mxc_isi);
	if (ret < 0) {
		dev_err(dev, "ISI_%d get clocks fail\n", mxc_isi->id);
		return ret;
	}

	ret = disp_mix_clks_get(mxc_isi);
	if (ret < 0) {
		dev_err(dev, "Failed to get disp mix clocks");
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
	disp_mix_sft_rstn(mxc_isi, false);
	disp_mix_clks_enable(mxc_isi, true);

	mxc_isi_clean_registers(mxc_isi);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to get IRQ resource\n");
		goto err;
	}
	mxc_isi->irq = ret;

	ret = devm_request_irq(dev, mxc_isi->irq, mxc_isi_irq_handler,
			       0, dev_name(dev), mxc_isi);
	if (ret < 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err;
	}

	mxc_isi_channel_set_chain_buf(mxc_isi);

	ret = devm_of_platform_populate(dev);
	if (ret < 0)
		dev_warn(dev, "Populate child platform device fail\n");

	mxc_isi_clk_disable(mxc_isi);
	disp_mix_clks_enable(mxc_isi, false);

	platform_set_drvdata(pdev, mxc_isi);
	pm_runtime_enable(dev);

	dev_info(dev, "mxc_isi.%d registered successfully\n", mxc_isi->id);
	return 0;

err:
	disp_mix_clks_enable(mxc_isi, false);
	disp_mix_sft_rstn(mxc_isi, true);
	mxc_isi_clk_disable(mxc_isi);
	return -ENXIO;
}

static void mxc_isi_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	of_platform_depopulate(dev);
	pm_runtime_disable(dev);
}

static int mxc_isi_pm_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

static int mxc_isi_pm_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static int mxc_isi_runtime_suspend(struct device *dev)
{
	return 0;
}

static int mxc_isi_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops mxc_isi_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(mxc_isi_pm_suspend, mxc_isi_pm_resume)
	SET_RUNTIME_PM_OPS(mxc_isi_runtime_suspend, mxc_isi_runtime_resume, NULL)
};

static const struct of_device_id mxc_isi_of_match[] = {
	{.compatible = "nxp,imx8mp-isi", .data = &mxc_imx8mp_data },
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
