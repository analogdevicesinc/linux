// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 NXP Semiconductor
 *
 */

#include "imx8-isi-hw.h"

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
	u32 status;

	spin_lock(&mxc_isi->slock);

	status = mxc_isi_get_irq_status(mxc_isi);
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

	if (status & (CHNL_STS_OFLW_PANIC_Y_BUF_MASK |
		      CHNL_STS_OFLW_PANIC_U_BUF_MASK |
		      CHNL_STS_OFLW_PANIC_V_BUF_MASK))
		dev_dbg(dev, "%s, IRQ Panic OFLW Error stat=0x%X\n", __func__, status);

	if (status & (CHNL_STS_OFLW_Y_BUF_MASK |
		      CHNL_STS_OFLW_U_BUF_MASK |
		      CHNL_STS_OFLW_V_BUF_MASK))
		dev_dbg(dev, "%s, IRQ OFLW Error stat=0x%X\n", __func__, status);

	if (status & (CHNL_STS_EXCS_OFLW_Y_BUF_MASK |
		      CHNL_STS_EXCS_OFLW_U_BUF_MASK |
		      CHNL_STS_EXCS_OFLW_V_BUF_MASK))
		dev_dbg(dev, "%s, IRQ EXCS OFLW Error stat=0x%X\n", __func__, status);

	spin_unlock(&mxc_isi->slock);
	return IRQ_HANDLED;
}

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

static int mxc_isi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mxc_isi_dev *mxc_isi;
	struct resource *res;
	int ret = 0;

	mxc_isi = devm_kzalloc(dev, sizeof(*mxc_isi), GFP_KERNEL);
	if (!mxc_isi)
		return -ENOMEM;

	mxc_isi->pdev = pdev;

	ret = mxc_isi_parse_dt(mxc_isi);
	if (ret < 0)
		return ret;

	if (mxc_isi->id >= MXC_ISI_MAX_DEVS || mxc_isi->id < 0) {
		dev_err(dev, "Invalid driver data or device id (%d)\n", mxc_isi->id);
		return -EINVAL;
	}

	spin_lock_init(&mxc_isi->slock);
	mutex_init(&mxc_isi->lock);
	atomic_set(&mxc_isi->usage_count, 0);

	mxc_isi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(mxc_isi->clk)) {
		dev_err(dev, "failed to get isi clk\n");
		return PTR_ERR(mxc_isi->clk);
	}

	ret = clk_prepare_enable(mxc_isi->clk);
	if (ret < 0) {
		dev_err(dev, "Prepare and enable isi clk error (%d)\n", ret);
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mxc_isi->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(mxc_isi->regs)) {
		dev_err(dev, "Failed to get ISI register map\n");
		return PTR_ERR(mxc_isi->regs);
	}

	mxc_isi_clean_registers(mxc_isi);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "Failed to get IRQ resource\n");
		return -ENXIO;
	}
	ret = devm_request_irq(dev, res->start, mxc_isi_irq_handler,
			       0, dev_name(dev), mxc_isi);
	if (ret < 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		return -EINVAL;
	}

	mxc_isi_channel_set_chain_buf(mxc_isi);

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret < 0)
		dev_warn(dev, "Populate child platform device fail\n");

	clk_disable_unprepare(mxc_isi->clk);

	platform_set_drvdata(pdev, mxc_isi);
	pm_runtime_enable(dev);

	dev_info(dev, "mxc_isi.%d registered successfully\n", mxc_isi->id);
	return 0;
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

	clk_disable_unprepare(mxc_isi->clk);
	return 0;
}

static int mxc_isi_runtime_resume(struct device *dev)
{
	struct mxc_isi_dev *mxc_isi = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(mxc_isi->clk);
	if (ret)
		dev_err(dev, "%s clk enable fail\n", __func__);

	return (ret) ? ret : 0;
}

static const struct dev_pm_ops mxc_isi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mxc_isi_pm_suspend, mxc_isi_pm_resume)
	SET_RUNTIME_PM_OPS(mxc_isi_runtime_suspend, mxc_isi_runtime_resume, NULL)
};

static const struct of_device_id mxc_isi_of_match[] = {
	{.compatible = "fsl,imx8-isi",},
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
