/*
 * Copyright 2017-2018 NXP
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include "mxc-media-dev.h"

static irqreturn_t mxc_isi_irq_handler(int irq, void *priv)
{
	struct mxc_isi_dev *mxc_isi = priv;
	struct device *dev = &mxc_isi->pdev->dev;
	u32 status;

	spin_lock(&mxc_isi->slock);

	status = mxc_isi_get_irq_status(mxc_isi);
	mxc_isi_clean_irq_status(mxc_isi, status);

	if (status & CHNL_STS_MEM_RD_DONE_MASK)
		mxc_isi_m2m_frame_read_done(mxc_isi);

	if (status & CHNL_STS_FRM_STRD_MASK) {
		if (mxc_isi->is_m2m)
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

/**
 * mxc_isi_adjust_mplane_format - adjust bytesperline or sizeimage
 */
void mxc_isi_adjust_mplane_format(struct mxc_isi_fmt *fmt, u32 width, u32 height,
			       struct v4l2_pix_format_mplane *pix)
{
	u32 bytesperline = 0;
	int i;

	pix->colorspace	= V4L2_COLORSPACE_JPEG;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = fmt->memplanes;
	pix->pixelformat = fmt->fourcc;
	pix->height = height;
	pix->width = width;

	for (i = 0; i < pix->num_planes; ++i) {
		struct v4l2_plane_pix_format *plane_fmt = &pix->plane_fmt[i];
		u32 bpl = plane_fmt->bytesperline;

		if (fmt->colplanes > 1 && (bpl == 0 || bpl < pix->width))
			bpl = pix->width; /* Planar */

		if (fmt->colplanes == 1 && /* Packed */
		    (bpl == 0 || ((bpl * 8) / fmt->depth[i]) < pix->width))
			bpl = (pix->width * fmt->depth[0]) / 8;

		if (i == 0)
			bytesperline = bpl;
		else if (i == 1 && fmt->memplanes == 3)
			bytesperline /= 2;

		plane_fmt->bytesperline = bytesperline;
		plane_fmt->sizeimage = max((pix->width * pix->height *
				   fmt->depth[i]) / 8, plane_fmt->sizeimage);
	}
}

static int mxc_isi_parse_dt(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;
	struct device_node *node = dev->of_node;
	int ret = 0;

	mxc_isi->id = of_alias_get_id(node, "isi");

	ret = of_property_read_u32_array(node, "interface",
			mxc_isi->interface, 3);
	if (ret < 0)
		return ret;

	mxc_isi->parallel_csi = of_property_read_bool(node, "parallel_csi");

	dev_dbg(dev, "%s, isi_%d,interface(%d, %d, %d)\n", __func__, mxc_isi->id,
			mxc_isi->interface[0], mxc_isi->interface[1], mxc_isi->interface[2]);

	mxc_isi->chain_buf = of_property_read_bool(node, "fsl,chain_buf");
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
		dev_err(dev, "Invalid driver data or device id (%d)\n",
			mxc_isi->id);
		return -EINVAL;
	}

	init_waitqueue_head(&mxc_isi->irq_queue);
	spin_lock_init(&mxc_isi->slock);
	mutex_init(&mxc_isi->lock);
	atomic_set(&mxc_isi->open_count, 0);

	mxc_isi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(mxc_isi->clk)) {
		dev_err(dev, "failed to get isi clk\n");
		return PTR_ERR(mxc_isi->clk);
	}
	ret = clk_prepare(mxc_isi->clk);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk error\n", __func__);
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mxc_isi->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(mxc_isi->regs)) {
		dev_err(dev, "Failed to get ISI register map\n");
		return PTR_ERR(mxc_isi->regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(dev, "Failed to get IRQ resource\n");
		return -ENXIO;
	}

	ret = devm_request_irq(dev, res->start, mxc_isi_irq_handler,
			       0, dev_name(dev), mxc_isi);
	if (ret < 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		return -EINVAL;
	}

	ret = mxc_isi_initialize_capture_subdev(mxc_isi);
	if (ret < 0) {
		dev_err(dev, "failed to init cap subdev (%d)\n", ret);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, mxc_isi);

	ret = clk_enable(mxc_isi->clk);
	if (ret < 0) {
		dev_err(dev, "%s, enable clk error\n", __func__);
		goto err_sclk;
	}

	mxc_isi_channel_set_chain_buf(mxc_isi);
	clk_disable_unprepare(mxc_isi->clk);

	pm_runtime_enable(dev);

	dev_dbg(dev, "mxc_isi.%d registered successfully\n", mxc_isi->id);

	return 0;

err_sclk:
	mxc_isi_unregister_capture_subdev(mxc_isi);
	return ret;
}

static int mxc_isi_remove(struct platform_device *pdev)
{
	struct mxc_isi_dev *mxc_isi = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	mxc_isi_unregister_capture_subdev(mxc_isi);
	pm_runtime_disable(dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxc_isi_pm_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

static int mxc_isi_pm_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}
#endif

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
MODULE_DESCRIPTION("MXC Image Subsystem driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ISI");
MODULE_VERSION("1.0");
