/*
 * Copyright 2018 NXP
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#define DEBUG
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <soc/imx8/sc/sci.h>
#include <dt-bindings/pinctrl/pads-imx8qxp.h>
#include <linux/init.h>

#include "mxc-parallel-csi.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

static int format;
module_param(format, int, 0644);
MODULE_PARM_DESC(format, "Format level (0-2)");

#ifdef DEBUG
static void mxc_pcsi_regs_dump(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = &pcsidev->pdev->dev;

	dev_dbg(dev, "HW_IF_CTRL_REG = 0x%08x\n",
		readl(pcsidev->csr_regs + IF_CTRL_REG));
	dev_dbg(dev, "HW_CSI_CTRL_REG = 0x%08x\n",
		readl(pcsidev->csr_regs + CSI_CTRL_REG));
	dev_dbg(dev, "HW_CSI_STATUS = 0x%08x\n",
		readl(pcsidev->csr_regs + CSI_STATUS));
	dev_dbg(dev, "HW_CSI_CTRL_REG1 = 0x%08x\n",
		readl(pcsidev->csr_regs + CSI_CTRL_REG1));
}
#else
static void mxc_pcsi_regs_dump(struct mxc_parallel_csi_dev *pcsidev) { }
#endif

static struct mxc_parallel_csi_dev *sd_to_mxc_pcsi_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct mxc_parallel_csi_dev, sd);
}

static int mxc_pcsi_clk_get(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = &pcsidev->pdev->dev;

	pcsidev->clk_pixel = devm_clk_get(dev, "pixel");
	if (IS_ERR(pcsidev->clk_pixel)) {
		dev_info(dev, "%s failed to get parallel csi pixel clk\n", __func__);
		return PTR_ERR(pcsidev->clk_pixel);
	}

	pcsidev->clk_ipg = devm_clk_get(dev, "ipg");
	if (IS_ERR(pcsidev->clk_ipg)) {
		dev_info(dev, "%s failed to get parallel ipg pixel clk\n", __func__);
		return PTR_ERR(pcsidev->clk_ipg);
	}

	return 0;
}

static int mxc_pcsi_clk_enable(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = &pcsidev->pdev->dev;
	int ret;

	if (pcsidev->clk_enable)
		return 0;

	ret = clk_prepare_enable(pcsidev->clk_pixel);
	if (ret < 0) {
		dev_info(dev, "%s, enable pixel clk error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(pcsidev->clk_ipg);
	if (ret < 0) {
		dev_info(dev, "%s, enable ipg clk error\n", __func__);
		return ret;
	}

	pcsidev->clk_enable = true;

	return 0;
}

static void mxc_pcsi_clk_disable(struct mxc_parallel_csi_dev *pcsidev)
{
	if (!pcsidev->clk_enable)
		return;

	clk_disable_unprepare(pcsidev->clk_pixel);
	clk_disable_unprepare(pcsidev->clk_ipg);

	pcsidev->clk_enable = false;
}

static void mxc_pcsi_sw_reset(struct mxc_parallel_csi_dev *pcsidev)
{
	u32 val;

	/* Softwaret Reset */
	val = CSI_CTRL_REG_SOFTRST;
	writel(val, pcsidev->csr_regs + CSI_CTRL_REG_SET);

	msleep(1);
	writel(val, pcsidev->csr_regs + CSI_CTRL_REG_CLR);
}

static void mxc_pcsi_csr_config(struct mxc_parallel_csi_dev *pcsidev)
{
	u32 val;

	/* Software Reset */
	mxc_pcsi_sw_reset(pcsidev);

	/* Config PL Data Type */
	val = IF_CTRL_REG_DATA_TYPE(DATA_TYPE_OUT_YUV444);
	writel(val, pcsidev->csr_regs + IF_CTRL_REG_SET);

	/* Enable sync Force */
	val = (CSI_CTRL_REG_HSYNC_FORCE_EN | CSI_CTRL_REG_VSYNC_FORCE_EN);
	writel(val, pcsidev->csr_regs + CSI_CTRL_REG_SET);

	/* Enable Pixel Link */
	val = IF_CTRL_REG_PL_ENABLE;
	writel(val , pcsidev->csr_regs + IF_CTRL_REG_SET);

	/* Enable Pixel Link */
	val = IF_CTRL_REG_PL_VALID;
	writel(val , pcsidev->csr_regs + IF_CTRL_REG_SET);

	/* Config CTRL REG */
	val = readl(pcsidev->csr_regs + CSI_CTRL_REG);
	val |= (
		CSI_CTRL_REG_DATA_TYPE_IN(DATA_TYPE_IN_YVYU_8BITS) |
		CSI_CTRL_REG_HSYNC_POL |
		CSI_CTRL_REG_MASK_VSYNC_COUNTER(3) |
		CSI_CTRL_REG_HSYNC_PULSE(2));

	if (pcsidev->uv_swap)
		val |= CSI_CTRL_REG_UV_SWAP_EN;

	if (pcsidev->mode & GATE_CLOCK_MODE)
		val |= CSI_CTRL_REG_GCLK_MODE_EN;
	else if (pcsidev->mode & CCIR_MODE) {
		val |= (CSI_CTRL_REG_CCIR_EN |
			CSI_CTRL_REG_CCIR_VSYNC_RESET_EN |
			CSI_CTRL_REG_CCIR_EXT_VSYNC_EN |
			CSI_CTRL_REG_CCIR_ECC_ERR_CORRECT_EN);
	}

	writel(val, pcsidev->csr_regs + CSI_CTRL_REG);
}

static void mxc_pcsi_config_ctrl_reg1(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = &pcsidev->pdev->dev;
	u32 val;

	if (pcsidev->format.width <= 0 || pcsidev->format.height <= 0) {
		dev_dbg(dev, "%s width/height invalid\n", __func__);
		return;
	}

	/* Config Pixel Width */
	val = (CSI_CTRL_REG1_PIXEL_WIDTH(pcsidev->format.width - 1) |
		CSI_CTRL_REG1_VSYNC_PULSE(pcsidev->format.width << 1));
	writel(val, pcsidev->csr_regs + CSI_CTRL_REG1);

}

static void mxc_pcsi_enable_csi(struct mxc_parallel_csi_dev *pcsidev)
{
	u32 val;

	/* Enable CSI */
	val = CSI_CTRL_REG_CSI_EN;
	writel(val, pcsidev->csr_regs + CSI_CTRL_REG_SET);

	/* Disable SYNC Force */
	val = (CSI_CTRL_REG_HSYNC_FORCE_EN | CSI_CTRL_REG_VSYNC_FORCE_EN);
	writel(val, pcsidev->csr_regs + CSI_CTRL_REG_CLR);
}

static void mxc_pcsi_disable_csi(struct mxc_parallel_csi_dev *pcsidev)
{
	u32 val;

	/* Enable Sync Force */
	val = (CSI_CTRL_REG_HSYNC_FORCE_EN | CSI_CTRL_REG_VSYNC_FORCE_EN);
	writel(val, pcsidev->csr_regs + CSI_CTRL_REG_SET);

	/* Disable CSI */
	val = CSI_CTRL_REG_CSI_EN;
	writel(val, pcsidev->csr_regs + CSI_CTRL_REG_CLR);

	/* Disable Pixel Link */
	val = IF_CTRL_REG_PL_VALID | IF_CTRL_REG_PL_ENABLE;
	writel(val , pcsidev->csr_regs + IF_CTRL_REG_CLR);
}

static struct media_pad *mxc_pcsi_get_remote_sensor_pad(
			struct mxc_parallel_csi_dev *pcsidev)
{
	struct v4l2_subdev *subdev = &pcsidev->sd;
	struct media_pad *sink_pad, *source_pad;
	int i;

	while (1) {
		source_pad = NULL;
		for (i = 0; i < subdev->entity.num_pads; i++) {
			sink_pad = &subdev->entity.pads[i];

			if (sink_pad->flags & MEDIA_PAD_FL_SINK) {
				source_pad = media_entity_remote_pad(sink_pad);
				if (source_pad)
					break;
			}
		}
		/* return first pad point in the loop  */
		return source_pad;
	}

	if (i == subdev->entity.num_pads)
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote pad found!\n", __func__);

	return NULL;
}

static int mxc_pcsi_get_sensor_fmt(struct mxc_parallel_csi_dev *pcsidev)
{
	struct v4l2_mbus_framefmt *mf = &pcsidev->format;
	struct v4l2_subdev *sen_sd;
	struct media_pad *source_pad;
	struct v4l2_subdev_format src_fmt;
	int ret;

	/* Get remote source pad */
	source_pad = mxc_pcsi_get_remote_sensor_pad(pcsidev);
	if (source_pad == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sen_sd == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	src_fmt.pad = source_pad->index;
	ret = v4l2_subdev_call(sen_sd, pad, get_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return -EINVAL;

	/* Update input frame size and formate  */
	memcpy(mf, &src_fmt.format, sizeof(struct v4l2_mbus_framefmt));

	if (mf->code == MEDIA_BUS_FMT_YUYV8_2X8)
		pcsidev->uv_swap = 1;

	dev_dbg(&pcsidev->pdev->dev, "width=%d, height=%d, fmt.code=0x%x\n", mf->width, mf->height, mf->code);

	return 0;
}

static int mxc_pcsi_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = mxc_pcsi_get_remote_sensor_pad(pcsidev);
	if (source_pad == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sen_sd == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	return v4l2_subdev_call(sen_sd, pad, enum_frame_size, NULL, fse);
}

static int mxc_pcsi_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = mxc_pcsi_get_remote_sensor_pad(pcsidev);
	if (source_pad == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sen_sd == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	return v4l2_subdev_call(sen_sd, pad, enum_frame_interval, NULL, fie);
}

static int mxc_pcsi_get_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mxc_pcsi_get_sensor_fmt(pcsidev);

	memcpy(mf, &pcsidev->format, sizeof(struct v4l2_mbus_framefmt));
	/* Source/Sink pads crop rectangle size */

	return 0;
}

static int mxc_pcsi_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int mxc_pcsi_s_power(struct v4l2_subdev *sd, int on)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = mxc_pcsi_get_remote_sensor_pad(pcsidev);
	if (source_pad == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sen_sd == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	return v4l2_subdev_call(sen_sd, core, s_power, on);
}

static int mxc_pcsi_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = mxc_pcsi_get_remote_sensor_pad(pcsidev);
	if (source_pad == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sen_sd == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	return v4l2_subdev_call(sen_sd, video, s_parm, a);
}

static int mxc_pcsi_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = mxc_pcsi_get_remote_sensor_pad(pcsidev);
	if (source_pad == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sen_sd == NULL) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}
	return v4l2_subdev_call(sen_sd, video, g_parm, a);
}

static int mxc_pcsi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct device *dev = &pcsidev->pdev->dev;

	dev_dbg(dev, "%s: %d, pcsidev: 0x%d\n", __func__, __LINE__, enable);

	if (enable) {
		pm_runtime_get_sync(dev);
		if (!pcsidev->running) {
			mxc_pcsi_get_sensor_fmt(pcsidev);
			mxc_pcsi_csr_config(pcsidev);
			mxc_pcsi_config_ctrl_reg1(pcsidev);
			mxc_pcsi_enable_csi(pcsidev);
			mxc_pcsi_regs_dump(pcsidev);
		}
		pcsidev->running++;
	} else {
		if (pcsidev->running)
			mxc_pcsi_disable_csi(pcsidev);
		pcsidev->running--;
		pm_runtime_put(dev);
	}

	return 0;
}

static int mxc_pcsi_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct platform_device *pdev = v4l2_get_subdevdata(sd);

	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		switch (local->index) {
		case MXC_PARALLEL_CSI_PAD_SOURCE:
			break;
		default:
			dev_err(&pdev->dev, "%s invalid source pad\n", __func__);
			return -EINVAL;
		}
	} else if (local->flags & MEDIA_PAD_FL_SINK) {
		switch (local->index) {
		case MXC_PARALLEL_CSI_PAD_SINK:
			break;
		default:
			dev_err(&pdev->dev, "%s invalid sink pad\n", __func__);
			return -EINVAL;
		}
	}
	return 0;
}

static struct v4l2_subdev_pad_ops pcsi_pad_ops = {
	.enum_frame_size = mxc_pcsi_enum_framesizes,
	.enum_frame_interval = mxc_pcsi_enum_frame_interval,
	.get_fmt = mxc_pcsi_get_fmt,
	.set_fmt = mxc_pcsi_set_fmt,
};

static struct v4l2_subdev_core_ops pcsi_core_ops = {
	.s_power = mxc_pcsi_s_power,
};

static struct v4l2_subdev_video_ops pcsi_video_ops = {
	.s_parm = mxc_pcsi_s_parm,
	.g_parm = mxc_pcsi_g_parm,
	.s_stream = mxc_pcsi_s_stream,
};

static struct v4l2_subdev_ops pcsi_subdev_ops = {
	.core = &pcsi_core_ops,
	.video = &pcsi_video_ops,
	.pad = &pcsi_pad_ops,
};
static const struct media_entity_operations mxc_pcsi_sd_media_ops = {
	.link_setup = mxc_pcsi_link_setup,
};

static int mxc_parallel_csi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem_res;
	struct mxc_parallel_csi_dev *pcsidev;
	int ret;

	pcsidev = devm_kzalloc(dev, sizeof(*pcsidev), GFP_KERNEL);
	if (!pcsidev)
		return -ENOMEM;

	pcsidev->pdev = pdev;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pcsidev->csr_regs = devm_ioremap_resource(dev, mem_res);
	if (IS_ERR(pcsidev->csr_regs)) {
		dev_dbg(dev, "Failed to get parallel CSI CSR register\n");
		return PTR_ERR(pcsidev->csr_regs);
	}

	ret = mxc_pcsi_clk_get(pcsidev);
	if (ret < 0)
		return ret;

	v4l2_subdev_init(&pcsidev->sd, &pcsi_subdev_ops);

	pcsidev->mode = GATE_CLOCK_MODE;

	pcsidev->sd.owner = THIS_MODULE;
	sprintf(pcsidev->sd.name, "%s", MXC_PARALLEL_CSI_SUBDEV_NAME);

	pcsidev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	pcsidev->sd.entity.function = MEDIA_ENT_F_IO_V4L;

	pcsidev->sd.dev = dev;

	pcsidev->pads[MXC_PARALLEL_CSI_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pcsidev->pads[MXC_PARALLEL_CSI_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&pcsidev->sd.entity,
				MXC_PARALLEL_CSI_PADS_NUM, pcsidev->pads);
	if (ret < 0)
		goto e_clkdis;

	pcsidev->sd.entity.ops = &mxc_pcsi_sd_media_ops;

	v4l2_set_subdevdata(&pcsidev->sd, pdev);

	platform_set_drvdata(pdev, pcsidev);

	pcsidev->running = 0;
	pm_runtime_enable(dev);

	dev_info(dev, "%s probe successfully\n", __func__);
	return 0;

e_clkdis:
	media_entity_cleanup(&pcsidev->sd.entity);
	return ret;
}

static int mxc_parallel_csi_remove(struct platform_device *pdev)
{
	struct mxc_parallel_csi_dev *pcsidev =
			(struct mxc_parallel_csi_dev *)platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);
	media_entity_cleanup(&pcsidev->sd.entity);
	mxc_pcsi_clk_disable(pcsidev);
	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static void parallel_csi_power_control(sc_pm_power_mode_t mode)
{
	sc_ipc_t ipcHndl;
	sc_err_t sciErr;
	uint32_t mu_id;

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		pr_err("Cannot obtain MU ID\n");
		return;
	}

	sciErr = sc_ipc_open(&ipcHndl, mu_id);
	if (sciErr != SC_ERR_NONE) {
		pr_err("sc_ipc_open failed! (sciError = %d)\n", sciErr);
		return;
	}

	sc_pm_set_resource_power_mode(ipcHndl, SC_R_PI_0, mode);

	if (sciErr != SC_ERR_NONE)
		pr_err("Set CI_PI resouce power mode failed! (sciError = %d)\n", sciErr);

	msleep(10);

	sc_ipc_close(mu_id);
}

static int parallel_csi_pm_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

static int parallel_csi_pm_resume(struct device *dev)
{
	struct mxc_parallel_csi_dev *pcsidev = dev_get_drvdata(dev);
	int ret;

	/* Power off CI_PI before set clock parent */
	parallel_csi_power_control(SC_PM_PW_MODE_OFF);

	pcsidev->clk_div = devm_clk_get(dev, "div");
	if (IS_ERR(pcsidev->clk_div)) {
		dev_err(dev, "%s: Get div clk fail\n", __func__);
		return PTR_ERR(pcsidev->clk_div);
	}

	pcsidev->clk_sel = devm_clk_get(dev, "sel");
	if (IS_ERR(pcsidev->clk_sel)) {
		dev_err(dev, "%s: Get sel clk fail\n", __func__);
		return PTR_ERR(pcsidev->clk_sel);
	}

	pcsidev->clk_dpll = devm_clk_get(dev, "dpll");
	if (IS_ERR(pcsidev->clk_dpll)) {
		dev_err(dev, "%s: Get DPLL clk fail\n", __func__);
		return PTR_ERR(pcsidev->clk_dpll);
	}

	ret = clk_set_parent(pcsidev->clk_sel, pcsidev->clk_dpll);
	if (ret < 0) {
		dev_err(dev, "sel clk set parent fail\n");
		return ret;
	}

	/* 160MHz for pixel and per clock */
	ret = clk_set_rate(pcsidev->clk_div, 160000000);
	if (ret < 0) {
		dev_err(dev, "div clk set rate fail\n");
		return ret;
	}

	/* Release parent clocks */
	devm_clk_put(dev, pcsidev->clk_dpll);
	devm_clk_put(dev, pcsidev->clk_sel);
	devm_clk_put(dev, pcsidev->clk_div);

	pm_runtime_enable(dev);

	return 0;
}

static int parallel_csi_runtime_suspend(struct device *dev)
{
	struct mxc_parallel_csi_dev *pcsidev = dev_get_drvdata(dev);

	mxc_pcsi_clk_disable(pcsidev);

	return 0;
}

static int parallel_csi_runtime_resume(struct device *dev)
{
	struct mxc_parallel_csi_dev *pcsidev = dev_get_drvdata(dev);
	int ret;

	ret = mxc_pcsi_clk_enable(pcsidev);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct dev_pm_ops parallel_csi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(parallel_csi_pm_suspend, parallel_csi_pm_resume)
	SET_RUNTIME_PM_OPS(parallel_csi_runtime_suspend,
						parallel_csi_runtime_resume, NULL)
};

static const struct of_device_id parallel_csi_of_match[] = {
	{	.compatible = "fsl,mxc-parallel-csi",},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, parallel_csi_of_match);


static struct platform_driver parallel_csi_driver = {
	.driver = {
		.name = MXC_PARALLEL_CSI_DRIVER_NAME,
		.of_match_table = parallel_csi_of_match,
		.pm = &parallel_csi_pm_ops,
	},
	.probe = mxc_parallel_csi_probe,
	.remove = mxc_parallel_csi_remove,
};

module_platform_driver(parallel_csi_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC PARALLEL CSI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" MXC_PARALLEL_CSI_DRIVER_NAME);
