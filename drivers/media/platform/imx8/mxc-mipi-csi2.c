/*
 * Copyright 2017 NXP
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
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

#include "mxc-mipi-csi2.h"
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

/* 0~ 80Mbps: 0xB
 * 80~250Mbps: 0x8
 * 250~1.5Gbps: 0x6*/
static u8 rxhs_settle[3] = {0xB, 0x8, 0x6};

static struct mxc_mipi_csi2_dev *sd_to_mxc_mipi_csi2_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct mxc_mipi_csi2_dev, sd);
}

/****************************************
 * rxhs-settle calculate
 * UI = 1000 / mipi csi phy clock
 * THS-SETTLE_mim = 85ns + 6 * UI
 * THS-SETTLE_max = 145ns +10 * UI
 * PRG_RXHS_SETTLE =  THS-SETTLE / (Tperiod of RxClk_ESC) + 1
 ****************************************/
static int calc_hs_settle(struct mxc_mipi_csi2_dev *csi2dev, u32 dphy_clk)
{
	u32 esc_rate;
	u32 hs_settle;
	u32 rxhs_settle;

	esc_rate = clk_get_rate(csi2dev->clk_esc) / 1000000;
	hs_settle = 115 + 8 * 1000 / dphy_clk;
	rxhs_settle = hs_settle / (1000 / esc_rate) - 1;
	return rxhs_settle;
}

#ifdef debug
static void mxc_mipi_csi2_reg_dump(struct mxc_mipi_csi2_dev *csi2dev)
{
	printk("MIPI CSI2 HC register dump, mipi csi%d\n", csi2dev->id);
	printk("MIPI CSI2 HC num of lanes     0x100 = 0x%x\n", readl(csi2dev->base_regs + 0x100));
	printk("MIPI CSI2 HC dis lanes        0x104 = 0x%x\n", readl(csi2dev->base_regs + 0x104));
	printk("MIPI CSI2 HC BIT ERR          0x108 = 0x%x\n", readl(csi2dev->base_regs + 0x108));
	printk("MIPI CSI2 HC IRQ STATUS       0x10C = 0x%x\n", readl(csi2dev->base_regs + 0x10C));
	printk("MIPI CSI2 HC IRQ MASK         0x110 = 0x%x\n", readl(csi2dev->base_regs + 0x110));
	printk("MIPI CSI2 HC ULPS STATUS      0x114 = 0x%x\n", readl(csi2dev->base_regs + 0x114));
	printk("MIPI CSI2 HC DPHY ErrSotHS    0x118 = 0x%x\n", readl(csi2dev->base_regs + 0x118));
	printk("MIPI CSI2 HC DPHY ErrSotSync  0x11c = 0x%x\n", readl(csi2dev->base_regs + 0x11c));
	printk("MIPI CSI2 HC DPHY ErrEsc      0x120 = 0x%x\n", readl(csi2dev->base_regs + 0x120));
	printk("MIPI CSI2 HC DPHY ErrSyncEsc  0x124 = 0x%x\n", readl(csi2dev->base_regs + 0x124));
	printk("MIPI CSI2 HC DPHY ErrControl  0x128 = 0x%x\n", readl(csi2dev->base_regs + 0x128));
	printk("MIPI CSI2 HC DISABLE_PAYLOAD  0x12C = 0x%x\n", readl(csi2dev->base_regs + 0x12C));
	printk("MIPI CSI2 HC DISABLE_PAYLOAD  0x130 = 0x%x\n", readl(csi2dev->base_regs + 0x130));
	printk("MIPI CSI2 HC IGNORE_VC        0x180 = 0x%x\n", readl(csi2dev->base_regs + 0x180));
	printk("MIPI CSI2 HC VID_VC           0x184 = 0x%x\n", readl(csi2dev->base_regs + 0x184));
	printk("MIPI CSI2 HC FIFO_SEND_LEVEL  0x188 = 0x%x\n", readl(csi2dev->base_regs + 0x188));
	printk("MIPI CSI2 HC VID_VSYNC        0x18C = 0x%x\n", readl(csi2dev->base_regs + 0x18C));
	printk("MIPI CSI2 HC VID_SYNC_FP      0x190 = 0x%x\n", readl(csi2dev->base_regs + 0x190));
	printk("MIPI CSI2 HC VID_HSYNC        0x194 = 0x%x\n", readl(csi2dev->base_regs + 0x194));
	printk("MIPI CSI2 HC VID_HSYNC_BP     0x198 = 0x%x\n", readl(csi2dev->base_regs + 0x198));
	printk("MIPI CSI2 CSR register dump\n");
	printk("MIPI CSI2 CSR PLM_CTRL        0x000 = 0x%x\n", readl(csi2dev->csr_regs + 0x000));
	printk("MIPI CSI2 CSR PHY_CTRL        0x004 = 0x%x\n", readl(csi2dev->csr_regs + 0x004));
	printk("MIPI CSI2 CSR PHY_Status      0x008 = 0x%x\n", readl(csi2dev->csr_regs + 0x008));
	printk("MIPI CSI2 CSR PHY_Test_Status 0x010 = 0x%x\n", readl(csi2dev->csr_regs + 0x010));
	printk("MIPI CSI2 CSR PHY_Test_Status 0x014 = 0x%x\n", readl(csi2dev->csr_regs + 0x014));
	printk("MIPI CSI2 CSR PHY_Test_Status 0x018 = 0x%x\n", readl(csi2dev->csr_regs + 0x018));
	printk("MIPI CSI2 CSR PHY_Test_Status 0x01C = 0x%x\n", readl(csi2dev->csr_regs + 0x01C));
	printk("MIPI CSI2 CSR PHY_Test_Status 0x020 = 0x%x\n", readl(csi2dev->csr_regs + 0x020));
	printk("MIPI CSI2 CSR VC Interlaced   0x030 = 0x%x\n", readl(csi2dev->csr_regs + 0x030));
	printk("MIPI CSI2 CSR Data Type Dis   0x038 = 0x%x\n", readl(csi2dev->csr_regs + 0x038));
	printk("MIPI CSI2 CSR 420 1st type    0x040 = 0x%x\n", readl(csi2dev->csr_regs + 0x040));
	printk("MIPI CSI2 CSR Ctr_Ck_Rst_Ctr  0x044 = 0x%x\n", readl(csi2dev->csr_regs + 0x044));
	printk("MIPI CSI2 CSR Stream Fencing  0x048 = 0x%x\n", readl(csi2dev->csr_regs + 0x048));
	printk("MIPI CSI2 CSR Stream Fencing  0x04C = 0x%x\n", readl(csi2dev->csr_regs + 0x04C));
}
#else
static void mxc_mipi_csi2_reg_dump(struct mxc_mipi_csi2_dev *csi2dev)
{
}
#endif

static void mipi_sc_fw_init(struct mxc_mipi_csi2_dev *csi2dev, char enable)
{
	sc_ipc_t ipcHndl;
	sc_err_t sciErr;
	sc_rsrc_t	rsrc_id;
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

	if (csi2dev->id == 1)
		rsrc_id = SC_R_CSI_1;
	else
		rsrc_id = SC_R_CSI_0;

	sciErr = sc_misc_set_control(ipcHndl, rsrc_id, SC_C_MIPI_RESET, enable);
	if (sciErr != SC_ERR_NONE)
		pr_err("sc_misc_MIPI reset failed! (sciError = %d)\n", sciErr);

	msleep(10);

	sc_ipc_close(mu_id);
}

static void mxc_mipi_csi2_reset(struct mxc_mipi_csi2_dev *csi2dev)
{
	u32 val;

	/* Reset MIPI CSI */
	val = CSI2SS_CTRL_CLK_RESET_EN | CSI2SS_CTRL_CLK_RESET_CLK_OFF;
	writel(val, csi2dev->csr_regs + CSI2SS_CTRL_CLK_RESET);
}

static void mxc_mipi_csi2_enable(struct mxc_mipi_csi2_dev *csi2dev)
{
	u32 val = 0;

	val = readl(csi2dev->csr_regs + CSI2SS_PLM_CTRL);
	while (val & CSI2SS_PLM_CTRL_PL_CLK_RUN) {
		msleep(10);
		val = readl(csi2dev->csr_regs + CSI2SS_PLM_CTRL);
		printk("Waiting pl clk running, val=0x%x\n", val);
	}

	/* Enable Pixel link Master*/
	val = readl(csi2dev->csr_regs + CSI2SS_PLM_CTRL);
	val |= CSI2SS_PLM_CTRL_ENABLE_PL;
	writel(val, csi2dev->csr_regs + CSI2SS_PLM_CTRL);

	val |= CSI2SS_PLM_CTRL_VALID_OVERRIDE;
	writel(val, csi2dev->csr_regs + CSI2SS_PLM_CTRL);

	/* PHY Enable */
	val = readl(csi2dev->csr_regs + CSI2SS_PHY_CTRL);
	val &= ~(CSI2SS_PHY_CTRL_PD_MASK | CSI2SS_PLM_CTRL_POLARITY_MASK);
	writel(val, csi2dev->csr_regs + CSI2SS_PHY_CTRL);

	/* Deassert reset */
	writel(1, csi2dev->csr_regs + CSI2SS_CTRL_CLK_RESET);
}

static void mxc_mipi_csi2_disable(struct mxc_mipi_csi2_dev *csi2dev)
{
	/* Disable Data lanes */
	writel(0xf, csi2dev->base_regs + CSI2RX_CFG_DISABLE_DATA_LANES);

	/* Disable Pixel Link */
	writel(0, csi2dev->csr_regs + CSI2SS_PLM_CTRL);

	/* Disable  PHY */
	writel(0, csi2dev->csr_regs + CSI2SS_PHY_CTRL);

	/* Reset */
	writel(2, csi2dev->csr_regs + CSI2SS_CTRL_CLK_RESET);
}

static void mxc_mipi_csi2_csr_config(struct mxc_mipi_csi2_dev *csi2dev)
{
	u32 val;

	/* format */
	val = 0;
	writel(val, csi2dev->csr_regs + CSI2SS_DATA_TYPE);

	/* polarity */
	val = readl(csi2dev->csr_regs + CSI2SS_PLM_CTRL);
	val &= ~(CSI2SS_PLM_CTRL_VSYNC_OVERRIDE |
			CSI2SS_PLM_CTRL_HSYNC_OVERRIDE |
			CSI2SS_PLM_CTRL_VALID_OVERRIDE |
			CSI2SS_PLM_CTRL_POLARITY_MASK);

	writel(val, csi2dev->csr_regs + CSI2SS_PLM_CTRL);

	val = CSI2SS_PHY_CTRL_RX_ENABLE |
		CSI2SS_PHY_CTRL_DDRCLK_EN << CSI2SS_PHY_CTRL_DDRCLK_EN_OFFSET |
		CSI2SS_PHY_CTRL_CONT_CLK_MODE << CSI2SS_PHY_CTRL_CONT_CLK_MODE_OFFSET |
		csi2dev->hs_settle << CSI2SS_PHY_CTRL_RX_HS_SETTLE_OFFSET |
		CSI2SS_PHY_CTRL_PD << CSI2SS_PHY_CTRL_PD_OFFSET |
		CSI2SS_PHY_CTRL_RTERM_SEL << CSI2SS_PHY_CTRL_RTERM_SEL_OFFSET;

	writel(val, csi2dev->csr_regs + CSI2SS_PHY_CTRL);
}

static void mxc_mipi_csi2_hc_config(struct mxc_mipi_csi2_dev *csi2dev)
{
	u32 val0, val1;
	u32 i;

	val0 = 0;

	/* Lanes */
	writel(csi2dev->num_lanes - 1, csi2dev->base_regs + CSI2RX_CFG_NUM_LANES);

	for (i = 0; i < csi2dev->num_lanes; i++)
		val0 |= (1 << (csi2dev->data_lanes[i] - 1));

	val1 = 0xF & ~val0;
	writel(val1, csi2dev->base_regs + CSI2RX_CFG_DISABLE_DATA_LANES);

	/* Mask interrupt */
	writel(0x1FF, csi2dev->base_regs + CSI2RX_IRQ_MASK);

	/* vid_vc */
	writel(3, csi2dev->base_regs + 0x184);
}

static struct media_pad *mxc_csi2_get_remote_sensor_pad(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct v4l2_subdev *subdev = &csi2dev->sd;
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
		v4l2_err(&csi2dev->v4l2_dev, "%s, No remote pad found!\n", __func__);

	return NULL;
}

static int mxc_csi2_get_sensor_fmt(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct v4l2_mbus_framefmt *mf = &csi2dev->format;
	struct v4l2_subdev *sen_sd;
	struct media_pad *source_pad;
	struct v4l2_subdev_format src_fmt;
	int ret;

	/* Get remote source pad */
	source_pad = mxc_csi2_get_remote_sensor_pad(csi2dev);
	if (source_pad == NULL) {
		v4l2_err(&csi2dev->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (sen_sd == NULL) {
		v4l2_err(&csi2dev->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	src_fmt.pad = source_pad->index;
	ret = v4l2_subdev_call(sen_sd, pad, get_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return -EINVAL;

	/* Update input frame size and formate  */
	memcpy(mf, &src_fmt.format, sizeof(struct v4l2_mbus_framefmt));

	dev_dbg(&csi2dev->pdev->dev, "width=%d, height=%d, fmt.code=0x%x\n", mf->width, mf->height, mf->code);

	/* Get rxhs settle */
	if (src_fmt.format.reserved[0] != 0)
		csi2dev->hs_settle = calc_hs_settle(csi2dev, src_fmt.format.reserved[0]);
	else {
		if (src_fmt.format.height * src_fmt.format.width > 1024 * 768)
			csi2dev->hs_settle = rxhs_settle[2];
		else if (src_fmt.format.height * src_fmt.format.width < 480 * 320)
			csi2dev->hs_settle = rxhs_settle[0];
		else
			csi2dev->hs_settle = rxhs_settle[1];
	}

	return 0;
}

static int mipi_csi2_clk_init(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;

	csi2dev->clk_apb = devm_clk_get(dev, "clk_apb");
	if (IS_ERR(csi2dev->clk_apb)) {
		dev_err(dev, "failed to get csi apb clk\n");
		return PTR_ERR(csi2dev->clk_apb);
	}

	csi2dev->clk_core = devm_clk_get(dev, "clk_core");
	if (IS_ERR(csi2dev->clk_core)) {
		dev_err(dev, "failed to get csi core clk\n");
		return PTR_ERR(csi2dev->clk_core);
	}

	csi2dev->clk_esc = devm_clk_get(dev, "clk_esc");
	if (IS_ERR(csi2dev->clk_esc)) {
		dev_err(dev, "failed to get csi esc clk\n");
		return PTR_ERR(csi2dev->clk_esc);
	}

	csi2dev->clk_pxl = devm_clk_get(dev, "clk_pxl");
	if (IS_ERR(csi2dev->clk_pxl)) {
		dev_err(dev, "failed to get csi pixel link clk\n");
		return PTR_ERR(csi2dev->clk_pxl);
	}

	return 0;
}

static int mipi_csi2_clk_enable(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	int ret;

	ret = clk_prepare_enable(csi2dev->clk_apb);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk_apb error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(csi2dev->clk_core);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk_core error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(csi2dev->clk_esc);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk_esc error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(csi2dev->clk_pxl);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk_pxl error\n", __func__);
		return ret;
	}

	return ret;
}

static void mipi_csi2_clk_disable(struct mxc_mipi_csi2_dev *csi2dev)
{
	clk_disable_unprepare(csi2dev->clk_apb);
	clk_disable_unprepare(csi2dev->clk_core);
	clk_disable_unprepare(csi2dev->clk_esc);
	clk_disable_unprepare(csi2dev->clk_pxl);
}

static int mipi_csi2_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

/* mipi csi2 subdev media entity operations */
static int mipi_csi2_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	/* TODO */
	/* Add MIPI source and sink pad link configuration */
	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		switch (local->index) {
		case MXC_MIPI_CSI2_VC0_PAD_SOURCE:
		case MXC_MIPI_CSI2_VC1_PAD_SOURCE:
		case MXC_MIPI_CSI2_VC2_PAD_SOURCE:
		case MXC_MIPI_CSI2_VC3_PAD_SOURCE:
			break;
		default:
			return 0;
		}
	} else if (local->flags & MEDIA_PAD_FL_SINK) {
		switch (local->index) {
		case MXC_MIPI_CSI2_VC0_PAD_SINK:
		case MXC_MIPI_CSI2_VC1_PAD_SINK:
		case MXC_MIPI_CSI2_VC2_PAD_SINK:
		case MXC_MIPI_CSI2_VC3_PAD_SINK:
			break;
		default:
			return 0;
		}
	}
	return 0;
}

static const struct media_entity_operations mipi_csi2_sd_media_ops = {
	.link_setup = mipi_csi2_link_setup,
};

/*
 * V4L2 subdev operations
 */
static int mipi_csi2_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int mipi_csi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct device *dev = &csi2dev->pdev->dev;
	int ret = 0;

	dev_dbg(&csi2dev->pdev->dev, "%s: %d, csi2dev: 0x%x\n",
		 __func__, enable, csi2dev->flags);

	if (enable) {
		pm_runtime_get_sync(dev);
		if (!csi2dev->running) {
			mxc_csi2_get_sensor_fmt(csi2dev);
			mxc_mipi_csi2_hc_config(csi2dev);
			mxc_mipi_csi2_reset(csi2dev);
			mxc_mipi_csi2_csr_config(csi2dev);
			mxc_mipi_csi2_enable(csi2dev);
			mxc_mipi_csi2_reg_dump(csi2dev);
		}
		csi2dev->running++;
	} else {

		if (csi2dev->running)
			mxc_mipi_csi2_disable(csi2dev);
		csi2dev->running--;
		pm_runtime_put(dev);
	}

	return ret;
}

static int mipi_csi2_get_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mxc_csi2_get_sensor_fmt(csi2dev);

	memcpy(mf, &csi2dev->format, sizeof(struct v4l2_mbus_framefmt));
	/* Source/Sink pads crop rectangle size */

	return 0;
}

static int mipi_csi2_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	return 0;
}

static const struct v4l2_subdev_internal_ops mipi_csi2_sd_internal_ops = {
	.open = mipi_csi2_open,
};

static struct v4l2_subdev_pad_ops mipi_csi2_pad_ops = {
	.get_fmt = mipi_csi2_get_fmt,
	.set_fmt = mipi_csi2_set_fmt,
};

static struct v4l2_subdev_core_ops mipi_csi2_core_ops = {
	.s_power = mipi_csi2_s_power,
};

static struct v4l2_subdev_video_ops mipi_csi2_video_ops = {
	.s_stream = mipi_csi2_s_stream,
};

static struct v4l2_subdev_ops mipi_csi2_subdev_ops = {
	.core = &mipi_csi2_core_ops,
	.video = &mipi_csi2_video_ops,
	.pad = &mipi_csi2_pad_ops,
};
static int mipi_csi2_parse_dt(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_fwnode_endpoint endpoint;
	u32 i;

	csi2dev->id = of_alias_get_id(node, "csi");

	csi2dev->vchannel = of_property_read_bool(node, "virtual-channel");

	node = of_graph_get_next_endpoint(node, NULL);
	if (!node) {
		dev_err(dev, "No port node at %s\n", node->full_name);
		return -EINVAL;
	}

	/* Get port node */
	v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &endpoint);

	csi2dev->num_lanes = endpoint.bus.mipi_csi2.num_data_lanes;
	for (i = 0; i < 4; i++)
		csi2dev->data_lanes[i] = endpoint.bus.mipi_csi2.data_lanes[i];

	of_node_put(node);

	return 0;
}

static int mipi_csi2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem_res;
	struct mxc_mipi_csi2_dev *csi2dev;
	int ret = -ENOMEM;

	dev_info(&pdev->dev, "%s\n", __func__);
	csi2dev = devm_kzalloc(dev, sizeof(*csi2dev), GFP_KERNEL);
	if (!csi2dev)
		return -ENOMEM;

	csi2dev->pdev = pdev;
	mutex_init(&csi2dev->lock);

	ret = mipi_csi2_parse_dt(csi2dev);
	if (ret < 0)
		return -EINVAL ;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi2dev->base_regs = devm_ioremap_resource(dev, mem_res);
	if (IS_ERR(csi2dev->base_regs)) {
		dev_err(dev, "Failed to get mipi csi2 HC register\n");
		return PTR_ERR(csi2dev->base_regs);
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	csi2dev->csr_regs = devm_ioremap_resource(dev, mem_res);
	if (IS_ERR(csi2dev->csr_regs)) {
		dev_err(dev, "Failed to get mipi CSR register\n");
		return PTR_ERR(csi2dev->csr_regs);
	}

	ret = mipi_csi2_clk_init(csi2dev);
	if (ret < 0)
		return -EINVAL ;

	v4l2_subdev_init(&csi2dev->sd, &mipi_csi2_subdev_ops);

	csi2dev->sd.owner = THIS_MODULE;
	snprintf(csi2dev->sd.name, sizeof(csi2dev->sd.name), "%s.%d",
		 MXC_MIPI_CSI2_SUBDEV_NAME, csi2dev->id);

	csi2dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	csi2dev->sd.entity.function = MEDIA_ENT_F_IO_V4L;

	csi2dev->pads[MXC_MIPI_CSI2_VC0_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2dev->pads[MXC_MIPI_CSI2_VC1_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2dev->pads[MXC_MIPI_CSI2_VC2_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2dev->pads[MXC_MIPI_CSI2_VC3_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2dev->pads[MXC_MIPI_CSI2_VC0_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2dev->pads[MXC_MIPI_CSI2_VC1_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2dev->pads[MXC_MIPI_CSI2_VC2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2dev->pads[MXC_MIPI_CSI2_VC3_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&csi2dev->sd.entity,
				MXC_MIPI_CSI2_VCX_PADS_NUM, csi2dev->pads);
	if (ret < 0)
		goto e_clkdis;

	csi2dev->sd.entity.ops = &mipi_csi2_sd_media_ops;

	/* This allows to retrieve the platform device id by the host driver */
	v4l2_set_subdevdata(&csi2dev->sd, pdev);

	/* .. and a pointer to the subdev. */
	platform_set_drvdata(pdev, csi2dev);

	mipi_sc_fw_init(csi2dev, 1);

	csi2dev->running = 0;
	csi2dev->flags = MXC_MIPI_CSI2_PM_POWERED;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	ret = mipi_csi2_clk_enable(csi2dev);
	if (ret < 0)
		goto e_clkdis;

	dev_info(&pdev->dev, "lanes: %d, name: %s\n",
		 csi2dev->num_lanes, csi2dev->sd.name);
	pm_runtime_put_sync(dev);

	return 0;

e_clkdis:
	media_entity_cleanup(&csi2dev->sd.entity);
	return ret;
}

static int mipi_csi2_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);

	pm_runtime_get_sync(&pdev->dev);
	mipi_sc_fw_init(csi2dev, 0);
	media_entity_cleanup(&csi2dev->sd.entity);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int mipi_csi2_suspend(struct device *dev, bool runtime)
{
	struct mxc_mipi_csi2_dev *csi2dev = dev_get_drvdata(dev);
	struct v4l2_subdev *sd = &csi2dev->sd;

	mutex_lock(&csi2dev->lock);
	if (csi2dev->flags & MXC_MIPI_CSI2_PM_POWERED) {
		if (csi2dev->running)
			mipi_csi2_s_stream(sd, false);

		mipi_csi2_clk_disable(csi2dev);
		csi2dev->flags &= ~MXC_MIPI_CSI2_PM_POWERED;

		if (runtime)
			csi2dev->flags |= MXC_MIPI_CSI2_RUNTIME_SUSPENDED;
		else
			csi2dev->flags |= MXC_MIPI_CSI2_PM_SUSPENDED;
	}
	mutex_unlock(&csi2dev->lock);
	return 0;
}

static int mipi_csi2_resume(struct device *dev, bool runtime)
{
	struct mxc_mipi_csi2_dev *csi2dev = dev_get_drvdata(dev);
	struct v4l2_subdev *sd = &csi2dev->sd;
	int ret;

	mutex_lock(&csi2dev->lock);
	if (!(csi2dev->flags & MXC_MIPI_CSI2_RUNTIME_SUSPENDED) &&
		!(csi2dev->flags & MXC_MIPI_CSI2_PM_SUSPENDED)) {
		mutex_unlock(&csi2dev->lock);
		return 0;
	}

	if (!(csi2dev->flags & MXC_MIPI_CSI2_PM_POWERED)) {
		ret = mipi_csi2_clk_enable(csi2dev);
		if (ret < 0) {
			mutex_unlock(&csi2dev->lock);
			dev_err(dev, "%s:%d fail\n", __func__, __LINE__);
			return -EAGAIN;
		}

		if (csi2dev->running)
			mipi_csi2_s_stream(sd, true);

		csi2dev->flags |= MXC_MIPI_CSI2_PM_POWERED;
		if (runtime)
			csi2dev->flags &= ~MXC_MIPI_CSI2_RUNTIME_SUSPENDED;
		else
			csi2dev->flags &= ~MXC_MIPI_CSI2_PM_SUSPENDED;
	}
	mutex_unlock(&csi2dev->lock);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int  mipi_csi2_pm_suspend(struct device *dev)
{
	return mipi_csi2_suspend(dev, false);
}

static int  mipi_csi2_pm_resume(struct device *dev)
{
	return mipi_csi2_resume(dev, false);
}
#endif

static int  mipi_csi2_runtime_suspend(struct device *dev)
{
	return mipi_csi2_suspend(dev, true);
}
static int  mipi_csi2_runtime_resume(struct device *dev)
{
	return mipi_csi2_resume(dev, true);
}

static const struct dev_pm_ops mipi_csi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mipi_csi2_pm_suspend, mipi_csi2_pm_resume)
	SET_RUNTIME_PM_OPS(mipi_csi2_runtime_suspend, mipi_csi2_runtime_resume, NULL)
};

static const struct of_device_id mipi_csi2_of_match[] = {
	{	.compatible = "fsl,mxc-mipi-csi2",},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mipi_csi2_of_match);


static struct platform_driver mipi_csi2_driver = {
	.driver = {
		.name = MXC_MIPI_CSI2_DRIVER_NAME,
		.of_match_table = mipi_csi2_of_match,
		.pm = &mipi_csi_pm_ops,
	},
	.probe = mipi_csi2_probe,
	.remove = mipi_csi2_remove,
};

module_platform_driver(mipi_csi2_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC MIPI CSI2 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" MXC_MIPI_CSI2_DRIVER_NAME);
