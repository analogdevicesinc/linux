/*
 * Driver for the Analog Devices AXI-JESD204-TX peripheral
 *
 * Copyright 2017 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/fpga/adi-axi-common.h>

#include <linux/jesd204/jesd204.h>

#include "axi_jesd204.h"

#define JESD204_TX_REG_MAGIC			0x0c

#define JESD204_TX_REG_CONF_NUM_LANES		0x10
#define JESD204_TX_REG_CONF_DATA_PATH_WIDTH	0x14

#define JESD204_TX_REG_IRQ_ENABLE		0x80
#define JESD204_TX_REG_IRQ_PENDING		0x84
#define JESD204_TX_REG_IRQ_SOURCE		0x88

#define JESD204_TX_REG_LINK_DISABLE		0xc0
#define JESD204_TX_REG_LINK_STATE		0xc4
#define JESD204_TX_REG_LINK_CLK_RATIO		0xc8

#define JESD204_TX_REG_SYSREF_CONF		0x100
#define JESD204_TX_REG_SYSREF_LMFC_OFFSET	0x104
#define JESD204_TX_REG_SYSREF_STATUS		0x108

#define JESD204_TX_REG_LANES_ENABLE		0x200
#define JESD204_TX_REG_CONF0			0x210
#define JESD204_TX_REG_CONF1			0x214
#define JESD204_TX_REG_CONF2			0x240
#define JESD204_TX_REG_CONF3			0x244

#define JESD204_TX_REG_MANUAL_SYNC_REQUEST	0x248

#define JESD204_TX_REG_LINK_STATUS		0x280

#define JESD204_TX_REG_ILAS(x, y) (((x) * 32 + (y) * 4) + 0x310)

#define JESD204_TX_MAGIC (('2' << 24) | ('0' << 16) | ('4' << 8) | ('T'))

/* JESD204_TX_REG_SYSREF_CONF */
#define JESD204_TX_REG_SYSREF_CONF_SYSREF_DISABLE	BIT(0)

/* JESD204_TX_REG_LINK_STATUS */
#define JESD204_LINK_STATUS_DATA			3

struct axi_jesd204_tx {
	void __iomem *base;
	struct device *dev;

	struct clk *axi_clk;
	struct clk *device_clk;
	struct clk *conv2_clk;

	struct jesd204_dev *jdev;

	int irq;

	unsigned int num_lanes;
	unsigned int data_path_width;
	enum jesd204_encoder encoder;

	/* Used for probe ordering */
	struct clk_hw dummy_clk;
	struct clk *lane_clk;
};

static const char * const axi_jesd204_tx_link_status_label[] = {
	"WAIT",
	"CGS",
	"ILAS",
	"DATA"
};

static ssize_t axi_jesd204_tx_status_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct axi_jesd204_tx *jesd = dev_get_drvdata(dev);
	unsigned int sysref_status;
	unsigned int link_disabled;
	unsigned int link_status;
	unsigned int link_config0;
	unsigned int clock_ratio;
	unsigned int clock_rate;
	unsigned int link_rate;
	unsigned int sysref_config;
	unsigned int lmfc_rate;
	int ret;

	link_disabled = readl_relaxed(jesd->base + JESD204_TX_REG_LINK_STATE);
	link_status = readl_relaxed(jesd->base + JESD204_TX_REG_LINK_STATUS);
	sysref_status = readl_relaxed(jesd->base + JESD204_TX_REG_SYSREF_STATUS);
	clock_ratio = readl_relaxed(jesd->base + JESD204_TX_REG_LINK_CLK_RATIO);
	sysref_config = readl_relaxed(jesd->base + JESD204_TX_REG_SYSREF_CONF);
	link_config0 = readl_relaxed(jesd->base + JESD204_TX_REG_CONF0);

	ret = scnprintf(buf, PAGE_SIZE, "Link is %s\n",
		(link_disabled & 0x1) ? "disabled" : "enabled");

	if (clock_ratio == 0) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Measured Link Clock: off\n");
	} else {
		clock_rate = DIV_ROUND_CLOSEST_ULL(100000ULL * clock_ratio,
			1ULL << 16);

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Measured Link Clock: %d.%.3d MHz\n",
			clock_rate / 1000, clock_rate % 1000);
	}

	clock_rate = DIV_ROUND_CLOSEST(clk_get_rate(jesd->device_clk), 1000);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Reported Link Clock: %d.%.3d MHz\n",
			clock_rate / 1000, clock_rate % 1000);

	if (!link_disabled) {
		const char *status = link_status & 0x10 ?
				"SYNC~: deasserted\n" : "SYNC~: asserted\n";

		clock_rate = clk_get_rate(jesd->lane_clk);
		if (jesd->encoder == JESD204_ENCODER_64B66B) {
			link_rate = DIV_ROUND_CLOSEST(clock_rate, 66);
			lmfc_rate = (clock_rate * 8) /
				(66 * ((link_config0 & 0xFF) + 1));
		} else {
			link_rate = DIV_ROUND_CLOSEST(clock_rate, 40);
			lmfc_rate = clock_rate /
				(10 * ((link_config0 & 0xFF) + 1));
		}
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Lane rate: %d.%.3d MHz\n"
			"Lane rate / %d: %d.%.3d MHz\n"
			"%s rate: %d.%.3d MHz\n",
			clock_rate / 1000, clock_rate % 1000,
			(jesd->encoder == JESD204_ENCODER_8B10B) ? 40 : 66,
			link_rate / 1000, link_rate % 1000,
			(jesd->encoder == JESD204_ENCODER_8B10B) ? "LMFC" :
				"LEMC",
			lmfc_rate / 1000, lmfc_rate % 1000);

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"%sLink status: %s\n"
			"SYSREF captured: %s\n"
			"SYSREF alignment error: %s\n",
			jesd->encoder == JESD204_ENCODER_64B66B ? "" :
								status,
			axi_jesd204_tx_link_status_label[link_status & 0x3],
			(sysref_config & JESD204_TX_REG_SYSREF_CONF_SYSREF_DISABLE) ?
				"disabled" : (sysref_status & 1) ? "Yes" : "No",
			(sysref_config & JESD204_TX_REG_SYSREF_CONF_SYSREF_DISABLE) ?
				"disabled" : (sysref_status & 2) ? "Yes" : "No");
	} else {
		ret += scnprintf(buf + ret, PAGE_SIZE, "External reset is %s\n",
			(link_disabled & 0x2) ? "asserted" : "deasserted");
	}

	return ret;
}

static DEVICE_ATTR(status, 0444, axi_jesd204_tx_status_read, NULL);

static ssize_t encoder_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct axi_jesd204_tx *jesd = dev_get_drvdata(dev);

	return sprintf(buf, "%s", jesd204_encoder_str(jesd->encoder));
}

static DEVICE_ATTR_RO(encoder);

static irqreturn_t axi_jesd204_tx_irq(int irq, void *devid)
{
	struct axi_jesd204_tx *jesd = devid;
	unsigned int pending;

	pending = readl(jesd->base + JESD204_TX_REG_IRQ_PENDING);
	if (!pending)
		return IRQ_NONE;

	writel(pending, jesd->base + JESD204_TX_REG_IRQ_PENDING);

	return IRQ_HANDLED;
}

static unsigned int axi_jesd204_tx_calc_ilas_chksum(
	const struct jesd204_link *config,
	unsigned int lane_id)
{
	unsigned int chksum;

	chksum = config->device_id;
	chksum += config->bank_id;
	chksum += lane_id;
	chksum += config->num_lanes - 1;
	chksum += config->scrambling;
	chksum += config->octets_per_frame - 1;
	chksum += config->frames_per_multiframe - 1;
	chksum += config->num_converters - 1;
	chksum += config->ctrl_bits_per_sample;
	chksum += config->converter_resolution - 1;
	chksum += config->bits_per_sample - 1;
	chksum += config->subclass;
	chksum += config->samples_per_conv_frame ? config->samples_per_conv_frame - 1 : 0;
	chksum += config->jesd_version;
	chksum += config->high_density;

	return chksum & 0xff;
}

static void axi_jesd204_tx_set_lane_ilas(struct axi_jesd204_tx *jesd,
	struct jesd204_link *config, unsigned int lane_id, unsigned int lane)
{
	unsigned int i;
	unsigned int val;

	for (i = 0; i < 4; i++) {
		switch (i) {
		case 0:
			val = config->device_id << 8;
			val |= config->bank_id << 24;
			break;
		case 1:
			val = lane_id;
			val |= (config->num_lanes - 1) << 8;
			val |= config->scrambling << 15;
			val |= (config->octets_per_frame - 1) << 16;
			val |= (config->frames_per_multiframe - 1) << 24;
			break;
		case 2:
			val = (config->num_converters - 1);
			val |= (config->converter_resolution - 1) << 8;
			val |= config->ctrl_bits_per_sample << 14;
			val |= (config->bits_per_sample - 1) << 16;
			val |= config->subclass << 21;
			val |= (config->samples_per_conv_frame ? config->samples_per_conv_frame  - 1 : 0) << 24;
			val |= config->jesd_version << 29;
			break;
		case 3:
			val = config->high_density << 7;
			val |= axi_jesd204_tx_calc_ilas_chksum(config, lane_id) << 24;
			break;
		}

		writel_relaxed(val, jesd->base + JESD204_TX_REG_ILAS(lane, i));
	}
}

static int axi_jesd204_tx_apply_config(struct axi_jesd204_tx *jesd,
	struct jesd204_link *config)
{
	unsigned int octets_per_multiframe;
	unsigned int multiframe_align;
	unsigned int val;
	unsigned int lane, i;

	octets_per_multiframe = config->frames_per_multiframe *
		config->octets_per_frame;

	multiframe_align = 1 << jesd->data_path_width;

	if (jesd->encoder == JESD204_ENCODER_64B66B &&
	    (octets_per_multiframe % 256) != 0) {
		dev_err(jesd->dev, "octets_per_frame * frames_per_multiframe must be a multiple of 256");
		return -EINVAL;
	}

	if (octets_per_multiframe % multiframe_align != 0) {
		dev_err(jesd->dev,
			"octets_per_frame * frames_per_multiframe must be a multiple of  %d\n",
			multiframe_align);
		return -EINVAL;
	}

	val = (octets_per_multiframe - 1);
	val |= (config->octets_per_frame - 1) << 16;

	if (config->subclass == JESD204_SUBCLASS_0)
		writel_relaxed(JESD204_TX_REG_SYSREF_CONF_SYSREF_DISABLE,
			       jesd->base + JESD204_TX_REG_SYSREF_CONF);

	writel_relaxed(val, jesd->base + JESD204_TX_REG_CONF0);

	if (jesd->encoder == JESD204_ENCODER_8B10B) {
		for (i = 0, lane = 0; lane < jesd->num_lanes; lane++) {
			unsigned int lane_id;

			if (i >= config->num_lanes)
				i = 0;

			lane_id = config->lane_ids[i++];
			axi_jesd204_tx_set_lane_ilas(jesd, config, lane_id, lane);

		}
	}

	if (config->sysref.lmfc_offset != JESD204_LMFC_OFFSET_UNINITIALIZED)
		writel_relaxed(config->sysref.lmfc_offset,
			jesd->base + JESD204_TX_REG_SYSREF_LMFC_OFFSET);

	return 0;
}

/*
 * FIXME: This is temporary. Configuration data is not part of the hardware
 * description and does not belong into the devicetree.
 */
static int axi_jesd204_tx_parse_dt_config(struct device_node *np,
	struct axi_jesd204_tx *jesd, struct jesd204_link *config)
{
	int ret;
	u32 val;

	config->device_id = 0;
	config->bank_id = 0;
	config->scrambling = true;
	config->num_lanes = jesd->num_lanes;
	config->jesd_version = JESD204_VERSION_B;
	config->subclass = JESD204_SUBCLASS_1;
	config->ctrl_bits_per_sample = 0;
	config->samples_per_conv_frame = 1;
	config->sysref.lmfc_offset = 0;

	ret = of_property_read_u32(np, "adi,octets-per-frame", &val);
	if (ret)
		return ret;
	config->octets_per_frame = val;

	ret = of_property_read_u32(np, "adi,frames-per-multiframe", &val);
	if (ret)
		return ret;
	config->frames_per_multiframe = val;

	config->high_density = of_property_read_bool(np, "adi,high-density");

	ret = of_property_read_u32(np, "adi,converter-resolution", &val);
	if (ret)
		return ret;
	config->converter_resolution = val;

	ret = of_property_read_u32(np, "adi,bits-per-sample", &val);
	if (ret)
		return ret;
	config->bits_per_sample = val;

	ret = of_property_read_u32(np, "adi,converters-per-device", &val);
	if (ret)
		return ret;
	config->num_converters = val;

	/* optional */
	ret = of_property_read_u32(np, "adi,control-bits-per-sample", &val);
	if (ret == 0)
		config->ctrl_bits_per_sample = val;

	ret = of_property_read_u32(np, "adi,subclass", &val);
	if (ret == 0)
		config->subclass = val;

	ret = of_property_read_u32(np, "adi,sysref-lmfc-offset", &val);
	if (ret == 0)
		config->sysref.lmfc_offset = val;

	return 0;
}

static bool axi_jesd_tx_regmap_rdwr(struct device *dev, unsigned int reg)
{
	struct axi_jesd204_tx *jesd = dev_get_drvdata(dev);
	unsigned int i;

	switch (reg) {
	case ADI_AXI_REG_VERSION:
	case ADI_AXI_REG_ID:
	case ADI_AXI_REG_SCRATCH:
	case JESD204_TX_REG_MAGIC:
	case JESD204_TX_REG_CONF_NUM_LANES:
	case JESD204_TX_REG_CONF_DATA_PATH_WIDTH:
	case JESD204_TX_REG_IRQ_ENABLE:
	case JESD204_TX_REG_IRQ_PENDING:
	case JESD204_TX_REG_IRQ_SOURCE:
	case JESD204_TX_REG_LINK_DISABLE:
	case JESD204_TX_REG_LINK_STATE:
	case JESD204_TX_REG_LINK_CLK_RATIO:
	case JESD204_TX_REG_LANES_ENABLE:
	case JESD204_TX_REG_CONF0:
	case JESD204_TX_REG_CONF1:
	case JESD204_TX_REG_CONF2:
	case JESD204_TX_REG_CONF3:
	case JESD204_TX_REG_LINK_STATUS:
	case JESD204_TX_REG_MANUAL_SYNC_REQUEST:
	case JESD204_TX_REG_SYSREF_CONF:
	case JESD204_TX_REG_SYSREF_LMFC_OFFSET:
	case JESD204_TX_REG_SYSREF_STATUS:
		return true;
	default:
		break;
	}

	for (i = 0; i < jesd->num_lanes; i++) {
		if (reg == JESD204_TX_REG_ILAS(i, 0) ||
			reg == JESD204_TX_REG_ILAS(i, 1) ||
			reg == JESD204_TX_REG_ILAS(i, 2) ||
			reg == JESD204_TX_REG_ILAS(i, 3))
			return true;
	}

	return false;
}

static const struct regmap_config axi_jesd_tx_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x800,
	.readable_reg = axi_jesd_tx_regmap_rdwr,
	.writeable_reg = axi_jesd_tx_regmap_rdwr,
};

static int axi_jesd204_tx_lane_clk_enable(struct clk_hw *clk)
{
	struct axi_jesd204_tx *jesd =
		container_of(clk, struct axi_jesd204_tx, dummy_clk);

	writel_relaxed(0x3, jesd->base + JESD204_TX_REG_SYSREF_STATUS);
	writel_relaxed(0x0, jesd->base + JESD204_TX_REG_LINK_DISABLE);

	return 0;
}

static void axi_jesd204_tx_lane_clk_disable(struct clk_hw *clk)
{
	struct axi_jesd204_tx *jesd =
		container_of(clk, struct axi_jesd204_tx, dummy_clk);

	writel_relaxed(0x1, jesd->base + JESD204_TX_REG_LINK_DISABLE);
}

static const struct clk_ops axi_jesd204_tx_dummy_clk_ops = {
	.enable = axi_jesd204_tx_lane_clk_enable,
	.disable = axi_jesd204_tx_lane_clk_disable,
};

/* FIXME: This is terrible and needs to be replaced */
static int axi_jesd204_register_dummy_clk(struct axi_jesd204_tx *jesd,
	struct device *dev)
{
	struct device_node *np = dev->of_node;
	const char *parent_name, *clk_name;
	struct clk_init_data init;
	struct clk *dummy_clk;
	int ret;

	ret = of_property_read_string(np, "clock-output-names",
		&clk_name);
	if (ret < 0)
		return ret;

	init.name = clk_name;
	init.ops = &axi_jesd204_tx_dummy_clk_ops;
	init.flags = CLK_SET_RATE_PARENT;

	parent_name = __clk_get_name(jesd->lane_clk);
	init.parent_names = &parent_name;
	init.num_parents = 1;

	jesd->dummy_clk.init = &init;

	dummy_clk = devm_clk_register(dev, &jesd->dummy_clk);
	if (IS_ERR(dummy_clk))
		return PTR_ERR(dummy_clk);

	of_clk_add_provider(np, of_clk_src_simple_get, dummy_clk);

	return 0;
}

static int axi_jesd204_tx_pcore_check(struct axi_jesd204_tx *jesd)
{
	unsigned int magic, version;

	magic = readl_relaxed(jesd->base + JESD204_TX_REG_MAGIC);
	if (magic != JESD204_TX_MAGIC) {
		dev_err(jesd->dev, "Unexpected peripheral identifier %.08x\n",
			magic);
		return -ENODEV;
	}

	version = readl_relaxed(jesd->base + ADI_AXI_REG_VERSION);
	if (ADI_AXI_PCORE_VER_MAJOR(version) != 1) {
		dev_err(jesd->dev, "Unsupported peripheral version %u.%u.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(version),
			ADI_AXI_PCORE_VER_MINOR(version),
			ADI_AXI_PCORE_VER_PATCH(version));
		return -ENODEV;
	}

	return 0;
}

static int axi_jesd204_tx_jesd204_link_setup(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct axi_jesd204_tx *jesd = dev_get_drvdata(dev);
	unsigned long link_rate, lane_rate;
	long rate;
	int ret;

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	case JESD204_STATE_OP_REASON_UNINIT:
		clk_disable_unprepare(jesd->lane_clk);
		clk_disable_unprepare(jesd->device_clk);
		return JESD204_STATE_CHANGE_DONE;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	if (jesd->num_lanes != lnk->num_lanes)
		jesd204_notice(jdev,
				"Possible instantiation for multiple chips; HDL lanes %u, Link[%u] lanes %u\n",
				jesd->num_lanes, lnk->link_id, lnk->num_lanes);

	ret = axi_jesd204_tx_apply_config(jesd, lnk);
	if (ret) {
		dev_err(dev, "%s: Apply config Link%u failed (%d)\n",
			__func__, lnk->link_id, ret);
		return ret;
	}

	ret = jesd204_link_get_device_clock(lnk, &link_rate);
	dev_dbg(dev, "%s: Link%u device clock rate %lu (%d)\n",
		__func__, lnk->link_id, link_rate, ret);
	if (ret) {
		dev_err(dev, "%s: Link%u get device clock rate failed (%d)\n",
			__func__, lnk->link_id, ret);
		return ret;
	}

	ret = jesd204_link_get_rate_khz(lnk, &lane_rate);
	dev_dbg(dev, "%s: Link%u lane rate %lu (%d)\n",
		__func__, lnk->link_id, lane_rate, ret);
	if (ret) {
		dev_err(dev, "%s: Link%u get rate failed (%d)\n",
			__func__, lnk->link_id, ret);
		return ret;
	}

	ret = clk_set_rate(jesd->device_clk, link_rate);
	if (ret) {
		dev_err(dev, "%s: Link%u set device clock rate %lu Hz failed (%d)\n",
			__func__, lnk->link_id, link_rate, ret);
		return ret;
	}

	rate = clk_get_rate(jesd->lane_clk);

	if (rate != lane_rate) {
		rate = clk_round_rate(jesd->lane_clk, lane_rate);
		if (rate != (long)lane_rate) {
			struct clk *parent;

			/*
			 * Check GT QPLL/CPLL reference clock and make
			 * it equal to the link/device rate
			 */
			parent = clk_get_parent(jesd->lane_clk);
			rate = clk_get_rate(parent);

			if (rate != (long)link_rate) {
				rate = clk_round_rate(parent, link_rate);
				if (rate == (long)link_rate) {
					ret = clk_set_rate(parent, link_rate);
					if (!ret && !IS_ERR(jesd->conv2_clk))
						ret = clk_set_rate(jesd->conv2_clk, link_rate);
				} else {
					ret = -EINVAL;
				}
				if (ret < 0) {
					dev_err(dev, "%s: Link%u set REFCLK to device/link rate %lu Hz failed (%d)\n",
						__func__, lnk->link_id, link_rate, ret);
				}
			}
		}
		ret = clk_set_rate(jesd->lane_clk, lane_rate);
		if (ret) {
			dev_err(dev, "%s: Link%u set lane rate %lu kHz failed (%d)\n",
				__func__, lnk->link_id, lane_rate, ret);
			return ret;
		}
	}

	ret = clk_prepare_enable(jesd->device_clk);
	if (ret) {
		dev_err(dev, "%s: Link%u enable device clock failed (%d)\n",
			__func__, lnk->link_id, ret);
		return ret;
	}

	ret = clk_prepare_enable(jesd->lane_clk);
	if (ret) {
		clk_disable_unprepare(jesd->device_clk);
		dev_err(dev, "%s: Link%u enable lane clock failed (%d)\n",
			__func__, lnk->link_id, ret);
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int axi_jesd204_tx_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct axi_jesd204_tx *jesd = dev_get_drvdata(dev);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	writel_relaxed(0x1, jesd->base + JESD204_TX_REG_LINK_DISABLE);
	udelay(1);
	writel_relaxed(0x3, jesd->base + JESD204_TX_REG_SYSREF_STATUS);
	writel_relaxed(0x0, jesd->base + JESD204_TX_REG_LINK_DISABLE);

	return JESD204_STATE_CHANGE_DONE;
}

static int axi_jesd204_tx_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct axi_jesd204_tx *jesd = dev_get_drvdata(dev);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	case JESD204_STATE_OP_REASON_UNINIT:
		writel_relaxed(0x1, jesd->base + JESD204_TX_REG_LINK_DISABLE);
		return JESD204_STATE_CHANGE_DONE;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int axi_jesd204_tx_jesd204_link_running(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct axi_jesd204_tx *jesd = dev_get_drvdata(dev);
	unsigned int link_status;
	int retry = 10;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason == JESD204_STATE_OP_REASON_INIT) {
		do {
			msleep(4);
			link_status = readl_relaxed(jesd->base + JESD204_TX_REG_LINK_STATUS) & 0x3;
		} while (link_status != JESD204_LINK_STATUS_DATA && retry--);

		if (link_status != JESD204_LINK_STATUS_DATA) {
			dev_err(dev, "%s: Link%u status failed (%s)\n",
				__func__, lnk->link_id,
				axi_jesd204_tx_link_status_label[link_status]);

			return JESD204_STATE_CHANGE_ERROR;
		}
	}

	return JESD204_STATE_CHANGE_DONE;
}
static const struct jesd204_dev_data jesd204_axi_jesd204_tx_init = {
	.state_ops = {
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = axi_jesd204_tx_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_SETUP] = {
			.per_link = axi_jesd204_tx_jesd204_link_setup,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = axi_jesd204_tx_jesd204_link_enable,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = axi_jesd204_tx_jesd204_link_running,
		},
	},
};

static int axi_jesd204_init_non_framework(struct device *dev,
					  struct axi_jesd204_tx *jesd)
{
	struct jesd204_link config;
	unsigned int lane;
	int ret;

	ret = axi_jesd204_tx_parse_dt_config(dev->of_node, jesd, &config);
	if (ret)
		return ret;

	config.lane_ids = devm_kcalloc(dev, jesd->num_lanes,
				       sizeof(*config.lane_ids),
				       GFP_KERNEL);
	if (!config.lane_ids)
		return -ENOMEM;

	for (lane = 0; lane < jesd->num_lanes; lane++)
		config.lane_ids[lane] = lane;

	ret = axi_jesd204_tx_apply_config(jesd, &config);
	if (ret)
		return ret;

	return axi_jesd204_register_dummy_clk(jesd, dev);
}

static int axi_jesd204_tx_probe(struct platform_device *pdev)
{
	struct axi_jesd204_tx *jesd;
	struct jesd204_dev *jdev;
	struct resource *res;
	int irq;
	int ret;
	u32 synth_1;

	if (!pdev->dev.of_node)
		return -ENODEV;

	jdev = devm_jesd204_dev_register(&pdev->dev,
					 &jesd204_axi_jesd204_tx_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	if (irq == 0)
		return -ENXIO;

	jesd = devm_kzalloc(&pdev->dev, sizeof(*jesd), GFP_KERNEL);
	if (!jesd)
		return -ENOMEM;

	jesd->dev = &pdev->dev;
	jesd->jdev = jdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	jesd->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jesd->base))
		return PTR_ERR(jesd->base);

	devm_regmap_init_mmio(&pdev->dev, jesd->base, &axi_jesd_tx_regmap_config);

	ret = axi_jesd204_tx_pcore_check(jesd);
	if (ret)
		return ret;

	jesd->axi_clk = devm_clk_get(&pdev->dev, "s_axi_aclk");
	if (IS_ERR(jesd->axi_clk))
		return PTR_ERR(jesd->axi_clk);

	jesd->device_clk = devm_clk_get(&pdev->dev, "device_clk");
	if (IS_ERR(jesd->device_clk))
		return PTR_ERR(jesd->device_clk);

	jesd->lane_clk = devm_clk_get(&pdev->dev, "lane_clk");
	if (IS_ERR(jesd->lane_clk))
		return PTR_ERR(jesd->lane_clk);

	/*
	 * Optional CPLL/QPLL REFCLK from a difference source
	 * which rate and state must be in sync with the main conv clk
	 * This is used in axi_jesd204_rx_jesd204_link_setup() where the
	 * main REFCLK is the parent of jesd->lane_clk.
	 */
	jesd->conv2_clk = devm_clk_get(&pdev->dev, "conv2");
	if (IS_ERR(jesd->conv2_clk)) {
		if (PTR_ERR(jesd->conv2_clk) != -ENOENT)
			return PTR_ERR(jesd->conv2_clk);
		jesd->conv2_clk = NULL;
	}

	ret = clk_prepare_enable(jesd->axi_clk);
	if (ret)
		return ret;

	if (jesd->conv2_clk) {
		ret = clk_prepare_enable(jesd->conv2_clk);
		if (ret)
			goto err_axi_clk_disable;
	}

	jesd->num_lanes = readl_relaxed(jesd->base + JESD204_TX_REG_CONF_NUM_LANES);
	jesd->data_path_width = readl_relaxed(jesd->base + JESD204_TX_REG_CONF_DATA_PATH_WIDTH);

	synth_1 = readl_relaxed(jesd->base + JESD204_REG_SYNTH_REG_1);
	jesd->encoder = JESD204_ENCODER_GET(synth_1);

	/* backward compatibility with older HDL cores */
	if (jesd->encoder == JESD204_ENCODER_UNKNOWN) {
		jesd->encoder = JESD204_ENCODER_8B10B;
	} else if (jesd->encoder >= JESD204_ENCODER_MAX) {
		dev_err(&pdev->dev, "Invalid encoder value from HDL core %u\n",
			jesd->encoder);
		goto err_conv2_clk_disable;
	}

	if (!jesd->jdev) {
		ret = axi_jesd204_init_non_framework(&pdev->dev, jesd);
		if (ret)
			goto err_conv2_clk_disable;
	}

	writel_relaxed(0xff, jesd->base + JESD204_TX_REG_IRQ_PENDING);
	writel_relaxed(0x00, jesd->base + JESD204_TX_REG_IRQ_ENABLE);

	ret = request_irq(irq, axi_jesd204_tx_irq, 0, dev_name(&pdev->dev),
		jesd);
	if (ret)
		goto err_uninit_non_framework;

	device_create_file(&pdev->dev, &dev_attr_status);
	device_create_file(&pdev->dev, &dev_attr_encoder);

	platform_set_drvdata(pdev, jesd);

	ret = jesd204_fsm_start(jesd->jdev, JESD204_LINKS_ALL);
	if (ret)
		goto err_remove_debugfs;

	return 0;
err_remove_debugfs:
	device_remove_file(&pdev->dev, &dev_attr_status);
	device_remove_file(&pdev->dev, &dev_attr_encoder);
	free_irq(irq, jesd);
err_uninit_non_framework:
	if (!jesd->jdev)
		 of_clk_del_provider(pdev->dev.of_node);
err_conv2_clk_disable:
	clk_disable_unprepare(jesd->conv2_clk);
err_axi_clk_disable:
	clk_disable_unprepare(jesd->axi_clk);

	return ret;
}

static int axi_jesd204_tx_remove(struct platform_device *pdev)
{
	struct axi_jesd204_tx *jesd = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

	jesd204_fsm_stop(jesd->jdev, JESD204_LINKS_ALL);

	device_remove_file(&pdev->dev, &dev_attr_status);
	device_remove_file(&pdev->dev, &dev_attr_encoder);

	free_irq(irq, jesd);

	if (!jesd->jdev)
		of_clk_del_provider(pdev->dev.of_node);

	writel_relaxed(0xff, jesd->base + JESD204_TX_REG_IRQ_PENDING);
	writel_relaxed(0x00, jesd->base + JESD204_TX_REG_IRQ_ENABLE);

	writel_relaxed(0x1, jesd->base + JESD204_TX_REG_LINK_DISABLE);

	clk_disable_unprepare(jesd->conv2_clk);
	clk_disable_unprepare(jesd->axi_clk);

	return 0;
}

static const struct of_device_id axi_jesd204_tx_of_match[] = {
	{ .compatible = "adi,axi-jesd204-tx-1.0" },
	{ .compatible = "adi,axi-jesd204-tx-1.3" },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adxcvr_of_match);

static struct platform_driver axi_jesd204_tx_driver = {
	.probe = axi_jesd204_tx_probe,
	.remove = axi_jesd204_tx_remove,
	.driver = {
		.name = "axi-jesd204-tx",
		.of_match_table = axi_jesd204_tx_of_match,
	},
};
module_platform_driver(axi_jesd204_tx_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for the Analog Devices AXI-JESD204-TX peripheral");
MODULE_LICENSE("GPL v2");
