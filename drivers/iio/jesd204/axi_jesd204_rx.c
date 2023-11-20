/*
 * Driver for the Analog Devices AXI-JESD204-RX peripheral
 *
 * Copyright 2017 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/bitfield.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/fpga/adi-axi-common.h>

#include <linux/jesd204/jesd204.h>

#include "axi_jesd204.h"

#define JESD204_RX_REG_MAGIC				0x0c

#define JESD204_RX_REG_SYNTH_NUM_LANES			0x10
#define JESD204_RX_REG_SYNTH_1				0x18
#define JESD204_RX_REG_SYNTH_ELASTIC_BUFFER_SIZE	0x40

#define JESD204_RX_REG_IRQ_ENABLE			0x80
#define JESD204_RX_REG_IRQ_PENDING			0x84
#define JESD204_RX_REG_IRQ_SOURCE			0x88

#define JESD204_RX_REG_LINK_DISABLE			0xc0
#define JESD204_RX_REG_LINK_STATE			0xc4
#define JESD204_RX_REG_LINK_CLK_RATIO			0xc8
#define JESD204_RX_REG_DEVICE_CLK_RATIO			0xcc

#define JESD204_RX_REG_SYSREF_CONF			0x100
#define JESD204_RX_REG_SYSREF_LMFC_OFFSET		0x104
#define JESD204_RX_REG_SYSREF_STATUS			0x108

#define JESD204_RX_REG_LANES_ENABLE			0x200
#define JESD204_RX_REG_LINK_CONF0			0x210
#define JESD204_RX_REG_LINK_CONF1			0x214
#define JESD204_RX_REG_LINK_CONF2			0x240
#define JESD204_RX_REG_LINK_CONF3			0x244
#define JESD204_RX_REG_LINK_CONF4			0x21C

#define JESD204_RX_REG_LINK_STATUS			0x280

#define JESD204_RX_REG_LANE_STATUS(x)		(((x) * 32) + 0x300)
#define JESD204_EMB_STATE_MASK			GENMASK(10, 8)
#define JESD204_EMB_STATE_GET(x) \
			FIELD_GET(JESD204_EMB_STATE_MASK, x)
#define JESD204_RX_REG_LANE_LATENCY(x)		(((x) * 32) + 0x304)
#define JESD204_RX_REG_LANE_ERRORS(x)		(((x) * 32) + 0x308)
#define JESD204_RX_REG_ILAS(x, y)		(((x) * 32 + (y) * 4) + 0x310)

#define JESD204_RX_MAGIC (('2' << 24) | ('0' << 16) | ('4' << 8) | ('R'))

/* JESD204_RX_REG_SYSREF_CONF */
#define JESD204_RX_REG_SYSREF_CONF_SYSREF_DISABLE	BIT(0)

/* JESD204_RX_REG_LINK_CONF2 */
#define JESD204_RX_LINK_CONF2_BUFFER_EARLY_RELEASE	BIT(16)

/* JESD204_RX_REG_IRQ_ENABLE */
#define JESD204_RX_IRQ_FRAME_ALIGNMENT_ERROR		BIT(0) /* 204B only */
#define JESD204_RX_IRQ_UNEXP_LANE_STATE_ERROR		BIT(1)

/* JESD204_RX_REG_LINK_STATUS */
#define JESD204_LINK_STATUS_DATA			3

struct axi_jesd204_rx {
	void __iomem *base;
	struct device *dev;

	struct clk *axi_clk;
	struct clk *device_clk;
	struct clk *link_clk;
	struct clk *conv2_clk;
	struct clk *sysref_clk;

	struct jesd204_dev *jdev;

	unsigned long axi_clk_freq;
	unsigned int irq;

	unsigned int num_lanes;
	unsigned int data_path_width;
	unsigned int tpl_data_path_width;
	unsigned int version;
	enum jesd204_encoder encoder;

	struct delayed_work watchdog_work;

	/* Used for probe ordering */
	struct clk_hw dummy_clk;
	struct clk *lane_clk;
};

enum {
	JESD204_EMB_STATE_INIT = 1,
	JESD204_EMB_STATE_HUNT,
	JESD204_EMB_STATE_LOCK = 4
};

static const char * const axi_jesd204_rx_link_status_label[] = {
	"RESET",
	"WAIT FOR PHY",
	"CGS",
	"DATA",
};

static const char * const axi_jesd204_rx_link_status_64b66b_l[] = {
	"RESET",
	"WAIT_BS",
	"BLOCK_SYNC",
	"DATA",
};

static unsigned long axi_jesd204_rx_calc_device_clk(struct axi_jesd204_rx *jesd,
	unsigned long link_rate)
{
	if (jesd->version >= ADI_AXI_PCORE_VER(1, 7, 'a'))
		return div_u64((u64) link_rate * jesd->data_path_width,
			jesd->tpl_data_path_width);

	return link_rate;
}

static ssize_t axi_jesd204_rx_status_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct axi_jesd204_rx *jesd = dev_get_drvdata(dev);
	unsigned int sysref_status;
	unsigned int link_disabled;
	unsigned int link_status;
	unsigned int link_config0;
	unsigned int clock_ratio;
	unsigned int clock_rate;
	unsigned int link_rate;
	unsigned int lane_rate;
	unsigned int sysref_config;
	unsigned int lmfc_rate;
	int ret;

	link_disabled = readl_relaxed(jesd->base + JESD204_RX_REG_LINK_STATE);
	link_status = readl_relaxed(jesd->base + JESD204_RX_REG_LINK_STATUS);
	sysref_status = readl_relaxed(jesd->base + JESD204_RX_REG_SYSREF_STATUS);
	clock_ratio = readl_relaxed(jesd->base + JESD204_RX_REG_LINK_CLK_RATIO);
	sysref_config = readl_relaxed(jesd->base + JESD204_RX_REG_SYSREF_CONF);
	link_config0 = readl_relaxed(jesd->base + JESD204_RX_REG_LINK_CONF0);

	ret = scnprintf(buf, PAGE_SIZE, "Link is %s\n",
		(link_disabled & 0x1) ? "disabled" : "enabled");

	if (clock_ratio == 0) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Measured Link Clock: off\n");
	} else {
		clock_rate = DIV_ROUND_CLOSEST_ULL(DIV_ROUND_CLOSEST(jesd->axi_clk_freq,
			1000) * clock_ratio, 1ULL << 16);

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Measured Link Clock: %d.%.3d MHz\n",
			clock_rate / 1000, clock_rate % 1000);
	}

	clock_rate = DIV_ROUND_CLOSEST(clk_get_rate(IS_ERR_OR_NULL(jesd->link_clk) ?
		 jesd->device_clk : jesd->link_clk), 1000);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"Reported Link Clock: %d.%.3d MHz\n",
		clock_rate / 1000, clock_rate % 1000);

	lane_rate = clk_get_rate(jesd->lane_clk);
	if (jesd->encoder == JESD204_ENCODER_64B66B) {
		link_rate = DIV_ROUND_CLOSEST(lane_rate, 66);
		lmfc_rate = (lane_rate * 8) /
			(66 * ((link_config0 & 0x3FF) + 1));
	} else {
		link_rate = DIV_ROUND_CLOSEST(lane_rate, 40);
		lmfc_rate = lane_rate /
			(10 * ((link_config0 & 0x3FF) + 1));
	}

	if (jesd->version >= ADI_AXI_PCORE_VER(1, 7, 'a')) {
		clock_ratio = readl_relaxed(jesd->base + JESD204_RX_REG_DEVICE_CLK_RATIO);
		if (clock_ratio == 0) {
			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"Measured Device Clock: off\n");
		} else {
			clock_rate = DIV_ROUND_CLOSEST_ULL(DIV_ROUND_CLOSEST(jesd->axi_clk_freq,
				1000) * clock_ratio, 1ULL << 16);

			ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"Measured Device Clock: %d.%.3d MHz\n",
				clock_rate / 1000, clock_rate % 1000);
		}

		clock_rate = DIV_ROUND_CLOSEST(clk_get_rate(jesd->device_clk), 1000);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Reported Device Clock: %d.%.3d MHz\n",
			clock_rate / 1000, clock_rate % 1000);

		clock_rate = axi_jesd204_rx_calc_device_clk(jesd, link_rate);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Desired Device Clock: %d.%.3d MHz\n",
			clock_rate / 1000, clock_rate % 1000);
	}

	if (!link_disabled) {
		const char *_status = (jesd->encoder == JESD204_ENCODER_8B10B) ?
			axi_jesd204_rx_link_status_label[link_status & 0x3] :
			axi_jesd204_rx_link_status_64b66b_l[link_status & 0x3];

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Lane rate: %d.%.3d MHz\n"
			"Lane rate / %d: %d.%.3d MHz\n"
			"%s rate: %d.%.3d MHz\n",
			lane_rate / 1000, lane_rate % 1000,
			(jesd->encoder == JESD204_ENCODER_8B10B) ? 40 : 66,
			link_rate / 1000, link_rate % 1000,
			(jesd->encoder == JESD204_ENCODER_8B10B) ? "LMFC" :
				"LEMC",
			lmfc_rate / 1000, lmfc_rate % 1000);

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Link status: %s\n"
			"SYSREF captured: %s\n"
			"SYSREF alignment error: %s\n",
			_status,
			(sysref_config & JESD204_RX_REG_SYSREF_CONF_SYSREF_DISABLE) ?
				"disabled" : (sysref_status & 1) ? "Yes" : "No",
			(sysref_config & JESD204_RX_REG_SYSREF_CONF_SYSREF_DISABLE) ?
				"disabled" : (sysref_status & 2) ? "Yes" : "No");
	} else {
		ret += scnprintf(buf + ret, PAGE_SIZE, "External reset is %s\n",
			(link_disabled & 0x2) ? "asserted" : "deasserted");
	}

	return ret;
}

static DEVICE_ATTR(status, 0444, axi_jesd204_rx_status_read, NULL);

static ssize_t encoder_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct axi_jesd204_rx *jesd = dev_get_drvdata(dev);

	return sprintf(buf, "%s", jesd204_encoder_str(jesd->encoder));
}

static DEVICE_ATTR_RO(encoder);

static const char *const axi_jesd204_rx_lane_status_label[] = {
	"INIT",
	"CHECK",
	"DATA",
	"UNKNOWN",
};

static unsigned int axi_jesd204_rx_get_lane_errors(struct axi_jesd204_rx *jesd,
	unsigned int lane)
{
	return readl_relaxed(jesd->base + JESD204_RX_REG_LANE_ERRORS(lane));
}

/* FIXME: This violates every single sysfs ABI recommendation */
static ssize_t __axi_jesd204_rx_laneinfo_8b10b_read(struct axi_jesd204_rx *jesd,
						    const u32 lane,
						    const u32 lane_status,
						    char *buf,
						    const int pos)
{
	int ret = pos;
	u32 lane_latency;
	u32 val[4];
	u32 octets_per_multiframe;

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "CGS state: %s\n",
		axi_jesd204_rx_lane_status_label[lane_status & 0x3]);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Initial Frame Synchronization: %s\n",
				(lane_status & BIT(4)) ? "Yes" : "No");
	if (!(lane_status & BIT(4)))
		return ret;

	octets_per_multiframe = readl_relaxed(jesd->base + JESD204_RX_REG_LINK_CONF0);
	octets_per_multiframe &= 0xffff;
	octets_per_multiframe += 1;

	lane_latency = readl_relaxed(jesd->base + JESD204_RX_REG_LANE_LATENCY(lane));
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Lane Latency: %d Multi-frames and %d Octets\n",
			lane_latency / octets_per_multiframe,
			lane_latency % octets_per_multiframe);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Initial Lane Alignment Sequence: %s\n",
				(lane_status & BIT(5)) ? "Yes" : "No");

	if (!(lane_status & BIT(5)))
		return ret;

	val[0] = readl_relaxed(jesd->base + JESD204_RX_REG_ILAS(lane, 0));
	val[1] = readl_relaxed(jesd->base + JESD204_RX_REG_ILAS(lane, 1));
	val[2] = readl_relaxed(jesd->base + JESD204_RX_REG_ILAS(lane, 2));
	val[3] = readl_relaxed(jesd->base + JESD204_RX_REG_ILAS(lane, 3));

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"DID: %d, BID: %d, LID: %d, L: %d, SCR: %d, F: %d\n",
		(val[0] >> 16) & 0xff,
		(val[0] >> 24) & 0xf,
		(val[1] >> 0) & 0x1f,
		((val[1] >> 8) & 0x1f) + 1,
		(val[1] >> 15) & 0x1,
		((val[1] >> 16) & 0xff) + 1
	);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"K: %d, M: %d, N: %d, CS: %d, N': %d, S: %d, HD: %d\n",
		((val[1] >> 24) & 0x1f) + 1,
		((val[2] >> 0) & 0xff) + 1,
		((val[2] >> 8) & 0x1f) + 1,
		(val[2] >> 14) & 0x3,
		((val[2] >> 16) & 0x1f) + 1,
		((val[2] >> 24) & 0x1f) + 1,
		(val[3] >> 7) & 0x1
	);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"FCHK: 0x%X, CF: %d\n",
		(val[3] >> 24) & 0xff,
		(val[3] >> 0) & 0x1f
	);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"ADJCNT: %d, PHADJ: %d, ADJDIR: %d, JESDV: %d, SUBCLASS: %d\n",
		(val[0] >> 28) & 0xff,
		(val[1] >> 5) & 0x1,
		(val[1] >> 6) & 0x1,
		(val[2] >> 29) & 0x7,
		(val[2] >> 21) & 0x7
	);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
		"FC: %lu\n", clk_get_rate(jesd->lane_clk));

	return ret;
}

static const char *const axi_jesd204_rx_emb_state_label[] = {
	"INVALID",
	"EMB_INIT",
	"EMB_HUNT",
	"INVALID",
	"EMB_LOCK",
	"INVALID",
	"INVALID",
	"INVALID",
};

static int __axi_jesd204_rx_laneinfo_64b66b_read(struct axi_jesd204_rx *jesd,
						 const u32 lane,
						 const u32 lane_status,
						 char *buf,
						 const int pos)
{
	int ret = pos;
	u8 extend_multiblock;

	extend_multiblock = JESD204_EMB_STATE_GET(lane_status);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			 "State of Extended multiblock alignment:%s\n",
			 axi_jesd204_rx_emb_state_label[extend_multiblock]);

	return ret;
}

/* FIXME: This violates every single sysfs ABI recommendation */
static ssize_t axi_jesd204_rx_laneinfo_read(struct device *dev,
					    struct device_attribute *attr,
					    char *buf, unsigned int lane)
{
	struct axi_jesd204_rx *jesd = dev_get_drvdata(dev);
	unsigned int lane_status;
	unsigned int errors;
	int ret = 0;

	lane_status = readl_relaxed(jesd->base +
				    JESD204_RX_REG_LANE_STATUS(lane));

	if (jesd->version >= ADI_AXI_PCORE_VER(1, 2, 'a')) {
		errors = axi_jesd204_rx_get_lane_errors(jesd, lane);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "Errors: %u\n",
				 errors);
	}

	if (jesd->encoder == JESD204_ENCODER_8B10B)
		ret = __axi_jesd204_rx_laneinfo_8b10b_read(jesd, lane,
							   lane_status, buf,
							   ret);
	else if (jesd->encoder == JESD204_ENCODER_64B66B)
		ret = __axi_jesd204_rx_laneinfo_64b66b_read(jesd, lane,
							    lane_status, buf,
							    ret);

	return ret;
}

#define JESD_LANE(_x) \
static ssize_t axi_jesd204_rx_lane##_x##_info_read(struct device *dev, \
			struct device_attribute *attr, char *buf) \
{ \
	return axi_jesd204_rx_laneinfo_read(dev, attr, buf, _x); \
} \
static DEVICE_ATTR(lane##_x##_info, S_IRUSR, axi_jesd204_rx_lane##_x##_info_read, NULL)

JESD_LANE(0);
JESD_LANE(1);
JESD_LANE(2);
JESD_LANE(3);
JESD_LANE(4);
JESD_LANE(5);
JESD_LANE(6);
JESD_LANE(7);
JESD_LANE(8);
JESD_LANE(9);
JESD_LANE(10);
JESD_LANE(11);
JESD_LANE(12);
JESD_LANE(13);
JESD_LANE(14);
JESD_LANE(15);
JESD_LANE(16);
JESD_LANE(17);
JESD_LANE(18);
JESD_LANE(19);
JESD_LANE(20);
JESD_LANE(21);
JESD_LANE(22);
JESD_LANE(23);

static const struct device_attribute *jesd204_rx_lane_devattrs[] = {
	&dev_attr_lane0_info,
	&dev_attr_lane1_info,
	&dev_attr_lane2_info,
	&dev_attr_lane3_info,
	&dev_attr_lane4_info,
	&dev_attr_lane5_info,
	&dev_attr_lane6_info,
	&dev_attr_lane7_info,
	&dev_attr_lane8_info,
	&dev_attr_lane9_info,
	&dev_attr_lane10_info,
	&dev_attr_lane11_info,
	&dev_attr_lane12_info,
	&dev_attr_lane13_info,
	&dev_attr_lane14_info,
	&dev_attr_lane15_info,
	&dev_attr_lane16_info,
	&dev_attr_lane17_info,
	&dev_attr_lane18_info,
	&dev_attr_lane19_info,
	&dev_attr_lane20_info,
	&dev_attr_lane21_info,
	&dev_attr_lane22_info,
	&dev_attr_lane23_info,
};

static irqreturn_t axi_jesd204_rx_irq(int irq, void *devid)
{
	struct axi_jesd204_rx *jesd = devid;
	unsigned int pending;

	pending = readl(jesd->base + JESD204_RX_REG_IRQ_PENDING);
	dev_dbg(jesd->dev, "%s: pending 0x%X\n", __func__, pending);
	if (!pending)
		return IRQ_NONE;

	writel_relaxed(0x1, jesd->base + JESD204_RX_REG_LINK_DISABLE);
	udelay(1);
	writel_relaxed(0x3, jesd->base + JESD204_RX_REG_SYSREF_STATUS);
	writel_relaxed(0x0, jesd->base + JESD204_RX_REG_LINK_DISABLE);

	writel(pending, jesd->base + JESD204_RX_REG_IRQ_PENDING);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t axi_jesd204_rx_irq_thread_fn(int irq, void *devid)
{
	struct axi_jesd204_rx *jesd = devid;

	jesd204_sysref_async_force(jesd->jdev);

	return IRQ_HANDLED;
}

static int axi_jesd204_rx_apply_config(struct axi_jesd204_rx *jesd,
	struct jesd204_link *config)
{
	unsigned int octets_per_multiframe;
	unsigned int val;
	unsigned int multiframe_align;

	octets_per_multiframe = config->frames_per_multiframe *
		config->octets_per_frame;

	multiframe_align = jesd->data_path_width;

	if (jesd->encoder == JESD204_ENCODER_64B66B &&
	    (octets_per_multiframe % 256) != 0) {
		dev_err(jesd->dev, "octets_per_frame * frames_per_multiframe must be a multiple of 256, got %u\n",
			octets_per_multiframe);
		return -EINVAL;
	}

	if (octets_per_multiframe % multiframe_align != 0) {
		dev_err(jesd->dev,
			"octets_per_frame * frames_per_multiframe must be a multiple of %u, got %u\n",
			multiframe_align, octets_per_multiframe);
		return -EINVAL;
	}

	val = (octets_per_multiframe - 1);
	val |= (config->octets_per_frame - 1) << 16;

	writel_relaxed(val, jesd->base + JESD204_RX_REG_LINK_CONF0);

	if (jesd->version >= ADI_AXI_PCORE_VER(1, 7, 'a')) {
		val = octets_per_multiframe / jesd->tpl_data_path_width - 1;
		writel_relaxed(val, jesd->base + JESD204_RX_REG_LINK_CONF4);
	}

	if (config->subclass == JESD204_SUBCLASS_0) {
		writel_relaxed(JESD204_RX_REG_SYSREF_CONF_SYSREF_DISABLE,
			       jesd->base + JESD204_RX_REG_SYSREF_CONF);
		writel_relaxed(JESD204_RX_LINK_CONF2_BUFFER_EARLY_RELEASE,
			       jesd->base + JESD204_RX_REG_LINK_CONF2);
	}

	if (config->sysref.lmfc_offset != JESD204_LMFC_OFFSET_UNINITIALIZED)
		writel_relaxed(config->sysref.lmfc_offset,
			jesd->base + JESD204_RX_REG_SYSREF_LMFC_OFFSET);

	return 0;
}

/*
 * FIXME: This is temporary. Configuration data is not part of the hardware
 * description and does not belong into the devicetree.
 */
static int axi_jesd204_rx_parse_dt_config(struct device_node *np,
	struct axi_jesd204_rx *jesd, struct jesd204_link *config)
{
	int ret;
	u32 val;

	ret = of_property_read_u32(np, "adi,octets-per-frame", &val);
	if (ret)
		return ret;
	config->octets_per_frame = val;

	ret = of_property_read_u32(np, "adi,frames-per-multiframe", &val);
	if (ret)
		return ret;
	config->frames_per_multiframe = val;

	config->high_density = of_property_read_bool(np, "adi,high-density");

	config->scrambling = true;
	config->num_lanes = jesd->num_lanes;
	config->jesd_version = JESD204_VERSION_B;
	config->subclass = JESD204_SUBCLASS_1;
	config->sysref.lmfc_offset = JESD204_LMFC_OFFSET_UNINITIALIZED;

	/* optional */
	ret = of_property_read_u32(np, "adi,subclass", &val);
	if (ret == 0)
		config->subclass = val;

	ret = of_property_read_u32(np, "adi,sysref-lmfc-offset", &val);
	if (ret == 0)
		config->sysref.lmfc_offset = val;

	return 0;
}

static bool axi_jesd_rx_regmap_rdwr(struct device *dev, unsigned int reg)
{
	struct axi_jesd204_rx *jesd = dev_get_drvdata(dev);
	unsigned int i;

	switch (reg) {
	case ADI_AXI_REG_VERSION:
	case ADI_AXI_REG_ID:
	case ADI_AXI_REG_SCRATCH:
	case JESD204_RX_REG_MAGIC:
	case JESD204_RX_REG_SYNTH_NUM_LANES:
	case JESD204_REG_SYNTH_DATA_PATH_WIDTH:
	case JESD204_RX_REG_SYNTH_ELASTIC_BUFFER_SIZE:
	case JESD204_RX_REG_IRQ_ENABLE:
	case JESD204_RX_REG_IRQ_PENDING:
	case JESD204_RX_REG_IRQ_SOURCE:
	case JESD204_RX_REG_LINK_DISABLE:
	case JESD204_RX_REG_LINK_STATE:
	case JESD204_RX_REG_LINK_CLK_RATIO:
	case JESD204_RX_REG_LINK_CONF0:
	case JESD204_RX_REG_LINK_CONF1:
	case JESD204_RX_REG_LINK_CONF2:
	case JESD204_RX_REG_LINK_CONF3:
	case JESD204_RX_REG_LINK_CONF4:
	case JESD204_RX_REG_LANES_ENABLE:
	case JESD204_RX_REG_LINK_STATUS:
	case JESD204_RX_REG_SYSREF_CONF:
	case JESD204_RX_REG_SYSREF_LMFC_OFFSET:
	case JESD204_RX_REG_SYSREF_STATUS:
		return true;
	default:
		break;
	}

	for (i = 0; i < jesd->num_lanes; i++) {
		if (reg == JESD204_RX_REG_LANE_STATUS(i) ||
			reg == JESD204_RX_REG_LANE_LATENCY(i) ||
			reg == JESD204_RX_REG_LANE_ERRORS(i) ||
			reg == JESD204_RX_REG_ILAS(i, 0) ||
			reg == JESD204_RX_REG_ILAS(i, 1) ||
			reg == JESD204_RX_REG_ILAS(i, 2) ||
			reg == JESD204_RX_REG_ILAS(i, 3))
			return true;
	}

	return false;
}

static const struct regmap_config axi_jesd_rx_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x800,
	.readable_reg = axi_jesd_rx_regmap_rdwr,
	.writeable_reg = axi_jesd_rx_regmap_rdwr,
};

/* Returns true if the link should be restarted */
static bool axi_jesd204_rx_check_lane_status(struct axi_jesd204_rx *jesd,
	unsigned int lane)
{
	unsigned int status;
	unsigned int errors;
	char error_str[sizeof(" (4294967295 errors)")];

	status = readl_relaxed(jesd->base + JESD204_RX_REG_LANE_STATUS(lane));

	if (jesd->encoder == JESD204_ENCODER_8B10B) {
		status &= 0x3;
		if (status != 0x0)
			return false;
	} else {
		status = JESD204_EMB_STATE_GET(status);
		if (status > JESD204_EMB_STATE_INIT &&
		    status <= JESD204_EMB_STATE_LOCK)
			return false;
	}

	if (jesd->version >= ADI_AXI_PCORE_VER(1, 2, 'a')) {
		errors = axi_jesd204_rx_get_lane_errors(jesd, lane);
		scnprintf(error_str, sizeof(error_str), " (%u errors)", errors);
	} else {
		error_str[0] = '\0';
	}

	dev_err(jesd->dev, "Lane %d desynced%s, restarting link\n", lane,
		error_str);

	return true;
}

static void axi_jesd204_rx_watchdog(struct work_struct *work)
{
	struct axi_jesd204_rx *jesd =
		container_of(work, struct axi_jesd204_rx, watchdog_work.work);
	unsigned int link_disabled;
	unsigned int link_status;
	bool restart = false;
	unsigned int i;

	link_disabled = readl_relaxed(jesd->base + JESD204_RX_REG_LINK_STATE);
	if (link_disabled)
		return;

	link_status = readl_relaxed(jesd->base + JESD204_RX_REG_LINK_STATUS);
	if (link_status == JESD204_LINK_STATUS_DATA) {
		for (i = 0; i < jesd->num_lanes; i++)
			restart |= axi_jesd204_rx_check_lane_status(jesd, i);

		if (restart) {
			writel_relaxed(0x1, jesd->base + JESD204_RX_REG_LINK_DISABLE);
			mdelay(100);
			writel_relaxed(0x0, jesd->base + JESD204_RX_REG_LINK_DISABLE);
			jesd204_sysref_async_force(jesd->jdev);
		}
	}

	schedule_delayed_work(&jesd->watchdog_work, HZ);
}

static int axi_jesd204_rx_lane_clk_enable(struct clk_hw *clk)
{
	struct axi_jesd204_rx *jesd =
		container_of(clk, struct axi_jesd204_rx, dummy_clk);

	writel_relaxed(0x3, jesd->base + JESD204_RX_REG_SYSREF_STATUS);
	writel_relaxed(0x0, jesd->base + JESD204_RX_REG_LINK_DISABLE);

	if (!jesd->irq)
		schedule_delayed_work(&jesd->watchdog_work, HZ);

	return 0;
}

static void axi_jesd204_rx_lane_clk_disable(struct clk_hw *clk)
{
	struct axi_jesd204_rx *jesd =
		container_of(clk, struct axi_jesd204_rx, dummy_clk);

	writel_relaxed(0x1, jesd->base + JESD204_RX_REG_LINK_DISABLE);
}

static const struct clk_ops axi_jesd204_rx_dummy_clk_ops = {
	.enable = axi_jesd204_rx_lane_clk_enable,
	.disable = axi_jesd204_rx_lane_clk_disable,
};

/* FIXME: This is terrible and needs to be replaced */
static int axi_jesd204_register_dummy_clk(struct axi_jesd204_rx *jesd,
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
	init.ops = &axi_jesd204_rx_dummy_clk_ops;
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

static int axi_jesd204_rx_pcore_check(struct axi_jesd204_rx *jesd)
{
	unsigned int magic, version;

	magic = readl_relaxed(jesd->base + JESD204_RX_REG_MAGIC);
	if (magic != JESD204_RX_MAGIC) {
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
	jesd->version = version;

	return 0;
}

static int axi_jesd204_rx_jesd204_link_pre_setup(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct axi_jesd204_rx *jesd = dev_get_drvdata(dev);
	unsigned long link_rate, lane_rate, device_rate;
	long rate;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

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

	device_rate = axi_jesd204_rx_calc_device_clk(jesd, link_rate);

	ret = clk_set_rate(jesd->device_clk, device_rate);
	if (ret) {
		dev_err(dev, "%s: Link%u set device clock rate %lu Hz failed (%d)\n",
			__func__, lnk->link_id, device_rate, ret);
		return ret;
	}

	if (!IS_ERR_OR_NULL(jesd->link_clk)) {
		ret = clk_set_rate(jesd->link_clk, link_rate);
		if (ret) {
			dev_err(dev, "%s: Link%u set link clock rate %lu Hz failed (%d)\n",
				__func__, lnk->link_id, link_rate, ret);
			return ret;
		}
	}

	rate = clk_round_rate(jesd->lane_clk, lane_rate);
	dev_dbg(dev, "%s: Link%u round lane rate %lu returned %ld\n",
		__func__, lnk->link_id, lane_rate, rate);

	if (rate != (long)lane_rate) {
		struct clk *parent;

		/*
		 * Check GT QPLL/CPLL reference clock and make
		 * it equal to the link/device rate
		 */
		parent = clk_get_parent(jesd->lane_clk);
		rate = clk_get_rate(parent);

		dev_dbg(dev, "%s: Link%u lane parent rate %ld link_rate %ld\n",
			__func__, lnk->link_id, rate, link_rate);

		if (rate != (long)link_rate) {
			rate = clk_round_rate(parent, link_rate);
			dev_dbg(dev, "%s: Link%u round lane parent rate %ld\n",
				__func__, lnk->link_id, rate);

			if (rate == (long)link_rate) {
				ret = clk_set_rate(parent, link_rate);
				if (!ret && !IS_ERR_OR_NULL(jesd->conv2_clk))
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


	return JESD204_STATE_CHANGE_DONE;
}

static int axi_jesd204_rx_jesd204_link_setup(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct axi_jesd204_rx *jesd = dev_get_drvdata(dev);
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		 __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	if (jesd->num_lanes != lnk->num_lanes)
		jesd204_notice(jdev,
				"Possible instantiation for multiple chips; HDL lanes %u, Link[%u] lanes %u\n",
				jesd->num_lanes, lnk->link_id, lnk->num_lanes);

	ret = axi_jesd204_rx_apply_config(jesd, lnk);
	if (ret) {
		dev_err(dev, "%s: Apply config Link%u failed (%d)\n",
			__func__, lnk->link_id, ret);
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int axi_jesd204_rx_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct axi_jesd204_rx *jesd = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	case JESD204_STATE_OP_REASON_UNINIT:
		if (!IS_ERR_OR_NULL(jesd->link_clk)) {
			if (__clk_is_enabled(jesd->link_clk))
				clk_disable_unprepare(jesd->link_clk);
		}
		if (!IS_ERR_OR_NULL(jesd->sysref_clk)) {
			if (__clk_is_enabled(jesd->sysref_clk))
				clk_disable_unprepare(jesd->sysref_clk);
		}
		if (__clk_is_enabled(jesd->device_clk))
			clk_disable_unprepare(jesd->device_clk);
		return JESD204_STATE_CHANGE_DONE;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	ret = clk_prepare_enable(jesd->device_clk);
	if (ret) {
		dev_err(dev, "%s: Link%u enable device clock failed (%d)\n",
			__func__, lnk->link_id, ret);
		return ret;
	}

	if (!IS_ERR_OR_NULL(jesd->sysref_clk)) {
		ret = clk_prepare_enable(jesd->sysref_clk);
		if (ret) {
			dev_err(dev, "%s: Link%u enable sysref clock failed (%d)\n",
				__func__, lnk->link_id, ret);
			return ret;
		}
	}

	if (!IS_ERR_OR_NULL(jesd->link_clk)) {
		ret = clk_prepare_enable(jesd->link_clk);
		if (ret) {
			dev_err(dev, "%s: Link%u enable link clock failed (%d)\n",
				__func__, lnk->link_id, ret);
			return ret;
		}
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int axi_jesd204_rx_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct axi_jesd204_rx *jesd = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	case JESD204_STATE_OP_REASON_UNINIT:
		if (!jesd->irq)
			cancel_delayed_work_sync(&jesd->watchdog_work);
		else
			disable_irq(jesd->irq);

		writel_relaxed(0x1, jesd->base + JESD204_RX_REG_LINK_DISABLE);

		if (__clk_is_enabled(jesd->lane_clk))
			clk_disable_unprepare(jesd->lane_clk);

		return JESD204_STATE_CHANGE_DONE;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	ret = clk_prepare_enable(jesd->lane_clk);
	if (ret) {
		dev_err(dev, "%s: Link%u enable lane clock failed (%d)\n",
			__func__, lnk->link_id, ret);
		return ret;
	}

	writel_relaxed(0x3, jesd->base + JESD204_RX_REG_SYSREF_STATUS);
	writel_relaxed(0x0, jesd->base + JESD204_RX_REG_LINK_DISABLE);

	if (!jesd->irq)
		schedule_delayed_work(&jesd->watchdog_work, HZ);
	else
		enable_irq(jesd->irq);

	return JESD204_STATE_CHANGE_DONE;
}

static int axi_jesd204_rx_jesd204_link_running(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct axi_jesd204_rx *jesd = dev_get_drvdata(dev);
	unsigned int link_status;
	int retry = 20;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason == JESD204_STATE_OP_REASON_INIT) {
		do {
			msleep(4);
			link_status = readl_relaxed(jesd->base + JESD204_RX_REG_LINK_STATUS) & 0x3;
		} while (link_status != JESD204_LINK_STATUS_DATA && retry--);

		if (link_status != JESD204_LINK_STATUS_DATA) {
			const char *_status = (jesd->encoder == JESD204_ENCODER_8B10B) ?
				axi_jesd204_rx_link_status_label[link_status] :
				axi_jesd204_rx_link_status_64b66b_l[link_status];

			dev_err(dev, "%s: Link%u status failed (%s)\n",
				__func__, lnk->link_id, _status);

			return JESD204_STATE_CHANGE_ERROR;
		}
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_axi_jesd204_rx_init = {
	.state_ops = {
		[JESD204_OP_LINK_PRE_SETUP] = {
			.per_link = axi_jesd204_rx_jesd204_link_pre_setup,
		},
		[JESD204_OP_LINK_SETUP] = {
			.per_link = axi_jesd204_rx_jesd204_link_setup,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = axi_jesd204_rx_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = axi_jesd204_rx_jesd204_link_enable,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = axi_jesd204_rx_jesd204_link_running,
		},
	},
};

static int axi_jesd204_init_non_framework(struct device *dev,
					  struct axi_jesd204_rx *jesd)
{
	struct jesd204_link config;
	int ret;

	ret = axi_jesd204_rx_parse_dt_config(dev->of_node, jesd, &config);
	if (ret)
		return ret;

	ret = axi_jesd204_rx_apply_config(jesd, &config);
	if (ret)
		return ret;

	return axi_jesd204_register_dummy_clk(jesd, dev);
}

static void axi_jesd204_rx_create_remove_devattrs(struct device *dev,
						  struct axi_jesd204_rx *jesd,
						  bool create)
{
	const struct device_attribute *dattr;
	unsigned int i, lanes;

	if (create) {
		device_create_file(dev, &dev_attr_status);
		device_create_file(dev, &dev_attr_encoder);
	} else {
		device_remove_file(dev, &dev_attr_status);
		device_remove_file(dev, &dev_attr_encoder);
	}

	if (jesd->num_lanes > 24) {
		dev_err(dev, "%s: Number of Lanes %u exceed max 24\n",
			__func__, jesd->num_lanes);
		lanes = 24;
	} else {
		lanes = jesd->num_lanes;
	}

	for (i = 0; i < lanes; i++) {
		dattr = jesd204_rx_lane_devattrs[i];
		if (create)
			device_create_file(dev, dattr);
		else
			device_remove_file(dev, dattr);
	}
}

static int axi_jesd204_rx_probe(struct platform_device *pdev)
{
	struct axi_jesd204_rx *jesd;
	struct jesd204_dev *jdev;
	struct resource *res;
	int irq;
	int ret;
	u32 tmp;

	if (!pdev->dev.of_node)
		return -ENODEV;

	jdev = devm_jesd204_dev_register(&pdev->dev, &jesd204_axi_jesd204_rx_init);
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

	devm_regmap_init_mmio(&pdev->dev, jesd->base, &axi_jesd_rx_regmap_config);

	ret = axi_jesd204_rx_pcore_check(jesd);
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
	jesd->conv2_clk = devm_clk_get_optional(&pdev->dev, "conv2");
	if (IS_ERR(jesd->conv2_clk))
		return PTR_ERR(jesd->conv2_clk);

	jesd->link_clk = devm_clk_get_optional(&pdev->dev, "link_clk");
	if (IS_ERR(jesd->link_clk))
		return PTR_ERR(jesd->link_clk);

	jesd->sysref_clk = devm_clk_get_optional(&pdev->dev, "sysref_clk");
	if (IS_ERR(jesd->sysref_clk))
		return PTR_ERR(jesd->sysref_clk);

	ret = clk_prepare_enable(jesd->axi_clk);
	if (ret)
		return ret;

	jesd->axi_clk_freq = clk_get_rate(jesd->axi_clk);
	if (!jesd->axi_clk_freq)
		jesd->axi_clk_freq = 100000000; /* 100 MHz */

	if (jesd->conv2_clk) {
		ret = clk_prepare_enable(jesd->conv2_clk);
		if (ret)
			goto err_axi_clk_disable;
	}

	jesd->num_lanes = readl_relaxed(jesd->base + JESD204_RX_REG_SYNTH_NUM_LANES);

	tmp = readl_relaxed(jesd->base + JESD204_REG_SYNTH_DATA_PATH_WIDTH);
	jesd->data_path_width = 1 << JESD204_SYNTH_DATA_PATH_WIDTH_GET(tmp);
	jesd->tpl_data_path_width = JESD204_TPL_DATA_PATH_WIDTH_GET(tmp);

	tmp = readl_relaxed(jesd->base + JESD204_REG_SYNTH_REG_1);
	jesd->encoder = JESD204_ENCODER_GET(tmp);

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

	writel_relaxed(0xff, jesd->base + JESD204_RX_REG_IRQ_PENDING);
	writel_relaxed(0x00, jesd->base + JESD204_RX_REG_IRQ_ENABLE);

	INIT_DELAYED_WORK(&jesd->watchdog_work, axi_jesd204_rx_watchdog);

	if (jesd->version >= ADI_AXI_PCORE_VER(1, 4, 'a')) {
		ret = devm_request_threaded_irq(&pdev->dev, irq, axi_jesd204_rx_irq,
						axi_jesd204_rx_irq_thread_fn,
						IRQF_ONESHOT, dev_name(&pdev->dev),
						jesd);
		if (ret)
			goto err_uninit_non_framework;

		disable_irq(irq);

		jesd->irq = irq;
		writel_relaxed(JESD204_RX_IRQ_FRAME_ALIGNMENT_ERROR |
				JESD204_RX_IRQ_UNEXP_LANE_STATE_ERROR,
				jesd->base + JESD204_RX_REG_IRQ_ENABLE);
	}

	platform_set_drvdata(pdev, jesd);

	axi_jesd204_rx_create_remove_devattrs(&pdev->dev, jesd, true);

	ret = jesd204_fsm_start(jesd->jdev, JESD204_LINKS_ALL);
	if (ret)
		goto err_remove_debugfs;

	dev_info(&pdev->dev, "AXI-JESD204-RX (%d.%.2d.%c) at 0x%08llX. Encoder %s, width %u/%u, lanes %d%s.",
		ADI_AXI_PCORE_VER_MAJOR(jesd->version),
		ADI_AXI_PCORE_VER_MINOR(jesd->version),
		ADI_AXI_PCORE_VER_PATCH(jesd->version),
		(unsigned long long)res->start,
		jesd204_encoder_str(jesd->encoder),
		jesd->data_path_width,
		jesd->tpl_data_path_width,
		jesd->num_lanes,
		jdev ? ", jesd204-fsm" : "");

	return 0;

err_remove_debugfs:
	axi_jesd204_rx_create_remove_devattrs(&pdev->dev, jesd, false);
err_uninit_non_framework:
	if (!jesd->jdev)
		of_clk_del_provider(pdev->dev.of_node);
err_conv2_clk_disable:
	clk_disable_unprepare(jesd->conv2_clk);
err_axi_clk_disable:
	clk_disable_unprepare(jesd->axi_clk);

	return ret;
}

static int axi_jesd204_rx_remove(struct platform_device *pdev)
{
	struct axi_jesd204_rx *jesd = platform_get_drvdata(pdev);

	jesd204_fsm_stop(jesd->jdev, JESD204_LINKS_ALL);

	axi_jesd204_rx_create_remove_devattrs(&pdev->dev, jesd, false);

	if (!jesd->jdev)
		of_clk_del_provider(pdev->dev.of_node);

	writel_relaxed(0xff, jesd->base + JESD204_RX_REG_IRQ_PENDING);
	writel_relaxed(0x00, jesd->base + JESD204_RX_REG_IRQ_ENABLE);

	writel_relaxed(0x1, jesd->base + JESD204_RX_REG_LINK_DISABLE);

	clk_disable_unprepare(jesd->conv2_clk);
	clk_disable_unprepare(jesd->axi_clk);

	return 0;
}

static const struct of_device_id axi_jesd204_rx_of_match[] = {
	{ .compatible = "adi,axi-jesd204-rx-1.0" },
	{ .compatible = "adi,axi-jesd204-rx-1.3" },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adxcvr_of_match);

static struct platform_driver axi_jesd204_rx_driver = {
	.probe = axi_jesd204_rx_probe,
	.remove = axi_jesd204_rx_remove,
	.driver = {
		.name = "axi-jesd204-rx",
		.of_match_table = axi_jesd204_rx_of_match,
	},
};
module_platform_driver(axi_jesd204_rx_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for the Analog Devices AXI-JESD204-RX peripheral");
MODULE_LICENSE("GPL v2");
