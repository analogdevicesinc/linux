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

#define PCORE_VERSION_MAJOR(version)		(version >> 16)
#define PCORE_VERSION_MINOR(version)		((version >> 8) & 0xff)
#define PCORE_VERSION_PATCH(version)		(version & 0xff)

#define JESD204_TX_REG_VERSION			0x00
#define JESD204_TX_REG_ID			0x04
#define JESD204_TX_REG_SCRATCH			0x08
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

struct jesd204_tx_config {
	uint8_t device_id;
	uint8_t bank_id;
	uint8_t lane_id;
	uint8_t lanes_per_device;
	uint8_t octets_per_frame;
	uint8_t frames_per_multiframe;
	uint8_t converters_per_device;
	uint8_t resolution;
	uint8_t bits_per_sample;
	uint8_t samples_per_frame;
	uint8_t jesd_version;
	uint8_t subclass_version;
	uint8_t control_bits_per_sample;
	bool enable_scrambling;
	bool high_density;
};

struct axi_jesd204_tx {
	void __iomem *base;
	struct device *dev;

	struct clk *axi_clk;
	struct clk *device_clk;

	int irq;

	unsigned int num_lanes;
	unsigned int data_path_width;

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
	unsigned int clock_ratio;
	unsigned int clock_rate;
	unsigned int link_rate;
	unsigned int sysref_config;
	int ret;

	link_disabled = readl_relaxed(jesd->base + JESD204_TX_REG_LINK_STATE);
	link_status = readl_relaxed(jesd->base + JESD204_TX_REG_LINK_STATUS);
	sysref_status = readl_relaxed(jesd->base + JESD204_TX_REG_SYSREF_STATUS);
	clock_ratio = readl_relaxed(jesd->base + JESD204_TX_REG_LINK_CLK_RATIO);
	sysref_config = readl_relaxed(jesd->base + JESD204_TX_REG_SYSREF_CONF);

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
		clock_rate = clk_get_rate(jesd->lane_clk);
		link_rate = DIV_ROUND_CLOSEST(clock_rate, 40);
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Lane rate: %d.%.3d MHz\n"
			"Lane rate / 40: %d.%.3d MHz\n",
			clock_rate / 1000, clock_rate % 1000,
			link_rate / 1000, link_rate % 1000);

		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"SYNC~: %s\n"
			"Link status: %s\n"
			"SYSREF captured: %s\n"
			"SYSREF alignment error: %s\n",
			(link_status & 0x10) ? "deasserted" : "asserted",
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
	const struct jesd204_tx_config *config)
{
	unsigned int chksum;

	chksum = config->device_id;
	chksum += config->bank_id;
	chksum += config->lane_id;
	chksum += config->lanes_per_device - 1;
	chksum += config->enable_scrambling;
	chksum += config->octets_per_frame - 1;
	chksum += config->frames_per_multiframe - 1;
	chksum += config->converters_per_device - 1;
	chksum += config->control_bits_per_sample;
	chksum += config->resolution - 1;
	chksum += config->bits_per_sample - 1;
	chksum += config->subclass_version;
	chksum += config->samples_per_frame  - 1;
	chksum += config->jesd_version;
	chksum += config->high_density;

	return chksum & 0xff;
}

static void axi_jesd204_tx_set_lane_ilas(struct axi_jesd204_tx *jesd,
	struct jesd204_tx_config *config, unsigned int lane)
{
	unsigned int i;
	unsigned int val;

	config->lane_id = lane;

	for (i = 0; i < 4; i++) {
		switch (i) {
		case 0:
			val = config->device_id << 8;
			val |= config->bank_id << 24;
			break;
		case 1:
			val = config->lane_id;
			val |= (config->lanes_per_device - 1) << 8;
			val |= config->enable_scrambling << 15;
			val |= (config->octets_per_frame - 1) << 16;
			val |= (config->frames_per_multiframe - 1) << 24;
			break;
		case 2:
			val = (config->converters_per_device - 1);
			val |= (config->resolution - 1) << 8;
			val |= config->control_bits_per_sample << 14;
			val |= (config->bits_per_sample - 1) << 16;
			val |= config->subclass_version << 21;
			val |= (config->samples_per_frame - 1) << 24;
			val |= config->jesd_version << 29;
			break;
		case 3:
			val = config->high_density << 7;
			val |= axi_jesd204_tx_calc_ilas_chksum(config) << 24;
			break;
		}

		writel_relaxed(val, jesd->base + JESD204_TX_REG_ILAS(lane, i));
	}
}

static int axi_jesd204_tx_apply_config(struct axi_jesd204_tx *jesd,
	struct jesd204_tx_config *config)
{
	unsigned int octets_per_multiframe;
	unsigned int multiframe_align;
	unsigned int val;
	unsigned int lane;

	octets_per_multiframe = config->frames_per_multiframe *
		config->octets_per_frame;

	multiframe_align = 1 << jesd->data_path_width;

	if (octets_per_multiframe % multiframe_align != 0) {
		dev_err(jesd->dev,
			"octets_per_frame * frames_per_multiframe must be a multiple of  %d\n",
			multiframe_align);
		return -EINVAL;
	}

	val = (octets_per_multiframe - 1);
	val |= (config->octets_per_frame - 1) << 16;

	if (config->subclass_version == 0)
		writel_relaxed(JESD204_TX_REG_SYSREF_CONF_SYSREF_DISABLE,
			       jesd->base + JESD204_TX_REG_SYSREF_CONF);

	writel_relaxed(val, jesd->base + JESD204_TX_REG_CONF0);

	for (lane = 0; lane < jesd->num_lanes; lane++)
		axi_jesd204_tx_set_lane_ilas(jesd, config, lane);

	return 0;
}

/*
 * FIXME: This is temporary. Configuration data is not part of the hardware
 * description and does not belong into the devicetree.
 */
static int axi_jesd204_tx_parse_dt_config(struct device_node *np,
	struct axi_jesd204_tx *jesd, struct jesd204_tx_config *config)
{
	int ret;
	u32 val;

	config->device_id = 0;
	config->bank_id = 0;
	config->enable_scrambling = true;
	config->lanes_per_device = jesd->num_lanes;
	config->jesd_version = 1;
	config->subclass_version = 1;
	config->control_bits_per_sample = 0;
	config->samples_per_frame = 1;

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
	config->resolution = val;

	ret = of_property_read_u32(np, "adi,bits-per-sample", &val);
	if (ret)
		return ret;
	config->bits_per_sample = val;

	ret = of_property_read_u32(np, "adi,converters-per-device", &val);
	if (ret)
		return ret;
	config->converters_per_device = val;

	/* optional */
	ret = of_property_read_u32(np, "adi,control-bits-per-sample", &val);
	if (ret == 0)
		config->control_bits_per_sample = val;

	ret = of_property_read_u32(np, "adi,subclass", &val);
	if (ret == 0)
		config->subclass_version = val;

	return 0;
}

static bool axi_jesd_tx_regmap_rdwr(struct device *dev, unsigned int reg)
{
	struct axi_jesd204_tx *jesd = dev_get_drvdata(dev);
	unsigned int i;

	switch (reg) {
	case JESD204_TX_REG_VERSION:
	case JESD204_TX_REG_ID:
	case JESD204_TX_REG_SCRATCH:
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
	struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
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

	dummy_clk = devm_clk_register(&pdev->dev, &jesd->dummy_clk);
	if (IS_ERR(dummy_clk))
		return PTR_ERR(dummy_clk);

	of_clk_add_provider(np, of_clk_src_simple_get, dummy_clk);

	return 0;
}

static int axi_jesd204_tx_probe(struct platform_device *pdev)
{
	struct jesd204_tx_config config;
	struct axi_jesd204_tx *jesd;
	unsigned int version, magic;
	struct resource *res;
	int irq;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	if (irq == 0)
		return -ENXIO;

	jesd = devm_kzalloc(&pdev->dev, sizeof(*jesd), GFP_KERNEL);
	if (!jesd)
		return -ENOMEM;

	jesd->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	jesd->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jesd->base))
		return PTR_ERR(jesd->base);

	devm_regmap_init_mmio(&pdev->dev, jesd->base, &axi_jesd_tx_regmap_config);

	jesd->axi_clk = devm_clk_get(&pdev->dev, "s_axi_aclk");
	if (IS_ERR(jesd->axi_clk))
		return PTR_ERR(jesd->axi_clk);

	jesd->device_clk = devm_clk_get(&pdev->dev, "device_clk");
	if (IS_ERR(jesd->device_clk))
		return PTR_ERR(jesd->device_clk);

	jesd->lane_clk = devm_clk_get(&pdev->dev, "lane_clk");
	if (IS_ERR(jesd->lane_clk))
		return PTR_ERR(jesd->lane_clk);

	ret = clk_prepare_enable(jesd->axi_clk);
	if (ret)
		return ret;

	magic = readl_relaxed(jesd->base + JESD204_TX_REG_MAGIC);
	if (magic != JESD204_TX_MAGIC) {
		dev_err(&pdev->dev, "Unexpected peripheral identifier %.08x\n",
			magic);
		ret = -ENODEV;
		goto err_axi_clk_disable;
	}

	version = readl_relaxed(jesd->base + JESD204_TX_REG_VERSION);
	if (PCORE_VERSION_MAJOR(version) != 1) {
		dev_err(&pdev->dev, "Unsupported peripheral version %u.%u.%c\n",
			PCORE_VERSION_MAJOR(version),
			PCORE_VERSION_MINOR(version),
			PCORE_VERSION_PATCH(version));
		ret = -ENODEV;
		goto err_axi_clk_disable;
	}

	jesd->num_lanes = readl_relaxed(jesd->base + JESD204_TX_REG_CONF_NUM_LANES);
	jesd->data_path_width = readl_relaxed(jesd->base + JESD204_TX_REG_CONF_DATA_PATH_WIDTH);

	ret = axi_jesd204_tx_parse_dt_config(pdev->dev.of_node, jesd, &config);
	if (ret)
		goto err_axi_clk_disable;

	ret = axi_jesd204_tx_apply_config(jesd, &config);
	if (ret)
		goto err_axi_clk_disable;

	writel_relaxed(0xff, jesd->base + JESD204_TX_REG_IRQ_PENDING);
	writel_relaxed(0x00, jesd->base + JESD204_TX_REG_IRQ_ENABLE);

	ret = request_irq(irq, axi_jesd204_tx_irq, 0, dev_name(&pdev->dev),
		jesd);
	if (ret)
		goto err_axi_clk_disable;

/* FIXME: Enabling the clock here and keeping it enabled will prevent
 * reconfiguration of the the clock when the lane rate changes. We need to find
 * a mechanism to disable the clock before link reconfiguration. For the time
 * being don't enable it and hope that some other driver does.
 *	ret = clk_prepare_enable(jesd->device_clk);
 *	if (ret)
 *	    goto err_free_irq;
 */

	ret = axi_jesd204_register_dummy_clk(jesd, pdev);
	if (ret)
		goto err_disable_device_clk;

	device_create_file(&pdev->dev, &dev_attr_status);

	platform_set_drvdata(pdev, jesd);

	return 0;
err_disable_device_clk:
/*
	clk_disable_unprepare(jesd->device_clk);
err_free_irq:
*/
	free_irq(irq, jesd);
err_axi_clk_disable:
	clk_disable_unprepare(jesd->axi_clk);

	return ret;
}

static int axi_jesd204_tx_remove(struct platform_device *pdev)
{
	struct axi_jesd204_tx *jesd = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

	of_clk_del_provider(pdev->dev.of_node);

	free_irq(irq, jesd);

	writel_relaxed(0xff, jesd->base + JESD204_TX_REG_IRQ_PENDING);
	writel_relaxed(0x00, jesd->base + JESD204_TX_REG_IRQ_ENABLE);

	writel_relaxed(0x1, jesd->base + JESD204_TX_REG_LINK_DISABLE);

/*	clk_disable_unprepare(jesd->device_clk); */
	clk_disable_unprepare(jesd->axi_clk);

	return 0;
}

static const struct of_device_id axi_jesd204_tx_of_match[] = {
	{ .compatible = "adi,axi-jesd204-tx-1.0" },
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
