/*
 * TDD HDL CORE driver
 *
 * Copyright 2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/fpga/adi-axi-common.h>

/* Transceiver TDD Control (axi_ad*) */

#define ADI_REG_TDD_CONTROL_0		0x0040
#define ADI_TDD_DMA_GATE_TX_EN		(1 << 5)
#define ADI_TDD_DMA_GATE_RX_EN		(1 << 4)
#define ADI_TDD_TXONLY_EN			(1 << 3)
#define ADI_TDD_RXONLY_EN			(1 << 2)
#define ADI_TDD_SECONDARY			(1 << 1)
#define ADI_TDD_ENABLE				(1 << 0)

#define ADI_REG_TDD_CONTROL_1		0x0044
#define ADI_TDD_BURST_COUNT(x)		(((x) & 0xFF) << 0)
#define ADI_TO_TDD_BURST_COUNT(x)	(((x) >> 0) & 0xFF)

#define ADI_REG_TDD_COUNTER_2		0x0048
#define ADI_TDD_COUNTER_INIT(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_COUNTER_INIT(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_FRAME_LENGTH	0x004c
#define ADI_TDD_FRAME_LENGTH(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_FRAME_LENGTH(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_SYNC_TERM_TYPE	0x0050
#define ADI_TDD_SYNC_PULSE_ENABLE	(1 << 0)

#define ADI_REG_TDD_VCO_RX_ON_1		0x0080
#define ADI_TDD_VCO_RX_ON_1(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_VCO_RX_ON_1(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_VCO_RX_OFF_1	0x0084
#define ADI_TDD_VCO_RX_OFF_1(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_VCO_RX_OFF_1(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_VCO_TX_ON_1		0x0088
#define ADI_TDD_VCO_TX_ON_1(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_VCO_TX_ON_1(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_VCO_TX_OFF_1	0x008C
#define ADI_TDD_VCO_TX_OFF_1(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_VCO_TX_OFF_1(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_RX_ON_1			0x0090
#define ADI_TDD_RX_ON_1(x)			(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_RX_ON_1(x)		(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_RX_OFF_1		0x0094
#define ADI_TDD_RX_OFF_1(x)			(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_RX_OFF_1(x)		(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_TX_ON_1			0x0098
#define ADI_TDD_TX_ON_1(x)			(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_TX_ON_1(x)		(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_TX_OFF_1		0x009C
#define ADI_TDD_TX_OFF_1(x)			(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_TX_OFF_1(x)		(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_TX_DP_ON_1		0x00A0
#define ADI_TDD_TX_DP_ON_1(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_TX_DP_ON_1(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_TX_DP_OFF_1		0x00A4
#define ADI_TDD_TX_DP_OFF_1(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_TX_DP_OFF_1(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_RX_DP_ON_1		0x00A8
#define ADI_TDD_RX_DP_ON_1(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_RX_DP_ON_1(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_RX_DP_OFF_1		0x00AC
#define ADI_TDD_RX_DP_OFF_1(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_RX_DP_OFF_1(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_VCO_RX_ON_2		0x00C0
#define ADI_TDD_VCO_RX_ON_2(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_VCO_RX_ON_2(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_VCO_RX_OFF_2	0x00C4
#define ADI_TDD_VCO_RX_OFF_2(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_VCO_RX_OFF_2(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_VCO_TX_ON_2		0x00C8
#define ADI_TDD_VCO_TX_ON_2(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_VCO_TX_ON_2(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_VCO_TX_OFF_2	0x00CC
#define ADI_TDD_VCO_TX_OFF_2(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_VCO_TX_OFF_2(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_RX_ON_2			0x00D0
#define ADI_TDD_RX_ON_2(x)			(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_RX_ON_2(x)		(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_RX_OFF_2		0x00D4
#define ADI_TDD_RX_OFF_2(x)			(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_RX_OFF_2(x)		(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_TX_ON_2			0x00D8
#define ADI_TDD_TX_ON_2(x)			(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_TX_ON_2(x)		(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_TX_OFF_2		0x00DC
#define ADI_TDD_TX_OFF_2(x)			(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_TX_OFF_2(x)		(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_TX_DP_ON_2		0x00E0
#define ADI_TDD_TX_DP_ON_2(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_TX_DP_ON_2(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_TX_DP_OFF_2		0x00E4
#define ADI_TDD_TX_DP_OFF_2(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_TX_DP_OFF_2(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_RX_DP_ON_2		0x00E8
#define ADI_TDD_RX_DP_ON_2(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_RX_DP_ON_2(x)	(((x) >> 0) & 0xFFFFFF)

#define ADI_REG_TDD_RX_DP_OFF_2		0x00EC
#define ADI_TDD_RX_DP_OFF_2(x)		(((x) & 0xFFFFFF) << 0)
#define ADI_TO_TDD_RX_DP_OFF_2(x)	(((x) >> 0) & 0xFFFFFF)

#define MAX_NUM_PROFILES			7

struct cf_axi_tdd_state {
	struct iio_info		iio_info;
	void __iomem		*regs;
	unsigned			version;
	unsigned			enable;
	unsigned			mode;
	unsigned			dma_mode;
	unsigned			profile;
	u32					config[MAX_NUM_PROFILES][27];

};

enum {
	CF_AXI_TDD_ENABLE,
	CF_AXI_TDD_ENABLE_MODE,
	CF_AXI_TDD_DMA_GATEING_MODE,
	CF_AXI_TDD_BURST_COUNT,
	CF_AXI_TDD_PROFILE_CONFIG,
};

static inline void tdd_write(struct cf_axi_tdd_state *st,
			     unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int tdd_read(struct cf_axi_tdd_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static inline void tdd_write_config(struct cf_axi_tdd_state *st, unsigned profile)
{
	int i;

	tdd_write(st, ADI_REG_TDD_COUNTER_2, st->config[profile][0]);
	tdd_write(st, ADI_REG_TDD_FRAME_LENGTH, st->config[profile][1]);
	tdd_write(st, ADI_REG_TDD_SYNC_TERM_TYPE, st->config[profile][2]);

	for (i = 0; i < 24; i++)
		tdd_write(st, ADI_REG_TDD_VCO_RX_ON_1 + i * 4, st->config[profile][3 + i]);

}

static inline void tdd_read_config(struct cf_axi_tdd_state *st, unsigned profile)
{
	int i;

	st->config[profile][0] = tdd_read(st, ADI_REG_TDD_COUNTER_2);
	st->config[profile][1] = tdd_read(st, ADI_REG_TDD_FRAME_LENGTH);
	st->config[profile][2] = tdd_read(st, ADI_REG_TDD_SYNC_TERM_TYPE);

	for (i = 0; i < 24; i++)
		st->config[profile][3 + i] = tdd_read(st, ADI_REG_TDD_VCO_RX_ON_1 + i * 4);

}

static ssize_t cf_axi_tdd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);
	bool state;
	unsigned long readin;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case CF_AXI_TDD_ENABLE:
		ret = strtobool(buf, &state);
		if (ret < 0)
			break;
		st->enable = (state ? ADI_TDD_ENABLE : 0);
		tdd_write(st, ADI_REG_TDD_CONTROL_0,
				  st->dma_mode | st->mode | st->enable);
		break;
	case CF_AXI_TDD_ENABLE_MODE:
		if (sysfs_streq(buf, "rx_tx"))
			st->mode = 0;
		else if (sysfs_streq(buf, "rx_only"))
			st->mode = ADI_TDD_RXONLY_EN;
		else if (sysfs_streq(buf, "tx_only"))
			st->mode = ADI_TDD_TXONLY_EN;
		tdd_write(st, ADI_REG_TDD_CONTROL_0,
				  st->dma_mode | st->mode | st->enable);
		break;
	case CF_AXI_TDD_DMA_GATEING_MODE:
		if (sysfs_streq(buf, "rx_tx"))
			st->dma_mode = ADI_TDD_DMA_GATE_RX_EN | ADI_TDD_DMA_GATE_TX_EN;
		else if (sysfs_streq(buf, "rx_only"))
			st->dma_mode = ADI_TDD_DMA_GATE_RX_EN;
		else if (sysfs_streq(buf, "tx_only"))
			st->dma_mode = ADI_TDD_DMA_GATE_TX_EN;
		else if (sysfs_streq(buf, "none"))
			st->dma_mode = 0;
		tdd_write(st, ADI_REG_TDD_CONTROL_0,
				  st->dma_mode | st->mode | st->enable);
		break;
	case CF_AXI_TDD_BURST_COUNT:
		ret = kstrtoul(buf, 0, &readin);
		if (ret)
			break;
		tdd_write(st, ADI_REG_TDD_CONTROL_1,
			  ADI_TDD_BURST_COUNT(readin));
		break;
	case CF_AXI_TDD_PROFILE_CONFIG:
		ret = kstrtoul(buf, 0, &readin);
		if (ret)
			break;
		if (readin > MAX_NUM_PROFILES) {
			ret = -EINVAL;
			break;
		}
		if (st->profile != readin) {
			tdd_read_config(st, st->profile);
			tdd_write_config(st, readin);
		}
		st->profile = readin;
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t cf_axi_tdd_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case CF_AXI_TDD_ENABLE:
		ret = sprintf(buf, "%d\n", st->enable);
		break;
	case CF_AXI_TDD_ENABLE_MODE:
		ret = sprintf(buf, "%s\n", (st->mode == ADI_TDD_RXONLY_EN) ?
			"rx_only" : (st->mode == ADI_TDD_TXONLY_EN) ?
			"tx_only" : "rx_tx");
		break;
	case CF_AXI_TDD_DMA_GATEING_MODE:
		switch(st->dma_mode) {
		case ADI_TDD_DMA_GATE_RX_EN | ADI_TDD_DMA_GATE_TX_EN:
			ret = sprintf(buf, "%s\n", "rx_tx");
			break;
		case ADI_TDD_DMA_GATE_RX_EN:
			ret = sprintf(buf, "%s\n", "rx_only");
			break;
		case ADI_TDD_DMA_GATE_TX_EN:
			ret = sprintf(buf, "%s\n", "tx_only");
			break;
		default:
			ret = sprintf(buf, "%s\n", "none");
			break;
		}
		break;
	case CF_AXI_TDD_BURST_COUNT:
		ret = sprintf(buf, "%d\n", tdd_read(st, ADI_REG_TDD_CONTROL_1));
		break;
	case CF_AXI_TDD_PROFILE_CONFIG:
		ret = sprintf(buf, "%d\n", st->profile);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
			cf_axi_tdd_show,
			cf_axi_tdd_store,
			CF_AXI_TDD_ENABLE);

static IIO_DEVICE_ATTR(enable_mode, S_IRUGO | S_IWUSR,
			cf_axi_tdd_show,
			cf_axi_tdd_store,
			CF_AXI_TDD_ENABLE_MODE);

static IIO_CONST_ATTR(enable_mode_available, "rx_tx rx_only tx_only");

static IIO_DEVICE_ATTR(dma_gateing_mode, S_IRUGO | S_IWUSR,
			cf_axi_tdd_show,
			cf_axi_tdd_store,
			CF_AXI_TDD_DMA_GATEING_MODE);

static IIO_CONST_ATTR(dma_gateing_mode_available, "none rx_only tx_only rx_tx");

static IIO_DEVICE_ATTR(burst_count, S_IRUGO | S_IWUSR,
			cf_axi_tdd_show,
			cf_axi_tdd_store,
			CF_AXI_TDD_BURST_COUNT);

static IIO_DEVICE_ATTR(profile_config, S_IRUGO | S_IWUSR,
			cf_axi_tdd_show,
			cf_axi_tdd_store,
			CF_AXI_TDD_PROFILE_CONFIG);

static struct attribute *cf_axi_tdd_attributes[] = {
	&iio_dev_attr_enable.dev_attr.attr,
	&iio_dev_attr_enable_mode.dev_attr.attr,
	&iio_const_attr_enable_mode_available.dev_attr.attr,
	&iio_dev_attr_dma_gateing_mode.dev_attr.attr,
	&iio_const_attr_dma_gateing_mode_available.dev_attr.attr,
	&iio_dev_attr_burst_count.dev_attr.attr,
	&iio_dev_attr_profile_config.dev_attr.attr,
	NULL,
};

static const struct attribute_group cf_axi_tdd_attribute_group = {
	.attrs = cf_axi_tdd_attributes,
};

static int cf_axi_tdd_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		tdd_write(st, reg & 0xFFFF, writeval);
		ret = 0;
	} else {
		ret = tdd_read(st, reg & 0xFFFF);
		*readval = ret;
		ret = 0;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info cf_axi_tdd_info = {
	.debugfs_reg_access = &cf_axi_tdd_reg_access,
	.attrs = &cf_axi_tdd_attribute_group,
};

/* Match table for of_platform binding */
static const struct of_device_id cf_axi_tdd_of_match[] = {
	{ .compatible = "adi,axi-tdd-1.00", .data = 0},
	{ },
};
MODULE_DEVICE_TABLE(of, cf_axi_tdd_of_match);

static int cf_axi_tdd_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	unsigned int expected_version;
	struct cf_axi_tdd_state *st;
	struct iio_dev *indio_dev;
	struct resource *res;
	int ret, i;
	char buf[32];

	dev_err(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	for (i = 0; i < MAX_NUM_PROFILES; i++) {
		snprintf(buf, sizeof(buf), "adi,profile-config%d", i);
		of_property_read_u32_array(np, buf, st->config[i], 15);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!st->regs)
		return -ENOMEM;

	st->version = tdd_read(st, ADI_AXI_REG_VERSION);
	expected_version = ADI_AXI_PCORE_VER(1, 0, 'a');

	if (ADI_AXI_PCORE_VER_MAJOR(st->version) !=
		ADI_AXI_PCORE_VER_MAJOR(expected_version)) {
		dev_err(&pdev->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(expected_version),
			ADI_AXI_PCORE_VER_MINOR(expected_version),
			ADI_AXI_PCORE_VER_PATCH(expected_version),
			ADI_AXI_PCORE_VER_MAJOR(st->version),
			ADI_AXI_PCORE_VER_MINOR(st->version),
			ADI_AXI_PCORE_VER_PATCH(st->version));
		return -ENODEV;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->info = &cf_axi_tdd_info;

	tdd_write_config(st, 0);

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "Analog Devices CF_AXI_TDD %s (%d.%.2d.%c) at 0x%08llX mapped"
		" to 0x%p\n",
		tdd_read(st, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER",
		ADI_AXI_PCORE_VER_MAJOR(st->version),
		ADI_AXI_PCORE_VER_MINOR(st->version),
		ADI_AXI_PCORE_VER_PATCH(st->version),
		(unsigned long long)res->start, st->regs);

	platform_set_drvdata(pdev, indio_dev);

	return 0;
}

static int cf_axi_tdd_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);

	return 0;
}

static struct platform_driver cf_axi_tdd_driver = {
	.driver = {
		.name = "cf_axi_tdd",
		.owner = THIS_MODULE,
		.of_match_table = cf_axi_tdd_of_match,
	},
	.probe		= cf_axi_tdd_probe,
	.remove		= cf_axi_tdd_remove,
};
module_platform_driver(cf_axi_tdd_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices TDD HDL CORE driver");
MODULE_LICENSE("GPL v2");
