// SPDX-License-Identifier: GPL-2.0+
// Copyright 2020 NXP

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/pm_runtime.h>

static ssize_t read(struct device *dev, struct device_attribute *attr, char *buf, unsigned int reg)
{
	struct fsl_xcvr *xcvr = dev_get_drvdata(dev);
	unsigned int val = 0;

	regmap_read(xcvr->regmap, reg, &val);

	return sprintf(buf, "%u\n", val);
}

#define XRDC_RO_ATTR(_name, _reg) \
static ssize_t _name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
{	/* read bitcounter */ \
	return read(dev, attr, buf, _reg); \
} \
static DEVICE_ATTR_RO(_name);

XRDC_RO_ATTR(rx_bitcnt, FSL_XCVR_RX_DPTH_BCR)
XRDC_RO_ATTR(tx_bitcnt, FSL_XCVR_TX_DPTH_BCR)
XRDC_RO_ATTR(rx_timestamp, FSL_XCVR_RX_DPTH_TSCR)
XRDC_RO_ATTR(tx_timestamp, FSL_XCVR_TX_DPTH_TSCR)
XRDC_RO_ATTR(rx_bitcnt_latched_timestamp, FSL_XCVR_RX_DPTH_BCRR)
XRDC_RO_ATTR(tx_bitcnt_latched_timestamp, FSL_XCVR_TX_DPTH_BCRR)

/* ============ */
static ssize_t show(struct device *dev, struct device_attribute *attr, char *buf, u32 reg, u32 bit)
{
	struct fsl_xcvr *xcvr = dev_get_drvdata(dev);
	unsigned int val = 0;

	regmap_read(xcvr->regmap, reg, &val);

	return sprintf(buf, "%u\n", (val & BIT(bit)) ? 1 : 0);
}

static ssize_t store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n,
		     u32 reg, u32 bit)
{
	struct fsl_xcvr *xcvr = dev_get_drvdata(dev);
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	regmap_update_bits(xcvr->regmap, reg, BIT(bit), val ? BIT(bit) : 0);

	return n;
}

#define XRDC_RW_ATTR(_name, _reg, _bit) \
static ssize_t _name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	return show(dev, attr, buf, _reg, _bit); \
} \
\
static ssize_t _name##_store(struct device *dev, struct device_attribute *attr, const char *buf, \
			     size_t n) \
{ \
	return store(dev, attr, buf, n, _reg, _bit); \
} \
static DEVICE_ATTR_RW(_name);

XRDC_RW_ATTR(rx_bitcnt_reset, FSL_XCVR_RX_DPTH_CNTR_CTRL, 8)
XRDC_RW_ATTR(tx_bitcnt_reset, FSL_XCVR_TX_DPTH_CNTR_CTRL, 8)
XRDC_RW_ATTR(rx_timestamp_enable, FSL_XCVR_RX_DPTH_CNTR_CTRL, 0)
XRDC_RW_ATTR(tx_timestamp_enable, FSL_XCVR_TX_DPTH_CNTR_CTRL, 0)
XRDC_RW_ATTR(rx_timestamp_increment, FSL_XCVR_RX_DPTH_CNTR_CTRL, 1)
XRDC_RW_ATTR(tx_timestamp_increment, FSL_XCVR_TX_DPTH_CNTR_CTRL, 1)
XRDC_RW_ATTR(rx_timestamp_reset, FSL_XCVR_RX_DPTH_CNTR_CTRL, 9)
XRDC_RW_ATTR(tx_timestamp_reset, FSL_XCVR_TX_DPTH_CNTR_CTRL, 9)

static struct attribute *fsl_xcvr_attrs[] = {
	&dev_attr_tx_bitcnt.attr,
	&dev_attr_rx_bitcnt.attr,
	&dev_attr_tx_timestamp.attr,
	&dev_attr_rx_timestamp.attr,
	&dev_attr_tx_bitcnt_latched_timestamp.attr,
	&dev_attr_rx_bitcnt_latched_timestamp.attr,
	&dev_attr_tx_timestamp_enable.attr,
	&dev_attr_rx_timestamp_enable.attr,
	&dev_attr_tx_timestamp_increment.attr,
	&dev_attr_rx_timestamp_increment.attr,
	&dev_attr_tx_bitcnt_reset.attr,
	&dev_attr_rx_bitcnt_reset.attr,
	&dev_attr_tx_timestamp_reset.attr,
	&dev_attr_rx_timestamp_reset.attr,
	NULL,
};

static struct attribute_group fsl_xcvr_attr_group = {
	.name  = "counters",
	.attrs = fsl_xcvr_attrs,
};

static struct attribute_group *fsl_xcvr_get_attr_grp(void)
{
	return &fsl_xcvr_attr_group;
}
