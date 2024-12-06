// SPDX-License-Identifier: GPL-2.0+
// Copyright 2020 NXP

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/syscon/imx7-iomuxc-gpr.h>

#include "fsl_sai.h"

static ssize_t tx_bitcnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	/* read bitcounter */
	regmap_read(sai->regmap, FSL_SAI_TBCTN, &val);

	return sprintf(buf, "%u\n", val);
}
static DEVICE_ATTR_RO(tx_bitcnt);

static ssize_t tx_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	/* read timestamp */
	regmap_read(sai->regmap, FSL_SAI_TTCTN, &val);

	return sprintf(buf, "%u\n", val);
}
static DEVICE_ATTR_RO(tx_timestamp);

static ssize_t tx_bitcnt_latched_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	/* read timestamp */
	regmap_read(sai->regmap, FSL_SAI_TTCAP, &val);

	return sprintf(buf, "%u\n", val);
}
static DEVICE_ATTR_RO(tx_bitcnt_latched_timestamp);

static ssize_t
tx_timestamp_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != 0)
		val = FSL_SAI_xTCTL_TSEN;
	regmap_update_bits(sai->regmap, FSL_SAI_TTCTL, FSL_SAI_xTCTL_TSEN, val);
	return n;
}
static DEVICE_ATTR_WO(tx_timestamp_enable);

static ssize_t
tx_timestamp_increment_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != 0)
		val = FSL_SAI_xTCTL_TSINC;
	regmap_update_bits(sai->regmap, FSL_SAI_TTCTL, FSL_SAI_xTCTL_TSINC, val);
	return n;
}
static DEVICE_ATTR_WO(tx_timestamp_increment);

static ssize_t
tx_bitcnt_reset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != 0)
		val = FSL_SAI_xTCTL_RBC;
	regmap_update_bits(sai->regmap, FSL_SAI_TTCTL, FSL_SAI_xTCTL_RBC, val);
	return n;
}
static DEVICE_ATTR_WO(tx_bitcnt_reset);

static ssize_t
tx_timestamp_reset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != 0)
		val = FSL_SAI_xTCTL_RTSC;
	regmap_update_bits(sai->regmap, FSL_SAI_TTCTL, FSL_SAI_xTCTL_RTSC, val);
	return n;
}
static DEVICE_ATTR_WO(tx_timestamp_reset);


static ssize_t rx_bitcnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	/* read bitcounter */
	regmap_read(sai->regmap, FSL_SAI_RBCTN, &val);

	return sprintf(buf, "%u\n", val);
}
static DEVICE_ATTR_RO(rx_bitcnt);

static ssize_t rx_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	/* read timestamp */
	regmap_read(sai->regmap, FSL_SAI_RTCTN, &val);

	return sprintf(buf, "%u\n", val);
}
static DEVICE_ATTR_RO(rx_timestamp);

static ssize_t rx_bitcnt_latched_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	/* read timestamp */
	regmap_read(sai->regmap, FSL_SAI_RTCAP, &val);

	return sprintf(buf, "%u\n", val);
}
static DEVICE_ATTR_RO(rx_bitcnt_latched_timestamp);

static ssize_t
rx_timestamp_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != 0)
		val = FSL_SAI_xTCTL_TSEN;
	regmap_update_bits(sai->regmap, FSL_SAI_RTCTL, FSL_SAI_xTCTL_TSEN, val);
	return n;
}
static DEVICE_ATTR_WO(rx_timestamp_enable);

static ssize_t
rx_timestamp_increment_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != 0)
		val = FSL_SAI_xTCTL_TSINC;
	regmap_update_bits(sai->regmap, FSL_SAI_RTCTL, FSL_SAI_xTCTL_TSINC, val);
	return n;
}
static DEVICE_ATTR_WO(rx_timestamp_increment);

static ssize_t
rx_bitcnt_reset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != 0)
		val = FSL_SAI_xTCTL_RBC;
	regmap_update_bits(sai->regmap, FSL_SAI_RTCTL, FSL_SAI_xTCTL_RBC, val);
	return n;
}
static DEVICE_ATTR_WO(rx_bitcnt_reset);

static ssize_t
rx_timestamp_reset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int val = 0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != 0)
		val = FSL_SAI_xTCTL_RTSC;
	regmap_update_bits(sai->regmap, FSL_SAI_RTCTL, FSL_SAI_xTCTL_RTSC, val);
	return n;
}
static DEVICE_ATTR_WO(rx_timestamp_reset);

static ssize_t
rx_monitor_spdif_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned char offset = sai->soc_data->reg_offset;
	unsigned int val = 0;
	bool enable = false;
	unsigned int reg;
	unsigned int shift;
	unsigned int mask;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != 0)
		enable = true;

	if (pm_runtime_active(&sai->pdev->dev) && enable) {
		dev_err(dev, "device is busy\n");
		return -EBUSY;
	}

	reg = IOMUXC_GPR6 + (sai->gpr_idx - 1) / 2 * 4;
	shift = ((sai->gpr_idx - 1) % 2) * 16;
	mask = 0x1F << shift;

	if (enable) {
		pm_runtime_get_sync(&sai->pdev->dev);
		/* Fix to MCLK3 */
		regmap_update_bits(sai->regmap_gpr, reg, mask, 0xF << shift);
		regmap_update_bits(sai->regmap, FSL_SAI_xCR2(false, offset),
				   FSL_SAI_CR2_MSEL_MASK, FSL_SAI_CR2_MSEL(0x3));
		regmap_update_bits(sai->regmap, FSL_SAI_xCR2(false, offset),
				   FSL_SAI_CR2_DIV_MASK, 0x0);
		regmap_update_bits(sai->regmap, FSL_SAI_xCR2(false, offset),
				   FSL_SAI_CR2_BCD_MSTR, FSL_SAI_CR2_BCD_MSTR);
		regmap_update_bits(sai->regmap, FSL_SAI_xCR4(false, offset),
				   FSL_SAI_CR4_FSD_MSTR, FSL_SAI_CR4_FSD_MSTR);
		regmap_update_bits(sai->regmap, FSL_SAI_xCSR(false, offset),
				   FSL_SAI_CSR_TERE, FSL_SAI_CSR_TERE);
		sai->monitor_spdif_start = true;
	} else {
		if (sai->monitor_spdif_start) {
			regmap_update_bits(sai->regmap, FSL_SAI_xCSR(false, offset),
					   FSL_SAI_CSR_TERE, 0);
			pm_runtime_put_sync(&sai->pdev->dev);
			sai->monitor_spdif_start = false;
		}
	}

	return n;
}
static DEVICE_ATTR_WO(rx_monitor_spdif);

static struct attribute *fsl_sai_attrs[] = {
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
	NULL,
};

static struct attribute_group fsl_sai_attr_group = {
	.attrs = fsl_sai_attrs,
};

const struct attribute_group *fsl_sai_get_dev_attribute_group(bool monitor_spdif)
{
	if (monitor_spdif)
		fsl_sai_attrs[ARRAY_SIZE(fsl_sai_attrs) - 2] = &dev_attr_rx_monitor_spdif.attr;

	return &fsl_sai_attr_group;
}
