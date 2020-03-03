/*
 * AD9739A SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "ad9739a.h"
#include "cf_axi_dds.h"

struct ad9739a_platform_data {
	bool mix_mode_en;
	u32 fsc_ua;
};

struct ad9739a_phy {
	struct ad9739a_platform_data *pdata;
};

static const char ad9739a_op_modes[2][16] = {"normal-baseband", "mix-mode"};

enum ad9739a_iio_dev_attr {
	AD9739A_FSC,
	AD9739A_OP_MODE_AVAIL,
	AD9739A_OP_MODE,
};

static inline struct ad9739a_phy *conv_to_phy(struct cf_axi_converter *conv)
{
	return conv->phy;
}

static int ad9739a_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[2];
	int ret;

	buf[0] = 0x80 | (0x7F & reg);

	ret = spi_write_then_read(spi, &buf[0], 1, &buf[1], 1);
	if (ret < 0)
		return ret;

	return buf[1];
}

static int ad9739a_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	buf[0] = reg & 0x7F;
	buf[1] = val;
	ret = spi_write_then_read(spi, buf, 2, NULL, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9739a_setup(struct cf_axi_converter *conv)
{
	struct spi_device *spi = conv->spi;
	int repeat, status;

	/* Configure for the 4-wire SPI mode with MSB. */
	ad9739a_write(spi, REG_MODE, 0x00);

	/* Software reset to default SPI values. */
	ad9739a_write(spi, REG_MODE, MODE_RESET);

	/* Clear the reset bit. */
	ad9739a_write(spi, REG_MODE, 0x00);

	/* Set the common-mode voltage of DACCLK_P and DACCLK_N inputs */
	ad9739a_write(spi, REG_CROSS_CNT1, CROSS_CNT1_CLKP_OFFSET(0xF));
	ad9739a_write(spi, REG_CROSS_CNT2, CROSS_CNT2_CLKN_OFFSET(0xF));

	/* Configure the Mu controller. */
	ad9739a_write(spi, REG_PHS_DET, PHS_DET_CMP_BST | PHS_DET_PHS_DET_AUTO_EN);
	ad9739a_write(spi, REG_MU_DUTY, MU_DUTY_MU_DUTY_AUTO_EN);
	ad9739a_write(spi, REG_MU_CNT2, MU_CNT2_SRCH_MODE(2) | MU_CNT2_SET_PHS(4));
	ad9739a_write(spi, REG_MU_CNT3, MU_CNT3_MUDEL(0x6C));

	for (repeat = 0; repeat < 3; repeat++) {
		ad9739a_write(spi, REG_MU_CNT4,
				MU_CNT4_SEARCH_TOL | MU_CNT4_RETRY | MU_CNT4_GUARD(0xB));
		ad9739a_write(spi, REG_MU_CNT1, MU_CNT1_GAIN(1));
		/* Enable the Mu controller search and track mode. */
		ad9739a_write(spi, REG_MU_CNT1, MU_CNT1_GAIN(1) | MU_CNT1_ENABLE);
		mdelay(10);
		status = ad9739a_read(spi, REG_MU_STAT1);
		if (status == MU_STAT1_MU_LKD)
			return 0;
	}

	dev_err(&spi->dev, "Mu lock failure\n\r");

	return -1;
}

static int ad9739a_set_fsc(struct cf_axi_converter *conv, u16 fsc_ua)
{
	struct spi_device *spi = conv->spi;
	struct ad9739a_phy *phy = conv_to_phy(conv);
	u32 reg_val;
	int ret;

	fsc_ua = clamp_t(u16, fsc_ua, AD9739A_MIN_FSC, AD9739A_MAX_FSC);
	reg_val = (fsc_ua - AD9739A_MIN_FSC) * 10 / 226;
	ret = ad9739a_write(spi, REG_FSC_1, FSC_1_FSC_1(reg_val));
	ret |= ad9739a_write(spi, REG_FSC_2, FSC_2_FSC_2((reg_val >> 8)));

	phy->pdata->fsc_ua = fsc_ua;

	return ret;
}

static int ad9739a_get_fsc(struct cf_axi_converter *conv, u16 *fsc_ua)
{
	struct ad9739a_phy *phy = conv_to_phy(conv);

	*fsc_ua = phy->pdata->fsc_ua;

	return 0;
}

static int ad9739a_set_op_mode(struct cf_axi_converter *conv, enum operation_mode op_mode)
{
	struct spi_device *spi = conv->spi;
	struct ad9739a_phy *phy = conv_to_phy(conv);
	int ret;

	if (op_mode == NORMAL_BASEBAND_OPERATION) {
		ret = ad9739a_write(spi, REG_DEC_CNT, DEC_CNT_DAC_DEC(NORMAL_BASEBAND));
		phy->pdata->mix_mode_en = false;
	} else {
		ret = ad9739a_write(spi, REG_DEC_CNT, DEC_CNT_DAC_DEC(MIX_MODE));
		phy->pdata->mix_mode_en = true;
	}

	return ret;
}

static int ad9739a_get_op_mode(struct cf_axi_converter *conv, enum operation_mode *op_mode)
{
	struct ad9739a_phy *phy = conv_to_phy(conv);

	*op_mode = phy->pdata->mix_mode_en ? MIX_MODE_OPERATION : NORMAL_BASEBAND_OPERATION;

	return 0;
}

static int ad9739a_prepare(struct cf_axi_converter *conv)
{
	struct spi_device *spi = conv->spi;
	struct ad9739a_phy *phy = conv_to_phy(conv);
	int repeat, status, ret;

	for (repeat = 0; repeat < 3; repeat++) {
		/* Set FINE_DEL_SKEW to 2. */
		ad9739a_write(spi, REG_LVDS_REC_CNT4,
				LVDS_REC_CNT4_DCI_DEL(0x7) | LVDS_REC_CNT4_FINE_DEL_SKEW(0x2));
		/* Disable the data Rx controller before enabling it. */
		ad9739a_write(spi, REG_LVDS_REC_CNT1, 0x00);
		/* Enable the data Rx controller for loop and IRQ. */
		ad9739a_write(spi, REG_LVDS_REC_CNT1, LVDS_REC_CNT1_RCVR_LOOP_ON);
		/* Enable the data Rx controller for search and track mode. */
		ad9739a_write(spi, REG_LVDS_REC_CNT1,
				LVDS_REC_CNT1_RCVR_LOOP_ON | LVDS_REC_CNT1_RCVR_CNT_ENA);
		mdelay(10);
		status = ad9739a_read(spi, REG_LVDS_REC_STAT9);
		if (status == (LVDS_REC_STAT9_RCVR_TRK_ON | LVDS_REC_STAT9_RCVR_LCK)) {
			break;
		}
	}

	if (repeat < 3) {
		ret = ad9739a_set_fsc(conv, phy->pdata->fsc_ua);
		ret |= ad9739a_set_op_mode(conv, phy->pdata->mix_mode_en);
		return ret;
	} else {
		dev_err(&spi->dev, "Rx data lock failure\n\r");
		return -1;
	}
}

static unsigned long long ad9739a_get_data_clk(struct cf_axi_converter *conv)
{
	return clk_get_rate(conv->clk[CLK_DAC]);
}

static int ad9739a_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	return 0;
}

static int ad9739a_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	return 0;
}

static ssize_t ad9739a_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret = 0;
	u16 fsc_ua;
	enum operation_mode op_mode;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case AD9739A_FSC:
		ret = ad9739a_get_fsc(conv, &fsc_ua);
		ret |= sprintf(buf, "%u\n", fsc_ua);
		break;
	case AD9739A_OP_MODE_AVAIL:
		ret = sprintf(buf, "%s\n", "normal-baseband mix-mode");
		break;
	case AD9739A_OP_MODE:
		ret = ad9739a_get_op_mode(conv, &op_mode);
		ret |= sprintf(buf, "%s\n", ad9739a_op_modes[op_mode]);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static ssize_t ad9739a_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret = 0;
	u16 fsc_ua;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case AD9739A_FSC:
		ret = kstrtou16(buf, 10, &fsc_ua);
		if (ret < 0)
			break;
		ret = ad9739a_set_fsc(conv, fsc_ua);
		break;
	case AD9739A_OP_MODE:
		if (sysfs_streq(buf, "mix-mode"))
			ad9739a_set_op_mode(conv, true);
		else
			ad9739a_set_op_mode(conv, false);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(full_scale_current, S_IRUGO | S_IWUSR,
					ad9739a_show,
					ad9739a_store,
					AD9739A_FSC);

static IIO_DEVICE_ATTR(operation_modes_available, S_IRUGO,
					ad9739a_show,
					NULL,
					AD9739A_OP_MODE_AVAIL);

static IIO_DEVICE_ATTR(operation_mode, S_IRUGO | S_IWUSR,
					ad9739a_show,
					ad9739a_store,
					AD9739A_OP_MODE);

static struct attribute *ad9739a_attributes[] = {
	&iio_dev_attr_full_scale_current.dev_attr.attr,
	&iio_dev_attr_operation_modes_available.dev_attr.attr,
	&iio_dev_attr_operation_mode.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9739a_attribute_group = {
	.attrs = ad9739a_attributes,
};

#ifdef CONFIG_OF
static struct ad9739a_platform_data *ad9739a_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ad9739a_platform_data *pdata;
	u32 tmp;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->mix_mode_en = of_property_read_bool(np, "adi,mix-mode-enable");

	tmp = 20000;
	of_property_read_u32(np, "adi,full-scale-current-ua", &tmp);
	pdata->fsc_ua = tmp;

	return pdata;
}
#else
static struct ad9739a_platform_data *ad9739a_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int ad9739a_probe(struct spi_device *spi)
{
	struct cf_axi_converter *conv;
	struct ad9739a_phy *phy;
	struct clk *clk;
	unsigned id;
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	if (spi->dev.of_node)
		phy->pdata = ad9739a_parse_dt(&spi->dev);
	else
		phy->pdata = spi->dev.platform_data;
	if (!phy->pdata) {
		ret = -EINVAL;
		dev_err(&spi->dev, "No platform data?\n");
		goto out;
	}

	id = ad9739a_read(spi, REG_PART_ID);
	if (id != AD9739A_ID) {
		ret = -ENODEV;
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		goto out;
	}

	conv->phy = phy;
	conv->write = ad9739a_write;
	conv->read = ad9739a_read;
	conv->setup = ad9739a_prepare;

	conv->get_data_clk = ad9739a_get_data_clk;
	conv->write_raw = ad9739a_write_raw;
	conv->read_raw = ad9739a_read_raw;
	conv->attrs = &ad9739a_attribute_group;
	conv->spi = spi;
	conv->id = ID_AD9739A;

	clk = clk_get(&conv->spi->dev, "dac_clk");
	if (IS_ERR(clk)) {
		ret = -EPROBE_DEFER;
		dev_err(&spi->dev, "Failed to get dac_clk\n");
		goto out;
	}

	ret = clk_prepare_enable(clk);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to prepare dac_clk\n");
		goto out;
	}

	conv->clk[CLK_DAC] = clk;

	ret = ad9739a_setup(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	spi_set_drvdata(spi, conv);

	return 0;
out:
	return ret;
}

static const struct spi_device_id ad9739a_id[] = {
	{"ad9739a", 9739},
	{}
};

MODULE_DEVICE_TABLE(spi, ad9739a_id);

static struct spi_driver ad9739a_driver = {
	.driver = {
		   .name = "ad9739a",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9739a_probe,
	.id_table = ad9739a_id,
};
module_spi_driver(ad9739a_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9739 DAC");
MODULE_LICENSE("GPL v2");
