/*
 * AD9783 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2021 Analog Devices Inc.
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
#include "ad9783.h"
#include "cf_axi_dds.h"

struct ad9783_platform_data {
	bool mix_mode_en;
	u32 fsc_ua;
};

struct ad9783_phy {
	struct ad9783_platform_data *pdata;
};

// similar to drivers/iio/frequency/m2k-dac.c: struct m2k_dac
struct ad9783_dac {
	void __iomem *regs;
	struct clk *clk;

	struct cf_axi_dds_state *dds;

	struct mutex lock;

	strct iio_dev *ch_indio_dev[2];
}

// similar to struct m2k_dac_ch
struct ad9783_dac_ch {
	struct m2k_dac *dac;
	unsigned int num;
}

static const char ad9783_op_modes[2][16] = {"normal-baseband", "mix-mode"};

enum ad9783_iio_dev_attr {
	AD9783_FSC,
	AD9783_OP_MODE_AVAIL,
	AD9783_OP_MODE,
};

static inline struct ad9783_phy *conv_to_phy(struct cf_axi_converter *conv)
{
	return conv->phy;
}

static int ad9783_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[2];
	int ret;

	buf[0] = 0x80 | (0x7F & reg);

	ret = spi_write_then_read(spi, &buf[0], 1, &buf[1], 1);
	if (ret < 0)
		return ret;

	return buf[1];
}

static int ad9783_write(struct spi_device *spi, unsigned reg, unsigned val)
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

static int ad9783_setup(struct cf_axi_converter *conv)
{
	struct spi_device *spi = conv->spi;

	/* Configure for the 4-wire SPI mode (bit 7) with MSB (bit 6). */
	/* Software reset (bit 5) to default SPI values. */
	ad9783_write(spi, REG_SPI_CTRL, 0x00);

	// bit 7 -> 0 = DAC input data is in 2's complement
	//       -> 1 = DAC input data is in unsigned binary format
	ad9783_write(spi, REG_DATA_CTRL, 0x80);
	ad9783_write(spi, REG_SETUP_HOLD, 0x27);
	ad9783_write(spi, REG_SMP, 0x8); // 7 is the value from datasheet

	return 0;
}

static int ad9783_set_fsc(struct cf_axi_converter *conv, u16 fsc_ua)
{
	struct spi_device *spi = conv->spi;
	struct ad9783_phy *phy = conv_to_phy(conv);
	u32 reg_val;
	int ret;

	fsc_ua = clamp_t(u16, fsc_ua, AD9783_MIN_FSC, AD9783_MAX_FSC);
	reg_val = (fsc_ua - AD9783_MIN_FSC) * 10 / 226;
	ret = ad9783_write(spi, REG_DAC_1_FSC, FSC_1_FSC_1(reg_val));
	ret |= ad9783_write(spi, REG_DAC_2_FSC, FSC_2_FSC_2((reg_val >> 8)));
	phy->pdata->fsc_ua = fsc_ua;

	return ret;
}

static int ad9783_get_fsc(struct cf_axi_converter *conv, u16 *fsc_ua)
{
	struct ad9783_phy *phy = conv_to_phy(conv);

	*fsc_ua = phy->pdata->fsc_ua;

	return 0;
}

static int ad9783_set_op_mode(struct cf_axi_converter *conv, enum operation_mode op_mode)
{
	struct spi_device *spi = conv->spi;
	struct ad9783_phy *phy = conv_to_phy(conv);
	int ret;

	// !!! TO BE DONE: separate the setting of the operation mode by DAC1 and DAC2
	if (op_mode == NORMAL_BASEBAND_OPERATION) {
		ret = ad9783_write(spi, REG_MIX_MODE, NORMAL_BASEBAND);
		phy->pdata->mix_mode_en = false;
	} else {
		ret = ad9783_write(spi, REG_MIX_MODE, MIX_MODE);
		phy->pdata->mix_mode_en = true;
	}

	return ret;
}

static int ad9783_get_op_mode(struct cf_axi_converter *conv, enum operation_mode *op_mode)
{
	struct ad9783_phy *phy = conv_to_phy(conv);

	*op_mode = phy->pdata->mix_mode_en ? MIX_MODE_OPERATION : NORMAL_BASEBAND_OPERATION;

	return 0;
}

static int ad9783_prepare(struct cf_axi_converter *conv)
{
	struct spi_device *spi = conv->spi;
	struct ad9783_phy *phy = conv_to_phy(conv);
	int repeat, status, ret;

	return 0;

	/*
	for (repeat = 0; repeat < 3; repeat++) {
		// Set FINE_DEL_SKEW to 2.
		ad9783_write(spi, REG_LVDS_REC_CNT4,
		LVDS_REC_CNT4_DCI_DEL(0x7) | LVDS_REC_CNT4_FINE_DEL_SKEW(0x2));
		// Disable the data Rx controller before enabling it.
		ad9783_write(spi, REG_LVDS_REC_CNT1, 0x00);
		// Enable the data Rx controller for loop and IRQ.
		ad9783_write(spi, REG_LVDS_REC_CNT1, LVDS_REC_CNT1_RCVR_LOOP_ON);
		// Enable the data Rx controller for search and track mode.
		ad9783_write(spi, REG_LVDS_REC_CNT1,
				LVDS_REC_CNT1_RCVR_LOOP_ON | LVDS_REC_CNT1_RCVR_CNT_ENA);
		mdelay(10);
		status = ad9783_read(spi, REG_LVDS_REC_STAT9);
		if (status == (LVDS_REC_STAT9_RCVR_TRK_ON | LVDS_REC_STAT9_RCVR_LCK)) {
			break;
		}
	}

	if (repeat < 3) {
		ret = ad9783_set_fsc(conv, phy->pdata->fsc_ua);
		ret |= ad9783_set_op_mode(conv, phy->pdata->mix_mode_en);
		return ret;
	} else {
		dev_err(&spi->dev, "Rx data lock failure\n\r");
		return -1;
	}
	*/
}


static unsigned long long ad9783_get_data_clk(struct cf_axi_converter *conv)
{
	return clk_get_rate(conv->clk[CLK_DAC]);
}

static int ad9783_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	return 0;
}

static int ad9783_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	return 0;
}

static ssize_t ad9783_show(struct device *dev,
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
	case AD9783_FSC:
		ret = ad9783_get_fsc(conv, &fsc_ua);
		ret |= sprintf(buf, "%u\n", fsc_ua);
		break;
	case AD9783_OP_MODE_AVAIL:
		ret = sprintf(buf, "%s\n", "normal-baseband mix-mode");
		break;
	case AD9783_OP_MODE:
		ret = ad9783_get_op_mode(conv, &op_mode);
		ret |= sprintf(buf, "%s\n", ad9783_op_modes[op_mode]);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static ssize_t ad9783_store(struct device *dev,
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
	case AD9783_FSC:
		ret = kstrtou16(buf, 10, &fsc_ua);
		if (ret < 0)
			break;
		ret = ad9783_set_fsc(conv, fsc_ua);
		break;
	case AD9783_OP_MODE:
		if (sysfs_streq(buf, "mix-mode"))
			ad9783_set_op_mode(conv, true);
		else
			ad9783_set_op_mode(conv, false);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(full_scale_current, S_IRUGO | S_IWUSR,
					ad9783_show,
					ad9783_store,
					AD9783_FSC);

static IIO_DEVICE_ATTR(operation_modes_available, S_IRUGO,
					ad9783_show,
					NULL,
					AD9783_OP_MODE_AVAIL);

static IIO_DEVICE_ATTR(operation_mode, S_IRUGO | S_IWUSR,
					ad9783_show,
					ad9783_store,
					AD9783_OP_MODE);

static struct attribute *ad9783_attributes[] = {
	&iio_dev_attr_full_scale_current.dev_attr.attr,
	&iio_dev_attr_operation_modes_available.dev_attr.attr,
	&iio_dev_attr_operation_mode.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9783_attribute_group = {
	.attrs = ad9783_attributes,
};

#ifdef CONFIG_OF
static struct ad9783_platform_data *ad9783_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ad9783_platform_data *pdata;
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
static struct ad9783_platform_data *ad9783_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

/* Function to check if the registers that contain the result sum are equal to
 * the known values
 * */
static bool ad9783_equal_bist_regs(struct spi_device *spi)
{
	int reg_1b, reg_1c;
	int reg_1d, reg_1e;

	// prepare BIST for reading operation; bring the sums into the registers
	ad9783_write(spi, REG_BIST_CTRL, 0xC0);

	reg_1b = ad9783_read(spi, REG_BIST_RES_1_LOW);
	reg_1c = ad9783_read(spi, REG_BIST_RES_1_HIGH);
	reg_1d = ad9783_read(spi, REG_BIST_RES_2_LOW);
	reg_1e = ad9783_read(spi, REG_BIST_RES_2_HIGH);

	return ((reg_1b == 0x8) && (reg_1c == 0x82)
	&& (reg_1b == reg_1d) && (reg_1c == reg_1e));
}

/*
 * Function that calibrates the device by trying multiple configurations for
 * the Timing Adjust register (or SAMP_DLY[4:0] found at 0x05).
 * */
static int ad9783_prepare_bist(struct spi_device *spi,
			       struct iio_dev *indio_dev)
{
	struct ad9783_dac_ch *ch = iio_priv(indio_dev);
	struct ad9783_dac * ad9783_dac = ch->dac;

	ad9783_write(spi, REG_DATA_CTRL, 0x80);
	ad9783_write(spi, REG_SETUP_HOLD, 0x0);
	//ad9783_write(spi, REG_SMP, 0x3);

	//
	ad9783_write(spi, REG_BIST_CTRL, 0x0);

	// dac_dds_sel to send 0's
	cf_axi_dds_datasel(ad9783_dac->dds, 0, DATA_SEL_ZERO);

	// enable BIST
	ad9783_write(spi, REG_BIST_CTRL, 0x20);
	ad9783_write(spi, REG_BIST_CTRL, 0x0);
	ad9783_write(spi, REG_BIST_CTRL, 0x80);

	// dac_dds_sel to send PN23
	cf_axi_dds_datasel(ad9783_dac->dds, 0, DATA_SEL_PNXX);

	//ad9783_write(spi, REG_BIST_CTRL, 0xC0);
	return 0;
}

static int ad9783_calibrate(struct cf_axi_converter *conv,
			    struct iio_dev *ad9783)
{
	pr_err("\n\n\n----------> Intru in calibrare <----------\n\n\n");
	struct spi_device *spi = conv->spi;
	int smp_dly;
	int reg_1b, reg_1c;
	int reg_1d, reg_1e;
	bool found = false;

	// initially hardcoded
	smp_dly = 3;
	ad9783_prepare_bist(spi, ad9783_dev);

	do {
		while (!ad9783_equal_bist_regs(spi)) {
			smp_dly++;
			ad9783_write(spi, REG_SMP, smp_dly);
			ad9783_prepare_bist(spi, ad9783_dev);
		}

		// if they are equal, then
		for (int i = 0; i < 100; i++) {
			ad9783_prepare_bist(spi, ad9783_dev);

			if (!ad9783_equal_bist_regs(spi)) {
				calibrated = false;
				i = 1001;
			}
		}

		if (calibrated) {
			found = true;
		}
	} while (!found);

	pr_err("\n\n\n----------> Found SMP_DLY=%d <----------\n\n\n", smp_dly);
/*
	bool calibrated = false;

	while (!calibrated) {
		smp_dly = ad9783_read(spi, REG_SMP);
		// enable BIST read; prepare sums in the assigned registers
	        ad9783_write(spi, REG_BIST_CTRL, 0xC0);
		reg_1b = ad9783_read(spi, REG_BIST_RES_1_LOW);
		reg_1c = ad9783_read(spi, REG_BIST_RES_1_HIGH);
		reg_1d = ad9783_read(spi, REG_BIST_RES_2_LOW);
		reg_1e = ad9783_read(spi, REG_BIST_RES_2_HIGH);

		if ((reg_1b == 0x8) && (reg_1c == 0x82)
		&& (reg_1b == reg_1d) && (reg_1c == reg_1e)) {
			// rulez de 1000 de ori sa ma asigur ca nu crapa pe
			// varianta asta, si daca e ok, atunci pot zice ca e calibrat

			for (int i = 0; i < 1000; i++) {
				ad9783_check_interface();
			}
		}
		else {
			smp_dly++;
			ad9783_write(spi, REG_SMP, smp_dly);
			calibrated = false;
		}
	}
	return 0;
*/
}

static int ad9783_probe(struct spi_device *spi)
{
	struct cf_axi_converter *conv;
	struct ad9783_phy *phy;
	struct clk *clk;
	unsigned id;
	int ret;

	pr_err("\n\n----------> Am intrat in probe <----------\n\n");

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	if (spi->dev.of_node) {
		phy->pdata = ad9783_parse_dt(&spi->dev);
	}
	else {
		phy->pdata = spi->dev.platform_data;
	}

	if (!phy->pdata) {
		ret = -EINVAL;
		dev_err(&spi->dev, "No platform data?\n");
		goto out;
	}

	id = (ad9783_read(spi, REG_VERSION_PART_ID) & 0x0F);

	pr_err("\n\n----------> PART_ID = %d <----------\n\n", id);

	if (id != AD9783_ID) {
		ret = -ENODEV;
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		goto out;
	}

	conv->phy = phy;
	conv->write = ad9783_write;
	conv->read = ad9783_read;
	conv->setup = ad9783_prepare;

	conv->get_data_clk = ad9783_get_data_clk;
	conv->write_raw = ad9783_write_raw;
	conv->read_raw = ad9783_read_raw;
	conv->attrs = &ad9783_attribute_group;
	conv->spi = spi;
	conv->id = ID_AD9783;

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

	//ret = ad9783_setup(conv);

	// AD9783 device related variables
	struct iio_dev *indio_dev;


	ret = ad9783_calibrate(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to calibrate device\n");
		goto out;
	}

	spi_set_drvdata(spi, conv);

	pr_err("\n\n----------> Ies din probe <----------\n\n");
	return 0;
out:
	pr_err("\n\n----------> Ies din probe cu ret <----------\n\n");
	return ret;
}

static const struct spi_device_id ad9783_id[] = {
	{"ad9783", 9783},
	{}
};

MODULE_DEVICE_TABLE(spi, ad9783_id);

static struct spi_driver ad9783_driver = {
	.driver = {
		   .name = "ad9783",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9783_probe,
	.id_table = ad9783_id,
};
module_spi_driver(ad9783_driver);

MODULE_AUTHOR("Iulia Moldovan <iulia.moldovan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9783 DAC");
MODULE_LICENSE("GPL v2");
