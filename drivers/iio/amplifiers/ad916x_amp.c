// SPDX-License-Identifier: GPL-2.0
/*
 * SPI Amplifier Driver for the A916x series
 *
 * Copyright 2019 Analog Devices Inc.
 *
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#define AD916X_AMP_REG(x)	x
#define AD916X_AMP_ENABLE	0x00
#define AD916X_AMP_DISABLE	0x3B

#define AD916x_FLD_ANA_FSC_LSB		0x3
#define AD916x_REG_ANA_FSC0		0x041
#define AD916x_REG_ANA_FSC1		0x042
#define AD916x_FLD_ANA_FSC_MSB		0xFF

#define AD916x_AMP_REG_SPI_INTFCONFA	0x00
#define AD916x_AMP_REG_POWERDOWN	0x10
#define AD916x_AMP_REG_TRIM_CM		0x18
#define AD916x_AMP_REG_DCOUTPUTVOLTAGE	0x19
#define AD916x_AMP_REG_ADC_START	0x1B
#define AD916x_AMP_REG_ADC_EOC		0x1C
#define AD916x_AMP_REG_ADC_RESULTS	0x1D
#define AD916x_AMP_REG_VOUT_TRIM	0xFF

#define AD916x_AMP_FLD_ST_ADC_CLKF_0	0x1
#define AD916x_AMP_FLD_ST_ADC_CLKF_1	0x2
#define AD916x_AMP_FLD_ADC_EOC		0x1
#define AD916x_AMP_FLD_PD_ADCCLOCK	0x1
#define AD916x_AMP_FLD_PD_BG		0x2
#define AD916x_AMP_FLD_PD_CMDACCURRENT	0x8
#define AD916x_AMP_FLD_CM_SET		0xF
#define AD916x_AMP_FLD_PD_PMIRROR	0x10
#define AD916x_AMP_FLD_SPI_4WIRE	0x18
#define AD916x_AMP_FLD_PD_NMIRROR	0x20
#define AD916x_AMP_FLD_SOFTRESET	0x81
#define AD916x_AMP_ENABLE_MASK		(AD916x_AMP_FLD_PD_NMIRROR \
					| AD916x_AMP_FLD_PD_PMIRROR \
					| AD916x_AMP_FLD_PD_CMDACCURRENT \
					| AD916x_AMP_FLD_PD_ADCCLOCK)

struct ad916x_amp_state {
	struct regmap *map;
	struct spi_device *spi;
	struct {
		u8 p_mir;
		u8 n_mir;
		u8 dac_current;
	} amp_pwr_down;
	u16 amp_cm;
	u8 amp_adc_pwr_down;
	u64 amp_vout_trim;
};

static const struct regmap_config ad916x_amp_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
};

static int ad916x_amp_en_set(struct ad916x_amp_state *st, int enable)
{
	if (enable)
		return regmap_update_bits(st->map,
				AD916x_AMP_REG_POWERDOWN,
				AD916x_AMP_ENABLE_MASK | AD916x_AMP_FLD_PD_BG,
				~(AD916x_AMP_ENABLE_MASK | AD916x_AMP_FLD_PD_BG));

	return regmap_update_bits(st->map,
				AD916x_AMP_REG_POWERDOWN,
				AD916x_AMP_ENABLE_MASK,
				AD916x_AMP_ENABLE_MASK);
}

static int ad916x_amp_pwr_down_update(struct ad916x_amp_state *st)
{
	u32 val;
	u32 mask;

	val = 0;
	if (st->amp_pwr_down.p_mir)
		val |= AD916x_AMP_FLD_PD_PMIRROR;

	if (st->amp_pwr_down.n_mir)
		val |= AD916x_AMP_FLD_PD_NMIRROR;

	if (st->amp_pwr_down.dac_current)
		val |= AD916x_AMP_FLD_PD_CMDACCURRENT;

	mask = AD916x_AMP_FLD_PD_PMIRROR | AD916x_AMP_FLD_PD_NMIRROR
		| AD916x_AMP_FLD_PD_CMDACCURRENT;
	return regmap_update_bits(st->map, AD916x_AMP_REG_POWERDOWN, mask, val);
}

static int ad916x_amp_adc_pwr_down(struct ad916x_amp_state *st, u32 adc_clk)
{
	u32 val;

	val = 0;
	if (adc_clk)
		val |= AD916x_AMP_FLD_PD_ADCCLOCK;

	return regmap_update_bits(st->map, AD916x_AMP_REG_POWERDOWN,
				AD916x_AMP_FLD_PD_ADCCLOCK, val);
}

static int ad916x_amp_vout_trim_set(struct ad916x_amp_state *st, u64 vos_adj_uv)
{
	u64 aux;
	u32 vout_trim;

	aux = (vos_adj_uv + 250000) * 255;
	div_s64(aux, 600000);
	vout_trim = aux;
	return regmap_write(st->map, AD916x_AMP_REG_DCOUTPUTVOLTAGE,
			   (vout_trim & AD916x_AMP_REG_VOUT_TRIM));
}

static int ad916x_amp_cm_set(struct ad916x_amp_state *st)
{
	int err;
	u32 val;
	u32 amp_current;
	u16 fsc_val;

	err = regmap_read(st->map, AD916x_REG_ANA_FSC0, &val);
	if (err < 0)
		return err;

	fsc_val = val & AD916x_FLD_ANA_FSC_LSB;
	err = regmap_read(st->map, AD916x_REG_ANA_FSC0, &val);
	if (err < 0)
		return err;

	fsc_val = val & AD916x_FLD_ANA_FSC_LSB;
	err = regmap_read(st->map, AD916x_REG_ANA_FSC1, &val);
	if (err < 0)
		return err;

	fsc_val = fsc_val + ((val & AD916x_FLD_ANA_FSC_MSB) << 2);
	amp_current = fsc_val * 10 / 1023 + 1;

	return regmap_write(st->map, AD916x_AMP_REG_TRIM_CM,
				amp_current & AD916x_AMP_FLD_CM_SET);
}

/* Attributes */
enum {
	AD916x_AMP_POWERDOWN_P_MIR,
	AD916x_AMP_POWERDOWN_N_MIR,
	AD916x_AMP_POWERDOWN_DAC_CURRENT,
	AD916x_AMP_ADC_PWR_DOWN,
	AD916x_AMP_CM,
	AD916x_AMP_VOUT_TRIM,
	AD916x_AMP_TEMP_ADC_FREQ,
	AD916x_AMP_TEMP_VBG_UV,
};

static int ad916x_amp_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val,
			       int *val2,
			       long mask)
{
	struct ad916x_amp_state *st = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = regmap_read(st->map, AD916x_AMP_REG_POWERDOWN, val);
		if (ret)
			return ret;

		*val = !(*val);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad916x_amp_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val,
				int val2,
				long mask)
{
	struct ad916x_amp_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		return ad916x_amp_en_set(st, !!val);
	default:
		return -EINVAL;
	}
}

static int ad916x_amp_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				 unsigned int writeval, unsigned int *readval)
{
	struct ad916x_amp_state *st = iio_priv(indio_dev);

	if (!readval)
		return regmap_write(st->map, reg, writeval);
	else
		return regmap_read(st->map, reg, readval);
}

static ssize_t ad916x_attr_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad916x_amp_state *st = iio_priv(indio_dev);

	switch ((u32)this_attr->address) {
	case AD916x_AMP_POWERDOWN_P_MIR:
		return sprintf(buf, "%u\n", st->amp_pwr_down.p_mir);
	case AD916x_AMP_POWERDOWN_N_MIR:
		return sprintf(buf, "%u\n", st->amp_pwr_down.n_mir);
	case AD916x_AMP_POWERDOWN_DAC_CURRENT:
		return sprintf(buf, "%u\n", st->amp_pwr_down.dac_current);
	case AD916x_AMP_ADC_PWR_DOWN:
		return sprintf(buf, "%u\n", st->amp_adc_pwr_down);
	case AD916x_AMP_CM:
		return sprintf(buf, "%u\n", st->amp_cm);
	case AD916x_AMP_VOUT_TRIM:
		return sprintf(buf, "%llu\n", st->amp_vout_trim);
	default:
		return -EINVAL;
	}
}

static ssize_t ad916x_attr_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad916x_amp_state *st = iio_priv(indio_dev);
	unsigned long long readin;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	switch ((u32)this_attr->address) {
	case AD916x_AMP_POWERDOWN_P_MIR:
		st->amp_pwr_down.p_mir = !!readin;
		ret = ad916x_amp_pwr_down_update(st);
		break;
	case AD916x_AMP_POWERDOWN_N_MIR:
		st->amp_pwr_down.n_mir = !!readin;
		ret = ad916x_amp_pwr_down_update(st);
		break;
	case AD916x_AMP_POWERDOWN_DAC_CURRENT:
		st->amp_pwr_down.dac_current = !!readin;
		ret = ad916x_amp_pwr_down_update(st);
		break;
	case AD916x_AMP_ADC_PWR_DOWN:
		st->amp_adc_pwr_down = !!readin;
		ret = ad916x_amp_adc_pwr_down(st, st->amp_adc_pwr_down);
		break;
	case AD916x_AMP_CM:
		st->amp_cm = readin;
		ret = ad916x_amp_cm_set(st);
		break;
	case AD916x_AMP_VOUT_TRIM:
		st->amp_vout_trim = readin;
		ret = ad916x_amp_vout_trim_set(st, st->amp_vout_trim);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

#define AD916x_IIO_DEVICE_ATTR(name, addr) IIO_DEVICE_ATTR(name,\
							   0644,\
							   ad916x_attr_show,\
							   ad916x_attr_store,\
							   (addr))

AD916x_IIO_DEVICE_ATTR(pwr_down_p_mir, AD916x_AMP_POWERDOWN_P_MIR);
AD916x_IIO_DEVICE_ATTR(pwr_down_n_mir, AD916x_AMP_POWERDOWN_N_MIR);
AD916x_IIO_DEVICE_ATTR(pwr_down_dac_current, AD916x_AMP_POWERDOWN_DAC_CURRENT);
AD916x_IIO_DEVICE_ATTR(amp_adc_pwr_down, AD916x_AMP_ADC_PWR_DOWN);
AD916x_IIO_DEVICE_ATTR(amp_cm, AD916x_AMP_CM);
AD916x_IIO_DEVICE_ATTR(amp_vout_trim, AD916x_AMP_VOUT_TRIM);

static struct attribute *ad916x_amp_attributes[] = {
	&iio_dev_attr_pwr_down_p_mir.dev_attr.attr,
	&iio_dev_attr_pwr_down_n_mir.dev_attr.attr,
	&iio_dev_attr_pwr_down_dac_current.dev_attr.attr,
	&iio_dev_attr_amp_adc_pwr_down.dev_attr.attr,
	&iio_dev_attr_amp_cm.dev_attr.attr,
	&iio_dev_attr_amp_vout_trim.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad916x_attr_group = {
	.attrs = ad916x_amp_attributes,
};

static const struct iio_info ad916x_amp_info = {
	.read_raw = ad916x_amp_read_raw,
	.write_raw = ad916x_amp_write_raw,
	.debugfs_reg_access = &ad916x_amp_reg_access,
	.attrs = &ad916x_attr_group,
};

#define AD916X_AMP_CHAN(index)	{ \
	.type = IIO_ALTVOLTAGE, \
	.indexed = 1, \
	.channel = index, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE), \
	.output = 1, \
}

static struct iio_chan_spec ad916x_amp_chan_spec[] = {
	AD916X_AMP_CHAN(0),
};

static int ad916x_amp_setup(struct ad916x_amp_state *st)
{
	int		err;

	err = regmap_write(st->map, AD916x_AMP_REG_SPI_INTFCONFA,
			   AD916x_AMP_FLD_SOFTRESET);
	if (err < 0)
		return err;

	if (st->spi->mode & SPI_3WIRE)
		err = regmap_write(st->map, AD916x_AMP_REG_SPI_INTFCONFA, 0);
	else
		/* set SPI to 4-wire mode */
		err = regmap_write(st->map, AD916x_AMP_REG_SPI_INTFCONFA,
				   AD916x_AMP_FLD_SPI_4WIRE);

	if (err < 0)
		return err;

	err = ad916x_amp_cm_set(st);
	if (err < 0)
		return err;

	err = ad916x_amp_en_set(st, 1);
	if (err < 0)
		return err;

	return 0;
}

static int ad916x_amp_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad916x_amp_state *st;
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct device_node *np = spi->dev.of_node;
	const char *dev_name;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "Failed to alloc iio dev\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);

	st->map = devm_regmap_init_spi(spi, &ad916x_amp_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	st->spi = spi;

	dev_name = np ? np->name : dev_id->name;

	ret = ad916x_amp_setup(st);
	if (ret < 0)
		return ret;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = dev_name;
	indio_dev->channels = ad916x_amp_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(ad916x_amp_chan_spec);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad916x_amp_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad916x_amp_dt_id[] = {
	{ .compatible = "adi,ad9166-amp" },
	{},
};
MODULE_DEVICE_TABLE(of, ad916x_amp_dt_id);

static const struct spi_device_id ad916x_amp_id[] = {
	{ "ad9166-amp", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, ad916x_amp_id);

static struct spi_driver ad916x_amp_driver = {
	.driver = {
		.name = "ad916x-amp",
		.of_match_table = ad916x_amp_dt_id,
	},
	.probe = ad916x_amp_probe,
	.id_table = ad916x_amp_id,
};

module_spi_driver(ad916x_amp_driver);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD916x Amplifier");
MODULE_LICENSE("GPL v2");
