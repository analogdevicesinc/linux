// SPDX-License-Identifier: GPL-2.0
/*
 * SPI Amplifier Driver for the A916x series
 *
 * Copyright 2019 Analog Devices Inc.
 *
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#define AD916x_AMP_REG_SPI_INTFCONFA	0x0
#define AD916x_AMP_SDOACTIVE	BIT(3) | BIT(4)

#define AD916x_AMP_REG_POWERDOWN	0x10
#define AD916X_AMP_ENABLE		0x00
#define AD916X_AMP_DISABLE		0x3B

#define AD916x_AMP_REG_TRIM_CM		0x18
#define AD916x_AMP_REG_DCOUTPUTVOLTAGE	0x19

struct ad916x_amp_state {
	struct regmap *map;
	struct spi_device *spi;
	s16 vos_adj_mv;
	u16 icm_ua;
};

static const struct regmap_config ad916x_amp_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
};

#define AD916x_VOS_ADJ_MIN_MV (-250)
#define AD916x_VOS_ADJ_MAX_MV 350
#define AD916x_VOS_ADJ_MIN_TO_MAX (AD916x_VOS_ADJ_MAX_MV - \
				   AD916x_VOS_ADJ_MIN_MV)
#define AD916x_AMP_VOUT_TRIM_MAX ((1 << 8) - 1 )

static int ad916x_amp_vos_adj_mv_set(struct ad916x_amp_state *st, s16 vos_adj_mv)
{
	u8 vout_trim;
	int rc;

	if (vos_adj_mv < AD916x_VOS_ADJ_MIN_MV
	    || vos_adj_mv > AD916x_VOS_ADJ_MAX_MV)
		return -EINVAL;

	vout_trim = (vos_adj_mv - AD916x_VOS_ADJ_MIN_MV)
		    * AD916x_AMP_VOUT_TRIM_MAX
		    / AD916x_VOS_ADJ_MIN_TO_MAX;

	rc = regmap_write(st->map, AD916x_AMP_REG_DCOUTPUTVOLTAGE, vout_trim);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to set DC offset: %d\n", rc);
		return rc;
	}

	st->vos_adj_mv = vos_adj_mv;

	return 0;
}

#define AD916x_ICM_MIN_UA 6400
#define AD916x_ICM_MAX_UA 30400
#define AD916x_ICM_MIN_TO_MAX (AD916x_ICM_MAX_UA - \
			       AD916x_ICM_MIN_UA)
#define AD916x_AMP_ICM_MAX ((1 << 4) - 1 )

static int ad916x_amp_icm_ua_set(struct ad916x_amp_state *st, u16 icm_ua)
{
	u8 amp_icm;
	int rc;

	if (icm_ua < AD916x_ICM_MIN_UA
	    || icm_ua > AD916x_ICM_MAX_UA)
		return -EINVAL;

	amp_icm = (icm_ua - AD916x_ICM_MIN_UA)
		      * AD916x_AMP_ICM_MAX
		      / AD916x_ICM_MIN_TO_MAX;

	rc = regmap_write(st->map, AD916x_AMP_REG_TRIM_CM, amp_icm);
	if (rc) {
		dev_err(&st->spi->dev, "Failed to set input common-mode "
			"current: %d\n", rc);
		return rc;
	}

	st->icm_ua = icm_ua;

	return 0;
}

/* Attributes */
enum {
	AD916x_AMP_ICM_UA,
	AD916x_AMP_ICM_UA_MIN,
	AD916x_AMP_ICM_UA_MAX,
	AD916x_AMP_VOS_ADJ_MV,
	AD916x_AMP_VOS_ADJ_MV_MIN,
	AD916x_AMP_VOS_ADJ_MV_MAX,
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
		if (val != 0 && val != 1)
			return -EINVAL;

		return regmap_write(st->map, AD916x_AMP_REG_POWERDOWN,
				    val == 1 ? AD916X_AMP_ENABLE :
				    AD916X_AMP_DISABLE);
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

	switch (this_attr->address) {
	case AD916x_AMP_VOS_ADJ_MV:
		return sprintf(buf, "%d\n", st->vos_adj_mv);
	case AD916x_AMP_VOS_ADJ_MV_MIN:
		return sprintf(buf, "%d\n", AD916x_VOS_ADJ_MIN_MV);
	case AD916x_AMP_VOS_ADJ_MV_MAX:
		return sprintf(buf, "%d\n", AD916x_VOS_ADJ_MAX_MV);
	case AD916x_AMP_ICM_UA:
		return sprintf(buf, "%u\n", st->icm_ua);
	case AD916x_AMP_ICM_UA_MIN:
		return sprintf(buf, "%u\n", AD916x_ICM_MIN_UA);
	case AD916x_AMP_ICM_UA_MAX:
		return sprintf(buf, "%u\n", AD916x_ICM_MAX_UA);
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
	unsigned int uval;
	int sval;
	int rc;

	switch (this_attr->address) {
	case AD916x_AMP_ICM_UA:
		rc = kstrtouint(buf, 0, &uval);
		break;
	case AD916x_AMP_VOS_ADJ_MV:
		rc = kstrtoint(buf, 0, &sval);
		break;
	default:
		rc = -EINVAL;
	}

	if (rc)
		return rc;

	switch (this_attr->address) {
	case AD916x_AMP_ICM_UA:
		rc = ad916x_amp_icm_ua_set(st, uval);
		break;
	case AD916x_AMP_VOS_ADJ_MV:
		rc = ad916x_amp_vos_adj_mv_set(st, sval);
		break;
	default:
		return -EINVAL;
	}

	return rc ? rc : len;
}

#define AD916x_IIO_DEVICE_ATTR(name, perm, addr) \
	IIO_DEVICE_ATTR(name, perm, ad916x_attr_show, ad916x_attr_store, addr)

AD916x_IIO_DEVICE_ATTR(icm_ua,
		       S_IRUGO | S_IWUSR,
		       AD916x_AMP_ICM_UA);
AD916x_IIO_DEVICE_ATTR(icm_ua_min,
		       S_IRUGO,
		       AD916x_AMP_ICM_UA_MIN);
AD916x_IIO_DEVICE_ATTR(icm_ua_max,
		       S_IRUGO,
		       AD916x_AMP_ICM_UA_MAX);
AD916x_IIO_DEVICE_ATTR(vos_adj_mv,
		       S_IRUGO | S_IWUSR,
		       AD916x_AMP_VOS_ADJ_MV);
AD916x_IIO_DEVICE_ATTR(vos_adj_mv_min,
		       S_IRUGO,
		       AD916x_AMP_VOS_ADJ_MV_MIN);
AD916x_IIO_DEVICE_ATTR(vos_adj_mv_max,
		       S_IRUGO,
		       AD916x_AMP_VOS_ADJ_MV_MAX);

static struct attribute *ad916x_amp_attributes[] = {
	&iio_dev_attr_icm_ua.dev_attr.attr,
	&iio_dev_attr_icm_ua_min.dev_attr.attr,
	&iio_dev_attr_icm_ua_max.dev_attr.attr,
	&iio_dev_attr_vos_adj_mv.dev_attr.attr,
	&iio_dev_attr_vos_adj_mv_min.dev_attr.attr,
	&iio_dev_attr_vos_adj_mv_max.dev_attr.attr,
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

static void ad916x_amp_setup(const struct ad916x_amp_state *st)
{
	if (!(st->spi->mode & SPI_3WIRE))
		/* set SPI to 4-wire mode */
		regmap_write(st->map, AD916x_AMP_REG_SPI_INTFCONFA,
			     AD916x_AMP_SDOACTIVE);
}

static int ad916x_amp_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad916x_amp_state *st;
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct device_node *np = spi->dev.of_node;
	const char *dev_name;
	s32 sval;
	u32 uval;
	int rc;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "Failed to alloc iio dev\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);

	st->map = devm_regmap_init_spi(spi, &ad916x_amp_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	if (of_property_read_s32(np, "adi,vos-adj-mv", &sval) == 0) {
		dev_info(&spi->dev, "vos-adj-mv: %d\n", sval);
		rc = ad916x_amp_vos_adj_mv_set(st, sval);
		if (rc)
			return rc;
	}

	if (of_property_read_u32(np, "adi,icm-ua", &uval) == 0) {
		dev_info(&spi->dev, "icm-ua: %u\n", uval);
		rc = ad916x_amp_icm_ua_set(st, uval);
		if (rc)
			return rc;
	}

	st->spi = spi;

	dev_name = np ? np->name : dev_id->name;

	ad916x_amp_setup(st);

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
