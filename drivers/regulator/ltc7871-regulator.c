// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices LTC7871 Voltage Regulator Driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/math.h>
#include <linux/crc8.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/bitfield.h>

#define LTC7871_REG_FAULT		0x01
#define LTC7871_REG_CONFIG2		0x06
#define LTC7871_REG_CHIP_CTRL		0x07
#define LTC7871_REG_IDAC_VLOW		0x08
#define LTC7871_REG_IDAC_VHIGH		0x09
#define LTC7871_REG_SETCUR		0x0A
#define LTC7871_REG_SSFM		0x0B

#define LTC7871_FAULT_OVER_TEMP		BIT(0)
#define LTC7871_FAULT_VHIGH_UV		BIT(4)
#define LTC7871_FAULT_VHIGH_OV		BIT(5)
#define LTC7871_FAULT_VLOW_OV		BIT(6)

#define LTC7871_CONFIG2_BUCK_BOOST	BIT(0)

#define LTC7871_MASK_CHIP_CTRL_WP	BIT(0)

#define	LTC7871_MASK_SSFM_FREQ_SPREAD	GENMASK(4, 3)
#define	LTC7871_MASK_SSFM_MOD_SIG_FREQ	GENMASK(2, 0)

#define LTC7871_CRC_INIT		0x41
#define LTC7871_CRC8_POLY		0x7
#define LTC7871_DATA_POS		1
#define LTC7871_CRC_POS			2
#define LTC7871_FRAME_SIZE		3

#define LTC7871_IDAC_MAX		63
#define LTC7871_IDAC_MIN		-64

DECLARE_CRC8_TABLE(ltc7871_crc8_table);

struct ltc7871 {
	struct spi_device *spi;
	struct regulator_dev *rdev;
	bool enable_chip_ctrl_wp;
	bool regulator_mode;
	u32 ra_ext;
	u32 rb_ext;
	u32 rc_ext;
	u32 rd_ext;
	u32 r1;
	u32 r2;
	u32 max_vol;
	u32 min_vol;
	s32 idac_setcur_uA;
	u32 freq_spread_range;
	u32 modulation_signal_freq;
};

static const unsigned int ltc7871_freq_spread_range[] = {
	12,
	15,
	10,
	8,
};

static const unsigned int ltc7871_modulation_signal_freq[] = {
	512,
	1024,
	2048,
	4096,
	256,
	128,
	64,
};

static int ltc7871_reg_read(struct spi_device *spi, u8 reg, int *val)
{
	int ret;
	struct spi_transfer t;
	u8 rx_buf[LTC7871_FRAME_SIZE] = {0};
	u8 tx_buf[LTC7871_FRAME_SIZE] = {0};

	tx_buf[0] = reg << 1 | 1;

	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = LTC7871_FRAME_SIZE;

	ret = spi_sync_transfer(spi, &t, 1);
	if (ret < 0)
		return ret;
	if (rx_buf[LTC7871_CRC_POS] != crc8(ltc7871_crc8_table, rx_buf,
				LTC7871_CRC_POS, LTC7871_CRC_INIT))
		return -EIO;

	return 0;
}

static int ltc7871_reg_write(struct spi_device *spi, u8 reg, int val)
{
	int ret;
	struct spi_transfer t;
	u8 rx_buf[LTC7871_FRAME_SIZE] = {0};
	u8 tx_buf[LTC7871_FRAME_SIZE] = {0};

	tx_buf[0] = reg << 1;
	tx_buf[1] = val;
	tx_buf[2] = crc8(ltc7871_crc8_table, tx_buf, LTC7871_CRC_POS, LTC7871_CRC_INIT);

	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = LTC7871_FRAME_SIZE;

	ret = spi_sync_transfer(spi, &t, 1);
	if (ret < 0)
		return ret;
	return 0;
}

static int ltc7871_get_error_flags(struct regulator_dev *rdev,
				    unsigned int *flags)
{
	u32 val;
	int ret;
	struct ltc7871 *ltc7871 = rdev_get_drvdata(rdev);

	ret = ltc7871_reg_read(ltc7871->spi, LTC7871_REG_FAULT, &val);
	if (ret)
		return ret;

	*flags = 0;

	if (FIELD_GET(LTC7871_FAULT_VHIGH_OV, val) ||
	    FIELD_GET(LTC7871_FAULT_VLOW_OV, val))
		*flags |= REGULATOR_ERROR_OVER_VOLTAGE_WARN;

	if (FIELD_GET(LTC7871_FAULT_VHIGH_UV, val))
		*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;

	if (FIELD_GET(LTC7871_FAULT_OVER_TEMP, val))
		*flags |= REGULATOR_ERROR_OVER_TEMP;

	return 0;
}

static s64 _ltc7871_dac_to_uV(u32 dac_val, int r1, int r2)
{
	return (1200 * (1000 + (div_s64(r2 * 1000, r1)))) - (dac_val * r2);
}

static s64 _ltc7871_uV_to_dac(s32 uV, int r1, int r2)
{
	s64 tmp;

	tmp = (1200 * (1000 + (div_s64(r2 * 1000, r1)))) - uV;
	tmp = div_s64(tmp, r2);

	return tmp;
}

static int ltc7871_set_voltage_sel(struct regulator_dev *rdev,
				    unsigned int sel)
{
	int reg;
	int addr;
	struct ltc7871 *ltc7871 = rdev_get_drvdata(rdev);

	if (sel < ltc7871->min_vol || sel > ltc7871->max_vol)
		return -EINVAL;

	if (ltc7871->regulator_mode)
		addr = LTC7871_REG_IDAC_VLOW;
	else
		addr = LTC7871_REG_IDAC_VHIGH;

	reg = _ltc7871_uV_to_dac(sel, ltc7871->r1, ltc7871->r2);

	return ltc7871_reg_write(ltc7871->spi, addr, reg);
}

static int ltc7871_get_voltage_sel(struct regulator_dev *rdev)
{
	int reg, ret;
	int addr;
	struct ltc7871 *ltc7871 = rdev_get_drvdata(rdev);

	if (ltc7871->regulator_mode)
		addr = LTC7871_REG_IDAC_VLOW;
	else
		addr = LTC7871_REG_IDAC_VHIGH;

	ret = ltc7871_reg_read(ltc7871->spi, addr, &reg);
	if (ret < 0)
		return ret;

	return _ltc7871_dac_to_uV(reg, ltc7871->r1, ltc7871->r2);
}

static int ltc7871_get_prop_index(const u32 *table, size_t table_size, u32 value)
{
	int i;

	for (i = 0; i < table_size; i++)
		if (table[i] == value)
			return i;

	return -EINVAL;
}

static int ltc7871_parse_fw(struct ltc7871 *chip)
{
	int reg, ret;
	int val1, val2;

	chip->idac_setcur_uA = 16;
	chip->freq_spread_range = 12;
	chip->modulation_signal_freq = 512;
	chip->enable_chip_ctrl_wp = 0;
	chip->ra_ext = 10000;
	chip->rb_ext = 107000;
	chip->rc_ext = 12700;
	chip->rd_ext = 499000;

	device_property_read_u32(&chip->spi->dev, "adi,ra-external-ohm", &chip->ra_ext);
	device_property_read_u32(&chip->spi->dev, "adi,rb-external-ohm", &chip->rb_ext);
	device_property_read_u32(&chip->spi->dev, "adi,rc-external-ohm", &chip->rc_ext);
	device_property_read_u32(&chip->spi->dev, "adi,rd-external-ohm", &chip->rd_ext);

	ret = ltc7871_reg_read(chip->spi, LTC7871_REG_CONFIG2, &reg);
	if (ret < 0)
		return ret;
	chip->regulator_mode = FIELD_GET(LTC7871_CONFIG2_BUCK_BOOST, reg);

	if (chip->regulator_mode) {
		chip->r1 = chip->ra_ext;
		chip->r2 = chip->rb_ext;
	} else {
		chip->r1 = chip->rc_ext;
		chip->r2 = chip->rd_ext;
	}
	chip->min_vol = _ltc7871_dac_to_uV(LTC7871_IDAC_MAX, chip->r1, chip->r2);
	chip->max_vol = _ltc7871_dac_to_uV(LTC7871_IDAC_MIN, chip->r1, chip->r2);

	ret = ltc7871_reg_read(chip->spi, LTC7871_REG_CHIP_CTRL, &reg);
	if (ret < 0)
		return ret;
	chip->enable_chip_ctrl_wp = device_property_read_bool(&chip->spi->dev, "adi,enable-chip-ctrl-wp");
	val1 = (FIELD_PREP(LTC7871_MASK_CHIP_CTRL_WP, chip->freq_spread_range)) | reg;
	ret = ltc7871_reg_write(chip->spi, LTC7871_REG_CHIP_CTRL, val1);
	if (ret)
		return ret;

	device_property_read_u32(&chip->spi->dev, "adi,idac-setcur-microamp", &chip->idac_setcur_uA);
	ret = ltc7871_reg_write(chip->spi, LTC7871_REG_SETCUR, chip->idac_setcur_uA);
	if (ret)
		return ret;

	ret = device_property_read_u32(&chip->spi->dev, "adi,freq-spread-range",
				       &chip->freq_spread_range);
	if (!ret) {
		ret = ltc7871_get_prop_index(ltc7871_freq_spread_range,
					    ARRAY_SIZE(ltc7871_freq_spread_range),
					    chip->freq_spread_range);
		if (ret < 0)
			return ret;
	}
	ret = device_property_read_u32(&chip->spi->dev, "adi,modulation-signal-freq", &chip->modulation_signal_freq);
	if (!ret) {
		ret = ltc7871_get_prop_index(ltc7871_modulation_signal_freq,
					    ARRAY_SIZE(ltc7871_modulation_signal_freq),
					    chip->modulation_signal_freq);
		if (ret < 0)
			return ret;
	}
	val1 = FIELD_PREP(LTC7871_MASK_SSFM_FREQ_SPREAD, chip->freq_spread_range);
	val2 = FIELD_PREP(LTC7871_MASK_SSFM_MOD_SIG_FREQ, chip->modulation_signal_freq);

	return ltc7871_reg_write(chip->spi, LTC7871_REG_SSFM, val1 | val2);
}

static const struct regulator_ops ltc7871_regulator_ops = {
	.set_voltage_sel = ltc7871_set_voltage_sel,
	.get_voltage_sel = ltc7871_get_voltage_sel,
	.get_error_flags = ltc7871_get_error_flags,
};

static struct regulator_desc ltc7871_regulator_desc = {
	.ops = &ltc7871_regulator_ops,
	.name = "ltc7871",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
};

static int ltc7871_probe(struct spi_device *spi)
{
	struct regulator_init_data *init_data;
	struct device *dev = &spi->dev;
	struct ltc7871 *chip;

	init_data = of_get_regulator_init_data(dev, spi->dev.of_node,
					       &ltc7871_regulator_desc);
	if (!init_data)
		return -EINVAL;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	crc8_populate_msb(ltc7871_crc8_table, LTC7871_CRC8_POLY);

	chip->spi = spi;

	return ltc7871_parse_fw(chip);
}

static const struct spi_device_id ltc7871_id[] = {
	{"ltc7871" },
	{"ltc7872" },
	{ },
};
MODULE_DEVICE_TABLE(spi, ltc7871_id);

static struct spi_driver ltc7871_driver = {
	.driver = {
		.name = "ltc7871"
	},
	.probe = ltc7871_probe,
	.id_table = ltc7871_id,
};
module_spi_driver(ltc7871_driver);

MODULE_DESCRIPTION("LTC7871 Voltage Regulator Driver");
MODULE_AUTHOR("Celine Joy Capua <celinejoy.capua@analog.com>");
MODULE_LICENSE("GPL");
