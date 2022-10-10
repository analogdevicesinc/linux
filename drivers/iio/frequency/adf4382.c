// SPDX-License-Identifier: GPL-2.0-only
/*
 * adf4382 driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>

/* ADF4382 REG0000 Map */
#define ADF4382_SOFT_RESET_R_MSK	BIT(7)
#define ADF4382_LSB_FIRST_R_MSK		BIT(6)
#define ADF4382_ADDRESS_ASC_R_MSK	BIT(5)
#define ADF4382_SDO_ACTIVE_R_MSK	BIT(4)
#define ADF4382_SDO_ACTIVE_MSK		BIT(3)
#define ADF4382_ADDRESS_ASC_MSK		BIT(2)
#define ADF4382_LSB_FIRST_MSK		BIT(1)
#define ADF4382_SOFT_RESET_MSK		BIT(0)

/* ADF4382 REG0000 Bit Definition */
#define ADF4382_SDO_ACTIVE_SPI_3W	0x0
#define ADF4382_SDO_ACTIVE_SPI_4W	0x1

#define ADF4382_ADDR_ASC_AUTO_DECR	0x0
#define ADF4382_ADDR_ASC_AUTO_INCR	0x1

#define ADF4382_LSB_FIRST_MSB		0x0
#define ADF4382_LSB_FIRST_LSB		0x1

#define ADF4382_SOFT_RESET_N_OP		0x0
#define ADF4382_SOFT_RESET_EN		0x1

/* ADF4382 REG0001 Map */
#define ADF4382_SINGLE_INSTR_MSK	BIT(7)
#define ADF4382_MASTER_RB_CTRL_MSK	BIT(5)

/* ADF4382 REG0001 Bit Definition */
#define ADF4382_SPI_STREAM_EN		0x0
#define ADF4382_SPI_STREAM_DIS		0x1

#define ADF4382_RB_SLAVE_REG		0x0
#define ADF4382_RB_MASTER_REG		0x1

/* ADF4382 REG0003 Bit Definition */
#define ADF4382_CHIP_TYPE		0x06

/* ADF4382 REG0004 Bit Definition */
#define ADF4382_PRODUCT_ID_LSB		0x0005

/* ADF4382 REG0005 Bit Definition */
#define ADF4382_PRODUCT_ID_MSB		0x0005

/* ADF4382 REG000A Map */
#define ADF4382_SCRATCHPAD_MSK		GENMASK(7, 0)

/* ADF4382 REG000C Bit Definition */
#define ADF4382_VENDOR_ID_LSB		0x56

/* ADF4382 REG000D Bit Definition */
#define ADF4382_VENDOR_ID_MSB		0x04

/* ADF4382 REG000F Bit Definition */
#define ADF4382_M_S_TRANSF_BIT_MSK	BIT(0)

/* ADF4382 REG0010 Map*/
#define ADF4382_N_INT_LSB_MSK		GENMASK(7, 0)

/* ADF4382 REG0011 Map*/
#define ADF4382_CLKOUT_DIV_MSK		GENMASK(7, 5)
#define ADF4382_INV_CLK_OUT_MSK		BIT(4)
#define ADF4382_N_INT_MSB_MSK		GENMASK(3, 0)

/* ADF4382 REG0012 Map */
#define ADF4382_FRAC1WORD_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0013 Map */
#define ADF4382_FRAC1WORD_MID1_MSK	GENMASK(7, 0)

/* ADF4382 REG0014 Map */
#define ADF4382_FRAC1WORD_MID2_MSK	GENMASK(7, 0)

/* ADF4382 REG0015 Map */
#define ADF4382_M_VCO_BAND_LSB_MSK	BIT(7)
#define ADF4382_M_VCO_CORE_MSK		BIT(6)
#define ADF4382_BIAS_DEC_MODE_MSK	GENMASK(5, 3)
#define ADF4382_INT_MODE_MSK		BIT(2)
#define ADF4382_PFD_POL_MSK		BIT(1)
#define ADF4382_FRAC1WORD_MSB_MSK	BIT(0)

/* ADF4382 REG0016 Map */
#define ADF4382_M_VCO_BAND_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0017 Map */
#define ADF4382_FRAC2WORD_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0018 Map */
#define ADF4382_FRAC2WORD_MID_MSK	GENMASK(7, 0)

/* ADF4382 REG0019 Map */
#define ADF4382_FRAC2WORD_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG001A Map */
#define ADF4382_MOD2WORD_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG001B Map */
#define ADF4382_MOD2WORD_MID_MSK	GENMASK(7, 0)

/* ADF4382 REG001C Map */
#define ADF4382_MOD2WORD_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG001D Map */
#define ADF4382_BLEED_I_LSB_MSK		GENMASK(7, 0)

/* ADF4382 REG001E Map */
#define ADF4382_EN_PHASE_RESYNC_MSK	BIT(7)
#define ADF4382_EN_REF_RST_MSK		BIT(6)
#define ADF4382_TIMED_SYNC_MSK		BIT(5)
#define ADF4382_BLEED_I_MSB_MSK		GENMASK(4, 0)

/* ADF4382 REG001F Map */
#define ADF4382_SW_SYNC_MSK		BIT(7)
#define ADF4382_SPARE_1F_MSK		BIT(6)
#define ADF4382_BLEED_POL_MSK		BIT(5)
#define ADF4382_EN_BLEED_MSK		BIT(4)
#define ADF4382_CP_I_MSK		GENMASK(3, 0)

/* ADF4382 REG0020 Map */
#define ADF4382_EN_AUTOCAL_MSK		BIT(7)
#define ADF4382_EN_RDBLR_MSK		BIT(6)
#define ADF4382_R_DIV_MSK		GENMASK(5, 0)

/* ADF4382 REG0021 Map */
#define ADF4382_PHASE_WORD_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0022 Map */
#define ADF4382_PHASE_WORD_MID_MSK	GENMASK(7, 0)

/* ADF4382 REG0023 Map */
#define ADF4382_PHASE_WORD_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0024 Map */
#define ADF4382_SPARE_24_MSK		GENMASK(7, 5)
#define ADF4382_DCLK_DIV_SEL_MSK	BIT(4)
#define ADF4382_DNCLK_DIV1_MSK		GENMASK(3, 2)
#define ADF4382_DCLK_DIV1_MSK		GENMASK(1, 0)

/* ADF4382 REG0025 Map */
#define ADF4382_RESYNC_WAIT_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0026 Map */
#define ADF4382_RESYNC_WAIT_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0027 Map */
#define ADF4382_CAL_BLEED_FINE_MIN_MSK	GENMASK(7, 4)
#define ADF4382_BLEED_ADJ_SCALE_MSK	GENMASK(3, 0)

/* ADF4382 REG0028 Map */
#define ADF4382_PH_RESYNC_RB_SEL_MSK	BIT(7)
#define ADF4382_LSB_P1_MSK		BIT(6)
#define ADF4382_VAR_MOD_EN_MSK		BIT(5)
#define ADF4382_DITHER1_SCALE_MSK	GENMASK(4, 2)
#define ADF4382_EN_DITHER2_MSK		BIT(1)
#define ADF4382_EN_DITHER1_MSK		BIT(0)

/* ADF4382 REG0029 Map */
#define ADF4382_CLK2_OPWR_MSK		GENMASK(7, 4)
#define ADF4382_CLK1_OPWR		GENMASK(3, 0)

/* ADF4382 REG002A Map */
#define ADF4382_FN_DBL_MSK		BIT(7)
#define ADF4382_PD_NDIV_MSK		BIT(6)
#define ADF4382_CLKOUT_BST_MSK		BIT(5)
#define ADF4382_PD_SYNC_MSK		BIT(4)
#define ADF4382_PD_CLK_MSK		BIT(3)
#define ADF4382_PD_RDET_MSK		BIT(2)
#define ADF4382_PD_ADC_MSK		BIT(1)
#define ADF4382_PD_CALGEN_MSK		BIT(0)

/* ADF4382 REG002B Map */
#define ADF4382_PD_ALL_MSK		BIT(7)
#define ADF4382_PD_RDIV_TL_MSK		BIT(6)
#define ADF4382_PD_NDIV_MSK		BIT(5)
#define ADF4382_PD_VCO_MSK		BIT(4)
#define ADF4382_PD_LD_MSK		BIT(3)
#define ADF4382_PD_PFDCP_MSK		BIT(2)
#define ADF4382_PD_CLKOUT1_MSK		BIT(1)
#define ADF4382_PD_CLKOUT2_MSK		BIT(0)

/* ADF4382 REG002C Map */
#define ADF4382_LDWIN_PW_MSK		GENMASK(7, 4)
#define ADF4382_LD_COUNT_OPWR_MSK	GENMASK(3, 0)

/* ADF4382 REG002D Map */
#define ADF4382_EN_DNCLK_MSK		BIT(7)
#define ADF4382_EN_DRCLK_MSK		BIT(6)
#define ADF4382_EN_LOL_MSK		BIT(5)
#define ADF4382_EN_LDWIN_MSK		BIT(4)
#define ADF4382_PDET_POL_MSK		BIT(3)
#define ADF4382_RST_LD_MSK		BIT(2)
#define ADF4382_LD_O_CTRL_MSK		GENMASK(1, 0)

/* ADF4382 REG002E Map */
#define ADF4382_MUXOUT_MSK		GENMASK(7, 4)
#define ADF4382_ABPW_WD_MSK		BIT(3)
#define ADF4382_EN_CPTEST_MSK		BIT(2)
#define ADF4382_CP_DOWN_MSK		BIT(1)
#define ADF4382_CP_UP_MSK		BIT(0)

/* ADF4382 REG002F Map*/
#define ADF4382_BST_REF_MSK		BIT(7)
#define ADF4382_FILT_REF_MSK		BIT(6)
#define ADF4382_RDBLR_DC_MSK		GENMASK(5, 0)

/* ADF4382 REG0030 Map */
#define ADF4382_MUTE_NCLK_MSK		BIT(7)
#define ADF4382_MUTE_RCLK_MSK		BIT(6)
#define ADF4382_REF_SEL_MSK		BIT(5)
#define ADF4382_INV_RDBLR_MSK		BIT(4)
#define ADF4382_RDBLR_DEL_SEL_MSK	GENMASK(3, 0)

/* ADF4382 REG0031 Map */
#define ADF4382_SYNC_DEL_MSK		GENMASK(7, 5)
#define ADF4382_RST_SYS_MSK		BIT(4)
#define ADF4382_EN_ADC_CLK_MSK		BIT(3)
#define ADF4382_EN_VCAL_MSK		BIT(2)
#define ADF4382_CAL_CT_SEL_MSK		BIT(1)
#define ADF4382_DCLK_MODE_MSK		BIT(0)

/* ADF4382 REG0032 Map */
#define ADF4382_SPARE_32_MSK		BIT(7)
#define ADF4382_BLEED_ADJ_CAL_MSK	BIT(6)
#define ADF4382_DEL_MODE_MSK		BIT(5)
#define ADF4382_EN_AUTO_ALIGN_MSK	BIT(4)
#define ADF4382_PHASE_ADJ_POL_MSK	BIT(3)
#define ADF4382_EFM3_MODE_MSK		GENMASK(2, 0)

/* ADF4382 REG0033 Map */
#define ADF4382_PHASE_ADJUST_MSK	GENMASK(7, 0)

/* ADF4382 REG0034 Map */
#define ADF4382_PHASE_ADJ_MSK		BIT(7)
#define ADF4382_DRCLK_DEL_MSK		GENMASK(6, 4)
#define ADF4382_DNCLK_DEL_MSK		GENMASK(3, 1)
#define ADF4382_RST_CNTR_MSK		BIT(0)

/* ADF4382 REG0035 Map */
#define ADF4382_SPARE_35_MSK		GENMASK(7, 6)
#define ADF4382_M_VCO_BIAS_MSK		GENMASK(5, 0)

/* ADF4382 REG0036 Map */
#define ADF4382_CLKODIV_DB_MSK		BIT(7)
#define ADF4382_DCLK_DIV_DB_MSK		BIT(6)
#define ADF4382_SPARE_36_MSK		GENMASK(5, 2)
#define ADF4382_EN_LUT_GEN_MSK		BIT(1)
#define ADF4382_EN_LUT_CAL_MSK		BIT(0)

/* ADF4382 REG0037 Map */
#define ADF4382_CAL_COUNT_TO_MSK	GENMASK(7, 0)

/* ADF4382 REG0038 Map */
#define ADF4382_CAL_VTUNE_TO_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0039 Map */
#define ADF4382_O_VCO_DB_MSK		BIT(7)
#define ADF4382_CAL_VTUNE_TO_MSB_MSK	GENMASK(6, 0)

/* ADF4382 REG003A Map */
#define ADF4382_CAL_VCO_TO_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG003B Map */
#define ADF4382_DEL_CTRL_DB_MSK		BIT(7)
#define ADF4382_CAL_VCO_TO_MSB_MSK	GENMASK(6, 0)

/* ADF4382 REG003C Map */
#define ADF4382_CNTR_DIV_WORD_MSK	GENMASK(7, 0)

/* ADF4382 REG003D Map */
#define ADF4382_SPARE_3D_MSK		BIT(7)
#define ADF4382_SYNC_SP_DB_MSK		BIT(6)
#define ADF4382_CMOS_OV_MSK		BIT(5)
#define ADF4382_READ_MODE_MSK		BIT(4)
#define ADF4382_CNTR_DIV_WORD_MSB_MSK	GENMASK(3, 0)

/* ADF4382 REG003E Map */
#define ADF4382_ADC_CLK_DIV_MSK		GENMASK(7, 0)

/* ADF4382 REG003F Map */
#define ADF4382_EN_ADC_CNV_MSK		BIT(7)
#define ADF4382_EN_ADC_VTEST_MSK	BIT(6)
#define ADF4382_ADC_VTEST_SEL_MSK	BIT(5)
#define ADF4382_ADC_MUX_SEL_MSK		BIT(4)
#define ADF4382_ADC_F_CONV_MSK		BIT(3)
#define ADF4382_ADC_C_CONV_MSK		BIT(2)
#define ADF4382_EN_ADC_MSK		BIT(1)
#define ADF4382_SPARE_3F_MSK		BIT(0)

/* ADF4382 REG0040 Map */
#define ADF4382_EXT_DIV_DEC_SEL_MSK	BIT(7)
#define ADF4382_ADC_CLK_TEST_SEL_MSK	BIT(6)
#define ADF4382_MUTE_CLKOUT2_MSK	GENMASK(5, 3)
#define ADF4382_MUTE_CLKOUT1_MSK	GENMASK(2, 0)

/* ADF4382 REG0041 Map */
#define ADF4382_EXT_DIV_MSK		GENMASK(7, 5)
#define ADF4382_EN_VCO_CAP_TEST_MSK	BIT(4)
#define ADF4382_EN_CALGEN_CAP_TEST_MSK	BIT(3)
#define ADF4382_EN_CP_CAP_TEST_MSK	BIT(2)
#define ADF4382_CAP_TEST_STATE_MSK	BIT(1)
#define ADF4382_TRANS_LOOP_SEL_MSK	BIT(0)

/* ADF4382 REG0042 Map */
#define ADF4382_NDIV_PWRUP_TIMEOUT_MSK	GENMASK(7, 0)

/* ADF4382 REG0043 Map */
#define ADF4382_CAL_BLEED_FINE_MAX_MSK	GENMASK(7, 0)

/* ADF4382 REG0044 Map */
#define ADF4382_VCAL_ZERO_MSK		BIT(7)
#define ADF4382_VPTAT_CALGEN_MSK	GENMASK(6, 0)

/* ADF4382 REG0045 Map */
#define ADF4382_SPARE_45_MSK		BIT(7)
#define ADF4382_VCTAT_CALGEN_MSK	GENMASK(6, 0)

/* ADF4382 REG0046 Map */
#define ADF4382_NVMDIN_MSK		GENMASK(7, 0)

/* ADF4382 REG0047 Map */
#define ADF4382_SPARE_47_MSK		BIT(7)
#define ADF4382_NVMADDR_MSK		GENMASK(6, 3)
#define ADF4382_NVMBIT_SEL		GENMASK(2, 0)

/* ADF4382 REG0048 Map */
#define ADF4382_TRIM_LATCH_MSK		BIT(7)
#define ADF4382_NVMTEST_MSK		BIT(6)
#define ADF4382_NVMPROG_MSK		BIT(5)
#define ADF4382_NVMRD_MSK		BIT(4)
#define ADF4382_NVMSTART_MSK		BIT(3)
#define ADF4382_NVMON_MSK		BIT(2)
#define ADF4382_MARGIN_MSK		GENMASK(1, 0)

/* ADF4382 REG0049 Map */
#define ADF4382_NVMDOUT_MSK		GENMASK(7, 0)

/* ADF4382 REG004A Map */
#define ADF4382_SCAN_MODE_CODE_MSK	GENMASK(7, 0)

/* ADF4382 REG004B Map */
#define ADF4382_TEMP_OFFSET_MSK		GENMASK(7, 0)

/* ADF4382 REG004C Map */
#define ADF4382_SPARE_4C_MSK		GENMASK(7, 6)
#define ADF4382_TEMP_SLOPE_MSK		GENMASK(5, 0)

/* ADF4382 REG004D Map */
#define ADF4382_VCO_FSM_TEST_MUX_MSK	GENMASK(7, 5)
#define ADF4382_SPARE_4D_MSK		GENMASK(4, 3)
#define ADF4382_O_VCO_BIAS_MSK		BIT(2)
#define ADF4382_O_VCO_BAND_MSK		BIT(1)
#define ADF4382_O_VCO_CORE_MSK		BIT(0)

/* ADF4382 REG004E Map */
#define ADF4382_VCO_FSM_TEST_MUX_MSK	GENMASK(7, 5)
#define ADF4382_O_VCO_BIAS_MSK		BIT(4)
#define ADF4382_SPARE_4D_MSK		GENMASK(3, 0)

/* ADF4382 REG004F Map */
#define ADF4382_LUT_SCALE_MSK		GENMASK(7, 0)

/* ADF4382 REG0050 Map */
#define ADF4382_SPARE0_MSK		GENMASK(7, 0)

/* ADF4382 REG0051 Map */
#define ADF4382_SPARE1_MSK		GENMASK(7, 0)

/* ADF4382 REG0052 Map */
#define ADF4382_SYNC_REF_SPARE_MSK	GENMASK(7, 4)
#define ADF4382_SYNC_MON_DEL_MSK	GENMASK(3, 0)

/* ADF4382 REG0053 Map */
#define ADF4382_SPARE_35_MSK		BIT(7)
#define ADF4382_PD_SYNC_MON_MSK		BIT(6)
#define ADF4382_SYNC_SEL_MSK		BIT(5)
#define ADF4382_RST_SYNC_MON_MSK	BIT(4)
#define ADF4382_SYNC_SH_DEL_MSK		GENMASK(3, 0)

/* ADF4382 REG0054 Map */
#define ADF4382_ADC_ST_CNV_MSK		BIT(0)

enum {
	ADF4382_FREQ,
};

struct adf4382_state {
	struct spi_device	*spi;
	struct regmap		*regmap;
	struct clk		*clkin;
	/* Protect against concurrent accesses to the device and data content */
	struct mutex		lock;
	struct notifier_block	nb;
};

static const struct regmap_config adf4382_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
	.max_register = 0x54,
};

static int adf4382_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int write_val,
			      unsigned int *read_val)
{
	struct adf4382_state *st = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(st->regmap, reg, read_val);

	return regmap_write(st->regmap, reg, write_val);
}

static const struct iio_info adf4382_info = {
	.debugfs_reg_access = &adf4382_reg_access,
};

int adf4382_set_freq(struct adf4382_state *st, u64 freq)
{
	return 0;
}

int adf4382_get_freq(struct adf4382_state *st, u64 *freq)
{
	return 0;
}

static ssize_t adf4382_write(struct iio_dev *indio_dev, uintptr_t private,
			     const struct iio_chan_spec *chan, const char *buf,
			     size_t len)
{
	struct adf4382_state *st = iio_priv(indio_dev);
	unsigned long long freq;
	int ret;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case ADF4382_FREQ:
		ret = kstrtoull(buf, 10, &freq);
		if (ret)
			break;

		ret = adf4382_set_freq(st, freq);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t adf4382_read(struct iio_dev *indio_dev, uintptr_t private,
			    const struct iio_chan_spec *chan, char *buf)
{
	struct adf4382_state *st = iio_priv(indio_dev);
	u64 val = 0;
	int ret;

	switch ((u32)private) {
	case ADF4382_FREQ:
		ret = adf4382_get_freq(st, &val);
		break;
	default:
		ret = -EINVAL;
		val = 0;
		break;
	}

	return ret ?: sysfs_emit(buf, "%llu\n", val);
}

#define _ADF4382_EXT_INFO(_name, _shared, _ident) { \
		.name = _name, \
		.read = adf4382_read, \
		.write = adf4382_write, \
		.private = _ident, \
		.shared = _shared, \
	}

static const struct iio_chan_spec_ext_info adf4382_ext_info[] = {
	/*
	 * Usually we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz.
	 */
	_ADF4382_EXT_INFO("frequency", IIO_SHARED_BY_ALL, ADF4382_FREQ),
	{ },
};

static const struct iio_chan_spec adf4382_channels[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.ext_info = adf4382_ext_info,
	},
};

static int adf4382_init(struct adf4382_state *st)
{
	return 0;
}

static int adf4382_freq_change(struct notifier_block *nb, unsigned long action, void *data)
{
	struct adf4382_state *st = container_of(nb, struct adf4382_state, nb);
	int ret;

	if (action == POST_RATE_CHANGE) {
		mutex_lock(&st->lock);
		ret = notifier_from_errno(adf4382_init(st));
		mutex_unlock(&st->lock);
		return ret;
	}

	return NOTIFY_OK;
}

static void adf4382_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void adf4382_clk_notifier_unreg(void *data)
{
	struct adf4382_state *st = data;

	clk_notifier_unregister(st->clkin, &st->nb);
}

static int adf4382_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct adf4382_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &adf4382_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	st = iio_priv(indio_dev);

	indio_dev->info = &adf4382_info;
	indio_dev->name = "adf4382";
	indio_dev->channels = adf4382_channels;
	indio_dev->num_channels = ARRAY_SIZE(adf4382_channels);

	st->regmap = regmap;
	st->spi = spi;

	mutex_init(&st->lock);

	ret = clk_prepare_enable(st->clkin);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adf4382_clk_disable, st->clkin);
	if (ret)
		return ret;

	st->nb.notifier_call = adf4382_freq_change;
	ret = clk_notifier_register(st->clkin, &st->nb);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adf4382_clk_notifier_unreg, st);
	if (ret)
		return ret;

	ret = adf4382_init(st);
	if (ret) {
		dev_err(&spi->dev, "adf4382 init failed\n");
		return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id adf4382_id[] = {
	{ "adf4382", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, adf4382_id);

static const struct of_device_id adf4382_of_match[] = {
	{ .compatible = "adi,adf4382" },
	{},
};
MODULE_DEVICE_TABLE(of, adf4382_of_match);

static struct spi_driver adf4382_driver = {
	.driver = {
		.name = "adf4382",
		.of_match_table = adf4382_of_match,
	},
	.probe = adf4382_probe,
	.id_table = adf4382_id,
};
module_spi_driver(adf4382_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF4382");
MODULE_LICENSE("GPL v2");
