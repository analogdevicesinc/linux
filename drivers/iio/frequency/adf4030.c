// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ADF4030 10-Channel Precision Synchronizer IC.
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clk/clkscale.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/jesd204/jesd204.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/units.h>
#include <linux/spi/spi.h>

/* REG 0x00 */
#define ADF4030_ADDRESS_ASCENSION_MSK		BIT(2)
#define ADF4030_SDO_ACTIVE_MSK			BIT(3)
#define ADF4030_SDO_ACTIVE_R_MSK		BIT(4)
#define ADF4030_ADDRESS_ASCENSION_R_MSK		BIT(5)

/* REG 0x10 */
#define ADF4030_TDC_TARGET_MSK		GENMASK(4, 0)

/* REG 0x11 */
#define ADF4030_TDC_SOURCE_MSK		GENMASK(4, 0)
#define ADF4030_EN_ALIGN_MSK		BIT(6)
#define ADF4030_MANUAL_MODE_MSK		BIT(7)

/* REG 0x15 */
#define ADF4030_FALL_EDGE_TRG_MSK		BIT(6)
#define ADF4030_FALL_EDGE_SRC_MSK		BIT(7)

/* REG 0x16 */
#define ADF4030_AVGEXP_MSK	GENMASK(3, 0)
#define ADF4030_TDC_ARM_M_MSK	BIT(7)

/* REG 0x17 */
#define ADF4030_NDEL_ADJ_MSK	BIT(7)
#define ADF4030_STOP_FSM_MSK	BIT(6)
#define ADF4030_ADEL_MSK	GENMASK(5, 0)

/* REG 0x35 */
#define ADF4030_ALIGN_THOLD_MSK	GENMASK(5, 0)
#define ADF4030_BSYNC_CAL_ON_1_0_MSK	GENMASK(7, 6)

/* REG 0x37 */
#define ADF4030_EN_ITER_MSK		BIT(0)
#define ADF4030_EN_CYCS_RED_MSK		BIT(1)
#define ADF4030_EN_SERIAL_ALIGN_MSK	BIT(2)
#define ADF4030_EN_BKGND_ALGN_MSK	BIT(3)
#define ADF4030_ALIGN_CYCLES_MSK	GENMASK(7, 5)

/* REG 0x3C */
#define ADF4030_PD_ADC_MSK	BIT(4)
#define ADF4030_PD_TDC_MSK	BIT(5)
#define ADF4030_PD_PLL_MSK	BIT(6)
#define ADF4030_PD_ALL_MSK	BIT(7)

/* REG 0x3F ... REG 0x51*/
#define ADF4030_DR_CM0_ADJ_MSK	GENMASK(5, 0)
#define ADF4030_BST_MSK		BIT(6)
#define ADF4030_ODIV_SEL_MSK	BIT(7)

/* REG REG 0x40 ... REG 0x52 */
#define ADF4030_AC_COUPLED_MSK		BIT(1)
#define ADF4030_LINK_TX_MSK		BIT(2)
#define ADF4030_LINK_RX_MSK		BIT(3)
#define ADF4030_FLOAT_TX_MSK		BIT(4)
#define ADF4030_FLOAT_RX_MSK		BIT(5)
#define ADF4030_AUTO_PD_RCV_MSK		BIT(7)

/* REG 0x54 */
#define ADF4030_ODIVA_MSK		GENMASK(3, 0)
#define ADF4030_ODIVB_MSK		GENMASK(7, 4)

/* REG 0x57 */
#define ADF4030_RDIV_MASK		GENMASK(4, 0)

/* REG 0x58 */
#define ADF4030_DIVREF_MSK		GENMASK(5, 4)

/* REG 0x5A */
#define ADF4030_PLL_CAL_EN_MSK		BIT(6)

/* REG 0x61 */
#define ADF4030_EN_ADC_MSK		BIT(0)
#define ADF4030_EN_ADC_CLK_MSK		BIT(1)
#define ADF4030_EN_ADC_CNV_MSK		BIT(2)

/* REG 0x72 */
#define ADF4030_START_CNV	BIT(0)

/*
 * bit 0: enables the ADC
 * bit 1: enabled ADC Clock
 * bit 2: enables ADC conversion
 */
#define ADF4030_ADC_CFG_MASK		GENMASK(2, 0)

/* REG 0x8F */
#define ADF4030_FSM_BUSY_MSK		BIT(0)
#define ADF4030_ADC_BUSY_MSK		BIT(1)
#define ADF4030_MATH_BUSY_MSK		BIT(2)
#define ADF4030_DL_BUSY_MSK		BIT(3)
#define ADF4030_TDC_BUSY_MSK		BIT(4)
#define ADF4030_REF_OK_MSK		BIT(6)

/* REG 0x90 */
#define ADF4030_PLL_LOCK_MSK		BIT(0)
#define ADF4030_TDC_ERR_MSK		GENMASK(2, 1)
#define ADF4030_TMP_ALIGN_ERR		BIT(3)

/* REG 0xBA */
#define ADF4030_CAL_BUSY_MSK		BIT(0)

/* REG 0xFF*/
#define ADF4030_SOFTRESET_CHIP_MSK	BIT(0)

#define ADI_ADF4030_REF_FREQ_MIN	10000000U
#define ADI_ADF4030_REF_FREQ_MAX	250000000U
#define ADI_ADF4030_PFD_FREQ_MIN	10000000U
#define ADI_ADF4030_PFD_FREQ_MAX	20000000U
#define ADI_ADF4030_VCO_FREQ_MIN	2375000000U
#define ADI_ADF4030_VCO_FREQ_MAX	2625000000U
#define ADI_ADF4030_BSYNC_FREQ_MIN	650000U
#define ADI_ADF4030_BSYNC_FREQ_MAX	250000000U
#define ADI_ADF4030_R_DIV_MIN		1U
#define ADI_ADF4030_R_DIV_MAX		31U
#define ADI_ADF4030_N_DIV_MIN		8U
#define ADI_ADF4030_N_DIV_MAX		255U
#define ADI_ADF4030_O_DIV_MIN		10U
#define ADI_ADF4030_O_DIV_MAX		4095U

#define ADI_ADF4030_ALIGN_CYCLES_MIN_COUNT	(0U)
#define ADI_ADF4030_ALIGN_CYCLES_MAX_COUNT	(8U)
#define ADI_ADF4030_ADEL_M_STEP_IN_FEMTO_SEC	(1400U)
#define ADI_ADF4030_ADEL_M_MAX_STEP_SIZE	(0x3FU)
#define ADI_ADF4030_MAX_THRESHOLD_IN_FEMTO_SEC	\
	(ADI_ADF4030_ADEL_M_MAX_STEP_SIZE * ADI_ADF4030_ADEL_M_STEP_IN_FEMTO_SEC)

#define ADF4030_NUM_CHAN 10

#define ADF4030_REG(x)	(x)

#define ADF4030_CIC_DEC_RATE	0xF

struct adf4030_output {
	struct clk_hw hw;
	struct adf4030_state *st;
	unsigned int address;
};

struct adf4030_chan_spec {
	const char *extended_name;
	unsigned int reference_chan;
	unsigned int delay;
	unsigned int rcm;
	unsigned int num;
	bool channel_io_reconfig_en;
	bool channel_output_en;
	bool align_on_sync_en;
	bool ac_coupled;
	bool invert_en;
	bool odivb_en;
	bool float_tx;
	bool float_rx;
	bool boost_en;
	bool link_tx;
	bool link_rx;
};

struct adf4030_state {
	struct iio_chan_spec iio_channels[ADF4030_NUM_CHAN + 1];
	struct adf4030_output outputs[ADF4030_NUM_CHAN];
	const char *clk_out_names[ADF4030_NUM_CHAN];
	struct clk_hw_onecell_data *clk_data;
	struct adf4030_chan_spec *channels;
	struct jesd204_dev *jdev;
	struct spi_device *spi;
	struct regmap *regmap;
	struct clk *refin;
	/*
	 * lock to protect against concurrent accesses of the HW and device
	 * global variables.
	 */
	struct mutex lock;
	u32 ref_freq;
	u32 vco_freq;
	u32 bsync_autoalign_iter;
	u32 bsync_autoalign_theshold_fs;
	u32 bsync_autoalign_ref_chan;
	u32 bsync_freq_odiv_a;
	u32 bsync_freq_odiv_b;
	u32 avgexp;
	bool bsync_autoalign_theshold_en;
	unsigned int num_channels;
	bool adc_enabled;
	bool spi_3wire_en;
	bool bsync_autoalign_en;
	u32 bsync_autoalign_mask;

	u8 vals[3] __aligned(IIO_DMA_MINALIGN);
};

#define adf4030_to_output(_hw) container_of(_hw, struct adf4030_output, hw)

static const struct regmap_config adf4030_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static const struct reg_sequence adf4030_reg_default[] = {
	{0x6A, 0x0A}, {0x69, 0x0A}, {0x66, 0x80}, {0x64, 0x1E}, {0x63, 0x1E},
	{0x62, 0x4C}, {0x61, 0x05}, {0x60, 0x2B}, {0x5F, 0x5D}, {0x5E, 0x32},
	{0x5D, 0x10}, {0x5C, 0x1E}, {0x5B, 0xC9}, {0x5A, 0x17}, {0x59, 0x49},
	{0x58, 0x53}, {0x57, 0x45}, {0x56, 0x7D}, {0x55, 0x01}, {0x54, 0x90},
	{0x53, 0x19}, {0x52, 0xE9}, {0x50, 0xE9}, {0x4E, 0xE9}, {0x4C, 0xE9},
	{0x4A, 0xE9}, {0x48, 0xE9}, {0x46, 0xE9}, {0x44, 0xE9}, {0x42, 0xE9},
	{0x40, 0xE9}, {0x3C, 0xFF}, {0x3B, 0xFC}, {0x37, 0x02}, {0x35, 0x05},
	{0x34, 0x24}, {0x33, 0x1D}, {0x32, 0x1D}, {0x31, 0x45}, {0x16, 0x06},
	{0x11, 0x1F}, {0x10, 0x1F}
};

static int adf4030_compute_r_n(u32 ref_freq, u32 vco_freq, u32 *rdiv, u32 *ndiv)
{
	u32 pfd_freq, i, max_r, min_r;

	if (vco_freq < ADI_ADF4030_VCO_FREQ_MIN || vco_freq > ADI_ADF4030_VCO_FREQ_MAX)
		return -EINVAL;

	if (ref_freq < ADI_ADF4030_REF_FREQ_MIN || ref_freq > ADI_ADF4030_REF_FREQ_MAX)
		return -EINVAL;

	max_r = ref_freq /  ADI_ADF4030_PFD_FREQ_MIN;
	min_r = DIV_ROUND_UP(ref_freq,  ADI_ADF4030_PFD_FREQ_MAX);

	for (i = min_r; i <= max_r; i++) {
		pfd_freq = ref_freq / i;

		*ndiv = vco_freq / pfd_freq;

		if ((vco_freq % pfd_freq) == 0 && *ndiv <= ADI_ADF4030_N_DIV_MAX) {
			*rdiv = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int adf4030_compute_odiv(u32 vco_freq, u32 bsync_out_freq, u32 *odiv)
{
	if (bsync_out_freq < ADI_ADF4030_BSYNC_FREQ_MIN ||
	    bsync_out_freq > ADI_ADF4030_BSYNC_FREQ_MAX)
		return -EINVAL;

	if (vco_freq % bsync_out_freq)
		return -EINVAL;

	*odiv = vco_freq / bsync_out_freq;

	if (*odiv > ADI_ADF4030_O_DIV_MAX)
		return -EINVAL;

	return 0;
}

static int adf4030_set_odiva_freq(struct adf4030_state *st, u32 bsync_out_freq_hz)
{
	u32 odiv;
	int ret;

	ret = adf4030_compute_odiv(st->vco_freq, bsync_out_freq_hz, &odiv);
	if (ret) {
		dev_err(&st->spi->dev,
			"Failed to compute ODIVA for Fvco=%u Hz and Fbsync=%u Hz\n",
			st->vco_freq, bsync_out_freq_hz);
		return ret;
	}

	st->bsync_freq_odiv_a = bsync_out_freq_hz;

	ret = regmap_write(st->regmap, ADF4030_REG(0x53), odiv);
	if (ret)
		return ret;

	return regmap_update_bits(st->regmap, ADF4030_REG(0x54),
				 ADF4030_ODIVA_MSK,
				 FIELD_PREP(ADF4030_ODIVA_MSK, odiv >> 8));
}

static int adf4030_chan_dir_set(const struct adf4030_state *st,
				struct adf4030_chan_spec *chan, bool initial)
{
	u32 reg;
	int ret;

	if (initial) {
		reg = 0x3F + (chan->num * 2);

		ret = regmap_write(st->regmap, ADF4030_REG(reg),
				   FIELD_PREP(ADF4030_DR_CM0_ADJ_MSK, chan->rcm) |
				   FIELD_PREP(ADF4030_BST_MSK, chan->boost_en) |
				   FIELD_PREP(ADF4030_ODIV_SEL_MSK,  chan->odivb_en));
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, ADF4030_REG(reg + 1),
				   FIELD_PREP(ADF4030_AC_COUPLED_MSK, chan->ac_coupled) |
				   FIELD_PREP(ADF4030_LINK_TX_MSK, chan->link_tx) |
				   FIELD_PREP(ADF4030_LINK_RX_MSK, chan->link_rx) |
				   FIELD_PREP(ADF4030_FLOAT_TX_MSK, chan->float_tx) |
				   FIELD_PREP(ADF4030_FLOAT_RX_MSK, chan->float_rx) |
				   FIELD_PREP(ADF4030_AUTO_PD_RCV_MSK, 1));
		if (ret)
			return ret;

		/* CHAN_INVx */
		if (chan->invert_en) {
			if (chan->num > 3)
				ret = regmap_set_bits(st->regmap, ADF4030_REG(0x15),
						      BIT(chan->num - 4));
			else
				ret = regmap_set_bits(st->regmap, ADF4030_REG(0x14),
						      BIT(chan->num + 4));
			if (ret)
				return ret;
		}
	}

	/* EN_DRIVE */
	if (chan->num > 7)
		ret = regmap_update_bits(st->regmap, ADF4030_REG(0x13),
					 BIT(chan->num - 8),
					 chan->channel_output_en ? BIT(chan->num - 8) : 0);
	else
		ret = regmap_update_bits(st->regmap, ADF4030_REG(0x12),
					 BIT(chan->num),
					 chan->channel_output_en ? BIT(chan->num) : 0);
	if (ret)
		return ret;

	/* PD_TX_PATH */
	if (chan->num > 5)
		return regmap_clear_bits(st->regmap, ADF4030_REG(0x3C),
					 BIT(chan->num - 6));

	return regmap_clear_bits(st->regmap, ADF4030_REG(0x3B),
				 BIT(chan->num + 2));
}

static int adf4030_tdc_measure(struct adf4030_state *st, u32 channel,
			       u32 source_channel, u32 source_out_freq_hz,
			       s64 *result)
{
	u32 raw_time_diff, m_time, regval;
	int ret, time_diff;
	s64 res_in_fs = 0;

	ret = regmap_update_bits(st->regmap, ADF4030_REG(0x11),
				 ADF4030_TDC_SOURCE_MSK,
				 FIELD_PREP(ADF4030_TDC_SOURCE_MSK, source_channel));
	if (ret)
		return ret;

	ret = regmap_clear_bits(st->regmap, ADF4030_REG(0x15),
				ADF4030_FALL_EDGE_SRC_MSK | ADF4030_FALL_EDGE_TRG_MSK);
	if (ret)
		return ret;

	ret = regmap_set_bits(st->regmap, ADF4030_REG(0x11),
			      ADF4030_MANUAL_MODE_MSK);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADF4030_REG(0x10),
			   FIELD_PREP(ADF4030_TDC_TARGET_MSK, channel));
	if (ret)
		return ret;

	ret = regmap_set_bits(st->regmap, ADF4030_REG(0x16),
			      ADF4030_TDC_ARM_M_MSK);
	if (ret)
		return ret;

	m_time = DIV_ROUND_UP_ULL(1000000ULL * (1 << (st->avgexp + 6)),
				  source_out_freq_hz);

	fsleep(m_time);

	/* Wait for TDC and MATH finished */
	ret = regmap_read_poll_timeout(st->regmap, ADF4030_REG(0x8F),
				       regval, !(regval & (ADF4030_TDC_BUSY_MSK |
				       ADF4030_MATH_BUSY_MSK)),
				       2000, 10000);
	if (ret) {
		dev_err(&st->spi->dev, "TDC measurement failed TDC_BUSY\n");
		return ret;
	}

	ret = regmap_bulk_read(st->regmap, ADF4030_REG(0x73), st->vals, 3);
	if (ret)
		return ret;

	ret = regmap_clear_bits(st->regmap, ADF4030_REG(0x16),
				ADF4030_TDC_ARM_M_MSK);
	if (ret)
		return ret;

	raw_time_diff = (st->vals[2] << 16) | (st->vals[1] << 8) | st->vals[0];
	time_diff = sign_extend32(raw_time_diff, 23);
	res_in_fs = div_s64(time_diff * 1000000000LL, 1 << 24);
	res_in_fs = div_s64(res_in_fs * 1000000LL, source_out_freq_hz);

	*result = res_in_fs;

	return 0;
}

static int adf4030_duty_cycle_measure(struct adf4030_state *st, u32 channel,
				      u32 source_out_freq_hz, u64 *result)
{
	u32 raw_time_diff, m_time, regval;
	int ret;

	ret = regmap_update_bits(st->regmap, ADF4030_REG(0x11),
				 ADF4030_TDC_SOURCE_MSK,
				 FIELD_PREP(ADF4030_TDC_SOURCE_MSK, channel));
	if (ret)
		return ret;

	ret = regmap_set_bits(st->regmap, ADF4030_REG(0x15),
			      ADF4030_FALL_EDGE_SRC_MSK);
	if (ret)
		return ret;

	ret = regmap_set_bits(st->regmap, ADF4030_REG(0x11),
			      ADF4030_MANUAL_MODE_MSK);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADF4030_REG(0x10),
			   FIELD_PREP(ADF4030_TDC_TARGET_MSK, channel));
	if (ret)
		return ret;

	ret = regmap_set_bits(st->regmap, ADF4030_REG(0x16),
			      ADF4030_TDC_ARM_M_MSK);
	if (ret)
		return ret;

	m_time = DIV_ROUND_UP_ULL(1000000ULL * (1 << (st->avgexp + 6)),
				  source_out_freq_hz);

	fsleep(m_time);

	/* Wait for TDC and MATH finished */
	ret = regmap_read_poll_timeout(st->regmap, ADF4030_REG(0x8F),
				       regval, !(regval & (ADF4030_TDC_BUSY_MSK |
				       ADF4030_MATH_BUSY_MSK)),
				       2000, 10000);
	if (ret) {
		dev_err(&st->spi->dev, "TDC measurement failed TDC_BUSY\n");
		return ret;
	}

	ret = regmap_bulk_read(st->regmap, ADF4030_REG(0x73), st->vals, 3);
	if (ret)
		return ret;

	ret = regmap_clear_bits(st->regmap, ADF4030_REG(0x16),
				ADF4030_TDC_ARM_M_MSK);
	if (ret)
		return ret;

	raw_time_diff = (st->vals[2] << 16) | (st->vals[1] << 8) | st->vals[0];
	*result = div_u64(raw_time_diff * 100000000LL, 1 << 24);

	return 0;
}

static int adf4030_auto_align_delay(const struct adf4030_state *st, u32 channel,
				    s64 delay_in_fs)
{
	u64 vco_period_in_fs;
	u16 tdc_offset;
	int ret;

	/* see units.h */
	vco_period_in_fs = DIV_U64_ROUND_CLOSEST(1000000000000000ULL, st->vco_freq);
	vco_period_in_fs = DIV_U64_ROUND_CLOSEST(vco_period_in_fs, 512);
	tdc_offset = div64_s64(delay_in_fs, vco_period_in_fs);

	ret = regmap_write(st->regmap, ADF4030_REG(0x1D + (channel * 2)),
			   tdc_offset & 0xFF);
	if (ret)
		return ret;

	return regmap_write(st->regmap, ADF4030_REG(0x1E + (channel * 2)),
			    tdc_offset >> 8);
}

static int adf4030_auto_align_iteration_set(struct adf4030_state *st,
					    u8 iter_number)
{
	if (iter_number > ADI_ADF4030_ALIGN_CYCLES_MAX_COUNT)
		return -EINVAL;

	return regmap_update_bits(st->regmap, ADF4030_REG(0x37),
				 ADF4030_ALIGN_CYCLES_MSK,
				 FIELD_PREP(ADF4030_ALIGN_CYCLES_MSK, iter_number - 1));
}

static int adf4030_auto_align_threshold(const struct adf4030_state *st,
					u32 threshold_in_fs)
{
	u8 threshold;

	if (threshold_in_fs > ADI_ADF4030_MAX_THRESHOLD_IN_FEMTO_SEC)
		return -EINVAL;

	threshold = threshold_in_fs / ADI_ADF4030_ADEL_M_STEP_IN_FEMTO_SEC;

	return regmap_update_bits(st->regmap, ADF4030_REG(0x35),
				  ADF4030_ALIGN_THOLD_MSK,
				  FIELD_PREP(ADF4030_ALIGN_THOLD_MSK, threshold));
}

static int adf4030_auto_align_threshold_en(const struct adf4030_state *st, bool enable)
{
	return regmap_update_bits(st->regmap, ADF4030_REG(0x37),
					  ADF4030_EN_ITER_MSK, enable ? ADF4030_EN_ITER_MSK : 0);
}

static int adf4030_auto_align_single_channel(const struct adf4030_state *st,
					     u32 channel, u32 source_channel)
{
	u32 regval = 0, retry = 3;
	int ret;

	ret = regmap_clear_bits(st->regmap, ADF4030_REG(0x37),
				ADF4030_EN_SERIAL_ALIGN_MSK);
	if (ret)
		return ret;

	regval = source_channel | FIELD_PREP(ADF4030_EN_ALIGN_MSK, 1);
	ret = regmap_write(st->regmap, ADF4030_REG(0x11), regval);
	if (ret)
		return ret;

	do {
		ret = regmap_write(st->regmap, ADF4030_REG(0x10), channel);
		if (ret)
			return ret;

		ret = regmap_read(st->regmap, ADF4030_REG(0x8F), &regval);
		if (ret)
			return ret;

		dev_dbg(&st->spi->dev, "Auto-aligning channel %d to channel %d (try %u)\n",
			channel, source_channel, retry - 3);

		/* Wait for FSM to complete */
		ret = regmap_read_poll_timeout(st->regmap, ADF4030_REG(0x8F),
					       regval, !(regval & ADF4030_FSM_BUSY_MSK),
					       4000, 3000000);
		if (ret) {
			dev_err(&st->spi->dev, "Autoalign failed FSM_BUSY\n");
			return ret;
		}
		ret = regmap_read(st->regmap, ADF4030_REG(0x90), &regval);
		if (ret)
			return ret;

	} while (regval & (ADF4030_TDC_ERR_MSK | ADF4030_TMP_ALIGN_ERR) && retry--);

	return 0;
}

static int adf4030_core_die_temp_get(struct adf4030_state *st, int *die_temp)
{
	u32 regval;
	int ret;

	/* Should we just enable the ADC during probe?! */
	if (!st->adc_enabled) {
		ret = regmap_set_bits(st->regmap, ADF4030_REG(0x61),
				      ADF4030_ADC_CFG_MASK);
		if (ret)
			return ret;

		ret = regmap_clear_bits(st->regmap, ADF4030_REG(0x3C),
					ADF4030_PD_ADC_MSK);
		if (ret)
			return ret;

		st->adc_enabled = true;
	}

	ret = regmap_write(st->regmap, ADF4030_REG(0x72), ADF4030_START_CNV);
	if (ret)
		return ret;

	/* Wait for ADC ready */
	ret = regmap_read_poll_timeout(st->regmap, ADF4030_REG(0x8F),
				       regval, !(regval & ADF4030_ADC_BUSY_MSK),
				       2000, 50000);
	if (ret) {
		dev_err(&st->spi->dev, "ADC capture failed ADC Busy\n");
		return ret;
	}

	ret = regmap_bulk_read(st->regmap, ADF4030_REG(0x92), st->vals, 2);
	if (ret)
		return ret;

	*die_temp = sign_extend32((st->vals[1] << 8) | st->vals[0], 8);

	return 0;
}

static int adf4030_set_background_serial_alignment(struct adf4030_state *st, bool enable, u32 channel_flags)
{
	u32 tmp;
	int ret;

	if (!channel_flags) {
		dev_err(&st->spi->dev, "No channels selected for background alignment\n");
		return -EINVAL;
	}


	ret = regmap_read(st->regmap, ADF4030_REG(0x37), &tmp);
	if (ret)
		return ret;

	/* If enabled : Disable Background alignment */
	if (tmp & ADF4030_EN_BKGND_ALGN_MSK) {
		ret = regmap_update_bits(st->regmap, ADF4030_REG(0x17), ADF4030_STOP_FSM_MSK, 0xFF);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, ADF4030_REG(0x37), ADF4030_EN_BKGND_ALGN_MSK, 0x0);
		if (ret)
			return ret;

		st->bsync_autoalign_en = false;

		ret = regmap_update_bits(st->regmap, ADF4030_REG(0x17), ADF4030_STOP_FSM_MSK, 0x0);
		if (ret)
			return ret;

	}

	if (!enable)
		return 0;

	ret = adf4030_core_die_temp_get(st, &tmp); /* Enable ADC, ignore temp */
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, ADF4030_REG(0x11),
				 ADF4030_MANUAL_MODE_MSK | ADF4030_EN_ALIGN_MSK,
				 FIELD_PREP(ADF4030_MANUAL_MODE_MSK, 0) |
				 FIELD_PREP(ADF4030_EN_ALIGN_MSK, 1));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, ADF4030_REG(0x35), ADF4030_BSYNC_CAL_ON_1_0_MSK,
				 FIELD_PREP(ADF4030_BSYNC_CAL_ON_1_0_MSK, channel_flags & 0x3));
	if (ret)
		return ret;

	/* Write ADF4030_BSYNC_CAL_ON_9_2 */
	ret = regmap_write(st->regmap, ADF4030_REG(0x36), (channel_flags >> 2));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, ADF4030_REG(0x37),
				 ADF4030_EN_SERIAL_ALIGN_MSK | ADF4030_EN_BKGND_ALGN_MSK, 0xFF);
	if (ret)
		return ret;

	st->bsync_autoalign_en = true;

	return regmap_write(st->regmap, ADF4030_REG(0x10), 0xFF);
}

enum {
	BSYNC_OUT_EN,
	BSYNC_REF_CHAN,
	BSYNC_ALIGN_EN,
	BSYNC_ALIGN_THRESH,
	BSYNC_ALIGN_THRESH_EN,
	BSYNC_ALIGN_ITER,
	BSYNC_DUTY_CYCLE,
	BSYNC_ALIGN_BG_EN,
};

static ssize_t adf4030_ext_info_read(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan, char *buf)
{
	struct adf4030_state *st = iio_priv(indio_dev);
	struct adf4030_chan_spec *ch;
	u64 tdc_result;
	u32 rem;
	int ret;

	guard(mutex)(&st->lock);
	ch = &st->channels[chan->address];

	switch (private) {
	case BSYNC_OUT_EN:
		return sysfs_emit(buf, "%u\n", ch->channel_output_en);
	case BSYNC_REF_CHAN:
		return sysfs_emit(buf, "%u\n", ch->reference_chan);
	case BSYNC_ALIGN_THRESH:
		return sysfs_emit(buf, "%u\n", st->bsync_autoalign_theshold_fs);
	case BSYNC_ALIGN_THRESH_EN:
		return sysfs_emit(buf, "%u\n", st->bsync_autoalign_theshold_en);
	case BSYNC_ALIGN_ITER:
		return sysfs_emit(buf, "%u\n", st->bsync_autoalign_iter);
	case BSYNC_DUTY_CYCLE:
		ret = adf4030_duty_cycle_measure(st, ch->num, st->bsync_freq_odiv_a, &tdc_result);
		if (ret)
			return ret;

		rem = do_div(tdc_result, MICRO);
		return sysfs_emit(buf, "%u.%06u\n", (u32)tdc_result, rem);
	case BSYNC_ALIGN_BG_EN:
		return sysfs_emit(buf, "%u\n", st->bsync_autoalign_en);
	default:
		return -EOPNOTSUPP;
	}
}

static ssize_t adf4030_ext_info_write(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct adf4030_state *st = iio_priv(indio_dev);
	struct adf4030_chan_spec *ch;
	long long readin;
	int ret = 0;

	ret = kstrtoll(buf, 10, &readin);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);
	ch = &st->channels[chan->address];

	switch (private) {
	case BSYNC_OUT_EN:
		if (!ch->channel_io_reconfig_en)
			return -EPERM;

		ch->channel_output_en = !!readin;
		ret = adf4030_chan_dir_set(st, ch, false);
		if (ret)
			return ret;
		return len;
	case BSYNC_REF_CHAN:
		if (readin < 0 || readin >= ADF4030_NUM_CHAN)
			return -EINVAL;
		ch->reference_chan = readin;
		return len;
	case BSYNC_ALIGN_THRESH:
		if (readin < 0 || readin > ADI_ADF4030_MAX_THRESHOLD_IN_FEMTO_SEC)
			return -EINVAL;
		ret = adf4030_auto_align_threshold(st, readin);
		if (ret)
			return ret;

		st->bsync_autoalign_theshold_fs = readin;
		return len;
	case BSYNC_ALIGN_THRESH_EN:
		if (readin < 0 || readin > 1)
			return -EINVAL;
		ret = adf4030_auto_align_threshold_en(st, readin);
		if (ret)
			return ret;

		st->bsync_autoalign_theshold_en = readin;
		return len;
	case BSYNC_ALIGN_ITER:
		if (readin < 0 || readin > ADI_ADF4030_ALIGN_CYCLES_MAX_COUNT)
			return -EINVAL;
		ret = adf4030_auto_align_iteration_set(st, readin);
		if (ret)
			return ret;

		st->bsync_autoalign_iter = readin;
		return len;
	case BSYNC_ALIGN_BG_EN:
		if (readin < 0 || readin > 1)
			return -EINVAL;
		ret = adf4030_set_background_serial_alignment(st, readin, st->bsync_autoalign_mask);
		if (ret)
			return ret;

		st->bsync_autoalign_ref_chan = readin;
		return len;
	default:
		return -EOPNOTSUPP;
	}
}

static struct iio_chan_spec_ext_info adf4030_ext_info[] = {
	{
		.name = "output_enable",
		.read = adf4030_ext_info_read,
		.write = adf4030_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = BSYNC_OUT_EN,
	},
	{
		.name = "reference_channel",
		.read = adf4030_ext_info_read,
		.write = adf4030_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = BSYNC_REF_CHAN,
	},
	{
		.name = "autoalign_threshold",
		.read = adf4030_ext_info_read,
		.write = adf4030_ext_info_write,
		.shared = IIO_SHARED_BY_TYPE,
		.private = BSYNC_ALIGN_THRESH,
	},
	{
		.name = "autoalign_threshold_en",
		.read = adf4030_ext_info_read,
		.write = adf4030_ext_info_write,
		.shared = IIO_SHARED_BY_TYPE,
		.private = BSYNC_ALIGN_THRESH_EN,
	},
	{
		.name = "autoalign_iteration",
		.read = adf4030_ext_info_read,
		.write = adf4030_ext_info_write,
		.shared = IIO_SHARED_BY_TYPE,
		.private = BSYNC_ALIGN_ITER,
	},
	{
		.name = "duty_cycle",
		.read = adf4030_ext_info_read,
		.write = adf4030_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = BSYNC_DUTY_CYCLE,
	},
	{
		.name = "background_serial_alignment_en",
		.read = adf4030_ext_info_read,
		.write = adf4030_ext_info_write,
		.shared = IIO_SHARED_BY_TYPE,
		.private = BSYNC_ALIGN_BG_EN,
	},
	{},
};

static int adf4030_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct adf4030_state *st = iio_priv(indio_dev);
	struct adf4030_chan_spec *ch;
	s64 tdc_result;
	int ret;

	ch = &st->channels[chan->address];

	guard(mutex)(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		*val = ch->odivb_en ? st->bsync_freq_odiv_b : st->bsync_freq_odiv_a;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		ret = adf4030_tdc_measure(st, ch->num, ch->reference_chan,
					  st->bsync_freq_odiv_a, &tdc_result);
		if (ret)
			return ret;

		*val = lower_32_bits(tdc_result);
		*val2 = upper_32_bits(tdc_result);

		return IIO_VAL_INT_64;
	case IIO_CHAN_INFO_PROCESSED:
		ret = adf4030_core_die_temp_get(st, val);
		if (ret < 0)
			return ret;
		*val *= 1000;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = 1 << (st->avgexp + 6);
		return IIO_VAL_INT;
	default:
		return -EOPNOTSUPP;
	}
};

static int adf4030_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct adf4030_state *st = iio_priv(indio_dev);
	struct adf4030_chan_spec *ch;
	int ret;

	ch = &st->channels[chan->address];

	guard(mutex)(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_PHASE:
		ret = adf4030_auto_align_delay(st, ch->num, val);
		if (ret)
			return ret;

		return adf4030_auto_align_single_channel(st, ch->num, ch->reference_chan);
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		st->avgexp = ilog2(val) - 6;
		return regmap_update_bits(st->regmap, ADF4030_REG(0x16),
					  ADF4030_AVGEXP_MSK,
					  FIELD_PREP(ADF4030_AVGEXP_MSK, st->avgexp));
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int adf4030_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct adf4030_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const int adf4030_avg_range[] = {64, 128, 256, 512, 1024, 2048, 4096,
					8192, 16384, 32768, 65536, 131072,
					262144, 524288, 1048576, 2097152};

static int adf4030_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*type = IIO_VAL_INT;
		*vals = adf4030_avg_range;
		*length = ARRAY_SIZE(adf4030_avg_range);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int adf4030_fwnode_xlate(struct iio_dev *indio_dev,
				const struct fwnode_reference_args *iiospec)
{
	struct adf4030_state *st = iio_priv(indio_dev);

	for (int i = 0; i < st->num_channels; i++) {
		if (st->channels[i].num == iiospec->args[0])
			return i;
	}

	return -EINVAL;
}

static const struct iio_info adf4030_iio_info = {
	.read_raw = &adf4030_read_raw,
	.write_raw = &adf4030_write_raw,
	.read_avail = &adf4030_read_avail,
	.fwnode_xlate = &adf4030_fwnode_xlate,
	.debugfs_reg_access = &adf4030_reg_access,
};

static unsigned long adf4030_clk_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct adf4030_output *clkout = adf4030_to_output(hw);
	struct adf4030_state *st  = clkout->st;
	struct iio_chan_spec *chan;
	struct adf4030_chan_spec *ch;
	unsigned int address;

	address = clkout->address;
	if (address >= st->num_channels)
		return -EINVAL;

	chan = &st->iio_channels[address];
	ch = &st->channels[chan->address];

	return ch->odivb_en ? st->bsync_freq_odiv_b : st->bsync_freq_odiv_a;
}

static const struct clk_ops adf4030_clk_ops = {
	.recalc_rate = adf4030_clk_recalc_rate,
};

static int adf4030_clk_register(struct adf4030_state *st, unsigned int address,
				unsigned int num)
{
	struct clk_init_data init;
	const char *parent_name;
	int ret;

	parent_name = __clk_get_name(st->refin);

	init.name = st->clk_out_names[num];
	init.ops = &adf4030_clk_ops;
	init.flags = 0;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	st->outputs[address].hw.init = &init;
	st->outputs[address].st = st;
	st->outputs[address].address = address;

	ret = devm_clk_hw_register(&st->spi->dev, &st->outputs[address].hw);
	if (ret)
		return dev_err_probe(&st->spi->dev, ret,
				     "Failed to register clock(%s)\n",
				     init.name);

	st->clk_data->hws[address] = &st->outputs[address].hw;
	return 0;
}

static int adf4030_status_show(struct seq_file *file, void *offset)
{
	struct iio_dev *indio_dev = spi_get_drvdata(file->private);
	struct adf4030_state *st = iio_priv(indio_dev);
	u32 stat_8F, stat_90;
	int ret;

	ret = regmap_read(st->regmap, ADF4030_REG(0x8F), &stat_8F);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADF4030_REG(0x90), &stat_90);
	if (ret)
		return ret;

	seq_printf(file,
		   "REF Status:\t%s\nPLL Status:\t%s\nTDC Error:\t%s\nTemp Align Error:\t%s\n",
		   stat_8F & ADF4030_REF_OK_MSK ?
		   "OK" : "Error",
		   stat_90 & ADF4030_PLL_LOCK_MSK ?
		   "Locked" : "Unlocked",
		   stat_90 & ADF4030_TDC_ERR_MSK ?
		   "Error" : "OK",
		   stat_90 & ADF4030_TMP_ALIGN_ERR ?
		   "Error" : "OK");

	seq_printf(file,
		   "FSM Status:\t%s\nADC Status:\t%s\nMath Status:\t%s\nDL Status:\t%s\nTDC Status:\t%s\n",
		   stat_8F & ADF4030_FSM_BUSY_MSK ?
		   "Busy" : "Idle",
		   stat_8F & ADF4030_ADC_BUSY_MSK ?
		   "Busy" : "Idle",
		   stat_8F & ADF4030_MATH_BUSY_MSK ?
		   "Busy" : "Idle",
		   stat_8F & ADF4030_DL_BUSY_MSK ?
		   "Busy" : "Idle",
		   stat_8F & ADF4030_TDC_BUSY_MSK ?
		   "Busy" : "Idle");

	return 0;
}

static int adf4030_jesd204_sysref(struct jesd204_dev *jdev)
{
	return 0;
}

static int adf4030_jesd204_link_supported(struct jesd204_dev *jdev,
					  enum jesd204_state_op_reason reason,
					  struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adf4030_state *st = iio_priv(indio_dev);
	unsigned long rate;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	ret = jesd204_link_get_lmfc_lemc_rate(lnk, &rate);
	if (ret)
		return ret;

	if (rate != st->bsync_freq_odiv_a) {
		ret = adf4030_set_odiva_freq(st, rate);
		if (ret)
			return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int adf4030_jesd204_clks_sync3(struct jesd204_dev *jdev,
				      enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adf4030_state *st = iio_priv(indio_dev);
	struct adf4030_chan_spec *chan;
	int ret, i;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	/* Configure clocks */
	for (i = 0; i < st->num_channels; i++) {
		chan = &st->channels[i];

		if (!chan->align_on_sync_en)
			continue;

		ret = adf4030_auto_align_single_channel(st,
							chan->num,
							chan->reference_chan);
		if (ret)
			return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data adf4030_jesd204_data = {
	.sysref_cb = adf4030_jesd204_sysref,
	.state_ops = {
		[JESD204_OP_LINK_SUPPORTED] = {
			.per_link = adf4030_jesd204_link_supported,
		},

		[JESD204_OP_CLK_SYNC_STAGE3] = {
			.per_device = adf4030_jesd204_clks_sync3,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},
};

static int adf4030_startup(struct adf4030_state *st, u32 ref_input_freq_hz,
			   u32 vco_out_freq_hz)
{
	struct device *dev = &st->spi->dev;
	u32 rdiv, ndiv, odiv, regval, coreclk;
	int ret;
	bool en = true;

	ret = regmap_set_bits(st->regmap, ADF4030_REG(0xFF),
			      ADF4030_SOFTRESET_CHIP_MSK);
	if (ret)
		return ret;

	fsleep(10);

	if (st->spi->mode & SPI_3WIRE || st->spi_3wire_en)
		en = false;

	/* Enable SDO and ADDRESS_ASCENSION */
	ret = regmap_write(st->regmap, ADF4030_REG(0x00),
			   FIELD_PREP(ADF4030_SDO_ACTIVE_MSK, en) |
			   FIELD_PREP(ADF4030_ADDRESS_ASCENSION_MSK, 1) |
			   FIELD_PREP(ADF4030_SDO_ACTIVE_R_MSK, en) |
			   FIELD_PREP(ADF4030_ADDRESS_ASCENSION_R_MSK, 1));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADF4030_REG(0x01), 0);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADF4030_REG(0x0A), 0xAD);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADF4030_REG(0x0A), &regval);
	if (ret)
		return ret;

	if (regval != 0xAD)
		return dev_err_probe(dev, -EIO,
				     "Failed SPI write/read verify test REG_0x0A=0x%X\n", regval);

	/* Set default registers */
	ret = regmap_multi_reg_write(st->regmap, adf4030_reg_default,
				     ARRAY_SIZE(adf4030_reg_default));
	if (ret)
		return ret;

	for (rdiv = 0; rdiv < 4; rdiv++) {
		coreclk = ref_input_freq_hz / (1 << rdiv);
		if (coreclk <= 125000000U) /* 125MHz */
			break;
	}

	if (rdiv == 4)
		return dev_err_probe(dev, -EINVAL,
				     "Reference input frequency too high\n");

	ret = regmap_update_bits(st->regmap, ADF4030_REG(0x58),
				 ADF4030_DIVREF_MSK,
				 FIELD_PREP(ADF4030_DIVREF_MSK, rdiv));
	if (ret)
		return ret;

	/* Calculate ADC_CLK_DIV */
	rdiv = coreclk / 1600000U;

	ret = regmap_write(st->regmap, ADF4030_REG(0x62), rdiv);
	if (ret)
		return ret;

	ret = regmap_clear_bits(st->regmap, ADF4030_REG(0x3C),
				ADF4030_PD_ALL_MSK | ADF4030_PD_TDC_MSK | ADF4030_PD_PLL_MSK);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADF4030_REG(0x8F), &regval);
	if (ret)
		return ret;
	if (!FIELD_GET(ADF4030_REF_OK_MSK, regval))
		return dev_err_probe(dev, -EIO,
				     "Reference input amplitude below threshold\n");

	ret = regmap_set_bits(st->regmap, ADF4030_REG(0x5A),
			      ADF4030_PLL_CAL_EN_MSK);
	if (ret)
		return ret;

	ret = adf4030_compute_r_n(ref_input_freq_hz, vco_out_freq_hz, &rdiv, &ndiv);
	if (ret)
		return dev_err_probe(&st->spi->dev, ret,
			"Failed to compute R and N dividers for Fref=%u Hz amd VCO=%u Hz\n",
			ref_input_freq_hz, vco_out_freq_hz);

	ret = regmap_update_bits(st->regmap, ADF4030_REG(0x57),
				 ADF4030_RDIV_MASK,
				 FIELD_PREP(ADF4030_RDIV_MASK, rdiv));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADF4030_REG(0x56), ndiv);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(st->regmap, ADF4030_REG(0xBA),
				       regval, !(regval & ADF4030_CAL_BUSY_MSK),
				       100, 1000);
	if (ret)
		return dev_err_probe(&st->spi->dev, ret, "PLL calibration failed\n");

	ret = regmap_read_poll_timeout(st->regmap, ADF4030_REG(0x90),
				       regval, regval & ADF4030_PLL_LOCK_MSK,
				       2000, 500000);
	if (ret)
		return dev_err_probe(&st->spi->dev, ret, "PLL failed to lock\n");

	ret = adf4030_set_odiva_freq(st, st->bsync_freq_odiv_a);
	if (ret)
		return ret;

	if (st->bsync_freq_odiv_b) {
		ret = adf4030_compute_odiv(vco_out_freq_hz, st->bsync_freq_odiv_b, &odiv);
		if (ret)
			return dev_err_probe(&st->spi->dev, ret,
				"Failed to compute ODIVB for Fvco=%u Hz and Fbsync=%u Hz\n",
				vco_out_freq_hz, st->bsync_freq_odiv_b);

		ret = regmap_write(st->regmap, ADF4030_REG(0x55), odiv >> 4);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, ADF4030_REG(0x54),
					 ADF4030_ODIVB_MSK,
					 FIELD_PREP(ADF4030_ODIVB_MSK, odiv));
		if (ret)
			return ret;
	}

	/* Set some defaults based datasheets limits */
	if (st->bsync_freq_odiv_a > 2000000U)
		st->avgexp = 15;
	else
		st->avgexp = 13;

	ret = regmap_update_bits(st->regmap, ADF4030_REG(0x16),
				 ADF4030_AVGEXP_MSK,
				 FIELD_PREP(ADF4030_AVGEXP_MSK, st->avgexp));
	if (ret)
		return ret;

	return 0;
}

static int adf4030_parse_fw(struct adf4030_state *st)
{
	struct device *dev = &st->spi->dev;
	unsigned int i, cnt = 0;
	int ret;

	st->spi_3wire_en = device_property_read_bool(dev,
						     "adi,spi-3wire-enable");

	ret = device_property_read_u32(dev, "adi,vco-frequency-hz",
				       &st->vco_freq);
	if (ret)
		return dev_err_probe(dev, -EINVAL,
				     "Missing mandatoy adi,vco-frequency-hz property");

	ret = device_property_read_u32(dev, "adi,bsync-frequency-hz",
				       &st->bsync_freq_odiv_a);
	if (ret)
		return dev_err_probe(dev, -EINVAL,
				     "Missing mandatoy adi,bsync-frequency-hz property");

	device_property_read_u32(dev, "adi,bsync-secondary-frequency-hz",
				 &st->bsync_freq_odiv_b);

	ret = device_property_read_u32(dev, "adi,bsync-autoalign-reference-channel",
				       &st->bsync_autoalign_ref_chan);
	if (ret)
		return ret;

	st->bsync_autoalign_iter = 8;
	device_property_read_u32(dev, "adi,bsync-autoalign-interation-count",
				 &st->bsync_autoalign_iter);

	st->bsync_autoalign_theshold_fs = 1400;
	device_property_read_u32(dev, "adi,bsync-autoalign-thehsold-fs",
				 &st->bsync_autoalign_theshold_fs);

	if (device_property_read_string_array(dev, "clock-output-names",
						st->clk_out_names,
						ARRAY_SIZE(st->clk_out_names))) {
		for (i = 0; i < ARRAY_SIZE(st->clk_out_names); i++) {
			st->clk_out_names[i] = devm_kasprintf(dev, GFP_KERNEL, "%s-clk%u",
							fwnode_get_name(dev_fwnode(dev)), i);

			if (!st->clk_out_names[i])
				return -ENOMEM;
		}
	}

	st->num_channels = device_get_child_node_count(dev);
	if (!st->num_channels || st->num_channels > ADF4030_NUM_CHAN)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid chan number(%u)\n",
				     st->num_channels);

	st->channels = devm_kcalloc(dev, st->num_channels,
				    sizeof(*st->channels), GFP_KERNEL);
	if (!st->channels)
		return -ENOMEM;

	device_for_each_child_node_scoped(dev, child) {
		st->channels[cnt].num = cnt;
		ret = fwnode_property_read_u32(child, "reg",
					       &st->channels[cnt].num);
		if (ret)
			return dev_err_probe(dev, -EINVAL,
					     "Missing mandatory reg property\n");
		if (st->channels[cnt].num >= ADF4030_NUM_CHAN)
			return dev_err_probe(dev, -EINVAL,
					     "Missing chan index(%u)\n",
					     st->channels[cnt].num);

		fwnode_property_read_u32(child, "adi,delay-fs",
					 &st->channels[cnt].delay);

		fwnode_property_read_string(child, "adi,extended-name",
					    &st->channels[cnt].extended_name);

		st->channels[cnt].reference_chan = st->bsync_autoalign_ref_chan;
		fwnode_property_read_u32(child, "adi,reference-channel-num",
					 &st->channels[cnt].reference_chan);
		if (st->channels[cnt].reference_chan >= ADF4030_NUM_CHAN &&
		    st->channels[cnt].reference_chan != 26) /* 26 == output of REFIN receiver */
			return dev_err_probe(dev, -EINVAL,
					     "Invalid reference channel(%u)\n",
					     st->channels[cnt].reference_chan);

		if (fwnode_property_read_bool(child, "adi,output-en"))
			st->channels[cnt].channel_output_en = true;
		if (fwnode_property_read_bool(child, "adi,input-output-reconfig-en"))
			st->channels[cnt].channel_io_reconfig_en = true;
		if (fwnode_property_read_bool(child, "adi,use-secondary-odiv-b-en"))
			st->channels[cnt].odivb_en = true;
		if (fwnode_property_read_bool(child, "auto-align-on-sync-en"))
			st->channels[cnt].align_on_sync_en = true;
		if (fwnode_property_read_bool(child, "adi,invert-en"))
			st->channels[cnt].invert_en = true;

		/* Pin Termination Config - please see datasheet */
		if (fwnode_property_read_bool(child, "adi,ac-coupled-en"))
			st->channels[cnt].ac_coupled = true;
		if (fwnode_property_read_bool(child, "adi,link-tx-en"))
			st->channels[cnt].link_tx = true;
		if (fwnode_property_read_bool(child, "adi,link-rx-en"))
			st->channels[cnt].link_rx = true;
		if (fwnode_property_read_bool(child, "adi,float-tx-en"))
			st->channels[cnt].float_tx  = true;
		if (fwnode_property_read_bool(child, "adi,float-rx-en"))
			st->channels[cnt].float_rx  = true;

		fwnode_property_read_u32(child, "adi,rcm", &st->channels[cnt].rcm);
		if (st->channels[cnt].rcm > 63)
			return dev_err_probe(dev, -EINVAL,
					     "Invalid RCM value(%u)\n",
					     st->channels[cnt].rcm);

		cnt++;
	}

	return 0;
}

static int adf4030_prepare(struct adf4030_state *st)
{
	struct device *dev = &st->spi->dev;
	struct adf4030_chan_spec *chan;
	int ret, i;

	/* Program the output channels */
	for (i = 0; i < st->num_channels; i++) {
		chan = &st->channels[i];

		st->iio_channels[i].type = IIO_ALTVOLTAGE;
		st->iio_channels[i].output = 1;
		st->iio_channels[i].indexed = 1;
		st->iio_channels[i].channel = chan->num;
		st->iio_channels[i].address = i;
		st->iio_channels[i].extend_name = chan->extended_name;
		st->iio_channels[i].ext_info = adf4030_ext_info;
		st->iio_channels[i].info_mask_separate =
			BIT(IIO_CHAN_INFO_FREQUENCY) |
			BIT(IIO_CHAN_INFO_PHASE);
		st->iio_channels[i].info_mask_shared_by_type_available =
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO);
		st->iio_channels[i].info_mask_shared_by_type =
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO);
	}

	st->iio_channels[i].type = IIO_TEMP;
	st->iio_channels[i].indexed = 1;
	st->iio_channels[i].info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED);
	st->iio_channels[i].scan_index = -1;
	st->iio_channels[i].address = i;

	st->clk_data = devm_kzalloc(&st->spi->dev,
				    struct_size(st->clk_data, hws, st->num_channels),
				    GFP_KERNEL);
	if (!st->clk_data)
		return -ENOMEM;

	/* Configure clocks */
	for (i = 0; i < st->num_channels; i++) {
		chan = &st->channels[i];

		ret = adf4030_clk_register(st, i, chan->num);
		if (ret)
			return ret;
	}

	st->clk_data->num = st->num_channels;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   st->clk_data);
}

static int adf4030_configure(struct adf4030_state *st)
{
	struct adf4030_chan_spec *chan;
	int ret, i;

	ret = adf4030_startup(st, st->ref_freq, st->vco_freq);
	if (ret)
		return ret;

	ret = adf4030_auto_align_threshold(st, st->bsync_autoalign_theshold_fs);
	if (ret)
		return ret;

	ret = adf4030_auto_align_iteration_set(st, st->bsync_autoalign_iter);
	if (ret)
		return ret;

	for (i = 0; i < st->num_channels; i++) {
		chan = &st->channels[i];

		ret = adf4030_chan_dir_set(st, chan, true);
		if (ret)
			return ret;

		if (!chan->delay)
			continue;

		ret = adf4030_auto_align_delay(st, chan->num, chan->delay);
		if (ret)
			return ret;
	}

	return 0;
}

static int adf4030_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adf4030_state *st;
	int ret, i;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	st->refin = devm_clk_get_optional_enabled(&spi->dev, "refin");
	if (IS_ERR(st->refin))
		return dev_err_probe(&spi->dev, PTR_ERR(st->refin),
				     "failed to get refin\n");

	st->jdev = devm_jesd204_dev_register(&spi->dev, &adf4030_jesd204_data);
	if (IS_ERR(st->jdev))
		return dev_err_probe(&spi->dev, PTR_ERR(st->jdev),
				     "failed to register JESD204 device\n");

	st->regmap = devm_regmap_init_spi(spi, &adf4030_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(st->regmap),
				     "Failed to allocate regmap\n");

	ret = adf4030_parse_fw(st);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to parse devicetree\n");

	if (st->refin)
		st->ref_freq = clk_get_rate(st->refin);

	for (i = 0; i < st->num_channels; i++)
		if (st->channels[i].align_on_sync_en)
			st->bsync_autoalign_mask |= BIT(st->channels[i].num);

	indio_dev->name = "adf4030";
	indio_dev->info = &adf4030_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->iio_channels;
	/* for the die temperature */
	indio_dev->num_channels = st->num_channels + 1;

	ret = adf4030_prepare(st);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to prepare device\n");

	ret = adf4030_configure(st);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to configure device\n");

	mutex_init(&st->lock);
	spi_set_drvdata(spi, indio_dev);

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to register IIO device\n");

	if (IS_ENABLED(CONFIG_DEBUG_FS))
		debugfs_create_devm_seqfile(&spi->dev, "status",
					    iio_get_debugfs_dentry(indio_dev),
					    adf4030_status_show);

	ret = devm_jesd204_fsm_start(&spi->dev, st->jdev, JESD204_LINKS_ALL);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to start JESD204 FSM\n");

	return 0;
}

static const struct spi_device_id adf4030_id[] = {
	{"adf4030", 4030},
	{ }
};
MODULE_DEVICE_TABLE(spi, adf4030_id);

static const struct of_device_id adf4030_of_match[] = {
	{ .compatible = "adi,adf4030" },
	{ }
};
MODULE_DEVICE_TABLE(of, adf4030_of_match);

static struct spi_driver adf4030_driver = {
	.driver = {
		.name = "adf4030",
		.of_match_table = adf4030_of_match,
	},
	.probe = adf4030_probe,
	.id_table = adf4030_id,
};
module_spi_driver(adf4030_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF4030 driver");
MODULE_LICENSE("GPL");
