/*
 * AD9361 Agile RF Transceiver
 *
 * Copyright 2013 Analog Devices Inc.
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
#include <linux/string.h>

#include <linux/of.h>
#include <linux/of_gpio.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"
#include "ad9361.h"

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#define AD_READ		(0 << 15)
#define AD_WRITE		(1 << 15)
#define AD_CNT(x)	((((x) - 1) & 0x7) << 12)
#define AD_ADDR(x)	((x) & 0x3FF)

#define CHIPID_AD9361			0x85
#define AD9361_DEF_OUTPUT_MODE		0x00
#define AD9361_REG_VREF_MASK		0xC0

enum ad9361_clocks {
	BB_REFCLK,
	RX_REFCLK,
	TX_REFCLK,
	BBPLL_CLK,
	ADC_CLK,
	R2_CLK,
	R1_CLK,
	CLKRF_CLK,
	RX_SAMPL_CLK,
	DAC_CLK,
	T2_CLK,
	T1_CLK,
	CLKTF_CLK,
	TX_SAMPL_CLK,
	RX_RFPLL,
	TX_RFPLL,
	NUM_AD9361_CLKS,
};

enum ad9361_pdata_rx_freq {
	BBPLL_FREQ,
	ADC_FREQ,
	R2_FREQ,
	R1_FREQ,
	CLKRF_FREQ,
	RX_SAMPL_FREQ,
};

enum ad9361_pdata_tx_freq {
	IGNORE,
	DAC_FREQ,
	T2_FREQ,
	T1_FREQ,
	CLKTF_FREQ,
	TX_SAMPL_FREQ,
};

enum {
	ID_AD9361,
};

struct rx_gain_info {
	enum rx_gain_table_type tbl_type;
	int starting_gain_db;
	int max_gain_db;
	int gain_step_db;
	int max_idx;
	int idx_step_offset;
};

struct ad9361_phy_platform_data {
	bool			rx2tx2;
	bool			fdd;
	bool			split_gt;
	bool 			use_extclk;
	bool			ensm_pin_level_mode;
	bool			ensm_pin_ctrl;
	unsigned			dcxo_coarse;
	unsigned			dcxo_fine;
	unsigned			rf_rx_input_sel;
	unsigned			rf_tx_output_sel;
	unsigned long		rx_path_clks[6];
	unsigned long		tx_path_clks[6];
	unsigned long long	rx_synth_freq;
	unsigned long long	tx_synth_freq;
	unsigned			rf_bandwidth_Hz;
	int			tx_atten;
	u8			pp_conf[3];
	u8			rx_clk_data_delay;
	u8			tx_clk_data_delay;
	u8			digital_io_ctrl;
	u8			lvds_bias_ctrl;
	u8			lvds_invert[2];

};

struct ad9361_rf_phy {
	struct spi_device 	*spi;
	struct clk 		*clk_refin;
	struct clk 		*clks[NUM_AD9361_CLKS];
	struct clk_onecell_data	clk_data;
	struct ad9361_phy_platform_data *pdata;
	struct bin_attribute 	bin;
	struct iio_dev 		*indio_dev;
	struct work_struct 	work;
	struct completion       complete;
	u8 			prev_ensm_state;
	u8			curr_ensm_state;
	struct rx_gain_info rx_gain[RXGAIN_TBLS_END];
	enum rx_gain_table_name current_table;
	bool 			ensm_pin_ctl_en;

	bool			auto_cal_en;
	unsigned long long 	last_rx_quad_cal_freq;
	unsigned long long 	last_tx_quad_cal_freq;
	unsigned long		flags;
	unsigned long		cal_threshold_freq;
	unsigned			current_bw_Hz;
	unsigned			ensm_conf1;
	unsigned			rate_governor;
	bool			bypass_rx_fir;
	bool			bypass_tx_fir;
	u8			tx_fir_int;
	u8			tx_fir_ntaps;
	u8			rx_fir_dec;
	u8			rx_fir_ntaps;
	u8			agc_mode[2];
};

struct refclk_scale {
	struct clk_hw	hw;
	struct spi_device *spi;
	struct ad9361_rf_phy *phy;
	unsigned int	mult;
	unsigned int	div;
	enum ad9361_clocks source;
};

struct rf_rx_gain {
	u32 ant;		/* Antenna number to read gain */
	s32 gain_db;		/* gain value in dB */
	u32 lmt_index;	/* LNA-MIXER-TIA gain index */
	u32 lmt_gain;		/* LNA-MIXER-TIA gain in dB */
	u32 lpf_gain;		/* Low pass filter gain in dB */
	u32 digital_gain;	/* Digital gain in dB */
};
struct rf_rssi {
	u32 ant;		/* Antenna number for which RSSI is reported */
	u32 symbol;		/* Runtime RSSI */
	u32 preamble;		/* Initial RSSI */
	s32 multiplier;	/* Multiplier to convert reported RSSI */
	u8 duration;		/* Duration to be considered for measuring */
};

const char ad9361_ensm_states[][10] = { "sleep", "", "", "", "", "alert", "tx", "tx flush",
	"rx", "rx_flush", "fdd", "fdd_flush"};

static int ad9361_spi_readm(struct spi_device *spi, unsigned reg,
			   unsigned char *rbuf, unsigned num)
{
	unsigned char buf[2];
	int ret;
	u16 cmd;

	if (num > MAX_MBYTE_SPI)
		return -EINVAL;

	cmd = AD_READ | AD_CNT(num) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;

	ret = spi_write_then_read(spi, &buf[0], 2, rbuf, num);
	if (ret < 0)
		return ret;

#ifdef _DEBUG
	{
		int i;
		for (i = 0; i < num; i++)
			dev_dbg(&spi->dev, "%s: reg 0x%X val 0x%X\n",
				__func__, reg--, rbuf[i]);
	}
#endif

	return 0;
}

static int ad9361_spi_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf;
	int ret;

	ret = ad9361_spi_readm(spi, reg, &buf, 1);
	if (ret < 0)
		return ret;

	return buf;
}

static int __ad9361_spi_readf(struct spi_device *spi, unsigned reg,
				 unsigned mask, unsigned offset)
{
	unsigned char buf;
	int ret;

	if (!mask)
		return -EINVAL;

	ret = ad9361_spi_readm(spi, reg, &buf, 1);
	if (ret < 0)
		return ret;

	buf &= mask;
	buf >>= offset;

	return buf;
}

#define ad9361_spi_readf(spi, reg, mask) \
	__ad9361_spi_readf(spi, reg, mask, __ffs(mask))

static int ad9361_spi_write(struct spi_device *spi,
			 unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;
	u16 cmd;

	cmd = AD_WRITE | AD_CNT(1) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;
	buf[2] = val;

	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0)
		return ret;

#ifdef _DEBUG
	dev_dbg(&spi->dev, "%s: reg 0x%X val 0x%X\n", __func__, reg, buf[2]);
#endif

	return 0;
}

static int __ad9361_spi_writef(struct spi_device *spi, unsigned reg,
				 unsigned mask, unsigned offset, unsigned val)
{
	unsigned char buf;
	int ret;

	if (!mask)
		return -EINVAL;

	ret = ad9361_spi_readm(spi, reg, &buf, 1);
	if (ret < 0)
		return ret;

	buf &= ~mask;
	buf |= ((val << offset) & mask);

	return ad9361_spi_write(spi, reg, buf);
}

#define ad9361_spi_writef(spi, reg, mask, val) \
	__ad9361_spi_writef(spi,reg, mask, __ffs(mask), val)

static int ad9361_spi_writem(struct spi_device *spi,
			 unsigned reg, unsigned char *tbuf, unsigned num)
{
	unsigned char buf[10];
	int ret;
	u16 cmd;

	if (num > MAX_MBYTE_SPI)
		return -EINVAL;

	cmd = AD_WRITE | AD_CNT(num) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;

	memcpy(&buf[2], tbuf, num);

	ret = spi_write_then_read(spi, buf, num + 2, NULL, 0);
	if (ret < 0)
		return ret;

#ifdef _DEBUG
	{
		int i;
		for (i = 0; i < num; i++)
			printk("%s: reg 0x%X val 0x%X\n", __func__, reg--, tbuf[i]);
	}
#endif

	return 0;
}

static int ad9361_check_cal_done(struct ad9361_rf_phy *phy, unsigned reg,
				 unsigned mask, bool done_state)
{
	unsigned timeout = 500;
	unsigned state;

	do {
		state = ad9361_spi_readf(phy->spi, reg, mask);
		if (state == done_state)
			return 0;

		msleep_interruptible(1);
	} while (timeout--);

	dev_err(&phy->spi->dev, "Calibration TIMEOUT (0x%X, 0x%X)", reg, mask);

	return -ETIMEDOUT;
}

static int ad9361_run_calibration(struct ad9361_rf_phy *phy, unsigned mask)
{
	int ret = ad9361_spi_write(phy->spi, CALIBRATION_CONTROL, mask);
	if (ret < 0)
		return ret;

	dev_dbg(&phy->spi->dev, "%s: CAL Mask 0x%X", __func__, mask);

	return ad9361_check_cal_done(phy, CALIBRATION_CONTROL, mask, 0);
}

enum rx_gain_table_name ad9361_gt_tableindex(unsigned long long freq)
{
	if (freq <= 1300000000ULL)
		return TBL_200_1300_MHZ;

	if (freq <= 4000000000ULL)
		return TBL_1300_4000_MHZ;

	return TBL_4000_6000_MHZ;
}

/* PLL operates between 47 .. 6000 MHz which is > 2^32 */

unsigned long ad9361_to_clk(unsigned long long freq)
{
	return (unsigned long)(freq >> 1);
}

unsigned long long ad9361_from_clk(unsigned long freq)
{
	return ((unsigned long long)freq << 1);
}

static int ad9361_load_gt(struct ad9361_rf_phy *phy, unsigned long long freq)
{
	struct spi_device *spi = phy->spi;
	const unsigned char (*tab)[3];
	unsigned band, index_max, i;

	dev_dbg(&phy->spi->dev, "%s: frequency %llu", __func__, freq);

	band = ad9361_gt_tableindex(freq);

	dev_dbg(&phy->spi->dev, "%s: frequency %llu (band %d)",
		__func__, freq, band);

	/* check if table is present */
	if (phy->current_table == band)
		return 0;

	ad9361_spi_writef(spi, AGC_CONFIG_2,
			       FULL_GAIN_TBL, !phy->pdata->split_gt);

	if (phy->pdata->split_gt) {
		tab = &split_gain_table[band][0];
		index_max = SIZE_SPLIT_TABLE;
	} else {
		tab = &full_gain_table[band][0];
		index_max = SIZE_FULL_TABLE;
	}

	ad9361_spi_write(spi, GAIN_TABLE_CONFIG, 0x1A); /* Start Gain Table Clock */

	for (i = 0; i < index_max; i++) {
		ad9361_spi_write(spi, GAIN_TABLE_ADDRESS, i); /* Gain Table Index */
		ad9361_spi_write(spi, GAIN_TABLE_WRITE_DATA1, tab[i][0]); /* Ext LNA, Int LNA, & Mixer Gain Word */
		ad9361_spi_write(spi, GAIN_TABLE_WRITE_DATA2, tab[i][1]); /* TIA & LPF Word */
		ad9361_spi_write(spi, GAIN_TABLE_WRITE_DATA3, tab[i][2]); /* DC Cal bit & Dig Gain Word */
		ad9361_spi_write(spi, GAIN_TABLE_CONFIG, 0x1E); /* Gain Table Index */
		ad9361_spi_write(spi, GAIN_TABLE_READ_DATA1, 0); /* Dummy Write to delay 3 ADCCLK/16 cycles */
		ad9361_spi_write(spi, GAIN_TABLE_READ_DATA1, 0); /* Dummy Write to delay ~1u */
	}

	ad9361_spi_write(spi, GAIN_TABLE_CONFIG, 0x1A); /* Clear Write Bit */
	ad9361_spi_write(spi, GAIN_TABLE_READ_DATA1, 0); /* Dummy Write to delay ~1u */
	ad9361_spi_write(spi, GAIN_TABLE_READ_DATA1, 0); /* Dummy Write to delay ~1u */
	ad9361_spi_write(spi, GAIN_TABLE_CONFIG, 0); /* Stop Gain Table Clock */

	phy->current_table = band;

	return 0;
}

static int ad9361_load_mixer_gm_subtable(struct ad9361_rf_phy *phy)
{
	int i, addr;

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(phy->spi, GM_SUB_TABLE_CONFIG, 0x2); /* Start Clock */

	for (i = 0, addr = ARRAY_SIZE(gm_st_ctrl); i < ARRAY_SIZE(gm_st_ctrl); i++) {
		ad9361_spi_write(phy->spi, GAIN_TABLE_ADDRESS, --addr); /* Gain Table Index */
		ad9361_spi_write(phy->spi, GM_SUB_TABLE_BIAS_WRITE, 0); /* Bias */
		ad9361_spi_write(phy->spi, GM_SUB_TABLE_GAIN, gm_st_gain[i]); /* Gain */
		ad9361_spi_write(phy->spi, GM_SUB_TABLE_CTL_WRITE, gm_st_ctrl[i]); /* Control */
		ad9361_spi_write(phy->spi, GM_SUB_TABLE_CONFIG, 0x6); /* Write Words */
		ad9361_spi_write(phy->spi, GM_SUB_TABLE_GAIN_READ, 0); /* Dummy Delay */
		ad9361_spi_write(phy->spi, GM_SUB_TABLE_GAIN_READ, 0); /* Dummy Delay */
	}

	ad9361_spi_write(phy->spi, GM_SUB_TABLE_CONFIG, 0x2); /* Clear Write */
	ad9361_spi_write(phy->spi, GM_SUB_TABLE_GAIN_READ, 0); /* Dummy Delay */
	ad9361_spi_write(phy->spi, GM_SUB_TABLE_GAIN_READ, 0); /* Dummy Delay */
	ad9361_spi_write(phy->spi, GM_SUB_TABLE_CONFIG, 0); /* Stop Clock */

	return 0;
}

static int ad9361_set_tx_atten(struct ad9361_rf_phy *phy, unsigned atten_mdb,
			       bool tx1, bool tx2)
{
	unsigned char buf[2];
	int ret = 0;

	dev_dbg(&phy->spi->dev, "%s : attenuation %u mdB tx1=%d tx2=%d",
		__func__, atten_mdb, tx1, tx2);

	if (atten_mdb > 89750) /* 89.75 dB */
		return -EINVAL;

	atten_mdb /= 250; /* Scale to 0.25dB / LSB */

	buf[0] = atten_mdb >> 8;
	buf[1] = atten_mdb & 0xFF;

	if (tx1)
		ret = ad9361_spi_writem(phy->spi, TX1_ATTEN_1, buf, 2);

	if (tx2)
		ret = ad9361_spi_writem(phy->spi, TX2_ATTEN_1, buf, 2);

	ad9361_spi_writef(phy->spi, TX2_DIG_ATTENUATION, 0x40, 1);


	return ret;
}

static int ad9361_get_tx_atten(struct ad9361_rf_phy *phy, unsigned tx_num)
{
	unsigned char buf[2];
	int ret = 0;
	unsigned code;

	ret = ad9361_spi_readm(phy->spi,
			       (tx_num == 1) ? TX1_ATTEN_1 : TX2_ATTEN_1,
			       buf, 2);

	if (ret < 0)
		return ret;

	code = (buf[0] << 8) | buf[1];

	code *= 250;

	return code;
}

unsigned ad9361_rfvco_tableindex(unsigned long freq)
{
	if (freq < 50000000UL)
		return LUT_FTDD_40;

	if (freq <= 70000000UL)
		return LUT_FTDD_60;

	return LUT_FTDD_80;
}

static int ad9361_rfpll_vco_init(struct ad9361_rf_phy *phy,
				 bool tx, unsigned long long vco_freq,
				 unsigned long ref_clk)
{
	struct spi_device *spi = phy->spi;
	const struct SynthLUT (*tab);
	int i = 0;
	unsigned range, offs = 0;

	range = ad9361_rfvco_tableindex(ref_clk);

	dev_dbg(&phy->spi->dev, "%s : vco_freq %llu : ref_clk %lu : range %d",
		__func__, vco_freq, ref_clk, range);

	do_div(vco_freq, 1000000UL); /* vco_freq in MHz */

	if (phy->pdata->fdd) {
		tab = &SynthLUT_FDD[range][0];
	} else {
		tab = &SynthLUT_TDD[range][0];

	}

	if (tx)
		offs = TX_VCO_OUTPUT - RX_VCO_OUTPUT;

	while (i < SYNTH_LUT_SIZE && tab[i].VCO_MHz > vco_freq)
		i++;

	dev_dbg(&phy->spi->dev, "%s : freq %d MHz : index %d",
		__func__, tab[i].VCO_MHz, i);

	ad9361_spi_writef(spi, RX_VCO_OUTPUT + offs, 0x40, 1);

	ad9361_spi_writef(spi, RX_VCO_OUTPUT + offs,
			       MASK_VCO_OUTPUT, tab[i].VCO_Output_Level);
	ad9361_spi_writef(spi, RX_ALC_VARACTOR + offs,
			       MASK_VCO_VARACTOR, tab[i].VCO_Varactor);
	ad9361_spi_writef(spi, RX_VCO_BIAS_1 + offs,
			       MASK_VCO_BIAS_REF, tab[i].VCO_Bias_Ref);
	ad9361_spi_writef(spi, RX_VCO_BIAS_1 + offs,
			       MASK_VCO_BIAS_TCF, tab[i].VCO_Bias_Tcf);
	ad9361_spi_writef(spi, RX_FORCE_VCO_TUNE_1 + offs,
			       MASK_FORCE_VCO_TUNE1, tab[i].VCO_Cal_Offset);
	ad9361_spi_writef(spi, RX_VCO_VARACTOR_CONTROL_1 + offs,
			       MASK_VCO_VARACTOR_CONTROL1,
			       tab[i].VCO_Varactor_Reference);

	ad9361_spi_writef(spi, RX_VCO_CAL_REF, 0x07, 0);
	ad9361_spi_writef(spi, RX_VCO_VARACTOR_CONTROL_0, 0x7F, 0x70);

	ad9361_spi_writef(spi, RX_CP_CURRENT + offs, MASK_CP_CURRENT,
			       tab[i].Charge_Pump_Current);
	ad9361_spi_write(spi, RX_LOOP_FILTER_1 + offs,
			 (tab[i].LF_C2 << 4) | tab[i].LF_C1);
	ad9361_spi_write(spi, RX_LOOP_FILTER_2 + offs,
			 (tab[i].LF_R1 << 4) | tab[i].LF_C3);
	ad9361_spi_writef(spi, RX_LOOP_FILTER_3 + offs,
			       MASK_LOOP_FILTER3, tab[i].LF_R3);

	return 0;
}

static int ad9361_get_split_table_gain(struct ad9361_rf_phy *phy, u32 idx_reg,
		struct rf_rx_gain *rx_gain)
{
	struct spi_device *spi = phy->spi;
	u32 val, tbl_addr, lna_index, tia_index, mixer_index;
	int rc = 0;

	rx_gain->lmt_index = ad9361_spi_readf(spi, idx_reg, FULL_TBL_IDX_MASK);
	tbl_addr = ad9361_spi_read(spi, REG_GAIN_TBL_ADDR);

	ad9361_spi_write(spi, REG_GAIN_TBL_ADDR, rx_gain->lmt_index);

	val = ad9361_spi_read(spi, REG_GAIN_TBL_READ_DATA1);
	lna_index = (val & LNA_GAIN_MASK) >> LNA_SHIFT;
	mixer_index = (val & MIXER_GAIN_MASK) >> MIXER_SHIFT;

	tia_index = ad9361_spi_readf(spi, REG_GAIN_TBL_READ_DATA2, TIA_GAIN_MASK);

	rx_gain->lmt_gain = lna_table[lna_index] +
				mixer_table[mixer_index] +
				tia_table[tia_index];

	ad9361_spi_write(spi, REG_GAIN_TBL_ADDR, tbl_addr);

	/* Read LPF Index */
	rx_gain->lpf_gain = ad9361_spi_readf(spi, idx_reg + 1, LPF_IDX_MASK);

	/* Read Digital Gain */
	rx_gain->digital_gain = ad9361_spi_readf(spi, idx_reg + 2, DIGITAL_IDX_MASK);

	rx_gain->gain_db = rx_gain->lmt_gain + rx_gain->lpf_gain +
				rx_gain->digital_gain;
	return rc;
}

static int ad9361_get_full_table_gain(struct ad9361_rf_phy *phy, u32 idx_reg,
		struct rf_rx_gain *rx_gain)
{
	struct spi_device *spi = phy->spi;
	u32 val;
	enum rx_gain_table_name tbl;
	struct rx_gain_info *gain_info;
	int rc = 0, rx_gain_db;

	tbl = ad9361_gt_tableindex(
		ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL])));

	val = ad9361_spi_readf(spi, idx_reg, FULL_TBL_IDX_MASK);

	gain_info = &phy->rx_gain[tbl];
	if (val > gain_info->idx_step_offset) {
		val = val - gain_info->idx_step_offset;
		rx_gain_db = gain_info->starting_gain_db +
			((val) * gain_info->gain_step_db);
	} else {
		rx_gain_db = gain_info->starting_gain_db;
	}

	rx_gain->gain_db = rx_gain_db;

	return rc;
}
static int ad9361_get_rx_gain(struct ad9361_rf_phy *phy,
		unsigned rx_id, struct rf_rx_gain *rx_gain)
{
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	u32 val, idx_reg;
	u8 gain_ctl_shift, rx_enable_mask;
	u8 fast_atk_shift;
	int rc = 0;

	if (rx_id == 1) {
		gain_ctl_shift = RX1_GAIN_CTL_SHIFT;
		idx_reg = REG_RX1_FULL_TBL_IDX;
		rx_enable_mask = RX1_EN;
		fast_atk_shift = RX1_FAST_ATK_SHIFT;

	} else if (rx_id == 2) {
		gain_ctl_shift = RX2_GAIN_CTL_SHIFT;
		idx_reg = REG_RX2_FULL_TBL_IDX;
		rx_enable_mask = RX2_EN;
		fast_atk_shift = RX2_FAST_ATK_SHIFT;
	} else {
		dev_err(dev, "Unknown Rx path %d\n", rx_id);
		rc = -EINVAL;
		goto out;
	}

	val = ad9361_spi_readf(spi, REG_RXEN_N_FILTER_CTRL, rx_enable_mask);

	if (!val) {
		dev_err(dev, "Rx%d is not enabled\n", rx_gain->ant);
		rc = -EAGAIN;
		goto out;
	}

	val = ad9361_spi_read(spi, REG_AGC_CONF1);

	val = (val >> gain_ctl_shift) & RX_GAIN_CTL_MASK;

	if (val == RX_GAIN_CTL_AGC_FAST_ATK) {
		/* In fast attack mode check whether Fast attack state machine
		 * has locked gain, if not then we can not read gain.
		 */
		val = ad9361_spi_read(spi, REG_FAST_ATK_STATE);
		val = (val >> fast_atk_shift) & FAST_ATK_MASK;
		if (val != FAST_ATK_GAIN_LOCKED) {
			dev_err(dev, "Failed to read gain, state m/c at %x\n",
				val);
			rc = -EAGAIN;
			goto out;
		}
	}

	val = ad9361_spi_read(spi, REG_AGC_CONF2);

	if (val & FULL_GAIN_TBL)
		rc = ad9361_get_full_table_gain(phy, idx_reg, rx_gain);
	else
		rc = ad9361_get_split_table_gain(phy, idx_reg, rx_gain);

out:
	return rc;
}

static void ad9361_ensm_force_state(struct ad9361_rf_phy *phy, u8 ensm_state)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	u8 dev_ensm_state;
	int rc;
	u32 val;

	dev_ensm_state = ad9361_spi_readf(spi, REG_DEV_STATE, ENSM_STATE_MASK);

	phy->prev_ensm_state = dev_ensm_state;

	if (dev_ensm_state == ensm_state) {
		dev_dbg(dev, "Nothing to do, device is already in %d state\n",
			ensm_state);
		goto out;
	}

	dev_dbg(dev, "Device is in %x state, forcing to %x\n", dev_ensm_state,
			ensm_state);

	val = ad9361_spi_read(spi, REG_ENSM_CONF1);


	/* Enable control through SPI writes, and take out from
	 * Alert
	 */
	if (val & ENSM_CONF1_ENSM_PIN_CTL_EN) {
		val &= ~ENSM_CONF1_ENSM_PIN_CTL_EN;
		phy->ensm_pin_ctl_en = 1;
	} else {
		phy->ensm_pin_ctl_en = 0;
	}

	if (dev_ensm_state & dev_ensm_state)
		val &= ~(ENSM_CONF1_TO_ALERT);

	switch (ensm_state) {

	case ENSM_STATE_TX:
		val |= ENSM_CONF1_FORCE_TX_ON;
		break;
	case ENSM_STATE_RX:
		val |= ENSM_CONF1_FORCE_RX_ON;
		break;
	case ENSM_STATE_FDD:
		val |= (ENSM_CONF1_FORCE_TX_ON | ENSM_CONF1_FORCE_RX_ON);
		break;
	case ENSM_STATE_ALERT:
		val &= ~(ENSM_CONF1_FORCE_TX_ON | ENSM_CONF1_FORCE_RX_ON);
		val |= ENSM_CONF1_TO_ALERT | ENSM_CONF1_FORCE_ALERT;
		break;
	default:
		dev_err(dev, "No handling for forcing %d ensm state\n",
		ensm_state);
		goto out;
	}

	rc = ad9361_spi_write(spi, REG_ENSM_CONF1, val);
	if (rc)
		dev_err(dev, "Failed to restore state\n");

out:
	return;

}

static void ad9361_ensm_restore_prev_state(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	int rc;
	u32 val;

	val = ad9361_spi_read(spi, REG_ENSM_CONF1);


	/* We are restoring state only, so clear State bits first
	 * which might have set while forcing a particular state
	 */
	val &= ~(ENSM_CONF1_FORCE_TX_ON | ENSM_CONF1_FORCE_RX_ON |
			ENSM_CONF1_TO_ALERT | ENSM_CONF1_FORCE_ALERT);

	switch (phy->prev_ensm_state) {

	case ENSM_STATE_TX:
		val |= ENSM_CONF1_FORCE_TX_ON;
		break;
	case ENSM_STATE_RX:
		val |= ENSM_CONF1_FORCE_RX_ON;
		break;
	case ENSM_STATE_FDD:
		val |= (ENSM_CONF1_FORCE_TX_ON | ENSM_CONF1_FORCE_RX_ON);
		break;
	case ENSM_STATE_ALERT:
		val |= ENSM_CONF1_TO_ALERT;
		break;
	case ENSM_STATE_INVALID:
		dev_dbg(dev, "No need to restore, ENSM state wasn't saved\n");
		goto out;
	default:
		dev_dbg(dev, "Could not restore to %d ENSM state\n",
		phy->prev_ensm_state);
		goto out;
	}

	rc = ad9361_spi_write(spi, REG_ENSM_CONF1, val);
	if (rc) {
		dev_err(dev, "Failed to write REG_ENSM_CONF1");
		goto out;
	}

	if (phy->ensm_pin_ctl_en) {
		val |= ENSM_CONF1_ENSM_PIN_CTL_EN;
		rc = ad9361_spi_write(spi, REG_ENSM_CONF1, val);
		if (rc)
			dev_err(dev, "Failed to write REG_ENSM_CONF1");
	}

out:
	return;
}

static int set_split_table_gain(struct ad9361_rf_phy *phy, u32 idx_reg,
		struct rf_rx_gain *rx_gain)
{
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	u32 val;
	int rc = 0;

	if ((rx_gain->lmt_index > MAX_LMT_INDEX) ||
			(rx_gain->lpf_gain > MAX_LPF_GAIN) ||
			(rx_gain->digital_gain > MAX_DIG_GAIN)) {
		dev_err(dev, "LMT_INDEX missing or greater than max value %d",
				MAX_LMT_INDEX);
		dev_err(dev, "LPF_GAIN missing or greater than max value %d",
				MAX_LPF_GAIN);
		dev_err(dev, "DIGITAL_GAIN cannot be more than %d",
				MAX_DIG_GAIN);
		rc = -EINVAL;
		goto out;
	}
	if (rx_gain->gain_db > 0)
		dev_dbg(dev, "Ignoring rx_gain value in split table mode.");
	if (rx_gain->lmt_index == 0 && rx_gain->lpf_gain == 0 &&
			rx_gain->digital_gain == 0) {
		dev_err(dev,
		"In split table mode, All LMT/LPF/digital gains cannot be 0");
		rc = -EINVAL;
		goto out;
	}

	ad9361_spi_writef(spi, idx_reg, FULL_TBL_IDX_MASK, rx_gain->lmt_index);

	ad9361_spi_writef(spi, idx_reg + 1, LPF_IDX_MASK, rx_gain->lpf_gain);

	val = ad9361_spi_read(spi, REG_AGC_CONF2);

	if (val & DIGITAL_GAIN_EN) {
		ad9361_spi_writef(spi, idx_reg + 2, DIGITAL_IDX_MASK, rx_gain->digital_gain);

	} else if (rx_gain->digital_gain > 0) {
		dev_err(dev, "Digital gain is disabled and cannot be set");
	}
out:
	return rc;
}

static int set_full_table_gain(struct ad9361_rf_phy *phy, u32 idx_reg,
		struct rf_rx_gain *rx_gain)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	enum rx_gain_table_name tbl;
	struct rx_gain_info *gain_info;
	u32 val;
	int rc = 0;

	if (rx_gain->lmt_index != ~0 || rx_gain->lpf_gain != ~0 ||
			rx_gain->digital_gain > 0)
		dev_dbg(dev,
			"Ignoring lmt/lpf/digital gains in Single Table mode");

	tbl = ad9361_gt_tableindex(
		ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL])));

	gain_info = &phy->rx_gain[tbl];
	if ((rx_gain->gain_db < gain_info->starting_gain_db) ||
		(rx_gain->gain_db > gain_info->max_gain_db)) {

		dev_err(dev, "Invalid gain %d, supported range [%d - %d]\n",
			rx_gain->gain_db, gain_info->starting_gain_db,
			gain_info->max_gain_db);
		rc = -EINVAL;
		goto out;

	}

	val = ((rx_gain->gain_db - gain_info->starting_gain_db) /
		gain_info->gain_step_db) + gain_info->idx_step_offset;
	ad9361_spi_writef(spi, idx_reg, FULL_TBL_IDX_MASK, val);

out:
	return rc;
}

static int ad9361_set_rx_gain(struct ad9361_rf_phy *phy,
		unsigned rx_id, struct rf_rx_gain *rx_gain)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	u32 val, idx_reg;
	u8 gain_ctl_shift, ensm_state;
	int rc = 0;

	if (rx_id == 1) {
		gain_ctl_shift = RX1_GAIN_CTL_SHIFT;
		idx_reg = REG_RX1_MGC_FULL_TBL_IDX;

	} else if (rx_id == 2) {
		gain_ctl_shift = RX2_GAIN_CTL_SHIFT;
		idx_reg = REG_RX2_MGC_FULL_TBL_IDX;
	} else {
		dev_err(dev, "Unknown Rx path %d\n", rx_id);
		rc = -EINVAL;
		goto out;

	}

	val = ad9361_spi_read(spi, REG_AGC_CONF1);
	val = (val >> gain_ctl_shift) & RX_GAIN_CTL_MASK;

	if (val != RX_GAIN_CTL_MGC) {
		dev_err(dev, "Rx gain can be set in MGC mode only\n");
		goto out;
	}

	if (phy->pdata->fdd)
		ensm_state = ENSM_STATE_FDD;
	else
		ensm_state = ENSM_STATE_RX;

	/* RX must be enabled while changing Gain */
	ad9361_ensm_force_state(phy, ensm_state);

	val = ad9361_spi_read(spi, REG_AGC_CONF2);
	if (val & FULL_GAIN_TBL)
		rc = set_full_table_gain(phy, idx_reg, rx_gain);
	else
		rc = set_split_table_gain(phy, idx_reg, rx_gain);

	/* Restore is done intentionally before checking rc, because
	 * we need to restore PHY to previous state even if write failed
	 */
	ad9361_ensm_restore_prev_state(phy);

	if (rc) {
		dev_err(dev, "Unable to write gain tbl idx reg: %d\n", idx_reg);
		goto out;
	}

out:
	return rc;

}

void ad9361_init_gain_info(struct rx_gain_info *rx_gain,
	enum rx_gain_table_type type, int starting_gain,
	int max_gain, int gain_step, int max_idx, int idx_offset)
{
	rx_gain->tbl_type = type;
	rx_gain->starting_gain_db = starting_gain;
	rx_gain->max_gain_db = max_gain;
	rx_gain->gain_step_db = gain_step;
	rx_gain->max_idx = max_idx;
	rx_gain->idx_step_offset = idx_offset;
}

int ad9361_init_gain_tables(struct ad9361_rf_phy *phy)
{
	struct rx_gain_info *rx_gain;

	/* Intialize Meta data according to default gain tables
	 * of AD9631. Changing/Writing of gain tables is not
	 * supported yet.
	 */
	rx_gain = &phy->rx_gain[TBL_200_1300_MHZ];
	ad9361_init_gain_info(rx_gain, RXGAIN_FULL_TBL, 1, 77, 1,
		RXGAIN_FULL_TBL_MAX_IDX, 0);

	rx_gain = &phy->rx_gain[TBL_1300_4000_MHZ];
	ad9361_init_gain_info(rx_gain, RXGAIN_FULL_TBL, -4, 71, 1,
		RXGAIN_FULL_TBL_MAX_IDX, 1);

	rx_gain = &phy->rx_gain[TBL_4000_6000_MHZ];
	ad9361_init_gain_info(rx_gain, RXGAIN_FULL_TBL, -10, 62, 1,
		RXGAIN_FULL_TBL_MAX_IDX, 4);

	return 0;
}

int ad9361_en_dis_tx(struct ad9361_rf_phy *phy, u32 tx_if, u32 enable)
{
	return ad9361_spi_writef(phy->spi, TX_ENABLE_REG,
			  (tx_if == 1) ? TX1_ENABLE_MASK : TX2_ENABLE_MASK,
			  enable);
}

int ad9361_en_dis_rx(struct ad9361_rf_phy *phy, u32 rx_if, u32 enable)
{
	return ad9361_spi_writef(phy->spi, RX_ENABLE_REG,
			  (rx_if == 1) ? RX1_ENABLE_MASK : RX2_ENABLE_MASK,
			  enable);
}


int ad9361_set_gain_ctrl_mode(struct ad9361_rf_phy *phy,
		struct rf_gain_ctrl *gain_ctrl)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	int rc = 0;
	u32 gain_ctl_shift, mode;
	unsigned char val;

	rc = ad9361_spi_readm(spi, REG_AGC_CONF1, &val, 1);
	if (rc) {
		dev_err(dev, "Unable to read AGC config1 register: %x\n",
				REG_AGC_CONF1);
		goto out;
	}

	switch (gain_ctrl->mode) {
	case RF_GAIN_MGC:
		mode = RX_GAIN_CTL_MGC;
		break;
	case RF_GAIN_FASTATTACK_AGC:
		mode = RX_GAIN_CTL_AGC_FAST_ATK;
		break;
	case RF_GAIN_SLOWATTACK_AGC:
		mode = RX_GAIN_CTL_AGC_SLOW_ATK;
		break;
	case RF_GAIN_HYBRID_AGC:
		mode = RX_GAIN_CTL_AGC_SLOW_ATK_HYBD;
		break;
	default:
		rc = -EINVAL;
		goto out;
	}

	if (gain_ctrl->ant == 1) {
		gain_ctl_shift = RX1_GAIN_CTL_SHIFT;
	} else if (gain_ctrl->ant == 2) {
		gain_ctl_shift = RX2_GAIN_CTL_SHIFT;
	} else {
		dev_err(dev, "Unknown Rx path %d\n", gain_ctrl->ant);
		rc = -EINVAL;
		goto out;
	}

	rc = ad9361_en_dis_rx(phy, gain_ctrl->ant, RX_DISABLE);
	if (rc) {
		dev_err(dev, "Unable to disable rx%d\n", gain_ctrl->ant);
		goto out;
	}

	val &= ~(RX_GAIN_CTL_MASK << gain_ctl_shift);
	val |= mode << gain_ctl_shift;
	if (mode == RX_GAIN_CTL_AGC_SLOW_ATK_HYBD)
		val |= SLOW_ATK_HYBD_BIT_EN;
	else
		val &= ~SLOW_ATK_HYBD_BIT_EN;

	rc = ad9361_spi_write(spi, REG_AGC_CONF1, val);
	if (rc) {
		dev_err(dev, "Unable to write AGC config1 register: %x\n",
				REG_AGC_CONF1);
		goto out;
	}

	rc = ad9361_en_dis_rx(phy, gain_ctrl->ant, RX_ENABLE);
out:
	return rc;
}

static int ad9361_read_rssi(struct ad9361_rf_phy *phy, struct rf_rssi *rssi)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	u32 weight[4], total_dur, val, temp;
	u32 subframe_size[4] = {SUBFRAME_SIZE_5MHZ,
				SUBFRAME_SIZE_10MHZ,
				SUBFRAME_SIZE_15MHZ,
				SUBFRAME_SIZE_20MHZ};
	u8 dur_buf[4] = {0}, i, j, index;
	u8 reg_val_buf[6];
	int rc;

	if (rssi->duration > 1) {
		dev_err(dev, "RSSI measurement duration > 1ms not supported\n");
		rc = -EINVAL;
		goto out;
	}

// 	val = ad9361_spi_read(spi, RSSI_CONFIG_REG);
// 	val &= RSSI_MEAS_MODE_MASK;
// 	val |= RSSI_GAIN_CHANGE_EN_AGC_MODE;
// 	rc = ad9361_spi_write(spi, RSSI_CONFIG_REG, val);

// 	if (rc) {
// 		dev_err(dev, "Unable to read/write rssi config reg\n");
// 		goto out;
// 	}
//
// 	index = clamp(DIV_ROUND_CLOSEST(phy->current_bw_Hz, 5000000UL), 0, 3);
//
// 	temp = subframe_size[index] * rssi->duration;
// 	for (i = 0, j = 0; temp != 0 && j < 4; i++) {
// 		if (temp & 0x01)
// 			dur_buf[j++] = i;
// 		temp = temp >> 1;
// 	}
//
// 	total_dur = (1 << dur_buf[0]) + (1 << dur_buf[1]) +
// 			(1 << dur_buf[2]) + (1 << dur_buf[3]);
// 	weight[0] = RSSI_MAX_WEIGHT * (1 << dur_buf[0]) / total_dur;
// 	weight[1] = RSSI_MAX_WEIGHT * (1 << dur_buf[1]) / total_dur;
// 	weight[2] = RSSI_MAX_WEIGHT * (1 << dur_buf[2]) / total_dur;
// 	weight[3] = RSSI_MAX_WEIGHT * (1 << dur_buf[3]) / total_dur;
// 	rc = ad9361_spi_write(spi, RSSI_MEAS_DUR_10_REG,
// 			((dur_buf[1] << 4) | dur_buf[0])) ||
// 		ad9361_spi_write(spi, RSSI_MEAS_DUR_32_REG,
// 			((dur_buf[3] << 4) | dur_buf[2])) ||
// 		ad9361_spi_write(spi, RSSI_WEIGHT0_REG, weight[0]) ||
// 		ad9361_spi_write(spi, RSSI_WEIGHT1_REG, weight[1]) ||
// 		ad9361_spi_write(spi, RSSI_WEIGHT2_REG, weight[2]) ||
// 		ad9361_spi_write(spi, RSSI_WEIGHT3_REG, weight[3]);
// 	if (rc) {
// 		dev_err(dev, "Unable to write rssi measurement duration\n");
// 		goto out;
// 	}

	rc = ad9361_spi_readm(spi, PREAMBLE_LSB,
			reg_val_buf, ARRAY_SIZE(reg_val_buf));


	if (rssi->ant == 1) {
		rssi->symbol = RSSI_RESOLUTION *
				((reg_val_buf[5] << LSB_SHIFT) +
				 (reg_val_buf[1] & RSSI_LSB_MASK1));
		rssi->preamble = RSSI_RESOLUTION *
				((reg_val_buf[4] << LSB_SHIFT) +
				 (reg_val_buf[0] & RSSI_LSB_MASK1));
	} else if (rssi->ant == 2) {
		rssi->symbol = RSSI_RESOLUTION *
				((reg_val_buf[3] << LSB_SHIFT) +
				 ((reg_val_buf[1] & RSSI_LSB_MASK2) >> 1));
		rssi->preamble = RSSI_RESOLUTION *
				((reg_val_buf[2] << LSB_SHIFT) +
				 ((reg_val_buf[0] & RSSI_LSB_MASK2) >> 1));
	} else
		rc = -EFAULT;

	rssi->multiplier = RSSI_MULTIPLIER;

out:
	return rc;
}

static int ad9361_rx_adc_setup(struct ad9361_rf_phy *phy, unsigned long bb_bw_Hz,
			 unsigned long adc_sampl_freq_Hz)
{

	unsigned long scale_snr_1e3, maxsnr, sqrt_inv_rc_tconst_1e3, tmp_1e3,
		scaled_adc_clk_1e6, inv_scaled_adc_clk_1e3, sqrt_term_1e3,
		min_sqrt_term_1e3;
	unsigned long long tmp, invrc_tconst_1e6;
	unsigned char data[40];
	unsigned i;
	int ret;

	unsigned char c3_msb = ad9361_spi_read(phy->spi, RX_BBF_C3_MSB);
	unsigned char c3_lsb = ad9361_spi_read(phy->spi, RX_BBF_C3_LSB);
	unsigned char r2346 = ad9361_spi_read(phy->spi, RX_BBF_R2346);

	dev_dbg(&phy->spi->dev, "%s : BBBW %lu : ADCfreq %lu",
		__func__, bb_bw_Hz, adc_sampl_freq_Hz);

	dev_dbg(&phy->spi->dev, "c3_msb 0x%X : c3_lsb 0x%X : r2346 0x%X : ",
		c3_msb, c3_lsb, r2346);

	bb_bw_Hz = clamp(bb_bw_Hz, 200000UL, 28000000UL);

	if (adc_sampl_freq_Hz < 80000000)
		scale_snr_1e3 = 1000;
	else
		scale_snr_1e3 = 1585; /* pow(10, scale_snr_dB/10); */

 	if (bb_bw_Hz >= 18000000) {
		invrc_tconst_1e6 = (160975ULL * r2346 *
			(160 * c3_msb + 10 * c3_lsb + 140) *
			(bb_bw_Hz)  * (1000 + (10 * (bb_bw_Hz - 18000000) / 1000000)));

		do_div(invrc_tconst_1e6, 1000UL);

	} else {
		invrc_tconst_1e6 = (160975ULL * r2346 *
			(160 * c3_msb + 10 * c3_lsb + 140) *
			(bb_bw_Hz));
	}

	do_div(invrc_tconst_1e6, 1000000000UL);

	if (invrc_tconst_1e6 > ULONG_MAX)
		dev_err(&phy->spi->dev, "invrc_tconst_1e6 > ULONG_MAX");

	sqrt_inv_rc_tconst_1e3 = int_sqrt((unsigned int)invrc_tconst_1e6);
	maxsnr = 640/160;
	scaled_adc_clk_1e6 = DIV_ROUND_CLOSEST(adc_sampl_freq_Hz, 640);
	inv_scaled_adc_clk_1e3 = DIV_ROUND_CLOSEST(640000000,
			DIV_ROUND_CLOSEST(adc_sampl_freq_Hz, 1000));
	tmp_1e3 = DIV_ROUND_CLOSEST(980000 + 20 * max_t(unsigned, 1000U,
			DIV_ROUND_CLOSEST(inv_scaled_adc_clk_1e3, maxsnr)), 1000);
	sqrt_term_1e3 = int_sqrt(scaled_adc_clk_1e6);
	min_sqrt_term_1e3 = min_t(unsigned, 1000U,
			int_sqrt(maxsnr * scaled_adc_clk_1e6));


	dev_dbg(&phy->spi->dev, "invrc_tconst_1e6 %llu, sqrt_inv_rc_tconst_1e3 %lu\n",
		invrc_tconst_1e6, sqrt_inv_rc_tconst_1e3);
	dev_dbg(&phy->spi->dev, "scaled_adc_clk_1e6 %lu, inv_scaled_adc_clk_1e3 %lu\n",
		scaled_adc_clk_1e6, inv_scaled_adc_clk_1e3);
	dev_dbg(&phy->spi->dev, "tmp_1e3 %lu, sqrt_term_1e3 %lu, min_sqrt_term_1e3 %lu\n",
		tmp_1e3, sqrt_term_1e3, min_sqrt_term_1e3);


	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0x24;
	data[4] = 0x24;
	data[5] = 0;
	data[6] = 0;

	tmp = -50000000 + 8ULL * scale_snr_1e3 * sqrt_inv_rc_tconst_1e3 *
		min_sqrt_term_1e3;
	do_div(tmp, 100000000UL);
	data[7] = min_t(unsigned long long, 124U, tmp);

	tmp = (invrc_tconst_1e6 >> 1) + 20 * inv_scaled_adc_clk_1e3 *
		data[7] / 80 * 1000ULL;
	do_div(tmp, invrc_tconst_1e6);
	data[8] = min_t(unsigned long long, 255U, tmp);

	tmp = (-500000 + 77ULL * sqrt_inv_rc_tconst_1e3 * min_sqrt_term_1e3);
	do_div(tmp, 1000000UL);
	data[10] = min_t(unsigned long long, 127U, tmp);

	data[9] = min_t(unsigned, 127U, ((800 * data[10]) / 1000));
	tmp = ((invrc_tconst_1e6 >> 1) + (20 * inv_scaled_adc_clk_1e3 *
		data[10] * 1000ULL));
	do_div(tmp, invrc_tconst_1e6 * 77);
	data[11] = min_t(unsigned long long, 255U, tmp);
	data[12] = min_t(unsigned, 127U, (-500000 + 80 * sqrt_inv_rc_tconst_1e3 *
		min_sqrt_term_1e3) / 1000000UL);

	tmp = -3*(long)(invrc_tconst_1e6 >> 1) + inv_scaled_adc_clk_1e3 *
		data[12] * (1000ULL * 20 / 80);
	do_div(tmp, invrc_tconst_1e6);
	data[13] = min_t(unsigned long long, 255, tmp);

	data[14] = 21 * (inv_scaled_adc_clk_1e3 / 10000);
	data[15] = min_t(unsigned, 127U, (500 + 1025 * data[7]) / 1000);
	data[16] = min_t(unsigned, 127U, (data[15] * tmp_1e3) / 1000);
	data[17] = data[15];
	data[18] = min_t(unsigned, 127U, (500 + 975 * data[10]) / 1000);
	data[19] = min_t(unsigned, 127U, (data[18] * tmp_1e3) / 1000);
	data[20] = data[18];
	data[21] = min_t(unsigned, 127U, (500 + 975 * data[12]) / 1000);
	data[22] = min_t(unsigned, 127, (data[21] * tmp_1e3) / 1000);
	data[23] = data[21];
	data[24] = 0x2E;
	data[25] = (128 + min_t(unsigned, 63000U, DIV_ROUND_CLOSEST(63 *
		scaled_adc_clk_1e6, 1000)) / 1000);
	data[26] = min_t(unsigned, 63U,63 * scaled_adc_clk_1e6 / 1000000 *
		(920 + 80 * inv_scaled_adc_clk_1e3 / 1000) / 1000);
	data[27] = min_t(unsigned, 63,(32 * sqrt_term_1e3) / 1000);
	data[28] = data[25];
	data[29] = data[26];
	data[30] = data[27];
	data[31] = data[25];
	data[32] = data[26];
	data[33] = min_t(unsigned, 63U, 63 * sqrt_term_1e3 / 1000);
	data[34] = min_t(unsigned, 127U, 64 * sqrt_term_1e3 / 1000);
	data[35] = 0x40;
	data[36] = 0x40;
	data[37] = 0x2C;
	data[38] = 0x00;
	data[39] = 0x00;

 	for (i = 0; i < 40; i++) {
		ret = ad9361_spi_write(phy->spi, 0x200 + i, data[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9361_rx_tia_calib(struct ad9361_rf_phy *phy, unsigned long bb_bw_Hz)
{
	unsigned long Cbbf, R2346;
	unsigned long long CTIA_fF;

	unsigned char reg1EB = ad9361_spi_read(phy->spi, RX_BBF_C3_MSB);
	unsigned char reg1EC = ad9361_spi_read(phy->spi, RX_BBF_C3_LSB);
	unsigned char reg1E6 = ad9361_spi_read(phy->spi, RX_BBF_R2346);
	unsigned char reg1DB, reg1DF, reg1DD, reg1DC, reg1DE, temp;

	dev_dbg(&phy->spi->dev, "%s : bb_bw_Hz %lu",
		__func__, bb_bw_Hz);

	bb_bw_Hz = clamp(bb_bw_Hz, 200000UL, 20000000UL);

	Cbbf = (reg1EB * 160) + (reg1EC * 10) + 140; /* fF */
	R2346 = 18300 * (reg1E6 & 0x7);

	CTIA_fF = Cbbf * R2346 * 560ULL;
	do_div(CTIA_fF, 3500000UL);

	if (bb_bw_Hz <= 3000000UL)
		reg1DB = 0xE0;
	else if (bb_bw_Hz <= 10000000UL)
		reg1DB = 0x60;
	else
		reg1DB = 0x20;

	if (CTIA_fF > 2920ULL) {
		reg1DC = 0x40;
		reg1DE = 0x40;
		temp = min(127U, DIV_ROUND_CLOSEST((u32)CTIA_fF - 400, 320U));
		reg1DD = temp;
		reg1DF = temp;
	} else {
		temp = DIV_ROUND_CLOSEST((u32)CTIA_fF - 400, 40U) + 0x40;
		reg1DC = temp;
		reg1DE = temp;
		reg1DD = 0;
		reg1DF = 0;
	}

	ad9361_spi_write(phy->spi, RX_TIA_CONFIG, reg1DB);
	ad9361_spi_write(phy->spi, TIA1_C_LSB, reg1DC);
	ad9361_spi_write(phy->spi, TIA1_C_MSB, reg1DD);
	ad9361_spi_write(phy->spi, TIA2_C_LSB, reg1DE);
	ad9361_spi_write(phy->spi, TIA2_C_MSB, reg1DF);

	return 0;
}

/* BASEBAND RX ANALOG FILTER CALIBRATION */

static int ad9361_rx_bb_analog_filter_calib(struct ad9361_rf_phy *phy,
					    unsigned long rx_bb_bw,
					    unsigned long bbpll_freq)
{
	unsigned long target, rxbbf_div;
	unsigned char tmp;
	int ret;

	dev_dbg(&phy->spi->dev, "%s : rx_bb_bw %lu bbpll_freq %lu",
		__func__, rx_bb_bw, bbpll_freq);

	rx_bb_bw = clamp(rx_bb_bw, 200000UL, 28000000UL);

	/* 1.4 * BBBW * 2PI / ln(2) */
	target =  126906 * (rx_bb_bw / 10000UL);
	rxbbf_div = min_t(unsigned long, 511UL, DIV_ROUND_UP(bbpll_freq, target));

	/* Set RX baseband filter divide value */
	ad9361_spi_write(phy->spi, RX_BBF_TUNE_DIVIDE, rxbbf_div);
	ad9361_spi_writef(phy->spi, RX_BBF_TUNE_CONFIG, BIT(0), rxbbf_div >> 8);

	/* Write the BBBW into registers 0x1FB and 0x1FC */
	ad9361_spi_write(phy->spi, RX_BBBW_MHZ, rx_bb_bw / 1000000UL);

	tmp = DIV_ROUND_CLOSEST((rx_bb_bw % 1000000UL) * 128, 1000000UL);
	ad9361_spi_write(phy->spi, RX_BBBW_KHZ, min_t(unsigned char, 127, tmp));

	ad9361_spi_write(phy->spi,RX_MIX_LO_CM, 0x3F); /* Set Rx Mix LO CM */
	ad9361_spi_write(phy->spi,RX_MIX_GM_CONFIG, 0x03); /* Set GM common mode */

	/* Enable the RX BBF tune circuit by writing 0x1E2=0x02 and 0x1E3=0x02 */
	ad9361_spi_write(phy->spi,RX1_TUNE_CONTROL, 0x02);
	ad9361_spi_write(phy->spi,RX2_TUNE_CONTROL, 0x02);

	/* Start the RX Baseband Filter calibration in register 0x016[7] */
	/* Calibration is complete when register 0x016[7] self clears */
	ret = ad9361_run_calibration(phy, RX_BB_TUNE_CAL);

	/* Disable the RX baseband filter tune circuit, write 0x1E2=3, 0x1E3=3 */
	ad9361_spi_write(phy->spi,RX1_TUNE_CONTROL, 0x03);
	ad9361_spi_write(phy->spi,RX2_TUNE_CONTROL, 0x03);

	return ret;
}

/* BASEBAND TX ANALOG FILTER CALIBRATION */

static int ad9361_tx_bb_analog_filter_calib(struct ad9361_rf_phy *phy,
					    unsigned long tx_bb_bw,
					    unsigned long bbpll_freq)
{
	unsigned long target, txbbf_div;
	int ret;

	dev_dbg(&phy->spi->dev, "%s : tx_bb_bw %lu bbpll_freq %lu",
		__func__, tx_bb_bw, bbpll_freq);

	tx_bb_bw = clamp(tx_bb_bw, 625000UL, 20000000UL);

	/* 1.6 * BBBW * 2PI / ln(2) */
	target =  132345 * (tx_bb_bw / 10000UL);
	txbbf_div = min_t(unsigned long, 511UL, DIV_ROUND_UP(bbpll_freq, target));

	/* Set TX baseband filter divide value */
	ad9361_spi_write(phy->spi, TX_BBF_TUNE_DIVIDER, txbbf_div);
	ad9361_spi_writef(phy->spi, TX_BBF_TUNE_MODE, BIT(0), txbbf_div >> 8);

	/* Enable the TX baseband filter tune circuit by setting 0x0CA=0x22. */
	ad9361_spi_write(phy->spi,TX_TUNE_CONTROL, 0x22);

	/* Start the TX Baseband Filter calibration in register 0x016[6] */
	/* Calibration is complete when register 0x016[] self clears */
	ret = ad9361_run_calibration(phy, TX_BB_TUNE_CAL);

	/* Disable the TX baseband filter tune circuit by writing 0x0CA=0x26. */
	ad9361_spi_write(phy->spi, TX_TUNE_CONTROL, 0x26);

	return ret;
}

/* BASEBAND TX SECONDARY FILTER */

static int ad9361_tx_bb_second_filter_calib(struct ad9361_rf_phy *phy,
					   unsigned long tx_rf_bw)
{
	unsigned long long cap;
	unsigned long corner, res, div;
	unsigned reg_conf, reg_res;
	int ret, i;

	dev_dbg(&phy->spi->dev, "%s : tx_rf_bw %lu",
		__func__, tx_rf_bw);

	tx_rf_bw = clamp(tx_rf_bw, 1060000UL, 40000000UL);

	/* BBBW * 5PI */
	corner = 15708 * (tx_rf_bw / 20000UL);

	for (i = 0, res = 1; i < 4; i++) {
		div = corner * res;
		cap = (500000000ULL) + (div >> 1);
		do_div(cap, div);
		cap -= 12ULL;
		if (cap < 64ULL)
			break;

		res <<= 1;
	}

	if (cap > 63ULL)
		cap = 63ULL;

	if(tx_rf_bw <= 9000000UL )
		reg_conf = 0x59;
	else if (tx_rf_bw <= 24000000UL)
		reg_conf = 0x56;
	else
		reg_conf = 0x57;

	switch (res) {
	case 1:
		reg_res = 0x0C;
		break;
	case 2:
		reg_res = 0x04;
		break;
	case 4:
		reg_res = 0x03;
		break;
	case 8:
		reg_res = 0x01;
		break;
	default:
		reg_res = 0x01;
	}

	ret = ad9361_spi_write(phy->spi, CONFIG0, reg_conf);
	ret |= ad9361_spi_write(phy->spi, RESISTOR, reg_res);
	ret |= ad9361_spi_write(phy->spi, CAPACITOR, (u8)cap);

	return ret;
}

/* RF SYNTHESIZER CHARGE PUMP CALIBRATION */

static int ad9361_txrx_synth_cp_calib(struct ad9361_rf_phy *phy,
					   unsigned long ref_clk_hz, bool tx)
{
	unsigned offs = tx ? 0x40 : 0;
	unsigned vco_cal_cnt;

	dev_dbg(&phy->spi->dev, "%s : ref_clk_hz %lu : is_tx %d",
		__func__, ref_clk_hz, tx);

	ad9361_spi_write(phy->spi, RX_LO_GEN_POWER_MODE + offs, 0x00);
	ad9361_spi_write(phy->spi, RX_VCO_LDO + offs, 0x0B);
	ad9361_spi_write(phy->spi, RX_VCO_PD_OVERRIDES + offs, 0x02);
	ad9361_spi_write(phy->spi, RX_VCO_BIAS_2 + offs, 0x0D);
	ad9361_spi_write(phy->spi, RX_CP_CURRENT + offs, 0x80);
	ad9361_spi_write(phy->spi, RX_CP_CONFIG + offs, 0x00);

	/* see Table 70 Example Calibration Times for RF VCO Cal */
	if (phy->pdata->fdd) {
		vco_cal_cnt = 0x8E;
	} else {
		if (ref_clk_hz >= 50000000UL)
			vco_cal_cnt = 0x86;
		else
			vco_cal_cnt = 0x82;
	}

	ad9361_spi_write(phy->spi, RX_VCO_CAL + offs, vco_cal_cnt);

	/* Enable FDD mode during calibrations */

	if (!phy->pdata->fdd)
		ad9361_spi_write(phy->spi, PARALLEL_PORT_CONFIG3, 0x10);

	ad9361_spi_write(phy->spi, ENSM_CONFIG_2, 0x04);
	ad9361_spi_write(phy->spi, ENSM_CONFIG_1, 0x05);
	ad9361_spi_write(phy->spi, ENSM_MODE, 0x01);

	ad9361_spi_write(phy->spi, RX_CP_CONFIG + offs, 0x04);

	return ad9361_check_cal_done(phy, RX_CAL_STATUS + offs, BIT(7), 1);
}

/* BASEBAND DC OFFSET CALIBRATION */
static int ad9361_bb_dc_offset_calib(struct ad9361_rf_phy *phy)
{
	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(phy->spi, BB_DC_OFFSET_COUNT, 0x3F);
	ad9361_spi_write(phy->spi, BB_DC_OFFSET_SHIFT, 0x0F);
	ad9361_spi_write(phy->spi, BB_DC_OFFSET_ATTEN, 0x01);

	return ad9361_run_calibration(phy, BBDC_CAL);
}


/* RF DC OFFSET CALIBRATION */

static int ad9361_rf_dc_offset_calib(struct ad9361_rf_phy *phy,
				     unsigned long long rx_freq)
{
	struct spi_device *spi = phy->spi;

	dev_dbg(&phy->spi->dev, "%s : rx_freq %llu",
		__func__, rx_freq);

	ad9361_spi_write(spi, ENSM_CONFIG_1, 0x15);
	ad9361_spi_write(spi, WAIT_COUNT, 0x20);

	if(rx_freq <= 4000000000ULL) {
		ad9361_spi_write(spi, RF_DC_OFFSET_COUNT, 0x32);
		ad9361_spi_write(spi, RF_DC_OFFSET_CONFIG_1, 0x24);
		ad9361_spi_write(spi, RF_DC_OFFSET_ATTEN, 0x05);
	} else {
		ad9361_spi_write(spi, RF_DC_OFFSET_COUNT, 0x28);
		ad9361_spi_write(spi, RF_DC_OFFSET_CONFIG_1, 0x34);
		ad9361_spi_write(spi, RF_DC_OFFSET_ATTEN, 0x06);
	}

	ad9361_spi_write(spi, RF_DC_OFFSET_CONFIG_2, 0x83);
	ad9361_spi_write(spi, RF_TX_PDET_COUNT2, 0x30);

	return ad9361_run_calibration(phy, RFDC_CAL);
}

/* TX QUADRATURE CALIBRATION */

static int ad9361_tx_quad_calib(struct ad9361_rf_phy *phy,
					   unsigned long bw)
{
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	unsigned long clktf, clkrf;
	int txnco_word, rxnco_word;
	unsigned char rx_phase = 0;
	const unsigned char (*tab)[3];
	unsigned index_max, i , lpf_tia_mask;
	/*
	 * Find NCO frequency that matches this equation:
	 * BW / 4 = Rx NCO freq = Tx NCO freq:
	 * Rx NCO = ClkRF * (rxNCO <1:0> + 1) / 32
	 * Tx NCO = ClkTF * (txNCO <1:0> + 1) / 32
	 */

	clkrf = clk_get_rate(phy->clks[CLKRF_CLK]);
	clktf = clk_get_rate(phy->clks[CLKTF_CLK]);

	dev_dbg(&phy->spi->dev, "%s : bw %lu clkrf %lu clktf %lu",
		__func__, bw, clkrf, clktf);

	txnco_word = DIV_ROUND_CLOSEST(bw * 8, clktf) - 1;
	txnco_word = clamp_t(int, txnco_word, 0, 3);

 	dev_dbg(dev, "Tx NCO frequency: %lu (BW/4: %lu) txnco_word %d\n",
		clktf * (txnco_word + 1) / 32, bw / 4, txnco_word);

	rxnco_word = txnco_word;

	if (clkrf == (2 * clktf)) {
		rx_phase = 0x0E;
		switch (txnco_word) {
		case 0:
			txnco_word++;
			break;
		case 1:
			rxnco_word--;
			break;
		case 2:
			rxnco_word-=2;
			txnco_word--;
			break;
		case 3:
			rxnco_word-=2;	/* REVISIT */
			rx_phase = 0x08;
			break;
		}
	} else if (clkrf == clktf) {
		switch (txnco_word) {
		case 0:
		case 3:
			rx_phase = 0x15;
			break;
		case 2:
			rx_phase = 0x1F;
			break;
		case 1:
			if (ad9361_spi_readf(spi, TX_ENABLE_FILTER, 0x3F) == 0x22)
				rx_phase = 0x15; 	/* REVISIT */
			else
				rx_phase = 0x1A;
			break;
		}
	} else
		dev_err(dev, "Error in %s line %d\n", __func__, __LINE__);

	ad9361_spi_write(spi, QUAD_CAL_NCO_FREQ_PHASE, (rxnco_word << 5) | rx_phase);
	ad9361_spi_writef(spi, KEXP_2, 0xC0, txnco_word);
	ad9361_spi_write(spi, QUAD_CAL_CONTROL, 0x7B);
	ad9361_spi_write(spi, QUAD_CAL_COUNT, 0xFF);
	ad9361_spi_write(spi, KEXP_1, 0x7F);
	ad9361_spi_write(spi, MAG_FTEST_THRESH, 0x01);
	ad9361_spi_write(spi, MAG_FTEST_THRESH_2, 0x01);

	if (phy->pdata->split_gt) {
		tab = &split_gain_table[phy->current_table][0];
		index_max = SIZE_SPLIT_TABLE;
		lpf_tia_mask = 0x20;
	} else {
		tab = &full_gain_table[phy->current_table][0];
		index_max = SIZE_FULL_TABLE;
		lpf_tia_mask = 0x3F;
	}

	for (i = 0; i < index_max; i++)
		if ((tab[i][1] & lpf_tia_mask) == 0x20) {
			ad9361_spi_write(spi, TX_QUAD_FULL_LMT_GAIN, i);
			break;
		}

	if (i >= index_max)
		dev_err(dev, "failed to find suitable LPF TIA value in gain table\n");

	ad9361_spi_write(spi, SETTLE_COUNT, 0xF0);
	ad9361_spi_write(spi, TX_QUAD_LPF_GAIN, 0x00);

	return ad9361_run_calibration(phy, TX_QUAD_CAL);
}

static int ad9361_rx_quad_calib(struct ad9361_rf_phy *phy,
					   unsigned long bw)
{
	return -EINVAL; /* TODO */
}

static int ad9361_tracking_control(struct ad9361_rf_phy *phy, bool bbdc_track,
				   bool rfdc_track, bool rxquad_track)
{
	struct spi_device *spi = phy->spi;

	dev_dbg(&spi->dev, "%s : bbdc_track=%d, rfdc_track=%d, rxquad_track=%d",
		__func__, bbdc_track, rfdc_track, rxquad_track);

	ad9361_spi_write(spi, CALIBRATION_CONFIG_2, 0x75);
	ad9361_spi_write(spi, CALIBRATION_CONFIG_3, 0x15);

	ad9361_spi_writef(spi, RF_DC_OFFSET_CONFIG_2, 0x07, 0x3); /* Gain change + Rx exit */
	ad9361_spi_writef(spi, RF_DC_OFFSET_CONFIG_2, 0x20, bbdc_track);
	ad9361_spi_writef(spi, RF_DC_OFFSET_CONFIG_2, 0x08, rfdc_track);
	ad9361_spi_writef(spi, CALIBRATION_CONFIG_1, 0x01, rxquad_track);

	if (phy->pdata->rx2tx2)
		ad9361_spi_writef(spi, CALIBRATION_CONFIG_1, 0x02, rxquad_track);

	ad9361_spi_writef(spi, CALIBRATION_CONFIG_1, 0x0C, 0x3);

	return 0;
}


/* REFERENCE CLOCK DELAY UNIT COUNTER REGISTER */
static int ad9361_set_ref_clk_cycles(struct ad9361_rf_phy *phy,
				    unsigned long ref_clk_hz)
{
	dev_dbg(&phy->spi->dev, "%s : ref_clk_hz %lu",
		__func__, ref_clk_hz);

	return ad9361_spi_write(phy->spi, REFERENCE_CLOCK_CYCLES,
				(ref_clk_hz / 1000000UL) - 1);
}

static int ad9361_set_dcxo_tune(struct ad9361_rf_phy *phy,
				    unsigned coarse, unsigned fine)
{
	dev_dbg(&phy->spi->dev, "%s : coarse %u fine %u",
		__func__, coarse, fine);

	ad9361_spi_write(phy->spi, DCXO_COARSE_TUNE, coarse);
	ad9361_spi_write(phy->spi, DCXO_FINE_TUNE_LOW, fine << 3);
	return ad9361_spi_write(phy->spi, DCXO_FINE_TUNE_HIGH,fine >> 5);
}

/* val
 * 0	(RX1A_N &  RX1A_P) and (RX2A_N & RX2A_P) enabled; balanced
 * 1	(RX1B_N &  RX1B_P) and (RX2B_N & RX2B_P) enabled; balanced
 * 2	(RX1C_N &  RX1C_P) and (RX2C_N & RX2C_P) enabled; balanced
 *
 * 3	RX1A_N and RX2A_N enabled; unbalanced
 * 4	RX1A_P and RX2A_P enabled; unbalanced
 * 5	RX1B_N and RX2B_N enabled; unbalanced
 * 6	RX1B_P and RX2B_P enabled; unbalanced
 * 7	RX1C_N and RX2C_N enabled; unbalanced
 * 8	RX1C_P and RX2C_P enabled; unbalanced
 */

static int ad9361_rf_port_setup(struct ad9361_rf_phy *phy,
				    unsigned rx_inputs, unsigned txb)
{
	unsigned val;

	if (rx_inputs > 8)
		return -EINVAL;

	if (rx_inputs < 3)
		val = 3 <<  (rx_inputs * 2);
	else
		val = 1 <<  (rx_inputs - 3);

	if (txb)
		val |= BIT(6); /* Select TX1B, TX2B */

	dev_dbg(&phy->spi->dev, "%s : INPUT_SELECT 0x%X",
		__func__, val);

	return ad9361_spi_write(phy->spi, INPUT_SELECT, val);
}

/*
 * Setup the Parallel Port (Digital Data Interface)
 */
static int ad9361_pp_port_setup(struct ad9361_rf_phy *phy, bool restore_c3)
{
	struct spi_device *spi = phy->spi;
	struct ad9361_phy_platform_data *pd = phy->pdata;

	dev_dbg(&phy->spi->dev, "%s", __func__);

	if (!pd->fdd)
		pd->pp_conf[2] |= 0x08;

	if (restore_c3) {
		return ad9361_spi_write(spi, PARALLEL_PORT_CONFIG3,
					pd->pp_conf[2]);
	}

	ad9361_spi_write(spi, PARALLEL_PORT_CONFIG1, pd->pp_conf[0]);
	ad9361_spi_write(spi, PARALLEL_PORT_CONFIG2, pd->pp_conf[1]);
	ad9361_spi_write(spi, PARALLEL_PORT_CONFIG3, pd->pp_conf[2]);
	ad9361_spi_write(spi, RX_CLOCK_DATA_DELAY, pd->rx_clk_data_delay);
	ad9361_spi_write(spi, TX_CLOCK_DATA_DELAY, pd->tx_clk_data_delay);

	ad9361_spi_write(spi, LVDS_BIAS_CONTROL, pd->lvds_bias_ctrl);
//	ad9361_spi_write(spi, DIGITAL_IO_CONTROL, pd->digital_io_ctrl);
	ad9361_spi_write(spi, LVDS_INVERT_CONTROL1, pd->lvds_invert[0]);
	ad9361_spi_write(spi, LVDS_INVERT_CONTROL2, pd->lvds_invert[1]);

	return 0;
}

static int ad9361_mgc_setup(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	/* FIXME later */

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(spi, 0x0fa, 0xe0); // Gain Control Mode Select
	ad9361_spi_write(spi, 0x0fb, 0x08); // Table, Digital Gain, Man Gain Ctrl
	ad9361_spi_write(spi, 0x0fc, 0x23); // Incr Step Size, ADC Overrange Size
	ad9361_spi_write(spi, 0x0fd, 0x4c); // Max Full/LMT Gain Table Index
	ad9361_spi_write(spi, 0x0fe, 0x44); // Decr Step Size, Peak Overload Time
	ad9361_spi_write(spi, 0x100, 0x6f); // Max Digital Gain
	ad9361_spi_write(spi, 0x104, 0x2f); // ADC Small Overload Threshold
	ad9361_spi_write(spi, 0x105, 0x3a); // ADC Large Overload Threshold
	ad9361_spi_write(spi, 0x107, 0x31); // Large LMT Overload Threshold
	ad9361_spi_write(spi, 0x108, 0x39); // Small LMT Overload Threshold
	ad9361_spi_write(spi, 0x109, 0x4c); // Rx1 Full/LMT Gain Index
	ad9361_spi_write(spi, 0x10a, 0x58); // Rx1 LPF Gain Index
	ad9361_spi_write(spi, 0x10b, 0x00); // Rx1 Digital Gain Index
	ad9361_spi_write(spi, 0x10c, 0x4c); // Rx2 Full/LMT Gain Index
	ad9361_spi_write(spi, 0x10d, 0x18); // Rx2 LPF Gain Index
	ad9361_spi_write(spi, 0x10e, 0x00); // Rx2 Digital Gain Index
	ad9361_spi_write(spi, 0x114, 0x30); // Low Power Threshold
	ad9361_spi_write(spi, 0x11a, 0x27); // Initial LMT Gain Limit
	ad9361_spi_write(spi, 0x081, 0x00); // Tx Symbol Gain Control

	return 0;
}


  //************************************************************
  // Setup AuxDAC
  //************************************************************
static int ad9361_auxdac_setup(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	/* FIXME later */

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(spi, 0x018, 0x00); // AuxDAC1 Word[9:2]
	ad9361_spi_write(spi, 0x019, 0x00); // AuxDAC2 Word[9:2]
	ad9361_spi_write(spi, 0x01a, 0x00); // AuxDAC1 Config and Word[1:0]
	ad9361_spi_write(spi, 0x01b, 0x00); // AuxDAC2 Config and Word[1:0]
	ad9361_spi_write(spi, 0x023, 0xff); // AuxDAC Manaul/Auto Control
	ad9361_spi_write(spi, 0x026, 0x00); // AuxDAC Manual Select Bit/GPO Manual Select
	ad9361_spi_write(spi, 0x030, 0x00); // AuxDAC1 Rx Delay
	ad9361_spi_write(spi, 0x031, 0x00); // AuxDAC1 Tx Delay
	ad9361_spi_write(spi, 0x032, 0x00); // AuxDAC2 Rx Delay
	ad9361_spi_write(spi, 0x033, 0x00); // AuxDAC2 Tx Delay

	return 0;
}
  //************************************************************
  // Setup AuxADC
  //************************************************************
static int ad9361_auxadc_setup(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	/* FIXME later */

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(spi, 0x00b, 0x00); // Temp Sensor Setup (Offset)
	ad9361_spi_write(spi, 0x00c, 0x00); // Temp Sensor Setup (Temp Window)
	ad9361_spi_write(spi, 0x00d, 0x03); // Temp Sensor Setup (Periodic Measure)
	ad9361_spi_write(spi, 0x00f, 0x04); // Temp Sensor Setup (Decimation)
	ad9361_spi_write(spi, 0x01c, 0x10); // AuxADC Setup (Clock Div)
	ad9361_spi_write(spi, 0x01d, 0x01); // AuxADC Setup (Decimation/Enable)

	return 0;
}
  //************************************************************
  // Setup Control Outs
  //************************************************************

static int ad9361_ctrl_outs_setup(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	/* FIXME later */

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(spi, 0x035, 0x00); // Ctrl Out index
	ad9361_spi_write(spi, 0x036, 0xff); // Ctrl Out [7:0] output enable

	return 0;
}
  //************************************************************
  // Setup GPO
  //************************************************************

static int ad9361_gpo_setup(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	/* FIXME later */

	dev_dbg(&phy->spi->dev, "%s", __func__);

	ad9361_spi_write(spi,0x020, 0x00); // GPO Auto Enable Setup in RX and TX
	ad9361_spi_write(spi,0x027, 0x03); // GPO Manual and GPO auto value in ALERT
	ad9361_spi_write(spi,0x028, 0x00); // GPO_0 RX Delay
	ad9361_spi_write(spi,0x029, 0x00); // GPO_1 RX Delay
	ad9361_spi_write(spi,0x02a, 0x00); // GPO_2 RX Delay
	ad9361_spi_write(spi,0x02b, 0x00); // GPO_3 RX Delay
	ad9361_spi_write(spi,0x02c, 0x00); // GPO_0 TX Delay
	ad9361_spi_write(spi,0x02d, 0x00); // GPO_1 TX Delay
	ad9361_spi_write(spi,0x02e, 0x00); // GPO_2 TX Delay
	ad9361_spi_write(spi,0x02f, 0x00); // GPO_3 TX Delay

	return 0;
}

static int ad9361_rssi_setup(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	/* FIXME later */

	dev_dbg(&phy->spi->dev, "%s", __func__);
	ad9361_spi_write(spi, 0x150, 0x0e); // RSSI Measurement Duration 0, 1
	ad9361_spi_write(spi, 0x151, 0x00); // RSSI Measurement Duration 2, 3
	ad9361_spi_write(spi, 0x152, 0xff); // RSSI Weighted Multiplier 0
	ad9361_spi_write(spi, 0x153, 0x00); // RSSI Weighted Multiplier 1
	ad9361_spi_write(spi, 0x154, 0x00); // RSSI Weighted Multiplier 2
	ad9361_spi_write(spi, 0x155, 0x00); // RSSI Weighted Multiplier 3
	ad9361_spi_write(spi, 0x156, 0x00); // RSSI Delay
	ad9361_spi_write(spi, 0x157, 0x00); // RSSI Wait
	ad9361_spi_write(spi, 0x158, 0x0d); // RSSI Mode Select
	ad9361_spi_write(spi, 0x15c, 0x67); // Power Measurement Duration

	return 0;
}

static int ad9361_ensm_set_state(struct ad9361_rf_phy *phy, u8 ensm_state)
{
	struct spi_device *spi = phy->spi;
	struct device *dev = &phy->spi->dev;
	int rc = 0;
	u32 val;

	if (phy->curr_ensm_state == ensm_state) {
		dev_dbg(dev, "Nothing to do, device is already in %d state\n",
			ensm_state);
		goto out;
	}

	dev_dbg(dev, "Device is in %x state, moving to %x\n", phy->curr_ensm_state,
			ensm_state);

	val = phy->ensm_conf1;

	switch (ensm_state) {
	case ENSM_STATE_TX:
		val |= ENSM_CONF1_FORCE_TX_ON;
		if (phy->pdata->fdd)
			rc = -EINVAL;
		else if (phy->curr_ensm_state != ENSM_STATE_ALERT)
			rc = -EINVAL;
		break;
	case ENSM_STATE_RX:
		val |= ENSM_CONF1_FORCE_RX_ON;
		if (phy->pdata->fdd)
			rc = -EINVAL;
		else if (phy->curr_ensm_state != ENSM_STATE_ALERT)
			rc = -EINVAL;
		break;
	case ENSM_STATE_FDD:
		val |= (ENSM_CONF1_FORCE_TX_ON | ENSM_CONF1_FORCE_RX_ON);
		if (!phy->pdata->fdd)
			rc = -EINVAL;
		break;
	case ENSM_STATE_ALERT:
		val &= ~(ENSM_CONF1_FORCE_TX_ON | ENSM_CONF1_FORCE_RX_ON);
		val |= ENSM_CONF1_TO_ALERT | ENSM_CONF1_FORCE_ALERT;
		break;
	case ENSM_STATE_SLEEP_WAIT:
		break;
	default:
		dev_err(dev, "No handling for forcing %d ensm state\n",
		ensm_state);
		goto out;
	}

	if (rc) {
		dev_err(dev, "Invalid ENSM state transition in %s mode\n",
			phy->pdata->fdd ? "FDD" : "TDD");
		goto out;
	}

	rc = ad9361_spi_write(spi, REG_ENSM_CONF1, val);
	if (rc)
		dev_err(dev, "Failed to restore state\n");

	phy->curr_ensm_state = ensm_state;

out:
	return rc;

}

static int ad9361_set_trx_clock_chain(struct ad9361_rf_phy *phy,
				      unsigned long *rx_path_clks,
				      unsigned long *tx_path_clks)
{
	struct device *dev = &phy->spi->dev;
	int ret, i, j, n;

	dev_dbg(&phy->spi->dev, "%s", __func__);

	if (!rx_path_clks || !tx_path_clks)
		return -EINVAL;

	ret = clk_set_rate(phy->clks[BBPLL_CLK], rx_path_clks[BBPLL_FREQ]);
	if (ret < 0)
		return ret;

	for (i = ADC_CLK, j = DAC_CLK, n = ADC_FREQ;
		i <= RX_SAMPL_CLK; i++, j++, n++) {
		ret = clk_set_rate(phy->clks[i], rx_path_clks[n]);
		if (ret < 0) {
			dev_err(dev, "Failed to set BB ref clock rate (%d)\n",
				ret);
			return ret;
		}
		ret = clk_set_rate(phy->clks[j], tx_path_clks[n]);
		if (ret < 0) {
			dev_err(dev, "Failed to set BB ref clock rate (%d)\n",
				ret);
			return ret;
		}
	}
	return 0;
}

static int ad9361_get_trx_clock_chain(struct ad9361_rf_phy *phy, unsigned long *rx_path_clks,
				      unsigned long *tx_path_clks)
{
	int i, j, n;
	unsigned long bbpll_freq;

	if (!rx_path_clks && !tx_path_clks)
		return -EINVAL;

	bbpll_freq = clk_get_rate(phy->clks[BBPLL_CLK]);

	if (rx_path_clks)
		rx_path_clks[BBPLL_FREQ] = bbpll_freq;

	if (tx_path_clks)
		tx_path_clks[BBPLL_FREQ] = bbpll_freq;

	for (i = ADC_CLK, j = DAC_CLK, n = ADC_FREQ;
		i <= RX_SAMPL_CLK; i++, j++, n++) {
		if (rx_path_clks)
			rx_path_clks[n] = clk_get_rate(phy->clks[i]);
		if (tx_path_clks)
			tx_path_clks[n] = clk_get_rate(phy->clks[j]);
	}

	return 0;
}


static int ad9361_calculate_rf_clock_chain(struct ad9361_rf_phy *phy,
				      unsigned long tx_sample_rate,
				      unsigned int low_power,
				      unsigned long *rx_path_clks,
				      unsigned long *tx_path_clks)
{
	unsigned long clktf, adc_rate, dac_rate = 0;
	unsigned long long bbpll_rate;
	int i, index_rx = -1, index_tx = -1;
	unsigned div, fir_intdec;
	const char clk_dividers[][4] = {
		{12,3,2,2},
		{8,2,2,2},
		{6,3,1,2},
		{4,2,2,1},
		{3,3,1,1},
		{2,2,1,1},
		{1,1,1,1},
	};

	if (phy->bypass_rx_fir && phy->bypass_tx_fir) {
		fir_intdec = 1;
	} else if ((phy->bypass_rx_fir != phy->bypass_tx_fir) &&
		((phy->tx_fir_int == phy->rx_fir_dec) == 1)) {
		fir_intdec = 1;
	} else if (phy->tx_fir_int != phy->rx_fir_dec) {
		dev_err(&phy->spi->dev,
			"%s: TRX FIR decimation / interpolation mismatch %d,%d",
			__func__, phy->rx_fir_dec, phy->tx_fir_int);
		return -EINVAL;
	} else {
		fir_intdec = phy->tx_fir_int;
	}

	dev_dbg(&phy->spi->dev, "%s: requested rate %lu FIR dec/int %d mode %s",
		__func__, tx_sample_rate, fir_intdec,
		low_power ? "Medium PWR" : "Highest OSR");

	if (tx_sample_rate > 56000000UL) /* FIXME */
		return -EINVAL;

	clktf = tx_sample_rate * fir_intdec;

	for (i = !!low_power; i < 7; i++) {
		adc_rate = clktf * clk_dividers[i][0];
		if ((adc_rate <= MAX_ADC_CLK) && (adc_rate >= MIN_ADC_CLK)) {
			if (adc_rate <= MAX_DAC_CLK) {
				index_rx = index_tx = i;
				dac_rate = adc_rate; /* ADC_CLK */
				break;
			} else {
				dac_rate = adc_rate / 2;  /* ADC_CLK/2 */
				index_tx = i + 2;
				index_rx = i;
				break;
			}
		}
	}

	if ((index_tx < 0 || index_tx > 6) && low_power == 0) {
		ad9361_calculate_rf_clock_chain(phy, tx_sample_rate,
			1, rx_path_clks, tx_path_clks);
	} else if (index_tx < 0 || index_tx > 6) {
		dev_err(&phy->spi->dev, "%s: Failed to find suitable dividers: %s",
		__func__, (adc_rate < MIN_ADC_CLK) ? "ADC clock below limit" : "BBPLL rate above limit");

		return -EINVAL;
	}

	/* Calculate target BBPLL rate */
	div = MAX_BBPLL_DIV;

	do {
		bbpll_rate = (unsigned long long)adc_rate * div;
		div >>= 1;

	} while ((bbpll_rate > MAX_BBPLL_FREQ) && (div >= MIN_BBPLL_DIV));

	rx_path_clks[BBPLL_FREQ] = bbpll_rate;
	rx_path_clks[ADC_FREQ] = adc_rate;
	rx_path_clks[R2_FREQ] = rx_path_clks[ADC_FREQ] / clk_dividers[index_rx][1];
	rx_path_clks[R1_FREQ] = rx_path_clks[R2_FREQ] / clk_dividers[index_rx][2];
	rx_path_clks[CLKRF_FREQ] = rx_path_clks[R1_FREQ] / clk_dividers[index_rx][3];
	rx_path_clks[RX_SAMPL_FREQ] = rx_path_clks[CLKRF_FREQ] / 	fir_intdec;

	tx_path_clks[BBPLL_FREQ] = bbpll_rate;
	tx_path_clks[DAC_FREQ] = dac_rate;
	tx_path_clks[T2_FREQ] = tx_path_clks[DAC_FREQ] / clk_dividers[index_tx][1];
	tx_path_clks[T1_FREQ] =tx_path_clks[T2_FREQ] / clk_dividers[index_tx][2];
	tx_path_clks[CLKTF_FREQ] = tx_path_clks[T1_FREQ] / clk_dividers[index_tx][3];
	tx_path_clks[TX_SAMPL_FREQ] = tx_path_clks[CLKTF_FREQ] / 	fir_intdec;

	return 0;
}

static int ad9361_setup(struct ad9361_rf_phy *phy)
{
	unsigned long refin_Hz, ref_freq;
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	int ret;
	unsigned real_bandwidth = phy->pdata->rf_bandwidth_Hz / 2;

	dev_dbg(dev, "%s", __func__);

	phy->ensm_conf1 =
		(phy->pdata->ensm_pin_level_mode ? ENSM_CONF1_LEVEL_MODE : 0) |
		(phy->pdata->ensm_pin_ctrl ? ENSM_CONF1_ENSM_PIN_CTL_EN : 0) |
		ENSM_CONF1_TO_ALERT;

	ad9361_spi_write(spi, SPI_CONFIGURATION, 0x81); /* RESET */
	ad9361_spi_write(spi, SPI_CONFIGURATION, 0x0);

	ad9361_spi_write(spi, CONTROL_REGISTER, 0x01);
	ad9361_spi_write(spi, BANDGAP_CONFIG0, 0x0E); /* Enable Master Bias */
	ad9361_spi_write(spi, BANDGAP_CONFIG1, 0x0E); /* Set Bandgap Trim */

	ad9361_set_dcxo_tune(phy, phy->pdata->dcxo_coarse, phy->pdata->dcxo_fine);

	refin_Hz = clk_get_rate(phy->clk_refin);

	if (refin_Hz < 40000000UL)
		ref_freq = 2 * refin_Hz;
	else if (refin_Hz < 80000000UL)
		ref_freq = refin_Hz;
	else if (refin_Hz < 160000000UL)
		ref_freq = refin_Hz / 2;
	else if (refin_Hz < 320000000UL)
		ref_freq = refin_Hz / 4;
	else
		return -EINVAL;

	ret = clk_set_rate(phy->clks[RX_REFCLK], ref_freq);
	if (ret < 0) {
		dev_err(dev, "Failed to set RX Synth ref clock rate (%d)\n", ret);
		return ret;
	}

	ret = clk_set_rate(phy->clks[TX_REFCLK], ref_freq);
	if (ret < 0) {
		dev_err(dev, "Failed to set TX Synth ref clock rate (%d)\n", ret);
		return ret;
	}

 	ret = clk_prepare_enable(phy->clks[RX_REFCLK]);
	if (ret < 0) {
		dev_err(dev, "Failed to enable RX Synth ref clock (%d)\n", ret);
		return ret;
	}

 	ret = clk_prepare_enable(phy->clks[TX_REFCLK]);
	if (ret < 0) {
		dev_err(dev, "Failed to enable TX Synth ref clock (%d)\n", ret);
		return ret;
	}

	ad9361_spi_writef(spi, REF_DIVIDE_CONFIG_1, 0x02, 1);
	ad9361_spi_writef(spi, REF_DIVIDE_CONFIG_2, 0x00, 3); /* FB DELAY */
	ad9361_spi_writef(spi, REF_DIVIDE_CONFIG_2, 0x60, 3); /* FB DELAY */

	ad9361_spi_write(spi, CLK_ENABLE, phy->pdata->use_extclk ? 0x17 : 0x7); /* Enable Clocks */

	ret = clk_set_rate(phy->clks[BB_REFCLK], ref_freq);
	if (ret < 0) {
		dev_err(dev, "Failed to set BB ref clock rate (%d)\n",
			ret);
		return ret;
	}

	ret = clk_prepare_enable(phy->clks[BB_REFCLK]);
	if (ret < 0) {
		dev_err(dev, "Failed to enable BB ref clock rate (%d)\n",
			ret);
		return ret;
	}

	ret = ad9361_set_trx_clock_chain(phy, phy->pdata->rx_path_clks,
				   phy->pdata->tx_path_clks);
	if (ret < 0)
		return ret;

	ad9361_en_dis_tx(phy, 1, TX_ENABLE);
	ad9361_en_dis_rx(phy, 1, RX_ENABLE);

	if (phy->pdata->rx2tx2) {
		ad9361_en_dis_tx(phy, 2, TX_ENABLE);
		ad9361_en_dis_rx(phy, 2, RX_ENABLE);
	}

	ret = ad9361_rf_port_setup(phy, phy->pdata->rf_rx_input_sel,
				   phy->pdata->rf_tx_output_sel);
	if (ret < 0)
		return ret;

	ret = ad9361_pp_port_setup(phy, false);
	if (ret < 0)
		return ret;

	ret = ad9361_auxdac_setup(phy);
	if (ret < 0)
		return ret;
	ret = ad9361_auxadc_setup(phy);
	if (ret < 0)
		return ret;
	ret = ad9361_ctrl_outs_setup(phy);
	if (ret < 0)
		return ret;
	ret = ad9361_gpo_setup(phy);
	if (ret < 0)
		return ret;

	ret = ad9361_set_ref_clk_cycles(phy, refin_Hz);
	if (ret < 0)
		return ret;

	ret = ad9361_txrx_synth_cp_calib(phy, ref_freq, false); /* RXCP */
	if (ret < 0)
		return ret;

	ret = ad9361_txrx_synth_cp_calib(phy, ref_freq, true); /* TXCP */
	if (ret < 0)
		return ret;

	ret = clk_set_rate(phy->clks[RX_RFPLL], ad9361_to_clk(phy->pdata->rx_synth_freq));
	if (ret < 0) {
		dev_err(dev, "Failed to set RX Synth rate (%d)\n",
			ret);
		return ret;
	}

	ret = clk_prepare_enable(phy->clks[RX_RFPLL]);
	if (ret < 0)
		return ret;

	ret = clk_set_rate(phy->clks[TX_RFPLL], ad9361_to_clk(phy->pdata->tx_synth_freq));
	if (ret < 0) {
		dev_err(dev, "Failed to set TX Synth rate (%d)\n",
			ret);
		return ret;
	}

	ret = clk_prepare_enable(phy->clks[TX_RFPLL]);
	if (ret < 0)
		return ret;

	ret = ad9361_load_mixer_gm_subtable(phy);
	if (ret < 0)
		return ret;

	ret = ad9361_mgc_setup(phy);
	if (ret < 0)
		return ret;

	ret = ad9361_rx_bb_analog_filter_calib(phy,
				real_bandwidth,
				clk_get_rate(phy->clks[BBPLL_CLK]));
	if (ret < 0)
		return ret;

	ret = ad9361_tx_bb_analog_filter_calib(phy,
				real_bandwidth,
				clk_get_rate(phy->clks[BBPLL_CLK]));
	if (ret < 0)
		return ret;

	ret = ad9361_rx_tia_calib(phy, real_bandwidth);
	if (ret < 0)
		return ret;

	ret = ad9361_tx_bb_second_filter_calib(phy, phy->pdata->rf_bandwidth_Hz);
	if (ret < 0)
		return ret;

	ret = ad9361_rx_adc_setup(phy,
				real_bandwidth,
				clk_get_rate(phy->clks[ADC_CLK]));
	if (ret < 0)
		return ret;

	ret = ad9361_bb_dc_offset_calib(phy);
	if (ret < 0)
		return ret;

	ret = ad9361_rf_dc_offset_calib(phy,
			ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL])));
	if (ret < 0)
		return ret;

	ret = ad9361_tx_quad_calib(phy, real_bandwidth);
	if (ret < 0)
		return ret;


	ret = ad9361_tracking_control(phy, true, true, true);
	if (ret < 0)
		return ret;

	if (!phy->pdata->fdd)
		ad9361_run_calibration(phy, TXMON_CAL);

	ad9361_pp_port_setup(phy, true);

	ad9361_spi_write(phy->spi, ENSM_MODE, phy->pdata->fdd ? 0x01 : 0x00);

	if (phy->pdata->fdd)
		ad9361_spi_write(phy->spi, ENSM_CONFIG_2, 0x04 |
			(phy->pdata->ensm_pin_ctrl ? 0x80 : 0)); /* Dual Synth */
	 else
		ad9361_spi_write(phy->spi, ENSM_CONFIG_2,
				 (phy->pdata->ensm_pin_ctrl ? 0x08 : 0)); /* single Synth */

	ret = ad9361_set_tx_atten(phy, phy->pdata->tx_atten, true, true);
	if (ret < 0)
		return ret;

	ret = ad9361_rssi_setup(phy);
	if (ret < 0)
		return ret;

	ad9361_ensm_set_state(phy, ENSM_STATE_FDD);

//	ad9361_spi_write(spi, ENSM_CONFIG_1, 0x60); // Ctrl Out index

	phy->current_bw_Hz = phy->pdata->rf_bandwidth_Hz;
	phy->auto_cal_en = true;
	phy->cal_threshold_freq = 100000000ULL; /* 100 MHz */

	return 0;

}

static int ad9361_do_calib_run(struct ad9361_rf_phy *phy, unsigned cal)
{
	int ret;

	ret = ad9361_tracking_control(phy, false, false, false);
	if (ret < 0)
		return ret;

	ad9361_ensm_force_state(phy, ENSM_STATE_ALERT);

	switch (cal) {
	case TX_QUAD_CAL:
		ret = ad9361_tx_quad_calib(phy, phy->current_bw_Hz / 2);
		break;
	case RX_QUAD_CAL:
		ret = ad9361_rx_quad_calib(phy, phy->current_bw_Hz / 2);
		break;
	case RFDC_CAL:
		ret = ad9361_rf_dc_offset_calib(phy,
			ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL])));
		break;
	default:
		ret = -EINVAL;
		break;
	}

	ad9361_tracking_control(phy, true, true, true);
	ad9361_ensm_restore_prev_state(phy);

	return ret;
}

static int ad9361_update_rf_bandwidth(struct ad9361_rf_phy *phy, unsigned rf_bw)
{
	unsigned long bbpll_freq;
	unsigned real_bandwidth = rf_bw / 2;
	int ret;

	bbpll_freq = clk_get_rate(phy->clks[BBPLL_CLK]);

	ret = ad9361_tracking_control(phy, false, false, false);
	if (ret < 0)
		return ret;


	ad9361_ensm_force_state(phy, ENSM_STATE_ALERT);

	ret = ad9361_rx_bb_analog_filter_calib(phy,
				real_bandwidth,
				bbpll_freq);
	if (ret < 0)
		return ret;

	ret = ad9361_tx_bb_analog_filter_calib(phy,
				real_bandwidth,
				bbpll_freq);
	if (ret < 0)
		return ret;

	ret = ad9361_rx_tia_calib(phy, real_bandwidth);
	if (ret < 0)
		return ret;

	ret = ad9361_tx_bb_second_filter_calib(phy, rf_bw);
	if (ret < 0)
		return ret;

	ret = ad9361_rx_adc_setup(phy,
				real_bandwidth,
				clk_get_rate(phy->clks[ADC_CLK]));
	if (ret < 0)
		return ret;

	ret = ad9361_tx_quad_calib(phy, real_bandwidth);
	if (ret < 0)
		return ret;

	phy->current_bw_Hz = rf_bw;

	ret = ad9361_tracking_control(phy, true, true, true);
	if (ret < 0)
		return ret;

	ad9361_ensm_restore_prev_state(phy);

	return 0;
}


static int ad9361_load_fir_filter_coef(struct ad9361_rf_phy *phy,
				       enum fir_dest dest, int gain_dB,
				       unsigned ntaps, short *coef)
{
	struct spi_device *spi = phy->spi;
	unsigned val, offs = 0, fir_conf = 0;

	dev_dbg(&phy->spi->dev, "%s: TAPS %d, gain %d, dest %d",
		__func__, ntaps, gain_dB, dest);

	if (coef == NULL || !ntaps || ntaps > 128 || ntaps % 16) {
		dev_err(&phy->spi->dev,
			"%s: Invalid parameters: TAPS %d, gain %d, dest 0x%X",
			__func__, ntaps, gain_dB, dest);

		return -EINVAL;
	}

	if (dest & FIR_IS_RX) {
		val = 3 - (gain_dB + 12) / 6;
		ad9361_spi_write(spi, RX_FIR_GAIN, val & 0x3);
		offs = RX_FIR_COEFF_ADDR - TX_FIR_COEFF_ADDR;
		phy->rx_fir_ntaps = ntaps;
	} else {
		if (gain_dB == -6)
			fir_conf = TX_FIR_GAIN_6DB;
		phy->tx_fir_ntaps = ntaps;
	}

	val = ntaps / 16 - 1;

	fir_conf |= FIR_NUM_TAPS(val) | FIR_SELECT(dest) | FIR_START_CLK;

	ad9361_spi_write(spi, TX_FIR_CONFIG + offs, fir_conf);

	for (val = 0; val < ntaps; val++) {
		ad9361_spi_write(spi, TX_FIR_COEFF_ADDR + offs, val);
		ad9361_spi_write(spi, TX_FIR_COEFF_WRDATA_LOW + offs,
				 coef[val] & 0xFF);
		ad9361_spi_write(spi, TX_FIR_COEFF_WRDATA_HIGH + offs,
				 coef[val] >> 8);
		ad9361_spi_write(spi, TX_FIR_CONFIG + offs, fir_conf | FIR_WRITE);
		ad9361_spi_write(spi, TX_FIR_COEFF_RDDATA_HIGH + offs, 0);
		ad9361_spi_write(spi, TX_FIR_COEFF_RDDATA_HIGH + offs, 0);
	}

	ad9361_spi_write(spi, TX_FIR_CONFIG + offs, fir_conf);
	fir_conf &= ~FIR_START_CLK;
	ad9361_spi_write(spi, TX_FIR_CONFIG + offs, fir_conf);

	return 0;
}

static int ad9361_parse_fir(struct ad9361_rf_phy *phy,
				 char *data, unsigned size)
{
	char *line;
	int i = 0, ret, tmp;
	int tx = -1, tx_gain, tx_int;
	int rx = -1, rx_gain, rx_dec;
	short coef[128];
	char *ptr = data;

	while ((line = strsep(&ptr, "\n"))) {
		if (line >= data + size) {
			break;
		}

		if (line[0] == '#')
			continue;

		if (tx < 0) {
			ret = sscanf(line, "TX %d GAIN %d INT %d", &tx, &tx_gain, &tx_int);
			if (ret == 3)
				continue;
			else
				tx = -1;
		}
		if (rx < 0) {
			ret = sscanf(line, "RX %d GAIN %d DEC %d", &rx, &rx_gain, &rx_dec);
			if (ret == 3)
				continue;
			else
				tx = -1;
		}
		ret = sscanf(line, "%d", &tmp);
		if (ret == 1) {
			coef[i++] = (short)tmp;
			continue;
		}
	}

	switch (tx) {
	case FIR_TX1:
	case FIR_TX2:
	case FIR_TX1_TX2:
		ret = ad9361_load_fir_filter_coef(phy, tx, tx_gain, i, coef);
		phy->tx_fir_int = tx_int;
		break;
	default:
		ret = -EINVAL;
	}

	if (ret < 0)
		return ret;

	switch (rx | FIR_IS_RX) {
	case FIR_RX1:
	case FIR_RX2:
	case FIR_RX1_RX2:
		ret = ad9361_load_fir_filter_coef(phy, rx | FIR_IS_RX,
						  rx_gain, i, coef);
		phy->rx_fir_dec = rx_dec;
		break;
	default:
		ret = -EINVAL;
	}

	if (ret < 0)
		return ret;


	return size;
}


static int ad9361_validate_enable_fir(struct ad9361_rf_phy *phy)
{
	struct device *dev = &phy->spi->dev;
	int ret;
	unsigned long rx[6], tx[6];
	unsigned max;

	dev_dbg(dev, "%s: TX FIR EN=%d/TAPS%d/INT%d, RX FIR EN=%d/TAPS%d/DEC%d",
		__func__, !phy->bypass_tx_fir, phy->tx_fir_ntaps, phy->tx_fir_int,
		!phy->bypass_rx_fir, phy->rx_fir_ntaps, phy->rx_fir_dec);

	if (!phy->bypass_tx_fir) {
		if (!(phy->tx_fir_int == 1 || phy->tx_fir_int == 2 ||
			phy->tx_fir_int == 4)) {
			dev_err(dev,
				"%s: Invalid: Interpolation %d in filter config",
				__func__, phy->tx_fir_int);
			return -EINVAL;
		}


		if (phy->tx_fir_int == 1 && phy->tx_fir_ntaps > 64) {
			dev_err(dev,
				"%s: Invalid: TAPS > 64 and Interpolation = 1",
				__func__);
			return -EINVAL;
		}
	}

	if (!phy->bypass_rx_fir) {
		if (!(phy->rx_fir_dec == 1 || phy->rx_fir_dec == 2 ||
			phy->rx_fir_dec == 4)) {
			dev_err(dev,
				"%s: Invalid: Decimation %d in filter config",
				__func__, phy->rx_fir_dec);

			return -EINVAL;
		}
	}

	ret = ad9361_calculate_rf_clock_chain(phy,
			clk_get_rate(phy->clks[TX_SAMPL_CLK]),
			phy->rate_governor, rx, tx);

	if (ret < 0) {
		dev_err(dev,
			"%s: Calculating filter rates failed %d",
			__func__, ret);

		return ret;
	}

	if (!phy->bypass_tx_fir) {
		max = ((rx[ADC_FREQ] / 2) / tx[TX_SAMPL_FREQ]) * 16;
		if (phy->tx_fir_ntaps > max) {
			dev_err(dev,
				"%s: Invalid: ratio ADC/2 / TX_SAMPL * 16 > TAPS",
				__func__);
			return -EINVAL;
		}
	}

	if (!phy->bypass_rx_fir) {
		max = ((rx[ADC_FREQ] / 2) / rx[RX_SAMPL_FREQ]) * 16;
		if (phy->rx_fir_ntaps > max) {
			dev_err(dev,
				"%s: Invalid: ratio ADC/2 / RX_SAMPL * 16 > TAPS",
				__func__);
			return -EINVAL;
		}
	}

	ret = ad9361_set_trx_clock_chain(phy, rx, tx);
	if (ret < 0)
		return ret;

	/*
	 * Workaround for clock framework since clocks don't change the we
	 * manually need to enable the filter
	 */

	if (phy->rx_fir_dec == 1) {
		ad9361_spi_writef(phy->spi, RX_ENABLE_FILTER, TX_RX_FIR, !phy->bypass_rx_fir);
	}

	if (phy->tx_fir_int == 1) {
		ad9361_spi_writef(phy->spi, TX_ENABLE_FILTER, TX_RX_FIR, !phy->bypass_tx_fir);
	}

	return ad9361_update_rf_bandwidth(phy, phy->current_bw_Hz);
}

static void ad9361_work_func(struct work_struct *work)
{
	struct ad9361_rf_phy *phy =
		container_of(work, struct ad9361_rf_phy, work);
	int ret;

	dev_dbg(&phy->spi->dev, "%s:", __func__);

	ret = ad9361_do_calib_run(phy, TX_QUAD_CAL);
	if (ret < 0)
		dev_err(&phy->spi->dev,
			"%s: TX QUAD cal failed", __func__);

	complete_all(&phy->complete);
	clear_bit(0, &phy->flags);
}

/*
 * AD9361 Clocks
 */

#define to_clk_priv(_hw) container_of(_hw, struct refclk_scale, hw)

static inline int ad9361_set_muldiv(struct refclk_scale *priv, unsigned mul, unsigned div)
{
	priv->mult = mul;
	priv->div = div;
	return 0;
}

static int ad9361_get_clk_scaler(struct clk_hw *hw)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	struct spi_device *spi = clk_priv->spi;
	unsigned tmp, tmp1;

	switch (clk_priv->source) {
	case BB_REFCLK:
		tmp = ad9361_spi_read(spi, CLOCK_CONTROL);
		tmp &= 0x3;
		break;
	case RX_REFCLK:
		tmp = ad9361_spi_readf(spi, REF_DIVIDE_CONFIG_1, BIT(0));
		tmp1 = ad9361_spi_readf(spi, REF_DIVIDE_CONFIG_2, BIT(7));
		tmp = (tmp << 1) | tmp1;
		break;
	case TX_REFCLK:
		tmp = ad9361_spi_readf(spi, REF_DIVIDE_CONFIG_2, 0x0C);
		break;
	case ADC_CLK:
		tmp = ad9361_spi_read(spi, BBPLL);
		return ad9361_set_muldiv(clk_priv, 1, 1 << (tmp & 0x7));
	case R2_CLK:
		tmp = ad9361_spi_readf(spi, RX_ENABLE_FILTER, THB3_DEC3);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case R1_CLK:
		tmp = ad9361_spi_readf(spi, RX_ENABLE_FILTER, THB2_DEC2);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case CLKRF_CLK:
		tmp = ad9361_spi_readf(spi, RX_ENABLE_FILTER, THB1_DEC1);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case RX_SAMPL_CLK:
		tmp = ad9361_spi_readf(spi, RX_ENABLE_FILTER, TX_RX_FIR);

		if (!tmp)
			tmp = 1; /* bypass filter */
		else
			tmp = (1 << (tmp - 1));

		return ad9361_set_muldiv(clk_priv, 1, tmp);
	case DAC_CLK:
		tmp = ad9361_spi_readf(spi, BBPLL, BIT(3));
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case T2_CLK:
		tmp = ad9361_spi_readf(spi, TX_ENABLE_FILTER, THB3_DEC3);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case T1_CLK:
		tmp = ad9361_spi_readf(spi, TX_ENABLE_FILTER, THB2_DEC2);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case CLKTF_CLK:
		tmp = ad9361_spi_readf(spi, TX_ENABLE_FILTER, THB1_DEC1);
		return ad9361_set_muldiv(clk_priv, 1, tmp + 1);
	case TX_SAMPL_CLK:
		tmp = ad9361_spi_readf(spi, TX_ENABLE_FILTER, TX_RX_FIR);

		if (!tmp)
			tmp = 1; /* bypass filter */
		else
			tmp = (1 << (tmp - 1));

		return ad9361_set_muldiv(clk_priv, 1, tmp);
	default:
		return -EINVAL;
	}

	/* REFCLK Scaler */
	switch (tmp) {
	case 0:
		ad9361_set_muldiv(clk_priv, 1, 1);
		break;
	case 1:
		ad9361_set_muldiv(clk_priv, 1, 2);
		break;
	case 2:
		ad9361_set_muldiv(clk_priv, 1, 4);
		break;
	case 3:
		ad9361_set_muldiv(clk_priv, 2, 1);
		break;
	default:
		return -EINVAL;

	}

	return 0;
}

static int ad9361_to_refclk_scaler(struct refclk_scale *clk_priv)
{
	/* REFCLK Scaler */
	switch (((clk_priv->mult & 0xF) << 4) | (clk_priv->div & 0xF)) {
	case 0x11:
		return 0;
	case 0x12:
		return 1;
	case 0x14:
		return 2;
	case 0x21:
		return 3;
	default:
		return -EINVAL;
	}
};

static int ad9361_set_clk_scaler(struct clk_hw *hw, bool set)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	struct spi_device *spi = clk_priv->spi;
	unsigned tmp;
	int ret;

	switch (clk_priv->source) {
	case BB_REFCLK:
		ret = ad9361_to_refclk_scaler(clk_priv);
		if (ret < 0)
			return ret;
		if (set)
			return ad9361_spi_writef(spi, CLOCK_CONTROL, 0x3, ret);
		break;

	case RX_REFCLK:
		ret = ad9361_to_refclk_scaler(clk_priv);
		if (ret < 0)
			return ret;
		if (set) {
			tmp = ret;
			ret = ad9361_spi_writef(spi, REF_DIVIDE_CONFIG_1,
						BIT(0), tmp >> 1);
			ret |= ad9361_spi_writef(spi, REF_DIVIDE_CONFIG_2,
						 BIT(7), tmp & 1);
			return ret;
		}
		break;
	case TX_REFCLK:
		ret = ad9361_to_refclk_scaler(clk_priv);
		if (ret < 0)
			return ret;
		if (set)
			return ad9361_spi_writef(spi, REF_DIVIDE_CONFIG_2,
						0x0C, ret);
		break;
	case ADC_CLK:
		tmp = ilog2((u8)clk_priv->div);
		if (clk_priv->mult != 1 || tmp > 6 || tmp < 1)
			return -EINVAL;

		if (set)
			return ad9361_spi_writef(spi, BBPLL, 0x7, tmp);
		break;
	case R2_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 3 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, RX_ENABLE_FILTER,
						 THB3_DEC3, clk_priv->div - 1);
		break;
	case R1_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, RX_ENABLE_FILTER,
						 THB2_DEC2, clk_priv->div - 1);
		break;
	case CLKRF_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, RX_ENABLE_FILTER,
						 THB1_DEC1, clk_priv->div - 1);
		break;
	case RX_SAMPL_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 4 ||
			clk_priv->div < 1 || clk_priv->div == 3)
			return -EINVAL;

		if (clk_priv->phy->bypass_rx_fir)
			tmp = 0;
		else
			tmp = ilog2(clk_priv->div) + 1;

		if (set)
			return ad9361_spi_writef(spi, RX_ENABLE_FILTER,
						 TX_RX_FIR, tmp);
		break;
	case DAC_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, BBPLL,
						 BIT(3), clk_priv->div - 1);
		break;
	case T2_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 3 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, TX_ENABLE_FILTER,
						 THB3_DEC3, clk_priv->div - 1);
		break;
	case T1_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, TX_ENABLE_FILTER,
						 THB2_DEC2, clk_priv->div - 1);
		break;
	case CLKTF_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 2 || clk_priv->div < 1)
			return -EINVAL;
		if (set)
			return ad9361_spi_writef(spi, TX_ENABLE_FILTER,
						 THB1_DEC1, clk_priv->div - 1);
		break;
	case TX_SAMPL_CLK:
		if (clk_priv->mult != 1 || clk_priv->div > 4 ||
			clk_priv->div < 1 || clk_priv->div == 3)
			return -EINVAL;

		if (clk_priv->phy->bypass_tx_fir)
			tmp = 0;
		else
			tmp = ilog2(clk_priv->div) + 1;

		if (set)
			return ad9361_spi_writef(spi, TX_ENABLE_FILTER,
						 TX_RX_FIR, tmp);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static unsigned long ad9361_clk_factor_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	unsigned long long int rate;

	ad9361_get_clk_scaler(hw);
	rate = (parent_rate * clk_priv->mult) / clk_priv->div;

	return (unsigned long)rate;
}

static long ad9361_clk_factor_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	int ret;

	if (rate >= *prate) {
		clk_priv->mult = DIV_ROUND_CLOSEST(rate, *prate);
		clk_priv->div = 1;

	} else {
		clk_priv->div = DIV_ROUND_CLOSEST(*prate, rate);
		clk_priv->mult = 1;
	}

	ret = ad9361_set_clk_scaler(hw, false);
	if (ret < 0)
		return ret;

	return (*prate / clk_priv->div) * clk_priv->mult;
}

static int ad9361_clk_factor_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, parent_rate);

	if (rate >= parent_rate) {
		clk_priv->mult = DIV_ROUND_CLOSEST(rate, parent_rate);
		clk_priv->div = 1;
	} else {
		clk_priv->div = DIV_ROUND_CLOSEST(parent_rate, rate);
		clk_priv->mult = 1;
	}

	return ad9361_set_clk_scaler(hw, true);
}

struct clk_ops refclk_scale_ops = {
	.round_rate = ad9361_clk_factor_round_rate,
	.set_rate = ad9361_clk_factor_set_rate,
	.recalc_rate = ad9361_clk_factor_recalc_rate,
};

/*
 * BBPLL
 */

static unsigned long ad9361_bbpll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	unsigned long long int rate;
	unsigned long fract, integer;
	unsigned char buf[4];

	ad9361_spi_readm(clk_priv->spi, INT_BB_FREQ_WORD, &buf[0],
			       INT_BB_FREQ_WORD - FRAC_BB_FREQ_WORD_1 + 1);

	fract = (buf[3] << 16) | (buf[2] << 8) | buf[1];
	integer = buf[0];

	rate = ((unsigned long long int)parent_rate * fract);
	do_div(rate, BBPLL_MODULUS);
	rate += (unsigned long long int)parent_rate * integer;

	return (unsigned long)rate;
}

static long ad9361_bbpll_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	unsigned long long tmp;
	unsigned fract, integer;

	if (rate > MAX_BBPLL_FREQ)
		return MAX_BBPLL_FREQ;

	if (rate < MIN_BBPLL_FREQ)
		return MIN_BBPLL_FREQ;

	tmp = do_div(rate, *prate);
	tmp = tmp * BBPLL_MODULUS + (*prate >> 1);
	do_div(tmp, *prate);

	integer = rate;
	fract = tmp;

	tmp = *prate * (unsigned long long)fract;
	do_div(tmp, BBPLL_MODULUS);
	tmp += *prate * integer;

	return tmp;
}

static int ad9361_bbpll_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	struct spi_device *spi = clk_priv->spi;
	unsigned long long tmp;
	unsigned fract, integer;
	int icp_val;
	unsigned char lf_defaults[3] = {0x35, 0x5B, 0xE8};

	dev_dbg(&spi->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, parent_rate);

	/*
	 * Setup Loop Filter and CP Current
	 * Scale is 150uA @ (1280MHz BBPLL, 40MHz REFCLK)
	 */
	tmp = rate * 150ULL;
	do_div(tmp, parent_rate * 32UL + (tmp >> 1));

	/* 25uA/LSB, Offset 25uA */
	icp_val = DIV_ROUND_CLOSEST((unsigned)tmp, 25U) - 1;

	icp_val = clamp(icp_val, 1, 64);

	ad9361_spi_write(spi, CP_CURRENT, icp_val);
	ad9361_spi_writem(spi, LOOP_FILTER_3, lf_defaults,
			  ARRAY_SIZE(lf_defaults));

	/* Allow calibration to occur and set cal count to 1024 for max accuracy */
	ad9361_spi_write(spi, VCO_CONTROL, 0xE0);
	/* Set calibration clock to REFCLK/4 for more accuracy */
	ad9361_spi_write(spi, SDM_CONTROL, 0x10);

	/* Calculate and set BBPLL frequency word */
	tmp = do_div(rate, parent_rate);
	tmp = tmp *(unsigned long long) BBPLL_MODULUS + (parent_rate >> 1);
	do_div(tmp, parent_rate);

	integer = rate;
	fract = tmp;

	ad9361_spi_write(spi, INT_BB_FREQ_WORD, integer);
	ad9361_spi_write(spi, FRAC_BB_FREQ_WORD_3, fract);
	ad9361_spi_write(spi, FRAC_BB_FREQ_WORD_2, fract >> 8);
	ad9361_spi_write(spi, FRAC_BB_FREQ_WORD_1, fract >> 16);

	ad9361_spi_write(spi, SDM_CONTROL_1, 0x05); /* Start BBPLL Calibration */
	ad9361_spi_write(spi, SDM_CONTROL_1, 0x01); /* Clear BBPLL start calibration bit */

	ad9361_spi_write(spi, VCO_PROGRAM_1, 0x86); /* Increase BBPLL KV and phase margin */
	ad9361_spi_write(spi, VCO_PROGRAM_2, 0x01); /* Increase BBPLL KV and phase margin */
	ad9361_spi_write(spi, VCO_PROGRAM_2, 0x05); /* Increase BBPLL KV and phase margin */

	return ad9361_check_cal_done(clk_priv->phy, CH1_OVERFLOW, BIT(7), 1);
}

struct clk_ops bbpll_clk_ops = {
	.round_rate = ad9361_bbpll_round_rate,
	.set_rate = ad9361_bbpll_set_rate,
	.recalc_rate = ad9361_bbpll_recalc_rate,
};

/*
 * RFPLL
 */

static unsigned long long ad9361_calc_rfpll_freq(unsigned long long parent_rate,
				   unsigned long long integer,
				   unsigned long long fract, unsigned vco_div)
{
	unsigned long long rate;

	rate = parent_rate * fract;
	do_div(rate, RFPLL_MODULUS);
	rate += parent_rate * integer;

	return rate >> (vco_div + 1);
}

static int ad9361_calc_rfpll_divder(unsigned long long freq,
			     unsigned long long parent_rate, unsigned *integer,
			     unsigned *fract, int *vco_div, unsigned long long *vco_freq)
{
	unsigned long long tmp;
	int div;

	if (freq > MAX_CARRIER_FREQ_HZ || freq < MIN_CARRIER_FREQ_HZ)
		return -EINVAL;

	div = -1;

	while (freq < MIN_VCO_FREQ_HZ) {
		freq <<= 1;
		div++;
	}

	*vco_div = div;
	*vco_freq = freq;
	tmp = do_div(freq, parent_rate);
	tmp = tmp * RFPLL_MODULUS + (parent_rate >> 1);
	do_div(tmp, parent_rate);
	*integer = freq;
	*fract = tmp;

	return 0;
}

static unsigned long ad9361_rfpll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	unsigned long fract, integer;
	unsigned char buf[5];
	unsigned reg, div_mask, vco_div;

	switch (clk_priv->source) {
	case RX_RFPLL:
		reg = RX_FRACTIONAL_BYTE_2;
		div_mask = RX_VCO_DIV;
		break;
	case TX_RFPLL:
		reg = TX_FRACTIONAL_BYTE_2;
		div_mask = TX_VCO_DIV;
		break;
	default:
		return -EINVAL;
	}

	ad9361_spi_readm(clk_priv->spi, reg, &buf[0], ARRAY_SIZE(buf));

	vco_div = ad9361_spi_readf(clk_priv->spi,
					RFPLL_DIVIDERS, div_mask);

	fract = (buf[0] << 16) | (buf[1] << 8) | buf[2];
	integer = buf[3] << 8 | buf[4];

	return ad9361_to_clk(ad9361_calc_rfpll_freq(parent_rate, integer,
					      fract, vco_div));
}

static long ad9361_rfpll_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	unsigned long long vco;
	unsigned fract, integer;
	int vco_div, ret;

	ret = ad9361_calc_rfpll_divder(ad9361_from_clk(rate), *prate, &integer, &fract, &vco_div, &vco);
	if (ret < 0)
		return ret;

	return ad9361_to_clk(ad9361_calc_rfpll_freq(*prate, integer, fract, vco_div));
}

static int ad9361_rfpll_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct refclk_scale *clk_priv = to_clk_priv(hw);
	struct ad9361_rf_phy *phy = clk_priv->phy;
	unsigned long long vco;
	unsigned char buf[5];
	unsigned reg, div_mask, fract, integer;
	int vco_div, ret;

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, parent_rate);

	ret = ad9361_calc_rfpll_divder(ad9361_from_clk(rate), parent_rate,
				&integer, &fract, &vco_div, &vco);
	if (ret < 0)
		return ret;

	switch (clk_priv->source) {
	case RX_RFPLL:
		reg = RX_FRACTIONAL_BYTE_2;
		div_mask = RX_VCO_DIV;
		break;
	case TX_RFPLL:
		reg = TX_FRACTIONAL_BYTE_2;
		div_mask = TX_VCO_DIV;
		break;
	default:
		return -EINVAL;

	}

	ad9361_rfpll_vco_init(phy, div_mask == TX_VCO_DIV,
			      vco, parent_rate);

	buf[0] = fract >> 16;
	buf[1] = fract >> 8;
	buf[2] = fract & 0xFF;
	buf[3] = integer >> 8;
	buf[4] = integer & 0xFF;

	ad9361_spi_writem(clk_priv->spi, reg, buf, 5);
	ad9361_spi_writef(clk_priv->spi, RFPLL_DIVIDERS, div_mask, vco_div);

	/* Load Gain Table */
	if (clk_priv->source == RX_RFPLL) {
		ret = ad9361_load_gt(phy, ad9361_from_clk(rate));
		if (ret < 0)
			return ret;
	}

	/* For RX LO we typically have the tracking option enabled
	 * so for now do nothing here.
	 */
	if (phy->auto_cal_en && (clk_priv->source == TX_RFPLL))
		if (abs(phy->last_tx_quad_cal_freq - ad9361_from_clk(rate)) >
			phy->cal_threshold_freq) {

			set_bit(0, &phy->flags);
			INIT_COMPLETION(phy->complete);
			schedule_work(&phy->work);
			phy->last_tx_quad_cal_freq = ad9361_from_clk(rate);
		}

	return 0;
}

struct clk_ops rfpll_clk_ops = {
	.round_rate = ad9361_rfpll_round_rate,
	.set_rate = ad9361_rfpll_set_rate,
	.recalc_rate = ad9361_rfpll_recalc_rate,
};

static struct clk *ad9361_clk_register(struct ad9361_rf_phy *phy, const char *name,
		const char *parent_name, unsigned long flags,
		unsigned source)
{
	struct refclk_scale *clk_priv;
	struct clk_init_data init;
	struct clk *clk;

	clk_priv = kmalloc(sizeof(*clk_priv), GFP_KERNEL);
	if (!clk_priv) {
		pr_err("%s: could not allocate fixed factor clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	/* struct refclk_scale assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	init.name = name;

	switch (source) {
	case BBPLL_CLK:
		init.ops = &bbpll_clk_ops;
		break;
	case RX_RFPLL:
	case TX_RFPLL:
		init.ops = &rfpll_clk_ops;
		break;
	default:
		init.ops = &refclk_scale_ops;
	}

	init.flags = flags;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	clk = clk_register(&phy->spi->dev, &clk_priv->hw);
	phy->clk_data.clks[source] = clk;

	if (IS_ERR(clk))
		kfree(clk_priv);

	return clk;
}

static int register_clocks(struct ad9361_rf_phy *phy)
{

	phy->clk_data.clks = devm_kzalloc(&phy->spi->dev,
					 sizeof(*phy->clk_data.clks) *
					 NUM_AD9361_CLKS, GFP_KERNEL);
	if (!phy->clk_data.clks) {
		dev_err(&phy->spi->dev, "could not allocate memory\n");
		return -ENOMEM;
	}
	phy->clk_data.clk_num = NUM_AD9361_CLKS;

	/* Scaled Reference Clocks */
	phy->clks[TX_REFCLK] = ad9361_clk_register(phy,
					"tx_refclk", "ad9361_ext_refclk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					TX_REFCLK);

	phy->clks[RX_REFCLK] = ad9361_clk_register(phy,
					"rx_refclk", "ad9361_ext_refclk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					RX_REFCLK);

	phy->clks[BB_REFCLK] = ad9361_clk_register(phy,
					"bb_refclk", "ad9361_ext_refclk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					BB_REFCLK);

	/* Base Band PLL Clock */
	phy->clks[BBPLL_CLK] = ad9361_clk_register(phy,
					"bbpll_clk", "bb_refclk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					BBPLL_CLK);

	phy->clks[ADC_CLK] = ad9361_clk_register(phy,
					"adc_clk", "bbpll_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					ADC_CLK);

	phy->clks[R2_CLK] = ad9361_clk_register(phy,
					"r2_clk", "adc_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					R2_CLK);

	phy->clks[R1_CLK] = ad9361_clk_register(phy,
					"r1_clk", "r2_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					R1_CLK);

	phy->clks[CLKRF_CLK] = ad9361_clk_register(phy,
					"clkrf_clk", "r1_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					CLKRF_CLK);

	phy->clks[RX_SAMPL_CLK] = ad9361_clk_register(phy,
					"rx_sampl_clk", "clkrf_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					RX_SAMPL_CLK);


	phy->clks[DAC_CLK] = ad9361_clk_register(phy,
					"dac_clk", "adc_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					DAC_CLK);

	phy->clks[T2_CLK] = ad9361_clk_register(phy,
					"t2_clk", "dac_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					T2_CLK);

	phy->clks[T1_CLK] = ad9361_clk_register(phy,
					"t1_clk", "t2_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					T1_CLK);

	phy->clks[CLKTF_CLK] = ad9361_clk_register(phy,
					"clktf_clk", "t1_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					CLKTF_CLK);

	phy->clks[TX_SAMPL_CLK] = ad9361_clk_register(phy,
					"tx_sampl_clk", "clktf_clk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					TX_SAMPL_CLK);

	phy->clks[RX_RFPLL] = ad9361_clk_register(phy,
					"rx_rfpll", "rx_refclk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					RX_RFPLL);

	phy->clks[TX_RFPLL] = ad9361_clk_register(phy,
					"tx_rfpll", "tx_refclk",
					CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
					TX_RFPLL);


	return 0;
}

// static int ad9361_scale_table[][2] = {
// 	{2000, 0}, {2100, 6}, {2200, 7},
// 	{2300, 8}, {2400, 9}, {2500, 10},
// };
//
// static void ad9361_convert_scale_table(struct axiadc_converter *conv)
// {
// 	int i;
//
// 	for (i = 0; i < conv->chip_info->num_scales; i++)
// 		conv->chip_info->scale_table[i][0] =
// 			(conv->chip_info->scale_table[i][0] * 1000000ULL) >>
// 			conv->chip_info->channel[0].scan_type.realbits;
//
// }
//
// static ssize_t ad9361_show_scale_available(struct iio_dev *indio_dev,
// 				   uintptr_t private,
// 					   const struct iio_chan_spec *chan,
// 					   char *buf)
// {
// 	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
// 	int i, len = 0;
//
// 	for (i = 0; i < conv->chip_info->num_scales; i++)
// 		len += sprintf(buf + len, "0.%06u ",
// 			       conv->chip_info->scale_table[i][0]);
//
// 	len += sprintf(buf + len, "\n");
//
// 	return len;
// }
//
//
// static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
// 	{
// 		.name = "scale_available",
// 		.read = ad9361_show_scale_available,
// 		.shared = true,
// 	},
// 	{ },
// };

#define AIM_CHAN(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			BIT(IIO_CHAN_INFO_CALIBBIAS) |			\
			BIT(IIO_CHAN_INFO_CALIBPHASE),			\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	/*.ext_info = axiadc_ext_info,*/			\
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 16, 0)}



static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD9361] = {
		.name = "AD9361",
		.max_rate = 250000000UL,
// 		.scale_table = ad9361_scale_table,
// 		.num_scales = ARRAY_SIZE(ad9361_scale_table),
		.max_testmode = 0,
		.num_channels = 4,
		.channel[0] = AIM_CHAN(0, 0, 12, 's'),
		.channel[1] = AIM_CHAN(1, 1, 12, 's'),
		.channel[2] = AIM_CHAN(2, 2, 12, 's'),
		.channel[3] = AIM_CHAN(3, 3, 12, 's'),
	},
};
static struct attribute *ad9361_attributes[] = {
//	&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9361_attribute_group = {
	.attrs = ad9361_attributes,
};

static int ad9361_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk = clk_get_rate(conv->clk);

		return IIO_VAL_INT;

	}
	return -EINVAL;
}

static int ad9361_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long r_clk;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		r_clk = clk_round_rate(conv->clk, val);
		if (r_clk < 0 || r_clk > conv->chip_info->max_rate) {
			dev_warn(&conv->spi->dev,
				"Error setting ADC sample rate %ld", r_clk);
			return -EINVAL;
		}

		ret = clk_set_rate(conv->clk, r_clk);
		if (ret < 0)
			return ret;

		return 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9361_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int i;

	for (i = 0; i < conv->chip_info->num_channels; i++)
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
			     ADI_ENABLE);

	return 0;
}

static int ad9361_register_axi_converter(struct ad9361_rf_phy *phy)
{
	struct axiadc_converter *conv;
	struct spi_device *spi = phy->spi;
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->id = ad9361_spi_read(spi, PRODUCT_ID) & PRODUCT_ID_MASK;
	if (conv->id != spi_get_device_id(spi)->driver_data) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
  		ret = -ENODEV;
  		goto out;
	}

	conv->chip_info = &axiadc_chip_info_tbl[ID_AD9361];
	conv->adc_output_mode = AD9361_DEF_OUTPUT_MODE |
			OUTPUT_MODE_TWOS_COMPLEMENT;

	conv->write = ad9361_spi_write;
	conv->read = ad9361_spi_read;
	conv->write_raw = ad9361_write_raw;
	conv->read_raw = ad9361_read_raw;
	conv->post_setup = ad9361_post_setup;
	conv->attrs = &ad9361_attribute_group;
	conv->spi = spi;
	conv->phy = phy;

	conv->clk = phy->clks[RX_SAMPL_CLK];
	conv->adc_clk = clk_get_rate(conv->clk);

	spi_set_drvdata(spi, conv); /* Take care here */

	return 0;
out:
	spi_set_drvdata(spi, NULL);
	return ret;

}

enum ad9361_iio_dev_attr {
	AD9361_RF_BANDWIDTH,
	AD9361_ENSM_MODE,
	AD9361_ENSM_MODE_AVAIL,
	AD9361_CALIB_MODE,
	AD9361_CALIB_MODE_AVAIL,
	AD9361_RX_PATH_FREQ,
	AD9361_TX_PATH_FREQ,
	AD9361_TRX_RATE_GOV,
	AD9361_TRX_RATE_GOV_AVAIL,
	AD9361_FIR_RX_ENABLE,
	AD9361_FIR_TX_ENABLE,

};

static ssize_t ad9361_phy_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	long readin;
	int ret = 0;
	unsigned val;
	bool res;

	mutex_lock(&indio_dev->mlock);

	switch ((u32)this_attr->address) {
	case AD9361_RF_BANDWIDTH:
		ret = kstrtol(buf, 10, &readin);
		if (ret)
			break;

		if (phy->current_bw_Hz != readin)
			ret = ad9361_update_rf_bandwidth(phy, readin);
		else
			ret = 0;
		break;
	case AD9361_ENSM_MODE:
		if (sysfs_streq(buf, "tx"))
			val = ENSM_STATE_TX;
		else if (sysfs_streq(buf, "rx"))
			val = ENSM_STATE_RX;
		else if (sysfs_streq(buf, "alert"))
			val = ENSM_STATE_ALERT;
		else if (sysfs_streq(buf, "fdd"))
			val = ENSM_STATE_FDD;
		else if (sysfs_streq(buf, "sleep"))
			val = ENSM_STATE_SLEEP_WAIT;
//		else if (sysfs_streq(buf, "pinctrl")
		else
			break;
		ret = ad9361_ensm_set_state(phy, val);
		break;
	case AD9361_TRX_RATE_GOV:
		if (sysfs_streq(buf, "highest_osr"))
			phy->rate_governor = 0;
		else if (sysfs_streq(buf, "low_power"))
			phy->rate_governor = 1;
		else
			ret = -EINVAL;
		break;
	case AD9361_FIR_RX_ENABLE:
		ret = strtobool(buf, &res);
		if (ret < 0)
			break;

		phy->bypass_rx_fir = !res;
		ret = ad9361_validate_enable_fir(phy);
		if (ret < 0)
			phy->bypass_rx_fir = true;

		break;
	case AD9361_FIR_TX_ENABLE:
		ret = strtobool(buf, &res);
		if (ret < 0)
			break;

		phy->bypass_tx_fir = !res;
		ret = ad9361_validate_enable_fir(phy);
		if (ret < 0)
			phy->bypass_tx_fir = true;

		break;
	case AD9361_CALIB_MODE:
		val = 0;
		if (sysfs_streq(buf, "auto"))
			phy->auto_cal_en = true;
		else if (sysfs_streq(buf, "manual"))
			phy->auto_cal_en = false;
		else if (sysfs_streq(buf, "rx_quad"))
			val = RX_QUAD_CAL;
		else if (sysfs_streq(buf, "tx_quad"))
			val = TX_QUAD_CAL;
		else if (sysfs_streq(buf, "rf_dc_offs"))
			val = RFDC_CAL;
		else
			break;

		if (val)
			ret = ad9361_do_calib_run(phy, val);

		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9361_phy_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	int ret = 0;
	unsigned long clk[6];


	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case AD9361_RF_BANDWIDTH:
		ret = sprintf(buf, "%u\n", phy->current_bw_Hz);
		break;
	case AD9361_ENSM_MODE:
		ret = sprintf(buf, "%s\n",
			      ad9361_ensm_states[ad9361_spi_readf
			      (phy->spi, REG_DEV_STATE, ENSM_STATE_MASK)]);
		break;
	case AD9361_ENSM_MODE_AVAIL:
		ret = sprintf(buf, "%s\n", phy->pdata->fdd ?
				"sleep alert fdd pinctrl" :
				"sleep alert rx tx pinctrl");
		break;
	case AD9361_TX_PATH_FREQ:
		ad9361_get_trx_clock_chain(phy, NULL, clk);
		ret = sprintf(buf, "%lu %lu %lu %lu %lu %lu\n",
			      clk[0], clk[1], clk[2], clk[3], clk[4], clk[5]);
		break;
	case AD9361_RX_PATH_FREQ:
		ad9361_get_trx_clock_chain(phy, clk, NULL);
		ret = sprintf(buf, "%lu %lu %lu %lu %lu %lu\n",
			      clk[0], clk[1], clk[2], clk[3], clk[4], clk[5]);
		break;
	case AD9361_TRX_RATE_GOV:
		ret = sprintf(buf, "%s\n", phy->rate_governor ?
				 "low_power" : "highest_osr");
		break;
	case AD9361_TRX_RATE_GOV_AVAIL:
		ret = sprintf(buf, "%s\n", "low_power highest_osr");
		break;
	case AD9361_FIR_RX_ENABLE:
		ret = sprintf(buf, "%d\n", !phy->bypass_rx_fir);
		break;
	case AD9361_FIR_TX_ENABLE:
		ret = sprintf(buf, "%d\n", !phy->bypass_tx_fir);
		break;
	case AD9361_CALIB_MODE_AVAIL:
		ret = sprintf(buf, "auto manual tx_quad rf_dc_offs rx_quad\n");
		break;
	case AD9361_CALIB_MODE:
		ret = sprintf(buf, "%s\n", phy->auto_cal_en ? "auto" : "manual");
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(in_voltage_rf_bandwidth, S_IRUGO | S_IWUSR,
 			ad9361_phy_show,
 			ad9361_phy_store,
 			AD9361_RF_BANDWIDTH);

static IIO_DEVICE_ATTR(out_voltage_rf_bandwidth, S_IRUGO | S_IWUSR,
 			ad9361_phy_show,
 			ad9361_phy_store,
 			AD9361_RF_BANDWIDTH);

static IIO_DEVICE_ATTR(ensm_mode, S_IRUGO | S_IWUSR,
 			ad9361_phy_show,
 			ad9361_phy_store,
 			AD9361_ENSM_MODE);

static IIO_DEVICE_ATTR(ensm_mode_available, S_IRUGO,
 			ad9361_phy_show,
 			NULL,
 			AD9361_ENSM_MODE_AVAIL);

static IIO_DEVICE_ATTR(calib_mode, S_IRUGO | S_IWUSR,
 			ad9361_phy_show,
 			ad9361_phy_store,
 			AD9361_CALIB_MODE);

static IIO_DEVICE_ATTR(calib_mode_available, S_IRUGO,
 			ad9361_phy_show,
 			NULL,
 			AD9361_CALIB_MODE_AVAIL);

static IIO_DEVICE_ATTR(rx_path_rates, S_IRUGO,
 			ad9361_phy_show,
 			NULL,
 			AD9361_RX_PATH_FREQ);

static IIO_DEVICE_ATTR(tx_path_rates, S_IRUGO,
 			ad9361_phy_show,
 			NULL,
 			AD9361_TX_PATH_FREQ);

static IIO_DEVICE_ATTR(trx_rate_governor, S_IRUGO,
 			ad9361_phy_show,
 			ad9361_phy_store,
 			AD9361_TRX_RATE_GOV);

static IIO_DEVICE_ATTR(trx_rate_governor_available, S_IRUGO,
 			ad9361_phy_show,
 			NULL,
 			AD9361_TRX_RATE_GOV_AVAIL);

static IIO_DEVICE_ATTR(in_voltage_filter_fir_en, S_IRUGO | S_IWUSR,
 			ad9361_phy_show,
 			ad9361_phy_store,
 			AD9361_FIR_RX_ENABLE);

static IIO_DEVICE_ATTR(out_voltage_filter_fir_en, S_IRUGO | S_IWUSR,
 			ad9361_phy_show,
 			ad9361_phy_store,
 			AD9361_FIR_TX_ENABLE);

static struct attribute *ad9361_phy_attributes[] = {
	&iio_dev_attr_in_voltage_filter_fir_en.dev_attr.attr,
	&iio_dev_attr_out_voltage_filter_fir_en.dev_attr.attr,
	&iio_dev_attr_in_voltage_rf_bandwidth.dev_attr.attr,
	&iio_dev_attr_out_voltage_rf_bandwidth.dev_attr.attr,
	&iio_dev_attr_ensm_mode.dev_attr.attr,
	&iio_dev_attr_ensm_mode_available.dev_attr.attr,
	&iio_dev_attr_calib_mode.dev_attr.attr,
	&iio_dev_attr_calib_mode_available.dev_attr.attr,
	&iio_dev_attr_tx_path_rates.dev_attr.attr,
	&iio_dev_attr_rx_path_rates.dev_attr.attr,
	&iio_dev_attr_trx_rate_governor.dev_attr.attr,
	&iio_dev_attr_trx_rate_governor_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9361_phy_attribute_group = {
	.attrs = ad9361_phy_attributes,
};


static int ad9361_phy_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = ad9361_spi_write(phy->spi, reg, writeval);
	} else {
		*readval =  ad9361_spi_read(phy->spi, reg);
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t ad9361_phy_lo_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	unsigned long long readin;
	unsigned long tmp;
	int ret = 0;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch (chan->channel) {
	case 0:
		tmp = clk_set_rate(phy->clks[RX_RFPLL],
				   ad9361_to_clk(readin));
		break;

	case 1:
		tmp = clk_set_rate(phy->clks[TX_RFPLL],
				   ad9361_to_clk(readin));
		if (test_bit(0, &phy->flags))
			wait_for_completion(&phy->complete);

		break;

	default:
		ret = -EINVAL;
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9361_phy_lo_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	unsigned long long val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch (chan->channel) {
	case 0:
		val = ad9361_from_clk(clk_get_rate(phy->clks[RX_RFPLL]));
		break;

	case 1:
		val = ad9361_from_clk(clk_get_rate(phy->clks[TX_RFPLL]));
		break;

	default:
		ret = -EINVAL;
		val = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

#define _AD9361_EXT_LO_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9361_phy_lo_read, \
	.write = ad9361_phy_lo_write, \
	.private = _ident, \
}

static const struct iio_chan_spec_ext_info ad9361_phy_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_AD9361_EXT_LO_INFO("frequency", 0),
	{ },
};

static int ad9361_set_agc_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);

	phy->agc_mode[chan->channel] = mode;

	return 0;
}

static int ad9361_get_agc_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);

	return phy->agc_mode[chan->channel];
}


static const char * const ad9361_agc_modes[] =
// 	{"manual", "fast_attack", "slow_attack", "hybrid"};
	{"manual"};

static const struct iio_enum ad9361_agc_modes_available = {
	.items = ad9361_agc_modes,
	.num_items = ARRAY_SIZE(ad9361_agc_modes),
	.get = ad9361_get_agc_mode,
	.set = ad9361_set_agc_mode,

};

static ssize_t ad9361_phy_rx_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
//	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	unsigned long long readin;
	int ret = 0;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch (chan->channel) {
	case 0:

		break;

	case 1:

		break;

	default:
		ret = -EINVAL;
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9361_phy_rx_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	struct rf_rssi rssi = {0};
	int val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);

	rssi.ant = chan->channel + 1;
	rssi.duration = 1;
	ret = ad9361_read_rssi(phy, &rssi);
	val = rssi.symbol;

// 	switch (chan->channel) {
// 	case 0:
//
// 		break;
//
// 	case 1:
//
// 		break;
//
// 	default:
// 		ret = -EINVAL;
// 		val = 0;
// 	}
	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : sprintf(buf, "-%u.%02u dB\n", val/100, val%100);
}

#define _AD9361_EXT_RX_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9361_phy_rx_read, \
	.write = ad9361_phy_rx_write, \
	.private = _ident, \
}


static const struct iio_chan_spec_ext_info ad9361_phy_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	IIO_ENUM_AVAILABLE("gain_control_mode", &ad9361_agc_modes_available),
	IIO_ENUM("gain_control_mode",true, &ad9361_agc_modes_available),
	_AD9361_EXT_RX_INFO("rssi", 1),
	{ },
};

static int ad9361_phy_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			ret = ad9361_get_tx_atten(phy, chan->channel + 1);
			if (ret < 0)
				return -EINVAL;

			*val = -1 * (ret / 1000);
			*val2 = (ret % 1000) * 1000;
			if (!*val)
				*val2 *= -1;

		} else {
			struct rf_rx_gain rx_gain = {0};
			ret = ad9361_get_rx_gain(phy, chan->channel + 1, &rx_gain);
			*val = rx_gain.gain_db;
			*val2 = 0;
		}
		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->output)
			*val = (int)clk_get_rate(phy->clks[TX_SAMPL_CLK]);
		else
			*val = (int)clk_get_rate(phy->clks[RX_SAMPL_CLK]);
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int ad9361_phy_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);
	unsigned code;
	unsigned long rx[6], tx[6];
	int ret;


	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			if (val > 0 || (val == 0 && val2 > 0)) {
				ret = -EINVAL;
				goto out;
			}

			code = ((abs(val) * 1000) + (abs(val2) / 1000));
			ret = ad9361_set_tx_atten(phy, code,
				chan->channel == 0, chan->channel == 1);
		} else {
			struct rf_rx_gain rx_gain = {0};
			rx_gain.gain_db = val;
			ret = ad9361_set_rx_gain(phy, chan->channel + 1, &rx_gain);
		}
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:

		ret = ad9361_calculate_rf_clock_chain(phy, val,
			phy->rate_governor, rx, tx);
		if (ret < 0)
			goto out;
		ad9361_set_trx_clock_chain(phy, rx, tx);
		ret = ad9361_update_rf_bandwidth(phy, phy->current_bw_Hz);
		break;

	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_chan_spec ad9361_phy_chan[] = {
{	/* RX LO */
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.extend_name = "RX_LO",
	.ext_info = ad9361_phy_ext_info,
}, {	/* TX LO */
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 1,
	.extend_name = "TX_LO",
	.ext_info = ad9361_phy_ext_info,
}, {	/* TX1 */
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
}, {	/* TX2 */
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
}, {	/* RX1 */
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.ext_info = ad9361_phy_rx_ext_info,

}, {	/* RX2 */
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.channel = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.ext_info = ad9361_phy_rx_ext_info,
}};

static const struct iio_info ad9361_phy_info = {
	.read_raw = &ad9361_phy_read_raw,
	.write_raw = &ad9361_phy_write_raw,
	.debugfs_reg_access = &ad9361_phy_reg_access,
	.attrs = &ad9361_phy_attribute_group,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_OF
struct ad9361_dport_config {
	u8 reg;
	u8 offset;
	char name[40];
};

static const struct ad9361_dport_config ad9361_dport_config[] = {
	{1, 7, "adi,pp-tx-swap-enable"},
	{1, 6, "adi,pp-rx-swap-enable"},
	{1, 5, "adi,tx-channel-swap-enable"},
	{1, 4, "adi,rx-channel-swap-enable"},
	{1, 3, "adi,rx-frame-pulse-mode-enable"},
//	{1, 2, "adi,2t2r-timing-enable"},
	{1, 1, "adi,invert-data-bus-enable"},
	{1, 0, "adi,invert-data-clk-enable"},
	{2, 7, "adi,fdd-alt-word-order-enable"},
	{2, 2, "adi,invert-rx-frame-enable"},
	{3, 7, "adi,fdd-rx-rate-2tx-enable"},
	{3, 6, "adi,swap-ports-enable"},
	{3, 5, "adi,single-data-rate-enable"},
	{3, 4, "adi,lvds-mode-enable"},
	{3, 3, "adi,half-duplex-mode-enable"},
	{3, 2, "adi,single-port-mode-enable"},
	{3, 1, "adi,full-port-enable"},
	{3, 0, "adi,full-duplex-swap-bits-enable"},
};

static struct ad9361_phy_platform_data *ad9361_phy_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ad9361_phy_platform_data *pdata;
	unsigned int tmp;
	unsigned long long tmpl;
	u32 array[6] = {0};
	int ret, i;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}


	pdata->fdd = of_property_read_bool(np,
			"adi,frequency-division-duplex-mode-enable");

	pdata->ensm_pin_level_mode = of_property_read_bool(np,
			"adi,ensm-enable-pin-level-mode-enable");

	pdata->ensm_pin_ctrl = of_property_read_bool(np,
			"adi,ensm-enable-txnrx-control-enable");

	for (i = 0; i < ARRAY_SIZE(ad9361_dport_config); i++)
		pdata->pp_conf[ad9361_dport_config[i].reg - 1] |=
			(of_property_read_bool(np, ad9361_dport_config[i].name)
			<< ad9361_dport_config[i].offset);

	tmp = 0;
	of_property_read_u32(np, "adi,delay-rx-data", &tmp);
	pdata->pp_conf[1] |= (tmp & 0x3);

	tmp = 0;
	of_property_read_u32(np, "adi,rx-data-clock-delay", &tmp);
	pdata->rx_clk_data_delay = (tmp & 0xF) << 4;
	tmp = 0;
	of_property_read_u32(np, "adi,rx-data-delay", &tmp);
	pdata->rx_clk_data_delay |= (tmp & 0xF);

	tmp = 0;
	of_property_read_u32(np, "adi,tx-fb-clock-delay", &tmp);
	pdata->tx_clk_data_delay = (tmp & 0xF) << 4;
	tmp = 0;
	of_property_read_u32(np, "adi,tx-data-delay", &tmp);
	pdata->tx_clk_data_delay |= (tmp & 0xF);


	tmp = 75;
	of_property_read_u32(np, "adi,lvds-bias-mV", &tmp);
	pdata->lvds_bias_ctrl = (tmp / 75) & 0x7;
	pdata->lvds_bias_ctrl |= (of_property_read_bool(np,
			"adi,lvds-rx-onchip-termination-enable") << 5);


	pdata->rx2tx2 = of_property_read_bool(np, "adi,2rx-2tx-mode-enable");

	pdata->fdd = of_property_read_bool(np,
			"adi,frequency-division-duplex-mode-enable");

	pdata->split_gt = of_property_read_bool(np,
			"adi,split-gain-table-mode-enable");

// 	pdata->ensm_pin_ctl_en = of_property_read_bool(np,
// 			"adi,ensm-state-pincontrol-enable");

	tmp = 0;
	of_property_read_u32(np, "adi,rx-rf-port-input-select", &tmp);
	pdata->rf_rx_input_sel = tmp;

	tmp = 0;
	of_property_read_u32(np, "adi,tx-rf-port-input-select", &tmp);
	pdata->rf_tx_output_sel = tmp;

	tmpl = 2400000000ULL;
	of_property_read_u64(np, "adi,rx-synthesizer-frequency-hz", &tmpl);
	pdata->rx_synth_freq = tmpl;

	tmpl = 2440000000ULL;
 	of_property_read_u64(np, "adi,tx-synthesizer-frequency-hz", &tmpl);
	pdata->tx_synth_freq = tmpl;

	ret = of_property_read_u32_array(np, "adi,dcxo-coarse-and-fine-tune",
			      array, 2);
	if (ret < 0) {
		if (ret == -EINVAL)
			pdata->use_extclk = true;
		else
			return NULL;
	}

	pdata->dcxo_coarse = array[0];
	pdata->dcxo_fine = array[1];


	ret = of_property_read_u32_array(np, "adi,rx-path-clock-frequencies",
			      pdata->rx_path_clks, ARRAY_SIZE(pdata->rx_path_clks));
	if (ret < 0)
		return NULL;

	ret = of_property_read_u32_array(np, "adi,tx-path-clock-frequencies",
			      pdata->tx_path_clks, ARRAY_SIZE(pdata->tx_path_clks));
	if (ret < 0)
		return NULL;

	tmp = 0;
	of_property_read_u32(np, "adi,rf-bandwidth-hz", &tmp);
	pdata->rf_bandwidth_Hz = tmp;

	tmp = 10000; /* -10 dB */
	of_property_read_u32(np, "adi,tx-attenuation-mdB", &tmp);
	pdata->tx_atten = tmp;

// 	ret = of_get_gpio(np, 0);
// 	if (ret < 0)
// 		pdata->gpio_lock_detect = -1;
// 	else
// 		pdata->gpio_lock_detect = ret;



	return pdata;
}
#else
static
struct ad9361_phy_platform_data *ad9361_phy_parse_dt(struct device *dev)
{
	return NULL;
}
#endif



static ssize_t
ad9361_fir_bin_write(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);

	return ad9361_parse_fir(phy, buf, count);
}

static ssize_t
ad9361_fir_bin_read(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct ad9361_rf_phy *phy = iio_priv(indio_dev);

	if (off)
		return 0;

	return sprintf(buf, "FIR Rx: %d,%d Tx: %d,%d\n",
		       phy->rx_fir_ntaps, phy->rx_fir_dec,
			phy->tx_fir_ntaps, phy->tx_fir_int);
}

static int ad9361_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9361_rf_phy *phy;
	struct clk *clk = NULL;
	int ret, rev;

	dev_info(&spi->dev, "%s : enter", __func__);

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;


	indio_dev = iio_device_alloc(sizeof(*phy));
	if (indio_dev == NULL)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;

	phy->pdata = ad9361_phy_parse_dt(&spi->dev);
	if (phy->pdata == NULL)
		return -EINVAL;

	phy->spi = spi;
	phy->clk_refin = clk;

	phy->current_table = RXGAIN_TBLS_END;
	phy->bypass_tx_fir = true;
	phy->bypass_rx_fir = true;
	phy->rate_governor = 1;

	ad9361_spi_write(spi, SPI_CONFIGURATION, 0x81); /* RESET */
	ad9361_spi_write(spi, SPI_CONFIGURATION, 0x0);

	ret = ad9361_spi_read(spi, PRODUCT_ID);
	if ((ret & PRODUCT_ID_MASK) != PRODUCT_ID_9361) {
		dev_err(&spi->dev, "%s : Unsupported PRODUCT_ID 0x%X",
			__func__, ret);
		ret = -ENODEV;
		goto out;
	}

	rev = ret & REV_MASK;

	INIT_WORK(&phy->work, ad9361_work_func);
	init_completion(&phy->complete);

	register_clocks(phy);

	ad9361_init_gain_tables(phy);

	ret = ad9361_setup(phy);
	if (ret < 0)
		goto out;

	of_clk_add_provider(spi->dev.of_node,
			    of_clk_src_onecell_get, &phy->clk_data);

	sysfs_bin_attr_init(&phy->bin);
	phy->bin.attr.name = "filter_fir_config";
	phy->bin.attr.mode = S_IWUSR;
	phy->bin.write = ad9361_fir_bin_write;
	phy->bin.read = ad9361_fir_bin_read;
	phy->bin.size = 4096;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ad9361-phy";
	indio_dev->info = &ad9361_phy_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9361_phy_chan;
	indio_dev->num_channels = ARRAY_SIZE(ad9361_phy_chan);

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out;
	ret = ad9361_register_axi_converter(phy);
	if (ret < 0)
		goto out1;
	ret = sysfs_create_bin_file(&indio_dev->dev.kobj, &phy->bin);
	if (ret < 0)
		goto out1;

	dev_info(&spi->dev, "%s : AD9361 Rev %d successfully initialized",
		 __func__, rev);

	return 0;

out1:
	iio_device_unregister(indio_dev);

out:
	clk_disable_unprepare(clk);
	iio_device_free(indio_dev);
	spi_set_drvdata(spi, NULL);

	return ret;
}

static int ad9361_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9361_rf_phy *phy = conv->phy;

	sysfs_remove_bin_file(&phy->indio_dev->dev.kobj, &phy->bin);
	iio_device_unregister(phy->indio_dev);
	clk_disable_unprepare(conv->clk);
	iio_device_free(phy->indio_dev);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad9361_id[] = {
	{"ad9361", PRODUCT_ID_9361},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9361_id);

static struct spi_driver ad9361_driver = {
	.driver = {
		.name	= "ad9361",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9361_probe,
	.remove		= ad9361_remove,
	.id_table	= ad9361_id,
};
module_spi_driver(ad9361_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9361 ADC");
MODULE_LICENSE("GPL v2");
