/*
 * AD9517 SPI Clock Generator with integrated VCO
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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/firmware.h>
#include <linux/of.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include <linux/iio/iio.h>

#define FIRMWARE	"ad9517.stp"

#define AD_READ		(1 << 15)
#define AD_WRITE		(0 << 15)
#define AD_CNT(x)	(((x) - 1) << 13)
#define AD_ADDR(x)	((x) & 0xFFF)

/*
* AD9517-X Registers
*/
#define AD9517_SERCONF	0x00
#define AD9517_PARTID	0x03
#define AD9517_RB_CTL	0x04
#define AD9517_PFD_CP	0x10
#define AD9517_RCNT_L	0x11
#define AD9517_RCNT_H	0x12
#define AD9517_ACNT	0x13
#define AD9517_BCNT_L	0x14
#define AD9517_BCNT_H	0x15
#define AD9517_PLL1	0x16
#define AD9517_PLL2	0x17
#define AD9517_PLL3	0x18
#define AD9517_PLL4	0x19
#define AD9517_PLL5	0x1A
#define AD9517_PLL6	0x1B
#define AD9517_PLL7	0x1C
#define AD9517_PLL8	0x1D
#define AD9517_PLL9	0x1E
#define AD9517_PLL_RB	0x1F

#define AD9517_OUT4_DELAY_BP 0xA0
#define AD9517_OUT4_DELAY_FS 0xA1
#define AD9517_OUT4_DELAY_FR 0xA2
#define AD9517_OUT5_DELAY_BP 0xA3
#define AD9517_OUT5_DELAY_FS 0xA4
#define AD9517_OUT5_DELAY_FR 0xA5
#define AD9517_OUT6_DELAY_BP 0xA6
#define AD9517_OUT6_DELAY_FS 0xA7
#define AD9517_OUT6_DELAY_FR 0xA8
#define AD9517_OUT7_DELAY_BP 0xA9
#define AD9517_OUT7_DELAY_FS 0xAA
#define AD9517_OUT7_DELAY_FR 0xAB

#define AD9517_OUT0_LVPECL 0xF0
#define AD9517_OUT1_LVPECL 0xF1
#define AD9517_OUT2_LVPECL 0xF4
#define AD9517_OUT3_LVPECL 0xF5

#define AD9517_OUT4_LVCMOS 0x140
#define AD9517_OUT5_LVCMOS 0x141
#define AD9517_OUT6_LVCMOS 0x142
#define AD9517_OUT7_LVCMOS 0x143

/* LVPECL Channel Dividers */
#define AD9517_PECLDIV0_1 0x190
#define AD9517_PECLDIV0_2 0x191
#define AD9517_PECLDIV0_3 0x192
#define AD9517_PECLDIV1_1 0x196
#define AD9517_PECLDIV1_2 0x197
#define AD9517_PECLDIV1_3 0x198

/* LVDS/CMOS Channel Dividers */
#define AD9517_CMOSDIV2_1	0x199
#define AD9517_CMOSDIV2_1_PHO	0x19A
#define AD9517_CMOSDIV2_2	0x19B
#define AD9517_CMOSDIV2_BYPASS	0x19C
#define AD9517_CMOSDIV2_DCCOFF	0x19D
#define AD9517_CMOSDIV3_1	0x19E
#define AD9517_CMOSDIV3_PHO	0x19F
#define AD9517_CMOSDIV3_2	0x1A0
#define AD9517_CMOSDIV3_BYPASS	0x1A1
#define AD9517_CMOSDIV3_DCCOFF	0x1A2

/* VCO Divider and CLK Input */
#define AD9517_VCO_DIVIDER	0x1E0
#define AD9517_INPUT_CLKS	0x1E1
#define AD9517_POWDOWN_SYNC	0x230

/* Update All Registers */
#define AD9517_TRANSFER		0x232

#define AD9517_PLL3_VCO_CAL	(1 << 0)
#define AD9517_TRANSFER_NOW	(1 << 0)
#define AD9517_PLL1_BCNT_BP	(1 << 3)
#define AD9517_VCO_DIVIDER_BP	(1 << 0)
#define AD9517_VCO_DIVIDER_SEL	(1 << 1)
#define AD9517_PECLDIV_VCO_SEL	(1 << 1)
#define AD9517_PECLDIV_3_BP	(1 << 7)
#define AD9517_CMOSDIV_BYPASS_2 (1 << 5)
#define AD9517_CMOSDIV_BYPASS_1 (1 << 4)
#define AD9517_SOFT_RESET	(0x24)
#define AD9517_SDO_ACTIVE	(0x81)
#define AD9517_LONG_INSTR	(0x18)

enum outputs {
	OUT_0,
	OUT_1,
	OUT_2,
	OUT_3,
	OUT_4,
	OUT_5,
	OUT_6,
	OUT_7,
	NUM_OUTPUTS,
};

#define AD9517_REG(addr, val)	((addr << 16) | (val & 0xFF))

struct ad9517_platform_data {
	unsigned long *regs;
	unsigned num_regs;
};

struct ad9517_outputs {
	struct ad9517_state *st;
	struct clk_hw hw;
	unsigned long freq;
	const char *parent_name;
	bool is_enabled;
	unsigned char pwrdwn_reg;
	unsigned char pwrdwn_bit;

};

struct ad9517_state {
	struct spi_device *spi;
	unsigned char regs[AD9517_TRANSFER+1];
	struct ad9517_outputs output[NUM_OUTPUTS];
	struct clk_onecell_data clk_data;
	unsigned long refin_freq;
	unsigned long clkin_freq;
	char *div0123_clk_parent_name;
	char *vco_divin_clk_parent_name;
};

#define IS_FD				(1 << 7)
#define AD9517_PLL1_PRESCALER_MASK	0x7
static const unsigned char to_prescaler[] = {
	1 | IS_FD,
	2 | IS_FD,
	2,
	4,
	8,
	16,
	32,
	3 | IS_FD,
};

enum {
	PWRDWN_REG,
	PWRDWN_MASK,
	PWRDWN_BIT,
};

static const unsigned short output_pwrdwn_lut[NUM_OUTPUTS][3] = {
	{AD9517_OUT0_LVPECL, 0x3, 1},
	{AD9517_OUT1_LVPECL, 0x3, 1},
	{AD9517_OUT2_LVPECL, 0x3, 1},
	{AD9517_OUT3_LVPECL, 0x3, 1},
	{AD9517_OUT4_LVCMOS, 0x1, 0},
	{AD9517_OUT5_LVCMOS, 0x1, 0},
	{AD9517_OUT6_LVCMOS, 0x1, 0},
	{AD9517_OUT7_LVCMOS, 0x1, 0},
};

#define to_ad9517_clk_output(_hw) container_of(_hw, struct ad9517_outputs, hw)

static int ad9517_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;
	u16 cmd;

	cmd = AD_READ | AD_CNT(1) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;


	ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);
	if (ret < 0)
		return ret;

	return buf[2];
}

static int ad9517_write(struct spi_device *spi,
			 unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;
	u16 cmd;

	cmd = AD_WRITE | AD_CNT(1) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;
	buf[2] = val;

	ret = spi_write(spi, buf, 3);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9517_parse_firmware(struct ad9517_state *st,
				 char *data, unsigned size)
{
	char *line;
	int ret;
	unsigned addr, val1, val2;

	while ((line = strsep(&data, "\n"))) {
		if (line >= data + size)
			break;

		ret = sscanf(line, "\"%x\",\"%x\",\"%x\"", &addr, &val1, &val2);
		if (ret == 3) {
			if (addr > AD9517_TRANSFER)
				return -EINVAL;
			st->regs[addr] = val2 & 0xFF;
		}
	}
	return 0;
}

static int ad9517_parse_pdata(struct ad9517_state *st,
				 struct ad9517_platform_data *pdata)
{
	int i;
	unsigned addr;

	if (!pdata->num_regs || (pdata->num_regs > AD9517_TRANSFER))
		return -EINVAL;

	for (i = 0; i < pdata->num_regs; i++) {
		addr = pdata->regs[i] >> 16;
		if (addr > AD9517_TRANSFER)
			return -EINVAL;
		st->regs[addr] = pdata->regs[i] & 0xFF;
	}
	return 0;
}

static int ad9517_setup(struct ad9517_state *st)
{
	struct spi_device *spi = st->spi;
	int ret, reg;
	unsigned cal_delay_ms, d1, d2;
	unsigned long pll_a_cnt, pll_b_cnt, pll_r_cnt, prescaler;
	unsigned long vco_freq;
	unsigned long vco_divin_freq;
	unsigned long div0123_freq;

	/* Setup PLL */
	for (reg = AD9517_PFD_CP; reg <= AD9517_PLL8; reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup fine delay adjust OUT4..OUT7 */
	for (reg = AD9517_OUT4_DELAY_BP; reg <= AD9517_OUT7_DELAY_FR; reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup LVPECL outputs OUT0..OUT3 */
	for (reg = AD9517_OUT0_LVPECL; reg <= AD9517_OUT3_LVPECL; reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup LVCMOS outputs OUT4..OUT7 */
	for (reg = AD9517_OUT4_LVCMOS; reg <= AD9517_OUT7_LVCMOS; reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup PECL Channel Dividers */
	for (reg = AD9517_PECLDIV0_1; reg <= AD9517_PECLDIV1_3; reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup LVDS/CMOS Channel Dividers */
	for (reg = AD9517_CMOSDIV2_1; reg <= AD9517_CMOSDIV3_DCCOFF; reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup VCO Divier and CLK Input */
	ret = ad9517_write(spi, AD9517_VCO_DIVIDER,
			   st->regs[AD9517_VCO_DIVIDER]);
	if (ret < 0)
		return ret;

	ret = ad9517_write(spi, AD9517_INPUT_CLKS, st->regs[AD9517_INPUT_CLKS]);
	if (ret < 0)
		return ret;

	/* Setup System */
	ret = ad9517_write(spi, AD9517_POWDOWN_SYNC,
			   st->regs[AD9517_POWDOWN_SYNC]);
	if (ret < 0)
		return ret;

	ret = ad9517_write(spi, AD9517_PLL3,
			   st->regs[AD9517_PLL3] & ~AD9517_PLL3_VCO_CAL);
	if (ret < 0)
		return ret;

	/* Update all registers */
	ret = ad9517_write(spi, AD9517_TRANSFER, AD9517_TRANSFER_NOW);
	if (ret < 0)
		return ret;

	/* Calibrate VCO */
	ret = ad9517_write(spi, AD9517_PLL3,
			   st->regs[AD9517_PLL3] | AD9517_PLL3_VCO_CAL);
	if (ret < 0)
		return ret;

	/* Update all registers */
	ret = ad9517_write(spi, AD9517_TRANSFER, AD9517_TRANSFER_NOW);
	if (ret < 0)
		return ret;

	/* Get PLL settings */
	pll_r_cnt = st->regs[AD9517_RCNT_H] << 8 | st->regs[AD9517_RCNT_L];
	pll_a_cnt = st->regs[AD9517_ACNT];
	pll_b_cnt = st->regs[AD9517_BCNT_H] << 8 | st->regs[AD9517_BCNT_L];

	if (st->regs[AD9517_PLL1] & AD9517_PLL1_BCNT_BP)
		pll_b_cnt = 1;

	prescaler = to_prescaler[st->regs[AD9517_PLL1] &
		    AD9517_PLL1_PRESCALER_MASK];

	if (prescaler & IS_FD)
		pll_a_cnt = 0;

	prescaler &= ~IS_FD;

	vco_freq = (st->refin_freq / pll_r_cnt * (prescaler  *
			pll_b_cnt + pll_a_cnt));

	/* tcal = 4400 * Rdiv * cal_div / Refin */
	cal_delay_ms = (4400 * pll_r_cnt *
		(2 << ((st->regs[AD9517_PLL3] >> 1) & 0x3))) /
		(st->refin_freq / 1000);

	msleep(cal_delay_ms);

	/* Internal clock distribution */

	if (st->regs[AD9517_INPUT_CLKS] & AD9517_VCO_DIVIDER_SEL) {
		if (!vco_freq)
			return -EINVAL;
		vco_divin_freq = vco_freq;
		st->vco_divin_clk_parent_name = "refclk";
	} else {
		if (!st->clkin_freq)
			return -EINVAL;
		vco_divin_freq = st->clkin_freq;
		st->vco_divin_clk_parent_name = "clkin";
	}

	if (st->regs[AD9517_INPUT_CLKS] & AD9517_VCO_DIVIDER_BP) {
		if (!st->clkin_freq)
			return -EINVAL;
		div0123_freq = st->clkin_freq;
		st->div0123_clk_parent_name = "clkin";
	} else {
		div0123_freq = vco_divin_freq /
			((st->regs[AD9517_VCO_DIVIDER] & 0x7) + 2);
		st->div0123_clk_parent_name = st->vco_divin_clk_parent_name;
	}

	/* Outputs */
	/* 0..1 */

	if (st->regs[AD9517_PECLDIV0_3] & AD9517_PECLDIV_VCO_SEL) {
		st->output[OUT_0].freq =
			st->output[OUT_1].freq =
			vco_divin_freq;
		st->output[OUT_0].parent_name =
			st->output[OUT_1].parent_name =
			st->vco_divin_clk_parent_name;
	} else {
		if (st->regs[AD9517_PECLDIV0_2] & AD9517_PECLDIV_3_BP)
			d1 = 1;
		else
			d1 = (st->regs[AD9517_PECLDIV0_1] & 0xF) +
				(st->regs[AD9517_PECLDIV0_1] >> 4) + 2;

		st->output[OUT_0].freq =
			st->output[OUT_1].freq =
			div0123_freq / d1;

		st->output[OUT_0].parent_name =
			st->output[OUT_1].parent_name =
			st->div0123_clk_parent_name;
	}

	/* 2..3 */

	if (st->regs[AD9517_PECLDIV1_3] & AD9517_PECLDIV_VCO_SEL) {
		st->output[OUT_2].freq =
			st->output[OUT_3].freq =
			vco_divin_freq;
		st->output[OUT_2].parent_name =
			st->output[OUT_3].parent_name =
			st->vco_divin_clk_parent_name;
	} else {
		if (st->regs[AD9517_PECLDIV1_2] & AD9517_PECLDIV_3_BP)
			d1 = 1;
		else
			d1 = (st->regs[AD9517_PECLDIV1_1] & 0xF) +
				(st->regs[AD9517_PECLDIV1_1] >> 4) + 2;

		st->output[OUT_2].freq =
			st->output[OUT_3].freq =
			div0123_freq / d1;

		st->output[OUT_2].parent_name =
			st->output[OUT_3].parent_name =
			st->div0123_clk_parent_name;
	}

	/* 4..5 */

	if (st->regs[AD9517_CMOSDIV2_BYPASS] & AD9517_CMOSDIV_BYPASS_1)
		d1 = 1;
	else
		d1 = (st->regs[AD9517_CMOSDIV2_1] & 0xF) +
			(st->regs[AD9517_CMOSDIV2_1] >> 4) + 2;

	if (st->regs[AD9517_CMOSDIV2_BYPASS] & AD9517_CMOSDIV_BYPASS_2)
		d2 = 1;
	else
		d2 = (st->regs[AD9517_CMOSDIV2_2] & 0xF) +
			(st->regs[AD9517_CMOSDIV2_2] >> 4) + 2;

	st->output[OUT_4].freq =
		st->output[OUT_5].freq =
		div0123_freq / (d1 * d2);

	st->output[OUT_4].parent_name =
		st->output[OUT_5].parent_name =
		st->div0123_clk_parent_name;

	/* 6..7 */

	if (st->regs[AD9517_CMOSDIV3_BYPASS] & AD9517_CMOSDIV_BYPASS_1)
		d1 = 1;
	else
		d1 = (st->regs[AD9517_CMOSDIV3_1] & 0xF) +
			(st->regs[AD9517_CMOSDIV3_1] >> 4) + 2;

	if (st->regs[AD9517_CMOSDIV3_BYPASS] & AD9517_CMOSDIV_BYPASS_2)
		d2 = 1;
	else
		d2 = (st->regs[AD9517_CMOSDIV3_2] & 0xF) +
			(st->regs[AD9517_CMOSDIV3_2] >> 4) + 2;

	st->output[OUT_6].freq =
		st->output[OUT_7].freq =
		div0123_freq / (d1 * d2);

	st->output[OUT_6].parent_name =
		st->output[OUT_7].parent_name =
		st->div0123_clk_parent_name;

	return 0;
}

static unsigned long ad9517_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	return to_ad9517_clk_output(hw)->freq;
}

static int ad9517_is_enabled(struct clk_hw *hw)
{
	return to_ad9517_clk_output(hw)->is_enabled;
}

static const struct clk_ops ad9517_clk_ops = {
	.recalc_rate = ad9517_recalc_rate,
	.is_enabled = ad9517_is_enabled,
};

static struct clk *ad9517_clk_register(struct ad9517_state *st, unsigned num)
{
	struct clk_init_data init;
	struct ad9517_outputs *output = &st->output[num];
	struct clk *clk;
	char name[8];

	sprintf(name, "out%d", num);

	init.name = name;
	init.ops = &ad9517_clk_ops;
	init.parent_names = &output->parent_name;
	init.num_parents = 1;

	output->hw.init = &init;
	output->st = st;

	/* register the clock */
	clk = clk_register(&st->spi->dev, &output->hw);
	st->clk_data.clks[num] = clk;

	return clk;
}

static int ad9517_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct ad9517_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = ad9517_write(st->spi, reg, writeval);
		/* Update all registers */
		ad9517_write(st->spi, AD9517_TRANSFER, AD9517_TRANSFER_NOW);
	} else {
		ret = ad9517_read(st->spi, reg);
		if (ret < 0)
			goto out_unlock;
		*readval = ret;
		ret = 0;
	}

out_unlock:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int ad9517_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad9517_state *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*val = st->output[chan->channel].is_enabled;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		*val = st->output[chan->channel].freq;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
};

static const struct iio_info ad9517_info = {
	.read_raw = &ad9517_read_raw,
	.debugfs_reg_access = &ad9517_reg_access,
	.driver_module = THIS_MODULE,
};

#define AD9517_CHAN(_chan)					\
	{ .type = IIO_ALTVOLTAGE,				\
	  .indexed = 1,						\
	  .channel = _chan,					\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | 	\
			BIT(IIO_CHAN_INFO_FREQUENCY),		\
	  .output = 1}

static const struct iio_chan_spec ad9517_chan[NUM_OUTPUTS] = {
	AD9517_CHAN(OUT_0),
	AD9517_CHAN(OUT_1),
	AD9517_CHAN(OUT_2),
	AD9517_CHAN(OUT_3),
	AD9517_CHAN(OUT_4),
	AD9517_CHAN(OUT_5),
	AD9517_CHAN(OUT_6),
	AD9517_CHAN(OUT_7),
};

static int ad9517_probe(struct spi_device *spi)
{
	struct ad9517_platform_data *pdata = spi->dev.platform_data;
	struct iio_dev *indio_dev;
	int out, ret, conf;
	const struct firmware *fw;
	struct ad9517_state *st;
	struct clk *clk, *ref_clk, *clkin;
	bool spi3wire = of_property_read_bool(
			spi->dev.of_node, "adi,spi-3wire-enable");

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	conf = AD9517_LONG_INSTR |
		((spi->mode & SPI_3WIRE || spi3wire) ? 0 : AD9517_SDO_ACTIVE);

	ret = ad9517_write(spi, AD9517_SERCONF,  conf | AD9517_SOFT_RESET);
	if (ret < 0)
		return ret;

	ret = ad9517_write(spi, AD9517_SERCONF, conf);
	if (ret < 0)
		return ret;

	ret = ad9517_read(spi, AD9517_PARTID);
	if (ret < 0)
		return ret;
	if (ret != spi_get_device_id(spi)->driver_data) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", ret);
 		return -ENODEV;
	}

	if (!pdata) {
		const char *name;
		if (spi->dev.of_node) {
			if (of_property_read_string(spi->dev.of_node, "firmware", &name))
				name = FIRMWARE;
		} else {
			name = FIRMWARE;
		}

		ret = request_firmware(&fw, name, &spi->dev);
		if (ret) {
			dev_err(&spi->dev,
				"request_firmware() failed with %i\n", ret);
			return ret;
		}
		ad9517_parse_firmware(st, (u8 *) fw->data, fw->size);
		release_firmware(fw);
	} else {
		ret = ad9517_parse_pdata(st, pdata);
		if (ret < 0) {
			dev_err(&spi->dev,
				"parse pdata failed with %i\n", ret);
			return ret;
		}
	}

	st->spi = spi;
	ref_clk = clk_get(&spi->dev, "refclk");
	clkin = clk_get(&spi->dev, "clkin");

	if (IS_ERR(ref_clk) && IS_ERR(clkin)) {
		ret = PTR_ERR(ref_clk);
		dev_err(&spi->dev, "failed getting clock (%d)\n", ret);
		goto out;
	}


	if (IS_ERR(ref_clk)) {
		ret = PTR_ERR(ref_clk);
		dev_warn(&spi->dev, "failed getting clock (%d)\n", ret);
	} else {
		st->refin_freq = clk_get_rate(ref_clk);
	}

	if (IS_ERR(clkin)) {
		ret = PTR_ERR(clkin);
		dev_warn(&spi->dev, "failed getting clock (%d)\n", ret);
		goto out;
	} else {
		st->clkin_freq = clk_get_rate(clkin);
	}

	ret = ad9517_setup(st);
	if (ret < 0)
		return ret;

	st->clk_data.clks = devm_kzalloc(&st->spi->dev,
					 sizeof(*st->clk_data.clks) *
					 NUM_OUTPUTS, GFP_KERNEL);
	if (!st->clk_data.clks) {
		dev_err(&st->spi->dev, "could not allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}
	st->clk_data.clk_num = NUM_OUTPUTS;

	for (out = 0; out < NUM_OUTPUTS; out++) {
		st->output[out].is_enabled =
			!(st->regs[output_pwrdwn_lut[out][PWRDWN_REG]] &
			output_pwrdwn_lut[out][PWRDWN_MASK]);
		clk = ad9517_clk_register(st, out);
		if (IS_ERR(clk))
			return PTR_ERR(clk);
	}

	of_clk_add_provider(st->spi->dev.of_node,
			    of_clk_src_onecell_get, &st->clk_data);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad9517_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9517_chan;
	indio_dev->num_channels = NUM_OUTPUTS;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto out;


	dev_info(&spi->dev, "probed\n");
	return 0;

out:
	spi_set_drvdata(spi, NULL);
	return ret;
}

static int ad9517_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);

	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad9517_id[] = {
	{"ad9517-0", 0x11},
	{"ad9517-1", 0x51},
	{"ad9517-2", 0x91},
	{"ad9517-3", 0x53},
	{"ad9517-4", 0xD3},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9517_id);

static struct spi_driver ad9517_driver = {
	.driver = {
		.name	= "ad9517",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9517_probe,
	.remove		= ad9517_remove,
	.id_table	= ad9517_id,
};
module_spi_driver(ad9517_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices AD9517");
MODULE_LICENSE("GPL v2");
