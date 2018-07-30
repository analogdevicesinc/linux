// SPDX-License-Identifier: GPL-2.0
/*
 * HMC7044 SPI High Performance, 3.2 GHz, 14-Output Jitter
 * Attenuator with JESD204B
 *
 * Copyright 2018 Analog Devices Inc.
 */
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/rational.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>

#include <linux/iio/iio.h>

#define HMC7044_WRITE		(0 << 15)
#define HMC7044_CNT(x)		(((x) - 1) << 13)
#define HMC7044_ADDR(x)		((x) & 0xFFF)

/* Global Control */
#define HMC7044_REG_SOFT_RESET		0x0000
#define HMC7044_SOFT_RESET		BIT(0)

#define HMC7044_REG_REQ_MODE_0		0x0001
#define HMC7044_RESEED_REQ		BIT(7)
#define HMC7044_HIGH_PERF_DISTRIB_PATH	BIT(6)
#define HMC7044_HIGH_PERF_PLL_VCO	BIT(5)
#define HMC7044_FORCE_HOLDOVER		BIT(4)
#define HMC7044_MUTE_OUT_DIV		BIT(3)
#define HMC7044_PULSE_GEN_REQ		BIT(2)
#define HMC7044_RESTART_DIV_FSM		BIT(1)
#define HMC7044_SLEEP_MODE		BIT(0)

#define HMC7044_REG_REQ_MODE_1		0x0002
#define HMC7044_PLL2_AUTOTUNE_TRIG	BIT(2)
#define HMC7044_SLIP_REQ		BIT(1)

#define HMC7044_REG_EN_CTRL_0		0x0003
#define HMC7044_RF_RESEEDER_EN		BIT(5)
#define HMC7044_VCO_SEL(x)		(((x) & 0x3) << 3)
#define HMC7044_VCO_EXT			0
#define HMC7044_VCO_HIGH		1
#define HMC7044_VCO_LOW			2
#define HMC7044_SYSREF_TIMER_EN		BIT(2)
#define HMC7044_PLL2_EN			BIT(1)
#define HMC7044_PLL1_EN			BIT(0)

#define HMC7044_REG_EN_CTRL_1		0x0004
#define HMC7044_SEVEN_PAIRS(x)		((x) & 0x7f)

/* PLL1 */
#define HMC7044_REG_CLKIN0_BUF_CTRL	0x000A
#define HMC7044_REG_CLKIN1_BUF_CTRL	0x000B
#define HMC7044_REG_CLKIN2_BUF_CTRL	0x000C
#define HMC7044_REG_CLKIN3_BUF_CTRL	0x000D
#define HMC7044_REG_OSCIN_BUF_CTRL	0x000E

#define HMC7044_HIGH_Z_EN		BIT(4)
#define HMC7044_LVPECL_EN		BIT(3)
#define HMC7044_AC_COUPLING_EN		BIT(2)
#define HMC7044_100_OHM_EN		BIT(1)
#define HMC7044_BUF_EN			BIT(0)

#define HMC7044_REG_CLKIN_PRESCALER(x)	(0x001C + (x))
#define HMC7044_REG_OSCIN_PRESCALER	0x0020

#define HMC7044_REG_PLL1_R_LSB		0x0021
#define HMC7044_R1_LSB(x)		((x) & 0xff)

#define HMC7044_REG_PLL1_R_MSB		0x0022
#define HMC7044_R1_MSB(x)		(((x) & 0xff00) >> 8)

#define HMC7044_REG_PLL1_N_LSB		0x0026
#define HMC7044_N1_LSB(x)		((x) & 0xff)

#define HMC7044_REG_PLL1_N_MSB		0x0027
#define HMC7044_N1_MSB(x)		(((x) & 0xff00) >> 8)

#define HMC7044_REG_PLL1_LOCK_DETECT	0x0028
#define HMC7044_LOCK_DETECT_SLIP	BIT(5)
#define HMC7044_LOCK_DETECT_TIMER(x)	((x) & 0x1f)

/* PLL2 */
#define HMC7044_REG_PLL2_FREQ_DOUBLER	0x0032
#define HMC7044_PLL2_FREQ_DOUBLER_DIS	BIT(0)

#define HMC7044_REG_PLL2_R_LSB		0x0033
#define HMC7044_R2_LSB(x)		((x) & 0xff)

#define HMC7044_REG_PLL2_R_MSB		0x0034
#define HMC7044_R2_MSB(x)		(((x) & 0xf00) >> 8)

#define HMC7044_REG_PLL2_N_LSB		0x0035
#define HMC7044_N2_LSB(x)		((x) & 0xff)

#define HMC7044_REG_PLL2_N_MSB		0x0036
#define HMC7044_N2_MSB(x)		(((x) & 0xff00) >> 8)

#define HMC7044_REG_OSCOUT_PATH		0x0039
#define HMC7044_REG_OSCOUT_DRIVER_0	0x003A
#define HMC7044_REG_OSCOUT_DRIVER_1	0x003B

/* GPIO/SDATA Control */
#define HMC7044_REG_GPI_CTRL(x)		(0x0046 + (x))
#define HMC7044_REG_GPI_SEL(x)		((x) & 0xf)

#define HMC7044_REG_GPO_CTRL(x)		(0x0050 + (x))
#define HMC7044_GPO_SEL(x)		(((x) & 0x3f) << 2)
#define HMC7044_GPO_MODE		BIT(1)
#define HMC7044_GPO_EN			BIT(0)

/* SYSREF/SYNC Control */
#define HMC7044_REG_PULSE_GEN		0x005A
#define HMC7044_PULSE_GEN_MODE(x)	((x) & 0x7)

#define HMC7044_REG_SYNC		0x005B
#define HMC7044_SYNC_RETIME		BIT(2)
#define HMC7044_SYNC_THROUGH_PLL2	BIT(1)
#define HMC7044_SYNC_POLARITY		BIT(0)

#define HMC7044_REG_SYSREF_TIMER_LSB	0x005C
#define HMC7044_SYSREF_TIMER_LSB(x)	((x) & 0xff)

#define HMC7044_REG_SYSREF_TIMER_MSB	0x005D
#define HMC7044_SYSREF_TIMER_MSB(x)	(((x) & 0xf00) >> 8)

/* Other Controls */
#define HMC7044_REG_CLK_OUT_DRV_LOW_PW	0x009F
#define HMC7044_REG_CLK_OUT_DRV_HIGH_PW	0x00A0
#define HMC7044_REG_PLL1_DELAY		0x00A5
#define HMC7044_REG_PLL1_HOLDOVER	0x00A8
#define HMC7044_REG_VTUNE_PRESET	0x00B0

/* Clock Distribution */
#define HMC7044_REG_CH_OUT_CRTL_0(ch)	(0x00C8 + 0xA * (ch))
#define HMC7044_HI_PERF_MODE		BIT(7)
#define HMC7044_SYNC_EN			BIT(6)
#define HMC7044_CH_EN			BIT(0)

#define HMC7044_REG_CH_OUT_CRTL_1(ch)	(0x00C9 + 0xA * (ch))
#define HMC7044_DIV_LSB(x)		((x) & 0xFF)

#define HMC7044_REG_CH_OUT_CRTL_2(ch)	(0x00CA + 0xA * (ch))
#define HMC7044_DIV_MSB(x)		(((x) >> 8) & 0xFF)

#define HMC7044_REG_CH_OUT_CRTL_3(ch)	(0x00CB + 0xA * (ch))
#define HMC7044_REG_CH_OUT_CRTL_4(ch)	(0x00CC + 0xA * (ch))
#define HMC7044_REG_CH_OUT_CRTL_5(ch)	(0x00CD + 0xA * (ch))
#define HMC7044_REG_CH_OUT_CRTL_6(ch)	(0x00CE + 0xA * (ch))
#define HMC7044_REG_CH_OUT_CRTL_7(ch)	(0x00CF + 0xA * (ch))

#define HMC7044_REG_CH_OUT_CRTL_8(ch)	(0x00D0 + 0xA * (ch))
#define HMC7044_DRIVER_MODE(x)		(((x) & 0x3) << 3)

#define HMC7044_NUM_CHAN	14

#define HMC7044_LOW_VCO_MIN	2150000
#define HMC7044_LOW_VCO_MAX	2880000
#define HMC7044_HIGH_VCO_MIN	2650000
#define HMC7044_HIGH_VCO_MAX	3200000

#define HMC7044_RECOMM_LCM_MIN	30000
#define HMC7044_RECOMM_LCM_MAX	70000
#define HMC7044_RECOMM_FPD1	10000

#define HMC7044_R1_MAX		65535
#define HMC7044_N1_MAX		65535

#define HMC7044_R2_MIN		1
#define HMC7044_R2_MAX		4095
#define HMC7044_N2_MIN		8
#define HMC7044_N2_MAX		65535

#define HMC7044_OUT_DIV_MIN	1
#define HMC7044_OUT_DIV_MAX	4094

struct hmc7044_output {
	unsigned int	address;
	struct clk_hw	hw;
	struct iio_dev	*indio_dev;
};

#define to_output(_hw) container_of(_hw, struct hmc7044_output, hw)

struct hmc7044_chan_spec {
	unsigned int		num;
	bool			disable;
	unsigned int		divider;
	unsigned int		driver_mode;
	const char		*extended_name;
};

struct hmc7044 {
	struct spi_device		*spi;
	u32				clkin_freq[4];
	u32				vcxo_freq;
	u32				pll2_freq;
	unsigned int			pll1_loop_bw;
	unsigned int			sysref_timer_div;
	unsigned int			pulse_gen_mode;
	unsigned int			in_buf_mode[5];
	unsigned int			gpi_ctrl[4];
	unsigned int			gpo_ctrl[4];
	const char			*clk_out_names[HMC7044_NUM_CHAN];
	unsigned int			num_channels;
	struct hmc7044_chan_spec	*channels;
	struct iio_chan_spec		iio_channels[HMC7044_NUM_CHAN];
	struct hmc7044_output		outputs[HMC7044_NUM_CHAN];
	struct clk			*clks[HMC7044_NUM_CHAN];
	struct clk_onecell_data		clk_data;
	struct mutex			lock;
};

static int hmc7044_write(struct iio_dev *indio_dev,
			 unsigned int reg,
			 unsigned int val)
{
	struct hmc7044 *hmc = iio_priv(indio_dev);
	unsigned char buf[3];
	u16 cmd;

	cmd = HMC7044_WRITE | HMC7044_CNT(1) | HMC7044_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;
	buf[2] = val;

	return spi_write(hmc->spi, buf, ARRAY_SIZE(buf));
}

static unsigned int hmc7044_calc_out_div(unsigned long parent_rate,
					 unsigned long rate)
{
	unsigned int div;

	div = DIV_ROUND_CLOSEST(parent_rate, rate);

	/* Supported odd divide ratios are 1, 3, and 5 */
	if ((div != 1) && (div != 3) && (div != 5) && (div % 2))
		div = DIV_ROUND_CLOSEST(parent_rate, rate * 2) * 2;

	div = clamp_t(unsigned int,
		      div,
		      HMC7044_OUT_DIV_MIN,
		      HMC7044_OUT_DIV_MAX);

	return div;
}

static int hmc7044_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct hmc7044 *hmc = iio_priv(indio_dev);
	struct hmc7044_chan_spec *ch;

	if (chan->address >= hmc->num_channels)
		return -EINVAL;

	ch = &hmc->channels[chan->address];

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		*val = hmc->pll2_freq / ch->divider;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
};

static int hmc7044_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct hmc7044 *hmc = iio_priv(indio_dev);
	struct hmc7044_chan_spec *ch;

	if (chan->address >= hmc->num_channels)
		return -EINVAL;

	ch = &hmc->channels[chan->address];

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		ch->divider = hmc7044_calc_out_div(hmc->pll2_freq, val);
		mutex_lock(&hmc->lock);
		hmc7044_write(indio_dev, HMC7044_REG_CH_OUT_CRTL_1(ch->num),
			      HMC7044_DIV_LSB(ch->divider));
		hmc7044_write(indio_dev, HMC7044_REG_CH_OUT_CRTL_2(ch->num),
			      HMC7044_DIV_MSB(ch->divider));
		mutex_unlock(&hmc->lock);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int hmc7044_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct hmc7044 *hmc = iio_priv(indio_dev);
	int ret;

	if (readval)
		return -EINVAL;

	mutex_lock(&hmc->lock);
	ret = hmc7044_write(indio_dev, reg, writeval);
	mutex_unlock(&hmc->lock);

	return ret;
}

static const struct iio_info hmc7044_iio_info = {
	.read_raw = &hmc7044_read_raw,
	.write_raw = &hmc7044_write_raw,
	.debugfs_reg_access = &hmc7044_reg_access,
	.driver_module = THIS_MODULE,
};

static long hmc7044_get_clk_attr(struct clk_hw *hw,
				 long mask)
{
	struct iio_dev *indio_dev = to_output(hw)->indio_dev;
	struct hmc7044 *hmc = iio_priv(indio_dev);
	struct iio_chan_spec *chan;
	unsigned int address;
	int val, ret;

	address = to_output(hw)->address;
	if (address >= hmc->num_channels)
		return -EINVAL;

	chan = &hmc->iio_channels[address];

	ret = hmc7044_read_raw(indio_dev, chan, &val, NULL, mask);

	if (ret == IIO_VAL_INT)
		return val;

	return ret;
}

static long hmc7044_set_clk_attr(struct clk_hw *hw,
				 long mask,
				 unsigned long val)
{
	struct iio_dev *indio_dev = to_output(hw)->indio_dev;
	struct hmc7044 *hmc = iio_priv(indio_dev);
	struct iio_chan_spec *chan;
	unsigned int address;

	address = to_output(hw)->address;
	if (address >= hmc->num_channels)
		return -EINVAL;

	chan = &hmc->iio_channels[address];

	return hmc7044_write_raw(indio_dev, chan, val, 0, mask);
}

static unsigned long hmc7044_clk_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	return hmc7044_get_clk_attr(hw, IIO_CHAN_INFO_FREQUENCY);
}

static long hmc7044_clk_round_rate(struct clk_hw *hw,
				   unsigned long rate,
				   unsigned long *parent_rate)
{
	struct hmc7044_output *out = to_output(hw);
	struct iio_dev *indio_dev = out->indio_dev;
	struct hmc7044 *hmc = iio_priv(indio_dev);
	unsigned int div;

	div = hmc7044_calc_out_div(hmc->pll2_freq, rate);

	return DIV_ROUND_CLOSEST(hmc->pll2_freq, div);
}

static int hmc7044_clk_set_rate(struct clk_hw *hw,
				unsigned long rate,
				unsigned long parent_rate)
{
	return hmc7044_set_clk_attr(hw, IIO_CHAN_INFO_FREQUENCY, rate);
}

static const struct clk_ops hmc7044_clk_ops = {
	.recalc_rate = hmc7044_clk_recalc_rate,
	.round_rate = hmc7044_clk_round_rate,
	.set_rate = hmc7044_clk_set_rate,
};

static int hmc7044_clk_register(struct iio_dev *indio_dev,
				unsigned int num,
				unsigned int address)
{
	struct hmc7044 *hmc = iio_priv(indio_dev);
	struct clk_init_data init;
	struct clk *clk;

	init.name = hmc->clk_out_names[num];
	init.ops = &hmc7044_clk_ops;
	init.flags = 0;
	init.num_parents = 0;

	hmc->outputs[num].hw.init = &init;
	hmc->outputs[num].indio_dev = indio_dev;
	hmc->outputs[num].address = address;

	clk = devm_clk_register(&hmc->spi->dev, &hmc->outputs[num].hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	hmc->clks[num] = clk;

	return 0;
}

static int hmc7044_setup(struct iio_dev *indio_dev)
{
	struct hmc7044 *hmc = iio_priv(indio_dev);
	struct hmc7044_chan_spec *chan;
	bool high_vco_en;
	bool pll2_freq_doubler_en;
	unsigned long vcxo_freq, pll2_freq;
	unsigned long clkin_freq[4];
	unsigned long lcm_freq;
	unsigned int in_prescaler[5];
	unsigned long pll1_lock_detect;
	unsigned long n1, r1;
	unsigned long pfd1_freq;
	unsigned long vco_limit;
	unsigned long n2[2], r2[2];
	unsigned int i;
	int ret;

	vcxo_freq = hmc->vcxo_freq / 1000;
	pll2_freq = hmc->pll2_freq / 1000;

	lcm_freq = vcxo_freq;
	for (i = 0; i < ARRAY_SIZE(clkin_freq); i++) {
		clkin_freq[i] = hmc->clkin_freq[i] / 1000;
		if (clkin_freq[i])
			lcm_freq = gcd(clkin_freq[i], lcm_freq);
	}

	while (lcm_freq > HMC7044_RECOMM_LCM_MAX)
		lcm_freq /= 2;

	for (i = 0; i < ARRAY_SIZE(clkin_freq); i++) {
		if (clkin_freq[i])
			in_prescaler[i] = clkin_freq[i] / lcm_freq;
		else
			in_prescaler[i] = 1;
	}
	in_prescaler[4] = vcxo_freq / lcm_freq;

	pll1_lock_detect = ilog2((lcm_freq * 4000) / hmc->pll1_loop_bw);

	/* fVCXO / N1 = fLCM / R1 */
	rational_best_approximation(vcxo_freq, lcm_freq,
				    HMC7044_N1_MAX, HMC7044_R1_MAX,
				    &n1, &r1);

	pfd1_freq = vcxo_freq / r1;
	while ((pfd1_freq > HMC7044_RECOMM_FPD1) &&
	       (n1 <= HMC7044_N1_MAX / 2) &&
	       (r1 <= HMC7044_R1_MAX / 2)) {
		pfd1_freq /= 2;
		n1 *= 2;
		r1 *= 2;
	}

	if (pll2_freq < HMC7044_LOW_VCO_MIN  ||
	    pll2_freq > HMC7044_HIGH_VCO_MAX)
		return -EINVAL;

	vco_limit = (HMC7044_LOW_VCO_MAX + HMC7044_HIGH_VCO_MIN) / 2;
	if (pll2_freq >= vco_limit)
		high_vco_en = true;
	else
		high_vco_en = false;

	/* fVCO / N2 = fVCXO * doubler / R2 */
	pll2_freq_doubler_en = true;
	rational_best_approximation(pll2_freq, vcxo_freq * 2,
				    HMC7044_N2_MAX, HMC7044_R2_MAX,
				    &n2[0], &r2[0]);

	if (pll2_freq != vcxo_freq * n2[0] / r2[0]) {
		rational_best_approximation(pll2_freq, vcxo_freq,
					    HMC7044_N2_MAX, HMC7044_R2_MAX,
					    &n2[1], &r2[1]);

		if (abs((int)pll2_freq - (int)(vcxo_freq * 2 * n2[0] / r2[0])) >
		    abs((int)pll2_freq - (int)(vcxo_freq * n2[1] / r2[1]))) {
			n2[0] = n2[1];
			r2[0] = r2[1];
			pll2_freq_doubler_en = false;
		}
	}

	while ((n2[0] < HMC7044_N2_MIN) && (r2[0] <= HMC7044_R2_MAX / 2)) {
		n2[0] *= 2;
		r2[0] *= 2;
	}
	if (n2[0] < HMC7044_N2_MIN)
		return -EINVAL;

	/* Resets all registers to default values */
	hmc7044_write(indio_dev, HMC7044_REG_SOFT_RESET, HMC7044_SOFT_RESET);
	mdelay(10);
	hmc7044_write(indio_dev, HMC7044_REG_SOFT_RESET, 0);
	mdelay(10);

	/* Load the configuration updates (provided by Analog Devices) */
	hmc7044_write(indio_dev, HMC7044_REG_CLK_OUT_DRV_LOW_PW, 0x4d);
	hmc7044_write(indio_dev, HMC7044_REG_CLK_OUT_DRV_HIGH_PW, 0xdf);
	hmc7044_write(indio_dev, HMC7044_REG_PLL1_DELAY, 0x06);
	hmc7044_write(indio_dev, HMC7044_REG_PLL1_HOLDOVER, 0x06);
	hmc7044_write(indio_dev, HMC7044_REG_VTUNE_PRESET, 0x04);

	/* Program PLL2 */

	/* Select the VCO range */
	hmc7044_write(indio_dev, HMC7044_REG_EN_CTRL_0,
		      HMC7044_RF_RESEEDER_EN |
		      HMC7044_VCO_SEL(high_vco_en ?
				      HMC7044_VCO_HIGH :
				      HMC7044_VCO_LOW) |
		      HMC7044_SYSREF_TIMER_EN | HMC7044_PLL2_EN |
		      HMC7044_PLL1_EN);

	/* Program the dividers */
	hmc7044_write(indio_dev, HMC7044_REG_PLL2_R_LSB,
		      HMC7044_R2_LSB(r2[0]));
	hmc7044_write(indio_dev, HMC7044_REG_PLL2_R_MSB,
		      HMC7044_R2_MSB(r2[0]));
	hmc7044_write(indio_dev, HMC7044_REG_PLL2_N_LSB,
		      HMC7044_N2_LSB(n2[0]));
	hmc7044_write(indio_dev, HMC7044_REG_PLL2_N_MSB,
		      HMC7044_N2_MSB(n2[0]));

	/* Program the reference doubler */
	hmc7044_write(indio_dev, HMC7044_REG_PLL2_FREQ_DOUBLER,
		      pll2_freq_doubler_en ? 0 : HMC7044_PLL2_FREQ_DOUBLER_DIS);

	/* Program PLL1 */

	/* Set the lock detect timer threshold */
	hmc7044_write(indio_dev, HMC7044_REG_PLL1_LOCK_DETECT,
		      HMC7044_LOCK_DETECT_TIMER(pll1_lock_detect));

	/* Set the LCM */
	for (i = 0; i < ARRAY_SIZE(clkin_freq); i++) {
		hmc7044_write(indio_dev, HMC7044_REG_CLKIN_PRESCALER(i),
			      in_prescaler[i]);
	}
	hmc7044_write(indio_dev, HMC7044_REG_OSCIN_PRESCALER,
		      in_prescaler[4]);

	/* Program the dividers */
	hmc7044_write(indio_dev, HMC7044_REG_PLL1_R_LSB,
		      HMC7044_R2_LSB(r1));
	hmc7044_write(indio_dev, HMC7044_REG_PLL1_R_MSB,
		      HMC7044_R2_MSB(r1));
	hmc7044_write(indio_dev, HMC7044_REG_PLL1_N_LSB,
		      HMC7044_N2_LSB(n1));
	hmc7044_write(indio_dev, HMC7044_REG_PLL1_N_MSB,
		      HMC7044_N2_MSB(n1));

	/* Program the SYSREF timer */

	/* Set the divide ratio */
	hmc7044_write(indio_dev, HMC7044_REG_SYSREF_TIMER_LSB,
		      HMC7044_SYSREF_TIMER_LSB(hmc->sysref_timer_div));
	hmc7044_write(indio_dev, HMC7044_REG_SYSREF_TIMER_MSB,
		      HMC7044_SYSREF_TIMER_MSB(hmc->sysref_timer_div));

	/* Set the pulse generator mode configuration */
	hmc7044_write(indio_dev, HMC7044_REG_PULSE_GEN,
		      HMC7044_PULSE_GEN_MODE(hmc->pulse_gen_mode));

	/* Enable the input buffers */
	hmc7044_write(indio_dev, HMC7044_REG_CLKIN0_BUF_CTRL,
		      hmc->in_buf_mode[0]);
	hmc7044_write(indio_dev, HMC7044_REG_CLKIN1_BUF_CTRL,
		      hmc->in_buf_mode[1]);
	hmc7044_write(indio_dev, HMC7044_REG_CLKIN2_BUF_CTRL,
		      hmc->in_buf_mode[2]);
	hmc7044_write(indio_dev, HMC7044_REG_CLKIN3_BUF_CTRL,
		      hmc->in_buf_mode[3]);
	hmc7044_write(indio_dev, HMC7044_REG_OSCIN_BUF_CTRL,
		      hmc->in_buf_mode[4]);

	/* Set GPIOs */
	for (i = 0; i < ARRAY_SIZE(hmc->gpi_ctrl); i++) {
		hmc7044_write(indio_dev, HMC7044_REG_GPI_CTRL(i),
			      hmc->gpi_ctrl[i]);
	}

	for (i = 0; i < ARRAY_SIZE(hmc->gpo_ctrl); i++) {
		hmc7044_write(indio_dev, HMC7044_REG_GPO_CTRL(i),
			      hmc->gpo_ctrl[i]);
	}

	mdelay(10);

	/* Program the output channels */
	for (i = 0; i < hmc->num_channels; i++) {
		chan = &hmc->channels[i];

		if (chan->num >= HMC7044_NUM_CHAN || chan->disable)
			continue;

		hmc7044_write(indio_dev, HMC7044_REG_CH_OUT_CRTL_0(chan->num),
			      HMC7044_HI_PERF_MODE | HMC7044_SYNC_EN |
			      HMC7044_CH_EN);
		hmc7044_write(indio_dev, HMC7044_REG_CH_OUT_CRTL_1(chan->num),
			      HMC7044_DIV_LSB(chan->divider));
		hmc7044_write(indio_dev, HMC7044_REG_CH_OUT_CRTL_2(chan->num),
			      HMC7044_DIV_MSB(chan->divider));
		hmc7044_write(indio_dev, HMC7044_REG_CH_OUT_CRTL_8(chan->num),
			      HMC7044_DRIVER_MODE(chan->driver_mode));

		hmc->iio_channels[i].type = IIO_ALTVOLTAGE;
		hmc->iio_channels[i].output = 1;
		hmc->iio_channels[i].indexed = 1;
		hmc->iio_channels[i].channel = chan->num;
		hmc->iio_channels[i].address = i;
		hmc->iio_channels[i].extend_name = chan->extended_name;
		hmc->iio_channels[i].info_mask_separate =
			BIT(IIO_CHAN_INFO_FREQUENCY);
	}
	mdelay(10);

	/* Do a restart to reset the system and initiate calibration */
	hmc7044_write(indio_dev, HMC7044_REG_REQ_MODE_0,
		      HMC7044_RESTART_DIV_FSM);
	mdelay(1);
	hmc7044_write(indio_dev, HMC7044_REG_REQ_MODE_0, 0);
	mdelay(1);

	for (i = 0; i < hmc->num_channels; i++) {
		chan = &hmc->channels[i];

		if (chan->num >= HMC7044_NUM_CHAN || chan->disable)
			continue;

		ret = hmc7044_clk_register(indio_dev, chan->num, i);
		if (ret)
			return ret;
	}

	hmc->clk_data.clks = hmc->clks;
	hmc->clk_data.clk_num = HMC7044_NUM_CHAN;

	return of_clk_add_provider(hmc->spi->dev.of_node,
				   of_clk_src_onecell_get,
				   &hmc->clk_data);
}

static int hmc7044_parse_dt(struct device *dev,
			    struct hmc7044 *hmc)
{
	struct device_node *np = dev->of_node, *chan_np;
	unsigned int cnt = 0;
	int ret;

	ret = of_property_read_u32_array(np, "adi,pll1-clkin-frequencies",
			hmc->clkin_freq, ARRAY_SIZE(hmc->clkin_freq));
	if (ret)
		return ret;

	hmc->pll1_loop_bw = 200;
	of_property_read_u32(np, "adi,pll1-loop-bandwidth-hz",
			     &hmc->pll1_loop_bw);

	ret = of_property_read_u32(np, "adi,vcxo-frequency",
				   &hmc->vcxo_freq);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "adi,pll2-output-frequency",
				   &hmc->pll2_freq);
	if (ret)
		return ret;

	hmc->sysref_timer_div = 256;
	of_property_read_u32(np, "adi,sysref-timer-divider",
			     &hmc->sysref_timer_div);

	hmc->pulse_gen_mode = 0;
	of_property_read_u32(np, "adi,pulse-generator-mode",
			     &hmc->pulse_gen_mode);

	hmc->in_buf_mode[0] = 0;
	of_property_read_u32(np, "adi,clkin0-buffer-mode",
			     &hmc->in_buf_mode[0]);
	hmc->in_buf_mode[1] = 0;
	of_property_read_u32(np, "adi,clkin1-buffer-mode",
			     &hmc->in_buf_mode[1]);
	hmc->in_buf_mode[2] = 0;
	of_property_read_u32(np, "adi,clkin2-buffer-mode",
			     &hmc->in_buf_mode[2]);
	hmc->in_buf_mode[3] = 0;
	of_property_read_u32(np, "adi,clkin3-buffer-mode",
			     &hmc->in_buf_mode[3]);
	hmc->in_buf_mode[4] = 0;
	of_property_read_u32(np, "adi,oscin-buffer-mode",
			     &hmc->in_buf_mode[4]);

	ret = of_property_read_u32_array(np, "adi,gpi-controls",
			hmc->gpi_ctrl, ARRAY_SIZE(hmc->gpi_ctrl));
	if (ret)
		return ret;

	ret = of_property_read_u32_array(np, "adi,gpo-controls",
			hmc->gpo_ctrl, ARRAY_SIZE(hmc->gpo_ctrl));
	if (ret)
		return ret;

	ret = of_property_read_string_array(np, "clock-output-names",
			hmc->clk_out_names, ARRAY_SIZE(hmc->clk_out_names));
	if (ret < 0)
		return ret;

	for_each_child_of_node(np, chan_np)
		hmc->num_channels++;
	if (hmc->num_channels > HMC7044_NUM_CHAN)
		return -EINVAL;

	hmc->channels = devm_kzalloc(dev,
		sizeof(struct hmc7044_chan_spec) * hmc->num_channels,
				     GFP_KERNEL);
	if (!hmc->channels)
		return -ENOMEM;

	for_each_child_of_node(np, chan_np) {
		hmc->channels[cnt].num = cnt;
		of_property_read_u32(chan_np, "reg",
				     &hmc->channels[cnt].num);
		hmc->channels[cnt].disable =
			of_property_read_bool(chan_np, "adi,disable");
		hmc->channels[cnt].divider = 4;
		of_property_read_u32(chan_np, "adi,divider",
				     &hmc->channels[cnt].divider);
		hmc->channels[cnt].driver_mode = 0;
		of_property_read_u32(chan_np, "adi,driver-mode",
				     &hmc->channels[cnt].driver_mode);
		of_property_read_string(chan_np, "adi,extended-name",
					&hmc->channels[cnt].extended_name);
		cnt++;
	}

	return 0;
}

static int hmc7044_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct hmc7044 *hmc;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*hmc));
	if (!indio_dev)
		return -ENOMEM;

	hmc = iio_priv(indio_dev);

	mutex_init(&hmc->lock);

	spi_set_drvdata(spi, indio_dev);

	hmc->spi = spi;

	ret = hmc7044_parse_dt(&spi->dev, hmc);
	if (ret)
		return ret;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &hmc7044_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = hmc->iio_channels;
	indio_dev->num_channels = hmc->num_channels;

	ret = hmc7044_setup(indio_dev);
	if (ret)
		return ret;

	return iio_device_register(indio_dev);
}

static int hmc7044_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	of_clk_del_provider(spi->dev.of_node);

	return 0;
}

static const struct spi_device_id hmc7044_id[] = {
	{"hmc7044", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, hmc7044_id);

static struct spi_driver hmc7044_driver = {
	.driver = {
		.name = "hmc7044",
	},
	.probe = hmc7044_probe,
	.remove = hmc7044_remove,
	.id_table = hmc7044_id,
};
module_spi_driver(hmc7044_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices HMC7044 driver");
MODULE_LICENSE("GPL v2");
