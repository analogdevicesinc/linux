// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for LTC6952 ultralow jitter, JESD204B/C clock generation IC.
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/rational.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>

/* Register address macro */
#define LTC6952_REG(x)		(x)

/* LTC6952_REG0 */
#define LTC6952_UNLOCK_MSK	BIT(6)
#define LTC6952_LOCK_MSK	BIT(4)
#define LTC6952_VCOOK_MSK	BIT(2)
#define LTC6952_REFOK_MSK	BIT(0)

/* LTC6952_REG1 */
#define LTC6952_INVSTAT_MSK	BIT(7)
#define LTC6952_INVSTAT(x)	FIELD_PREP(LTC6952_INVSTAT_MSK, x)
#define LTC6952_STAT_OUT_MSK	GENMASK(6, 0)
#define LTC6952_STAT_OUT(x)	FIELD_PREP(LTC6952_STAT_OUT_MSK, x)

/* LTC6952_REG2 */
#define LTC6952_PDALL_MSK	BIT(7)
#define LTC6952_PDALL(x)	FIELD_PREP(LTC6952_PDALL_MSK, x)
#define LTC6952_PDPLL_MSK	BIT(6)
#define LTC6952_PDPLL(x)	FIELD_PREP(LTC6952_PDPLL_MSK, x)
#define LTC6952_PDVCOPK_MSK	BIT(5)
#define LTC6952_PDVCOPK(x)	FIELD_PREP(LTC6952_PDVCOPK_MSK, x)
#define LTC6952_PDREFPK_MSK	BIT(4)
#define LTC6952_PDREFPK(x)	FIELD_PREP(LTC6952_PDREFPK_MSK, x)
#define LTC6952_BST_MSK		BIT(3)
#define LTC6952_BST(x)		FIELD_PREP(LTC6952_BST_MSK, x)
#define LTC6952_FILTR_MSK	BIT(2)
#define LTC6952_FILTR(x)	FIELD_PREP(LTC6952_FILTR_MSK, x)
#define LTC6952_FILTV_MSK	BIT(1)
#define LTC6952_FILTV(x)	FIELD_PREP(LTC6952_FILTV_MSK, x)
#define LTC6952_POR_MSK		BIT(0)
#define LTC6952_POR(x)		FIELD_PREP(LTC6952_POR_MSK, x)

/* LTC6952_REG3 */
#define LTC6952_PD3_MSK		GENMASK(7, 6)
#define LTC6952_PD3(x)		FIELD_PREP(LTC6952_PD3_MSK, x)
#define LTC6952_PD2_MSK		GENMASK(5, 4)
#define LTC6952_PD2(x)		FIELD_PREP(LTC6952_PD2_MSK, x)
#define LTC6952_PD1_MSK		GENMASK(3, 2)
#define LTC6952_PD1(x)		FIELD_PREP(LTC6952_PD1_MSK, x)
#define LTC6952_PD0_MSK		GENMASK(1, 0)
#define LTC6952_PD0(x)		FIELD_PREP(LTC6952_PD0_MSK, x)

/* LTC6952_REG4 */
#define LTC6952_PD7_MSK		GENMASK(7, 6)
#define LTC6952_PD7(x)		FIELD_PREP(LTC6952_PD7_MSK, x)
#define LTC6952_PD6_MSK		GENMASK(5, 4)
#define LTC6952_PD6(x)		FIELD_PREP(LTC6952_PD6_MSK, x)
#define LTC6952_PD5_MSK		GENMASK(3, 2)
#define LTC6952_PD5(x)		FIELD_PREP(LTC6952_PD5_MSK, x)
#define LTC6952_PD4_MSK		GENMASK(1, 0)
#define LTC6952_PD4(x)		FIELD_PREP(LTC6952_PD4_MSK, x)

/* LTC6952_REG5 */
#define LTC6952_TEMPO_MSK	BIT(7)
#define LTC6952_TEMPO(x)	FIELD_PREP(LTC6952_TEMPO_MSK, x)
#define LTC6952_PD10_MSK	GENMASK(5, 4)
#define LTC6952_PD10(x)		FIELD_PREP(LTC6952_PD10_MSK, x)
#define LTC6952_PD9_MSK		GENMASK(3, 2)
#define LTC6952_PD9(x)		FIELD_PREP(LTC6952_PD9_MSK, x)
#define LTC6952_PD8_MSK		GENMASK(1, 0)
#define LTC6952_PD8(x)		FIELD_PREP(LTC6952_PD8_MSK, x)

#define LTC6952_PD_MSK(ch)	GENMASK(((ch) & 0x03) * 2 + 1, ((ch) & 0x03) * 2)
#define LTC6952_PD(ch, x)	((x) << ((ch) & 0x03))

/* LTC6952_REG6 */
#define LTC6952_RAO_MSK		BIT(7)
#define LTC6952_RAO(x)		FIELD_PREP(LTC6952_RAO_MSK, x)
#define LTC6952_PARSYNC_MSK	BIT(6)
#define LTC6952_PARSYNC(x)	FIELD_PREP(LTC6952_PARSYNC_MSK, x)
#define LTC6952_LKWIN_MSK	BIT(4)
#define LTC6952_LKWIN(x)	FIELD_PREP(LTC6952_LKWIN_MSK, x)
#define LTC6952_LKCT_MSK	GENMASK(3, 2)
#define LTC6952_LKCT(x)		FIELD_PREP(LTC6952_LKCT_MSK, x)
#define LTC6952_RD_HIGH_MSK	GENMASK(1, 0)
#define LTC6952_RD_HIGH(x)	FIELD_PREP(LTC6952_RD_HIGH_MSK, x)

/* LTC6952_REG10 */
#define LTC6952_CPRST_MSK	BIT(7)
#define LTC6952_CPRST(x)	FIELD_PREP(LTC6952_CPRST_MSK, x)
#define LTC6952_CPUP_MSK	BIT(6)
#define LTC6952_CPUP(x)		FIELD_PREP(LTC6952_CPUP_MSK, x)
#define LTC6952_CPDN_MSK	BIT(5)
#define LTC6952_CPDN(x)		FIELD_PREP(LTC6952_CPDN_MSK, x)
#define LTC6952_CP_MSK		GENMASK(4, 0)
#define LTC6952_CP(x)		FIELD_PREP(LTC6952_CP_MSK, x)

/* LTC6952_REG11 */
#define LTC6952_CPMID_MSK	BIT(7)
#define LTC6952_CPMID(x)	FIELD_PREP(LTC6952_CPMID_MSK, x)
#define LTC6952_CPWIDE_MSK	BIT(6)
#define LTC6952_CPWIDE(x)	FIELD_PREP(LTC6952_CPWIDE_MSK, x)
#define LTC6952_CPINV_MSK	BIT(5)
#define LTC6952_CPINV(x)	FIELD_PREP(LTC6952_CPINV_MSK, x)
#define LTC6952_EZMD_MSK	BIT(4)
#define LTC6952_EZMD(x)		FIELD_PREP(LTC6952_EZMD_MSK, x)
#define LTC6952_SRQMD_MSK	BIT(3)
#define LTC6952_SRQMD(x)	FIELD_PREP(LTC6952_SRQMD_MSK, x)
#define LTC6952_SYSCT_MSK	GENMASK(2, 1)
#define LTC6952_SYSCT(x)	FIELD_PREP(LTC6952_SYSCT_MSK, x)
#define LTC6952_SSRQ_MSK	BIT(0)
#define LTC6952_SSRQ(x)		FIELD_PREP(LTC6952_SSRQ_MSK, x)

/* LTC6952_REG12,16,20,24,28,32,36,40,44,48,52 */
#define LTC6952_MP_MSK		GENMASK(7, 3)
#define LTC6952_MP(x)		FIELD_PREP(LTC6952_MP_MSK, x)
#define LTC6952_MD_MSK		GENMASK(2, 0)
#define LTC6952_MD(x)		FIELD_PREP(LTC6952_MD_MSK, x)

/* LTC6952_REG13,17,21,25,29,33,37,41,45,49,53 */
#define LTC6952_SRQEN_MSK	BIT(7)
#define LTC6952_SRQEN(x)	FIELD_PREP(LTC6952_SRQEN_MSK, x)
#define LTC6952_MODE_MSK	GENMASK(6, 5)
#define LTC6952_MODE(x)		FIELD_PREP(LTC6952_MODE_MSK, x)
#define LTC6952_OINV_MSK	BIT(4)
#define LTC6952_OINV(x)		FIELD_PREP(LTC6952_OINV_MSK, x)
#define LTC6952_DDEL_HIGH_MSK	GENMASK(3, 0)
#define LTC6952_DDEL_HIGH(x)	FIELD_PREP(LTC6952_DDEL_HIGH_MSK, x)

/* LTC6952_REG15,19,23,27,31,35,39,43,47,51,55 */
#define LTC6952_ADEL_MSK	GENMASK(5, 0)
#define LTC6952_ADEL(x)		FIELD_PREP(LTC6952_ADEL_MSK, x)

/* LTC6952_REG56 */
#define LTC6952_REV_MSK		GENMASK(7, 4)
#define LTC6952_PART_MSK	GENMASK(3, 0)

#define LTC6952_CMD_READ	0x1
#define LTC6952_CMD_WRITE	0x0
#define LTC6952_CMD_ADDR(x)	((x) << 1)

#define LTC6952_NUM_CLKS	11

#define LTC6952_N_MAX		65535
#define LTC6952_R_MAX		1023

#define LTC6952_PFD_FREQ_MAX	167000

#define LTC6952_OUT_DIV_MIN	1
#define LTC6952_OUT_DIV_MAX	1048576

#define LTC6952_CP_CURRENT_MIN	423
#define LTC6952_CP_CURRENT_MAX	11200

#define LTC6952_CH_OFFSET(ch)	(0x04 * (ch))

struct ltc6952_clk_hw {
	unsigned int	address;
	struct clk_hw	hw;
	struct ltc6952_driver_data *drvdata;
};

struct ltc6952_clk_state {
	unsigned int		num;
	unsigned int		out_divider;
	unsigned int		mp;
	unsigned int		md;
	unsigned int		digital_delay;
	unsigned int		analog_delay;
	unsigned int		sysref_mode;
	unsigned int		power_down_mode;
	const char		*extended_name;
};

struct ltc6952_driver_data {
	struct spi_device		*spi;
	struct clk			*ref_clk;
	struct clk			*vco_clk;
	u32				ref_freq;
	u32				vco_freq;
	struct clk			*clks[LTC6952_NUM_CLKS];
	struct clk_onecell_data		clk_data;
	const char			*clk_out_names[LTC6952_NUM_CLKS];
	bool				follower;
	u32				cp_current;
	u8				num_clks;
	struct ltc6952_clk_state	*clk_state;
	struct ltc6952_clk_hw		clks_hw[LTC6952_NUM_CLKS];
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		__be32 d32;
		u8 d8[2];
	} data ____cacheline_aligned;
};

#define to_ltc6952_clk(_hw) container_of(_hw, struct ltc6952_clk_hw, hw)

static int ltc6952_write(struct ltc6952_driver_data *drvdata,
			 unsigned int reg,
			 u8 val)
{
	drvdata->data.d8[0] =	LTC6952_CMD_WRITE | LTC6952_CMD_ADDR(reg);
	drvdata->data.d8[1] = val;

	return spi_write(drvdata->spi, drvdata->data.d8, ARRAY_SIZE(drvdata->data.d8));
}

static int ltc6952_read(struct ltc6952_driver_data *drvdata,
			unsigned int reg,
			u8 *val)
{
	u8 cmd;

	cmd = LTC6952_CMD_READ | LTC6952_CMD_ADDR(reg);

	return spi_write_then_read(drvdata->spi, &cmd, 1, val, 1);
}

static int ltc6952_update_bits(struct ltc6952_driver_data *drvdata,
				 unsigned int addr,
				 unsigned long mask,
				 unsigned int val)
{
	u8 readval;
	int ret;

	ret = ltc6952_read(drvdata, addr, &readval);
	if (ret < 0)
		return ret;

	readval &= ~mask;
	readval |= val;

	return ltc6952_write(drvdata, addr, readval);
}

static unsigned int ltc6952_calc_out_div(unsigned long parent_rate,
					 unsigned long rate)
{
	unsigned int div;

	div = DIV_ROUND_CLOSEST(parent_rate, rate);

	div = clamp_t(unsigned int,
		      div,
		      LTC6952_OUT_DIV_MIN,
		      LTC6952_OUT_DIV_MAX);

	return div;
}

static int ltc6952_calculate_divider(struct ltc6952_clk_state *clk_st)
{
	int mp, md;
	unsigned int out_divider;

	/* M(x) = (MP(x) + 1)2^MD(x) */
	clk_st->md = 0;
	for (mp = 0; mp < 32; mp++) {
		if (clk_st->out_divider == (mp + 1)) {
			clk_st->mp = mp;
			clk_st->md = 0;
			return 0;
		}

		/* MD works only if MP is greater than 15 */
		if (mp <= 15)
			continue;
		for (md = 0; md < 7; md++) {
			out_divider = (mp + 1) << md;
			if (clk_st->out_divider == out_divider) {
				clk_st->mp = mp;
				clk_st->md = md;
				return 0;
			}
			if (clk_st->out_divider < out_divider)
				break;
		}
	}

	return -EINVAL;
}

static unsigned long ltc6952_clk_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	unsigned int address = clk_hw->address;

	return drvdata->vco_freq / drvdata->clk_state[address].out_divider;
}

static long ltc6952_clk_round_rate(struct clk_hw *hw,
				   unsigned long rate,
				   unsigned long *parent_rate)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	unsigned int div;

	div = ltc6952_calc_out_div(drvdata->vco_freq, rate);

	return DIV_ROUND_CLOSEST(drvdata->vco_freq, div);
}

static int ltc6952_clk_set_rate(struct clk_hw *hw,
				unsigned long rate,
				unsigned long parent_rate)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	struct ltc6952_clk_state *clk_st = &drvdata->clk_state[clk_hw->address];
	int ret;

	clk_st->out_divider = ltc6952_calc_out_div(drvdata->vco_freq, rate);
	ret = ltc6952_calculate_divider(clk_st);
	if (ret < 0)
		return ret;

	ret = ltc6952_write(drvdata, LTC6952_REG(0x0C) +
			    LTC6952_CH_OFFSET(clk_st->num),
			    LTC6952_MP(clk_st->mp) | LTC6952_MD(clk_st->md));

	return ret;
}

static int ltc6952_clk_get_phase(struct clk_hw *hw)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	struct ltc6952_clk_state *clk_st = &drvdata->clk_state[clk_hw->address];
	unsigned int val;
	u8 tmp1, tmp2;
	int ret;

	ret = ltc6952_read(drvdata, LTC6952_REG(0x0D) +
			   LTC6952_CH_OFFSET(clk_st->num),
			   &tmp1);
	if (ret < 0)
		return ret;

	ret = ltc6952_read(drvdata, LTC6952_REG(0x0E) +
			   LTC6952_CH_OFFSET(clk_st->num),
			   &tmp2);
	if (ret < 0)
		return ret;

	val = ((tmp1 & 0x0F) << 8) + (tmp2 & 0xFF);
	val = DIV_ROUND_CLOSEST(val * 3141592, clk_st->out_divider);
	val = val / 1000000;

	return val;
}

static int ltc6952_clk_set_phase(struct clk_hw *hw, int degrees)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	struct ltc6952_clk_state *clk_st = &drvdata->clk_state[clk_hw->address];
	unsigned int code, tmp;
	int ret;

	code = degrees * 1000000;
	tmp = DIV_ROUND_CLOSEST(code * clk_st->out_divider, 3141592);
	tmp = clamp_t(unsigned int, tmp, 0, 4095);

	ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x0D) +
				 LTC6952_CH_OFFSET(clk_st->num),
				 LTC6952_DDEL_HIGH_MSK,
				 LTC6952_DDEL_HIGH((tmp & 0xF00) >> 8));
	if (ret < 0)
		return ret;

	ret = ltc6952_write(drvdata, LTC6952_REG(0x0E) +
			    LTC6952_CH_OFFSET(clk_st->num), tmp);
	if (ret < 0)
		return ret;

	/* Phase Syncronization */
	ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x0B),
				 LTC6952_SSRQ_MSK, LTC6952_SSRQ(1));
	if (ret < 0)
		return ret;

	usleep_range(1000, 1001); /* sleep > 1000us */
	return ltc6952_update_bits(drvdata, LTC6952_REG(0x0B),
				 LTC6952_SSRQ_MSK, LTC6952_SSRQ(0));
}

static int ltc6952_clk_prepare(struct clk_hw *hw)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	struct ltc6952_clk_state *clk_st = &drvdata->clk_state[clk_hw->address];

	/* Enable channel */
	return ltc6952_update_bits(drvdata, LTC6952_REG(0x03) + (clk_st->num >> 2),
				   LTC6952_PD_MSK(clk_st->num),
				   LTC6952_PD(clk_hw->address, 0));
}

static void ltc6952_clk_unprepare(struct clk_hw *hw)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	struct ltc6952_clk_state *clk_st = &drvdata->clk_state[clk_hw->address];

	/* Enable channel */
	ltc6952_update_bits(drvdata, LTC6952_REG(0x03) + (clk_st->num >> 2),
			    LTC6952_PD_MSK(clk_st->num),
			    LTC6952_PD(clk_hw->address, 1));
}

static const struct clk_ops ltc6952_clk_ops = {
	.prepare = ltc6952_clk_prepare,
	.unprepare = ltc6952_clk_unprepare,
	.recalc_rate = ltc6952_clk_recalc_rate,
	.round_rate = ltc6952_clk_round_rate,
	.set_rate = ltc6952_clk_set_rate,
	.set_phase = ltc6952_clk_set_phase,
	.get_phase = ltc6952_clk_get_phase,
};

static int ltc6952_parse_dt(struct device *dev,
			    struct ltc6952_driver_data *drvdata)
{
	struct device_node *np = dev->of_node, *chan_np;
	unsigned int cnt = 0;
	int ret;

	drvdata->follower = of_property_read_bool(np, "adi,follower-mode-enable");

	ret = of_property_read_u32(np, "adi,charge-pump-microamp",
				   &drvdata->cp_current);
	if (ret < 0)
		drvdata->cp_current = LTC6952_CP_CURRENT_MAX;
	else
		drvdata->cp_current = clamp_t(u32, drvdata->cp_current,
			LTC6952_CP_CURRENT_MIN, LTC6952_CP_CURRENT_MAX);

	ret = of_property_read_string_array(np, "clock-output-names",
			drvdata->clk_out_names, ARRAY_SIZE(drvdata->clk_out_names));
	if (ret < 0)
		return ret;

	drvdata->num_clks = of_get_available_child_count(np);
	if (drvdata->num_clks > LTC6952_NUM_CLKS)
		return -EINVAL;

	drvdata->clk_state = devm_kzalloc(dev,
		sizeof(struct ltc6952_clk_state) * drvdata->num_clks,
				     GFP_KERNEL);
	if (!drvdata->clk_state)
		return -ENOMEM;

	for_each_child_of_node(np, chan_np) {
		drvdata->clk_state[cnt].num = cnt;
		of_property_read_u32(chan_np, "reg",
				     &drvdata->clk_state[cnt].num);

		if (of_property_read_u32(chan_np, "adi,divider",
					 &drvdata->clk_state[cnt].out_divider))
			drvdata->clk_state[cnt].out_divider = 4;

		of_property_read_u32(chan_np, "adi,digital-delay",
				     &drvdata->clk_state[cnt].digital_delay);

		of_property_read_u32(chan_np, "adi,analog-delay",
				     &drvdata->clk_state[cnt].analog_delay);

		of_property_read_string(chan_np, "adi,extended-name",
					&drvdata->clk_state[cnt].extended_name);

		cnt++;
	}

	return 0;
}

static int ltc6952_clk_register(struct ltc6952_driver_data *drvdata,
				unsigned int num,
				unsigned int address)
{
	struct clk_init_data init;
	struct clk *clk;

	init.name = drvdata->clk_out_names[num];
	init.ops = &ltc6952_clk_ops;
	init.flags = 0;
	init.num_parents = 0;

	drvdata->clks_hw[num].hw.init = &init;
	drvdata->clks_hw[num].address = address;

	clk = devm_clk_register(&drvdata->spi->dev, &drvdata->clks_hw[num].hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	drvdata->clks[num] = clk;

	return 0;
}

static int ltc6952_setup_clks(struct ltc6952_driver_data *drvdata)
{
	struct ltc6952_clk_state *clk_st;
	int ret, i;

	/* Program the output channels */
	for (i = 0; i < drvdata->num_clks; i++) {
		clk_st = &drvdata->clk_state[i];

		if (clk_st->num >= LTC6952_NUM_CLKS)
			continue;

		ret = ltc6952_calculate_divider(clk_st);
		if (ret < 0)
			return ret;

		ret = ltc6952_write(drvdata, LTC6952_REG(0x0c) +
				     LTC6952_CH_OFFSET(clk_st->num),
				     LTC6952_MP(clk_st->mp) | LTC6952_MD(clk_st->md));
		if (ret < 0)
			return ret;

		/* Enable sync or sysref */
		ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x0D) +
					  LTC6952_CH_OFFSET(clk_st->num),
					  LTC6952_SRQEN_MSK,
					  LTC6952_SRQEN(1));
		if (ret < 0)
			return ret;

		/* Set channel delay */
		ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x0D) +
					  LTC6952_CH_OFFSET(clk_st->num),
					  LTC6952_DDEL_HIGH_MSK,
					  LTC6952_DDEL_HIGH((clk_st->digital_delay &
							     0xF00) >> 8));
		if (ret < 0)
			return ret;

		ret = ltc6952_write(drvdata, LTC6952_REG(0x0E) +
				     LTC6952_CH_OFFSET(clk_st->num),
				     clk_st->digital_delay);
		if (ret < 0)
			return ret;

		ret = ltc6952_write(drvdata, LTC6952_REG(0x0F) +
				     LTC6952_CH_OFFSET(clk_st->num),
				     LTC6952_ADEL(clk_st->analog_delay));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ltc6952_config_deviders(struct ltc6952_driver_data *drvdata)
{
	unsigned long vco_freq, ref_freq;
	unsigned long pfd_freq;
	unsigned long n, r;
	int ret;

	vco_freq = drvdata->vco_freq / 1000;
	ref_freq = drvdata->ref_freq / 1000;

	/* fVCO / N = fREF / R */
	rational_best_approximation(vco_freq, ref_freq,
				    LTC6952_N_MAX, LTC6952_R_MAX,
				    &n, &r);

	pfd_freq = vco_freq / n;
	while ((pfd_freq > LTC6952_PFD_FREQ_MAX) &&
	       (n <= LTC6952_N_MAX / 2) &&
	       (r <= LTC6952_R_MAX / 2)) {
		pfd_freq /= 2;
		n *= 2;
		r *= 2;
	}

	/* Program the dividers */
	ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x06),
				  LTC6952_RD_HIGH_MSK,
				  LTC6952_RD_HIGH((r & 0x300) >> 8));
	if (ret < 0)
		return ret;

	ret = ltc6952_write(drvdata, LTC6952_REG(0x07), r);
	if (ret < 0)
		return ret;

	ret = ltc6952_write(drvdata, LTC6952_REG(0x08), n >> 8);
	if (ret < 0)
		return ret;

	return ltc6952_write(drvdata, LTC6952_REG(0x09), n);
}

static int ltc6952_setup(struct ltc6952_driver_data *drvdata)
{
	struct ltc6952_clk_state *clk_st;
	unsigned int i;
	u32 cp_current;
	int ret;

	/* Resets all registers to default values */
	ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x02),
				 LTC6952_POR_MSK, LTC6952_POR(1));
	if (ret < 0)
		return ret;

	if (drvdata->follower) {
		ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x02),
					LTC6952_PDPLL_MSK, LTC6952_PDPLL(1));
		if (ret < 0)
			return ret;
		goto follower;
	}

	ret = ltc6952_config_deviders(drvdata);
	if (ret < 0)
		return ret;

	/* PLL lock cycle count  */
	ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x06), LTC6952_LKCT_MSK,
				 LTC6952_LKCT(0x03));
	if (ret < 0)
		return ret;

	/* Disable CP Hi-Z and set the CP current */
	cp_current = drvdata->cp_current / LTC6952_CP_CURRENT_MIN;
	ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x0A),
				 LTC6952_CPRST_MSK | LTC6952_CP_MSK,
				 LTC6952_CPRST(0x0) | LTC6952_CP(cp_current));
	if (ret < 0)
		return ret;

follower:
	ret = ltc6952_setup_clks(drvdata);
	if (ret <0)
		return ret;

	/* Phase Syncronization */
	ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x0B),
				  LTC6952_SSRQ_MSK, LTC6952_SSRQ(1));
	if (ret < 0)
		return ret;

	usleep_range(1000, 1001); /* sleep > 1000us */
	ret = ltc6952_update_bits(drvdata, LTC6952_REG(0x0B),
				  LTC6952_SSRQ_MSK, LTC6952_SSRQ(0));
	if (ret < 0)
		return ret;

	/* Configure clocks */
	for (i = 0; i < drvdata->num_clks; i++) {
		clk_st = &drvdata->clk_state[i];

		if (clk_st->num >= LTC6952_NUM_CLKS)
			continue;

		ret = ltc6952_clk_register(drvdata, clk_st->num, i);
		if (ret)
			return ret;
	}

	drvdata->clk_data.clks = drvdata->clks;
	drvdata->clk_data.clk_num = drvdata->num_clks;

	return of_clk_add_provider(drvdata->spi->dev.of_node,
				   of_clk_src_onecell_get,
				   &drvdata->clk_data);
}

static int ltc6952_probe(struct spi_device *spi)
{
	struct ltc6952_driver_data *drvdata;
	int ret;

	drvdata = devm_kzalloc(&spi->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	spi_set_drvdata(spi, drvdata);

	drvdata->spi = spi;

	drvdata->ref_clk = devm_clk_get(&spi->dev, "ref-clk");
	if (IS_ERR(drvdata->ref_clk))
		return PTR_ERR(drvdata->ref_clk);

	ret = clk_prepare_enable(drvdata->ref_clk);
	if (ret < 0)
		return ret;

	drvdata->ref_freq = clk_get_rate(drvdata->ref_clk);

	drvdata->vco_clk = devm_clk_get(&spi->dev, "vco-clk");
	if (IS_ERR(drvdata->vco_clk))
		return PTR_ERR(drvdata->vco_clk);

	ret = clk_prepare_enable(drvdata->vco_clk);
	if (ret < 0)
		goto error_disable_clk;

	drvdata->vco_freq = clk_get_rate(drvdata->vco_clk);

	ret = ltc6952_parse_dt(&spi->dev, drvdata);
	if (ret < 0)
		goto error_disable_clk;

	ret = ltc6952_setup(drvdata);
	if (ret < 0)
		goto error_disable_clk;

	return 0;

error_disable_clk:
	clk_disable_unprepare(drvdata->ref_clk);

	return ret;
}

static int ltc6952_remove(struct spi_device *spi)
{
	of_clk_del_provider(spi->dev.of_node);

	return 0;
}

static const struct of_device_id ltc6952_of_match[] = {
	{ .compatible = "adi,ltc6952" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc6952_of_match);

static struct spi_driver ltc6952_driver = {
	.driver = {
		.name = "ltc6952",
		.of_match_table = ltc6952_of_match,
	},
	.probe = ltc6952_probe,
	.remove = ltc6952_remove,
};
module_spi_driver(ltc6952_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC6952 driver");
MODULE_LICENSE("GPL v2");
