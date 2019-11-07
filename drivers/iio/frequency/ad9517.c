/*
 * AD9517 SPI Clock Generator with integrated VCO
 *
 * Copyright 2013-2015 Analog Devices Inc.
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
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/mutex.h>

#include <linux/gpio/consumer.h>

#include <linux/clk.h>
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

/* LVDS/CMOS delay registers */
#define AD9517_OUT_DELAY_BP(x) (0xA0 + (x) * 3)
#define AD9517_OUT_DELAY_FS(x) (0xA1 + (x) * 3)
#define AD9517_OUT_DELAY_FR(x) (0xA2 + (x) * 3)

#define AD9517_OUT_LVPECL(x) (0xF0 + (x))

#define AD9517_OUT_CMOS(x) (0x140 + (x))

/* LVPECL Channel Dividers */
#define AD9517_PECLDIV_1(x) (0x190 + (x) * 3)
#define AD9517_PECLDIV_2(x) (0x191 + (x) * 3)
#define AD9517_PECLDIV_3(x) (0x192 + (x) * 3)

/* LVDS/CMOS Channel Dividers */
#define AD9517_CMOSDIV_1(x)		(0x199 + (x) * 5)
#define AD9517_CMOSDIV_1_PHO(x)		(0x19A + (x) * 5)
#define AD9517_CMOSDIV_2(x)		(0x19B + (x) * 5)
#define AD9517_CMOSDIV_BYPASS(x)	(0x19C + (x) * 5)
#define AD9517_CMOSDIV_DCCOFF(x)	(0x19D + (x) * 5)

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

#define MAX_NUM_DIVIDERS 5
#define MAX_NUM_OUTPUTS 10

/*
 * The address field of the channel is used to identify the output type
 * (LVDS/CMOS or LVPECL) and the offset in the register map. The offset is
 * storeed in the lowerd 8 bits and the type in bit 9.
 */
#define AD9517_ADDRESS_CHAN_TYPE_LVPECL 0x100

#define AD9517_ADDRESS_LVPECL(x) (AD9517_ADDRESS_CHAN_TYPE_LVPECL | (x))
#define AD9517_ADDRESS_CMOS(x) (x)

#define AD9517_ADDRESS_INDEX(x) ((x) & 0xff)

/* Two channels share one divider */
#define AD9517_ADDRESS_DIVIDER_INDEX(x) (AD9517_ADDRESS_INDEX(x) / 2)

struct ad9517_platform_data {
	unsigned long *regs;
	unsigned num_regs;
};

struct ad9517_clk_div {
	struct ad9517_state *st;
	unsigned int address;

	struct clk_hw hw;
};

struct ad9517_outputs {
	struct ad9517_state *st;
	struct clk_hw hw;
	const char *parent_name;
	unsigned int address;
};

struct ad9517_state {
	struct spi_device *spi;
	unsigned char regs[AD9517_TRANSFER+1];

	struct ad9517_clk_div clk_divs[MAX_NUM_DIVIDERS];

	struct ad9517_outputs output[MAX_NUM_OUTPUTS];
	struct clk *clks[MAX_NUM_OUTPUTS];
	struct clk_onecell_data clk_data;

	unsigned long refin_freq;
	unsigned long clkin_freq;
	unsigned long div0123_freq;
	unsigned long vco_divin_freq;

	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_sync;

	struct mutex lock;
};

/* Registers not listed are implicitly initialized to 0x00 */
static const unsigned char ad9517_default_regs[AD9517_TRANSFER+1] = {
	[AD9517_PFD_CP] = 0x7d,
	[AD9517_RCNT_L] = 0x01,
	[AD9517_BCNT_L] = 0x03,
	[AD9517_PLL1] = 0x06,
	[AD9517_PLL3] = 0x06,
	[AD9517_OUT_DELAY_BP(0)] = 0x01,
	[AD9517_OUT_DELAY_BP(1)] = 0x01,
	[AD9517_OUT_DELAY_BP(2)] = 0x01,
	[AD9517_OUT_DELAY_BP(3)] = 0x01,
	[AD9517_OUT_LVPECL(0)] = 0x0a,
	[AD9517_OUT_LVPECL(1)] = 0x0a,
	[AD9517_OUT_LVPECL(2)] = 0x0a,
	[AD9517_OUT_LVPECL(3)] = 0x0a,
	[AD9517_OUT_LVPECL(4)] = 0x0a,
	[AD9517_OUT_LVPECL(5)] = 0x0a,
	[AD9517_OUT_CMOS(0)] = 0x43,
	[AD9517_OUT_CMOS(1)] = 0x43,
	[AD9517_OUT_CMOS(2)] = 0x43,
	[AD9517_OUT_CMOS(3)] = 0x43,
	[AD9517_PECLDIV_2(0)] = 0x80,
	[AD9517_PECLDIV_2(1)] = 0x80,
	[AD9517_PECLDIV_2(2)] = 0x80,
	[AD9517_CMOSDIV_1(0)] = 0x22,
	[AD9517_CMOSDIV_2(0)] = 0x11,
	[AD9517_CMOSDIV_1(1)] = 0x22,
	[AD9517_CMOSDIV_2(1)] = 0x11,
	[AD9517_VCO_DIVIDER] = 0x02,
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

#define to_ad9517_clk_output(_hw) container_of(_hw, struct ad9517_outputs, hw)

static inline struct ad9517_clk_div *clk_to_ad9517_clk_div(struct clk_hw *hw)
{
	return container_of(hw, struct ad9517_clk_div, hw);
}

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
				 const char *data, unsigned size)
{
	unsigned addr, val1, val2;
	char *line;
	int ret;

	line = kmalloc(size + 1, GFP_KERNEL);
	if (!line)
		return -ENOMEM;
	memcpy(line, data, size);
	line[size] = '\0';
	while (line) {
		ret = sscanf(line, "\"%x\",\"%x\",\"%x\"", &addr, &val1, &val2);
		if (ret == 3) {
			if (addr > AD9517_TRANSFER) {
				kfree(line);
				return -EINVAL;
			}
			st->regs[addr] = val2 & 0xFF;
		}
		line = strchr(line, '\n');
		if (line != NULL)
		    line++;
	}
	kfree(line);

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

static int ad9517_calc_divider_hi_lo(unsigned ratio, unsigned *hi, unsigned *lo)
{

	if (ratio < 2 || ratio > 32)
		return -EINVAL;

	*hi = ratio / 2 - 1;
	*lo = *hi + (ratio % 2);

	return 0;
}

static int ad9517_calc_d12_dividers(unsigned vco, unsigned out,  unsigned *d1_val, unsigned *d2_val)
{
	unsigned d1, d2, _d2 = 0, _d1 = 0, ratio;
	unsigned err, min = UINT_MAX;

	ratio = DIV_ROUND_CLOSEST(vco, out);
	ratio = clamp_t(unsigned, ratio, 1, 32 * 32);

	if (ratio == 1) {
		*d1_val = 1;
		*d2_val = 1; /* Bypass */
		return 0;
	}

	if (ratio <= 32) {
		*d1_val = ratio;
		*d2_val = 1; /* Bypass */
		return 0;
	}

	for (d1 = 1; d1 <= 32; d1++) {
		d2 = DIV_ROUND_CLOSEST(ratio, d1);

		if (d2 > 32 || !d2)
			continue;

		err = abs(out - vco / (d1 * d2));
		if (err < min) {
			_d1 = d1;
			_d2 = d2;
			min = err;
		}

		if (err == 0)
			break;
	}

	*d2_val = min(_d2, _d1);
	*d1_val = max(_d2, _d1);

   return 0;
}

static int ad9517_lvdscmos_set_frequency(struct ad9517_state *st,
	unsigned int addr, unsigned int val)
{
	unsigned reg_bypass, reg_div1, reg_div2;
	unsigned d1, d2, hi, lo;
	unsigned int reg_index;
	int ret;

	reg_index = AD9517_ADDRESS_DIVIDER_INDEX(addr);

	reg_bypass = AD9517_CMOSDIV_BYPASS(reg_index);
	reg_div1 = AD9517_CMOSDIV_1(reg_index);
	reg_div2 = AD9517_CMOSDIV_2(reg_index);

	ad9517_calc_d12_dividers(st->div0123_freq, val, &d1, &d2);

	if (d1 == 1)
		st->regs[reg_bypass] |= AD9517_CMOSDIV_BYPASS_1;
	else
		st->regs[reg_bypass] &= ~AD9517_CMOSDIV_BYPASS_1;

	if (d2 == 1)
		st->regs[reg_bypass] |= AD9517_CMOSDIV_BYPASS_2;
	else
		st->regs[reg_bypass] &= ~AD9517_CMOSDIV_BYPASS_2;

	ret = ad9517_write(st->spi, reg_bypass, st->regs[reg_bypass]);
	if (ret < 0)
		return ret;

	ret = ad9517_calc_divider_hi_lo(d1, &hi, &lo);
	if (ret >= 0) {
		st->regs[reg_div1] = (hi << 4) | (lo & 0xF);
		ret = ad9517_write(st->spi, reg_div1, st->regs[reg_div1]);
		if (ret < 0)
			return ret;
	}

	ret = ad9517_calc_divider_hi_lo(d2, &hi, &lo);
	if (ret >= 0) {
		st->regs[reg_div2] = (hi << 4) | (lo & 0xF);
		ret = ad9517_write(st->spi, reg_div2, st->regs[reg_div2]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9517_lvpecl_set_frequency(struct ad9517_state *st,
	unsigned int addr, unsigned int val)
{
	unsigned reg_bypass, reg_div1, reg_bypass2;
	unsigned d1, hi, lo;
	unsigned int reg_index;
	int ret;

	reg_index = AD9517_ADDRESS_DIVIDER_INDEX(addr);

	reg_bypass = AD9517_PECLDIV_3(reg_index);
	reg_div1 = AD9517_PECLDIV_1(reg_index);
	reg_bypass2 = AD9517_PECLDIV_2(reg_index);

	if (val > st->div0123_freq) {
		st->regs[reg_bypass] |= AD9517_PECLDIV_VCO_SEL;
		return ad9517_write(st->spi, reg_bypass, st->regs[reg_bypass]);
	} else {
		st->regs[reg_bypass] &= ~AD9517_PECLDIV_VCO_SEL;
		ret = ad9517_write(st->spi, reg_bypass, st->regs[reg_bypass]);
		if (ret < 0)
			return ret;
	}

	d1 = DIV_ROUND_CLOSEST(st->div0123_freq, val);
	d1 = clamp_t(unsigned, d1, 1, 32);

	if (d1 ==  1) {
		st->regs[reg_bypass2] |= AD9517_PECLDIV_3_BP;
	} else {
		st->regs[reg_bypass2] &= ~AD9517_PECLDIV_3_BP;
	}

	ret = ad9517_write(st->spi, reg_bypass2, st->regs[reg_bypass2]);
	if (ret < 0)
		return ret;

	ret = ad9517_calc_divider_hi_lo(d1, &hi, &lo);
	if (ret >= 0) {
		st->regs[reg_div1] = (hi << 4) | (lo & 0xF);
		ret = ad9517_write(st->spi, reg_div1, st->regs[reg_div1]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9517_set_frequency(struct ad9517_state *st, unsigned int address,
	unsigned int rate)
{
	int ret;

	mutex_lock(&st->lock);
	if (address & AD9517_ADDRESS_CHAN_TYPE_LVPECL)
		ret = ad9517_lvpecl_set_frequency(st, address, rate);
	else
		ret = ad9517_lvdscmos_set_frequency(st, address, rate);

	if (ret < 0)
		goto out_unlock;

	ret = ad9517_write(st->spi, AD9517_TRANSFER, AD9517_TRANSFER_NOW);

out_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static unsigned long ad9517_get_frequency(struct ad9517_state *st,
	unsigned int address)
{
	unsigned int reg_index;
	unsigned long rate;
	unsigned d1, d2;

	reg_index = AD9517_ADDRESS_DIVIDER_INDEX(address);

	mutex_lock(&st->lock);
	if (address & AD9517_ADDRESS_CHAN_TYPE_LVPECL) {
		unsigned int div1, div2, div3;

		div1 = st->regs[AD9517_PECLDIV_1(reg_index)];
		div2 = st->regs[AD9517_PECLDIV_2(reg_index)];
		div3 = st->regs[AD9517_PECLDIV_3(reg_index)];

		if (div3 & AD9517_PECLDIV_VCO_SEL) {
			rate = st->vco_divin_freq;
		} else {
			if (div2 & AD9517_PECLDIV_3_BP)
				d1 = 1;
			else
				d1 = (div1 & 0xF) + (div1 >> 4) + 2;

			rate = st->div0123_freq / d1;
		}
	} else {
		unsigned int bypass, div1, div2;

		bypass = st->regs[AD9517_CMOSDIV_BYPASS(reg_index)];
		div1 = st->regs[AD9517_CMOSDIV_1(reg_index)];
		div2 = st->regs[AD9517_CMOSDIV_2(reg_index)];

		if (bypass & AD9517_CMOSDIV_BYPASS_1)
			d1 = 1;
		else
			d1 = (div1 & 0xF) + (div1 >> 4) + 2;

		if (bypass & AD9517_CMOSDIV_BYPASS_2)
			d2 = 1;
		else
			d2 = (div2 & 0xF) + (div2 >> 4) + 2;

		rate = st->div0123_freq / (d1 * d2);
	}
	mutex_unlock(&st->lock);

	return rate;
}

static int ad9517_out_enable(struct ad9517_state *st, unsigned int address,
	unsigned int val)
{
	unsigned int mask, reg;
	int ret;

	if (address & AD9517_ADDRESS_CHAN_TYPE_LVPECL) {
		reg = AD9517_OUT_LVPECL(AD9517_ADDRESS_INDEX(address));
		mask = 3;
	} else {
		reg = AD9517_OUT_CMOS(AD9517_ADDRESS_INDEX(address));
		mask = 1;
	}

	mutex_lock(&st->lock);
	if (val)
		st->regs[reg] &= ~mask;
	else
		st->regs[reg] |= mask;

	ret = ad9517_write(st->spi, reg, st->regs[reg]);
	if (ret < 0)
		goto out_unlock;

	ret = ad9517_write(st->spi, AD9517_TRANSFER, AD9517_TRANSFER_NOW);

out_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static bool ad9517_is_enabled(struct ad9517_state *st, unsigned int address)
{
	unsigned int mask, reg;

	if (address & AD9517_ADDRESS_CHAN_TYPE_LVPECL) {
		reg = AD9517_OUT_LVPECL(AD9517_ADDRESS_INDEX(address));
		mask = 3;
	} else {
		reg = AD9517_OUT_CMOS(AD9517_ADDRESS_INDEX(address));
		mask = 1;
	}

	return (st->regs[reg] & mask) == 0;
}

static const char *ad9517_get_parent_name(struct ad9517_state *st,
	const char *name)
{
	int i;

	i = of_property_match_string(st->spi->dev.of_node, "clock-names", name);
	if (i < 0)
		return ERR_PTR(i);

	return of_clk_get_parent_name(st->spi->dev.of_node, i);
}

static int ad9517_setup(struct ad9517_state *st, unsigned int num_outputs)
{
	struct spi_device *spi = st->spi;
	int ret, reg;
	unsigned cal_delay_ms;
	unsigned long pll_a_cnt, pll_b_cnt, pll_r_cnt, prescaler;
	unsigned long vco_freq;
	unsigned long vco_divin_freq;
	unsigned long div0123_freq;
	bool uses_vco = false;
	bool uses_clkin = false;
	const char *div0123_parent_name;
	const char *vco_divin_parent_name;
	const char *parent_name;
	unsigned int reg_index;
	unsigned int div3;
	unsigned int i;

	gpiod_set_value(st->gpio_sync, 1);

	/* Setup PLL */
	for (reg = AD9517_PFD_CP; reg <= AD9517_PLL8; reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup fine delay adjust OUT4..OUT7 */
	for (reg = AD9517_OUT_DELAY_BP(0); reg <= AD9517_OUT_DELAY_FR(3); reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup LVPECL outputs OUT0..OUT3 */
	for (reg = AD9517_OUT_LVPECL(0); reg <= AD9517_OUT_LVPECL(3); reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup LVDS/CMOS outputs OUT4..OUT7 */
	for (reg = AD9517_OUT_CMOS(0); reg <= AD9517_OUT_CMOS(3); reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup PECL Channel Dividers */
	for (reg = AD9517_PECLDIV_1(0); reg <= AD9517_PECLDIV_3(2); reg++) {
		ret = ad9517_write(spi, reg, st->regs[reg]);
		if (ret < 0)
			return ret;
	}

	/* Setup LVDS/CMOS Channel Dividers */
	for (reg = AD9517_CMOSDIV_1(0); reg <= AD9517_CMOSDIV_DCCOFF(1); reg++) {
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

	if (st->regs[AD9517_INPUT_CLKS] & AD9517_VCO_DIVIDER_SEL)
		uses_vco = true;
	else
		uses_clkin = true;

	if (st->regs[AD9517_INPUT_CLKS] & AD9517_VCO_DIVIDER_BP)
		uses_clkin = true;

	if (uses_vco) {
		if (st->refin_freq == 0) {
			dev_err(&st->spi->dev, "Invalid or missing REFIN clock\n");
			return -EINVAL;
		}

		vco_freq = (st->refin_freq / pll_r_cnt * (prescaler  *
				pll_b_cnt + pll_a_cnt));

		/* tcal = 4400 * Rdiv * cal_div / Refin */
		cal_delay_ms = (4400 * pll_r_cnt *
			(2 << ((st->regs[AD9517_PLL3] >> 1) & 0x3))) /
			(st->refin_freq / 1000);

		msleep(cal_delay_ms);
	}

	if (uses_clkin) {
		if (st->clkin_freq == 0) {
			dev_err(&st->spi->dev, "Invalid or missing CLKIN clock\n");
			return -EINVAL;
		}
	}

	/* Internal clock distribution */
	if (uses_vco) {
		vco_divin_freq = vco_freq;
		vco_divin_parent_name = ad9517_get_parent_name(st, "refclk");
	} else {
		vco_divin_freq = st->clkin_freq;
		vco_divin_parent_name = ad9517_get_parent_name(st, "clkin");
	}

	if (IS_ERR(vco_divin_parent_name))
		return PTR_ERR(vco_divin_parent_name);

	if (st->regs[AD9517_INPUT_CLKS] & AD9517_VCO_DIVIDER_BP) {
		div0123_freq = st->clkin_freq;
		div0123_parent_name = ad9517_get_parent_name(st, "clkin");
		if (IS_ERR(div0123_parent_name))
			return PTR_ERR(div0123_parent_name);
	} else {
		div0123_freq = vco_divin_freq /
			((st->regs[AD9517_VCO_DIVIDER] & 0x7) + 2);
		div0123_parent_name = vco_divin_parent_name;
	}

	/* CMOS/LVDS outputs */
	for (i = 0; i < num_outputs; i += 2) {
		reg_index = AD9517_ADDRESS_DIVIDER_INDEX(st->output[i].address);
		div3 = st->regs[AD9517_PECLDIV_3(reg_index)];

		/* LVPECL have an additional MUX infront of the divider */
		if ((st->output[i].address & AD9517_ADDRESS_CHAN_TYPE_LVPECL) &&
		    (div3 & AD9517_PECLDIV_VCO_SEL))
			parent_name = vco_divin_parent_name;
		else
			parent_name = div0123_parent_name;

		st->output[i].parent_name = parent_name;
		st->output[i+1].parent_name = parent_name;
	}

	st->div0123_freq = div0123_freq;
	st->vco_divin_freq = vco_divin_freq;

	gpiod_set_value(st->gpio_sync, 0);

	return 0;
}

static unsigned long ad9517_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	return ad9517_get_frequency(clk_to_ad9517_clk_div(hw)->st,
				    clk_to_ad9517_clk_div(hw)->address);
}

static long ad9517_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *prate)
{
	struct ad9517_state *st = clk_to_ad9517_clk_div(hw)->st;
	unsigned int address = clk_to_ad9517_clk_div(hw)->address;
	long rrate;
	unsigned d1, d2;

	if (address & AD9517_ADDRESS_CHAN_TYPE_LVPECL) {
		if (rate > st->div0123_freq) {
			rrate = st->vco_divin_freq;
		} else {
			d1 = DIV_ROUND_CLOSEST(st->div0123_freq, rate);
			d1 = clamp_t(unsigned, d1, 1, 32);
			rrate = st->div0123_freq / d1;
		}
	} else {
		ad9517_calc_d12_dividers(st->div0123_freq, rate, &d1, &d2);
		rrate = st->div0123_freq / (d1 * d2);
	}

	return rrate;
}

static int ad9517_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long prate)
{
	struct ad9517_state *st = clk_to_ad9517_clk_div(hw)->st;
	unsigned int address = clk_to_ad9517_clk_div(hw)->address;

	return ad9517_set_frequency(st, address, rate);
}

static int ad9517_clk_is_enabled(struct clk_hw *hw)
{
	return ad9517_is_enabled(to_ad9517_clk_output(hw)->st,
				 to_ad9517_clk_output(hw)->address);
}

static int ad9517_clk_prepare(struct clk_hw *hw)
{
	struct ad9517_state *st = to_ad9517_clk_output(hw)->st;

	return ad9517_out_enable(st, to_ad9517_clk_output(hw)->address, 1);
}

static void ad9517_clk_unprepare(struct clk_hw *hw)
{
	struct ad9517_state *st = to_ad9517_clk_output(hw)->st;

	ad9517_out_enable(st, to_ad9517_clk_output(hw)->address, 0);
}

static const struct clk_ops ad9517_clk_div_ops = {
	.recalc_rate = ad9517_recalc_rate,
	.set_rate = ad9517_clk_set_rate,
	.round_rate = ad9517_clk_round_rate,
};

static const struct clk_ops ad9517_clk_ops = {
	.is_enabled = ad9517_clk_is_enabled,
	.prepare = ad9517_clk_prepare,
	.unprepare = ad9517_clk_unprepare,
};

static struct clk *ad9517_clk_register(struct ad9517_state *st,
	unsigned int num)
{
	struct ad9517_outputs *output = &st->output[num];
	struct clk_init_data init;
	unsigned int div_num;
	char div_name[128];
	const char *parent_name;
	struct clk *clk;
	char name[8];
	int ret;

	div_num = num / 2;

	snprintf(div_name, sizeof(div_name), "%s-div%d",
		dev_name(&st->spi->dev), div_num);
	parent_name = div_name;

	/* Register a clock divider for every second clock */
	if (num % 2 == 0) {
		init.ops = &ad9517_clk_div_ops;
		init.parent_names = &output->parent_name;
		init.num_parents = 1;
		init.name = div_name;
		init.flags = 0;

		st->clk_divs[div_num].st = st;
		st->clk_divs[div_num].hw.init = &init;
		st->clk_divs[div_num].address = output->address;

		clk = devm_clk_register(&st->spi->dev,
			&st->clk_divs[div_num].hw);
		if (IS_ERR(clk))
			return clk;
	}

	init.ops = &ad9517_clk_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_PARENT;

	ret = of_property_read_string_index(st->spi->dev.of_node,
		"clock-output-names", num, &init.name);
	if (ret < 0) {
		sprintf(name, "out%d", num);
		init.name = name;
	}

	output->hw.init = &init;
	output->st = st;

	/* register the clock */
	clk = devm_clk_register(&st->spi->dev, &output->hw);
	st->clk_data.clks[num] = clk;

	return clk;
}

static int ad9517_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct ad9517_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
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
	mutex_unlock(&st->lock);

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
		*val = ad9517_is_enabled(st, chan->address);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		*val = (int) ad9517_get_frequency(st, chan->address);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
};

static int ad9517_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long m)
{
	struct ad9517_state *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		return ad9517_out_enable(st, chan->address, val);
	case IIO_CHAN_INFO_FREQUENCY:
		return ad9517_set_frequency(st, chan->address, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad9517_info = {
	.read_raw = &ad9517_read_raw,
	.write_raw = ad9517_write_raw,
	.debugfs_reg_access = &ad9517_reg_access,
};

#define AD9517_CHAN(_chan, _addr)				\
	{ .type = IIO_ALTVOLTAGE,				\
	  .indexed = 1,						\
	  .channel = _chan,					\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | 	\
			BIT(IIO_CHAN_INFO_FREQUENCY),		\
	  .output = 1,						\
	  .address = _addr }

static const struct iio_chan_spec ad9516_chan[] = {
	AD9517_CHAN(0, AD9517_ADDRESS_LVPECL(0)),
	AD9517_CHAN(1, AD9517_ADDRESS_LVPECL(1)),
	AD9517_CHAN(2, AD9517_ADDRESS_LVPECL(2)),
	AD9517_CHAN(3, AD9517_ADDRESS_LVPECL(3)),
	AD9517_CHAN(4, AD9517_ADDRESS_LVPECL(4)),
	AD9517_CHAN(5, AD9517_ADDRESS_LVPECL(5)),
	AD9517_CHAN(6, AD9517_ADDRESS_CMOS(0)),
	AD9517_CHAN(7, AD9517_ADDRESS_CMOS(1)),
	AD9517_CHAN(8, AD9517_ADDRESS_CMOS(2)),
	AD9517_CHAN(9, AD9517_ADDRESS_CMOS(3)),
};

static const struct iio_chan_spec ad9517_chan[] = {
	AD9517_CHAN(0, AD9517_ADDRESS_LVPECL(0)),
	AD9517_CHAN(1, AD9517_ADDRESS_LVPECL(1)),
	/* Register map gap between channel 1 and 2 */
	AD9517_CHAN(2, AD9517_ADDRESS_LVPECL(4)),
	AD9517_CHAN(3, AD9517_ADDRESS_LVPECL(5)),
	AD9517_CHAN(4, AD9517_ADDRESS_CMOS(0)),
	AD9517_CHAN(5, AD9517_ADDRESS_CMOS(1)),
	AD9517_CHAN(6, AD9517_ADDRESS_CMOS(2)),
	AD9517_CHAN(7, AD9517_ADDRESS_CMOS(3)),
};

enum ad9517_device_type {
	AD9516,
	AD9517,
	AD9518,
};

#define AD9517_DRIVER_DATA(type, part_id) \
	((type << 8) | part_id)

struct ad9517_device_info {
	unsigned int num_channels;
	const struct iio_chan_spec *channels;
};

static const struct ad9517_device_info ad9517_device_info[] = {
	[AD9516] = {
		.num_channels = ARRAY_SIZE(ad9516_chan),
		.channels = ad9516_chan,
	},
	[AD9517] = {
		.num_channels = ARRAY_SIZE(ad9517_chan),
		.channels = ad9517_chan,
	},
	[AD9518] = { /* Same layout as AD9516 but only LVPECL outputs */
		.num_channels = 6,
		.channels = ad9516_chan,
	},
};

static int ad9517_probe(struct spi_device *spi)
{
	struct ad9517_platform_data *pdata = spi->dev.platform_data;
	const struct spi_device_id *id;
	struct iio_dev *indio_dev;
	int out, ret, conf;
	const struct firmware *fw;
	struct ad9517_state *st;
	struct clk *clk, *ref_clk, *clkin;
	bool spi3wire = of_property_read_bool(
			spi->dev.of_node, "adi,spi-3wire-enable");
	unsigned int device_type, part_id;

	id = spi_get_device_id(spi);
	device_type = id->driver_data >> 8;
	part_id = id->driver_data & 0xff;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);

	st->gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset",
	    GPIOD_OUT_HIGH);
	if (st->gpio_reset) {
		udelay(10);
		gpiod_set_value(st->gpio_reset, 0);
	}

	st->gpio_sync = devm_gpiod_get_optional(&spi->dev, "sync",
	    GPIOD_OUT_HIGH);

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
	if (ret != part_id) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", ret);
 		return -ENODEV;
	}

	if (!pdata) {
		const char *name;
		if (spi->dev.of_node) {
			if (of_property_read_string(spi->dev.of_node, "firmware", &name))
				name = NULL;
		} else {
			name = FIRMWARE;
		}

		if (name) {
			ret = request_firmware(&fw, name, &spi->dev);
			if (ret) {
				dev_err(&spi->dev,
					"request_firmware() failed with %i\n", ret);
				return ret;
			}
			ad9517_parse_firmware(st, fw->data, fw->size);
			release_firmware(fw);
		} else {
			memcpy(st->regs, ad9517_default_regs, sizeof(st->regs));
		}
	} else {
		ret = ad9517_parse_pdata(st, pdata);
		if (ret < 0) {
			dev_err(&spi->dev,
				"parse pdata failed with %i\n", ret);
			return ret;
		}
	}

	st->spi = spi;

	ref_clk = devm_clk_get(&spi->dev, "refclk");
	if (IS_ERR(ref_clk)) {
		ret = PTR_ERR(ref_clk);
		if (ret != -ENOENT) {
			dev_err(&spi->dev, "Failed getting REFIN clock (%d)\n", ret);
			return ret;
		}
	} else {
		st->refin_freq = clk_get_rate(ref_clk);
		clk_prepare_enable(ref_clk);

	}

	clkin = devm_clk_get(&spi->dev, "clkin");
	if (IS_ERR(clkin)) {
		ret = PTR_ERR(clkin);
		if (ret != -ENOENT) {
			dev_err(&spi->dev, "Failed getting CLK clock (%d)\n", ret);
			return ret;
		}
	} else {
		st->clkin_freq = clk_get_rate(clkin);
		clk_prepare_enable(clkin);
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad9517_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9517_device_info[device_type].channels;
	indio_dev->num_channels = ad9517_device_info[device_type].num_channels;

	/* Needs to be done before ad9517_setup() */
	for (out = 0; out < indio_dev->num_channels; out++)
		st->output[out].address = indio_dev->channels[out].address;

	ret = ad9517_setup(st, indio_dev->num_channels);
	if (ret < 0)
		return ret;

	st->clk_data.clks = st->clks;
	st->clk_data.clk_num = indio_dev->num_channels;

	for (out = 0; out < indio_dev->num_channels; out++) {
		clk = ad9517_clk_register(st, out);
		if (IS_ERR(clk))
			return PTR_ERR(clk);
	}

	of_clk_add_provider(st->spi->dev.of_node,
			    of_clk_src_onecell_get, &st->clk_data);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_of_clk_del_provider;

	spi_set_drvdata(spi, indio_dev);

	dev_info(&spi->dev, "AD9517 successfully initialized");

	return 0;

err_of_clk_del_provider:
	of_clk_del_provider(spi->dev.of_node);
	return ret;
}

static int ad9517_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);
	of_clk_del_provider(spi->dev.of_node);

	return 0;
}

static const struct spi_device_id ad9517_id[] = {
	{"ad9516-0", AD9517_DRIVER_DATA(AD9516, 0x01) },
	{"ad9516-1", AD9517_DRIVER_DATA(AD9516, 0x41) },
	{"ad9516-2", AD9517_DRIVER_DATA(AD9516, 0x81) },
	{"ad9516-3", AD9517_DRIVER_DATA(AD9516, 0x43) },
	{"ad9516-4", AD9517_DRIVER_DATA(AD9516, 0xc3) },
	{"ad9516-5", AD9517_DRIVER_DATA(AD9516, 0xc1) },
	{"ad9517-0", AD9517_DRIVER_DATA(AD9517, 0x11) },
	{"ad9517-1", AD9517_DRIVER_DATA(AD9517, 0x51) },
	{"ad9517-2", AD9517_DRIVER_DATA(AD9517, 0x91) },
	{"ad9517-3", AD9517_DRIVER_DATA(AD9517, 0x53) },
	{"ad9517-4", AD9517_DRIVER_DATA(AD9517, 0xD3) },
	{"ad9518-0", AD9517_DRIVER_DATA(AD9518, 0x21) },
	{"ad9518-1", AD9517_DRIVER_DATA(AD9518, 0x61) },
	{"ad9518-2", AD9517_DRIVER_DATA(AD9518, 0xa1) },
	{"ad9518-3", AD9517_DRIVER_DATA(AD9518, 0x63) },
	{"ad9518-4", AD9517_DRIVER_DATA(AD9518, 0xe3) },
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
