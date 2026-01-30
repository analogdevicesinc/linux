// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Silicon Labs Si5391
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/math64.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#define SI5391_NUM_INPUTS 4

#define SI5391_MAX_NUM_OUTPUTS 12

#define SI5391_NUM_SYNTH 5

/* Range of the synthesizer fractional divider */
#define SI5391_SYNTH_N_MIN	10
#define SI5391_SYNTH_N_MAX	4095

/* The chip can get its input clock from 3 input pins or an XTAL */

/* There is one PLL running at 13500–14256 MHz */
#define SI5391_PLL_VCO_MIN 13500000000ull
#define SI5311_PLL_VCO_MAX 14256000000ull

/* The 5 frequency synthesizers obtain their input from the PLL */
struct clk_si5391_synth {
	struct clk_hw hw;
	struct clk_si5391 *data;
	u8 index;
};
#define to_clk_si5391_synth(_hw) \
	container_of(_hw, struct clk_si5391_synth, hw)

/* The output stages can be connected to any synth (full mux) */
struct clk_si5391_output {
	struct clk_hw hw;
	struct clk_si5391 *data;
	struct regulator *vddo_reg;
	u8 index;
};
#define to_clk_si5391_output(_hw) \
	container_of(_hw, struct clk_si5391_output, hw)

struct clk_si5391 {
	struct clk_hw hw;
	struct regmap *regmap;
	struct i2c_client *i2c_client;
	struct clk_si5391_synth synth[SI5391_NUM_SYNTH];
	struct clk_si5391_output clk[SI5391_MAX_NUM_OUTPUTS];
	struct clk *input_clk[SI5391_NUM_INPUTS];
	const char *input_clk_name[SI5391_NUM_INPUTS];
	const u16 *reg_output_offset;
	const u16 *reg_rdiv_offset;
	u64 freq_vco; /* 13500–14256 MHz */
	u8 num_outputs;
	u8 num_synth;
	u16 chip_id;
	bool xaxb_ext_clk;
	bool iovdd_33;
};
#define to_clk_si5391(_hw)	container_of(_hw, struct clk_si5391, hw)

struct clk_si5391_output_config {
	u8 out_format_drv_bits;
	u8 out_cm_ampl_bits;
	u8 vdd_sel_bits;
	bool synth_master;
	bool always_on;
};

#define SI5391_PAGE		0x0001
#define SI5391_PN_BASE		0x0002
#define SI5391_DEVICE_REV	0x0005
#define SI5391_STATUS		0x000C
#define SI5391_LOS		0x000D
#define SI5391_STATUS_STICKY	0x0011
#define SI5391_LOS_STICKY	0x0012
#define SI5391_SOFT_RST		0x001C
#define SI5391_IN_SEL		0x0021
#define SI5391_DEVICE_READY	0x00FE
#define SI5391_XAXB_CFG		0x090E
#define SI5391_IO_VDD_SEL	0x0943
#define SI5391_IN_EN		0x0949
#define SI5391_INX_TO_PFD_EN	0x094A

/* Status bits */
#define SI5391_STATUS_SYSINCAL	BIT(0)
#define SI5391_STATUS_LOSXAXB	BIT(1)
#define SI5391_STATUS_LOSREF	BIT(2)
#define SI5391_STATUS_LOL	BIT(3)

/* Input selection */
#define SI5391_IN_SEL_MASK	0x06
#define SI5391_IN_SEL_SHIFT	1
#define SI5391_IN_SEL_REGCTRL	0x01
#define SI5391_INX_TO_PFD_SHIFT	4

/* XTAL config bits */
#define SI5391_XAXB_CFG_EXTCLK_EN	BIT(0)
#define SI5391_XAXB_CFG_PDNB		BIT(1)

/* Input dividers (48-bit) */
#define SI5391_IN_PDIV(x)	(0x0208 + ((x) * 10))
#define SI5391_IN_PSET(x)	(0x020E + ((x) * 10))
#define SI5391_PX_UPD		0x0230

/* PLL configuration */
#define SI5391_PLL_M_NUM	0x0235
#define SI5391_PLL_M_DEN	0x023B

/* Output configuration */
#define SI5391_OUT_CONFIG(output)	\
			((output)->data->reg_output_offset[(output)->index])
#define SI5391_OUT_FORMAT(output)	(SI5391_OUT_CONFIG(output) + 1)
#define SI5391_OUT_CM(output)		(SI5391_OUT_CONFIG(output) + 2)
#define SI5391_OUT_MUX_SEL(output)	(SI5391_OUT_CONFIG(output) + 3)
#define SI5391_OUT_R_REG(output)	\
			((output)->data->reg_rdiv_offset[(output)->index])

#define SI5391_OUT_MUX_VDD_SEL_MASK 0x38

/* Synthesize N divider */
#define SI5391_SYNTH_N_NUM(x)	(0x0302 + ((x) * 11))
#define SI5391_SYNTH_N_DEN(x)	(0x0308 + ((x) * 11))
#define SI5391_SYNTH_N_UPD(x)	(0x030C + ((x) * 11))

/* Synthesizer output enable, phase bypass, power mode */
#define SI5391_SYNTH_N_CLK_TO_OUTX_EN	0x0A03
#define SI5391_SYNTH_N_PIBYP		0x0A04
#define SI5391_SYNTH_N_PDNB		0x0A05
#define SI5391_SYNTH_N_CLK_DIS		0x0B4A

#define SI5391_REGISTER_MAX	0xBFF

/* SI5391_OUT_CONFIG bits */
#define SI5391_OUT_CFG_PDN		BIT(0)
#define SI5391_OUT_CFG_OE		BIT(1)
#define SI5391_OUT_CFG_RDIV_FORCE2	BIT(2)

/* Static configuration (to be moved to firmware) */
struct si5391_reg_default {
	u16 address;
	u8 value;
};

static const char * const si5391_input_clock_names[] = {
	"in0", "in1", "in2", "xtal"
};

static const u16 si5391_reg_output_offset[] = {
	0x0103, /* OUT0A */
	0x0108, /* OUT0 */
	0x010D, /* OUT1 */
	0x0112, /* OUT2 */
	0x0117, /* OUT3 */
	0x011C, /* OUT4 */
	0x0121, /* OUT5 */
	0x0126, /* OUT6 */
	0x012B, /* OUT7 */
	0x0130, /* OUT8 */
	0x0135, /* OUT9 */
	0x013A, /* OUT9A */
};

/* The location of the R divider registers */
static const u16 si5391_reg_rdiv_offset[] = {
	0x0247, /* R0A_REG */
	0x024A, /* R0_REG */
	0x024D, /* R1_REG */
	0x0250, /* R2_REG */
	0x0253, /* R3_REG */
	0x0256, /* R4_REG */
	0x0259, /* R5_REG */
	0x025C, /* R6_REG */
	0x025F, /* R7_REG */
	0x0262, /* R8_REG */
	0x0265, /* R9_REG */
	0x0268, /* R9A_REG */
};

/*
 * Si5391 default register configuration
 * Generated from ClockBuilder Pro configuration
 * These values configure the PLL, dividers, and outputs
 */
static const struct si5391_reg_default si5391_reg_defaults[] = {
	/* Start configuration registers */
	{ 0x0006, 0x00 },
	{ 0x0007, 0x00 },
	{ 0x0008, 0x00 },
	{ 0x000B, 0x74 },
	{ 0x0017, 0xD0 },
	{ 0x0018, 0xFF },
	{ 0x0021, 0x0F },
	{ 0x0022, 0x00 },
	{ 0x002B, 0x02 },
	{ 0x002C, 0x20 },
	{ 0x002D, 0x00 },
	{ 0x002E, 0x00 },
	{ 0x002F, 0x00 },
	{ 0x0030, 0x00 },
	{ 0x0031, 0x00 },
	{ 0x0032, 0x00 },
	{ 0x0033, 0x00 },
	{ 0x0034, 0x00 },
	{ 0x0035, 0x00 },
	{ 0x0036, 0x00 },
	{ 0x0037, 0x00 },
	{ 0x0038, 0x00 },
	{ 0x0039, 0x00 },
	{ 0x003A, 0x00 },
	{ 0x003B, 0x00 },
	{ 0x003C, 0x00 },
	{ 0x003D, 0x00 },
	{ 0x0041, 0x00 },
	{ 0x0042, 0x00 },
	{ 0x0043, 0x00 },
	{ 0x0044, 0x00 },
	{ 0x009E, 0x00 },
	{ 0x0102, 0x01 },
	{ 0x0103, 0x01 },
	{ 0x0104, 0x09 },
	{ 0x0105, 0x3B },
	{ 0x0106, 0x28 },
	{ 0x0108, 0x01 },
	{ 0x0109, 0x09 },
	{ 0x010A, 0x3B },
	{ 0x010B, 0x28 },
	{ 0x010D, 0x01 },
	{ 0x010E, 0x09 },
	{ 0x010F, 0x3B },
	{ 0x0110, 0x28 },
	{ 0x0112, 0x01 },
	{ 0x0113, 0x09 },
	{ 0x0114, 0x3B },
	{ 0x0115, 0x28 },
	{ 0x0117, 0x01 },
	{ 0x0118, 0x09 },
	{ 0x0119, 0x3B },
	{ 0x011A, 0x28 },
	{ 0x011C, 0x01 },
	{ 0x011D, 0x09 },
	{ 0x011E, 0x3B },
	{ 0x011F, 0x28 },
	{ 0x0121, 0x01 },
	{ 0x0122, 0x09 },
	{ 0x0123, 0x3B },
	{ 0x0124, 0x28 },
	{ 0x0126, 0x01 },
	{ 0x0127, 0x09 },
	{ 0x0128, 0x3B },
	{ 0x0129, 0x28 },
	{ 0x012B, 0x01 },
	{ 0x012C, 0x09 },
	{ 0x012D, 0x3B },
	{ 0x012E, 0x28 },
	{ 0x0130, 0x06 },
	{ 0x0131, 0x09 },
	{ 0x0132, 0x3E },
	{ 0x0133, 0x18 },
	{ 0x0135, 0x06 },
	{ 0x0136, 0x09 },
	{ 0x0137, 0x3E },
	{ 0x0138, 0x18 },
	{ 0x013A, 0x01 },
	{ 0x013B, 0x09 },
	{ 0x013C, 0x3B },
	{ 0x013D, 0x28 },
	{ 0x013F, 0x00 },
	{ 0x0140, 0x00 },
	{ 0x0141, 0x40 },
	{ 0x0206, 0x00 },
	{ 0x0208, 0x00 },
	{ 0x0209, 0x00 },
	{ 0x020A, 0x00 },
	{ 0x020B, 0x00 },
	{ 0x020C, 0x00 },
	{ 0x020D, 0x00 },
	{ 0x020E, 0x00 },
	{ 0x020F, 0x00 },
	{ 0x0210, 0x00 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x00 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x00 },
	{ 0x0215, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x0218, 0x00 },
	{ 0x0219, 0x00 },
	{ 0x021A, 0x00 },
	{ 0x021B, 0x00 },
	{ 0x021C, 0x00 },
	{ 0x021D, 0x00 },
	{ 0x021E, 0x00 },
	{ 0x021F, 0x00 },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x00 },
	{ 0x0222, 0x00 },
	{ 0x0223, 0x00 },
	{ 0x0224, 0x00 },
	{ 0x0225, 0x00 },
	{ 0x0226, 0x00 },
	{ 0x0227, 0x00 },
	{ 0x0228, 0x00 },
	{ 0x0229, 0x00 },
	{ 0x022A, 0x00 },
	{ 0x022B, 0x00 },
	{ 0x022C, 0x00 },
	{ 0x022D, 0x00 },
	{ 0x022E, 0x00 },
	{ 0x022F, 0x00 },
	/* PLL configuration - M = 266.666... for 14.4 GHz VCO */
	{ 0x0235, 0x00 },  /* M_NUM byte 0 (LSB) */
	{ 0x0236, 0x00 },  /* M_NUM byte 1 */
	{ 0x0237, 0x00 },  /* M_NUM byte 2 */
	{ 0x0238, 0x00 },  /* M_NUM byte 3 */
	{ 0x0239, 0xC8 },  /* M_NUM byte 4 (bits 32-39) */
	{ 0x023A, 0x00 },  /* M_NUM byte 5 (bits 40-43) */
	{ 0x023B, 0x00 },  /* M_DEN byte 0 (LSB) */
	{ 0x023C, 0x00 },  /* M_DEN byte 1 */
	{ 0x023D, 0x00 },  /* M_DEN byte 2 */
	{ 0x023E, 0xC0 },  /* M_DEN byte 3 (MSB) */
	/* R dividers */
	{ 0x0247, 0x00 },
	{ 0x0248, 0x00 },
	{ 0x0249, 0x00 },
	{ 0x024A, 0x00 },
	{ 0x024B, 0x00 },
	{ 0x024C, 0x00 },
	{ 0x024D, 0x00 },
	{ 0x024E, 0x00 },
	{ 0x024F, 0x00 },
	{ 0x0250, 0x00 },
	{ 0x0251, 0x00 },
	{ 0x0252, 0x00 },
	{ 0x0253, 0x00 },
	{ 0x0254, 0x00 },
	{ 0x0255, 0x00 },
	{ 0x0256, 0x00 },
	{ 0x0257, 0x00 },
	{ 0x0258, 0x00 },
	{ 0x0259, 0x00 },
	{ 0x025A, 0x00 },
	{ 0x025B, 0x00 },
	{ 0x025C, 0x00 },
	{ 0x025D, 0x00 },
	{ 0x025E, 0x00 },
	{ 0x025F, 0x00 },
	{ 0x0260, 0x00 },
	{ 0x0261, 0x00 },
	{ 0x0262, 0x00 },  /* R8_REG - OUT8 divider */
	{ 0x0263, 0x00 },
	{ 0x0264, 0x00 },
	{ 0x0265, 0x00 },  /* R9_REG - OUT9 divider */
	{ 0x0266, 0x00 },
	{ 0x0267, 0x00 },
	{ 0x0268, 0x00 },
	{ 0x0269, 0x00 },
	{ 0x026A, 0x00 },
	/* Design ID */
	{ 0x026B, 0x00 },
	{ 0x026C, 0x00 },
	{ 0x026D, 0x00 },
	{ 0x026E, 0x00 },
	{ 0x026F, 0x00 },
	{ 0x0270, 0x00 },
	{ 0x0271, 0x00 },
	{ 0x0272, 0x00 },
	/* N dividers - N0 = 18 for 800 MHz */
	{ 0x0302, 0x00 },  /* N0_NUM LSB */
	{ 0x0303, 0x00 },
	{ 0x0304, 0x00 },
	{ 0x0305, 0x00 },
	{ 0x0306, 0x12 },  /* N0_NUM byte 4 (bits 32-39) */
	{ 0x0307, 0x00 },  /* N0_NUM byte 5 (bits 40-43) */
	{ 0x0308, 0x00 },  /* N0_DEN LSB */
	{ 0x0309, 0x00 },
	{ 0x030A, 0x00 },
	{ 0x030B, 0x80 },  /* N0_DEN MSB */
	{ 0x030C, 0x00 },  /* N0_UPDATE */
	{ 0x030D, 0x00 },
	{ 0x030E, 0x00 },
	{ 0x030F, 0x00 },
	{ 0x0310, 0x00 },
	{ 0x0311, 0x00 },
	{ 0x0312, 0x00 },
	{ 0x0313, 0x00 },
	{ 0x0314, 0x00 },
	{ 0x0315, 0x00 },
	{ 0x0316, 0x00 },
	{ 0x0317, 0x00 },
	{ 0x0318, 0x00 },
	{ 0x0319, 0x00 },
	{ 0x031A, 0x00 },
	{ 0x031B, 0x00 },
	{ 0x031C, 0x00 },
	{ 0x031D, 0x00 },
	{ 0x031E, 0x00 },
	{ 0x031F, 0x00 },
	{ 0x0320, 0x00 },
	{ 0x0321, 0x00 },
	{ 0x0322, 0x00 },
	{ 0x0323, 0x00 },
	{ 0x0324, 0x00 },
	{ 0x0325, 0x00 },
	{ 0x0326, 0x00 },
	{ 0x0327, 0x00 },
	{ 0x0328, 0x00 },
	{ 0x0329, 0x00 },
	{ 0x032A, 0x00 },
	{ 0x032B, 0x00 },
	{ 0x032C, 0x00 },
	{ 0x032D, 0x00 },
	{ 0x032E, 0x00 },
	{ 0x032F, 0x00 },
	{ 0x0330, 0x00 },
	{ 0x0331, 0x00 },
	{ 0x0332, 0x00 },
	{ 0x0333, 0x00 },
	{ 0x0334, 0x00 },
	{ 0x0335, 0x00 },
	{ 0x0336, 0x00 },
	{ 0x0337, 0x00 },
	{ 0x0338, 0x00 },
	{ 0x0339, 0x1F },
	/* Additional configuration */
	{ 0x090E, 0x02 },
	{ 0x091C, 0x04 },
	{ 0x0943, 0x00 },
	{ 0x0949, 0x00 },
	{ 0x094A, 0x00 },
	{ 0x094E, 0x49 },
	{ 0x094F, 0xF2 },
	{ 0x095E, 0x00 },
	{ 0x0A02, 0x00 },
	{ 0x0A03, 0x01 },
	{ 0x0A04, 0x01 },
	{ 0x0A05, 0x01 },
	{ 0x0A14, 0x00 },
	{ 0x0A1A, 0x00 },
	{ 0x0A20, 0x00 },
	{ 0x0A26, 0x00 },
	{ 0x0A2C, 0x00 },
	{ 0x0B44, 0x0F },
	{ 0x0B4A, 0x1E },
	{ 0x0B57, 0x0E },
	{ 0x0B58, 0x00 },
};
/* Read and interpret a 44-bit followed by a 32-bit value in the regmap */
static int si5391_decode_44_32(struct regmap *regmap, unsigned int reg,
	u64 *val1, u32 *val2)
{
	int err;
	u8 r[10];

	err = regmap_bulk_read(regmap, reg, r, 10);
	if (err < 0)
		return err;

	*val1 = ((u64)((r[5] & 0x0f) << 8 | r[4]) << 32) |
		 (get_unaligned_le32(r));
	*val2 = get_unaligned_le32(&r[6]);

	return 0;
}

static int si5391_encode_44_32(struct regmap *regmap, unsigned int reg,
	u64 n_num, u32 n_den)
{
	u8 r[10];

	/* Shift left as far as possible without overflowing */
	while (!(n_num & BIT_ULL(43)) && !(n_den & BIT(31))) {
		n_num <<= 1;
		n_den <<= 1;
	}

	/* 44 bits (6 bytes) numerator */
	put_unaligned_le32(n_num, r);
	r[4] = (n_num >> 32) & 0xff;
	r[5] = (n_num >> 40) & 0x0f;
	/* 32 bits denominator */
	put_unaligned_le32(n_den, &r[6]);

	/* Program the fraction */
	return regmap_bulk_write(regmap, reg, r, sizeof(r));
}

/* VCO, we assume it runs at a constant frequency */
static unsigned long si5391_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_si5391 *data = to_clk_si5391(hw);
	int err;
	u64 res;
	u64 m_num;
	u32 m_den;
	unsigned int shift;

	/* Assume that PDIV is not being used, just read the PLL setting */
	err = si5391_decode_44_32(data->regmap, SI5391_PLL_M_NUM,
				&m_num, &m_den);
	if (err < 0)
		return 0;

	if (!m_num || !m_den)
		return 0;

	/*
	 * Though m_num is 64-bit, only the upper bits are actually used. While
	 * calculating m_num and m_den, they are shifted as far as possible to
	 * the left. To avoid 96-bit division here, we just shift them back so
	 * we can do with just 64 bits.
	 */
	shift = 0;
	res = m_num;
	while (res & 0xffff00000000ULL) {
		++shift;
		res >>= 1;
	}
	res *= parent_rate;
	do_div(res, (m_den >> shift));

	/* We cannot return the actual frequency in 32 bit, store it locally */
	data->freq_vco = res;

	/* Report kHz since the value is out of range */
	do_div(res, 1000);

	return (unsigned long)res;
}

static int si5391_clk_get_selected_input(struct clk_si5391 *data)
{
	int err;
	u32 val;

	err = regmap_read(data->regmap, SI5391_IN_SEL, &val);
	if (err < 0)
		return err;

	return (val & SI5391_IN_SEL_MASK) >> SI5391_IN_SEL_SHIFT;
}

static u8 si5391_clk_get_parent(struct clk_hw *hw)
{
	struct clk_si5391 *data = to_clk_si5391(hw);
	int res = si5391_clk_get_selected_input(data);

	if (res < 0)
		return 0; /* Apparently we cannot report errors */

	return res;
}

static int si5391_clk_reparent(struct clk_si5391 *data, u8 index)
{
	int err;
	u8 val;

	val = (index << SI5391_IN_SEL_SHIFT) & SI5391_IN_SEL_MASK;
	/* Enable register-based input selection */
	val |= SI5391_IN_SEL_REGCTRL;

	err = regmap_update_bits(data->regmap,
		SI5391_IN_SEL, SI5391_IN_SEL_REGCTRL | SI5391_IN_SEL_MASK, val);
	if (err < 0)
		return err;

	if (index < 3) {
		/* Enable input buffer for selected input */
		err = regmap_update_bits(data->regmap,
				SI5391_IN_EN, 0x07, BIT(index));
		if (err < 0)
			return err;

		/* Enables the input to phase detector */
		err = regmap_update_bits(data->regmap, SI5391_INX_TO_PFD_EN,
				0x7 << SI5391_INX_TO_PFD_SHIFT,
				BIT(index + SI5391_INX_TO_PFD_SHIFT));
		if (err < 0)
			return err;

		/* Power down XTAL oscillator and buffer */
		err = regmap_update_bits(data->regmap, SI5391_XAXB_CFG,
				SI5391_XAXB_CFG_PDNB, 0);
		if (err < 0)
			return err;

		/*
		 * Set the P divider to "1". There's no explanation in the
		 * datasheet of these registers, but the clockbuilder software
		 * programs a "1" when the input is being used.
		 */
		err = regmap_write(data->regmap, SI5391_IN_PDIV(index), 1);
		if (err < 0)
			return err;

		err = regmap_write(data->regmap, SI5391_IN_PSET(index), 1);
		if (err < 0)
			return err;

		/* Set update PDIV bit */
		err = regmap_write(data->regmap, SI5391_PX_UPD, BIT(index));
		if (err < 0)
			return err;
	} else {
		/* Disable all input buffers */
		err = regmap_update_bits(data->regmap, SI5391_IN_EN, 0x07, 0);
		if (err < 0)
			return err;

		/* Disable input to phase detector */
		err = regmap_update_bits(data->regmap, SI5391_INX_TO_PFD_EN,
				0x7 << SI5391_INX_TO_PFD_SHIFT, 0);
		if (err < 0)
			return err;

		/* Power up XTAL oscillator and buffer, select clock mode */
		err = regmap_update_bits(data->regmap, SI5391_XAXB_CFG,
				SI5391_XAXB_CFG_PDNB | SI5391_XAXB_CFG_EXTCLK_EN,
				SI5391_XAXB_CFG_PDNB | (data->xaxb_ext_clk ?
					SI5391_XAXB_CFG_EXTCLK_EN : 0));
		if (err < 0)
			return err;
	}

	return 0;
}

static int si5391_clk_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_si5391 *data = to_clk_si5391(hw);

	return si5391_clk_reparent(data, index);
}

static const struct clk_ops si5391_clk_ops = {
	.determine_rate = clk_hw_determine_rate_no_reparent,
	.set_parent = si5391_clk_set_parent,
	.get_parent = si5391_clk_get_parent,
	.recalc_rate = si5391_clk_recalc_rate,
};

/* Synthesizers, there are 5 synthesizers that connect to any of the outputs */

/* The synthesizer is on if all power and enable bits are set */
static int si5391_synth_clk_is_on(struct clk_hw *hw)
{
	struct clk_si5391_synth *synth = to_clk_si5391_synth(hw);
	int err;
	u32 val;
	u8 index = synth->index;

	err = regmap_read(synth->data->regmap,
			SI5391_SYNTH_N_CLK_TO_OUTX_EN, &val);
	if (err < 0)
		return 0;

	if (!(val & BIT(index)))
		return 0;

	err = regmap_read(synth->data->regmap, SI5391_SYNTH_N_PDNB, &val);
	if (err < 0)
		return 0;

	if (!(val & BIT(index)))
		return 0;

	/* This bit must be 0 for the synthesizer to receive clock input */
	err = regmap_read(synth->data->regmap, SI5391_SYNTH_N_CLK_DIS, &val);
	if (err < 0)
		return 0;

	return !(val & BIT(index));
}

static void si5391_synth_clk_unprepare(struct clk_hw *hw)
{
	struct clk_si5391_synth *synth = to_clk_si5391_synth(hw);
	u8 index = synth->index; /* In range 0..5 */
	u8 mask = BIT(index);

	/* Disable output */
	regmap_update_bits(synth->data->regmap,
		SI5391_SYNTH_N_CLK_TO_OUTX_EN, mask, 0);
	/* Power down */
	regmap_update_bits(synth->data->regmap,
		SI5391_SYNTH_N_PDNB, mask, 0);
	/* Disable clock input to synth (set to 1 to disable) */
	regmap_update_bits(synth->data->regmap,
		SI5391_SYNTH_N_CLK_DIS, mask, mask);
}

static int si5391_synth_clk_prepare(struct clk_hw *hw)
{
	struct clk_si5391_synth *synth = to_clk_si5391_synth(hw);
	int err;
	u8 index = synth->index;
	u8 mask = BIT(index);

	/* Power up */
	err = regmap_update_bits(synth->data->regmap,
		SI5391_SYNTH_N_PDNB, mask, mask);
	if (err < 0)
		return err;

	/* Enable clock input to synth (set bit to 0 to enable) */
	err = regmap_update_bits(synth->data->regmap,
		SI5391_SYNTH_N_CLK_DIS, mask, 0);
	if (err < 0)
		return err;

	/* Enable output */
	return regmap_update_bits(synth->data->regmap,
		SI5391_SYNTH_N_CLK_TO_OUTX_EN, mask, mask);
}

/* Synth clock frequency: Fvco * n_den / n_den, with Fvco in 13500-14256 MHz */
static unsigned long si5391_synth_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_si5391_synth *synth = to_clk_si5391_synth(hw);
	u64 f;
	u64 n_num;
	u32 n_den;
	int err;

	err = si5391_decode_44_32(synth->data->regmap,
			SI5391_SYNTH_N_NUM(synth->index), &n_num, &n_den);
	if (err < 0)
		return err;
	/* Check for bogus/uninitialized settings */
	if (!n_num || !n_den)
		return 0;

	/*
	 * n_num and n_den are shifted left as much as possible, so to prevent
	 * overflow in 64-bit math, we shift n_den 4 bits to the right
	 */
	f = synth->data->freq_vco;
	f *= n_den >> 4;

	/* Now we need to do 64-bit division: f/n_num */
	/* And compensate for the 4 bits we dropped */
	f = div64_u64(f, (n_num >> 4));

	return f;
}

static long si5391_synth_clk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	struct clk_si5391_synth *synth = to_clk_si5391_synth(hw);
	u64 f;

	/* The synthesizer accuracy is such that anything in range will work */
	f = synth->data->freq_vco;
	do_div(f, SI5391_SYNTH_N_MAX);
	if (rate < f)
		return f;

	f = synth->data->freq_vco;
	do_div(f, SI5391_SYNTH_N_MIN);
	if (rate > f)
		return f;

	return rate;
}

static int si5391_synth_program(struct clk_si5391_synth *synth,
	u64 n_num, u32 n_den, bool is_integer)
{
	int err;
	u8 index = synth->index;

	err = si5391_encode_44_32(synth->data->regmap,
			SI5391_SYNTH_N_NUM(index), n_num, n_den);

	err = regmap_update_bits(synth->data->regmap,
		SI5391_SYNTH_N_PIBYP, BIT(index), is_integer ? BIT(index) : 0);
	if (err < 0)
		return err;

	return regmap_write(synth->data->regmap,
		SI5391_SYNTH_N_UPD(index), 0x01);
}


static int si5391_synth_clk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_si5391_synth *synth = to_clk_si5391_synth(hw);
	u64 n_num;
	u32 n_den;
	u32 r;
	u32 g;
	bool is_integer;

	n_num = synth->data->freq_vco;

	/* see if there's an integer solution */
	r = do_div(n_num, rate);
	is_integer = (r == 0);
	if (is_integer) {
		/* Integer divider equal to n_num */
		n_den = 1;
	} else {
		/* Calculate a fractional solution */
		g = gcd(r, rate);
		n_den = rate / g;
		n_num *= n_den;
		n_num += r / g;
	}

	dev_dbg(&synth->data->i2c_client->dev,
			"%s(%u): n=0x%llx d=0x%x %s\n", __func__,
				synth->index, n_num, n_den,
				is_integer ? "int" : "frac");

	return si5391_synth_program(synth, n_num, n_den, is_integer);
}

static const struct clk_ops si5391_synth_clk_ops = {
	.is_prepared = si5391_synth_clk_is_on,
	.prepare = si5391_synth_clk_prepare,
	.unprepare = si5391_synth_clk_unprepare,
	.recalc_rate = si5391_synth_clk_recalc_rate,
	.round_rate = si5391_synth_clk_round_rate,
	.set_rate = si5391_synth_clk_set_rate,
};

static int si5391_output_clk_is_on(struct clk_hw *hw)
{
	struct clk_si5391_output *output = to_clk_si5391_output(hw);
	int err;
	u32 val;

	err = regmap_read(output->data->regmap,
			SI5391_OUT_CONFIG(output), &val);
	if (err < 0)
		return err;

	/* Bit 0=PDN, 1=OE so only a value of 0x2 enables the output */
	return (val & 0x03) == SI5391_OUT_CFG_OE;
}

/* Disables and then powers down the output */
static void si5391_output_clk_unprepare(struct clk_hw *hw)
{
	struct clk_si5391_output *output = to_clk_si5391_output(hw);

	regmap_update_bits(output->data->regmap,
			SI5391_OUT_CONFIG(output),
			SI5391_OUT_CFG_OE, 0);
	regmap_update_bits(output->data->regmap,
			SI5391_OUT_CONFIG(output),
			SI5391_OUT_CFG_PDN, SI5391_OUT_CFG_PDN);
}

/* Powers up and then enables the output */
static int si5391_output_clk_prepare(struct clk_hw *hw)
{
	struct clk_si5391_output *output = to_clk_si5391_output(hw);
	int err;

	err = regmap_update_bits(output->data->regmap,
			SI5391_OUT_CONFIG(output),
			SI5391_OUT_CFG_PDN, 0);
	if (err < 0)
		return err;

	return regmap_update_bits(output->data->regmap,
			SI5391_OUT_CONFIG(output),
			SI5391_OUT_CFG_OE, SI5391_OUT_CFG_OE);
}

static unsigned long si5391_output_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_si5391_output *output = to_clk_si5391_output(hw);
	int err;
	u32 val;
	u32 r_divider;
	u8 r[3];

	err = regmap_read(output->data->regmap,
			SI5391_OUT_CONFIG(output), &val);
	if (err < 0)
		return err;

	/* If SI5391_OUT_CFG_RDIV_FORCE2 is set, r_divider is 2 */
	if (val & SI5391_OUT_CFG_RDIV_FORCE2)
		return parent_rate / 2;

	err = regmap_bulk_read(output->data->regmap,
			SI5391_OUT_R_REG(output), r, 3);
	if (err < 0)
		return err;

	/* Calculate value as 24-bit integer*/
	r_divider = r[2] << 16 | r[1] << 8 | r[0];

	/* If Rx_REG is zero, the divider is disabled, so return a "0" rate */
	if (!r_divider)
		return 0;

	/* Divider is 2*(Rx_REG+1) */
	r_divider += 1;
	r_divider <<= 1;


	return parent_rate / r_divider;
}

static int si5391_output_clk_determine_rate(struct clk_hw *hw,
					    struct clk_rate_request *req)
{
	unsigned long rate = req->rate;
	unsigned long r;

	if (!rate)
		return 0;

	r = req->best_parent_rate >> 1;

	/* If rate is an even divisor, no changes to parent required */
	if (r && !(r % rate))
		return 0;

	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		if (rate > 200000000) {
			/* minimum r-divider is 2 */
			r = 2;
		} else {
			/* Take a parent frequency near 400 MHz */
			r = (400000000u / rate) & ~1;
		}
		req->best_parent_rate = r * rate;
	} else {
		/* We cannot change our parent's rate, report what we can do */
		r /= rate;
		rate = req->best_parent_rate / (r << 1);
	}

	req->rate = rate;
	return 0;
}

static int si5391_output_clk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_si5391_output *output = to_clk_si5391_output(hw);
	u32 r_div;
	int err;
	u8 r[3];

	if (!rate)
		return -EINVAL;

	/* Frequency divider is (r_div + 1) * 2 */
	r_div = (parent_rate / rate) >> 1;

	if (r_div <= 1)
		r_div = 0;
	else if (r_div >= BIT(24))
		r_div = BIT(24) - 1;
	else
		--r_div;

	/* For a value of "2", we set the "OUT0_RDIV_FORCE2" bit */
	err = regmap_update_bits(output->data->regmap,
			SI5391_OUT_CONFIG(output),
			SI5391_OUT_CFG_RDIV_FORCE2,
			(r_div == 0) ? SI5391_OUT_CFG_RDIV_FORCE2 : 0);
	if (err < 0)
		return err;

	/* Always write Rx_REG, because a zero value disables the divider */
	r[0] = r_div ? (r_div & 0xff) : 1;
	r[1] = (r_div >> 8) & 0xff;
	r[2] = (r_div >> 16) & 0xff;
	return regmap_bulk_write(output->data->regmap,
			SI5391_OUT_R_REG(output), r, 3);
}

static int si5391_output_reparent(struct clk_si5391_output *output, u8 index)
{
	return regmap_update_bits(output->data->regmap,
		SI5391_OUT_MUX_SEL(output), 0x07, index);
}

static int si5391_output_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_si5391_output *output = to_clk_si5391_output(hw);

	if (index >= output->data->num_synth)
		return -EINVAL;

	return si5391_output_reparent(output, index);
}

static u8 si5391_output_get_parent(struct clk_hw *hw)
{
	struct clk_si5391_output *output = to_clk_si5391_output(hw);
	u32 val;

	regmap_read(output->data->regmap, SI5391_OUT_MUX_SEL(output), &val);

	return val & 0x7;
}

static const struct clk_ops si5391_output_clk_ops = {
	.is_prepared = si5391_output_clk_is_on,
	.prepare = si5391_output_clk_prepare,
	.unprepare = si5391_output_clk_unprepare,
	.recalc_rate = si5391_output_clk_recalc_rate,
	.determine_rate = si5391_output_clk_determine_rate,
	.set_rate = si5391_output_clk_set_rate,
	.set_parent = si5391_output_set_parent,
	.get_parent = si5391_output_get_parent,
};

/*
 * The chip can be bought in a pre-programmed version, or one can program the
 * NVM in the chip to boot up in a preset mode. This routine tries to determine
 * if that's the case, or if we need to reset and program everything from
 * scratch. Returns negative error, or true/false.
 */
static int si5391_is_programmed_already(struct clk_si5391 *data)
{
	int err;
	u8 r[4];

	/* Read the PLL divider value, it must have a non-zero value */
	err = regmap_bulk_read(data->regmap, SI5391_PLL_M_DEN,
			r, ARRAY_SIZE(r));
	if (err < 0)
		return err;

	return !!get_unaligned_le32(r);
}

static struct clk_hw *
of_clk_si5391_get(struct of_phandle_args *clkspec, void *_data)
{
	struct clk_si5391 *data = _data;
	unsigned int idx = clkspec->args[1];
	unsigned int group = clkspec->args[0];

	switch (group) {
	case 0:
		if (idx >= data->num_outputs) {
			dev_err(&data->i2c_client->dev,
				"invalid output index %u\n", idx);
			return ERR_PTR(-EINVAL);
		}
		return &data->clk[idx].hw;
	case 1:
		if (idx >= data->num_synth) {
			dev_err(&data->i2c_client->dev,
				"invalid synthesizer index %u\n", idx);
			return ERR_PTR(-EINVAL);
		}
		return &data->synth[idx].hw;
	case 2:
		if (idx > 0) {
			dev_err(&data->i2c_client->dev,
				"invalid PLL index %u\n", idx);
			return ERR_PTR(-EINVAL);
		}
		return &data->hw;
	default:
		dev_err(&data->i2c_client->dev, "invalid group %u\n", group);
		return ERR_PTR(-EINVAL);
	}
}

static int si5391_probe_chip_id(struct clk_si5391 *data)
{
	pr_err("\n si5391: %s: probing chip id\n", __FUNCTION__);
	int err;
	u8 reg[4];
	u16 model;

	err = regmap_bulk_read(data->regmap, SI5391_PN_BASE, reg,
				ARRAY_SIZE(reg));
	if (err < 0) {
		dev_err(&data->i2c_client->dev, "Failed to read chip ID\n");
		return err;
	}

	model = get_unaligned_le16(reg);
	pr_err("\n si5391: %s: 2: model=%x\n", __FUNCTION__, model);

	dev_info(&data->i2c_client->dev, "Chip: %x Grade: %u Rev: %u\n",
		 model, reg[2], reg[3]);

	//case 0x5341:
	//	data->num_outputs = SI5341_MAX_NUM_OUTPUTS;
	//	data->num_synth = SI5341_NUM_SYNTH;
	//	data->reg_output_offset = si5341_reg_output_offset;
	//	data->reg_rdiv_offset = si5341_reg_rdiv_offset;
	//	pr_err("\n si5341: %s: 3: max num outputs=%d, num synth=%d, reg output offset=%d. reg rdiv offset=%d\n",
	//			__FUNCTION__, data->num_outputs, data->num_synth, data->reg_output_offset, data->reg_rdiv_offset);
	//	break;
	switch (model) {
	case 0x5391:
		data->num_outputs = SI5391_MAX_NUM_OUTPUTS;
		data->num_synth = SI5391_NUM_SYNTH;
		data->reg_output_offset = si5391_reg_output_offset;
		data->reg_rdiv_offset = si5391_reg_rdiv_offset;
		pr_err("\n si5391: %s: max num outputs=%d, num synth=%d, reg output offset[9]=%d (OUT8). reg rdiv offset[9]=%d (OUT8)\n",
				__FUNCTION__, data->num_outputs, data->num_synth, data->reg_output_offset[9], data->reg_rdiv_offset[9]);
		break;
	default:
		dev_err(&data->i2c_client->dev, "Model '%x' not supported\n",
			model);
		return -EINVAL;
	}

	data->chip_id = model;

	return 0;
}

/* Read active settings into the regmap cache for later reference */
static int si5391_read_settings(struct clk_si5391 *data)
{
	int err;
	u8 i;
	u8 r[10];

	err = regmap_bulk_read(data->regmap, SI5391_PLL_M_NUM, r, 10);
	if (err < 0)
		return err;

	err = regmap_bulk_read(data->regmap,
				SI5391_SYNTH_N_CLK_TO_OUTX_EN, r, 3);
	if (err < 0)
		return err;

	err = regmap_bulk_read(data->regmap,
				SI5391_SYNTH_N_CLK_DIS, r, 1);
	if (err < 0)
		return err;

	for (i = 0; i < data->num_synth; ++i) {
		err = regmap_bulk_read(data->regmap,
					SI5391_SYNTH_N_NUM(i), r, 10);
		if (err < 0)
			return err;
	}

	for (i = 0; i < data->num_outputs; ++i) {
		err = regmap_bulk_read(data->regmap,
					data->reg_output_offset[i], r, 4);
		if (err < 0)
			return err;

		err = regmap_bulk_read(data->regmap,
					data->reg_rdiv_offset[i], r, 3);
		if (err < 0)
			return err;
	}

	return 0;
}

static int si5391_write_multiple(struct clk_si5391 *data,
	const struct si5391_reg_default *values, unsigned int num_values)
{
	unsigned int i;
	int res;

	for (i = 0; i < num_values; ++i) {
		res = regmap_write(data->regmap,
			values[i].address, values[i].value);
		if (res < 0) {
			dev_err(&data->i2c_client->dev,
				"Failed to write %#x:%#x\n",
				values[i].address, values[i].value);
			return res;
		}
	}

	return 0;
}

static const struct si5391_reg_default si5391_preamble[] = {
	{ 0x0B25, 0x00 },
	{ 0x0502, 0x01 },
	{ 0x0505, 0x03 },
	{ 0x0957, 0x17 },
	{ 0x0B4E, 0x1A },
};

static int si5391_send_preamble(struct clk_si5391 *data)
{
	int res;
	u32 revision;

	/* For revision 2 and up, the values are slightly different */
	res = regmap_read(data->regmap, SI5391_DEVICE_REV, &revision);
	if (res < 0)
		return res;

	/* Write "preamble" as specified by datasheet */
	res = regmap_write(data->regmap, 0xB24, revision < 2 ? 0xD8 : 0xC0);
	if (res < 0)
		return res;

	/* The si5342..si5345 require a different preamble */
	if (data->chip_id == 0x5391 || data->chip_id == 0x5391)
		res = si5391_write_multiple(data,
			si5391_preamble, ARRAY_SIZE(si5391_preamble));
	if (res < 0)
		return res;

	/* Datasheet specifies a 300ms wait after sending the preamble */
	msleep(300);

	return 0;
}

/* Perform a soft reset and write post-amble */
static int si5391_finalize_defaults(struct clk_si5391 *data)
{
	int res;
	u32 revision;

	res = regmap_write(data->regmap, SI5391_IO_VDD_SEL,
			   data->iovdd_33 ? 1 : 0);
	if (res < 0)
		return res;

	res = regmap_read(data->regmap, SI5391_DEVICE_REV, &revision);
	if (res < 0)
		return res;

	dev_dbg(&data->i2c_client->dev, "%s rev=%u\n", __func__, revision);

	res = regmap_write(data->regmap, SI5391_SOFT_RST, 0x01);
	if (res < 0)
		return res;

	/* Datasheet does not explain these nameless registers */
	res = regmap_write(data->regmap, 0xB24, revision < 2 ? 0xDB : 0xC3);
	if (res < 0)
		return res;
	res = regmap_write(data->regmap, 0x0B25, 0x02);
	if (res < 0)
		return res;

	return 0;
}


static const struct regmap_range si5391_regmap_volatile_range[] = {
	regmap_reg_range(0x000C, 0x0012), /* Status */
	regmap_reg_range(0x001C, 0x001E), /* reset, finc/fdec */
	regmap_reg_range(0x00E2, 0x00FE), /* NVM, interrupts, device ready */
	/* Update bits for P divider and synth config */
	regmap_reg_range(SI5391_PX_UPD, SI5391_PX_UPD),
	regmap_reg_range(SI5391_SYNTH_N_UPD(0), SI5391_SYNTH_N_UPD(0)),
	regmap_reg_range(SI5391_SYNTH_N_UPD(1), SI5391_SYNTH_N_UPD(1)),
	regmap_reg_range(SI5391_SYNTH_N_UPD(2), SI5391_SYNTH_N_UPD(2)),
	regmap_reg_range(SI5391_SYNTH_N_UPD(3), SI5391_SYNTH_N_UPD(3)),
	regmap_reg_range(SI5391_SYNTH_N_UPD(4), SI5391_SYNTH_N_UPD(4)),
};

static const struct regmap_access_table si5391_regmap_volatile = {
	.yes_ranges = si5391_regmap_volatile_range,
	.n_yes_ranges = ARRAY_SIZE(si5391_regmap_volatile_range),
};

/* Pages 0, 1, 2, 3, 9, A, B are valid, so there are 12 pages */
static const struct regmap_range_cfg si5391_regmap_ranges[] = {
	{
		.range_min = 0,
		.range_max = SI5391_REGISTER_MAX,
		.selector_reg = SI5391_PAGE,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 256,
	},
};

static int si5391_wait_device_ready(struct i2c_client *client)
{
	int count;

	/* Datasheet warns: Any attempt to read or write any register other
	 * than DEVICE_READY before DEVICE_READY reads as 0x0F may corrupt the
	 * NVM programming and may corrupt the register contents, as they are
	 * read from NVM. Note that this includes accesses to the PAGE register.
	 * Also: DEVICE_READY is available on every register page, so no page
	 * change is needed to read it.
	 * Do this outside regmap to avoid automatic PAGE register access.
	 * May take up to 300ms to complete.
	 */
	for (count = 0; count < 15; ++count) {
		s32 result = i2c_smbus_read_byte_data(client,
						      SI5391_DEVICE_READY);
		if (result < 0)
			return result;
		if (result == 0x0F)
			return 0;
		msleep(20);
	}
	dev_err(&client->dev, "timeout waiting for DEVICE_READY\n");
	return -EIO;
}

static const struct regmap_config si5391_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_MAPLE,
	.ranges = si5391_regmap_ranges,
	.num_ranges = ARRAY_SIZE(si5391_regmap_ranges),
	.max_register = SI5391_REGISTER_MAX,
	.volatile_table = &si5391_regmap_volatile,
};

static int si5391_dt_parse_dt(struct clk_si5391 *data,
			      struct clk_si5391_output_config *config)
{
	struct device_node *child;
	struct device_node *np = data->i2c_client->dev.of_node;
	u32 num;
	u32 val;

	memset(config, 0, sizeof(struct clk_si5391_output_config) *
				SI5391_MAX_NUM_OUTPUTS);

	for_each_child_of_node(np, child) {
		if (of_property_read_u32(child, "reg", &num)) {
			dev_err(&data->i2c_client->dev, "missing reg property of %s\n",
				child->name);
			goto put_child;
		}

		if (num >= SI5391_MAX_NUM_OUTPUTS) {
			dev_err(&data->i2c_client->dev, "invalid clkout %d\n", num);
			goto put_child;
		}

		if (!of_property_read_u32(child, "silabs,format", &val)) {
			/* Set cm and ampl conservatively to 3v3 settings */
			switch (val) {
			case 1: /* normal differential */
				config[num].out_cm_ampl_bits = 0x33;
				break;
			case 2: /* low-power differential */
				config[num].out_cm_ampl_bits = 0x13;
				break;
			case 4: /* LVCMOS */
				config[num].out_cm_ampl_bits = 0x33;
				/* Set SI recommended impedance for LVCMOS */
				config[num].out_format_drv_bits |= 0xc0;
				break;
			default:
				dev_err(&data->i2c_client->dev,
					"invalid silabs,format %u for %u\n",
					val, num);
				goto put_child;
			}
			config[num].out_format_drv_bits &= ~0x07;
			config[num].out_format_drv_bits |= val & 0x07;
			/* Always enable the SYNC feature */
			config[num].out_format_drv_bits |= 0x08;
		}

		if (!of_property_read_u32(child, "silabs,common-mode", &val)) {
			if (val > 0xf) {
				dev_err(&data->i2c_client->dev,
					"invalid silabs,common-mode %u\n",
					val);
				goto put_child;
			}
			config[num].out_cm_ampl_bits &= 0xf0;
			config[num].out_cm_ampl_bits |= val & 0x0f;
		}

		if (!of_property_read_u32(child, "silabs,amplitude", &val)) {
			if (val > 0xf) {
				dev_err(&data->i2c_client->dev,
					"invalid silabs,amplitude %u\n",
					val);
				goto put_child;
			}
			config[num].out_cm_ampl_bits &= 0x0f;
			config[num].out_cm_ampl_bits |= (val << 4) & 0xf0;
		}

		if (of_property_read_bool(child, "silabs,disable-high"))
			config[num].out_format_drv_bits |= 0x10;

		config[num].synth_master =
			of_property_read_bool(child, "silabs,synth-master");

		config[num].always_on =
			of_property_read_bool(child, "always-on");

		config[num].vdd_sel_bits = 0x08;
		if (data->clk[num].vddo_reg) {
			int vdd = regulator_get_voltage(data->clk[num].vddo_reg);

			switch (vdd) {
			case 3300000:
				config[num].vdd_sel_bits |= 0 << 4;
				break;
			case 1800000:
				config[num].vdd_sel_bits |= 1 << 4;
				break;
			case 2500000:
				config[num].vdd_sel_bits |= 2 << 4;
				break;
			default:
				dev_err(&data->i2c_client->dev,
					"unsupported vddo voltage %d for %s\n",
					vdd, child->name);
				goto put_child;
			}
		} else {
			/* chip seems to default to 2.5V when not set */
			dev_warn(&data->i2c_client->dev,
				"no regulator set, defaulting vdd_sel to 2.5V for %s\n",
				child->name);
			config[num].vdd_sel_bits |= 2 << 4;
		}
	}

	return 0;

put_child:
	of_node_put(child);
	return -EINVAL;
}

/*
 * If not pre-configured, calculate and set the PLL configuration manually.
 * For low-jitter performance, the PLL should be set such that the synthesizers
 * only need integer division.
 * Without any user guidance, we'll set the PLL to 14GHz, which still allows
 * the chip to generate any frequency on its outputs, but jitter performance
 * may be sub-optimal.
 */
static int si5391_initialize_pll(struct clk_si5391 *data)
{
	struct device_node *np = data->i2c_client->dev.of_node;
	u32 m_num = 0;
	u32 m_den = 0;
	int sel;

	if (of_property_read_u32(np, "silabs,pll-m-num", &m_num)) {
		dev_err(&data->i2c_client->dev,
			"PLL configuration requires silabs,pll-m-num\n");
	}
	if (of_property_read_u32(np, "silabs,pll-m-den", &m_den)) {
		dev_err(&data->i2c_client->dev,
			"PLL configuration requires silabs,pll-m-den\n");
	}

	if (!m_num || !m_den) {
		dev_err(&data->i2c_client->dev,
			"PLL configuration invalid, assume 14GHz\n");
		sel = si5391_clk_get_selected_input(data);
		if (sel < 0)
			return sel;

		m_den = clk_get_rate(data->input_clk[sel]) / 10;
		m_num = 1400000000;
	}
	pr_err("\n si5391: %s: %d: m_num=%d, m_den=%d\n", __FUNCTION__, __LINE__, m_num, m_den);

	return si5391_encode_44_32(data->regmap,
			SI5391_PLL_M_NUM, m_num, m_den);
}

static int si5391_clk_select_active_input(struct clk_si5391 *data)
{
	int res;
	int err;
	int i;

	res = si5391_clk_get_selected_input(data);
	if (res < 0)
		return res;

	/* If the current register setting is invalid, pick the first input */
	if (!data->input_clk[res]) {
		dev_dbg(&data->i2c_client->dev,
			"Input %d not connected, rerouting\n", res);
		res = -ENODEV;
		for (i = 0; i < SI5391_NUM_INPUTS; ++i) {
			if (data->input_clk[i]) {
				res = i;
				break;
			}
		}
		if (res < 0) {
			dev_err(&data->i2c_client->dev,
				"No clock input available\n");
			return res;
		}
	}

	/* Make sure the selected clock is also enabled and routed */
	err = si5391_clk_reparent(data, res);
	if (err < 0)
		return err;

	err = clk_prepare_enable(data->input_clk[res]);
	if (err < 0)
		return err;

	return res;
}

static ssize_t input_present_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct clk_si5391 *data = dev_get_drvdata(dev);
	u32 status;
	int res = regmap_read(data->regmap, SI5391_STATUS, &status);

	if (res < 0)
		return res;
	res = !(status & SI5391_STATUS_LOSREF);
	return sysfs_emit(buf, "%d\n", res);
}
static DEVICE_ATTR_RO(input_present);

static ssize_t input_present_sticky_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct clk_si5391 *data = dev_get_drvdata(dev);
	u32 status;
	int res = regmap_read(data->regmap, SI5391_STATUS_STICKY, &status);

	if (res < 0)
		return res;
	res = !(status & SI5391_STATUS_LOSREF);
	return sysfs_emit(buf, "%d\n", res);
}
static DEVICE_ATTR_RO(input_present_sticky);

static ssize_t pll_locked_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct clk_si5391 *data = dev_get_drvdata(dev);
	u32 status;
	int res = regmap_read(data->regmap, SI5391_STATUS, &status);

	if (res < 0)
		return res;
	res = !(status & SI5391_STATUS_LOL);
	return sysfs_emit(buf, "%d\n", res);
}
static DEVICE_ATTR_RO(pll_locked);

static ssize_t pll_locked_sticky_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct clk_si5391 *data = dev_get_drvdata(dev);
	u32 status;
	int res = regmap_read(data->regmap, SI5391_STATUS_STICKY, &status);

	if (res < 0)
		return res;
	res = !(status & SI5391_STATUS_LOL);
	return sysfs_emit(buf, "%d\n", res);
}
static DEVICE_ATTR_RO(pll_locked_sticky);

static ssize_t clear_sticky_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct clk_si5391 *data = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;
	if (val) {
		int res = regmap_write(data->regmap, SI5391_STATUS_STICKY, 0);

		if (res < 0)
			return res;
	}
	return count;
}
static DEVICE_ATTR_WO(clear_sticky);

static const struct attribute *si5391_attributes[] = {
	&dev_attr_input_present.attr,
	&dev_attr_input_present_sticky.attr,
	&dev_attr_pll_locked.attr,
	&dev_attr_pll_locked_sticky.attr,
	&dev_attr_clear_sticky.attr,
	NULL
};

static int si5391_probe(struct i2c_client *client)
{
	pr_err("\n si5391: %s: entering probe\n", __FUNCTION__);
	struct clk_si5391 *data;
	struct clk_init_data init;
	struct clk *input;
	const char *root_clock_name;
	const char *synth_clock_names[SI5391_NUM_SYNTH] = { NULL };
	int err;
	unsigned int i;
	struct clk_si5391_output_config config[SI5391_MAX_NUM_OUTPUTS];
	bool initialization_required;
	u32 status;

	pr_err("\n si5391: %s: %d: before allocating\n", __FUNCTION__, __LINE__);
	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->i2c_client = client;

	pr_err("\n si5391: %s: %d: before waiting device to be ready\n", __FUNCTION__, __LINE__);
	/* Must be done before otherwise touching hardware */
	err = si5391_wait_device_ready(client);
	if (err)
		return err;

	pr_err("\n si5391: %s: %d: after device ready (%d)\n", __FUNCTION__, __LINE__, err);
	for (i = 0; i < SI5391_NUM_INPUTS; ++i) {
		input = devm_clk_get(&client->dev, si5391_input_clock_names[i]);
		pr_err("\n si5391: %s: %d: num input i=%d\n", __FUNCTION__, __LINE__, i);
		if (IS_ERR(input)) {
			if (PTR_ERR(input) == -EPROBE_DEFER) {
				pr_err("\n si5391: %s: %d: num input i=%d probe defer\n", __FUNCTION__, __LINE__, i);
				return -EPROBE_DEFER;
			}
			data->input_clk_name[i] = si5391_input_clock_names[i];
			pr_err("\n si5391: %s: %d: num input i=%d ERROR input clk name=%s\n", __FUNCTION__, __LINE__, i, data->input_clk_name[i]);
		} else {
			data->input_clk[i] = input;
			data->input_clk_name[i] = __clk_get_name(input);
			pr_err("\n si5391: %s: %d: num input i=%d input clk name=%s\n", __FUNCTION__, __LINE__, i, data->input_clk_name[i]);
		}
	}

	for (i = 0; i < SI5391_MAX_NUM_OUTPUTS; ++i) {
		char reg_name[10];
		pr_err("\n si5391: %s: %d: num output i=%d\n", __FUNCTION__, __LINE__, i);

		snprintf(reg_name, sizeof(reg_name), "vddo%d", i);
		data->clk[i].vddo_reg = devm_regulator_get_optional(
			&client->dev, reg_name);
		pr_err("\n si5391: %s: %d: num output i=%d reg_name=%s\n", __FUNCTION__, __LINE__, i, reg_name);
		if (IS_ERR(data->clk[i].vddo_reg)) {
			err = PTR_ERR(data->clk[i].vddo_reg);
			data->clk[i].vddo_reg = NULL;
			pr_err("\n si5391: %s: %d: num output i=%d reg_name=%s ERROR\n", __FUNCTION__, __LINE__, i, reg_name);
			if (err == -ENODEV)
				continue;
			goto cleanup;
		} else {
			err = regulator_enable(data->clk[i].vddo_reg);
			pr_err("\n si5391: %s: %d: num output i=%d reg_name=%s regulator enable\n", __FUNCTION__, __LINE__, i, reg_name);
			if (err) {
				dev_err(&client->dev,
					"si5391: failed to enable %s regulator: %d\n",
					reg_name, err);
				data->clk[i].vddo_reg = NULL;
				goto cleanup;
			}
		}
	}

	err = si5391_dt_parse_dt(data, config);
	if (err) {
		pr_err("\n si5391: %s: %d: parse dt error\n", __FUNCTION__, __LINE__);
		goto cleanup;
	}
	pr_err("\n si5391: %s: %d: parse dt ok\n", __FUNCTION__, __LINE__);

	if (of_property_read_string(client->dev.of_node, "clock-output-names",
			&init.name)) {
		init.name = client->dev.of_node->name;
		pr_err("\n si5391: %s: %d: read OK clock-output-names, init.name=%s\n", __FUNCTION__, __LINE__, init.name);
	}
	root_clock_name = init.name;
	pr_err("\n si5391: %s: %d: root_clock_name=%s\n", __FUNCTION__, __LINE__, root_clock_name);

	data->regmap = devm_regmap_init_i2c(client, &si5391_regmap_config);
	if (IS_ERR(data->regmap)) {
		pr_err("\n si5391: %s: %d: root_clock_name=%s\n", __FUNCTION__, __LINE__, root_clock_name);
		err = PTR_ERR(data->regmap);
		goto cleanup;
	}

	pr_err("\n si5391: %s: %d: before i2c_set_clientdata\n", __FUNCTION__, __LINE__);
	i2c_set_clientdata(client, data);
	pr_err("\n si5391: %s: %d: after i2c_set_clientdata\n", __FUNCTION__, __LINE__);

	err = si5391_probe_chip_id(data);
	pr_err("\n si5391: %s: %d: after probe chip id\n", __FUNCTION__, __LINE__);
	if (err < 0)
		goto cleanup;

	if (of_property_read_bool(client->dev.of_node, "silabs,reprogram")) {
		initialization_required = true;
		pr_err("\n si5391: %s: %d: initialization required\n", __FUNCTION__, __LINE__);
	} else {
		err = si5391_is_programmed_already(data);
		if (err < 0) {
			pr_err("\n si5391: %s: %d: not reprogrammed\n", __FUNCTION__, __LINE__);
			goto cleanup;
		}
		pr_err("\n si5391: %s: %d: reprogrammed\n", __FUNCTION__, __LINE__);

		initialization_required = !err;
	}
	data->xaxb_ext_clk = of_property_read_bool(client->dev.of_node,
						   "silabs,xaxb-ext-clk");
	pr_err("\n si5391: %s: %d: set xaxb-ext-clk=%d\n", __FUNCTION__, __LINE__, data->xaxb_ext_clk);
	data->iovdd_33 = of_property_read_bool(client->dev.of_node,
					       "silabs,iovdd-33");
	pr_err("\n si5391: %s: %d: set iovdd-33=%d\n", __FUNCTION__, __LINE__, data->iovdd_33);

	if (initialization_required) {
		pr_err("\n si5391: %s: %d: entered required initialization\n", __FUNCTION__, __LINE__);
		/* Populate the regmap cache in preparation for "cache only" */
		err = si5391_read_settings(data);
		if (err < 0)
			goto cleanup;

		err = si5391_send_preamble(data);
		if (err < 0)
			goto cleanup;

		/*
		 * We intend to send all 'final' register values in a single
		 * transaction. So cache all register writes until we're done
		 * configuring.
		 */
		regcache_cache_only(data->regmap, true);

		/* Write the configuration pairs from the firmware blob */
		err = si5391_write_multiple(data, si5391_reg_defaults,
					ARRAY_SIZE(si5391_reg_defaults));
		if (err < 0)
			goto cleanup;
	}

	/* Input must be up and running at this point */
	err = si5391_clk_select_active_input(data);
	if (err < 0) {
		pr_err("\n si5391: %s: %d: ERROR clk select active input\n", __FUNCTION__, __LINE__);
		goto cleanup;
	}

	if (initialization_required) {
		/* PLL configuration is required */
		//err = si5391_initialize_pll(data);
		//if (err < 0) {
		//	pr_err("\n si5391: %s: %d: ERROR clk select active input\n", __FUNCTION__, __LINE__);
		//	goto cleanup;
		//}
		pr_err("\n si5391: %s: %d: using default pll configuration with values calculated by ClockBuilder Pro\n", __FUNCTION__, __LINE__);
	}

	/* Register the PLL */
	pr_err("\n si5391: %s: %d: init.name=%s\n", __FUNCTION__, __LINE__, init.name);
	init.parent_names = data->input_clk_name;
	init.num_parents = SI5391_NUM_INPUTS;
	init.ops = &si5391_clk_ops;
	init.flags = 0;
	data->hw.init = &init;

	err = devm_clk_hw_register(&client->dev, &data->hw);
	if (err) {
		dev_err(&client->dev, "clock registration failed\n");
		goto cleanup;
	}
	pr_err("\n si5391: %s: %d: clock registration done\n", __FUNCTION__, __LINE__);

	/* Clear init pointer after registration - critical to avoid conflicts */
	data->hw.init = NULL;

	init.num_parents = 1;
	init.parent_names = &root_clock_name;
	init.ops = &si5391_synth_clk_ops;
	init.flags = 0;
	pr_err("\n si5391: %s: %d: data->num_synth=%d\n", __FUNCTION__, __LINE__, data->num_synth);
	for (i = 0; i < data->num_synth; ++i) {
		synth_clock_names[i] = devm_kasprintf(&client->dev, GFP_KERNEL,
				"%s_N%u", client->dev.of_node->name, i);
		if (!synth_clock_names[i]) {
			err = -ENOMEM;
			goto free_clk_names;
		}
		pr_err("\n si5391: %s: %d: synth[%d] name='%s', parent='%s', node_name='%s'\n",
			__FUNCTION__, __LINE__, i, synth_clock_names[i], root_clock_name, client->dev.of_node->name);
		init.name = synth_clock_names[i];
		data->synth[i].index = i;
		data->synth[i].data = data;
		data->synth[i].hw.init = &init;
		pr_err("\n si5391: %s: %d: About to register synth[%d], init.name=%s, hw=%p\n",
			__FUNCTION__, __LINE__, i, init.name, &data->synth[i].hw);
		err = devm_clk_hw_register(&client->dev, &data->synth[i].hw);
		if (err) {
			dev_err(&client->dev,
				"synth N%u registration failed\n", i);
			goto free_clk_names;
		}
		/* Clear init pointer after registration */
		data->synth[i].hw.init = NULL;
		pr_err("\n si5391: %s: %d: input clock registration done i=%d\n", __FUNCTION__, __LINE__, i);
	}

	init.num_parents = data->num_synth;
	init.parent_names = synth_clock_names;
	init.ops = &si5391_output_clk_ops;
	pr_err("\n si5391: %s: %d: data->num_outputs=%d\n", __FUNCTION__, __LINE__, data->num_outputs);
	for (i = 0; i < data->num_outputs; ++i) {
		init.name = kasprintf(GFP_KERNEL, "%s_out%d",
			client->dev.of_node->name, i);
		if (!init.name) {
			err = -ENOMEM;
			goto free_clk_names;
		}
		init.flags = config[i].synth_master ? CLK_SET_RATE_PARENT : 0;
		data->clk[i].index = i;
		data->clk[i].data = data;
		data->clk[i].hw.init = &init;
		if (config[i].out_format_drv_bits & 0x07) {
			regmap_write(data->regmap,
				SI5391_OUT_FORMAT(&data->clk[i]),
				config[i].out_format_drv_bits);
			regmap_write(data->regmap,
				SI5391_OUT_CM(&data->clk[i]),
				config[i].out_cm_ampl_bits);
			regmap_update_bits(data->regmap,
				SI5391_OUT_MUX_SEL(&data->clk[i]),
				SI5391_OUT_MUX_VDD_SEL_MASK,
				config[i].vdd_sel_bits);
		}
		err = devm_clk_hw_register(&client->dev, &data->clk[i].hw);
		kfree(init.name); /* clock framework made a copy of the name */
		if (err) {
			dev_err(&client->dev,
				"output %u registration failed\n", i);
			goto free_clk_names;
		}
		/* Clear init pointer after registration */
		data->clk[i].hw.init = NULL;
		if (config[i].always_on)
			clk_prepare(data->clk[i].hw.clk);
		pr_err("\n si5391: %s: %d: output clock registration done i=%d\n", __FUNCTION__, __LINE__, i);
	}

	err = devm_of_clk_add_hw_provider(&client->dev, of_clk_si5391_get,
			data);
	if (err) {
		dev_err(&client->dev, "unable to add clk provider\n");
		goto free_clk_names;
	}
	pr_err("\n si5391: %s: %d: added clock provider\n", __FUNCTION__, __LINE__);

	if (initialization_required) {
		/* Synchronize */
		regcache_cache_only(data->regmap, false);
		err = regcache_sync(data->regmap);
		if (err < 0)
			goto free_clk_names;

		err = si5391_finalize_defaults(data);
		if (err < 0)
			goto free_clk_names;
		pr_err("\n si5391: %s: %d: initialization sync done\n", __FUNCTION__, __LINE__);
	}

	/* wait for device to report input clock present and PLL lock */
	err = regmap_read_poll_timeout(data->regmap, SI5391_STATUS, status,
		!(status & (SI5391_STATUS_LOSREF | SI5391_STATUS_LOL)),
	       10000, 250000);
	if (err) {
		pr_err("\n si5391: %s: %d: error waiting for input clock or pll lock\n", __FUNCTION__, __LINE__);
		dev_err(&client->dev, "Error waiting for input clock or PLL lock\n");
		goto free_clk_names;
	}
	pr_err("\n si5391: %s: %d: waiting done\n", __FUNCTION__, __LINE__);

	/* clear sticky alarm bits from initialization */
	err = regmap_write(data->regmap, SI5391_STATUS_STICKY, 0);
	if (err) {
		pr_err("\n si5391: %s: %d: error unable to clear sticky status\n", __FUNCTION__, __LINE__);
		dev_err(&client->dev, "unable to clear sticky status\n");
		goto free_clk_names;
	}
	pr_err("\n si5391: %s: %d: cleared sticky status\n", __FUNCTION__, __LINE__);

	err = sysfs_create_files(&client->dev.kobj, si5391_attributes);
	if (err)
		dev_err(&client->dev, "unable to create sysfs files\n");

free_clk_names:
	/* Free the names, clk framework makes copies */
	for (i = 0; i < data->num_synth; ++i)
		 devm_kfree(&client->dev, (void *)synth_clock_names[i]);

cleanup:
	if (err) {
		for (i = 0; i < SI5391_MAX_NUM_OUTPUTS; ++i) {
			if (data->clk[i].vddo_reg)
				regulator_disable(data->clk[i].vddo_reg);
		}
	}
	return err;
}

static void si5391_remove(struct i2c_client *client)
{
	struct clk_si5391 *data = i2c_get_clientdata(client);
	int i;

	sysfs_remove_files(&client->dev.kobj, si5391_attributes);

	for (i = 0; i < SI5391_MAX_NUM_OUTPUTS; ++i) {
		if (data->clk[i].vddo_reg)
			regulator_disable(data->clk[i].vddo_reg);
	}
}

static const struct i2c_device_id si5391_id[] = {
	{ "si5391", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, si5391_id);

static const struct of_device_id clk_si5391_of_match[] = {
	{ .compatible = "silabs,si5391" },
	{ }
};
MODULE_DEVICE_TABLE(of, clk_si5391_of_match);

static struct i2c_driver si5391_driver = {
	.driver = {
		.name = "si5391",
		.of_match_table = clk_si5391_of_match,
	},
	.probe		= si5391_probe,
	.remove		= si5391_remove,
	.id_table	= si5391_id,
};
module_i2c_driver(si5391_driver);

MODULE_AUTHOR("Mike Looijmans <mike.looijmans@topic.nl>");
MODULE_DESCRIPTION("Si5391 driver");
MODULE_LICENSE("GPL");
