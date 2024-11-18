/*
 * Copyright 2018-2023 NXP
 * Copyright 2018 INPHI
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Inphi is a registered trademark of Inphi Corporation
 *
 */

#include <linux/delay.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include "phy-inphi-in112525.h"

#define IN112525_CH_NUM		2
#define IN112525_LANE_NUM	4
#define IN112525_LANE_PER_CH	2
#define ALL_LANES		4

struct in112525_priv;

struct in112525_ch {
	struct phy *phy;
	struct in112525_priv *priv;
	int lane[IN112525_LANE_PER_CH];
	int id;
};

struct in112525_priv {
	struct mdio_device *mdiodev;
	struct in112525_ch ch[IN112525_CH_NUM];
	struct in112525_s03_vco_codes s03_vco_codes;

	struct delayed_work lock_check;
};

static const struct in112525_config in112525_s03_config[] = {
	[INIT_10GE] = {
		.enable_otu_protocol = 0,
		.enable_external_refclk = 0,
		.enable_prescaler = 0,
		.tx_pll_mpy_ratio = 20,
		.enable_half_rate = 1,
		.enable_extended_range = 1,
		.tx_pll_refclk_source = RECOV_CLK,
		.ctle_mode = MODE_25_25_10,
		.rx_common_mode = 3,
		.rx_odt_override = 0,
		.l0_phase_adjust_val = IN112525_PHASE_ADJUST_VAL,
		.l1_phase_adjust_val = IN112525_PHASE_ADJUST_VAL,
		.l2_phase_adjust_val = IN112525_PHASE_ADJUST_VAL,
		.l3_phase_adjust_val = IN112525_PHASE_ADJUST_VAL
	},

	[INIT_25GE] = {
		.enable_otu_protocol = 0,
		.enable_external_refclk = 0,
		.enable_prescaler = 0,
		.tx_pll_mpy_ratio = 10,
		.enable_half_rate = 0,
		.enable_extended_range = 0,
		.tx_pll_refclk_source = RECOV_CLK,
		.ctle_mode = MODE_25_25_10,
		.rx_common_mode = 3,
		.rx_odt_override = 0,
		.l0_phase_adjust_val = IN112525_PHASE_ADJUST_VAL,
		.l1_phase_adjust_val = IN112525_PHASE_ADJUST_VAL,
		.l2_phase_adjust_val = IN112525_PHASE_ADJUST_VAL,
		.l3_phase_adjust_val = IN112525_PHASE_ADJUST_VAL
	},
};

/* Lookup table to map the multiply options and MUX selects to the bit values */
static const unsigned char tx_pll_mpy_map[][2] = {
	/* MPY  MS, LS */
	[10] = {0,  0},
	[20] = {1,  0},
	[40] = {2,  0},
	[8]  = {0,  1},
	[16] = {1,  1},
	[32] = {2,  1},
	[33] = {1,  6},
	[66] = {2,  6},
	[15] = {0,  7},
	[30] = {1,  7},
	[60] = {2,  7},
};

static int mdio_wr(struct mdio_device *mdiodev, u32 regnum, u16 val)
{
	return mdiobus_c45_write(mdiodev->bus, mdiodev->addr, MDIO_MMD_VEND1, regnum, val);
}

static int mdio_rd(struct mdio_device *mdiodev, u32 regnum)
{
	return mdiobus_c45_read(mdiodev->bus, mdiodev->addr, MDIO_MMD_VEND1, regnum);
}

static int bit_test(int value, int bit_field)
{
	int bit_mask = (1 << bit_field);
	int result;

	result = ((value & bit_mask) == bit_mask);
	return result;
}

static void WAIT(int delay_cycles)
{
	usleep_range(delay_cycles * 10, delay_cycles * 11);
}

static void rx_reset_assert(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	int mask, val;

	if (lane == ALL_LANES) {
		val = mdio_rd(mdiodev, PHYMISC_REG2);
		mask = (1 << 15);
		mdio_wr(mdiodev, PHYMISC_REG2, val + mask);
	} else {
		val = mdio_rd(mdiodev, PHYCTRL_REG8 + lane * 0x100);
		mask = (1 << 6);
		mdio_wr(mdiodev, lane * 0x100 + PHYCTRL_REG8, val + mask);
	}
}

static void rx_reset_de_assert(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	int mask, val;

	if (lane == ALL_LANES) {
		val = mdio_rd(mdiodev, PHYMISC_REG2);
		mask = 0xffff - (1 << 15);
		mdio_wr(mdiodev, PHYMISC_REG2, val & mask);
	} else {
		val = mdio_rd(mdiodev, PHYCTRL_REG8 + lane * 0x100);
		mask = 0xffff - (1 << 6);
		mdio_wr(mdiodev, lane * 0x100 + PHYCTRL_REG8, val & mask);
	}
}

static void tx_core_assert(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	int recal, val, val2, core_reset;

	if (lane == 4) {
		val = mdio_rd(mdiodev, PHYMISC_REG2);
		recal = 1 << 10;
		mdio_wr(mdiodev, PHYMISC_REG2, val | recal);
	} else {
		val2 = mdio_rd(mdiodev, PHYMISC_REG3);
		core_reset = (1 << (lane + 8));
		mdio_wr(mdiodev, PHYMISC_REG3, val2 | core_reset);
	}
}

static void tx_core_de_assert(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	int val, recal, val2, core_reset;

	if (lane == ALL_LANES) {
		val = mdio_rd(mdiodev, PHYMISC_REG2);
		recal = 0xffff - (1 << 10);
		mdio_wr(mdiodev, PHYMISC_REG2, val & recal);
	} else {
		val2 = mdio_rd(mdiodev, PHYMISC_REG3);
		core_reset = 0xffff - (1 << (lane + 8));
		mdio_wr(mdiodev, PHYMISC_REG3, val2 & core_reset);
	}
}

static void tx_pll_assert(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	int val, recal;

	if (lane == ALL_LANES) {
		val = mdio_rd(mdiodev, PHYMISC_REG2);
		recal = (1 << 12);
		mdio_wr(mdiodev, PHYMISC_REG2, val | recal);
	} else {
		val = mdio_rd(mdiodev, lane * 0x100 + PHYCTRL_REG4);
		recal = (1 << 15);
		mdio_wr(mdiodev, lane * 0x100 + PHYCTRL_REG4, val | recal);
	}
}

static void tx_pll_de_assert(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	int recal, val;

	if (lane == ALL_LANES) {
		val = mdio_rd(mdiodev, PHYMISC_REG2);
		recal = 0xefff;
		mdio_wr(mdiodev, PHYMISC_REG2, val & recal);
	} else {
		val = mdio_rd(mdiodev, lane * 0x100 + PHYCTRL_REG4);
		recal = 0x7fff;
		mdio_wr(mdiodev, lane * 0x100 + PHYCTRL_REG4, val & recal);
	}
}

static void tx_restart(struct in112525_priv *priv, int lane)
{
	tx_core_assert(priv, lane);
	tx_pll_assert(priv, lane);
	tx_pll_de_assert(priv, lane);
	WAIT(150);
	tx_core_de_assert(priv, lane);
}

static bool tx_pll_lock_test(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	bool locked = true;
	int i, val;

	if (lane == ALL_LANES) {
		for (i = 0; i < ALL_LANES; i++) {
			val = mdio_rd(mdiodev, i * 0x100 + PHYSTAT_REG3);
			if (!(val & BIT(15)))
				locked = false;
		}
	} else {
		val = mdio_rd(mdiodev, lane * 0x100 + PHYSTAT_REG3);
		if (!(val & BIT(15)))
			locked = false;
	}

	return locked;
}

static void save_vco_codes(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	int value0, value1, value2, value3;

	if (lane == 0 || lane == ALL_LANES) {
		value0 = mdio_rd(mdiodev, PHYMISC_REG5);
		mdio_wr(mdiodev, PHYMISC_REG7, value0 + IN112525_RX_VCO_CODE_OFFSET);
		priv->s03_vco_codes.l0_vco_code = value0;
	}
	if (lane == 1 || lane == ALL_LANES) {
		value1 = mdio_rd(mdiodev, PHYMISC_REG5 + 0x100);
		mdio_wr(mdiodev, PHYMISC_REG7 + 0x100,
			value1 + IN112525_RX_VCO_CODE_OFFSET);
		priv->s03_vco_codes.l1_vco_code = value1;
	}
	if (lane == 2 || lane == ALL_LANES) {
		value2 = mdio_rd(mdiodev, PHYMISC_REG5 + 0x200);
		mdio_wr(mdiodev, PHYMISC_REG7 + 0x200,
			value2 + IN112525_RX_VCO_CODE_OFFSET);
		priv->s03_vco_codes.l2_vco_code = value2;
	}
	if (lane == 3 || lane == ALL_LANES) {
		value3 = mdio_rd(mdiodev, PHYMISC_REG5 + 0x300);
		mdio_wr(mdiodev, PHYMISC_REG7 + 0x300,
			value3 + IN112525_RX_VCO_CODE_OFFSET);
		priv->s03_vco_codes.l3_vco_code = value3;
	}
}

static bool az_complete_test(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	bool success = true;
	int value;

	if (lane == 0 || lane == ALL_LANES) {
		value = mdio_rd(mdiodev, PHYCTRL_REG5);
		if (!(value & BIT(2)))
			success = false;
	}
	if (lane == 1 || lane == ALL_LANES) {
		value = mdio_rd(mdiodev, PHYCTRL_REG5 + 0x100);
		if (!(value & BIT(2)))
			success = false;
	}
	if (lane == 2 || lane == ALL_LANES) {
		value = mdio_rd(mdiodev, PHYCTRL_REG5 + 0x200);
		if (!(value & BIT(2)))
			success = false;
	}
	if (lane == 3 || lane == ALL_LANES) {
		value = mdio_rd(mdiodev, PHYCTRL_REG5 + 0x300);
		if (!(value & BIT(2)))
			success = false;
	}

	return success;
}

#define AZ_OFFSET_LANE_UPDATE(mdiodev, reg, lane) \
	mdio_wr(mdiodev, (reg) + (lane) * 0x100,  \
		(mdio_rd(mdiodev, (reg) + (lane) * 0x100) >> 8))

static void lane_save_az_offsets(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;

	AZ_OFFSET_LANE_UPDATE(mdiodev, PHYMISC_REG20, lane);
	AZ_OFFSET_LANE_UPDATE(mdiodev, PHYMISC_REG20 + 1, lane);
	AZ_OFFSET_LANE_UPDATE(mdiodev, PHYMISC_REG20 + 2, lane);
	AZ_OFFSET_LANE_UPDATE(mdiodev, PHYMISC_REG20 + 3, lane);
	AZ_OFFSET_LANE_UPDATE(mdiodev, PHYMISC_REG21, lane);
	AZ_OFFSET_LANE_UPDATE(mdiodev, PHYMISC_REG21 + 1, lane);
	AZ_OFFSET_LANE_UPDATE(mdiodev, PHYMISC_REG21 + 2, lane);
	AZ_OFFSET_LANE_UPDATE(mdiodev, PHYMISC_REG21 + 3, lane);
	AZ_OFFSET_LANE_UPDATE(mdiodev, PHYMISC_REG22, lane);
}

static void save_az_offsets(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	int i;

	if (lane == ALL_LANES) {
		for (i = 0; i < ALL_LANES; i++)
			lane_save_az_offsets(priv, i);
	} else {
		lane_save_az_offsets(priv, lane);
	}

	mdio_wr(mdiodev, PHYCTRL_REG7, 0x0001);
}

static void in112525_lane_recovery(struct in112525_priv *priv, int lane)
{
	struct mdio_device *mdiodev = priv->mdiodev;
	struct device *dev = &mdiodev->dev;
	bool az_pass;
	int i, value;

	switch (lane) {
	case 0:
	case 1:
	case 2:
	case 3:
		rx_reset_assert(priv, lane);
		WAIT(2000);
		break;
	case ALL_LANES:
		mdio_wr(mdiodev, PHYMISC_REG2, 0x9C00);
		WAIT(2000);
		while (1) {
			value = mdio_rd(mdiodev, PHYMISC_REG2);
			if (bit_test(value, 4))
				break;
		}
		break;
	default:
		break;
	}

	if (lane == 0 || lane == ALL_LANES)
		mdio_wr(mdiodev, PHYMISC_REG7, L0_VCO_CODE_trim);
	if (lane == 1 || lane == ALL_LANES)
		mdio_wr(mdiodev, PHYMISC_REG7 + 0x100, L1_VCO_CODE_trim);
	if (lane == 2 || lane == ALL_LANES)
		mdio_wr(mdiodev, PHYMISC_REG7 + 0x200, L2_VCO_CODE_trim);
	if (lane == 3 || lane == ALL_LANES)
		mdio_wr(mdiodev, PHYMISC_REG7 + 0x300, L3_VCO_CODE_trim);

	if (lane == 0 || lane == 4)
		mdio_wr(mdiodev, PHYCTRL_REG5, 0x0418);
	if (lane == 1 || lane == 4)
		mdio_wr(mdiodev, PHYCTRL_REG5 + 0x100, 0x0418);
	if (lane == 2 || lane == 4)
		mdio_wr(mdiodev, PHYCTRL_REG5 + 0x200, 0x0418);
	if (lane == 3 || lane == 4)
		mdio_wr(mdiodev, PHYCTRL_REG5 + 0x300, 0x0418);

	mdio_wr(mdiodev, PHYCTRL_REG7, 0x0000);
	rx_reset_de_assert(priv, lane);

	if (lane == 0 || lane == 4) {
		mdio_wr(mdiodev, PHYCTRL_REG5, 0x0410);
		mdio_wr(mdiodev, PHYCTRL_REG5, 0x0412);
	}
	if (lane == 1 || lane == 4) {
		mdio_wr(mdiodev, PHYCTRL_REG5 + 0x100, 0x0410);
		mdio_wr(mdiodev, PHYCTRL_REG5 + 0x100, 0x0412);
	}
	if (lane == 2 || lane == 4) {
		mdio_wr(mdiodev, PHYCTRL_REG5 + 0x200, 0x0410);
		mdio_wr(mdiodev, PHYCTRL_REG5 + 0x200, 0x0412);
	}
	if (lane == 3 || lane == 4) {
		mdio_wr(mdiodev, PHYCTRL_REG5 + 0x300, 0x0410);
		mdio_wr(mdiodev, PHYCTRL_REG5 + 0x300, 0x0412);
	}

	for (i = 0; i < 64; i++) {
		/* wait 1000 times 10us */
		WAIT(10000);
		az_pass = az_complete_test(priv, lane);
		if (az_pass) {
			save_az_offsets(priv, lane);
			break;
		}
	}

	if (!az_pass) {
		dev_err(dev, "auto-zero calibration timed out for lane %d\n", lane);
		return;
	}

	mdio_wr(mdiodev, lane * 0x100 + PHYMISC_REG4, 0x0002);
	mdio_wr(mdiodev, lane * 0x100 + PHYMISC_REG6, 0x2028);
	mdio_wr(mdiodev, lane * 0x100 + PHYCTRL_REG5, 0x0010);
	WAIT(100);
	mdio_wr(mdiodev, lane * 0x100 + PHYCTRL_REG5, 0x0110);
	WAIT(3000);
	mdio_wr(mdiodev, lane * 0x100 + PHYMISC_REG6, 0x3020);

	if (lane == ALL_LANES) {
		mdio_wr(mdiodev, PHYMISC_REG2, 0x1C00);
		mdio_wr(mdiodev, PHYMISC_REG2, 0x0C00);
	} else {
		tx_restart(priv, lane);
		/* delay > 10ms is required */
		WAIT(1100);
	}

	if (lane == ALL_LANES) {
		if (bit_test(mdio_rd(priv->mdiodev, PHYMISC_REG2), 6) == 0)
			dev_dbg(dev, "TX PLL not locked on ALL lanes\n");
	} else {
		if (!tx_pll_lock_test(priv, lane)) {
			dev_dbg(dev, "TX PLL not locked on lane %d\n", lane);
			return;
		}
	}

	save_vco_codes(priv, lane);

	if (lane == ALL_LANES) {
		mdio_wr(mdiodev, PHYMISC_REG2, 0x0400);
		mdio_wr(mdiodev, PHYMISC_REG2, 0x0000);
		value = mdio_rd(mdiodev, PHYCTRL_REG1);
		value = value & 0xffbf;
		mdio_wr(mdiodev, PHYCTRL_REG2, value);
	} else {
		tx_core_de_assert(priv, lane);
	}

	if (lane == ALL_LANES) {
		mdio_wr(mdiodev, PHYMISC_REG1, 0x8000);
		mdio_wr(mdiodev, PHYMISC_REG1, 0x0000);
	}

	WAIT(100);
}

static void in112525_phy_reset_assert(struct phy *phy)
{
	struct in112525_ch *ch = phy_get_drvdata(phy);
	struct in112525_priv *priv = ch->priv;
	int i;

	for (i = 0; i < IN112525_LANE_PER_CH; i++) {
		rx_reset_assert(priv, ch->lane[i]);
		tx_pll_assert(priv, ch->lane[i]);
		tx_core_assert(priv, ch->lane[i]);
	}
}

static void in112525_phy_reset_deassert(struct phy *phy)
{
	struct in112525_ch *ch = phy_get_drvdata(phy);
	struct in112525_priv *priv = ch->priv;
	int i;

	for (i = 0; i < IN112525_LANE_PER_CH; i++) {
		rx_reset_de_assert(priv, ch->lane[i]);
		tx_pll_de_assert(priv, ch->lane[i]);
		tx_core_de_assert(priv, ch->lane[i]);
	}
}

static void in112525_phy_recovery(struct phy *phy)
{
	struct in112525_ch *ch = phy_get_drvdata(phy);
	struct in112525_priv *priv = ch->priv;
	int i;

	for (i = 0; i < IN112525_LANE_PER_CH; i++) {
		in112525_lane_recovery(priv, ch->lane[i]);
		in112525_lane_recovery(priv, ch->lane[i]);
	}
}

static void in112525_phy_cfg_extended_range(struct phy *phy, const struct in112525_config *cfg)
{
	struct in112525_ch *ch = phy_get_drvdata(phy);
	struct in112525_priv *priv = ch->priv;
	struct mdio_device *mdiodev = priv->mdiodev;
	int i;

	if (cfg->enable_extended_range) {
		for (i = 0; i < IN112525_LANE_PER_CH; i++) {
			mdio_wr(mdiodev, PHYCTRL_REG10_LANE(ch->lane[i]), 0x2032);
			mdio_wr(mdiodev, PHYCTRL_REG12_LANE(ch->lane[i]), 0x0007);
		}
	} else {
		for (i = 0; i < IN112525_LANE_PER_CH; i++) {
			mdio_wr(mdiodev, PHYCTRL_REG10_LANE(ch->lane[i]), 0xA02D);
			mdio_wr(mdiodev, PHYCTRL_REG12_LANE(ch->lane[i]), 0x0005);
		}
	}
}

static void in112525_phy_cfg_rate(struct phy *phy, const struct in112525_config *cfg)
{
	struct in112525_ch *ch = phy_get_drvdata(phy);
	struct in112525_priv *priv = ch->priv;
	struct mdio_device *mdiodev = priv->mdiodev;
	int i;

	if (cfg->enable_half_rate) {
		for (i = 0; i < IN112525_LANE_PER_CH; i++)
			mdio_wr(mdiodev, PHYCTRL_REG14_LANE(ch->lane[i]), 0x0020);
	} else {
		for (i = 0; i < IN112525_LANE_PER_CH; i++)
			mdio_wr(mdiodev, PHYCTRL_REG14_LANE(ch->lane[i]), 0x0000);
	}
}

static void in112525_phy_cfg_tx_pll(struct phy *phy, const struct in112525_config *cfg)
{
	struct in112525_ch *ch = phy_get_drvdata(phy);
	struct in112525_priv *priv = ch->priv;
	struct mdio_device *mdiodev = priv->mdiodev;
	int tx_pll_MSDIV_value;
	int tx_pll_LSDIV_value;
	int tx_pll_ctrl2_value;
	int tx_pll_iqdiv;
	int i;

	tx_pll_MSDIV_value = tx_pll_mpy_map[cfg->tx_pll_mpy_ratio][0];
	tx_pll_LSDIV_value = tx_pll_mpy_map[cfg->tx_pll_mpy_ratio][1];
	tx_pll_iqdiv = (cfg->enable_half_rate) ? 1 : 0;
	tx_pll_ctrl2_value =
		(cfg->tx_pll_refclk_source << 11) +
		(tx_pll_iqdiv << 8) +
		(tx_pll_MSDIV_value << 4) +
		tx_pll_LSDIV_value;

	for (i = 0; i < IN112525_LANE_PER_CH; i++)
		mdio_wr(mdiodev, PHYCTRL_REG11_LANE(ch->lane[i]), tx_pll_ctrl2_value);
}

static int in112525_phy_init(struct phy *phy, const struct in112525_config *cfg)
{
	struct in112525_ch *ch = phy_get_drvdata(phy);
	struct in112525_priv *priv = ch->priv;
	struct mdio_device *mdiodev = priv->mdiodev;
	struct device *dev = &mdiodev->dev;
	u32 reg_value;
	u32 reg;
	int i;

	/* Put the chip in hw/sw  and MDIO reset */
	mdio_wr(mdiodev, PHYCTRL_REG0,
		IN112525_HRESET | IN112525_SRESET | IN112525_MDIOINIT);

	/* De-assert MDIO init */
	mdio_wr(mdiodev, PHYCTRL_REG0, IN112525_HRESET | IN112525_SRESET);

	if (cfg->enable_prescaler)
		mdio_wr(mdiodev, IN112525_PRESCALE_20M, 0x0001);

	if (cfg->enable_external_refclk)
		mdio_wr(mdiodev, PHYMISC_REG3, (1 << 15));
	else
		mdio_wr(mdiodev, PHYMISC_REG3, 0x0);

	mdio_wr(mdiodev, PHYCTRL_REG0, IN112525_SRESET);

	WAIT(1000);

	reg_value = mdio_rd(mdiodev, IN112525_EFUSE_REG);
	if (!(reg_value & IN112525_EFUSE_DONE)) {
		dev_err(dev, "IN112525_s03 init failed: EFUSE Done not set\n");
		return -1;
	}
	WAIT(1000);

	reg_value = mdio_rd(mdiodev, PHYMISC_REG2);
	if (!(reg_value & IN112525_CALIBRATION_DONE)) {
		dev_err(dev, "IN112525_s03 init failed: CALIBRATION_DONE not set\n");
		return -1;
	}

	mdio_wr(mdiodev, PHYMISC_REG2,
		IN112525_RX_PLL_RESET |
		IN112525_TX_PLL_RESET |
		IN112525_TX_SERDES_RESET |
		IN112525_CORE_DATAPATH_RESET);

	if (cfg->enable_otu_protocol)
		mdio_wr(mdiodev, PHYCTRL_REG0, 0x8C00);
	else
		mdio_wr(mdiodev, PHYCTRL_REG0, 0x8200);

	in112525_phy_cfg_extended_range(phy, cfg);

	mdio_wr(mdiodev, PHYCTRL_REG18, 0x00ff);
	mdio_wr(mdiodev, PHYCTRL_REG19, 0x002d);
	mdio_wr(mdiodev, PHYCTRL_REG19, 0x802d);
	mdio_wr(mdiodev, PHYCTRL_REG19, 0x0000);
	mdio_wr(mdiodev, PHYCTRL_REG18, 0x00e9);
	mdio_wr(mdiodev, PHYCTRL_REG19, 0x0008);
	mdio_wr(mdiodev, PHYCTRL_REG19, 0x8008);
	mdio_wr(mdiodev, PHYCTRL_REG19, 0x0000);

	in112525_phy_cfg_tx_pll(phy, cfg);

	in112525_phy_cfg_rate(phy, cfg);

	/* set the CTLE mode (bw on the front end stages)
	 * for example '25:25:10', '10:10:10' etc.
	 */
	for (i = 0; i < ALL_LANES; i++) {
		reg = i * 0x100 + PHYCTRL_REG8;
		reg_value = mdio_rd(mdiodev, reg);
		reg_value = reg_value & 0xFF7C;
		/* put bits 7,1,0 for EQ */
		reg_value = reg_value | cfg->ctle_mode;
		mdio_wr(mdiodev, reg, reg_value);
	}

	/* rx common code settings */
	mdio_wr(mdiodev, PHYMISC_REG32, cfg->rx_common_mode);
	mdio_wr(mdiodev, PHYMISC_REG32 + 0x100, cfg->rx_common_mode);
	mdio_wr(mdiodev, PHYMISC_REG32 + 0x200, cfg->rx_common_mode - 1);
	mdio_wr(mdiodev, PHYMISC_REG32 + 0x300, cfg->rx_common_mode - 1);

	if (cfg->rx_odt_override)
		mdio_wr(mdiodev, PHYMISC_REG31, cfg->rx_odt_override);

	priv->s03_vco_codes.l0_vco_code = mdio_rd(mdiodev, PHYMISC_REG7);
	priv->s03_vco_codes.l1_vco_code = mdio_rd(mdiodev, PHYMISC_REG7 + 0x100);
	priv->s03_vco_codes.l2_vco_code = mdio_rd(mdiodev, PHYMISC_REG7 + 0x200);
	priv->s03_vco_codes.l3_vco_code = mdio_rd(mdiodev, PHYMISC_REG7 + 0x300);

	if (cfg->enable_extended_range) {
		priv->s03_vco_codes.l0_vco_code = (int)(priv->s03_vco_codes.l0_vco_code * 4 / 5);
		priv->s03_vco_codes.l1_vco_code = (int)(priv->s03_vco_codes.l1_vco_code * 4 / 5);
		priv->s03_vco_codes.l2_vco_code = (int)(priv->s03_vco_codes.l2_vco_code * 4 / 5);
		priv->s03_vco_codes.l3_vco_code = (int)(priv->s03_vco_codes.l3_vco_code * 4 / 5);
	}

	mdio_wr(mdiodev, PHYMISC_REG2, 0x0);
	WAIT(10000);

	in112525_lane_recovery(priv, ALL_LANES);

	return 0;
}

static void in112525_phy_set_mode(struct phy *phy, const struct in112525_config *cfg)
{
	in112525_phy_reset_assert(phy);

	in112525_phy_cfg_extended_range(phy, cfg);
	in112525_phy_cfg_tx_pll(phy, cfg);
	in112525_phy_cfg_rate(phy, cfg);

	in112525_phy_reset_deassert(phy);
	in112525_phy_recovery(phy);
}

static int in112525_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct in112525_ch *ch = phy_get_drvdata(phy);
	struct in112525_priv *priv = ch->priv;
	const struct in112525_config *cfg;

	if (mode != PHY_MODE_ETHERNET)
		return -EOPNOTSUPP;

	if ((submode != PHY_INTERFACE_MODE_10GBASER) &&
	    (submode != PHY_INTERFACE_MODE_25GBASER))
		return -EOPNOTSUPP;

	/* Stop the workqueue for the moment */
	cancel_delayed_work_sync(&priv->lock_check);

	/* Setup the correct configuration */
	switch (submode) {
	case PHY_INTERFACE_MODE_10GBASER:
		cfg = &in112525_s03_config[INIT_10GE];
		break;
	case PHY_INTERFACE_MODE_25GBASER:
		cfg = &in112525_s03_config[INIT_25GE];
		break;
	default:
		return -EOPNOTSUPP;
	}
	in112525_phy_set_mode(phy, cfg);

	/* Schedule the workqueue again */
	queue_delayed_work(system_power_efficient_wq, &priv->lock_check,
			   msecs_to_jiffies(2500));

	return 0;
}

static const struct phy_ops in112525_ops = {
	.set_mode	= in112525_set_mode,
	.owner		= THIS_MODULE,
};

static struct phy *in112525_xlate(struct device *dev,
				  const struct of_phandle_args *args)
{
	struct in112525_priv *priv = dev_get_drvdata(dev);
	int idx = args->args[0];

	if (WARN_ON(idx >= IN112525_CH_NUM))
		return ERR_PTR(-EINVAL);

	return priv->ch[idx].phy;
}

#define work_to_in112525(w) container_of((w), struct in112525_priv, lock_check.work)

static void in112525_lock_check(struct work_struct *work)
{
	int lanes_lock, lane0_lock, lane1_lock, lane2_lock, lane3_lock;
	struct in112525_priv *priv = work_to_in112525(work);
	struct mdio_device *mdiodev = priv->mdiodev;

	lane0_lock = bit_test(mdio_rd(mdiodev, 0x123), 15);
	lane1_lock = bit_test(mdio_rd(mdiodev, 0x223), 15);
	lane2_lock = bit_test(mdio_rd(mdiodev, 0x323), 15);
	lane3_lock = bit_test(mdio_rd(mdiodev, 0x423), 15);

	/* Check if the chip had any successful lane lock from the previous stage
	 * stage, if not, then start fresh.
	 */
	lanes_lock = lane0_lock | lane1_lock | lane2_lock | lane3_lock;

	if (!lanes_lock) {
		in112525_lane_recovery(priv, ALL_LANES);
	} else {
		if (!lane0_lock)
			in112525_lane_recovery(priv, 0);
		if (!lane1_lock)
			in112525_lane_recovery(priv, 1);
		if (!lane2_lock)
			in112525_lane_recovery(priv, 2);
		if (!lane3_lock)
			in112525_lane_recovery(priv, 3);
	}

	queue_delayed_work(system_power_efficient_wq, &priv->lock_check,
			   msecs_to_jiffies(2500));
}

static int in112525_probe(struct mdio_device *mdiodev)
{
	const struct in112525_config *cfg = &in112525_s03_config[INIT_25GE];
	struct device *dev = &mdiodev->dev;
	struct phy_provider *provider;
	struct in112525_priv *priv;
	int i, err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->mdiodev = mdiodev;

	for (i = 0; i < IN112525_CH_NUM; i++) {
		struct in112525_ch *ch = &priv->ch[i];
		struct phy *phy;

		phy = devm_phy_create(dev, NULL, &in112525_ops);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		ch->priv = priv;
		ch->phy = phy;
		ch->id = i;
		if (ch->id == 0) {
			ch->lane[0] = 0;
			ch->lane[1] = 2;
		} else {
			ch->lane[0] = 1;
			ch->lane[1] = 3;
		}

		phy_set_drvdata(phy, ch);

		err = in112525_phy_init(phy, cfg);
		if (err)
			dev_warn(dev, "in112525_phy_init() failed with %d for channel %d\n",
				 err, i);
	}

	INIT_DELAYED_WORK(&priv->lock_check, in112525_lock_check);
	queue_delayed_work(system_power_efficient_wq, &priv->lock_check,
			   msecs_to_jiffies(2500));

	dev_set_drvdata(dev, priv);
	provider = devm_of_phy_provider_register(dev, in112525_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static void in112525_remove(struct mdio_device *mdiodev)
{
	struct in112525_priv *priv = dev_get_drvdata(&mdiodev->dev);

	cancel_delayed_work_sync(&priv->lock_check);
}

static const struct of_device_id in112525_of_match[] = {
	{ .compatible = "inphi,in112525", },
	{ },
};

MODULE_DEVICE_TABLE(of, in112525_of_match);

static struct mdio_driver in112525_driver = {
	.probe  = in112525_probe,
	.remove = in112525_remove,
	.mdiodrv.driver = {
		.name = "in112525",
		.of_match_table = in112525_of_match,
	},
};

mdio_module_driver(in112525_driver);
