// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX96724 Quad GMSL2 Deserializer Driver
 *
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/regmap.h>

#include "max_des.h"

#define MAX96724_REG0				0x0

#define MAX96724_REG6				0x6
#define MAX96724_REG6_LINK_EN			GENMASK(3, 0)

#define MAX96724_DEBUG_EXTRA			0x9
#define MAX96724_DEBUG_EXTRA_PCLK_SRC		GENMASK(1, 0)
#define MAX96724_DEBUG_EXTRA_PCLK_SRC_25MHZ	0b00
#define MAX96724_DEBUG_EXTRA_PCLK_SRC_75MHZ	0b01
#define MAX96724_DEBUG_EXTRA_PCLK_SRC_USE_PIPE	0b10

#define MAX96724_REG26(x)			(0x10 + (x) / 2)
#define MAX96724_REG26_RX_RATE_PHY(x)		(GENMASK(1, 0) << (4 * ((x) % 2)))
#define MAX96724_REG26_RX_RATE_3GBPS		0b01
#define MAX96724_REG26_RX_RATE_6GBPS		0b10

#define MAX96724_PWR1				0x13
#define MAX96724_PWR1_RESET_ALL			BIT(6)

#define MAX96724_CTRL1				0x18
#define MAX96724_CTRL1_RESET_ONESHOT		GENMASK(3, 0)

#define MAX96724_VIDEO_PIPE_SEL(p)		(0xf0 + (p) / 2)
#define MAX96724_VIDEO_PIPE_SEL_STREAM(p)	(GENMASK(1, 0) << (4 * ((p) % 2)))
#define MAX96724_VIDEO_PIPE_SEL_LINK(p)		(GENMASK(3, 2) << (4 * ((p) % 2)))

#define MAX96724_VIDEO_PIPE_EN			0xf4
#define MAX96724_VIDEO_PIPE_EN_MASK(p)		BIT(p)
#define MAX96724_VIDEO_PIPE_EN_STREAM_SEL_ALL	BIT(4)

#define MAX96724_VPRBS(p)			(0x1dc + (p) * 0x20)
#define MAX96724_VPRBS_VIDEO_LOCK		BIT(0)
#define MAX96724_VPRBS_PATGEN_CLK_SRC		BIT(7)
#define MAX96724_VPRBS_PATGEN_CLK_SRC_150MHZ	0b0
#define MAX96724_VPRBS_PATGEN_CLK_SRC_375MHZ	0b1

#define MAX96724_BACKTOP12			0x40b
#define MAX96724_BACKTOP12_CSI_OUT_EN		BIT(1)

#define MAX96724_BACKTOP21(p)			(0x414 + (p) / 4 * 0x20)
#define MAX96724_BACKTOP21_BPP8DBL(p)		BIT(4 + (p) % 4)

#define MAX96724_BACKTOP22(x)			(0x415 + (x) * 0x3)
#define MAX96724_BACKTOP22_PHY_CSI_TX_DPLL	GENMASK(4, 0)
#define MAX96724_BACKTOP22_PHY_CSI_TX_DPLL_EN	BIT(5)

#define MAX96724_BACKTOP24(p)			(0x417 + (p) / 4 * 0x20)
#define MAX96724_BACKTOP24_BPP8DBL_MODE(p)	BIT(4 + (p) % 4)

#define MAX96724_BACKTOP30(p)			(0x41d + (p) / 4 * 0x20)
#define MAX96724_BACKTOP30_BPP10DBL3		BIT(4)
#define MAX96724_BACKTOP30_BPP10DBL3_MODE	BIT(5)

#define MAX96724_BACKTOP31(p)			(0x41e + (p) / 4 * 0x20)
#define MAX96724_BACKTOP31_BPP10DBL2		BIT(6)
#define MAX96724_BACKTOP31_BPP10DBL2_MODE	BIT(7)

#define MAX96724_BACKTOP32(p)			(0x41f + (p) / 4 * 0x20)
#define MAX96724_BACKTOP32_BPP12(p)		BIT(p)
#define MAX96724_BACKTOP32_BPP10DBL0		BIT(4)
#define MAX96724_BACKTOP32_BPP10DBL0_MODE	BIT(5)
#define MAX96724_BACKTOP32_BPP10DBL1		BIT(6)
#define MAX96724_BACKTOP32_BPP10DBL1_MODE	BIT(7)

#define MAX96724_MIPI_PHY0			0x8a0
#define MAX96724_MIPI_PHY0_PHY_CONFIG		GENMASK(4, 0)
#define MAX96724_MIPI_PHY0_PHY_4X2		BIT(0)
#define MAX96724_MIPI_PHY0_PHY_2X4		BIT(2)
#define MAX96724_MIPI_PHY0_PHY_1X4A_2X2		BIT(3)
#define MAX96724_MIPI_PHY0_PHY_1X4B_2X2		BIT(4)
#define MAX96724_MIPI_PHY0_FORCE_CSI_OUT_EN	BIT(7)

#define MAX96724_MIPI_PHY2			0x8a2
#define MAX96724_MIPI_PHY2_PHY_STDB_N_4(x)	(GENMASK(5, 4) << ((x) / 2 * 2))
#define MAX96724_MIPI_PHY2_PHY_STDB_N_2(x)	(BIT(4 + (x)))

#define MAX96724_MIPI_PHY3(x)			(0x8a3 + (x) / 2)
#define MAX96724_MIPI_PHY3_PHY_LANE_MAP_4	GENMASK(7, 0)
#define MAX96724_MIPI_PHY3_PHY_LANE_MAP_2(x)	(GENMASK(3, 0) << (4 * ((x) % 2)))

#define MAX96724_MIPI_PHY5(x)			(0x8a5 + (x) / 2)
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_4_0_1	GENMASK(1, 0)
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_4_2_3	GENMASK(4, 3)
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_4_CLK	BIT(5)
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_2(x)	(GENMASK(1, 0) << (3 * ((x) % 2)))
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_2_CLK(x)	BIT(2 + 3 * ((x) % 2))

#define MAX96724_MIPI_PHY13			0x8ad
#define MAX96724_MIPI_PHY13_T_T3_PREBEGIN	GENMASK(5, 0)
#define MAX96724_MIPI_PHY13_T_T3_PREBEGIN_64X7	FIELD_PREP(MAX96724_MIPI_PHY13_T_T3_PREBEGIN, 63)

#define MAX96724_MIPI_PHY14			0x8ae
#define MAX96724_MIPI_PHY14_T_T3_PREP		GENMASK(1, 0)
#define MAX96724_MIPI_PHY14_T_T3_PREP_55NS	FIELD_PREP(MAX96724_MIPI_PHY14_T_T3_PREP, 0b01)
#define MAX96724_MIPI_PHY14_T_T3_POST		GENMASK(6, 2)
#define MAX96724_MIPI_PHY14_T_T3_POST_32X7	FIELD_PREP(MAX96724_MIPI_PHY14_T_T3_POST, 31)

#define MAX96724_MIPI_CTRL_SEL			0x8ca
#define MAX96724_MIPI_CTRL_SEL_MASK(p)		(GENMASK(1, 0) << ((p) * 2))

#define MAX96724_MIPI_PHY25(x)			(0x8d0 + (x) / 2)
#define MAX96724_MIPI_PHY25_CSI2_TX_PKT_CNT(x)	(GENMASK(3, 0) << (4 * ((x) % 2)))

#define MAX96724_MIPI_PHY27(x)			(0x8d2 + (x) / 2)
#define MAX96724_MIPI_PHY27_PHY_PKT_CNT(x)	(GENMASK(3, 0) << (4 * ((x) % 2)))

#define MAX96724_MIPI_TX3(x)			(0x903 + (x) * 0x40)
#define MAX96724_MIPI_TX3_DESKEW_INIT_8X32K	FIELD_PREP(GENMASK(2, 0), 0b001)
#define MAX96724_MIPI_TX3_DESKEW_INIT_AUTO	BIT(7)

#define MAX96724_MIPI_TX4(x)			(0x904 + (x) * 0x40)
#define MAX96724_MIPI_TX4_DESKEW_PER_2K		FIELD_PREP(GENMASK(2, 0), 0b001)
#define MAX96724_MIPI_TX4_DESKEW_PER_AUTO	BIT(7)

#define MAX96724_MIPI_TX10(x)			(0x90a + (x) * 0x40)
#define MAX96724_MIPI_TX10_CSI2_CPHY_EN		BIT(5)
#define MAX96724_MIPI_TX10_CSI2_LANE_CNT	GENMASK(7, 6)

#define MAX96724_MIPI_TX11(p)			(0x90b + (p) * 0x40)
#define MAX96724_MIPI_TX12(p)			(0x90c + (p) * 0x40)

#define MAX96724_MIPI_TX13(p, x)		(0x90d + (p) * 0x40 + (x) * 0x2)
#define MAX96724_MIPI_TX13_MAP_SRC_DT		GENMASK(5, 0)
#define MAX96724_MIPI_TX13_MAP_SRC_VC		GENMASK(7, 6)

#define MAX96724_MIPI_TX14(p, x)		(0x90e + (p) * 0x40 + (x) * 0x2)
#define MAX96724_MIPI_TX14_MAP_DST_DT		GENMASK(5, 0)
#define MAX96724_MIPI_TX14_MAP_DST_VC		GENMASK(7, 6)

#define MAX96724_MIPI_TX45(p, x)		(0x92d + (p) * 0x40 + (x) / 4)
#define MAX96724_MIPI_TX45_MAP_DPHY_DEST(x)	(GENMASK(1, 0) << (2 * ((x) % 4)))

#define MAX96724_MIPI_TX51(x)			(0x933 + (x) * 0x40)
#define MAX96724_MIPI_TX51_ALT_MEM_MAP_12	BIT(0)
#define MAX96724_MIPI_TX51_ALT_MEM_MAP_8	BIT(1)
#define MAX96724_MIPI_TX51_ALT_MEM_MAP_10	BIT(2)
#define MAX96724_MIPI_TX51_ALT2_MEM_MAP_8	BIT(4)

#define MAX96724_MIPI_TX54(x)			(0x936 + (x) * 0x40)
#define MAX96724_MIPI_TX54_TUN_EN		BIT(0)

#define MAX96724_MIPI_TX57(x)			(0x939 + (x) * 0x40)
#define MAX96724_MIPI_TX57_TUN_DEST		GENMASK(5, 4)
#define MAX96724_MIPI_TX57_DIS_AUTO_TUN_DET	BIT(6)
#define MAX96724_DET(p)				BIT(p)

#define MAX96724_PATGEN_0			0x1050
#define MAX96724_PATGEN_0_VTG_MODE		GENMASK(1, 0)
#define MAX96724_PATGEN_0_VTG_MODE_FREE_RUNNING	0b11
#define MAX96724_PATGEN_0_DE_INV		BIT(2)
#define MAX96724_PATGEN_0_HS_INV		BIT(3)
#define MAX96724_PATGEN_0_VS_INV		BIT(4)
#define MAX96724_PATGEN_0_GEN_DE		BIT(5)
#define MAX96724_PATGEN_0_GEN_HS		BIT(6)
#define MAX96724_PATGEN_0_GEN_VS		BIT(7)

#define MAX96724_PATGEN_1			0x1051
#define MAX96724_PATGEN_1_PATGEN_MODE		GENMASK(5, 4)
#define MAX96724_PATGEN_1_PATGEN_MODE_DISABLED	0b00
#define MAX96724_PATGEN_1_PATGEN_MODE_CHECKER	0b01
#define MAX96724_PATGEN_1_PATGEN_MODE_GRADIENT	0b10

#define MAX96724_VS_DLY_2			0x1052
#define MAX96724_VS_HIGH_2			0x1055
#define MAX96724_VS_LOW_2			0x1058
#define MAX96724_V2H_2				0x105b
#define MAX96724_HS_HIGH_1			0x105e
#define MAX96724_HS_LOW_1			0x1060
#define MAX96724_HS_CNT_1			0x1062
#define MAX96724_V2D_2				0x1064
#define MAX96724_DE_HIGH_1			0x1067
#define MAX96724_DE_LOW_1			0x1069
#define MAX96724_DE_CNT_1			0x106b
#define MAX96724_GRAD_INCR			0x106d
#define MAX96724_CHKR_COLOR_A_L			0x106e
#define MAX96724_CHKR_COLOR_B_L			0x1071
#define MAX96724_CHKR_RPT_A			0x1074
#define MAX96724_CHKR_RPT_B			0x1075
#define MAX96724_CHKR_ALT			0x1076

#define MAX96724_DE_DET				0x11f0
#define MAX96724_HS_DET				0x11f1
#define MAX96724_VS_DET				0x11f2
#define MAX96724_HS_POL				0x11f3
#define MAX96724_VS_POL				0x11f4
#define MAX96724_DET(p)				BIT(p)

#define MAX96724_DPLL_0(x)			(0x1c00 + (x) * 0x100)
#define MAX96724_DPLL_0_CONFIG_SOFT_RST_N	BIT(0)

#define MAX96724_PHY1_ALT_CLOCK			5

#define MAX96724_NAME				"max96724"
#define MAX96724_PINCTRL_NAME			MAX96724_NAME "-pinctrl"
#define MAX96724_GPIOCHIP_NAME			MAX96724_NAME "-gpiochip"
#define MAX96724_GPIO_NUM			9

static const struct regmap_config max96724_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x1f00,
};

struct max96724_priv {
	struct max_des des;
	const struct max96724_chip_info *info;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct pinctrl_desc pctldesc;
	struct gpio_chip gc;
	struct pinctrl_dev *pctldev;

	struct gpio_desc *gpiod_enable;
};

struct max96724_chip_info {
	unsigned int versions;
	unsigned int modes;
	bool supports_pipe_stream_autoselect;
	unsigned int num_pipes;

	int (*set_pipe_phy)(struct max_des *des, struct max_des_pipe *pipe,
			    struct max_des_phy *phy);
	int (*set_pipe_tunnel_phy)(struct max_des *des, struct max_des_pipe *pipe,
				   struct max_des_phy *phy);
	int (*set_pipe_tunnel_enable)(struct max_des *des, struct max_des_pipe *pipe,
				      bool enable);
};

#define des_to_priv(_des) \
	container_of(_des, struct max96724_priv, des)

/* ---------- Pin definitions (MFP0–MFP8, 9 GPIO-capable pins) ---------- */

#define MAX96724_PIN(n) \
	PINCTRL_PIN(n, "mfp" __stringify(n))

static const struct pinctrl_pin_desc max96724_pins[] = {
	MAX96724_PIN(0),
	MAX96724_PIN(1),
	MAX96724_PIN(2),
	MAX96724_PIN(3),
	MAX96724_PIN(4),
	MAX96724_PIN(5),
	MAX96724_PIN(6),
	MAX96724_PIN(7),
	MAX96724_PIN(8),
};

#define MAX96724_GROUP_PINS(name, ...) \
	static const unsigned int name ## _pins[] = { __VA_ARGS__ }

MAX96724_GROUP_PINS(mfp0, 0);
MAX96724_GROUP_PINS(mfp1, 1);
MAX96724_GROUP_PINS(mfp2, 2);
MAX96724_GROUP_PINS(mfp3, 3);
MAX96724_GROUP_PINS(mfp4, 4);
MAX96724_GROUP_PINS(mfp5, 5);
MAX96724_GROUP_PINS(mfp6, 6);
MAX96724_GROUP_PINS(mfp7, 7);
MAX96724_GROUP_PINS(mfp8, 8);

#define MAX96724_GROUP(name) \
	PINCTRL_PINGROUP(__stringify(name), name ## _pins, ARRAY_SIZE(name ## _pins))

static const struct pingroup max96724_ctrl_groups[] = {
	MAX96724_GROUP(mfp0),
	MAX96724_GROUP(mfp1),
	MAX96724_GROUP(mfp2),
	MAX96724_GROUP(mfp3),
	MAX96724_GROUP(mfp4),
	MAX96724_GROUP(mfp5),
	MAX96724_GROUP(mfp6),
	MAX96724_GROUP(mfp7),
	MAX96724_GROUP(mfp8),
};

#define MAX96724_FUNC_GROUPS(name, ...) \
	static const char * const name ## _groups[] = { __VA_ARGS__ }

MAX96724_FUNC_GROUPS(gpio, "mfp0", "mfp1", "mfp2", "mfp3", "mfp4", "mfp5",
			   "mfp6", "mfp7", "mfp8");

enum max96724_func {
	max96724_func_gpio,
};

#define MAX96724_FUNC(name)						\
	[max96724_func_ ## name] =					\
		PINCTRL_PINFUNCTION(__stringify(name), name ## _groups,	\
				    ARRAY_SIZE(name ## _groups))

static const struct pinfunction max96724_functions[] = {
	MAX96724_FUNC(gpio),
};

/* ---------- Per-link GPIO forwarding pinctrl parameters ---------- */

enum max96724_pinctrl_params {
	MAX96724_PINCTRL_PULL_STRENGTH_WEAK = PIN_CONFIG_END + 1,
	MAX96724_PINCTRL_JITTER_COMPENSATION_EN,
	MAX96724_PINCTRL_GMSL_TX_EN_A,
	MAX96724_PINCTRL_GMSL_RX_EN_A,
	MAX96724_PINCTRL_GMSL_TX_ID_A,
	MAX96724_PINCTRL_GMSL_RX_ID_A,
	MAX96724_PINCTRL_GMSL_TX_EN_B,
	MAX96724_PINCTRL_GMSL_RX_EN_B,
	MAX96724_PINCTRL_GMSL_TX_ID_B,
	MAX96724_PINCTRL_GMSL_RX_ID_B,
	MAX96724_PINCTRL_GMSL_TX_EN_C,
	MAX96724_PINCTRL_GMSL_RX_EN_C,
	MAX96724_PINCTRL_GMSL_TX_ID_C,
	MAX96724_PINCTRL_GMSL_RX_ID_C,
	MAX96724_PINCTRL_GMSL_TX_EN_D,
	MAX96724_PINCTRL_GMSL_RX_EN_D,
	MAX96724_PINCTRL_GMSL_TX_ID_D,
	MAX96724_PINCTRL_GMSL_RX_ID_D,
	MAX96724_PINCTRL_INPUT_VALUE,
};

static const struct pinconf_generic_params max96724_cfg_params[] = {
	{ "maxim,pull-strength-weak", MAX96724_PINCTRL_PULL_STRENGTH_WEAK, 0 },
	{ "maxim,jitter-compensation", MAX96724_PINCTRL_JITTER_COMPENSATION_EN, 0 },
	{ "maxim,gmsl-tx-a", MAX96724_PINCTRL_GMSL_TX_EN_A, 0 },
	{ "maxim,gmsl-rx-a", MAX96724_PINCTRL_GMSL_RX_EN_A, 0 },
	{ "maxim,gmsl-tx-id-a", MAX96724_PINCTRL_GMSL_TX_ID_A, 0 },
	{ "maxim,gmsl-rx-id-a", MAX96724_PINCTRL_GMSL_RX_ID_A, 0 },
	{ "maxim,gmsl-tx-b", MAX96724_PINCTRL_GMSL_TX_EN_B, 0 },
	{ "maxim,gmsl-rx-b", MAX96724_PINCTRL_GMSL_RX_EN_B, 0 },
	{ "maxim,gmsl-tx-id-b", MAX96724_PINCTRL_GMSL_TX_ID_B, 0 },
	{ "maxim,gmsl-rx-id-b", MAX96724_PINCTRL_GMSL_RX_ID_B, 0 },
	{ "maxim,gmsl-tx-c", MAX96724_PINCTRL_GMSL_TX_EN_C, 0 },
	{ "maxim,gmsl-rx-c", MAX96724_PINCTRL_GMSL_RX_EN_C, 0 },
	{ "maxim,gmsl-tx-id-c", MAX96724_PINCTRL_GMSL_TX_ID_C, 0 },
	{ "maxim,gmsl-rx-id-c", MAX96724_PINCTRL_GMSL_RX_ID_C, 0 },
	{ "maxim,gmsl-tx-d", MAX96724_PINCTRL_GMSL_TX_EN_D, 0 },
	{ "maxim,gmsl-rx-d", MAX96724_PINCTRL_GMSL_RX_EN_D, 0 },
	{ "maxim,gmsl-tx-id-d", MAX96724_PINCTRL_GMSL_TX_ID_D, 0 },
	{ "maxim,gmsl-rx-id-d", MAX96724_PINCTRL_GMSL_RX_ID_D, 0 },
};

/* ---------- Pinctrl group/function ops ---------- */

static int max96724_ctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(max96724_ctrl_groups);
}

static const char *max96724_ctrl_get_group_name(struct pinctrl_dev *pctldev,
						unsigned int selector)
{
	return max96724_ctrl_groups[selector].name;
}

static int max96724_ctrl_get_group_pins(struct pinctrl_dev *pctldev,
					unsigned int selector,
					const unsigned int **pins,
					unsigned int *num_pins)
{
	*pins = (unsigned int *)max96724_ctrl_groups[selector].pins;
	*num_pins = max96724_ctrl_groups[selector].npins;

	return 0;
}

static int max96724_mux_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(max96724_functions);
}

static const char *max96724_mux_get_function_name(struct pinctrl_dev *pctldev,
						  unsigned int selector)
{
	return max96724_functions[selector].name;
}

static int max96724_mux_get_groups(struct pinctrl_dev *pctldev,
				   unsigned int selector,
				   const char * const **groups,
				   unsigned int * const num_groups)
{
	*groups = max96724_functions[selector].groups;
	*num_groups = max96724_functions[selector].ngroups;

	return 0;
}

static int max96724_mux_set(struct pinctrl_dev *pctldev, unsigned int selector,
			    unsigned int group)
{
	return 0;
}

/* ---------- Pin config register mapping ---------- */

/*
 * Per-GPIO base: 0x300 + offset * 3 (3 bytes per GPIO).
 *   Byte 0 (GPIO_A): OUT_DIS, TX_EN_A, RX_EN_A, GPIO_IN, GPIO_OUT,
 *                     TX_COMP_EN, RES_CFG
 *   Byte 1 (GPIO_B): TX_ID_A[4:0], OUT_TYPE, PULL_UPDN_SEL[7:6]
 *   Byte 2 (GPIO_C): RX_ID_A[4:0]
 *
 * Links B/C/D use separate register regions at non-uniform offsets.
 * The conditional offset adjustments match the MAX96724 register map.
 */
static int max96724_get_pin_config_reg(unsigned int offset, u32 param,
				       unsigned int *reg, unsigned int *mask,
				       unsigned int *val)
{
	*reg = 0x300 + offset * 0x3;

	switch (param) {
	case PIN_CONFIG_OUTPUT_ENABLE:
		*mask = BIT(0);
		*val = 0b0;
		return 0;
	case PIN_CONFIG_INPUT_ENABLE:
		*mask = BIT(0);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_EN_A:
		*mask = BIT(1);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_EN_A:
		*mask = BIT(2);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_INPUT_VALUE:
		*mask = BIT(3);
		*val = 0b1;
		return 0;
	case PIN_CONFIG_OUTPUT:
		*mask = BIT(4);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_JITTER_COMPENSATION_EN:
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_PULL_STRENGTH_WEAK:
		*mask = BIT(7);
		*val = 0b0;
		return 0;

	/* Link A TX_ID / drive / bias / RX_ID — all in bytes 1-2 */
	case MAX96724_PINCTRL_GMSL_TX_ID_A:
		*reg += 1;
		*mask = GENMASK(4, 0);
		return 0;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		*reg += 1;
		*mask = BIT(5);
		*val = 0b0;
		return 0;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		*reg += 1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case PIN_CONFIG_BIAS_DISABLE:
		*reg += 1;
		*mask = GENMASK(7, 6);
		*val = 0b00;
		return 0;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		*reg += 1;
		*mask = GENMASK(7, 6);
		*val = 0b10;
		return 0;
	case PIN_CONFIG_BIAS_PULL_UP:
		*reg += 1;
		*mask = GENMASK(7, 6);
		*val = 0b01;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_ID_A:
		*reg += 2;
		*mask = GENMASK(4, 0);
		return 0;

	/* Link B TX — non-uniform offsets from base */
	case MAX96724_PINCTRL_GMSL_TX_EN_B:
		*reg += 0x37;
		if (offset > 2)
			*reg += 1;
		if (offset > 7)
			*reg += 1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_ID_B:
		*reg += 0x37;
		if (offset > 2)
			*reg += 1;
		if (offset > 7)
			*reg += 1;
		*mask = GENMASK(4, 0);
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_EN_B:
		*reg += 0x38;
		if (offset > 2)
			*reg += 1;
		if (offset > 7)
			*reg += 1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_ID_B:
		*reg += 0x38;
		if (offset > 2)
			*reg += 1;
		if (offset > 7)
			*reg += 1;
		*mask = GENMASK(4, 0);
		return 0;

	/* Link C */
	case MAX96724_PINCTRL_GMSL_TX_EN_C:
		*reg += 0x6d;
		if (offset > 0)
			*reg += 1;
		if (offset > 5)
			*reg += 1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_ID_C:
		*reg += 0x6d;
		if (offset > 0)
			*reg += 1;
		if (offset > 5)
			*reg += 1;
		*mask = GENMASK(4, 0);
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_EN_C:
		*reg += 0x6e;
		if (offset > 0)
			*reg += 1;
		if (offset > 5)
			*reg += 1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_ID_C:
		*reg += 0x6e;
		if (offset > 0)
			*reg += 1;
		if (offset > 5)
			*reg += 1;
		*mask = GENMASK(4, 0);
		return 0;

	/* Link D */
	case MAX96724_PINCTRL_GMSL_TX_EN_D:
		*reg += 0xa4;
		if (offset > 3)
			*reg += 1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_TX_ID_D:
		*reg += 0xa4;
		if (offset > 3)
			*reg += 1;
		*mask = GENMASK(4, 0);
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_EN_D:
		*reg += 0xa5;
		if (offset > 3)
			*reg += 1;
		*mask = BIT(5);
		*val = 0b1;
		return 0;
	case MAX96724_PINCTRL_GMSL_RX_ID_D:
		*reg += 0xa5;
		if (offset > 3)
			*reg += 1;
		*mask = GENMASK(4, 0);
		return 0;

	default:
		return -ENOTSUPP;
	}
}

/* ---------- Pin config get/set ---------- */

static int max96724_conf_pin_config_get(struct pinctrl_dev *pctldev,
					unsigned int offset,
					unsigned long *config)
{
	struct max96724_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	u32 param = pinconf_to_config_param(*config);
	unsigned int reg, mask, val, en_val;
	int ret;

	ret = max96724_get_pin_config_reg(offset, param, &reg, &mask, &en_val);
	if (ret)
		return ret;

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_PUSH_PULL:
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
		ret = regmap_read(priv->regmap, reg, &val);
		if (ret)
			return ret;

		val = field_get(mask, val) == en_val;
		if (!val)
			return -EINVAL;

		break;
	case MAX96724_PINCTRL_JITTER_COMPENSATION_EN:
	case MAX96724_PINCTRL_PULL_STRENGTH_WEAK:
	case MAX96724_PINCTRL_GMSL_TX_EN_A:
	case MAX96724_PINCTRL_GMSL_TX_EN_B:
	case MAX96724_PINCTRL_GMSL_TX_EN_C:
	case MAX96724_PINCTRL_GMSL_TX_EN_D:
	case MAX96724_PINCTRL_GMSL_RX_EN_A:
	case MAX96724_PINCTRL_GMSL_RX_EN_B:
	case MAX96724_PINCTRL_GMSL_RX_EN_C:
	case MAX96724_PINCTRL_GMSL_RX_EN_D:
	case MAX96724_PINCTRL_INPUT_VALUE:
	case PIN_CONFIG_OUTPUT_ENABLE:
	case PIN_CONFIG_INPUT_ENABLE:
	case PIN_CONFIG_OUTPUT:
		ret = regmap_read(priv->regmap, reg, &val);
		if (ret)
			return ret;

		val = field_get(mask, val) == en_val;
		break;
	case MAX96724_PINCTRL_GMSL_TX_ID_A:
	case MAX96724_PINCTRL_GMSL_TX_ID_B:
	case MAX96724_PINCTRL_GMSL_TX_ID_C:
	case MAX96724_PINCTRL_GMSL_TX_ID_D:
	case MAX96724_PINCTRL_GMSL_RX_ID_A:
	case MAX96724_PINCTRL_GMSL_RX_ID_B:
	case MAX96724_PINCTRL_GMSL_RX_ID_C:
	case MAX96724_PINCTRL_GMSL_RX_ID_D:
		ret = regmap_read(priv->regmap, reg, &val);
		if (ret)
			return ret;

		val = field_get(mask, val);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, val);

	return 0;
}

static int max96724_conf_pin_config_set_one(struct max96724_priv *priv,
					    unsigned int offset,
					    unsigned long config)
{
	u32 param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	unsigned int reg, mask, val, en_val;
	int ret;

	ret = max96724_get_pin_config_reg(offset, param, &reg, &mask, &en_val);
	if (ret)
		return ret;

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_PUSH_PULL:
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
		val = field_prep(mask, en_val);
		ret = regmap_update_bits(priv->regmap, reg, mask, val);
		break;
	case MAX96724_PINCTRL_JITTER_COMPENSATION_EN:
	case MAX96724_PINCTRL_PULL_STRENGTH_WEAK:
	case MAX96724_PINCTRL_GMSL_TX_EN_A:
	case MAX96724_PINCTRL_GMSL_TX_EN_B:
	case MAX96724_PINCTRL_GMSL_TX_EN_C:
	case MAX96724_PINCTRL_GMSL_TX_EN_D:
	case MAX96724_PINCTRL_GMSL_RX_EN_A:
	case MAX96724_PINCTRL_GMSL_RX_EN_B:
	case MAX96724_PINCTRL_GMSL_RX_EN_C:
	case MAX96724_PINCTRL_GMSL_RX_EN_D:
	case PIN_CONFIG_OUTPUT_ENABLE:
	case PIN_CONFIG_INPUT_ENABLE:
	case PIN_CONFIG_OUTPUT:
		val = field_prep(mask, arg ? en_val : ~en_val);
		ret = regmap_update_bits(priv->regmap, reg, mask, val);
		break;
	case MAX96724_PINCTRL_GMSL_TX_ID_A:
	case MAX96724_PINCTRL_GMSL_TX_ID_B:
	case MAX96724_PINCTRL_GMSL_TX_ID_C:
	case MAX96724_PINCTRL_GMSL_TX_ID_D:
	case MAX96724_PINCTRL_GMSL_RX_ID_A:
	case MAX96724_PINCTRL_GMSL_RX_ID_B:
	case MAX96724_PINCTRL_GMSL_RX_ID_C:
	case MAX96724_PINCTRL_GMSL_RX_ID_D:
		val = field_prep(mask, arg);
		ret = regmap_update_bits(priv->regmap, reg, mask, val);
		break;
	default:
		return -ENOTSUPP;
	}

	if (ret)
		return ret;

	/* Setting OUTPUT auto-enables OUTPUT_ENABLE */
	if (param == PIN_CONFIG_OUTPUT) {
		config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, 1);
		ret = max96724_conf_pin_config_set_one(priv, offset, config);
		if (ret)
			return ret;
	}

	/* Enable jitter compensation on all links when set on link A */
	if (param == MAX96724_PINCTRL_JITTER_COMPENSATION_EN) {
		unsigned int jreg, jmask, jval;

		max96724_get_pin_config_reg(offset,
					    MAX96724_PINCTRL_GMSL_TX_ID_B,
					    &jreg, &jmask, &jval);
		regmap_update_bits(priv->regmap, jreg, BIT(6), BIT(6));

		max96724_get_pin_config_reg(offset,
					    MAX96724_PINCTRL_GMSL_TX_ID_C,
					    &jreg, &jmask, &jval);
		regmap_update_bits(priv->regmap, jreg, BIT(6), BIT(6));

		max96724_get_pin_config_reg(offset,
					    MAX96724_PINCTRL_GMSL_TX_ID_D,
					    &jreg, &jmask, &jval);
		regmap_update_bits(priv->regmap, jreg, BIT(6), BIT(6));
	}

	return 0;
}

static int max96724_conf_pin_config_set(struct pinctrl_dev *pctldev,
					unsigned int offset,
					unsigned long *configs,
					unsigned int num_configs)
{
	struct max96724_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	int ret;

	while (num_configs--) {
		ret = max96724_conf_pin_config_set_one(priv, offset, *configs);
		if (ret)
			return ret;

		configs++;
	}

	return 0;
}

/* ---------- GPIO chip operations ---------- */

static int max96724_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT_ENABLE, 0);
	struct max96724_priv *priv = gpiochip_get_data(gc);
	int ret;

	ret = max96724_conf_pin_config_get(priv->pctldev, offset, &config);
	if (ret)
		return ret;

	return pinconf_to_config_argument(config) ? GPIO_LINE_DIRECTION_OUT
						  : GPIO_LINE_DIRECTION_IN;
}

static int max96724_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_INPUT_ENABLE, 1);
	struct max96724_priv *priv = gpiochip_get_data(gc);

	return max96724_conf_pin_config_set_one(priv, offset, config);
}

static int max96724_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
					  int value)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, value);
	struct max96724_priv *priv = gpiochip_get_data(gc);

	return max96724_conf_pin_config_set_one(priv, offset, config);
}

static int max96724_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	unsigned long config = pinconf_to_config_packed(MAX96724_PINCTRL_INPUT_VALUE, 0);
	struct max96724_priv *priv = gpiochip_get_data(gc);
	int ret;

	ret = max96724_conf_pin_config_get(priv->pctldev, offset, &config);
	if (ret)
		return ret;

	return pinconf_to_config_argument(config);
}

static int max96724_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	unsigned long config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, value);
	struct max96724_priv *priv = gpiochip_get_data(gc);

	return max96724_conf_pin_config_set_one(priv, offset, config);
}

/* ---------- Pinctrl/pinmux/pinconf ops structs ---------- */

static const struct pinctrl_ops max96724_ctrl_ops = {
	.get_groups_count = max96724_ctrl_get_groups_count,
	.get_group_name = max96724_ctrl_get_group_name,
	.get_group_pins = max96724_ctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinconf_ops max96724_conf_ops = {
	.pin_config_get = max96724_conf_pin_config_get,
	.pin_config_set = max96724_conf_pin_config_set,
	.is_generic = true,
};

static const struct pinmux_ops max96724_mux_ops = {
	.get_functions_count = max96724_mux_get_functions_count,
	.get_function_name = max96724_mux_get_function_name,
	.get_function_groups = max96724_mux_get_groups,
	.set_mux = max96724_mux_set,
	.strict = true,
};

/* ---------- GPIO chip probe ---------- */

static int max96724_gpiochip_probe(struct max96724_priv *priv)
{
	struct device *dev = priv->dev;
	int ret;

	priv->pctldesc = (struct pinctrl_desc) {
		.owner = THIS_MODULE,
		.name = MAX96724_PINCTRL_NAME,
		.pins = max96724_pins,
		.npins = ARRAY_SIZE(max96724_pins),
		.pctlops = &max96724_ctrl_ops,
		.confops = &max96724_conf_ops,
		.pmxops = &max96724_mux_ops,
		.custom_params = max96724_cfg_params,
		.num_custom_params = ARRAY_SIZE(max96724_cfg_params),
	};

	ret = devm_pinctrl_register_and_init(dev, &priv->pctldesc, priv, &priv->pctldev);
	if (ret)
		return ret;

	ret = pinctrl_enable(priv->pctldev);
	if (ret)
		return ret;

	priv->gc = (struct gpio_chip) {
		.owner = THIS_MODULE,
		.label = MAX96724_GPIOCHIP_NAME,
		.base = -1,
		.ngpio = MAX96724_GPIO_NUM,
		.parent = dev,
		.can_sleep = true,
		.request = gpiochip_generic_request,
		.free = gpiochip_generic_free,
		.set_config = gpiochip_generic_config,
		.get_direction = max96724_gpio_get_direction,
		.direction_input = max96724_gpio_direction_input,
		.direction_output = max96724_gpio_direction_output,
		.get = max96724_gpio_get,
		.set_rv = max96724_gpio_set,
	};

	return devm_gpiochip_add_data(dev, &priv->gc, priv);
}

/* -------------------------------------------------------------------- */

static int max96724_wait_for_device(struct max96724_priv *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < 10; i++) {
		unsigned int val;

		ret = regmap_read(priv->regmap, MAX96724_REG0, &val);
		if (!ret && val)
			return 0;

		msleep(100);

		dev_err(priv->dev, "Retry %u waiting for deserializer: %d\n", i, ret);
	}

	return ret;
}

static int max96724_reset(struct max96724_priv *priv)
{
	int ret;

	ret = max96724_wait_for_device(priv);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, MAX96724_PWR1,
				 MAX96724_PWR1_RESET_ALL,
				 FIELD_PREP(MAX96724_PWR1_RESET_ALL, 1));
	if (ret)
		return ret;

	fsleep(10000);

	return max96724_wait_for_device(priv);
}

static int __maybe_unused max96724_reg_read(struct max_des *des, unsigned int reg,
					    unsigned int *val)
{
	struct max96724_priv *priv = des_to_priv(des);

	return regmap_read(priv->regmap, reg, val);
}

static int __maybe_unused max96724_reg_write(struct max_des *des, unsigned int reg,
					     unsigned int val)
{
	struct max96724_priv *priv = des_to_priv(des);

	return regmap_write(priv->regmap, reg, val);
}

static unsigned int max96724_phy_id(struct max_des *des, struct max_des_phy *phy)
{
	unsigned int num_hw_data_lanes = max_des_phy_hw_data_lanes(des, phy);

	/* PHY 1 is the master PHY when combining PHY 0 and PHY 1. */
	if (phy->index == 0 && num_hw_data_lanes == 4)
		return 1;

	if (phy->index == 1 && !des->phys[1].enabled)
		return 0;

	return phy->index;
}

static int max96724_log_pipe_status(struct max_des *des,
				    struct max_des_pipe *pipe)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;
	unsigned int val, mask;
	int ret;

	ret = regmap_read(priv->regmap, MAX96724_VPRBS(index), &val);
	if (ret)
		return ret;

	dev_info(priv->dev, "\tvideo_lock: %u\n",
		 !!(val & MAX96724_VPRBS_VIDEO_LOCK));

	mask = MAX96724_DET(index);

	ret = regmap_read(priv->regmap, MAX96724_DE_DET, &val);
	if (ret)
		return ret;

	dev_info(priv->dev, "\tde_det: %u\n", !!(val & mask));

	ret = regmap_read(priv->regmap, MAX96724_HS_DET, &val);
	if (ret)
		return ret;

	dev_info(priv->dev, "\ths_det: %u\n", !!(val & mask));

	ret = regmap_read(priv->regmap, MAX96724_VS_DET, &val);
	if (ret)
		return ret;

	dev_info(priv->dev, "\tvs_det: %u\n", !!(val & mask));

	ret = regmap_read(priv->regmap, MAX96724_HS_POL, &val);
	if (ret)
		return ret;

	dev_info(priv->dev, "\ths_pol: %u\n", !!(val & mask));

	ret = regmap_read(priv->regmap, MAX96724_VS_POL, &val);
	if (ret)
		return ret;

	dev_info(priv->dev, "\tvs_pol: %u\n", !!(val & mask));

	return 0;
}

static int max96724_log_phy_status(struct max_des *des,
				   struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = max96724_phy_id(des, phy);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, MAX96724_MIPI_PHY25(index), &val);
	if (ret)
		return ret;

	dev_info(priv->dev, "\tcsi2_pkt_cnt: %lu\n",
		 field_get(MAX96724_MIPI_PHY25_CSI2_TX_PKT_CNT(index), val));

	ret = regmap_read(priv->regmap, MAX96724_MIPI_PHY27(index), &val);
	if (ret)
		return ret;

	dev_info(priv->dev, "\tphy_pkt_cnt: %lu\n",
		 field_get(MAX96724_MIPI_PHY27_PHY_PKT_CNT(index), val));

	return 0;
}

static int max96724_set_enable(struct max_des *des, bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);

	return regmap_assign_bits(priv->regmap, MAX96724_BACKTOP12,
				  MAX96724_BACKTOP12_CSI_OUT_EN, enable);
}

static const unsigned int max96724_phys_configs_reg_val[] = {
	MAX96724_MIPI_PHY0_PHY_1X4A_2X2,
	MAX96724_MIPI_PHY0_PHY_2X4,

	MAX96724_MIPI_PHY0_PHY_4X2,
	MAX96724_MIPI_PHY0_PHY_1X4A_2X2,
	MAX96724_MIPI_PHY0_PHY_1X4B_2X2,
	MAX96724_MIPI_PHY0_PHY_2X4,
};

static const struct max_serdes_phys_config max96724_phys_configs[] = {
	/*
	 * PHY 1 can be in 4-lane mode (combining lanes of PHY 0 and PHY 1)
	 * but only use the data lanes of PHY0, while continuing to use the
	 * clock lane of PHY 1.
	 * Specifying clock-lanes as 5 turns on alternate clocking mode.
	 */
	{ { 2, 0, 2, 2 }, { MAX96724_PHY1_ALT_CLOCK, 0, 0, 0 } },
	{ { 2, 0, 4, 0 }, { MAX96724_PHY1_ALT_CLOCK, 0, 0, 0 } },

	/*
	 * When combining PHY 0 and PHY 1 to make them function in 4-lane mode,
	 * PHY 1 is the master PHY, but we use PHY 0 here to maintain
	 * compatibility.
	 */
	{ { 2, 2, 2, 2 } },
	{ { 4, 0, 2, 2 } },
	{ { 2, 2, 4, 0 } },
	{ { 4, 0, 4, 0 } },
};

static int max96724_init_tpg(struct max_des *des)
{
	const struct reg_sequence regs[] = {
		{ MAX96724_GRAD_INCR, MAX_SERDES_GRAD_INCR },
		REG_SEQUENCE_3_LE(MAX96724_CHKR_COLOR_A_L,
				  MAX_SERDES_CHECKER_COLOR_A),
		REG_SEQUENCE_3_LE(MAX96724_CHKR_COLOR_B_L,
				  MAX_SERDES_CHECKER_COLOR_B),
		{ MAX96724_CHKR_RPT_A, MAX_SERDES_CHECKER_SIZE },
		{ MAX96724_CHKR_RPT_B, MAX_SERDES_CHECKER_SIZE },
		{ MAX96724_CHKR_ALT, MAX_SERDES_CHECKER_SIZE },
	};
	struct max96724_priv *priv = des_to_priv(des);

	return regmap_multi_reg_write(priv->regmap, regs, ARRAY_SIZE(regs));
}

static int max96724_init(struct max_des *des)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int i;
	int ret;

	if (priv->info->set_pipe_tunnel_enable) {
		for (i = 0; i < des->ops->num_pipes; i++) {
			ret = regmap_set_bits(priv->regmap, MAX96724_MIPI_TX57(i),
					      MAX96724_MIPI_TX57_DIS_AUTO_TUN_DET);
			if (ret)
				return ret;
		}
	}

	if (priv->info->supports_pipe_stream_autoselect) {
		/* Enable stream autoselect. */
		ret = regmap_set_bits(priv->regmap, MAX96724_VIDEO_PIPE_EN,
				      MAX96724_VIDEO_PIPE_EN_STREAM_SEL_ALL);
		if (ret)
			return ret;
	}

	/* Set PHY mode. */
	ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY0,
				 MAX96724_MIPI_PHY0_PHY_CONFIG,
				 max96724_phys_configs_reg_val[des->phys_config]);
	if (ret)
		return ret;

	return max96724_init_tpg(des);
}

static int max96724_init_phy(struct max_des *des, struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	bool is_cphy = phy->bus_type == V4L2_MBUS_CSI2_CPHY;
	unsigned int num_data_lanes = phy->mipi.num_data_lanes;
	unsigned int dpll_freq = phy->link_frequency * 2;
	unsigned int num_hw_data_lanes;
	unsigned int index;
	unsigned int used_data_lanes = 0;
	unsigned int val, mask;
	unsigned int i;
	int ret;

	index = max96724_phy_id(des, phy);
	num_hw_data_lanes = max_des_phy_hw_data_lanes(des, phy);

	ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_TX10(index),
				 MAX96724_MIPI_TX10_CSI2_LANE_CNT,
				 FIELD_PREP(MAX96724_MIPI_TX10_CSI2_LANE_CNT,
					    num_data_lanes - 1));
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX10(index),
				 MAX96724_MIPI_TX10_CSI2_CPHY_EN, is_cphy);
	if (ret)
		return ret;

	/* Configure lane mapping. */
	val = 0;
	for (i = 0; i < num_hw_data_lanes ; i++) {
		unsigned int map;

		if (i < num_data_lanes)
			map = phy->mipi.data_lanes[i] - 1;
		else
			map = ffz(used_data_lanes);

		val |= map << (i * 2);
		used_data_lanes |= BIT(map);
	}

	if (num_hw_data_lanes == 4)
		mask = MAX96724_MIPI_PHY3_PHY_LANE_MAP_4;
	else
		mask = MAX96724_MIPI_PHY3_PHY_LANE_MAP_2(index);

	ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY3(index),
				 mask, field_prep(mask, val));
	if (ret)
		return ret;

	/* Configure lane polarity. */
	for (i = 0, val = 0; i < num_data_lanes; i++)
		if (phy->mipi.lane_polarities[i + 1])
			val |= BIT(i);

	if (num_hw_data_lanes == 4) {
		ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY5(index),
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_4_0_1 |
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_4_2_3,
					 FIELD_PREP(MAX96724_MIPI_PHY5_PHY_POL_MAP_4_0_1,
						    val) |
					 FIELD_PREP(MAX96724_MIPI_PHY5_PHY_POL_MAP_4_2_3,
						    val >> 2));
		if (ret)
			return ret;

		ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_PHY5(index),
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_4_CLK,
					 phy->mipi.lane_polarities[0]);
		if (ret)
			return ret;
	} else {
		ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY5(index),
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_2(index),
					 field_prep(MAX96724_MIPI_PHY5_PHY_POL_MAP_2(index), val));
		if (ret)
			return ret;

		ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_PHY5(index),
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_2_CLK(index),
					 phy->mipi.lane_polarities[0]);
		if (ret)
			return ret;
	}

	if (!is_cphy && dpll_freq > 1500000000ull) {
		/* Enable initial deskew with 2 x 32k UI. */
		ret = regmap_write(priv->regmap, MAX96724_MIPI_TX3(index),
				   MAX96724_MIPI_TX3_DESKEW_INIT_AUTO |
				   MAX96724_MIPI_TX3_DESKEW_INIT_8X32K);
		if (ret)
			return ret;

		/* Enable periodic deskew with 2 x 1k UI.. */
		ret = regmap_write(priv->regmap, MAX96724_MIPI_TX4(index),
				   MAX96724_MIPI_TX4_DESKEW_PER_AUTO |
				   MAX96724_MIPI_TX4_DESKEW_PER_2K);
		if (ret)
			return ret;
	} else {
		/* Disable initial deskew. */
		ret = regmap_write(priv->regmap, MAX96724_MIPI_TX3(index), 0x0);
		if (ret)
			return ret;

		/* Disable periodic deskew. */
		ret = regmap_write(priv->regmap, MAX96724_MIPI_TX4(index), 0x0);
		if (ret)
			return ret;
	}

	if (is_cphy) {
		/* Configure C-PHY timings. */
		ret = regmap_write(priv->regmap, MAX96724_MIPI_PHY13,
				   MAX96724_MIPI_PHY13_T_T3_PREBEGIN_64X7);
		if (ret)
			return ret;

		ret = regmap_write(priv->regmap, MAX96724_MIPI_PHY14,
				   MAX96724_MIPI_PHY14_T_T3_PREP_55NS |
				   MAX96724_MIPI_PHY14_T_T3_POST_32X7);
		if (ret)
			return ret;
	}

	/* Put DPLL block into reset. */
	ret = regmap_clear_bits(priv->regmap, MAX96724_DPLL_0(index),
				MAX96724_DPLL_0_CONFIG_SOFT_RST_N);
	if (ret)
		return ret;

	/* Set DPLL frequency. */
	ret = regmap_update_bits(priv->regmap, MAX96724_BACKTOP22(index),
				 MAX96724_BACKTOP22_PHY_CSI_TX_DPLL,
				 FIELD_PREP(MAX96724_BACKTOP22_PHY_CSI_TX_DPLL,
					    div_u64(dpll_freq, 100000000)));
	if (ret)
		return ret;

	/* Enable DPLL frequency. */
	ret = regmap_set_bits(priv->regmap, MAX96724_BACKTOP22(index),
			      MAX96724_BACKTOP22_PHY_CSI_TX_DPLL_EN);
	if (ret)
		return ret;

	/* Pull DPLL block out of reset. */
	return regmap_set_bits(priv->regmap, MAX96724_DPLL_0(index),
			       MAX96724_DPLL_0_CONFIG_SOFT_RST_N);
}

static int max96724_set_phy_mode(struct max_des *des, struct max_des_phy *phy,
				 struct max_des_phy_mode *mode)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = max96724_phy_id(des, phy);
	int ret;

	/* Set alternate memory map modes. */
	ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX51(index),
				 MAX96724_MIPI_TX51_ALT_MEM_MAP_12,
				 mode->alt_mem_map12);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX51(index),
				 MAX96724_MIPI_TX51_ALT_MEM_MAP_8,
				 mode->alt_mem_map8);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX51(index),
				 MAX96724_MIPI_TX51_ALT_MEM_MAP_10,
				 mode->alt_mem_map10);
	if (ret)
		return ret;

	return regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX51(index),
				  MAX96724_MIPI_TX51_ALT2_MEM_MAP_8,
				  mode->alt2_mem_map8);
}

static int max96724_set_phy_enable(struct max_des *des, struct max_des_phy *phy,
				   bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = max96724_phy_id(des, phy);
	unsigned int num_hw_data_lanes;
	unsigned int mask;

	num_hw_data_lanes = max_des_phy_hw_data_lanes(des, phy);

	if (num_hw_data_lanes == 4)
		/* PHY 1 -> bits [1:0] */
		/* PHY 2 -> bits [3:2] */
		mask = MAX96724_MIPI_PHY2_PHY_STDB_N_4(index);
	else
		mask = MAX96724_MIPI_PHY2_PHY_STDB_N_2(index);

	return regmap_assign_bits(priv->regmap, MAX96724_MIPI_PHY2, mask, enable);
}

static int max96724_set_pipe_remap(struct max_des *des,
				   struct max_des_pipe *pipe,
				   unsigned int i,
				   struct max_des_remap *remap)
{
	struct max96724_priv *priv = des_to_priv(des);
	struct max_des_phy *phy = &des->phys[remap->phy];
	unsigned int phy_id = max96724_phy_id(des, phy);
	unsigned int index = pipe->index;
	int ret;

	/* Set source Data Type and Virtual Channel. */
	/* TODO: implement extended Virtual Channel. */
	ret = regmap_write(priv->regmap, MAX96724_MIPI_TX13(index, i),
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_DT,
				      remap->from_dt) |
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_VC,
				      remap->from_vc));
	if (ret)
		return ret;

	/* Set destination Data Type and Virtual Channel. */
	/* TODO: implement extended Virtual Channel. */
	ret = regmap_write(priv->regmap, MAX96724_MIPI_TX14(index, i),
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_DT,
				      remap->to_dt) |
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_VC,
				      remap->to_vc));
	if (ret)
		return ret;

	/* Set destination PHY. */
	return regmap_update_bits(priv->regmap, MAX96724_MIPI_TX45(index, i),
				  MAX96724_MIPI_TX45_MAP_DPHY_DEST(i),
				  field_prep(MAX96724_MIPI_TX45_MAP_DPHY_DEST(i),
					     phy_id));
}

static int max96724_set_pipe_remaps_enable(struct max_des *des,
					   struct max_des_pipe *pipe,
					   unsigned int mask)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;
	int ret;

	ret = regmap_write(priv->regmap, MAX96724_MIPI_TX11(index), mask);
	if (ret)
		return ret;

	return regmap_write(priv->regmap, MAX96724_MIPI_TX12(index), mask >> 8);
}

static int max96724_set_pipe_tunnel_phy(struct max_des *des,
					struct max_des_pipe *pipe,
					struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int phy_index = max96724_phy_id(des, phy);

	return regmap_update_bits(priv->regmap, MAX96724_MIPI_TX57(pipe->index),
				  MAX96724_MIPI_TX57_TUN_DEST,
				  FIELD_PREP(MAX96724_MIPI_TX57_TUN_DEST,
					     phy_index));
}

static int max96724_set_pipe_phy(struct max_des *des, struct max_des_pipe *pipe,
				 struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int phy_index = max96724_phy_id(des, phy);

	return regmap_update_bits(priv->regmap, MAX96724_MIPI_CTRL_SEL,
				  MAX96724_MIPI_CTRL_SEL_MASK(pipe->index),
				  field_prep(MAX96724_MIPI_CTRL_SEL_MASK(pipe->index),
					     phy_index));
}

static int max96724_set_pipe_enable(struct max_des *des, struct max_des_pipe *pipe,
				    bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;

	return regmap_assign_bits(priv->regmap, MAX96724_VIDEO_PIPE_EN,
				  MAX96724_VIDEO_PIPE_EN_MASK(index), enable);
}

static int max96724_set_pipe_stream_id(struct max_des *des, struct max_des_pipe *pipe,
				       unsigned int stream_id)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;

	return regmap_update_bits(priv->regmap, MAX96724_VIDEO_PIPE_SEL(index),
				  MAX96724_VIDEO_PIPE_SEL_STREAM(index),
				  field_prep(MAX96724_VIDEO_PIPE_SEL_STREAM(index),
					     stream_id));
}

static int max96724_set_pipe_link(struct max_des *des, struct max_des_pipe *pipe,
				  struct max_des_link *link)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;

	return regmap_update_bits(priv->regmap, MAX96724_VIDEO_PIPE_SEL(index),
				  MAX96724_VIDEO_PIPE_SEL_LINK(index),
				  field_prep(MAX96724_VIDEO_PIPE_SEL_LINK(index),
					     link->index));
}

static int max96724_set_pipe_mode(struct max_des *des,
				  struct max_des_pipe *pipe,
				  struct max_des_pipe_mode *mode)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;
	unsigned int reg, mask, mode_mask;
	int ret;

	/* Set 8bit double mode. */
	ret = regmap_assign_bits(priv->regmap, MAX96724_BACKTOP21(index),
				 MAX96724_BACKTOP21_BPP8DBL(index), mode->dbl8);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96724_BACKTOP24(index),
				 MAX96724_BACKTOP24_BPP8DBL_MODE(index),
				 mode->dbl8mode);
	if (ret)
		return ret;

	/* Set 10bit double mode. */
	if (index % 4 == 3) {
		reg = MAX96724_BACKTOP30(index);
		mask = MAX96724_BACKTOP30_BPP10DBL3;
		mode_mask = MAX96724_BACKTOP30_BPP10DBL3_MODE;
	} else if (index % 4 == 2) {
		reg = MAX96724_BACKTOP31(index);
		mask = MAX96724_BACKTOP31_BPP10DBL2;
		mode_mask = MAX96724_BACKTOP31_BPP10DBL2_MODE;
	} else if (index % 4 == 1) {
		reg = MAX96724_BACKTOP32(index);
		mask = MAX96724_BACKTOP32_BPP10DBL1;
		mode_mask = MAX96724_BACKTOP32_BPP10DBL1_MODE;
	} else {
		reg = MAX96724_BACKTOP32(index);
		mask = MAX96724_BACKTOP32_BPP10DBL0;
		mode_mask = MAX96724_BACKTOP32_BPP10DBL0_MODE;
	}

	ret = regmap_assign_bits(priv->regmap, reg, mask, mode->dbl10);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, reg, mode_mask, mode->dbl10mode);
	if (ret)
		return ret;

	/* Set 12bit double mode. */
	return regmap_assign_bits(priv->regmap, MAX96724_BACKTOP32(index),
				  MAX96724_BACKTOP32_BPP12(index), mode->dbl12);
}

static int max96724_set_pipe_tunnel_enable(struct max_des *des,
					   struct max_des_pipe *pipe, bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);

	return regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX54(pipe->index),
				  MAX96724_MIPI_TX54_TUN_EN, enable);
}

static int max96724_select_links(struct max_des *des, unsigned int mask)
{
	struct max96724_priv *priv = des_to_priv(des);
	int ret;

	ret = regmap_update_bits(priv->regmap, MAX96724_REG6, MAX96724_REG6_LINK_EN,
				 field_prep(MAX96724_REG6_LINK_EN, mask));
	if (ret)
		return ret;

	ret = regmap_set_bits(priv->regmap, MAX96724_CTRL1,
			      MAX96724_CTRL1_RESET_ONESHOT);
	if (ret)
		return ret;

	msleep(60);

	return 0;
}

static int max96724_set_link_version(struct max_des *des,
				     struct max_des_link *link,
				     enum max_serdes_gmsl_version version)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = link->index;
	unsigned int val;

	if (version == MAX_SERDES_GMSL_2_6GBPS)
		val = MAX96724_REG26_RX_RATE_6GBPS;
	else
		val = MAX96724_REG26_RX_RATE_3GBPS;

	return regmap_update_bits(priv->regmap, MAX96724_REG26(index),
				  MAX96724_REG26_RX_RATE_PHY(index),
				  field_prep(MAX96724_REG26_RX_RATE_PHY(index), val));
}

static int max96724_set_tpg_timings(struct max96724_priv *priv,
				    const struct max_serdes_tpg_timings *tm)
{
	const struct reg_sequence regs[] = {
		REG_SEQUENCE_3(MAX96724_VS_DLY_2, tm->vs_dly),
		REG_SEQUENCE_3(MAX96724_VS_HIGH_2, tm->vs_high),
		REG_SEQUENCE_3(MAX96724_VS_LOW_2, tm->vs_low),
		REG_SEQUENCE_3(MAX96724_V2H_2, tm->v2h),
		REG_SEQUENCE_2(MAX96724_HS_HIGH_1, tm->hs_high),
		REG_SEQUENCE_2(MAX96724_HS_LOW_1, tm->hs_low),
		REG_SEQUENCE_2(MAX96724_HS_CNT_1, tm->hs_cnt),
		REG_SEQUENCE_3(MAX96724_V2D_2, tm->v2d),
		REG_SEQUENCE_2(MAX96724_DE_HIGH_1, tm->de_high),
		REG_SEQUENCE_2(MAX96724_DE_LOW_1, tm->de_low),
		REG_SEQUENCE_2(MAX96724_DE_CNT_1, tm->de_cnt),
	};
	int ret;

	ret = regmap_multi_reg_write(priv->regmap, regs, ARRAY_SIZE(regs));
	if (ret)
		return ret;

	return regmap_write(priv->regmap, MAX96724_PATGEN_0,
			    FIELD_PREP(MAX96724_PATGEN_0_VTG_MODE,
				       MAX96724_PATGEN_0_VTG_MODE_FREE_RUNNING) |
			    FIELD_PREP(MAX96724_PATGEN_0_DE_INV, tm->de_inv) |
			    FIELD_PREP(MAX96724_PATGEN_0_HS_INV, tm->hs_inv) |
			    FIELD_PREP(MAX96724_PATGEN_0_VS_INV, tm->vs_inv) |
			    FIELD_PREP(MAX96724_PATGEN_0_GEN_DE, tm->gen_de) |
			    FIELD_PREP(MAX96724_PATGEN_0_GEN_HS, tm->gen_hs) |
			    FIELD_PREP(MAX96724_PATGEN_0_GEN_VS, tm->gen_vs));
}

static int max96724_set_tpg_clk(struct max96724_priv *priv, u32 clock)
{
	bool patgen_clk_src = 0;
	u8 pclk_src;
	int ret;

	switch (clock) {
	case 25000000:
		pclk_src = MAX96724_DEBUG_EXTRA_PCLK_SRC_25MHZ;
		break;
	case 75000000:
		pclk_src = MAX96724_DEBUG_EXTRA_PCLK_SRC_75MHZ;
		break;
	case 150000000:
		pclk_src = MAX96724_DEBUG_EXTRA_PCLK_SRC_USE_PIPE;
		patgen_clk_src = MAX96724_VPRBS_PATGEN_CLK_SRC_150MHZ;
		break;
	case 375000000:
		pclk_src = MAX96724_DEBUG_EXTRA_PCLK_SRC_USE_PIPE;
		patgen_clk_src = MAX96724_VPRBS_PATGEN_CLK_SRC_375MHZ;
		break;
	case 0:
		return 0;
	default:
		return -EINVAL;
	}

	/*
	 * TPG data is always injected on link 0, which is always routed to
	 * pipe 0.
	 */
	ret = regmap_update_bits(priv->regmap, MAX96724_VPRBS(0),
				 MAX96724_VPRBS_PATGEN_CLK_SRC,
				 FIELD_PREP(MAX96724_VPRBS_PATGEN_CLK_SRC,
					    patgen_clk_src));
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, MAX96724_DEBUG_EXTRA,
				  MAX96724_DEBUG_EXTRA_PCLK_SRC,
				  FIELD_PREP(MAX96724_DEBUG_EXTRA_PCLK_SRC,
					     pclk_src));
}

static int max96724_set_tpg_mode(struct max96724_priv *priv, bool enable)
{
	unsigned int patgen_mode;

	switch (priv->des.tpg_pattern) {
	case MAX_SERDES_TPG_PATTERN_GRADIENT:
		patgen_mode = MAX96724_PATGEN_1_PATGEN_MODE_GRADIENT;
		break;
	case MAX_SERDES_TPG_PATTERN_CHECKERBOARD:
		patgen_mode = MAX96724_PATGEN_1_PATGEN_MODE_CHECKER;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(priv->regmap, MAX96724_PATGEN_1,
				  MAX96724_PATGEN_1_PATGEN_MODE,
				  FIELD_PREP(MAX96724_PATGEN_1_PATGEN_MODE,
					     enable ? patgen_mode
						    : MAX96724_PATGEN_1_PATGEN_MODE_DISABLED));
}

static int max96724_set_tpg(struct max_des *des,
			    const struct max_serdes_tpg_entry *entry)
{
	struct max96724_priv *priv = des_to_priv(des);
	struct max_serdes_tpg_timings timings = { 0 };
	int ret;

	ret = max_serdes_get_tpg_timings(entry, &timings);
	if (ret)
		return ret;

	ret = max96724_set_tpg_timings(priv, &timings);
	if (ret)
		return ret;

	ret = max96724_set_tpg_clk(priv, timings.clock);
	if (ret)
		return ret;

	ret = max96724_set_tpg_mode(priv, entry);
	if (ret)
		return ret;

	return regmap_assign_bits(priv->regmap, MAX96724_MIPI_PHY0,
				  MAX96724_MIPI_PHY0_FORCE_CSI_OUT_EN, !!entry);
}

static const struct max_serdes_tpg_entry max96724_tpg_entries[] = {
	MAX_TPG_ENTRY_640X480P60_RGB888,
	MAX_TPG_ENTRY_1920X1080P30_RGB888,
	MAX_TPG_ENTRY_1920X1080P60_RGB888,
};

static const struct max_des_ops max96724_ops = {
	.num_phys = 4,
	.num_links = 4,
	.num_remaps_per_pipe = 16,
	.phys_configs = {
		.num_configs = ARRAY_SIZE(max96724_phys_configs),
		.configs = max96724_phys_configs,
	},
	.tpg_entries = {
		.num_entries = ARRAY_SIZE(max96724_tpg_entries),
		.entries = max96724_tpg_entries,
	},
	.tpg_mode = MAX_SERDES_GMSL_PIXEL_MODE,
	.tpg_patterns = BIT(MAX_SERDES_TPG_PATTERN_CHECKERBOARD) |
			BIT(MAX_SERDES_TPG_PATTERN_GRADIENT),
	.use_atr = true,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.reg_read = max96724_reg_read,
	.reg_write = max96724_reg_write,
#endif
	.log_pipe_status = max96724_log_pipe_status,
	.log_phy_status = max96724_log_phy_status,
	.set_enable = max96724_set_enable,
	.init = max96724_init,
	.init_phy = max96724_init_phy,
	.set_phy_mode = max96724_set_phy_mode,
	.set_phy_enable = max96724_set_phy_enable,
	.set_pipe_stream_id = max96724_set_pipe_stream_id,
	.set_pipe_link = max96724_set_pipe_link,
	.set_pipe_enable = max96724_set_pipe_enable,
	.set_pipe_remap = max96724_set_pipe_remap,
	.set_pipe_remaps_enable = max96724_set_pipe_remaps_enable,
	.set_pipe_mode = max96724_set_pipe_mode,
	.set_tpg = max96724_set_tpg,
	.select_links = max96724_select_links,
	.set_link_version = max96724_set_link_version,
};

static const struct max96724_chip_info max96724_info = {
	.versions = BIT(MAX_SERDES_GMSL_2_3GBPS) |
		    BIT(MAX_SERDES_GMSL_2_6GBPS),
	.modes = BIT(MAX_SERDES_GMSL_PIXEL_MODE) |
		 BIT(MAX_SERDES_GMSL_TUNNEL_MODE),
	.set_pipe_tunnel_enable = max96724_set_pipe_tunnel_enable,
	.set_pipe_phy = max96724_set_pipe_phy,
	.set_pipe_tunnel_phy = max96724_set_pipe_tunnel_phy,
	.supports_pipe_stream_autoselect = true,
	.num_pipes = 4,
};

static const struct max96724_chip_info max96724f_info = {
	.versions = BIT(MAX_SERDES_GMSL_2_3GBPS),
	.modes = BIT(MAX_SERDES_GMSL_PIXEL_MODE) |
		 BIT(MAX_SERDES_GMSL_TUNNEL_MODE),
	.set_pipe_tunnel_enable = max96724_set_pipe_tunnel_enable,
	.set_pipe_phy = max96724_set_pipe_phy,
	.set_pipe_tunnel_phy = max96724_set_pipe_tunnel_phy,
	.supports_pipe_stream_autoselect = true,
	.num_pipes = 4,
};

static const struct max96724_chip_info max96712_info = {
	.versions = BIT(MAX_SERDES_GMSL_2_3GBPS) |
		    BIT(MAX_SERDES_GMSL_2_6GBPS),
	.modes = BIT(MAX_SERDES_GMSL_PIXEL_MODE),
	.num_pipes = 8,
};

static int max96724_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96724_priv *priv;
	struct max_des_ops *ops;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ops = devm_kzalloc(dev, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;

	priv->info = device_get_match_data(dev);
	if (!priv->info) {
		dev_err(dev, "Failed to get match data\n");
		return -ENODEV;
	}

	priv->dev = dev;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &max96724_i2c_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->gpiod_enable = devm_gpiod_get_optional(&client->dev, "enable",
						     GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpiod_enable))
		return PTR_ERR(priv->gpiod_enable);

	if (priv->gpiod_enable) {
		/* PWDN must be held for 1us for reset */
		udelay(1);

		gpiod_set_value_cansleep(priv->gpiod_enable, 1);

		/* Maximum power-up time (tLOCK) 4ms */
		usleep_range(4000, 5000);
	}

	*ops = max96724_ops;
	ops->versions = priv->info->versions;
	ops->modes = priv->info->modes;
	ops->num_pipes = priv->info->num_pipes;
	ops->set_pipe_tunnel_enable = priv->info->set_pipe_tunnel_enable;
	ops->set_pipe_phy = priv->info->set_pipe_phy;
	ops->set_pipe_tunnel_phy = priv->info->set_pipe_tunnel_phy;
	priv->des.ops = ops;

	ret = max96724_reset(priv);
	if (ret)
		return ret;

	ret = max96724_gpiochip_probe(priv);
	if (ret)
		return ret;

	return max_des_probe(client, &priv->des);
}

static void max96724_remove(struct i2c_client *client)
{
	struct max96724_priv *priv = i2c_get_clientdata(client);

	max_des_remove(&priv->des);

	gpiod_set_value_cansleep(priv->gpiod_enable, 0);
}

static const struct of_device_id max96724_of_table[] = {
	{ .compatible = "maxim,max96712", .data = &max96712_info },
	{ .compatible = "maxim,max96724", .data = &max96724_info },
	{ .compatible = "maxim,max96724f", .data = &max96724f_info },
	{ .compatible = "maxim,max96724r", .data = &max96724f_info },
	{ },
};
MODULE_DEVICE_TABLE(of, max96724_of_table);

static struct i2c_driver max96724_i2c_driver = {
	.driver	= {
		.name = "max96724",
		.of_match_table	= max96724_of_table,
	},
	.probe = max96724_probe,
	.remove = max96724_remove,
};

module_i2c_driver(max96724_i2c_driver);

MODULE_IMPORT_NS("MAX_SERDES");
MODULE_DESCRIPTION("Maxim MAX96724 Quad GMSL2 Deserializer Driver");
MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_LICENSE("GPL");
