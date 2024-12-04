// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * AD9545 Network Clock Generator/Synchronizer
 *
 * Copyright (C) 2020-2023 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/rational.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <dt-bindings/clock/ad9545.h>

#define AD9545_CONFIG_0			0x0000
#define AD9545_PRODUCT_ID_LOW		0x0004
#define AD9545_PRODUCT_ID_HIGH		0x0005
#define AD9545_IO_UPDATE		0x000F
#define AD9545_M0_PIN			0x0102
#define AD9545_CHIP_ID			0x0121
#define AD9545_SYS_CLK_FB_DIV		0x0200
#define AD9545_SYS_CLK_INPUT		0x0201
#define AD9545_SYS_CLK_REF_FREQ		0x0202
#define AD9545_STABILITY_TIMER		0x0207
#define AD9545_COMPENSATE_TDCS		0x0280
#define AD9545_COMPENSATE_NCOS		0x0281
#define AD9545_COMPENSATE_DPLL		0x0282
#define AD9545_AUX_DPLL_CHANGE_LIMIT	0x0283
#define AD9545_AUX_DPLL_SOURCE		0x0284
#define AD9545_AUX_DPLL_LOOP_BW		0x0285
#define AD9545_REF_A_CTRL		0x0300
#define AD9545_REF_A_RDIV		0x0400
#define AD9545_REF_A_PERIOD		0x0404
#define AD9545_REF_A_OFFSET_LIMIT	0x040C
#define AD9545_REF_A_MONITOR_HYST	0x040F
#define AD9545_REF_A_VALID_TIMER	0x0410
#define AD9545_PHASE_LOCK_THRESH	0x0800
#define AD9545_PHASE_LOCK_FILL_RATE	0x0803
#define AD9545_PHASE_LOCK_DRAIN_RATE	0x0804
#define AD9545_FREQ_LOCK_THRESH		0x0805
#define AD9545_FREQ_LOCK_FILL_RATE	0x0808
#define AD9545_FREQ_LOCK_DRAIN_RATE	0x0809
#define AD9545_DPLL0_FTW		0x1000
#define AD9545_DPLL0_SLEW_RATE		0x1011
#define AD9545_MODULATION_COUNTER_A0	0x10C2
#define AD9545_MODULATION_COUNTER_B0	0x10C6
#define AD9545_MODULATION_COUNTER_C0	0x10CA
#define AD9545_MODULATOR_A0		0x10CF
#define AD9545_MODULATOR_B0		0x10D0
#define AD9545_MODULATOR_C0		0x10D1
#define	AD9545_NSHOT_REQ_CH0		0x10D3
#define AD9545_NSHOT_EN_AB0		0x10D4
#define AD9545_NSHOT_EN_C0		0x10D5
#define AD9545_DRIVER_0A_CONF		0x10D7
#define AD9545_SYNC_CTRL0		0x10DB
#define AD9545_APLL0_M_DIV		0x1081
#define AD9545_Q0A_DIV			0x1100
#define AD9545_Q0A_PHASE		0x1104
#define AD9545_Q0A_PHASE_CONF		0x1108
#define AD9545_DPLL0_EN			0x1200
#define AD9545_DPLL0_SOURCE		0x1201
#define AD9545_DPLL0_ZERO_DELAY_FB	0x1202
#define AD9545_DPLL0_FB_MODE		0x1203
#define AD9545_DPLL0_LOOP_BW		0x1204
#define AD9545_DPLL0_HITLESS_N		0x1208
#define AD9545_DPLL0_N_DIV		0x120C
#define AD9545_DPLL0_FRAC		0x1210
#define AD9545_DPLL0_MOD		0x1213
#define AD9545_DPLL0_FAST_L1		0x1216
#define AD9545_DPLL0_FAST_L2		0x1217
#define AD9545_MODULATION_COUNTER_A1	0x14C2
#define AD9545_MODULATION_COUNTER_B1	0x14C6
#define AD9545_MODULATOR_A1		0x14CF
#define AD9545_MODULATOR_B1		0x14D0
#define AD9545_NSHOT_EN_AB1		0x14D4
#define AD9545_DRIVER_1A_CONF		0x14D7
#define AD9545_Q1A_DIV			0x1500
#define AD9545_Q1A_PHASE		0x1504
#define AD9545_Q1A_PHASE_CONF		0x1508
#define AD9545_CALIB_CLK		0x2000
#define AD9545_POWER_DOWN_REF		0x2001
#define AD9545_PWR_CALIB_CH0		0x2100
#define AD9545_CTRL_CH0			0x2101
#define AD9545_DIV_OPS_Q0A		0x2102
#define AD9545_DPLL0_MODE		0x2105
#define AD9545_DPLL0_FAST_MODE		0x2106
#define AD9545_DIV_OPS_Q1A		0x2202
#define AD9545_NCO0_CENTER_FREQ		0x2800
#define AD9545_NCO0_OFFSET_FREQ		0x2807
#define AD9545_NCO0_TAG_RATIO		0x280B
#define AD9545_NCO0_TAG_DELTA		0x280D
#define AD9545_NCO0_TYPE_ADJUST		0x280F
#define AD9545_NCO0_DELTA_RATE_LIMIT	0x2810
#define AD9545_NCO0_DELTA_ADJUST	0x2814
#define AD9545_NCO0_CYCLE_ADJUST	0x2819
#define AD9545_TDC0_DIV			0x2A00
#define AD9545_TDC0_PERIOD		0x2A01
#define AD9545_PLL_STATUS		0x3001
#define AD9545_MISC			0x3002
#define AD9545_TEMP0			0x3003
#define AD9545_REFA_STATUS		0x3005
#define AD9545_PLL0_STATUS		0x3100
#define AD9545_PLL0_OPERATION		0x3101

#define AD9545_SYS_CLK_STABILITY_PERIOD_MASK	GENMASK(19, 0)

#define AD9545_REF_CTRL_DIF_MSK			GENMASK(3, 2)
#define AD9545_REF_CTRL_REFA_MSK		GENMASK(5, 4)
#define AD9545_REF_CTRL_REFAA_MSK		GENMASK(7, 6)

#define AD9545_UPDATE_REGS			0x1
#define AD9545_RESET_REGS			0x81

#define AD9545_MX_PIN(x)			(AD9545_M0_PIN + (x))

#define AD9545_SYNC_CTRLX(x)			(AD9545_SYNC_CTRL0 + ((x) * 0x400))
#define AD9545_REF_X_RDIV(x)			(AD9545_REF_A_RDIV + ((x) * 0x20))
#define AD9545_REF_X_PERIOD(x)			(AD9545_REF_A_PERIOD + ((x) * 0x20))
#define AD9545_REF_X_OFFSET_LIMIT(x)		(AD9545_REF_A_OFFSET_LIMIT + ((x) * 0x20))
#define AD9545_REF_X_MONITOR_HYST(x)		(AD9545_REF_A_MONITOR_HYST + ((x) * 0x20))
#define AD9545_REF_X_VALID_TIMER(x)		(AD9545_REF_A_VALID_TIMER + ((x) * 0x20))
#define AD9545_REF_X_PHASE_LOCK_FILL(x)		(AD9545_PHASE_LOCK_FILL_RATE + ((x) * 0x20))
#define AD9545_REF_X_PHASE_LOCK_DRAIN(x)	(AD9545_PHASE_LOCK_DRAIN_RATE + ((x) * 0x20))
#define AD9545_REF_X_FREQ_LOCK_FILL(x)		(AD9545_FREQ_LOCK_FILL_RATE + ((x) * 0x20))
#define AD9545_REF_X_FREQ_LOCK_DRAIN(x)		(AD9545_FREQ_LOCK_DRAIN_RATE + ((x) * 0x20))

#define AD9545_SOURCEX_PHASE_THRESH(x)		(AD9545_PHASE_LOCK_THRESH + ((x) * 0x20))
#define AD9545_SOURCEX_FREQ_THRESH(x)		(AD9545_FREQ_LOCK_THRESH + ((x) * 0x20))
#define AD9545_NCOX_PHASE_THRESH(x)		(AD9545_SOURCEX_PHASE_THRESH((x) + 4))
#define AD9545_NCOX_FREQ_THRESH(x)		(AD9545_SOURCEX_FREQ_THRESH((x) + 4))

#define AD9545_APLLX_M_DIV(x)			(AD9545_APLL0_M_DIV + ((x) * 0x400))

#define AD9545_Q0_DIV(x)			(AD9545_Q0A_DIV + ((x) * 0x9))
#define AD9545_Q1_DIV(x)			(AD9545_Q1A_DIV + ((x) * 0x9))
#define AD9545_QX_DIV(x) ({					\
	typeof(x) x_ = (x);					\
								\
	(x_ > 5) ? AD9545_Q1_DIV(x_ - 6) : AD9545_Q0_DIV(x_);	\
})

#define AD9545_Q0_PHASE(x)			(AD9545_Q0A_PHASE + ((x) * 0x9))
#define AD9545_Q1_PHASE(x)			(AD9545_Q1A_PHASE + ((x) * 0x9))
#define AD9545_QX_PHASE(x) ({						\
	typeof(x) x_ = (x);						\
									\
	(x_ > 5) ? AD9545_Q1_PHASE(x_ - 6) : AD9545_Q0_PHASE(x_);	\
})

#define AD9545_Q0_PHASE_CONF(x)			(AD9545_Q0A_PHASE_CONF + ((x) * 0x9))
#define AD9545_Q1_PHASE_CONF(x)			(AD9545_Q1A_PHASE_CONF + ((x) * 0x9))
#define AD9545_QX_PHASE_CONF(x) ({						\
	typeof(x) x_ = (x);							\
										\
	(x_ > 5) ? AD9545_Q1_PHASE_CONF(x_ - 6) : AD9545_Q0_PHASE_CONF(x_);	\
})

#define AD9545_NSHOT_REQ_CH(x)			(AD9545_NSHOT_REQ_CH0 + ((x) * 0x400))
#define AD9545_DPLLX_FTW(x)			(AD9545_DPLL0_FTW + ((x) * 0x400))
#define AD9545_DPLLX_SLEW_RATE(x)		(AD9545_DPLL0_SLEW_RATE + ((x) * 0x400))
#define AD9545_DPLLX_EN(x, y)			(AD9545_DPLL0_EN + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_SOURCE(x, y)		(AD9545_DPLL0_SOURCE + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FB_PATH(x, y)		(AD9545_DPLL0_ZERO_DELAY_FB + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FB_MODE(x, y)		(AD9545_DPLL0_FB_MODE + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_LOOP_BW(x, y)		(AD9545_DPLL0_LOOP_BW + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_HITLESS_N(x, y)		(AD9545_DPLL0_HITLESS_N + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_N_DIV(x, y)		(AD9545_DPLL0_N_DIV + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FRAC_DIV(x, y)		(AD9545_DPLL0_FRAC + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_MOD_DIV(x, y)		(AD9545_DPLL0_MOD + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FAST_L1(x, y)		(AD9545_DPLL0_FAST_L1 + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FAST_L2(x, y)		(AD9545_DPLL0_FAST_L2 + ((x) * 0x400) + ((y) * 0x20))

#define AD9545_DIV_OPS_Q0(x)			(AD9545_DIV_OPS_Q0A + (x))
#define AD9545_DIV_OPS_Q1(x)			(AD9545_DIV_OPS_Q1A + (x))
#define AD9545_DIV_OPS_QX(x) ({						\
	typeof(x) x_ = (x) / 2;						\
									\
	(x_ > 2) ? AD9545_DIV_OPS_Q1(x_ - 3) : AD9545_DIV_OPS_Q0(x_);	\
})

#define AD9545_PWR_CALIB_CHX(x)			(AD9545_PWR_CALIB_CH0 + ((x) * 0x100))
#define AD9545_PLLX_STATUS(x)			(AD9545_PLL0_STATUS + ((x) * 0x100))
#define AD9545_PLLX_OPERATION(x)		(AD9545_PLL0_OPERATION + ((x) * 0x100))
#define AD9545_CTRL_CH(x)			(AD9545_CTRL_CH0 + ((x) * 0x100))
#define AD9545_DPLLX_FAST_MODE(x)		(AD9545_DPLL0_FAST_MODE + ((x) * 0x100))
#define AD9545_REFX_STATUS(x)			(AD9545_REFA_STATUS + (x))

#define AD9545_PROFILE_SEL_MODE_MSK		GENMASK(3, 2)
#define AD9545_PROFILE_SEL_MODE(x)		FIELD_PREP(AD9545_PROFILE_SEL_MODE_MSK, x)

#define AD9545_NCOX_CENTER_FREQ(x)		(AD9545_NCO0_CENTER_FREQ + ((x) * 0x40))
#define AD9545_NCOX_OFFSET_FREQ(x)		(AD9545_NCO0_OFFSET_FREQ + ((x) * 0x40))
#define AD9545_NCOX_TAG_RATIO(x)		(AD9545_NCO0_TAG_RATIO + ((x) * 0x40))
#define AD9545_NCOX_TAG_DELTA(x)		(AD9545_NCO0_TAG_DELTA + ((x) * 0x40))
#define AD9545_NCOX_TYPE_ADJUST(x)		(AD9545_NCO0_TYPE_ADJUST + ((x) * 0x40))
#define AD9545_NCOX_DELTA_RATE_LIMIT(x)		(AD9545_NCO0_DELTA_RATE_LIMIT + ((x) * 0x40))
#define AD9545_NCOX_DELTA_ADJUST(x)		(AD9545_NCO0_DELTA_ADJUST + ((x) * 0x40))
#define AD9545_NCOX_CYCLE_ADJUST(x)		(AD9545_NCO0_CYCLE_ADJUST + ((x) * 0x40))

/*
 * AD9545 AUX NCO center frequency register has 16-bit integer part and
 * 40-bit fractional part.
 */
#define AD9545_NCO_CENTER_FREQ_INT_WIDTH	16
#define AD9545_NCO_CENTER_FREQ_FRAC_WIDTH	40
#define AD9545_NCO_CENTER_FREQ_WIDTH		(AD9545_NCO_CENTER_FREQ_INT_WIDTH + \
						 AD9545_NCO_CENTER_FREQ_FRAC_WIDTH)

#define AD9545_NCO_CENTER_FREQ_MSK		GENMASK_ULL(AD9545_NCO_CENTER_FREQ_WIDTH - 1, 0)
#define AD9545_NCO_CENTER_FREQ_INT_MSK		GENMASK_ULL(AD9545_NCO_CENTER_FREQ_WIDTH - 1, \
							    AD9545_NCO_CENTER_FREQ_FRAC_WIDTH)
#define AD9545_NCO_CENTER_FREQ_FRAC_MSK		GENMASK_ULL(AD9545_NCO_CENTER_FREQ_FRAC_WIDTH - 1, 0)

#define AD9545_NCO_CENTER_FREQ_MAX		FIELD_MAX(AD9545_NCO_CENTER_FREQ_MSK)
#define AD9545_NCO_CENTER_FREQ_INT_MAX		FIELD_MAX(AD9545_NCO_CENTER_FREQ_INT_MSK)
#define AD9545_NCO_CENTER_FREQ_FRAC_MAX		FIELD_MAX(AD9545_NCO_CENTER_FREQ_FRAC_MSK)

/*
 * AD9545 AUX NCO offset frequency register has 8-bit integer part and
 * 24-bit fractional part.
 */
#define AD9545_NCO_OFFSET_FREQ_INT_WIDTH	8
#define AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH	24
#define AD9545_NCO_OFFSET_FREQ_WIDTH		(AD9545_NCO_OFFSET_FREQ_INT_WIDTH + \
						 AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH)

#define AD9545_NCO_OFFSET_FREQ_MSK		GENMASK_ULL(AD9545_NCO_OFFSET_FREQ_WIDTH - 1, 0)
#define AD9545_NCO_OFFSET_FREQ_INT_MSK		GENMASK_ULL(AD9545_NCO_OFFSET_FREQ_WIDTH - 1, \
							    AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH)
#define AD9545_NCO_OFFSET_FREQ_FRAC_MSK		GENMASK_ULL(AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH - 1, 0)

#define AD9545_NCO_OFFSET_FREQ_MAX		FIELD_MAX(AD9545_NCO_OFFSET_FREQ_MSK)
#define AD9545_NCO_OFFSET_FREQ_INT_MAX		FIELD_MAX(AD9545_NCO_OFFSET_FREQ_INT_MSK)
#define AD9545_NCO_OFFSET_FREQ_FRAC_MAX		FIELD_MAX(AD9545_NCO_OFFSET_FREQ_FRAC_MSK)

#define AD9545_NCO_FREQ_INT_MAX			(AD9545_NCO_CENTER_FREQ_INT_MAX + \
						 AD9545_NCO_OFFSET_FREQ_INT_MAX)

#define AD9545_TDCX_DIV(x)			(AD9545_TDC0_DIV + ((x) * 0x9))
#define AD9545_TDCX_PERIOD(x)			(AD9545_TDC0_PERIOD + ((x) * 0x9))

/* AD9545 MX PIN bitfields */
#define AD9545_MX_TO_TDCX(x)			(0x30 + (x))

/* AD9545 COMPENSATE TDCS bitfields */
#define AD9545_COMPENSATE_TDCS_VIA_AUX_DPLL	0x4

/* AD9545 COMPENSATE NCOS bitfields */
#define AD9545_COMPENSATE_NCOS_VIA_AUX_DPLL	0x44

/* AD9545 COMPENSATE DPLL bitfields */
#define AD9545_COMPNESATE_VIA_AUX_DPLL		0x44

/* define AD9545_DPLLX_EN bitfields */
#define AD9545_EN_PROFILE_MSK			BIT(0)
#define AD9545_SEL_PRIORITY_MSK			GENMASK(5, 1)

/* define AD9545_DPLLX_FB_MODE bitfields */
#define AD9545_EN_HITLESS_MSK			BIT(0)
#define AD9545_TAG_MODE_MSK			GENMASK(4, 2)
#define AD9545_BASE_FILTER_MSK			BIT(7)

/* AD9545_PWR_CALIB_CHX bitfields */
#define AD9545_PWR_DOWN_CH			BIT(0)
#define AD9545_CALIB_APLL			BIT(1)

/* AD9545_SYNC_CTRLX bitfields */
#define AD9545_SYNC_CTRL_DPLL_REF_MSK		BIT(2)
#define AD9545_SYNC_CTRL_MODE_MSK		GENMASK(1, 0)

/* AD9545_QX_PHASE_CONF bitfields */
#define AD9545_QX_HALF_DIV_MSK			BIT(5)
#define AD9545_QX_PHASE_32_MSK			BIT(6)

/* AD9545_DIV_OPS_QX bitfields */
#define AD9545_DIV_OPS_MUTE_A_MSK		BIT(2)
#define AD9545_DIV_OPS_MUTE_AA_MSK		BIT(3)

/* AD9545 Modulator bitfields */
#define AD9545_MODULATOR_EN			BIT(0)

/* AD9545_NSHOT_REQ_CH bitfields */
#define AD9545_NSHOT_NR_MSK			GENMASK(5, 0)

/* AD9545_CTRL_CH bitfields */
#define AD9545_CTRL_CH_NSHOT_MSK		BIT(0)

/* AD9545_PLL_STATUS bitfields */
#define AD9545_PLLX_LOCK(x, y)			((1 << (4 + (x))) & (y))

/* AD9545_MISC bitfields */
#define AD9545_MISC_AUX_NC0_ERR_MSK		GENMASK(5, 4)
#define AD9545_MISC_AUX_NC1_ERR_MSK		GENMASK(7, 6)
#define AD9545_AUX_DPLL_LOCK_MSK		BIT(1)
#define AD9545_AUX_DPLL_REF_FAULT		BIT(2)

/* AD9545_REFX_STATUS bitfields */
#define AD9545_REFX_SLOW_MSK			BIT(0)
#define AD9545_REFX_FAST_MSK			BIT(1)
#define AD9545_REFX_JITTER_MSK			BIT(2)
#define AD9545_REFX_FAULT_MSK			BIT(3)
#define AD9545_REFX_VALID_MSK			BIT(4)
#define AD9545_REFX_LOS_MSK			BIT(5)

/* AD9545_PLL0_STATUS bitfields */
#define AD9545_PLL_LOCKED			BIT(0)

/* AD9545_PLL0_OPERATION bitfields */
#define AD9545_PLL_FREERUN			BIT(0)
#define AD9545_PLL_HOLDOVER			BIT(1)
#define AD9545_PLL_ACTIVE			BIT(3)
#define AD9545_PLL_ACTIVE_PROFILE		GENMASK(6, 4)

#define AD9545_SYS_PLL_STABLE_MSK		GENMASK(1, 0)
#define AD9545_SYS_PLL_STABLE(x)		(((x) & AD9545_SYS_PLL_STABLE_MSK) == 0x3)

#define AD9545_APLL_LOCKED(x)			((x) & BIT(3))

/*  AD9545 tagging modes */
#define AD9545_NO_TAGGING			0
#define AD9545_FB_PATH_TAG			2

#define AD9545_SYS_CLK_STABILITY_MS	50

#define AD9545_R_DIV_MSK		GENMASK(29, 0)
#define AD9545_R_DIV_MAX		0x40000000
#define AD9545_IN_MAX_TDC_FREQ_HZ	200000

#define AD9545_MAX_REFS			4

#define AD9545_APLL_M_DIV_MIN		1
#define AD9545_APLL_M_DIV_MAX		255

#define AD9545_DPLL_MAX_N		1073741823
#define AD9545_DPLL_MAX_FRAC		16777215
#define AD9545_DPLL_MAX_MOD		16777215
#define AD9545_MAX_DPLL_PROFILES	6

#define AD9545_MAX_NSHOT_PULSES		63

#define AD9545_MAX_ZERO_DELAY_RATE	200000000

static const unsigned int ad9545_apll_rate_ranges_hz[2][2] = {
	{2424000000U, 3232000000U}, {3232000000U, 4040000000U}
};

static const unsigned int ad9545_apll_pfd_rate_ranges_hz[2] = {
	162000000U, 350000000U
};

static const unsigned short ad9545_vco_calibration_op[][2] = {
	{AD9545_CALIB_CLK, 0},
	{AD9545_IO_UPDATE, AD9545_UPDATE_REGS},
	{AD9545_CALIB_CLK, BIT(2)},
	{AD9545_IO_UPDATE, AD9545_UPDATE_REGS},
};

static const u8 ad9545_tdc_source_mapping[] = {
	0, 1, 2, 3, 8, 9,
};

static const u32 ad9545_fast_acq_excess_bw_map[] = {
	0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024,
};

static const u32 ad9545_fast_acq_timeout_map[] = {
	1, 10, 50, 100, 500, 1000, 10000, 50000,
};

static const u32 ad9545_hyst_scales_bp[] = {
	0, 3125, 6250, 12500, 25000, 50000, 75000, 87500
};

static const u32 ad9545_out_source_ua[] = {
	7500, 12500, 15000
};

static const u32 ad9545_rate_change_limit_map[] = {
	715, 1430, 2860, 5720, 11440, 22880, 45760,
};

static const char * const ad9545_ref_m_clk_names[] = {
	"Ref-M0", "Ref-M1", "Ref-M2",
};

static const char * const ad9545_ref_clk_names[] = {
	"Ref-A", "Ref-AA", "Ref-B", "Ref-BB",
};

static const char * const ad9545_in_clk_names[] = {
	"Ref-A-Div", "Ref-AA-Div", "Ref-B-Div", "Ref-BB-Div",
};

static const char * const ad9545_out_clk_names[] = {
	"Q0A-div", "Q0AA-div", "Q0B-div", "Q0BB-div", "Q0C-div", "Q0CC-div", "Q1A-div", "Q1AA-div",
	"Q1B-div", "Q1BB-div",
};

static const char * const ad9545_pll_clk_names[] = {
	"PLL0", "PLL1",
};

static const char * const ad9545_aux_nco_clk_names[] = {
	"AUX_NCO0", "AUX_NCO1",
};

static const char * const ad9545_aux_tdc_clk_names[] = {
	"AUX_TDC0", "AUX_TDC1",
};

static const char *ad9545_aux_dpll_name = "AUX_DPLL";

enum ad9545_ref_mode {
	AD9545_SINGLE_ENDED = 0,
	AD9545_DIFFERENTIAL,
};

enum ad9545_single_ended_config {
	AD9545_AC_COUPLED_IF = 0,
	AD9545_DC_COUPLED_1V2,
	AD9545_DC_COUPLED_1V8,
	AD9545_IN_PULL_UP,
};

enum ad9545_diferential_config {
	AD9545_AC_COUPLED = 0,
	AD9545_DC_COUPLED,
	AD9545_DC_COUPLED_LVDS,
};

enum ad9545_output_mode {
	AD9545_SINGLE_DIV_DIF = 0,
	AD9545_SINGLE_DIV,
	AD9545_DUAL_DIV,
};

struct ad9545_outputs_regs {
	u16	modulator_reg;
	u16	modulation_counter_reg;
	u16	nshot_en_reg;
	u8	nshot_en_msk;
};

static const struct ad9545_outputs_regs ad9545_out_regs[] = {
	{
		.modulator_reg = AD9545_MODULATOR_A0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_A0,
		.nshot_en_reg = AD9545_NSHOT_EN_AB0,
		.nshot_en_msk = BIT(0),
	},
	{
		.modulator_reg = AD9545_MODULATOR_A0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_A0,
		.nshot_en_reg = AD9545_NSHOT_EN_AB0,
		.nshot_en_msk = BIT(2),
	},
	{
		.modulator_reg = AD9545_MODULATOR_B0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_B0,
		.nshot_en_reg = AD9545_NSHOT_EN_AB0,
		.nshot_en_msk = BIT(4),
	},
	{
		.modulator_reg = AD9545_MODULATOR_B0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_B0,
		.nshot_en_reg = AD9545_NSHOT_EN_AB0,
		.nshot_en_msk = BIT(6),
	},
	{
		.modulator_reg = AD9545_MODULATOR_C0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_C0,
		.nshot_en_reg = AD9545_NSHOT_EN_C0,
		.nshot_en_msk = BIT(0),
	},
	{
		.modulator_reg = AD9545_MODULATOR_C0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_C0,
		.nshot_en_reg = AD9545_NSHOT_EN_C0,
		.nshot_en_msk = BIT(2),
	},
	{
		.modulator_reg = AD9545_MODULATOR_A1,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_A1,
		.nshot_en_reg = AD9545_NSHOT_EN_AB1,
		.nshot_en_msk = BIT(0),
	},
	{
		.modulator_reg = AD9545_MODULATOR_A1,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_A1,
		.nshot_en_reg = AD9545_NSHOT_EN_AB1,
		.nshot_en_msk = BIT(2),
	},
	{
		.modulator_reg = AD9545_MODULATOR_B1,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_B1,
		.nshot_en_reg = AD9545_NSHOT_EN_AB1,
		.nshot_en_msk = BIT(4),
	},
	{
		.modulator_reg = AD9545_MODULATOR_B1,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_B1,
		.nshot_en_reg = AD9545_NSHOT_EN_AB1,
		.nshot_en_msk = BIT(6),
	},
};

struct ad9545_out_clk {
	struct ad9545_state		*st;
	bool				output_used;
	bool				source_current;
	enum ad9545_output_mode		output_mode;
	u32				source_ua;
	struct clk_hw			hw;
	unsigned int			address;
	unsigned long			rate_requested_hz;
};

struct ad9545_dpll_profile {
	unsigned int			address;
	unsigned int			parent_index;
	unsigned int			priority;
	unsigned int			loop_bw_uhz;
	unsigned int			fast_acq_excess_bw;
	unsigned int			fast_acq_timeout_ms;
	unsigned int			fast_acq_settle_ms;
	bool				en;
	u8				tdc_source;
	bool				fb_tagging;
};

struct ad9545_pll_clk {
	struct ad9545_state		*st;
	bool				pll_used;
	unsigned int			address;
	struct clk_hw			hw;
	unsigned int			num_parents;
	const struct clk_hw		**parents;
	struct ad9545_dpll_profile	profiles[AD9545_MAX_DPLL_PROFILES];
	unsigned int			free_run_freq;
	unsigned int			fast_acq_trigger_mode;
	unsigned long			rate_requested_hz;
	bool				internal_zero_delay;
	u8				internal_zero_delay_source;
	unsigned long			internal_zero_delay_source_rate_hz;
	u32				slew_rate_limit_ps;
};

struct ad9545_ref_in_clk {
	struct clk_hw			hw;
	struct ad9545_state		*st;
	u32				r_div_ratio;
	bool				ref_used;
	u32				d_tol_ppb;
	u8				monitor_hyst_scale;
	u32				valid_t_ms;
	struct clk			*parent_clk;
	unsigned int			address;
	enum ad9545_ref_mode		mode;
	unsigned int			freq_thresh_ps;
	unsigned int			phase_thresh_ps;
	unsigned int			phase_lock_fill_rate;
	unsigned int			phase_lock_drain_rate;
	unsigned int			freq_lock_fill_rate;
	unsigned int			freq_lock_drain_rate;
	union {
		enum ad9545_single_ended_config		s_conf;
		enum ad9545_diferential_config		d_conf;
	};
};

struct ad9545_aux_nco_clk {
	struct clk_hw			hw;
	bool				nco_used;
	struct ad9545_state		*st;
	unsigned int			address;
	unsigned int			freq_thresh_ps;
	unsigned int			phase_thresh_ps;
};

struct ad9545_aux_tdc_clk {
	struct clk_hw			hw;
	bool				tdc_used;
	struct ad9545_state		*st;
	unsigned int			address;
	unsigned int			pin_nr;
};

struct ad9545_aux_dpll_clk {
	struct clk_hw			hw;
	bool				dpll_used;
	struct ad9545_state		*st;
	unsigned int			source;
	unsigned int			loop_bw_mhz;
	unsigned int			rate_change_limit;
};

struct ad9545_sys_clk {
	bool				sys_clk_freq_doubler;
	bool				sys_clk_crystal;
	u32				ref_freq_hz;
	u32				sys_freq_hz;
};

struct ad9545_state {
	struct device			*dev;
	struct regmap			*regmap;
	struct ad9545_sys_clk		sys_clk;
	struct ad9545_aux_dpll_clk	aux_dpll_clk;
	struct ad9545_pll_clk		pll_clks[ARRAY_SIZE(ad9545_pll_clk_names)];
	struct ad9545_ref_in_clk	ref_in_clks[ARRAY_SIZE(ad9545_ref_clk_names)];
	struct ad9545_out_clk		out_clks[ARRAY_SIZE(ad9545_out_clk_names)];
	struct ad9545_aux_nco_clk	aux_nco_clks[ARRAY_SIZE(ad9545_aux_nco_clk_names)];
	struct ad9545_aux_tdc_clk	aux_tdc_clks[ARRAY_SIZE(ad9545_aux_tdc_clk_names)];
	struct clk_hw			**clks[4];
};

#define to_ref_in_clk(_hw)	container_of(_hw, struct ad9545_ref_in_clk, hw)
#define to_aux_dpll_clk(_hw)	container_of(_hw, struct ad9545_aux_dpll_clk, hw)
#define to_pll_clk(_hw)		container_of(_hw, struct ad9545_pll_clk, hw)
#define to_out_clk(_hw)		container_of(_hw, struct ad9545_out_clk, hw)
#define to_nco_clk(_hw)		container_of(_hw, struct ad9545_aux_nco_clk, hw)
#define to_tdc_clk(_hw)		container_of(_hw, struct ad9545_aux_tdc_clk, hw)

static int ad9545_parse_dt_inputs(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	struct clk *clk;
	bool prop_found;
	int ref_ind;
	u32 val;
	int ret = 0;
	int i;

	fwnode = dev_fwnode(st->dev);

	prop_found = false;
	fwnode_for_each_available_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,r-divider-ratio"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &ref_ind);
		if (ret < 0) {
			dev_err(st->dev, "reg not specified in ref node.");
			goto out_fail;
		}

		if (ref_ind > 3) {
			ret = -EINVAL;
			goto out_fail;
		}

		st->ref_in_clks[ref_ind].ref_used = true;
		st->ref_in_clks[ref_ind].address = ref_ind;
		st->ref_in_clks[ref_ind].st = st;

		ret = fwnode_property_read_u32(child, "adi,phase-lock-fill-rate", &val);
		if (!ret)
			st->ref_in_clks[ref_ind].phase_lock_fill_rate = val;

		ret = fwnode_property_read_u32(child, "adi,phase-lock-drain-rate", &val);
		if (!ret)
			st->ref_in_clks[ref_ind].phase_lock_drain_rate = val;

		ret = fwnode_property_read_u32(child, "adi,freq-lock-fill-rate", &val);
		if (!ret)
			st->ref_in_clks[ref_ind].freq_lock_fill_rate = val;

		ret = fwnode_property_read_u32(child, "adi,freq-lock-drain-rate", &val);
		if (!ret)
			st->ref_in_clks[ref_ind].freq_lock_drain_rate = val;

		prop_found = fwnode_property_present(child, "adi,single-ended-mode");
		if (prop_found) {
			st->ref_in_clks[ref_ind].mode = AD9545_SINGLE_ENDED;
			ret = fwnode_property_read_u32(child, "adi,single-ended-mode", &val);
			if (ret < 0)
				goto out_fail;

			st->ref_in_clks[ref_ind].s_conf = val;
		} else {
			st->ref_in_clks[ref_ind].mode = AD9545_DIFFERENTIAL;
			ret = fwnode_property_read_u32(child, "adi,differential-mode", &val);
			if (ret < 0)
				goto out_fail;

			st->ref_in_clks[ref_ind].d_conf = val;
		}

		ret = fwnode_property_read_u32(child, "adi,r-divider-ratio", &val);
		if (ret < 0) {
			dev_err(st->dev, "No r-divider-ratio specified for ref: %d", ref_ind);
			goto out_fail;
		}

		st->ref_in_clks[ref_ind].r_div_ratio = val;

		ret = fwnode_property_read_u32(child, "adi,ref-dtol-pbb", &val);
		if (ret < 0) {
			dev_err(st->dev, "No ref-dtol-pbb specified for ref: %d", ref_ind);
			goto out_fail;
		}

		st->ref_in_clks[ref_ind].d_tol_ppb = val;

		ret = fwnode_property_read_u32(child, "adi,ref-monitor-hysteresis-pbb", &val);
		if (ret < 0) {
			dev_err(st->dev, "No ref-monitor-hysteresis-pbb specified for ref: %d",
				ref_ind);
			goto out_fail;
		}

		for (i = 0; i < ARRAY_SIZE(ad9545_hyst_scales_bp); i++) {
			if (ad9545_hyst_scales_bp[i] == val) {
				st->ref_in_clks[ref_ind].monitor_hyst_scale = i;
				break;
			}
		}

		if (i == ARRAY_SIZE(ad9545_hyst_scales_bp)) {
			ret = -EINVAL;
			goto out_fail;
		}

		ret = fwnode_property_read_u32(child, "adi,ref-validation-timer-ms", &val);
		if (ret < 0) {
			dev_err(st->dev, "No ref-validation-timer-ms specified for ref: %d",
				ref_ind);
			goto out_fail;
		}

		st->ref_in_clks[ref_ind].valid_t_ms = val;

		ret = fwnode_property_read_u32(child, "adi,freq-lock-threshold-ps", &val);
		if (ret < 0) {
			dev_err(st->dev, "No freq-lock-threshold-ps specified for ref: %d",
				ref_ind);
			goto out_fail;
		}

		st->ref_in_clks[ref_ind].freq_thresh_ps = val;

		ret = fwnode_property_read_u32(child, "adi,phase-lock-threshold-ps", &val);
		if (ret < 0) {
			dev_err(st->dev, "No phase-lock-threshold-ps specified for ref: %d",
				ref_ind);
			goto out_fail;
		}

		st->ref_in_clks[ref_ind].phase_thresh_ps = val;

		clk = devm_clk_get(st->dev, ad9545_ref_clk_names[ref_ind]);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			goto out_fail;
		}

		st->ref_in_clks[ref_ind].parent_clk = clk;
	}

out_fail:
	fwnode_handle_put(child);
	return ret;
}

static int ad9545_parse_dt_pll_profiles(struct ad9545_state *st, const struct fwnode_handle *child,
					u32 addr)
{
	struct fwnode_handle *profile_node;
	int ret = 0;

	/* parse DPLL profiles */
	fwnode_for_each_available_child_node(child, profile_node) {
		u32 profile_addr;
		u32 val;

		ret = fwnode_property_read_u32(profile_node, "reg", &profile_addr);
		if (ret < 0) {
			dev_err(st->dev, "Could not read Profile reg property.");
			goto out_fail;
		}

		if (profile_addr >= AD9545_MAX_DPLL_PROFILES) {
			ret = -EINVAL;
			goto out_fail;
		}

		st->pll_clks[addr].profiles[profile_addr].en = true;
		st->pll_clks[addr].profiles[profile_addr].address = profile_addr;

		ret = fwnode_property_read_u32(profile_node, "adi,pll-loop-bandwidth-uhz", &val);
		if (ret < 0) {
			dev_err(st->dev, "Could not read Profile %d, pll-loop-bandwidth.",
				profile_addr);
			goto out_fail;
		}

		st->pll_clks[addr].profiles[profile_addr].loop_bw_uhz = val;

		ret = fwnode_property_read_u32(profile_node, "adi,fast-acq-trigger-mode", &val);
		if (!ret)
			st->pll_clks[addr].fast_acq_trigger_mode = val;

		ret = fwnode_property_read_u32(profile_node, "adi,fast-acq-excess-bw", &val);
		if (!ret)
			st->pll_clks[addr].profiles[profile_addr].fast_acq_excess_bw = val;

		ret = fwnode_property_read_u32(profile_node, "adi,fast-acq-timeout-ms", &val);
		if (!ret)
			st->pll_clks[addr].profiles[profile_addr].fast_acq_timeout_ms = val;

		ret = fwnode_property_read_u32(profile_node, "adi,fast-acq-lock-settle-ms",
					       &val);
		if (!ret)
			st->pll_clks[addr].profiles[profile_addr].fast_acq_settle_ms = val;

		ret = fwnode_property_read_u32(profile_node, "adi,profile-priority", &val);
		if (!ret)
			st->pll_clks[addr].profiles[profile_addr].priority = val;

		ret = fwnode_property_read_u32(profile_node, "adi,pll-source", &val);
		if (ret < 0) {
			dev_err(st->dev, "Could not read Profile %d, pll-source.",
				profile_addr);
			goto out_fail;
		}

		if (val > 5) {
			ret = -EINVAL;
			goto out_fail;
		}

		st->pll_clks[addr].profiles[profile_addr].tdc_source = val;
	}

out_fail:
	fwnode_handle_put(profile_node);
	return ret;
}

static int ad9545_parse_dt_plls(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	bool prop_found;
	u32 val;
	u32 addr;
	int ret = 0;

	fwnode = dev_fwnode(st->dev);

	prop_found = false;
	fwnode_for_each_available_child_node(fwnode, child) {
		if (strncmp(fwnode_get_name(child), "pll-clk", strlen("pll-clk")))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &addr);
		if (ret < 0)
			goto out_fail;

		if (addr > 1) {
			ret = -EINVAL;
			goto out_fail;
		}

		st->pll_clks[addr].pll_used = true;
		st->pll_clks[addr].address = addr;

		ret = fwnode_property_read_u32(child, "adi,pll-slew-rate-limit-ps", &val);
		if (!ret)
			st->pll_clks[addr].slew_rate_limit_ps = val;

		prop_found = fwnode_property_present(child, "adi,pll-internal-zero-delay-feedback");
		st->pll_clks[addr].internal_zero_delay = prop_found;

		ret = fwnode_property_read_u32(child, "adi,pll-internal-zero-delay-feedback", &val);
		if (prop_found && !ret) {
			if (val >= ARRAY_SIZE(ad9545_out_clk_names)) {
				dev_err(st->dev, "Invalid zero-delay fb path: %u.", val);
				ret = -EINVAL;
				goto out_fail;
			}

			st->pll_clks[addr].internal_zero_delay_source = val;
		}

		ret = fwnode_property_read_u32(child, "adi,pll-internal-zero-delay-feedback-hz",
					       &val);
		if (prop_found && !ret) {
			if (val >= AD9545_MAX_ZERO_DELAY_RATE) {
				dev_err(st->dev, "Invalid zero-delay output rate: %u.", val);
				ret = -EINVAL;
				goto out_fail;
			}

			st->pll_clks[addr].internal_zero_delay_source_rate_hz = val;
		}

		ret = ad9545_parse_dt_pll_profiles(st, child, addr);
		if (ret)
			goto out_fail;
	}

out_fail:
	fwnode_handle_put(child);
	return ret;
}

static int ad9545_parse_dt_outputs(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	bool prop_found;
	int out_ind;
	u32 val;
	int ret = 0;

	fwnode = dev_fwnode(st->dev);

	prop_found = false;
	fwnode_for_each_available_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,output-mode"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &out_ind);
		if (ret < 0) {
			dev_err(st->dev, "No reg specified for output.");
			goto out_fail;
		}

		if (out_ind > 9) {
			ret = -EINVAL;
			goto out_fail;
		}

		st->out_clks[out_ind].output_used = true;
		st->out_clks[out_ind].address = out_ind;

		if (fwnode_property_present(child, "adi,current-source"))
			st->out_clks[out_ind].source_current = true;

		ret = fwnode_property_read_u32(child, "adi,current-source-microamp", &val);
		if (ret < 0) {
			dev_err(st->dev, "No current-source-microamp specified for output: %d",
				out_ind);
			goto out_fail;
		}

		st->out_clks[out_ind].source_ua = val;

		ret = fwnode_property_read_u32(child, "adi,output-mode", &val);
		if (ret < 0) {
			dev_err(st->dev, "No output-mode specified for output: %d", out_ind);
			goto out_fail;
		}

		st->out_clks[out_ind].output_mode = val;
	}

out_fail:
	fwnode_handle_put(child);
	return ret;
}

static int ad9545_parse_dt_ncos(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	bool prop_found;
	u32 val;
	u32 addr;
	int ret = 0;

	fwnode = dev_fwnode(st->dev);

	prop_found = false;
	fwnode_for_each_available_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,freq-lock-threshold-ps") ||
		    fwnode_property_present(child, "adi,ref-dtol-pbb"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &addr);
		if (ret < 0) {
			dev_err(st->dev, "No reg specified for aux. NCO.");
			goto out_fail;
		}

		if (addr > 1) {
			ret = -EINVAL;
			goto out_fail;
		}

		st->aux_nco_clks[addr].nco_used = true;
		st->aux_nco_clks[addr].address = addr;
		st->aux_nco_clks[addr].st = st;

		ret = fwnode_property_read_u32(child, "adi,freq-lock-threshold-ps", &val);
		if (ret < 0) {
			dev_err(st->dev, "No freq-lock-threshold-ps specified for aux. NCO: %d",
				addr);
			goto out_fail;
		}

		st->aux_nco_clks[addr].freq_thresh_ps = val;

		ret = fwnode_property_read_u32(child, "adi,phase-lock-threshold-ps", &val);
		if (ret < 0) {
			dev_err(st->dev, "No phase-lock-threshold-ps specified for aux. NCO: %d",
				addr);
			goto out_fail;
		}

		st->aux_nco_clks[addr].phase_thresh_ps = val;
	}

out_fail:
	fwnode_handle_put(child);
	return ret;
}

static int ad9545_parse_dt_tdcs(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	u32 val;
	u32 addr;
	int ret = 0;

	fwnode = dev_fwnode(st->dev);
	fwnode_for_each_available_child_node(fwnode, child) {
		if (!fwnode_property_present(child, "adi,pin-nr"))
			continue;

		ret = fwnode_property_read_u32(child, "reg", &addr);
		if (ret < 0) {
			dev_err(st->dev, "No reg specified for aux. TDC.");
			goto out_fail;
		}

		if (addr > 1) {
			dev_err(st->dev, "Invalid address: %u for aux. TDC.", addr);
			ret = -EINVAL;
			goto out_fail;
		}

		st->aux_tdc_clks[addr].tdc_used = true;
		st->aux_tdc_clks[addr].address = addr;
		st->aux_tdc_clks[addr].st = st;

		ret = fwnode_property_read_u32(child, "adi,pin-nr", &val);
		if (ret < 0) {
			dev_err(st->dev, "No source pin specified for aux. TDC: %d", addr);
			goto out_fail;
		}

		if (val >= ARRAY_SIZE(ad9545_ref_m_clk_names)) {
			dev_err(st->dev, "Invalid Mx pin-nr: %d", val);
			ret = -EINVAL;
			goto out_fail;
		}

		st->aux_tdc_clks[addr].pin_nr = val;
	}

out_fail:
	fwnode_handle_put(child);
	return ret;
}

static int ad9545_parse_dt_aux_dpll(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	u32 val;
	int ret;

	fwnode = dev_fwnode(st->dev);
	child = fwnode_get_named_child_node(fwnode, "aux-dpll");
	if (!child)
		return 0;

	st->aux_dpll_clk.dpll_used = true;
	st->aux_dpll_clk.st = st;

	ret = fwnode_property_read_u32(child, "adi,rate-change-limit", &val);
	if (!ret)
		st->aux_dpll_clk.rate_change_limit = val;

	ret = fwnode_property_read_u32(child, "adi,compensation-source", &val);
	if (ret < 0) {
		dev_err(st->dev, "No TDC source specified for aux. DPLL.");
		goto out_fail;
	}

	st->aux_dpll_clk.source = val;

	ret = fwnode_property_read_u32(child, "adi,aux-dpll-bw-mhz", &val);
	if (ret < 0) {
		dev_err(st->dev, "No loop bw specified for aux. DPLL.");
		goto out_fail;
	}

	st->aux_dpll_clk.loop_bw_mhz = val;

out_fail:
	fwnode_handle_put(child);
	return ret;
}

static int ad9545_parse_dt(struct ad9545_state *st)
{
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = dev_fwnode(st->dev);

	ret = fwnode_property_read_u32(fwnode, "adi,ref-frequency-hz", &st->sys_clk.ref_freq_hz);
	if (ret < 0) {
		dev_err(st->dev, "No ref-frequency-hz specified.");
		return ret;
	}

	st->sys_clk.sys_clk_crystal = fwnode_property_present(fwnode, "adi,ref-crystal");
	st->sys_clk.sys_clk_freq_doubler = fwnode_property_present(fwnode, "adi,freq-doubler");

	ret = ad9545_parse_dt_inputs(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt_plls(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt_outputs(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt_ncos(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt_tdcs(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt_aux_dpll(st);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9545_check_id(struct ad9545_state *st)
{
	u32 chip_id;
	u32 val;
	int ret;

	ret = regmap_read(st->regmap, AD9545_PRODUCT_ID_LOW, &val);
	if (ret < 0)
		return ret;

	chip_id = val;
	ret = regmap_read(st->regmap, AD9545_PRODUCT_ID_HIGH, &val);
	if (ret < 0)
		return ret;

	chip_id += val << 8;
	if (chip_id != AD9545_CHIP_ID) {
		dev_err(st->dev, "Unrecognized CHIP_ID 0x%X\n", chip_id);
		return -ENODEV;
	}

	return 0;
}

static int ad9545_io_update(struct ad9545_state *st)
{
	return regmap_write(st->regmap, AD9545_IO_UPDATE, AD9545_UPDATE_REGS);
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>

static int ad9545_aux_dpll_clk_debugfs_show(struct seq_file *s, void *p)
{
	struct ad9545_aux_dpll_clk *ref_clk = s->private;
	u32 regval;
	int ret;

	ret = ad9545_io_update(ref_clk->st);
	if (ret < 0)
		return ret;

	ret = regmap_read(ref_clk->st->regmap, AD9545_MISC, &regval);
	if (ret < 0)
		return ret;

	seq_printf(s, "%s:\n", ad9545_aux_dpll_name);
	seq_printf(s, "Lock status: %s\n",
		   (regval & AD9545_AUX_DPLL_LOCK_MSK) ? "Locked" : "Unlocked");
	seq_printf(s, "Reference: %s\n",
		   (regval & AD9545_AUX_DPLL_REF_FAULT) ? "Faulted" : "Valid");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ad9545_aux_dpll_clk_debugfs);

static void ad9545_aux_dpll_clk_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	struct ad9545_aux_dpll_clk *dpll_clk = to_aux_dpll_clk(hw);

	if (dpll_clk->dpll_used)
		debugfs_create_file(ad9545_aux_dpll_name, 0444,
				    dentry, dpll_clk, &ad9545_aux_dpll_clk_debugfs_fops);
}

static int ad9545_in_clk_debugfs_show(struct seq_file *s, void *p)
{
	struct ad9545_ref_in_clk *ref_clk = s->private;
	u32 regval;
	int ret;

	ret = ad9545_io_update(ref_clk->st);
	if (ret < 0)
		return ret;

	ret = regmap_read(ref_clk->st->regmap, AD9545_REFX_STATUS(ref_clk->address), &regval);
	if (ret < 0)
		return ret;

	seq_printf(s, "%s:\n", ad9545_in_clk_names[ref_clk->address]);
	seq_puts(s, "Reference: ");

	if (regval & AD9545_REFX_VALID_MSK)
		seq_puts(s, "Valid\n");
	else
		seq_printf(s, "Faulted:%s%s%s%s\n",
			   (regval & AD9545_REFX_LOS_MSK) ? " LOS" : "",
			   (regval & AD9545_REFX_JITTER_MSK) ? " Excess Jitter" : "",
			   (regval & AD9545_REFX_FAST_MSK) ? " Too Fast" : "",
			   (regval & AD9545_REFX_SLOW_MSK) ? " Too Slow" : "");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ad9545_in_clk_debugfs);

static void ad9545_in_clk_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	struct ad9545_ref_in_clk *ref_clk = to_ref_in_clk(hw);

	if (ref_clk->ref_used)
		debugfs_create_file(ad9545_in_clk_names[ref_clk->address], 0444,
				    dentry, ref_clk, &ad9545_in_clk_debugfs_fops);
}

static int ad9545_pll_clk_debugfs_show(struct seq_file *s, void *p)
{
	struct ad9545_pll_clk *pll = s->private;
	__le16 reg;
	u32 regval;
	s16 temp;
	int ret;

	ret = ad9545_io_update(pll->st);
	if (ret < 0)
		return ret;

	seq_printf(s, "PLL%d:\n", pll->address);

	ret = regmap_read(pll->st->regmap, AD9545_PLLX_STATUS(pll->address), &regval);
	if (ret < 0)
		return ret;

	seq_printf(s, "PLL status: %s\n", (regval & AD9545_PLL_LOCKED) ? "Locked" : "Unlocked");

	ret = regmap_read(pll->st->regmap, AD9545_PLLX_OPERATION(pll->address), &regval);
	if (ret < 0)
		return ret;

	seq_printf(s, "Freerun Mode: %s\n", (regval & AD9545_PLL_FREERUN) ? "On" : "Off");
	seq_printf(s, "Holdover Mode: %s\n", (regval & AD9545_PLL_HOLDOVER) ? "On" : "Off");
	seq_printf(s, "PLL Profile: %s\n", (regval & AD9545_PLL_ACTIVE) ? "On" : "Off");
	seq_printf(s, "Profile Number: %ld\n", FIELD_GET(AD9545_PLL_ACTIVE_PROFILE, regval));

	ret = regmap_bulk_read(pll->st->regmap, AD9545_TEMP0, &reg, 2);
	if (ret < 0)
		return ret;

	temp = le16_to_cpu(reg);
	temp /= (1 << 7);
	seq_printf(s, "Temperature: %d C\n", temp);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ad9545_pll_clk_debugfs);

static void ad9545_pll_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	struct ad9545_pll_clk *pll = to_pll_clk(hw);

	if (pll->pll_used)
		debugfs_create_file(ad9545_pll_clk_names[pll->address], 0444, dentry,
				    pll, &ad9545_pll_clk_debugfs_fops);
}
#else
#define ad9545_aux_dpll_clk_debug_init NULL
#define ad9545_in_clk_debug_init NULL
#define ad9545_pll_debug_init NULL
#endif

static int ad9545_sys_clk_setup(struct ad9545_state *st)
{
	u64 ref_freq_milihz;
	u32 stability_timer;
	__le64 regval64;
	__le32 regval;
	u8 div_ratio;
	u32 fosc;
	int ret;
	u8 val;
	u32 fs;
	int i;

	/*
	 * System frequency must be between 2250 MHz and 2415 MHz.
	 * fs = fosc * K / j
	 * K - feedback divider ratio [4, 255]
	 * j = 1/2 if frequency doubler is enabled
	 */
	fosc = DIV_ROUND_UP(st->sys_clk.ref_freq_hz, 1000000);

	if (st->sys_clk.sys_clk_freq_doubler)
		fosc *= 2;

	div_ratio = 0;
	for (i = 4; i < 256; i++) {
		fs = i * fosc;

		if (fs > 2250 && fs < 2415) {
			div_ratio = i;
			break;
		}
	}

	if (!div_ratio) {
		dev_err(st->dev, "No feedback divider ratio for sys clk PLL found.\n");
		return -EINVAL;
	}

	st->sys_clk.sys_freq_hz = st->sys_clk.ref_freq_hz * div_ratio;
	if (st->sys_clk.sys_clk_freq_doubler)
		st->sys_clk.sys_freq_hz *= 2;

	ret = regmap_write(st->regmap, AD9545_SYS_CLK_FB_DIV, div_ratio);
	if (ret < 0)
		return ret;

	/* enable crystal maintaining amplifier */
	val = 0;
	if (st->sys_clk.sys_clk_crystal)
		val |= BIT(3);

	if (st->sys_clk.sys_clk_freq_doubler)
		val |= BIT(0);

	ret = regmap_write(st->regmap, AD9545_SYS_CLK_INPUT, val);
	if (ret < 0)
		return ret;

	/* write reference frequency provided at XOA, XOB in milliherz */
	ref_freq_milihz = mul_u32_u32(st->sys_clk.ref_freq_hz, 1000);
	regval64 = cpu_to_le64(ref_freq_milihz);

	ret = regmap_bulk_write(st->regmap, AD9545_SYS_CLK_REF_FREQ, &regval64, 5);
	if (ret < 0)
		return ret;

	stability_timer = FIELD_PREP(AD9545_SYS_CLK_STABILITY_PERIOD_MASK,
				     AD9545_SYS_CLK_STABILITY_MS);
	regval = cpu_to_le32(stability_timer);
	return regmap_bulk_write(st->regmap, AD9545_STABILITY_TIMER,
				 &regval, 3);
}

static int ad9545_get_q_div(struct ad9545_state *st, int addr, u32 *q_div)
{
	__le32 regval;
	int ret;

	ret = regmap_bulk_read(st->regmap, AD9545_QX_DIV(addr), &regval, 4);
	if (ret < 0)
		return ret;

	*q_div = le32_to_cpu(regval);

	return 0;
}

static int ad9545_set_q_div(struct ad9545_state *st, int addr, u32 q_div)
{
	__le32 regval;
	int ret;

	regval = cpu_to_le32(q_div);
	ret = regmap_bulk_write(st->regmap, AD9545_QX_DIV(addr), &regval, 4);
	if (ret < 0)
		return ret;

	return ad9545_io_update(st);
}

static unsigned long ad9545_out_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);
	u32 qdiv;
	int ret;

	ret = ad9545_get_q_div(clk->st, clk->address, &qdiv);
	if (ret < 0) {
		dev_err(clk->st->dev, "Could not read Q div value.");
		return 0;
	}

	return div_u64(parent_rate, qdiv);
}

static long ad9545_out_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	unsigned long out_rate;
	u32 qdiv;

	qdiv = div64_u64(*parent_rate, rate);
	if (!qdiv)
		out_rate = *parent_rate;
	else
		out_rate = div_u64(*parent_rate, qdiv);

	return out_rate;
}

static int ad9545_out_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);
	u32 qdiv = div64_u64(parent_rate, rate);

	if (!qdiv)
		qdiv = 1;

	return ad9545_set_q_div(clk->st, clk->address, qdiv);
}

static int ad9545_out_clk_set_phase(struct clk_hw *hw, int degrees)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);
	u64 phase_code;
	u32 phase_conf;
	__le64 regval;
	u32 half_div;
	u32 qdiv;
	int ret;

	ret = ad9545_get_q_div(clk->st, clk->address, &qdiv);
	if (ret < 0)
		return ret;

	ret = regmap_read(clk->st->regmap, AD9545_QX_PHASE_CONF(clk->address), &phase_conf);
	if (ret < 0)
		return ret;

	half_div = !!(phase_conf & AD9545_QX_HALF_DIV_MSK);

	/* Qxy phase bitfield depends on the current Q div value */
	phase_code = qdiv;
	phase_code = div_u64((phase_code * 2 + half_div) * degrees, 360);

	/* Qxy phase bitfield is 33 bits long, with last bit in PHASE_CONF reg */
	regval = cpu_to_le64(phase_code & 0xFFFFFFFF);
	ret = regmap_bulk_write(clk->st->regmap, AD9545_QX_PHASE(clk->address), &regval, 4);
	if (ret < 0)
		return ret;

	if (phase_code > U32_MAX) {
		ret = regmap_update_bits(clk->st->regmap, AD9545_QX_PHASE_CONF(clk->address),
					 AD9545_QX_PHASE_32_MSK, AD9545_QX_PHASE_32_MSK);
		if (ret < 0)
			return ret;
	}

	return ad9545_io_update(clk->st);
}

static int ad9545_out_clk_get_phase(struct clk_hw *hw)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);
	u64 input_edges_nr;
	u64 phase_code;
	__le32 regval;
	u32 phase_conf;
	u32 qdiv;
	int ret;

	ret = ad9545_get_q_div(clk->st, clk->address, &qdiv);
	if (ret < 0)
		return ret;

	ret = regmap_read(clk->st->regmap, AD9545_QX_PHASE_CONF(clk->address), &phase_conf);
	if (ret < 0)
		return ret;

	ret = regmap_bulk_read(clk->st->regmap, AD9545_QX_PHASE(clk->address), &regval, 4);
	if (ret < 0)
		return ret;

	/* Qxy phase bitfield is 33 bits long, with last bit in PHASE_CONF reg */
	phase_code = !!(phase_conf & AD9545_QX_PHASE_32_MSK);
	phase_code = (phase_code >> 32) + cpu_to_le32(regval);

	input_edges_nr = 2 * qdiv + !!(phase_conf & AD9545_QX_HALF_DIV_MSK);

	/*
	 * phase = 360 * (Qxy Phase / E) where:
	 * E is the total number of input edges per output period of the Q-divider.
	 */
	return div64_u64(phase_code * 360, input_edges_nr);
}

static int ad9545_output_muting(struct ad9545_out_clk *clk, bool mute)
{
	u8 regval = 0;
	int ret;
	u8 mask;

	if (clk->address % 2)
		mask = AD9545_DIV_OPS_MUTE_AA_MSK;
	else
		mask = AD9545_DIV_OPS_MUTE_A_MSK;

	if (mute)
		regval = mask;

	ret = regmap_update_bits(clk->st->regmap, AD9545_DIV_OPS_QX(clk->address), mask, regval);
	if (ret < 0)
		return ret;

	return ad9545_io_update(clk->st);
}

static int ad9545_trigger_n_shot(struct ad9545_out_clk *clk)
{
	u8 channel = 0;
	int ret;

	if (clk->address > AD9545_Q0CC)
		channel = 1;

	ret = regmap_update_bits(clk->st->regmap, AD9545_CTRL_CH(channel), AD9545_CTRL_CH_NSHOT_MSK,
				 AD9545_CTRL_CH_NSHOT_MSK);
	if (ret < 0)
		return ret;

	ret = ad9545_io_update(clk->st);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(clk->st->regmap, AD9545_CTRL_CH(channel), AD9545_CTRL_CH_NSHOT_MSK,
				 0);
	if (ret < 0)
		return ret;

	return ad9545_io_update(clk->st);
}

static int ad9545_out_clk_enable(struct clk_hw *hw)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);

	if (clk_get_nshot(hw->clk))
		return ad9545_trigger_n_shot(clk);
	else
		return ad9545_output_muting(clk, false);
}

static void ad9545_out_clk_disable(struct clk_hw *hw)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);

	/*
	 * When the n-shot feature is enabled,
	 * the clocks are automatically muted at
	 * the end of the burst.
	 */
	if (!clk_get_nshot(hw->clk))
		ad9545_output_muting(clk, true);
}

static int ad9545_out_clk_is_enabled(struct clk_hw *hw)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);
	u32 regval;
	int ret;
	u8 mask;

	if (clk->address % 2)
		mask = AD9545_DIV_OPS_MUTE_AA_MSK;
	else
		mask = AD9545_DIV_OPS_MUTE_A_MSK;

	ret = regmap_read(clk->st->regmap, AD9545_DIV_OPS_QX(clk->address), &regval);
	if (ret < 0)
		return ret;

	return !!(mask & regval);
}

static int ad9545_setup_n_shot_mode(struct ad9545_out_clk *clk, int nshot)
{
	u8 channel = 0;
	u32 reg;
	int ret;
	u32 mask;

	if (nshot > AD9545_MAX_NSHOT_PULSES || nshot < 0)
		return -EINVAL;

	reg = ad9545_out_regs[clk->address].nshot_en_reg;
	mask = ad9545_out_regs[clk->address].nshot_en_msk;

	ret = regmap_update_bits(clk->st->regmap, reg, mask, nshot ? mask : 0);
	if (ret < 0)
		return ret;

	if (nshot) {
		if (clk->address > AD9545_Q0CC)
			channel = 1;

		ret = regmap_update_bits(clk->st->regmap, AD9545_NSHOT_REQ_CH(channel),
					 AD9545_NSHOT_NR_MSK,
					 FIELD_PREP(AD9545_NSHOT_NR_MSK, nshot));
		if (ret < 0)
			return ret;
	}

	return ad9545_io_update(clk->st);
}

static int ad9545_out_clk_set_nshot(struct clk_hw *hw, int nshot)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);

	return ad9545_setup_n_shot_mode(clk, nshot);
}

static int ad9545_out_clk_get_nshot(struct clk_hw *hw)
{
	struct ad9545_out_clk *clk = to_out_clk(hw);
	int channel = 0;
	int ret;
	u32 val;

	ret = regmap_read(clk->st->regmap, ad9545_out_regs[clk->address].nshot_en_reg, &val);
	if (ret < 0)
		return ret;

	if (!(ad9545_out_regs[clk->address].nshot_en_msk & val))
		return 0;

	if (clk->address > AD9545_Q0CC)
		channel = 1;

	ret = regmap_read(clk->st->regmap, AD9545_NSHOT_REQ_CH(channel), &val);
	if (ret < 0)
		return ret;

	return FIELD_GET(AD9545_NSHOT_NR_MSK, val);
}

static const struct clk_ops ad9545_out_clk_ops = {
	.enable = ad9545_out_clk_enable,
	.disable = ad9545_out_clk_disable,
	.is_enabled = ad9545_out_clk_is_enabled,
	.recalc_rate = ad9545_out_clk_recalc_rate,
	.round_rate = ad9545_out_clk_round_rate,
	.set_rate = ad9545_out_clk_set_rate,
	.set_phase = ad9545_out_clk_set_phase,
	.get_phase = ad9545_out_clk_get_phase,
	.set_nshot = ad9545_out_clk_set_nshot,
	.get_nshot = ad9545_out_clk_get_nshot,
};

static int ad9545_outputs_setup(struct ad9545_state *st)
{
	struct clk_init_data init[ARRAY_SIZE(ad9545_out_clk_names)] = {0};
	int out_i;
	u16 addr;
	int ret;
	u8 reg;
	int i;
	int j;

	/* configure current sources */
	for (i = 0; i < ARRAY_SIZE(ad9545_out_clk_names) / 2; i++) {
		st->out_clks[i * 2].st = st;
		st->out_clks[i * 2 + 1].st = st;

		if (st->out_clks[i * 2].output_used)
			out_i = i * 2;
		else if (st->out_clks[i * 2 + 1].output_used)
			out_i = i * 2 + 1;
		else
			continue;

		reg = 0;
		if (st->out_clks[out_i].source_current)
			reg = 1;

		for (j = 0; j < ARRAY_SIZE(ad9545_out_source_ua); j++)
			if (ad9545_out_source_ua[j] == st->out_clks[out_i].source_ua)
				reg |= FIELD_PREP(GENMASK(2, 1), j);

		reg |= FIELD_PREP(GENMASK(4, 3), st->out_clks[out_i].output_mode);

		if (i < 3)
			addr = AD9545_DRIVER_0A_CONF + i;
		else
			addr = AD9545_DRIVER_1A_CONF + (i - 3);

		ret = regmap_write(st->regmap, addr, reg);
		if (ret < 0)
			return ret;
	}

	st->clks[AD9545_CLK_OUT] = devm_kzalloc(st->dev, ARRAY_SIZE(ad9545_out_clk_names) *
						sizeof(struct clk *), GFP_KERNEL);
	if (!st->clks[AD9545_CLK_OUT])
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(ad9545_out_clk_names); i++) {
		if (!st->out_clks[i].output_used)
			continue;

		init[i].name = ad9545_out_clk_names[i];
		init[i].ops = &ad9545_out_clk_ops;

		if (i > 5)
			init[i].parent_names = &ad9545_pll_clk_names[1];
		else
			init[i].parent_names = &ad9545_pll_clk_names[0];

		init[i].num_parents = 1;

		st->out_clks[i].hw.init = &init[i];
		ret = devm_clk_hw_register(st->dev, &st->out_clks[i].hw);
		if (ret < 0)
			return ret;

		st->clks[AD9545_CLK_OUT][i] = &st->out_clks[i].hw;
	}

	for (i = 0; i < ARRAY_SIZE(st->pll_clks); i++) {
		reg = 0;

		/* For hitless mode, wait for DPLL reference in order to sync outputs */
		if (st->pll_clks[i].internal_zero_delay) {
			reg = FIELD_PREP(AD9545_SYNC_CTRL_DPLL_REF_MSK, 1);

			ret = regmap_write(st->regmap, AD9545_SYNC_CTRLX(i), reg);
			if (ret < 0)
				return ret;

			ret = ad9545_io_update(st);
			if (ret < 0)
				return ret;
		}

		reg |= FIELD_PREP(AD9545_SYNC_CTRL_MODE_MSK, 1);
		ret = regmap_write(st->regmap, AD9545_SYNC_CTRLX(i), reg);
		if (ret < 0)
			return ret;

		ret = ad9545_io_update(st);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9545_set_r_div(struct ad9545_state *st, u32 div, int addr)
{
	__le32 regval;
	u32 val;
	int ret;

	if (div > AD9545_R_DIV_MAX || div == 0) {
		dev_err(st->dev, "Invalid R-divider value (addr: %d): %u",
			addr, div);
		return -EINVAL;
	}

	/* r-div ratios are mapped from 0 onward */
	div -= 1;

	val = FIELD_PREP(AD9545_R_DIV_MSK, div);
	regval = cpu_to_le32(val);
	ret = regmap_bulk_write(st->regmap, AD9545_REF_X_RDIV(addr), &regval, 4);
	if (ret < 0)
		return ret;

	return ad9545_io_update(st);
}

static int ad9545_get_r_div(struct ad9545_state *st, int addr, u32 *r_div)
{
	__le32 regval;
	u32 val, div;
	int ret;

	ret = regmap_bulk_read(st->regmap, AD9545_REF_X_RDIV(addr), &regval, 4);
	if (ret < 0)
		return ret;
	val = le32_to_cpu(regval);
	div = FIELD_GET(AD9545_R_DIV_MSK, val);

	/* r-div ratios are mapped from 0 onward */
	*r_div = div + 1;

	return 0;
}

static unsigned long ad9545_in_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ad9545_ref_in_clk *clk = to_ref_in_clk(hw);
	u32 div;
	int ret;

	ret = ad9545_get_r_div(clk->st, clk->address, &div);
	if (ret < 0) {
		dev_err(clk->st->dev, "Could not read r div value.");
		return 1;
	}

	return DIV_ROUND_CLOSEST(parent_rate, div);
}

static const struct clk_ops ad9545_in_clk_ops = {
	.recalc_rate = ad9545_in_clk_recalc_rate,
	.debug_init = ad9545_in_clk_debug_init,
};

static int ad9545_input_refs_setup(struct ad9545_state *st)
{
	struct clk_init_data init[4] = {0};
	__le32 regval;
	__le64 regval64;
	u64 period_es;
	int ret;
	u32 val;
	u8 reg;
	int i;

	/* configure input references */
	for (i = 0; i < ARRAY_SIZE(st->ref_in_clks); i += 2) {
		if (st->ref_in_clks[i].mode == AD9545_DIFFERENTIAL) {
			reg = BIT(0);
			reg |= FIELD_PREP(AD9545_REF_CTRL_DIF_MSK, st->ref_in_clks[i].d_conf);
		} else {
			reg = 0;
			reg |= FIELD_PREP(AD9545_REF_CTRL_REFA_MSK, st->ref_in_clks[i].s_conf);
			reg |= FIELD_PREP(AD9545_REF_CTRL_REFAA_MSK, st->ref_in_clks[i + 1].s_conf);
		}

		ret = regmap_write(st->regmap, AD9545_REF_A_CTRL + i * 2, reg);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(st->ref_in_clks); i++) {
		if (!st->ref_in_clks[i].ref_used)
			continue;

		/* configure refs r dividers */
		ret = ad9545_set_r_div(st, st->ref_in_clks[i].r_div_ratio, i);
		if (ret < 0)
			return ret;

		/* write nominal period in attoseconds */
		period_es = 1000000000000000000ULL;
		val = clk_get_rate(st->ref_in_clks[i].parent_clk);
		if (!val)
			return -EINVAL;

		period_es = div_u64(period_es, val);

		regval = cpu_to_le32(st->ref_in_clks[i].d_tol_ppb);
		ret = regmap_bulk_write(st->regmap, AD9545_REF_X_OFFSET_LIMIT(i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval64 = cpu_to_le64(period_es);
		ret = regmap_bulk_write(st->regmap, AD9545_REF_X_PERIOD(i), &regval64, 8);
		if (ret < 0)
			return ret;

		ret = regmap_write(st->regmap, AD9545_REF_X_MONITOR_HYST(i),
				   st->ref_in_clks[i].monitor_hyst_scale);
		if (ret < 0)
			return ret;

		regval = cpu_to_le32(st->ref_in_clks[i].freq_thresh_ps);
		ret = regmap_bulk_write(st->regmap, AD9545_SOURCEX_FREQ_THRESH(i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval = cpu_to_le32(st->ref_in_clks[i].valid_t_ms);
		ret = regmap_bulk_write(st->regmap, AD9545_REF_X_VALID_TIMER(i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval = cpu_to_le32(st->ref_in_clks[i].phase_thresh_ps);
		ret = regmap_bulk_write(st->regmap, AD9545_SOURCEX_PHASE_THRESH(i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval = st->ref_in_clks[i].freq_lock_fill_rate;
		if (regval) {
			ret = regmap_write(st->regmap, AD9545_REF_X_FREQ_LOCK_FILL(i), regval);
			if (ret < 0)
				return ret;
		}

		regval = st->ref_in_clks[i].freq_lock_drain_rate;
		if (regval) {
			ret = regmap_write(st->regmap, AD9545_REF_X_FREQ_LOCK_DRAIN(i), regval);
			if (ret < 0)
				return ret;
		}

		regval = st->ref_in_clks[i].phase_lock_fill_rate;
		if (regval) {
			ret = regmap_write(st->regmap, AD9545_REF_X_PHASE_LOCK_FILL(i), regval);
			if (ret < 0)
				return ret;
		}

		regval = st->ref_in_clks[i].phase_lock_drain_rate;
		if (regval) {
			ret = regmap_write(st->regmap, AD9545_REF_X_PHASE_LOCK_DRAIN(i), regval);
			if (ret < 0)
				return ret;
		}

		init[i].name = ad9545_in_clk_names[i];
		init[i].ops = &ad9545_in_clk_ops;
		init[i].parent_names = &ad9545_ref_clk_names[i];
		init[i].num_parents = 1;

		st->ref_in_clks[i].hw.init = &init[i];
		ret = devm_clk_hw_register(st->dev, &st->ref_in_clks[i].hw);
		if (ret < 0)
			return ret;
	}

	/* disable unused references */
	reg = 0;
	for (i = 0; i < ARRAY_SIZE(st->ref_in_clks); i++) {
		if (!st->ref_in_clks[i].ref_used)
			reg |= (1 << i);
	}

	return regmap_write(st->regmap, AD9545_POWER_DOWN_REF, reg);
}

static int ad9545_calc_ftw(struct ad9545_pll_clk *clk, u32 freq, u64 *tuning_word)
{
	u64 ftw = 1;
	u32 ftw_frac;
	u32 ftw_int;

	/*
	 * In case of unlock event the DPLL will go in open-loop mode and output
	 * the freq given by the freerun tuning word.
	 * DPLLx Freerun TW = (2 ^ 48) × (f NCO /f System )
	 */
	ftw = mul_u64_u32_div(ftw << 48, freq, clk->st->sys_clk.sys_freq_hz);

	/*
	 * Check if FTW is valid:
	 * (2 ^ 48) / FTW = INT.FRAC where:
	 * 7 ≤ INT ≤ 13 and 0.05 ≤ FRAC ≤ 0.95
	 */
	ftw_int = div64_u64(1ULL << 48, ftw);
	if (ftw_int < 7 || ftw_int > 13)
		return -EINVAL;

	div_u64_rem(div64_u64(100 * (1ULL << 48), ftw), 100, &ftw_frac);
	if (ftw_frac < 5 || ftw_frac > 95)
		return -EINVAL;

	*tuning_word = ftw;

	return 0;
}

static int ad9545_set_freerun_freq(struct ad9545_pll_clk *clk, u32 freq)
{
	__le64 regval;
	u64 ftw;
	int ret;

	if (!freq)
		return 0;

	ret = ad9545_calc_ftw(clk, freq, &ftw);
	if (ret < 0)
		return ret;

	regval = cpu_to_le64(ftw);
	ret = regmap_bulk_write(clk->st->regmap, AD9545_DPLLX_FTW(clk->address), &regval, 6);
	if (ret < 0)
		return ret;

	clk->free_run_freq = freq;
	return ad9545_io_update(clk->st);
}

static u32 ad9545_calc_m_div(unsigned long rate)
{
	u32 m_div;

	/*
	 * PFD of APLL has input frequency limits in 162 - 350 Mghz range.
	 * Use APLL to upconvert this freq to Ghz range.
	 */
	m_div = div_u64(rate, ad9545_apll_pfd_rate_ranges_hz[0] / 2 +
			ad9545_apll_pfd_rate_ranges_hz[1] / 2);
	m_div = clamp_t(u8, m_div, AD9545_APLL_M_DIV_MIN, AD9545_APLL_M_DIV_MAX);

	return m_div;
}

/*
 * Zero delay mode forbids use of the Fractional
 * part of DPLL in calculation, turning
 * the Digital PLL part in an Integer-N PLL.
 * The configuration turns into two cascaded Integer N PLLs.
 */
static u64 ad9545_calc_pll_zero_delay_params(struct ad9545_pll_clk *clk, unsigned long rate,
					     unsigned long parent_rate, u32 *m, u32 *n,
					     int profile_nr)
{
	unsigned long out_rate_hz;
	u32 m_div, n_div, q_div;
	int output_nr;

	output_nr = clk->internal_zero_delay_source;
	out_rate_hz = clk->internal_zero_delay_source_rate_hz;

	if (!out_rate_hz || !parent_rate)
		return rate;

	/*
	 * The frequency translation factor for internal zero delay mode:
	 * f_out_x = N * f_ref_div_x
	 */
	n_div = DIV_ROUND_CLOSEST(out_rate_hz, parent_rate);
	if (!n_div)
		return rate;

	/* set q-div beforehand */
	q_div = DIV_ROUND_UP_ULL(rate, out_rate_hz);
	ad9545_set_q_div(clk->st, output_nr, q_div);

	/* half divider at output requires APLL to generate twice the frequency demanded */
	rate *= 2;

	/* Find a suitable M divider */
	for (m_div = AD9545_APLL_M_DIV_MIN; m_div < AD9545_APLL_M_DIV_MAX; m_div++) {
		unsigned long dpll_rate;

		if (rate % m_div != 0)
			continue;

		dpll_rate = DIV_ROUND_UP_ULL(rate, m_div);
		if (dpll_rate >= ad9545_apll_pfd_rate_ranges_hz[0] &&
		    dpll_rate <= ad9545_apll_pfd_rate_ranges_hz[1])
			break;
	}

	*m = m_div;
	*n = n_div;

	return q_div * out_rate_hz;
}

static u64 ad9545_calc_pll_params(struct ad9545_pll_clk *clk, unsigned long rate,
				  unsigned long parent_rate, bool zero_delay, u32 *m, u32 *n,
				  unsigned long *frac, unsigned long *mod, int profile_nr)
{
	u32 min_dpll_n_div;
	u64 output_rate;
	u32 dpll_n_div;
	u32 m_div;
	u64 den;
	u64 num;

	if (zero_delay) {
		*frac = 0;
		*mod = 1;

		return ad9545_calc_pll_zero_delay_params(clk, rate, parent_rate, m, n, profile_nr);
	}

	/* half divider at output requires APLL to generate twice the frequency demanded */
	rate *= 2;

	m_div = ad9545_calc_m_div(rate);

	/*
	 * If N + FRAC / MOD = rate / (m_div * parent_rate)
	 * and N = [rate / (m_div * past_rate)]:
	 * We get: FRAC/MOD = (rate / (m_div * parent_rate)) - N
	 */
	dpll_n_div = div64_u64(rate, parent_rate * m_div);

	/*
	 * APLL has to be able to satisfy output freq bounds
	 * thus output of DPLL has a lower bound
	 */
	min_dpll_n_div = div_u64(ad9545_apll_rate_ranges_hz[clk->address][0],
				 AD9545_APLL_M_DIV_MAX * parent_rate);
	dpll_n_div = clamp_t(u32, dpll_n_div, min_dpll_n_div, AD9545_DPLL_MAX_N);

	num = rate - (dpll_n_div * m_div * parent_rate);
	den = m_div * parent_rate;

	rational_best_approximation(num, den, AD9545_DPLL_MAX_FRAC, AD9545_DPLL_MAX_MOD, frac, mod);
	*m = m_div;
	*n = dpll_n_div;

	output_rate = mul_u64_u32_div(*frac * parent_rate, m_div, *mod);
	output_rate += parent_rate * dpll_n_div * m_div;

	return (u32)DIV_ROUND_CLOSEST(output_rate, 2);
}

static int ad9545_tdc_source_valid(struct ad9545_pll_clk *clk, unsigned int tdc_source)
{
	unsigned int regval;
	int ret;

	if (tdc_source >= AD9545_MAX_REFS) {
		ret = regmap_read(clk->st->regmap, AD9545_MISC, &regval);
		if (ret < 0)
			return ret;

		if (tdc_source == AD9545_MAX_REFS)
			return !(regval & AD9545_MISC_AUX_NC0_ERR_MSK);
		else
			return !(regval & AD9545_MISC_AUX_NC1_ERR_MSK);
	} else {
		ret = regmap_read(clk->st->regmap, AD9545_REFX_STATUS(tdc_source), &regval);
		if (ret < 0)
			return ret;

		return !!(regval & AD9545_REFX_VALID_MSK);
	}
}

static u8 ad9545_pll_get_parent(struct clk_hw *hw)
{
	struct ad9545_pll_clk *clk = to_pll_clk(hw);
	struct ad9545_dpll_profile *profile;
	u8 best_prio = 0xFF;
	u8 best_parent;
	int ret;
	int i;

	ret = ad9545_io_update(clk->st);
	if (ret < 0)
		return ret;

	/*
	 * A DPLL will pick a parent clock depending
	 * on the priorities and if it is a valid timestamp source.
	 */
	for (i = 0; i < AD9545_MAX_DPLL_PROFILES; i++) {
		profile = &clk->profiles[i];
		if (!profile->en)
			continue;

		ret = ad9545_tdc_source_valid(clk, profile->tdc_source);
		if (ret < 0)
			return clk->num_parents;

		if (ret > 0 && profile->priority < best_prio) {
			best_prio = profile->priority;
			best_parent = profile->parent_index;
		}
	}

	if (best_prio != 0xFF)
		return best_parent;

	return clk->num_parents;
}

static unsigned long ad9545_pll_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ad9545_pll_clk *clk = to_pll_clk(hw);
	unsigned long output_rate;
	__le32 regval;
	u32 frac;
	u32 mod;
	int ret;
	u32 m;
	u32 n;
	int i;

	m = 0;
	ret = regmap_read(clk->st->regmap, AD9545_APLLX_M_DIV(clk->address), &m);
	if (ret < 0)
		return ret;

	/*
	 * If no ref is valid, pll will run in free run mode.
	 * At this point the NCO of the DPLL will output a free run frequency
	 * thus the output frequency of the PLL block will be:
	 * f NCO * M-div / 2
	 */
	i = ad9545_pll_get_parent(hw);
	if (i == clk->num_parents)
		return DIV_ROUND_UP(clk->free_run_freq, 2) * m;

	parent_rate = clk_hw_get_rate(clk->parents[i]);

	ret = regmap_bulk_read(clk->st->regmap, AD9545_DPLLX_N_DIV(clk->address, i), &regval, 4);
	if (ret < 0)
		return ret;

	n = le32_to_cpu(regval) + 1;

	regval = 0;
	ret = regmap_bulk_read(clk->st->regmap, AD9545_DPLLX_FRAC_DIV(clk->address, i), &regval, 3);
	if (ret < 0)
		return ret;

	frac = le32_to_cpu(regval);

	regval = 0;
	ret = regmap_bulk_read(clk->st->regmap, AD9545_DPLLX_MOD_DIV(clk->address, i), &regval, 3);
	if (ret < 0)
		return ret;

	mod = le32_to_cpu(regval);

	if (clk->internal_zero_delay) {
		int output_nr = clk->internal_zero_delay_source;
		u32 q_div;

		ret = ad9545_get_q_div(clk->st, output_nr, &q_div);
		if (ret < 0)
			return ret;

		if (!m)
			return 0;

		if (clk->profiles[i].fb_tagging) {
			ret = regmap_bulk_read(clk->st->regmap,
					       ad9545_out_regs[output_nr].modulation_counter_reg,
					       &regval, 4);
			if (ret < 0)
				return ret;

			n = le32_to_cpu(regval);
		} else {
			ret = regmap_bulk_read(clk->st->regmap,
					       AD9545_DPLLX_HITLESS_N(clk->address, i),
					       &regval, 4);
			if (ret < 0)
				return ret;

			n = le32_to_cpu(regval) + 1;
		}

		output_rate = 2 * q_div * parent_rate * n;
	} else {
		if (!mod)
			return 0;

		/* Output rate of APLL = parent_rate * (N + (Frac / Mod)) * M */
		output_rate = mul_u64_u32_div(frac * parent_rate, m, mod);
		output_rate += parent_rate * n * m;
	}

	return output_rate / 2;
}

static long ad9545_pll_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	struct ad9545_pll_clk *clk = to_pll_clk(hw);
	unsigned long frac;
	unsigned long mod;
	bool zero_delay;
	int ret;
	u64 ftw;
	int i;
	u32 m;
	u32 n;

	clk->rate_requested_hz = rate;

	/* if no ref is valid, check if requested rate can be set in free run mode */
	i = ad9545_pll_get_parent(hw);
	if (i == clk->num_parents) {
		/* in free run mode output freq is given by f NCO * m / 2 */
		m = ad9545_calc_m_div(rate * 2);
		ret = ad9545_calc_ftw(clk,  div_u64(rate * 2, m), &ftw);
		if (ret < 0)
			return ret;

		return rate;
	}

	zero_delay = clk->internal_zero_delay;

	return ad9545_calc_pll_params(clk, rate, *parent_rate, zero_delay, &m, &n, &frac, &mod, i);
}

static int ad9545_pll_set_feedback_tagging(struct ad9545_pll_clk *clk, unsigned long rate,
					   unsigned long parent_rate, unsigned int profile_nr,
					   u32 *n)
{
	unsigned long out_rate_hz;
	int fb_output_nr;
	__le32 regval;
	u32 q_div;
	int ret;
	u8 val;

	clk->profiles[profile_nr].fb_tagging = true;

	val = FIELD_PREP(AD9545_TAG_MODE_MSK, AD9545_FB_PATH_TAG) | AD9545_BASE_FILTER_MSK;
	ret = regmap_update_bits(clk->st->regmap, AD9545_DPLLX_FB_MODE(clk->address, profile_nr),
				 AD9545_BASE_FILTER_MSK | AD9545_TAG_MODE_MSK, val);
	if (ret < 0)
		return ret;

	fb_output_nr = clk->internal_zero_delay_source;
	out_rate_hz = clk->internal_zero_delay_source_rate_hz;

	if (!out_rate_hz)
		return 0;

	q_div = DIV_ROUND_UP_ULL(rate, out_rate_hz);

	/* Limit tagging frequency via N-Div */
	*n = DIV_ROUND_UP(out_rate_hz, AD9545_IN_MAX_TDC_FREQ_HZ);
	regval = cpu_to_le32(*n - 1);
	ret = regmap_bulk_write(clk->st->regmap, AD9545_DPLLX_HITLESS_N(clk->address, profile_nr),
				&regval, 4);
	if (ret < 0)
		return ret;

	/* determine modulation period */
	regval = cpu_to_le32(DIV_ROUND_CLOSEST_ULL(rate, q_div * parent_rate));
	ret = regmap_bulk_write(clk->st->regmap,
				ad9545_out_regs[fb_output_nr].modulation_counter_reg,
				&regval, 4);
	if (ret < 0)
		return ret;

	return regmap_update_bits(clk->st->regmap, ad9545_out_regs[fb_output_nr].modulator_reg,
				  AD9545_MODULATOR_EN, AD9545_MODULATOR_EN);
}

static int ad9545_pll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct ad9545_pll_clk *clk = to_pll_clk(hw);
	unsigned long out_rate;
	unsigned long frac;
	unsigned long mod;
	int fb_output_nr;
	bool zero_delay;
	__le32 regval;
	u32 q_div;
	int ret;
	u32 m;
	u32 n;
	int i;

	/*
	 * When setting a PLL rate, precalculate params for all enabled profiles.
	 * At this point there may or may not be a valid reference.
	 */

	if (!rate)
		return -EINVAL;

	for (i = 0; i < clk->num_parents; i++) {
		parent_rate = clk_hw_get_rate(clk->parents[i]);

		zero_delay = clk->internal_zero_delay;
		out_rate = ad9545_calc_pll_params(clk, rate, parent_rate, zero_delay, &m, &n, &frac,
						  &mod, i);

		/*
		 * Feedback under 2 kHz needs to be from tagged
		 * sources when hitless mode is active.
		 */
		clk->profiles[i].fb_tagging = false;
		if (zero_delay && parent_rate < 2000) {
			ret = ad9545_pll_set_feedback_tagging(clk, rate, parent_rate, i, &n);
		} else if (zero_delay) {
			ret = regmap_write(clk->st->regmap,
					   AD9545_DPLLX_FB_MODE(clk->address, i),
					   FIELD_PREP(AD9545_EN_HITLESS_MSK, 1));
		} else {
			ret = regmap_write(clk->st->regmap,
					   AD9545_DPLLX_FB_MODE(clk->address, i), 0);
		}

		if (ret < 0)
			return ret;

		if (zero_delay) {
			/*
			 * In zero delay mode: Phase Buildout N needs to be programmed to:
			 * N BUILDOUT = N HITLESS × 2 × Q/M
			 */

			fb_output_nr = clk->internal_zero_delay_source;
			if (!clk->internal_zero_delay_source_rate_hz)
				return 0;

			q_div = DIV_ROUND_UP_ULL(rate, clk->internal_zero_delay_source_rate_hz);
			regval = cpu_to_le32(DIV_ROUND_UP(n * 2 * q_div, m) - 1);
		} else {
			regval = cpu_to_le32(n - 1);
		}

		ret = regmap_bulk_write(clk->st->regmap,
					AD9545_DPLLX_N_DIV(clk->address, i),
					&regval, 4);
		if (ret < 0)
			return ret;

		regval = cpu_to_le32(n - 1);
		ret = regmap_bulk_write(clk->st->regmap,
					AD9545_DPLLX_HITLESS_N(clk->address, i),
					&regval, 4);
		if (ret < 0)
			return ret;

		ret = regmap_write(clk->st->regmap, AD9545_APLLX_M_DIV(clk->address), m);
		if (ret < 0)
			return ret;

		regval = cpu_to_le32(frac);
		ret = regmap_bulk_write(clk->st->regmap, AD9545_DPLLX_FRAC_DIV(clk->address, i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval = cpu_to_le32(mod);
		ret = regmap_bulk_write(clk->st->regmap, AD9545_DPLLX_MOD_DIV(clk->address, i),
					&regval, 3);
		if (ret < 0)
			return ret;
	}

	return ad9545_set_freerun_freq(clk, div_u64(rate * 2, m));
}

static const struct clk_ops ad9545_pll_clk_ops = {
	.recalc_rate = ad9545_pll_clk_recalc_rate,
	.round_rate = ad9545_pll_clk_round_rate,
	.set_rate = ad9545_pll_set_rate,
	.get_parent = ad9545_pll_get_parent,
	.debug_init = ad9545_pll_debug_init,
};

static int ad9545_pll_fast_acq_setup(struct ad9545_pll_clk *pll, int profile)
{
	struct ad9545_state *st = pll->st;
	unsigned int tmp;
	int ret;
	u8 reg;
	int i;

	tmp = pll->profiles[profile].fast_acq_excess_bw;
	for (i = 0; i < ARRAY_SIZE(ad9545_fast_acq_excess_bw_map); i++)
		if (tmp == ad9545_fast_acq_excess_bw_map[i])
			break;

	if (i < ARRAY_SIZE(ad9545_fast_acq_excess_bw_map)) {
		ret = regmap_write(st->regmap, AD9545_DPLLX_FAST_L1(pll->address, profile), i);
		if (ret < 0)
			return ret;
	} else {
		dev_err(st->dev, "Wrong fast_acq_excess_bw value for DPLL %u, profile: %d.",
			pll->address, profile);
		return -EINVAL;
	}

	tmp = pll->profiles[profile].fast_acq_settle_ms;
	for (i = 0; i < ARRAY_SIZE(ad9545_fast_acq_timeout_map); i++)
		if (tmp == ad9545_fast_acq_timeout_map[i])
			break;

	if (i == ARRAY_SIZE(ad9545_fast_acq_timeout_map)) {
		dev_err(st->dev, "Wrong fast_acq_settle_ms value for DPLL %u, profile %d.",
			pll->address, profile);
		return -EINVAL;
	}

	reg = i;

	tmp = pll->profiles[profile].fast_acq_timeout_ms;
	for (i = 0; i < ARRAY_SIZE(ad9545_fast_acq_timeout_map); i++)
		if (tmp == ad9545_fast_acq_timeout_map[i])
			break;

	if (i == ARRAY_SIZE(ad9545_fast_acq_timeout_map)) {
		dev_err(st->dev, "Wrong fast_acq_timeout_ms value for for DPLL %u, profile %d.",
			pll->address, profile);
		return -EINVAL;
	}

	reg |= i << 4;

	ret = regmap_write(st->regmap, AD9545_DPLLX_FAST_L2(pll->address, profile), reg);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, AD9545_DPLLX_FAST_MODE(pll->address),
			    pll->fast_acq_trigger_mode);
}

static int ad9545_plls_setup(struct ad9545_state *st)
{
	struct clk_init_data init[2] = {0};
	struct ad9545_pll_clk *pll;
	struct clk_hw *hw;
	int tdc_source;
	__le32 regval;
	int ret;
	u8 reg;
	int i;
	int j;

	st->clks[AD9545_CLK_PLL] = devm_kzalloc(st->dev, ARRAY_SIZE(ad9545_pll_clk_names) *
						sizeof(struct clk *), GFP_KERNEL);
	if (!st->clks[AD9545_CLK_PLL])
		return -ENOMEM;

	for (i = 0; i < 2; i++) {
		pll = &st->pll_clks[i];
		if (!pll->pll_used)
			continue;

		pll->st = st;
		pll->address = i;

		init[i].name = ad9545_pll_clk_names[i];
		init[i].ops = &ad9545_pll_clk_ops;
		init[i].num_parents = 0;
		for (j = 0; j < AD9545_MAX_DPLL_PROFILES; j++)
			if (pll->profiles[j].en)
				init[i].num_parents++;

		init[i].parent_hws = devm_kzalloc(st->dev, init[i].num_parents *
						  sizeof(*init[i].parent_hws), GFP_KERNEL);
		if (!init[i].parent_hws)
			return -ENOMEM;

		pll->num_parents = init[i].num_parents;
		pll->parents = init[i].parent_hws;

		if (!pll->slew_rate_limit_ps) {
			regval = cpu_to_le32(pll->slew_rate_limit_ps);
			ret = regmap_bulk_write(st->regmap, AD9545_DPLLX_SLEW_RATE(i), &regval, 4);
			if (ret < 0)
				return ret;
		}

		for (j = 0; j < AD9545_MAX_DPLL_PROFILES; j++) {
			if (!pll->profiles[j].en)
				continue;

			pll->profiles[j].parent_index = j;

			/* enable pll profile */
			reg = AD9545_EN_PROFILE_MSK |
				FIELD_PREP(AD9545_SEL_PRIORITY_MSK, pll->profiles[j].priority);
			ret = regmap_write(st->regmap, AD9545_DPLLX_EN(i, j), reg);
			if (ret < 0)
				return ret;

			reg = pll->internal_zero_delay_source;
			if (i > 0) {
				/* output numbering in this reg starts from 0 for each DPLL */
				if (reg > 5)
					reg -= 6;
			}

			ret = regmap_write(st->regmap, AD9545_DPLLX_FB_PATH(i, j), reg);
			if (ret < 0)
				return ret;

			reg = FIELD_PREP(AD9545_EN_HITLESS_MSK, pll->internal_zero_delay);
			ret = regmap_write(st->regmap, AD9545_DPLLX_FB_MODE(i, j), reg);
			if (ret < 0)
				return ret;

			/* set TDC source */
			tdc_source = pll->profiles[j].tdc_source;
			ret = regmap_write(st->regmap, AD9545_DPLLX_SOURCE(i, j),
					   ad9545_tdc_source_mapping[tdc_source]);
			if (ret < 0)
				return ret;

			regval = cpu_to_le32(pll->profiles[j].loop_bw_uhz);
			ret = regmap_bulk_write(st->regmap, AD9545_DPLLX_LOOP_BW(i, j), &regval, 4);
			if (ret < 0)
				return ret;

			if (pll->profiles[j].tdc_source >= ARRAY_SIZE(ad9545_ref_clk_names))
				hw = &st->aux_nco_clks[tdc_source - ARRAY_SIZE(ad9545_ref_clk_names)].hw;
			else
				hw = &st->ref_in_clks[tdc_source].hw;
			init[i].parent_hws[j] = hw;

			if (pll->profiles[j].fast_acq_excess_bw > 0) {
				ret = ad9545_pll_fast_acq_setup(pll, j);
				if (ret < 0)
					return ret;
			}
		}

		pll->hw.init = &init[i];
		ret = devm_clk_hw_register(st->dev, &pll->hw);
		if (ret < 0)
			return ret;

		st->clks[AD9545_CLK_PLL][i] = &pll->hw;
	}

	return 0;
}

static int ad9545_get_nco_center_freq(struct ad9545_state *st, int addr, u64 *freq)
{
	__le64 regval64;
	int ret;

	regval64 = 0;
	ret = regmap_bulk_read(st->regmap, AD9545_NCOX_CENTER_FREQ(addr), &regval64, 7);
	if (ret < 0)
		return ret;

	*freq = le64_to_cpu(regval64);

	return 0;
}

static int ad9545_set_nco_center_freq(struct ad9545_state *st, int addr, u64 freq)
{
	__le64 regval64;

	if (freq > AD9545_NCO_CENTER_FREQ_MAX)
		return -EINVAL;

	regval64 = cpu_to_le64(freq);

	return regmap_bulk_write(st->regmap, AD9545_NCOX_CENTER_FREQ(addr), &regval64, 7);
}

static int ad9545_get_nco_offset_freq(struct ad9545_state *st, int addr, u32 *freq)
{
	__le32 regval;
	int ret;

	ret = regmap_bulk_read(st->regmap, AD9545_NCOX_OFFSET_FREQ(addr), &regval, 4);
	if (ret < 0)
		return ret;

	*freq = le32_to_cpu(regval);

	return 0;
}

static int ad9545_set_nco_offset_freq(struct ad9545_state *st, int addr, u32 freq)
{
	__le32 regval;

	regval = cpu_to_le32(freq);

	return regmap_bulk_write(st->regmap, AD9545_NCOX_OFFSET_FREQ(addr), &regval, 4);
}

static int ad9545_get_nco_freq(struct ad9545_state *st, int addr, u64 *freq)
{
	u64 center_freq;
	u32 offset_freq;
	int shift;
	int ret;

	ret = ad9545_get_nco_center_freq(st, addr, &center_freq);
	if (ret < 0)
		return ret;

	ret = ad9545_get_nco_offset_freq(st, addr, &offset_freq);
	if (ret < 0)
		return ret;

	/* align center frequency and offset frequency, and then add */
	shift = AD9545_NCO_CENTER_FREQ_FRAC_WIDTH - AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH;
	*freq = center_freq + ((u64)offset_freq << shift);

	return 0;
}

static int ad9545_get_nco_freq_hz(struct ad9545_state *st, int addr, u32 *freq)
{
	u64 val64;
	u32 freq_int;
	int ret;

	ret = ad9545_get_nco_freq(st, addr, &val64);
	if (ret < 0)
		return ret;

	freq_int = val64 >> AD9545_NCO_CENTER_FREQ_FRAC_WIDTH;

	if (val64 & BIT_ULL(AD9545_NCO_CENTER_FREQ_FRAC_WIDTH - 1))
		freq_int++;

	*freq = freq_int;

	return 0;
}

static int ad9545_set_nco_freq_hz(struct ad9545_state *st, int addr, u32 freq)
{
	u64 center_freq;
	u32 center_freq_int, offset_freq, offset_freq_int;
	bool use_fractional;
	int ret;

	if (freq > AD9545_NCO_FREQ_INT_MAX + 1)
		return -EINVAL;

	use_fractional = (freq == AD9545_NCO_FREQ_INT_MAX + 1);
	if (use_fractional)
		freq--;

	if (freq <= AD9545_NCO_CENTER_FREQ_INT_MAX) {
		center_freq_int = freq;
		offset_freq_int = 0;
	} else {
		center_freq_int = AD9545_NCO_CENTER_FREQ_INT_MAX;
		offset_freq_int = freq - center_freq_int;
	}

	center_freq = FIELD_PREP(AD9545_NCO_CENTER_FREQ_INT_MSK, center_freq_int);
	if (use_fractional)
		center_freq |= BIT_ULL(AD9545_NCO_CENTER_FREQ_FRAC_WIDTH - 1);

	ret = ad9545_set_nco_center_freq(st, addr, center_freq);
	if (ret < 0)
		return ret;

	offset_freq = FIELD_PREP(AD9545_NCO_OFFSET_FREQ_INT_MSK, offset_freq_int);
	if (use_fractional)
		offset_freq |= BIT(AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH - 1);

	ret = ad9545_set_nco_offset_freq(st, addr, offset_freq);
	if (ret < 0)
		return ret;

	return ad9545_io_update(st);
}

static unsigned long ad9545_nco_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ad9545_aux_nco_clk *clk = to_nco_clk(hw);
	u32 rate;
	int ret;

	ret = ad9545_get_nco_freq_hz(clk->st, clk->address, &rate);
	if (ret < 0) {
		dev_err(clk->st->dev, "Could not read NCO freq.");
		return 0;
	}

	return rate;
}

static long ad9545_nco_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	return min(rate, (unsigned long)AD9545_NCO_FREQ_INT_MAX + 1);
}

static int ad9545_nco_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct ad9545_aux_nco_clk *clk = to_nco_clk(hw);

	return ad9545_set_nco_freq_hz(clk->st, clk->address, rate);
}

static const struct clk_ops ad9545_nco_clk_ops = {
	.recalc_rate = ad9545_nco_clk_recalc_rate,
	.round_rate = ad9545_nco_clk_round_rate,
	.set_rate = ad9545_nco_clk_set_rate,
};

static int ad9545_aux_ncos_setup(struct ad9545_state *st)
{
	struct clk_init_data init[2] = {0};
	struct ad9545_aux_nco_clk *nco;
	__le32 regval;
	int ret;
	int i;

	st->clks[AD9545_CLK_NCO] = devm_kzalloc(st->dev, ARRAY_SIZE(ad9545_aux_nco_clk_names) *
						sizeof(struct clk *), GFP_KERNEL);
	if (!st->clks[AD9545_CLK_NCO])
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(st->aux_nco_clks); i++) {
		nco = &st->aux_nco_clks[i];
		if (!nco->nco_used)
			continue;

		regval = cpu_to_le32(nco->freq_thresh_ps);
		ret = regmap_bulk_write(st->regmap, AD9545_NCOX_FREQ_THRESH(i), &regval, 3);
		if (ret < 0)
			return ret;

		regval = cpu_to_le32(nco->phase_thresh_ps);
		ret = regmap_bulk_write(st->regmap, AD9545_NCOX_PHASE_THRESH(i), &regval, 3);
		if (ret < 0)
			return ret;

		init[i].name = ad9545_aux_nco_clk_names[i];
		init[i].ops = &ad9545_nco_clk_ops;

		nco->hw.init = &init[i];
		ret = devm_clk_hw_register(st->dev, &nco->hw);
		if (ret < 0)
			return ret;

		st->clks[AD9545_CLK_NCO][i] = &nco->hw;
	}

	return 0;
}

static int ad9545_set_tdc_div(struct ad9545_aux_tdc_clk *clk, u32 div)
{
	int ret;

	if (!div)
		return -EINVAL;

	ret = regmap_write(clk->st->regmap, AD9545_TDCX_DIV(clk->address), --div);
	if (ret < 0)
		return ret;

	return ad9545_io_update(clk->st);
}

static unsigned long ad9545_tdc_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ad9545_aux_tdc_clk *clk = to_tdc_clk(hw);
	int ret;
	u32 div;

	ret = regmap_read(clk->st->regmap, AD9545_TDCX_DIV(clk->address), &div);
	if (ret < 0) {
		dev_err(clk->st->dev, "Could not read TDC freq.");
		return ret;
	}

	div++;

	return DIV_ROUND_UP_ULL((u64)parent_rate, div);
}

static long ad9545_tdc_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	u32 div;

	if (!*parent_rate || !rate)
		return 0;

	div = clamp_t(u8, DIV_ROUND_UP_ULL((u64)*parent_rate, rate), 1, U8_MAX);

	return DIV_ROUND_UP_ULL((u64)*parent_rate, div);
}

static int ad9545_tdc_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct ad9545_aux_tdc_clk *clk = to_tdc_clk(hw);
	__le64 regval64;
	u64 period_es;
	u32 div;
	int ret;

	if (!parent_rate || !rate)
		return 0;

	/* write parent rate received at the Mx pin in attoseconds */
	period_es = div_u64(1000000000000000000ULL, parent_rate);
	regval64 = cpu_to_le64(period_es);
	ret = regmap_bulk_write(clk->st->regmap, AD9545_TDCX_PERIOD(clk->address), &regval64, 8);
	if (ret < 0)
		return ret;

	div = clamp_t(u8, DIV_ROUND_UP_ULL((u64)parent_rate, rate), 1, U8_MAX);

	return ad9545_set_tdc_div(clk, div);
}

static const struct clk_ops ad9545_aux_tdc_clk_ops = {
	.recalc_rate = ad9545_tdc_clk_recalc_rate,
	.round_rate = ad9545_tdc_clk_round_rate,
	.set_rate = ad9545_tdc_clk_set_rate,
};

static int ad9545_aux_tdcs_setup(struct ad9545_state *st)
{
	struct clk_init_data init[ARRAY_SIZE(ad9545_aux_tdc_clk_names)] = {0};
	struct ad9545_aux_tdc_clk *tdc;
	int ret;
	int i;

	st->clks[AD9545_CLK_AUX_TDC] = devm_kcalloc(st->dev, ARRAY_SIZE(ad9545_aux_tdc_clk_names),
						    sizeof(*st->clks[AD9545_CLK_AUX_TDC]),
						    GFP_KERNEL);
	if (!st->clks[AD9545_CLK_AUX_TDC])
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(st->aux_tdc_clks); i++) {
		tdc = &st->aux_tdc_clks[i];
		if (!tdc->tdc_used)
			continue;

		/* redirect Mx pin to this TDC */
		ret = regmap_write(st->regmap, AD9545_MX_PIN(tdc->pin_nr),
				   AD9545_MX_TO_TDCX(tdc->address));
		if (ret < 0)
			return ret;

		init[i].name = ad9545_aux_tdc_clk_names[i];
		init[i].ops = &ad9545_aux_tdc_clk_ops;
		init[i].parent_names = &ad9545_ref_m_clk_names[tdc->pin_nr];
		init[i].num_parents = 1;
		tdc->hw.init = &init[i];
		ret = devm_clk_hw_register(st->dev, &tdc->hw);
		if (ret < 0)
			return ret;

		st->clks[AD9545_CLK_AUX_TDC][i] = &tdc->hw;
	}

	return 0;
}

static unsigned long ad9545_aux_dpll_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	return parent_rate;
}

static const struct clk_ops ad9545_aux_dpll_clk_ops = {
	.recalc_rate = ad9545_aux_dpll_clk_recalc_rate,
	.debug_init = ad9545_aux_dpll_clk_debug_init,
};

static int ad9545_aux_dpll_setup(struct ad9545_state *st)
{
	struct ad9545_aux_dpll_clk *clk;
	struct clk_init_data init;
	__le16 regval;
	int ret;
	u32 val;
	int i;

	clk = &st->aux_dpll_clk;
	if (!clk->dpll_used)
		return 0;

	memset(&init, 0, sizeof(struct clk_init_data));

	if (clk->source < ARRAY_SIZE(ad9545_in_clk_names)) {
		val = clk->source;
		init.parent_names = &ad9545_in_clk_names[val];
	} else {
		val = clk->source - ARRAY_SIZE(ad9545_in_clk_names);
		if (val > ARRAY_SIZE(ad9545_aux_tdc_clk_names))
			return -EINVAL;
		init.parent_names = &ad9545_aux_tdc_clk_names[val];
		val = val + 0x6;
	}

	ret = regmap_write(st->regmap, AD9545_AUX_DPLL_SOURCE, val);
	if (ret < 0)
		return ret;

	/* write loop bandwidth in dHz */
	regval = cpu_to_le16(clk->loop_bw_mhz / 100);
	ret = regmap_bulk_write(st->regmap, AD9545_AUX_DPLL_LOOP_BW, &regval, 2);
	if (ret < 0)
		return ret;

	val = 0;
	for (i = 0; i < ARRAY_SIZE(ad9545_rate_change_limit_map); i++) {
		if (ad9545_rate_change_limit_map[i] == clk->rate_change_limit) {
			val = i;
			break;
		}
	}

	ret = regmap_write(st->regmap, AD9545_AUX_DPLL_CHANGE_LIMIT, val);
	if (ret < 0)
		return ret;

	/* add compensation destination */
	ret = regmap_write(st->regmap, AD9545_COMPENSATE_DPLL, AD9545_COMPNESATE_VIA_AUX_DPLL);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, AD9545_COMPENSATE_NCOS, AD9545_COMPENSATE_NCOS_VIA_AUX_DPLL);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, AD9545_COMPENSATE_TDCS, AD9545_COMPENSATE_TDCS_VIA_AUX_DPLL);
	if (ret < 0)
		return ret;

	init.name = ad9545_aux_dpll_name;
	init.ops = &ad9545_aux_dpll_clk_ops;
	init.num_parents = 1;

	clk->hw.init = &init;
	return devm_clk_hw_register(st->dev, &clk->hw);
}

static int ad9545_calib_system_clock(struct ad9545_state *st)
{
	int ret;
	u32 reg;
	int i;
	int j;

	for (i = 0; i < 2; i++) {
		for (j = 0; j < ARRAY_SIZE(ad9545_vco_calibration_op); j++) {
			ret = regmap_write(st->regmap, ad9545_vco_calibration_op[j][0],
					   ad9545_vco_calibration_op[j][1]);
			if (ret < 0)
				return ret;
		}

		/* wait for sys pll to lock and become stable */
		msleep(50 + AD9545_SYS_CLK_STABILITY_MS);

		ret = regmap_read(st->regmap, AD9545_PLL_STATUS, &reg);
		if (ret < 0)
			return ret;

		if (AD9545_SYS_PLL_STABLE(reg)) {
			ret = regmap_write(st->regmap, AD9545_CALIB_CLK, 0);
			if (ret < 0)
				return ret;

			return ad9545_io_update(st);
		}
	}

	dev_err(st->dev, "System PLL unlocked.\n");
	return -EIO;
}

static int ad9545_calib_apll(struct ad9545_state *st, int i)
{
	int cal_count;
	u32 reg;
	int ret;

	/* APLL VCO calibration operation */
	cal_count = 0;
	while (cal_count < 2) {
		ret = regmap_write(st->regmap, AD9545_PWR_CALIB_CHX(i), 0);
		if (ret < 0)
			return ret;

		ret = ad9545_io_update(st);
		if (ret < 0)
			return ret;

		ret = regmap_write(st->regmap, AD9545_PWR_CALIB_CHX(i),
				   AD9545_CALIB_APLL);
		if (ret < 0)
			return ret;

		ret = ad9545_io_update(st);
		if (ret < 0)
			return ret;

		cal_count += 1;
		msleep(100);

		ret = regmap_read(st->regmap, AD9545_PLLX_STATUS(i), &reg);
		if (ret < 0)
			return ret;

		if (AD9545_APLL_LOCKED(reg)) {
			ret = regmap_write(st->regmap, AD9545_PWR_CALIB_CHX(i), 0);
			if (ret < 0)
				return ret;

			ret = ad9545_io_update(st);
			if (ret < 0)
				return ret;

			cal_count = 2;
			break;
		}
	}

	return ret;
}

static int ad9545_calib_aplls(struct ad9545_state *st)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(ad9545_pll_clk_names); i++) {
		if (!st->pll_clks[i].pll_used)
			continue;

		ret = ad9545_calib_apll(st, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct clk_hw *ad9545_clk_hw_twocell_get(struct of_phandle_args *clkspec, void *data)
{
	unsigned int clk_address = clkspec->args[1];
	unsigned int clk_type = clkspec->args[0];
	struct clk_hw ***clks = data;

	if (clk_type > AD9545_CLK_AUX_TDC) {
		pr_err("%s: invalid clock type %u\n", __func__, clk_type);
		return ERR_PTR(-EINVAL);
	}
	if ((clk_type == AD9545_CLK_PLL && clk_address > AD9545_PLL1) ||
	    (clk_type == AD9545_CLK_OUT && clk_address > AD9545_Q1BB) ||
	    (clk_type == AD9545_CLK_NCO && clk_address > AD9545_NCO1) ||
	    (clk_type == AD9545_CLK_AUX_TDC && clk_address > AD9545_CLK_AUX_TDC1)) {
		pr_err("%s: invalid clock address %u\n", __func__, clk_address);
		return ERR_PTR(-EINVAL);
	}

	return clks[clk_type][clk_address];
}

static int ad9545_setup(struct ad9545_state *st)
{
	int ret;
	u32 val;
	int i;

	ret = regmap_update_bits(st->regmap, AD9545_CONFIG_0, AD9545_RESET_REGS, AD9545_RESET_REGS);
	if (ret < 0)
		return ret;

	ret = ad9545_sys_clk_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_input_refs_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_aux_ncos_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_calib_system_clock(st);
	if (ret < 0)
		return ret;

	ret = ad9545_aux_tdcs_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_aux_dpll_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_calib_aplls(st);
	if (ret < 0)
		return ret;

	ret = ad9545_io_update(st);
	if (ret < 0)
		return ret;

	ret = ad9545_plls_setup(st);
	if (ret < 0)
		return ret;

	ret = ad9545_outputs_setup(st);
	if (ret < 0)
		return ret;

	ret = devm_of_clk_add_hw_provider(st->dev, ad9545_clk_hw_twocell_get,
					  &st->clks[AD9545_CLK_OUT]);
	if (ret < 0)
		return ret;

	ret = ad9545_calib_aplls(st);
	if (ret < 0)
		return ret;

	/* check locks */
	ret = regmap_read(st->regmap, AD9545_PLL_STATUS, &val);
	for (i = 0; i < ARRAY_SIZE(st->pll_clks); i++)
		if (st->pll_clks[i].pll_used && !AD9545_PLLX_LOCK(i, val))
			dev_warn(st->dev, "PLL%d unlocked.\n", i);

	if (st->aux_dpll_clk.dpll_used) {
		ret = regmap_read(st->regmap, AD9545_MISC, &val);
		if (ret < 0)
			return ret;

		if (!(val & AD9545_AUX_DPLL_LOCK_MSK))
			dev_warn(st->dev, "Aux DPLL unlocked.\n");

		if (val & AD9545_AUX_DPLL_REF_FAULT)
			dev_warn(st->dev, "Aux DPLL reference fault.\n");
	}

	return 0;
}

int ad9545_probe(struct device *dev, struct regmap *regmap)
{
	struct ad9545_state *st;
	int ret;

	st = devm_kzalloc(dev, sizeof(struct ad9545_state), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->dev = dev;
	st->regmap = regmap;

	ret = ad9545_check_id(st);
	if (ret < 0)
		return ret;

	ret = ad9545_parse_dt(st);
	if (ret < 0)
		return ret;

	return ad9545_setup(st);
}
EXPORT_SYMBOL_GPL(ad9545_probe);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9545");
MODULE_LICENSE("Dual BSD/GPL");
