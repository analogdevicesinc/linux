/*
 * ADI AXI-ADXCVR Module
 *
 * Copyright 2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_adxcvr
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#define PCORE_VER(major, minor, letter)	((major << 16) | (minor << 8) | letter)
#define PCORE_VER_MAJOR(version)		(version >> 16)
#define PCORE_VER_MINOR(version)		((version >> 8) & 0xff)
#define PCORE_VER_LETTER(version)		(version & 0xff)

#define ADXCVR_REG_VERSION			0x0000
#define ADXCVR_VERSION(x)			(((x) & 0xffffffff) << 0)
#define ADXCVR_VERSION_IS(x, y, z)	((x) << 16 | (y) << 8 | (z))
#define ADXCVR_VERSION_MAJOR(x)		((x) >> 16)

#define ADXCVR_REG_ID				0x0004

#define ADXCVR_REG_SCRATCH			0x0008

#define ADXCVR_REG_RESETN			0x0010
#define ADXCVR_RESETN				(1 << 0)

#define ADXCVR_REG_STATUS			0x0014
#define ADXCVR_STATUS				(1 << 0)

#define ADXCVR_REG_CONTROL			0x0020
#define ADXCVR_LPM_DFE_N			(1 << 12)
#define ADXCVR_RATE(x)				(((x) & 0x7) << 8)
#define ADXCVR_SYSCLK_SEL(x)		(((x) & 0x3) << 4)
#define ADXCVR_OUTCLK_SEL(x)		(((x) & 0x7) << 0)

#define ADXCVR_REG_CM_SEL			0x0040

#define ADXCVR_REG_CM_CONTROL		0x0044
#define ADXCVR_CM_WR				(1 << 28)
#define ADXCVR_CM_ADDR(x)			(((x) & 0xFFF) << 16)
#define ADXCVR_CM_WDATA(x)			(((x) & 0xFFFF) << 0)

#define ADXCVR_REG_CM_STATUS		0x0048
#define ADXCVR_CM_BUSY				(1 << 16)
#define ADXCVR_CM_RDATA(x)			(((x) & 0xFFFF) << 0)

#define ADXCVR_REG_CH_SEL			0x0060

#define ADXCVR_REG_CH_CONTROL		0x0064
#define ADXCVR_CH_WR				(1 << 28)
#define ADXCVR_CH_ADDR(x)			(((x) & 0xFFF) << 16)
#define ADXCVR_CH_WDATA(x)			(((x) & 0xFFFF) << 0)

#define ADXCVR_REG_CH_STATUS		0x0068
#define ADXCVR_CH_BUSY				(1 << 16)
#define ADXCVR_CH_RDATA(x)			(((x) & 0xFFFF) << 0)

#define ADXCVR_BROADCAST			0xff

#define TXOUT_DIV_ADDR				0x88
#define TXOUT_DIV_MASK				0x70
#define TXOUT_DIV_OFFSET			0x4
#define TXOUT_DIV_WIDTH				0x3
#define TXOUT_DIV_DEFAULT			0x0

#define RXOUT_DIV_ADDR				0x88
#define RXOUT_DIV_MASK				0x7
#define RXOUT_DIV_OFFSET			0x0
#define RXOUT_DIV_WIDTH				0x3
#define RXOUT_DIV_DEFAULT			0x0

#define RXCDR_CFG0_ADDR				0xa8
#define RXCDR_CFG0_MASK				0xffff
#define RXCDR_CFG0_OFFSET			0x0
#define RXCDR_CFG0_WIDTH			0x10
#define RXCDR_CFG0_DEFAULT			0x0

#define RXCDR_CFG1_ADDR				0xa9
#define RXCDR_CFG1_MASK				0xffff
#define RXCDR_CFG1_OFFSET			0x0
#define RXCDR_CFG1_WIDTH			0x10
#define RXCDR_CFG1_DEFAULT			0x0

#define RXCDR_CFG2_ADDR				0xaa
#define RXCDR_CFG2_MASK				0xffff
#define RXCDR_CFG2_OFFSET			0x0
#define RXCDR_CFG2_WIDTH			0x10
#define RXCDR_CFG2_DEFAULT			0x0

#define RXCDR_CFG3_ADDR				0xab
#define RXCDR_CFG3_MASK				0xffff
#define RXCDR_CFG3_OFFSET			0x0
#define RXCDR_CFG3_WIDTH			0x10
#define RXCDR_CFG3_DEFAULT			0x0

#define RXCDR_CFG4_ADDR				0xac
#define RXCDR_CFG4_MASK				0xff
#define RXCDR_CFG4_OFFSET			0x0
#define RXCDR_CFG4_WIDTH			0x8
#define RXCDR_CFG4_DEFAULT			0x0

#define RX_DFE_LPM_CFG_ADDR			0x29
#define RX_DFE_LPM_CFG_MASK			0xffff
#define RX_DFE_LPM_CFG_OFFSET		0x0
#define RX_DFE_LPM_CFG_WIDTH		0x10
#define RX_DFE_LPM_CFG_DEFAULT		0x0

#define QPLL_CFG0_ADDR				0x32
#define QPLL_CFG0_MASK				0xffff
#define QPLL_CFG0_BAND_MASK			0x40
#define QPLL_CFG0_OFFSET			0x0
#define QPLL_CFG0_WIDTH				0x10
#define QPLL_CFG0_DEFAULT			0x0

#define QPLL_CFG1_ADDR				0x33
#define QPLL_CFG1_MASK				0x7ff
#define QPLL_CFG1_OFFSET			0x0
#define QPLL_CFG1_WIDTH				0xb
#define QPLL_CFG1_DEFAULT			0x0

#define QPLL_REFCLK_DIV_M_ADDR		0x33
#define QPLL_REFCLK_DIV_M_MASK		0xf800
#define QPLL_REFCLK_DIV_M_OFFSET	0xb
#define QPLL_REFCLK_DIV_M_WIDTH		0x5
#define QPLL_REFCLK_DIV_M_DEFAULT	0x0

#define QPLL_FBDIV_N_ADDR			0x36
#define QPLL_FBDIV_N_MASK			0x3ff
#define QPLL_FBDIV_N_OFFSET			0x0
#define QPLL_FBDIV_N_WIDTH			0xa
#define QPLL_FBDIV_N_DEFAULT		0x0

#define QPLL_FBDIV_RATIO_ADDR		0x37
#define QPLL_FBDIV_RATIO_MASK		0x40
#define QPLL_FBDIV_RATIO_OFFSET		0x6
#define QPLL_FBDIV_RATIO_WIDTH		0x1
#define QPLL_FBDIV_RATIO_DEFAULT	0x0

#define CPLL_CFG0_ADDR				0x5c
#define CPLL_CFG0_MASK				0xff00
#define CPLL_CFG0_OFFSET			0x8
#define CPLL_CFG0_WIDTH				0x8
#define CPLL_CFG0_DEFAULT			0x0

#define CPLL_CFG1_ADDR				0x5d
#define CPLL_CFG1_MASK				0xffff
#define CPLL_CFG1_OFFSET			0x0
#define CPLL_CFG1_WIDTH				0x10
#define CPLL_CFG1_DEFAULT			0x0

#define CPLL_REFCLK_DIV_M_ADDR		0x5e
#define CPLL_REFCLK_DIV_M_MASK		0x1f00
#define CPLL_REFCLK_DIV_M_OFFSET	0x8
#define CPLL_REFCLK_DIV_M_WIDTH		0x5
#define CPLL_REFCLK_DIV_M_DEFAULT	0x0

#define CPLL_FB_DIV_45_N1_ADDR		0x5e
#define CPLL_FB_DIV_45_N1_MASK		0x80
#define CPLL_FB_DIV_45_N1_OFFSET	0x7
#define CPLL_FB_DIV_45_N1_WIDTH		0x1
#define CPLL_FB_DIV_45_N1_DEFAULT	0x0

#define CPLL_FBDIV_N2_ADDR			0x5e
#define CPLL_FBDIV_N2_MASK			0x7f
#define CPLL_FBDIV_N2_OFFSET		0x0
#define CPLL_FBDIV_N2_WIDTH			0x7
#define CPLL_FBDIV_N2_DEFAULT		0x0

#define ENC_8B10B					810

enum refclk_ppm {
	PM_200,
	PM_700,
	PM_1250,
};

struct adxcvr_state {
	struct device		*dev;
	void __iomem		*regs;
	struct clk			*conv_clk;
	struct clk			*lane_rate_div40_clk;
	struct clk_hw		out_clk_hw;
	bool				out_clk_enabled;
	struct work_struct	work;
	unsigned long		lane_rate;
	bool				gth_enable;
	bool				tx_enable;
	u32					sys_clk_sel;
	u32					out_clk_sel;
	bool				cpll_enable;
	bool				lpm_enable;
	uint16_t			encoding;
	enum refclk_ppm		ppm;
};

static inline unsigned int adxcvr_read(struct adxcvr_state *st,
									   unsigned int reg)
{
	dev_vdbg(st->dev, "%s: reg 0x%X val 0x%X\n", __func__,
			 reg, ioread32(st->regs + reg));

	return ioread32(st->regs + reg);
}

static inline void adxcvr_write(struct adxcvr_state *st,
								unsigned int reg,
								unsigned int val)
{
	dev_vdbg(st->dev, "%s: reg 0x%X val 0x%X\n", __func__,
			 reg, val);

	iowrite32(val, st->regs + reg);
}

static unsigned int adxcvr_drp_read(struct adxcvr_state *st,
									unsigned int reg)
{
	bool ch_sel;
	int timeout = 20;
	unsigned int val;

	switch (reg) {
	case QPLL_CFG0_ADDR:
	case QPLL_CFG1_ADDR:
	case QPLL_FBDIV_N_ADDR:
	case QPLL_FBDIV_RATIO_ADDR:
		ch_sel = false;
		break;
	default:
		ch_sel = true;
	}

	adxcvr_write(st, ch_sel ? ADXCVR_REG_CH_SEL : ADXCVR_REG_CM_SEL,
				 ADXCVR_BROADCAST);
	adxcvr_write(st, ch_sel ? ADXCVR_REG_CH_CONTROL : ADXCVR_REG_CM_CONTROL,
				 ADXCVR_CM_ADDR(reg));
	adxcvr_write(st, ch_sel ? ADXCVR_REG_CH_CONTROL : ADXCVR_REG_CM_CONTROL,
				 ch_sel ? ADXCVR_CH_ADDR(reg) : ADXCVR_CM_ADDR(reg));
	do {
		val = adxcvr_read(st, ch_sel ?
						  ADXCVR_REG_CH_STATUS : ADXCVR_REG_CM_STATUS);
		if (val & (ch_sel ? ADXCVR_CH_BUSY : ADXCVR_CM_BUSY)) {
			mdelay(1);
			continue;
		}
		dev_dbg(st->dev, "%s: reg 0x%X val 0x%X\n",
				__func__, reg, val & 0xFFFF);

		return ch_sel ? ADXCVR_CH_RDATA(val) : ADXCVR_CM_RDATA(val);
	} while (timeout--);

	dev_err(st->dev, "%s: Timeout!", __func__);

	return -ETIMEDOUT;
}

static int adxcvr_drp_write(struct adxcvr_state *st,
							unsigned int reg,
							unsigned int val)
{
	bool ch_sel;
	int timeout = 20;
	unsigned int read_val;

	switch (reg) {
	case QPLL_CFG0_ADDR:
	case QPLL_CFG1_ADDR:
	case QPLL_FBDIV_N_ADDR:
	case QPLL_FBDIV_RATIO_ADDR:
		ch_sel = false;
		break;
	default:
		ch_sel = true;
	}

	adxcvr_write(st, ch_sel ? ADXCVR_REG_CH_SEL : ADXCVR_REG_CM_SEL,
			ADXCVR_BROADCAST);
	adxcvr_write(st, ch_sel ? ADXCVR_REG_CH_CONTROL : ADXCVR_REG_CM_CONTROL,
			ch_sel ? (ADXCVR_CH_WR | ADXCVR_CH_ADDR(reg) | ADXCVR_CH_WDATA(val)) :
			(ADXCVR_CM_WR | ADXCVR_CM_ADDR(reg) | ADXCVR_CM_WDATA(val)));

	do {
		if (!(adxcvr_read(st, ch_sel ? ADXCVR_REG_CH_STATUS : ADXCVR_REG_CM_STATUS)
			  & (ch_sel ? ADXCVR_CH_BUSY : ADXCVR_CM_BUSY))) {
			read_val = adxcvr_drp_read(st, reg);
			if (val != read_val)
				dev_err(st->dev, "%s: MISMATCH reg 0x%X val 0x%X != read 0x%X\n",
						__func__, reg, val, read_val);
			return 0;
		}
		mdelay(1);
	} while (timeout--);

	dev_err(st->dev, "%s: Timeout!", __func__);

	return -ETIMEDOUT;
}

static int __adxcvr_drp_writef(struct adxcvr_state *st,
							   u32 reg,
							   u32 mask,
							   u32 offset,
							   u32 val)
{
	u32 tmp;
	int ret;

	if (!mask)
		return -EINVAL;

	ret = adxcvr_drp_read(st, reg);
	if (ret < 0)
		return ret;

	tmp = ret;

	tmp &= ~mask;
	tmp |= ((val << offset) & mask);

	return adxcvr_drp_write(st, reg, tmp);
}

#define adxcvr_drp_writef(st, reg, mask, val) \
	__adxcvr_drp_writef(st, reg, mask, __ffs(mask), val)

static int adxcvr_set_lpm_dfe_mode(struct adxcvr_state *st,
								   unsigned int lpm)
{
	if (st->gth_enable) {
		if (lpm) {
			adxcvr_drp_write(st, 0x036, 0x0032);
			adxcvr_drp_write(st, 0x039, 0x1000);
			adxcvr_drp_write(st, 0x062, 0x1980);
		} else {
			adxcvr_drp_write(st, 0x036, 0x0002);
			adxcvr_drp_write(st, 0x039, 0x0000);
			adxcvr_drp_write(st, 0x062, 0x0000);
		}
	} else {
		if (lpm)
			adxcvr_drp_write(st, 0x029, 0x0104);
		else
			adxcvr_drp_write(st, 0x029, 0x0954);
	}

	return 0;
}

static int adxcvr_status_error(struct device *dev)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);
	int timeout = 100;
	unsigned int status;

	do {
		mdelay(1);
		status = adxcvr_read(st, ADXCVR_REG_STATUS);
	} while ((timeout--) && (status == 0));

	if (!status) {
		dev_err(dev, "%s Error: %x", st->tx_enable ? "TX" : "RX", status);
		return -EIO;
	}

	return 0;
}

static void adxcvr_work_func(struct work_struct *work)
{
	struct adxcvr_state *st =
		container_of(work, struct adxcvr_state, work);

	unsigned long div40_rate;
	int ret;

	if (!IS_ERR(st->lane_rate_div40_clk)) {

		div40_rate = st->lane_rate * (1000 / 40);

		dev_dbg(st->dev, "%s: setting MMCM on %s rate %lu\n",
			__func__, st->tx_enable ? "TX" : "RX", div40_rate);

		if (__clk_is_enabled(st->lane_rate_div40_clk))
			clk_disable_unprepare(st->lane_rate_div40_clk);

		ret = clk_set_rate(st->lane_rate_div40_clk, div40_rate);
		if (ret < 0)
			dev_err(st->dev, "%s: setting MMCM on %s rate %lu failed (%d)\n",
				__func__, st->tx_enable ? "TX" : "RX", div40_rate, ret);

		ret = clk_prepare_enable(st->lane_rate_div40_clk);
		if (ret < 0)
			dev_err(st->dev, "%s: enabling MMCM rate %lu failed (%d)\n",
				__func__, div40_rate, ret);
	}
}

static int adxcvr_clk_enable(struct clk_hw *hw)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);
	int ret;

	dev_dbg(st->dev, "%s: %s", __func__, st->tx_enable ? "TX" : "RX");

	adxcvr_write(st, ADXCVR_REG_RESETN, 0);

	if (!st->tx_enable)
		adxcvr_set_lpm_dfe_mode(st, st->lpm_enable);

	adxcvr_write(st, ADXCVR_REG_RESETN, ADXCVR_RESETN);

	mdelay(100);

	ret = adxcvr_status_error(st->dev);

	return ret;
}

static void adxcvr_clk_disable(struct clk_hw *hw)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);

	st->out_clk_enabled = false;
}

static int adxcvr_clk_is_enabled(struct clk_hw *hw)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);

	return st->out_clk_enabled;
}

static long adxcvr_gth_rxcdr_settings(struct adxcvr_state *st,
			u32 rxout_div)
{
	u16 cfg0, cfg1, cfg2, cfg3, cfg4;

	if (st->tx_enable)
		return 0; /* Do Nothing */

	switch (st->ppm) {
	case PM_200:
		cfg0 = 0x0018;
		break;
	case PM_700:
	case PM_1250:
		cfg0 = 0x8018;
		break;
	default:
		return -EINVAL;
	}

	if (st->encoding == ENC_8B10B) {

		cfg1 = 0xC208;
		cfg3 = 0x07FE;
		cfg3 = 0x0020;

		switch (rxout_div) {
		case 0: /* 1 */
			cfg2 = 0x2000;
			break;
		case 1: /* 2 */
			cfg2 = 0x1000;
			break;
		case 2: /* 4 */
			cfg2 = 0x0800;
			break;
		case 3: /* 8 */
			cfg2 = 0x0400;
			break;
		default:
			return -EINVAL;
		}

	} else {
		dev_warn(st->dev, "%s: GTH PRBS CDR not implemented\n", __func__);
	}

	adxcvr_drp_write(st, RXCDR_CFG0_ADDR, cfg0);
	adxcvr_drp_write(st, RXCDR_CFG1_ADDR, cfg1);
	adxcvr_drp_write(st, RXCDR_CFG2_ADDR, cfg2);
	adxcvr_drp_write(st, RXCDR_CFG3_ADDR, cfg3);
	adxcvr_drp_write(st, RXCDR_CFG4_ADDR, cfg4);

	return 0;
}

static long adxcvr_rxcdr_settings(struct adxcvr_state *st,
			u32 rxout_div)
{
	u16 cfg0, cfg1, cfg2, cfg3, cfg4;

	if (st->tx_enable)
		return 0; /* Do Nothing */

	if (st->gth_enable)
		return adxcvr_gth_rxcdr_settings(st, rxout_div);

	cfg2 = 0x23FF;
	cfg0 = 0x0020;

	switch (st->ppm) {
	case PM_200:
		cfg3 = 0x0000;
		break;
	case PM_700:
	case PM_1250:
		cfg3 = 0x8000;
		break;
	default:
		return -EINVAL;
	}

	if (st->lane_rate > 6600000 && rxout_div == 1)
		cfg4 = 0x0B;
	else
		cfg4 = 0x03;

	if (st->encoding == ENC_8B10B) {

		switch (rxout_div) {
		case 0: /* 1 */
			cfg1 = 0x1040;
			break;
		case 1: /* 2 */
			cfg1 = 0x1020;
			break;
		case 2: /* 4 */
			cfg1 = 0x1010;
			break;
		case 3: /* 8 */
			cfg1 = 0x1008;
			break;
		default:
			return -EINVAL;
		}

	} else {

		switch (rxout_div) {
		case 0: /* 1 */
			if (st->lpm_enable) {
				if (st->lane_rate  > 6600000) {
					if (st->ppm == PM_1250)
						cfg1 = 0x1020;
					else
						cfg1 = 0x1040;
				} else {
					cfg1 = 0x1020;
				}
			} else { /* DFE */
				if (st->lane_rate  > 6600000) {
					if (st->ppm == PM_1250)
						cfg1 = 0x1020;
					else
						cfg1 = 0x1040;
				} else {
					if (st->ppm == PM_1250)
						cfg1 = 0x1020;
					else
						cfg1 = 0x2040;
				}
			}
			break;
		case 1: /* 2 */
			cfg1 = 0x4020;
			break;
		case 2: /* 4 */
			cfg1 = 0x4010;
			break;
		case 3: /* 8 */
			cfg1 = 0x4008;
			break;
		default:
			return -EINVAL;
		}
	}


	adxcvr_drp_write(st, RXCDR_CFG0_ADDR, cfg0);
	adxcvr_drp_write(st, RXCDR_CFG1_ADDR, cfg1);
	adxcvr_drp_write(st, RXCDR_CFG2_ADDR, cfg2);
	adxcvr_drp_write(st, RXCDR_CFG3_ADDR, cfg3);
	adxcvr_drp_writef(st, RXCDR_CFG4_ADDR, RXCDR_CFG4_MASK, cfg4);

	return 0;
}

static long adxcvr_calc_cpll_settings(struct adxcvr_state *st,
					   unsigned long refclk_kHz,
					   unsigned long laneRate_kHz,
					   u32 *refclk_div, u32 *out_div,
					   u32 *fbdiv_45, u32 *fbdiv)
{
	u32 n1, n2, d, m;
	u32 pllFreq_kHz;

	/* Possible Xilinx GTX PLL parameters for Virtex 7 CPLL.  Find one that works for the desired laneRate. */
	/* Attribute encoding, DRP encoding */
	const u8 _N1[][2] = {{5, 1}, {4, 0} };
	const u8 _N2[][2] = {{5, 3}, {4, 2}, {3, 1}, {2, 0}, {1, 16} };
	const u8 _D[][2] = {{1, 0}, {2, 1}, {4, 2}, {8, 3} };
	const u8 _M[][2] = {{1, 16}, {2, 0} };

	for (m = 0; m < ARRAY_SIZE(_M); m++) {
		for (d = 0; d < ARRAY_SIZE(_D); d++) {
			for (n1 = 0; n1 < ARRAY_SIZE(_N1); n1++) {
				for (n2 = 0; n2 < ARRAY_SIZE(_N2); n2++) {
					pllFreq_kHz = refclk_kHz * _N1[n1][0] * _N2[n2][0] / _M[m][0];

					if ((pllFreq_kHz > 3300000) || (pllFreq_kHz < 1600000)) /* GTH 3.75 GHz */
						continue;

					if ((pllFreq_kHz * 2 / _D[d][0]) == laneRate_kHz) {
						if (refclk_div && out_div && fbdiv_45 && fbdiv) {
							*refclk_div = _M[m][1];
							*out_div = _D[d][1];
							*fbdiv_45 = _N1[n1][1];
							*fbdiv = _N2[n2][1];
						}

						dev_dbg(st->dev, "%s: M %d, D %d, N1 %d, N2 %d\n",
							__func__, _M[m][0], _D[d][0],
							_N1[n1][0], _N2[n2][0]);

						return laneRate_kHz;
					}
				}
			}
		}
	}

	dev_dbg(st->dev, "%s: Failed to find matching dividers for %lu kHz rate\n",
		__func__, laneRate_kHz);

	return -EINVAL;
}

static long adxcvr_calc_qpll_settings(struct adxcvr_state *st,
					   unsigned long refclk_kHz,
					   unsigned long laneRate_kHz,
					   u32 *refclk_div, u32 *out_div,
					   u32 *fbdiv, u32 *fbdiv_ratio,
					   u32 *lowband)
{
	/* Calculate the FPGA GTX PLL settings M, D, N1, N2 */
	u32 n, d, m;
	u32 pllVcoFreq_kHz;
	u32 pllOutFreq_kHz;

	/* Possible Xilinx GTX QPLL parameters for Virtex 7 QPLL.  Find one that works for the desired laneRate. */
	/* Attribute encoding, DRP encoding */
	const u16 _N[][2] = {{16, 32}, {20, 48}, {32, 96}, {40, 128},
			     {64, 224}, {66, 320}, {80, 288}, {100, 368} };
	const u8 _D[][2] = {{1, 0}, {2, 1}, {4, 2}, {8, 3}, {16, 4} };
	const u8 _M[][2] = {{1, 16}, {2, 0}, {3, 1}, {4, 2} };
	u8 _lowBand = 0;

	for (m = 0; m < ARRAY_SIZE(_M); m++) {
		for (d = 0; d < ARRAY_SIZE(_D); d++) {
			for (n = 0; n < ARRAY_SIZE(_N); n++) {

				pllVcoFreq_kHz = refclk_kHz * _N[n][0] / _M[m][0];
				pllOutFreq_kHz = pllVcoFreq_kHz / 2;

				if ((pllVcoFreq_kHz >= 5930000) && (pllVcoFreq_kHz <= 8000000)) {
					/* low band = 5.93G to 8.0GHz VCO */
					_lowBand = 1;
				} else if ((pllVcoFreq_kHz >= 9800000) && (pllVcoFreq_kHz <= 12500000)) {
					/* high band = 9.8G to 12.5GHz VCO */
					_lowBand = 0;
				} else {
					continue; /* if Pll out of range, not valid case, keep trying */
				}

				if ((pllOutFreq_kHz * 2 / _D[d][0]) == laneRate_kHz) {
					if (refclk_div && out_div && fbdiv_ratio && fbdiv && lowband) {
						*refclk_div = _M[m][1];
						*out_div = _D[d][1];
						*fbdiv = _N[n][1];
						*fbdiv_ratio = (_N[n][0] == 66) ? 0 : 1;
					}
					if (lowband)
						*lowband = _lowBand;

					dev_dbg(st->dev, "%s: M %d, D %d, N %d, ratio %d, lowband %d\n",
						__func__, _M[m][0], _D[d][0],
						_N[n][0], (_N[n][0] == 66) ? 0 : 1,
						_lowBand);

					return laneRate_kHz;
				}

			}
		}
	}

	dev_dbg(st->dev, "%s: Failed to find matching dividers for %lu kHz rate\n",
		__func__, laneRate_kHz);

	return -EINVAL;
}

static unsigned long adxcvr_clk_recalc_rate(struct clk_hw *hw,
											unsigned long parent_rate)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);

	dev_dbg(st->dev, "%s: Parent Rate %lu Hz",
		__func__, parent_rate);

	if (st->cpll_enable) {
		unsigned int refclk_div_m, out_div, N1, N2, M;

		refclk_div_m = adxcvr_drp_read(st, CPLL_REFCLK_DIV_M_ADDR);
		out_div = adxcvr_drp_read(st, RXOUT_DIV_ADDR);

		switch ((refclk_div_m & CPLL_FB_DIV_45_N1_MASK) >> CPLL_FB_DIV_45_N1_OFFSET) {
		case 0:
			N1 = 4;
			break;
		case 1:
			N1 = 5;
			break;
		}

		switch ((refclk_div_m & CPLL_REFCLK_DIV_M_MASK) >> CPLL_REFCLK_DIV_M_OFFSET) {
		case 0:
			M = 2;
			break;
		case 16:
			M = 1;
			break;
		}

		switch (refclk_div_m & CPLL_FBDIV_N2_MASK) {
		case 3:
			N2 = 5;
			break;
		case 2:
			N2 = 4;
			break;
		case 1:
			N2 = 3;
			break;
		case 0:
			N2 = 2;
			break;
		case 16:
			N2 = 1;
			break;
		}

		if (st->tx_enable)
			out_div = (out_div & TXOUT_DIV_MASK) >> TXOUT_DIV_OFFSET;
		else
			out_div = (out_div & RXOUT_DIV_MASK) >> RXOUT_DIV_OFFSET;

		out_div = (1 << out_div);

		dev_dbg(st->dev, "%s  CPLL %lu   %lu\n", __func__, st->lane_rate,
			((parent_rate / 1000) * N1 * N2 * 2) / (M * out_div));

		dev_dbg(st->dev, "%s  CPLL N1=%d N2=%d M=%d out_div=%d\n", __func__, N1, N2, M, out_div);

		return ((parent_rate / 1000) * N1 * N2 * 2) / (M * out_div);
	} else {
		unsigned int refclk_div_m, fb_div, out_div, N, M;
		unsigned long rate;
		u32 set_lowband;
		u32 lowband;

		refclk_div_m = adxcvr_drp_read(st, QPLL_REFCLK_DIV_M_ADDR);
		fb_div = adxcvr_drp_read(st, QPLL_FBDIV_N_ADDR);
		out_div = adxcvr_drp_read(st, RXOUT_DIV_ADDR);
		set_lowband = adxcvr_drp_read(st, QPLL_CFG0_ADDR) & QPLL_CFG0_BAND_MASK;

		switch ((refclk_div_m & QPLL_REFCLK_DIV_M_MASK) >> QPLL_REFCLK_DIV_M_OFFSET) {
		case 16:
			M = 1;
			break;
		case 0:
			M = 2;
			break;
		case 1:
			M = 3;
			break;
		case 2:
			M = 4;
			break;
		}

		switch (fb_div & QPLL_FBDIV_N_MASK) {
		case 32:
			N = 16;
			break;
		case 48:
			N = 20;
			break;
		case 96:
			N = 32;
			break;
		case 128:
			N = 40;
			break;
		case 224:
			N = 64;
			break;
		case 320:
			N = 66;
			break;
		case 288:
			N = 80;
			break;
		case 368:
			N = 100;
			break;
		}

		if (st->tx_enable)
			out_div = (out_div & TXOUT_DIV_MASK) >> TXOUT_DIV_OFFSET;
		else
			out_div = (out_div & RXOUT_DIV_MASK) >> RXOUT_DIV_OFFSET;

		out_div = (1 << out_div);

		dev_dbg(st->dev, "%s QPLL  %lu %lu\n", __func__, st->lane_rate,
			((parent_rate / 1000) * N) / (M * out_div));

		dev_dbg(st->dev, "%s QPLL N=%d M=%d out_div=%d\n", __func__, N, M, out_div);

		rate = ((parent_rate / 1000) * N) / (M * out_div);

		adxcvr_calc_qpll_settings(st, parent_rate / 1000, rate,
						     NULL, NULL, NULL, NULL,
						     &lowband);
		lowband = lowband ? QPLL_CFG0_BAND_MASK : 0;

		if (lowband !=  set_lowband)
			adxcvr_drp_writef(st, QPLL_CFG0_ADDR,
							  QPLL_CFG0_BAND_MASK, lowband ? 1 : 0);

		return rate;

	}
}


static long adxcvr_clk_round_rate(struct clk_hw *hw, unsigned long rate,
								  unsigned long *prate)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);
	int ret;

	dev_dbg(st->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, *prate);

	if (st->cpll_enable)
		ret = adxcvr_calc_cpll_settings(st, *prate / 1000, rate,
						      NULL, NULL, NULL, NULL);
	else
		ret = adxcvr_calc_qpll_settings(st, *prate / 1000, rate,
						     NULL, NULL, NULL, NULL,
						     NULL);

	return ret;
}

static int adxcvr_clk_set_rate(struct clk_hw *hw, unsigned long rate,
							   unsigned long parent_rate)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);
	u32 refclk_div, out_div, fbdiv_45, fbdiv, fbdiv_ratio, lowband;
	int ret;

	dev_dbg(st->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, parent_rate);

	if (st->cpll_enable)
		ret = adxcvr_calc_cpll_settings(st, parent_rate / 1000, rate,
						     &refclk_div, &out_div,
						     &fbdiv_45, &fbdiv);
	else
		ret = adxcvr_calc_qpll_settings(st, parent_rate / 1000, rate,
						     &refclk_div, &out_div,
						     &fbdiv, &fbdiv_ratio,
						     &lowband);
	if (ret < 0)
		return ret;

	st->lane_rate = rate;

	adxcvr_write(st, ADXCVR_REG_RESETN, 0);

	if (st->cpll_enable) {
		adxcvr_drp_writef(st, CPLL_REFCLK_DIV_M_ADDR,
						  CPLL_REFCLK_DIV_M_MASK |
						  CPLL_FB_DIV_45_N1_MASK |
						  CPLL_FBDIV_N2_MASK,
						  (refclk_div << 8) | (fbdiv_45 << 7) | fbdiv);
	} else {
		adxcvr_drp_writef(st, QPLL_CFG0_ADDR,
						  QPLL_CFG0_BAND_MASK, lowband);

		adxcvr_drp_writef(st, QPLL_REFCLK_DIV_M_ADDR,
						  QPLL_REFCLK_DIV_M_MASK, refclk_div);

		adxcvr_drp_writef(st, QPLL_FBDIV_N_ADDR,
						  QPLL_FBDIV_N_MASK, fbdiv);

		adxcvr_drp_writef(st, QPLL_FBDIV_RATIO_ADDR,
						  QPLL_FBDIV_RATIO_MASK, fbdiv_ratio);
	}

	ret = adxcvr_drp_writef(st, RXOUT_DIV_ADDR,
							st->tx_enable ? TXOUT_DIV_MASK : RXOUT_DIV_MASK, out_div);

	adxcvr_rxcdr_settings(st, out_div);

	adxcvr_write(st, ADXCVR_REG_RESETN, ADXCVR_RESETN);

	if (!IS_ERR(st->lane_rate_div40_clk))
		schedule_work(&st->work);

	return ret;
}

static const struct clk_ops clkout_ops = {
	.recalc_rate = adxcvr_clk_recalc_rate,
	.enable = adxcvr_clk_enable,
	.disable = adxcvr_clk_disable,
	.is_enabled = adxcvr_clk_is_enabled,
	.round_rate = adxcvr_clk_round_rate,
	.set_rate = adxcvr_clk_set_rate,

};

static int adxcvr_clk_register(struct device *dev, struct device_node *node,
	const char *parent_name)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);
	struct clk_init_data init;
	struct clk *clk;
	const char *clk_name;
	int ret;

	ret = of_property_read_string_index(node, "clock-output-names",
		0, &clk_name);
	if (ret < 0)
		return ret;

	init.name = clk_name;
	init.ops = &clkout_ops;
	init.flags = 0;

	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	st->out_clk_hw.init = &init;

	/* register the clock */
	clk = devm_clk_register(dev, &st->out_clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

static int adxcvr_parse_dt(struct adxcvr_state *st,
						   struct device_node *np)
{
	st->gth_enable =
		of_property_read_bool(np, "adi,transceiver-gth-enable");
	st->tx_enable =
		of_property_read_bool(np, "adi,link-is-transmit-enable");

	of_property_read_u32(np, "adi,sys-clk-select",
				&st->sys_clk_sel);
	of_property_read_u32(np, "adi,out-clk-select",
				&st->out_clk_sel);

	st->cpll_enable = of_property_read_bool(np,
				"adi,use-cpll-enable");
	st->lpm_enable = of_property_read_bool(np,
				"adi,use-lpm-enable");

	st->encoding = ENC_8B10B;
	st->ppm = PM_200; /* TODO use clock accuracy */

	INIT_WORK(&st->work, adxcvr_work_func);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id adxcvr_of_match[] = {
	{ .compatible = "adi,axi-adxcvr-1.0", .data = (void *) 1},
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adxcvr_of_match);

static int adxcvr_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct adxcvr_state *st;
	struct resource *mem; /* IO mem resources */
	unsigned int version;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->conv_clk = devm_clk_get(&pdev->dev, "conv");
	if (IS_ERR(st->conv_clk))
		return PTR_ERR(st->conv_clk);

	st->lane_rate_div40_clk = devm_clk_get(&pdev->dev, "div40");
	if (IS_ERR(st->lane_rate_div40_clk)) {
		if (PTR_ERR(st->lane_rate_div40_clk) != -ENOENT)
			return PTR_ERR(st->lane_rate_div40_clk);
	}

	ret = clk_prepare_enable(st->conv_clk);
	if (ret < 0)
		return ret;

	ret = adxcvr_parse_dt(st, np);
	if (ret < 0)
		goto disable_unprepare;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs)) {
		ret = PTR_ERR(st->regs);
		goto disable_unprepare;
	}

	st->dev = &pdev->dev;
	version = adxcvr_read(st, ADXCVR_REG_VERSION);
	platform_set_drvdata(pdev, st);

	adxcvr_write(st, ADXCVR_REG_RESETN, 0);

	adxcvr_write(st, ADXCVR_REG_CONTROL,
				 ((st->lpm_enable ? ADXCVR_LPM_DFE_N : 0) |
				  ADXCVR_SYSCLK_SEL(st->sys_clk_sel) |
				  ADXCVR_OUTCLK_SEL(st->out_clk_sel)));

	ret = adxcvr_clk_register(&pdev->dev, np, __clk_get_name(st->conv_clk));
	if (ret)
		return ret;

	dev_info(&pdev->dev, "AXI-ADXCVR (%d.%.2d.%c) at 0x%08llX mapped to 0x%p,",
		PCORE_VER_MAJOR(version),
		PCORE_VER_MINOR(version),
		PCORE_VER_LETTER(version),
		(unsigned long long)mem->start, st->regs);

	return 0;

disable_unprepare:
	clk_disable_unprepare(st->conv_clk);

	return ret;
}

/**
 * adxcvr_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int adxcvr_remove(struct platform_device *pdev)
{
	struct adxcvr_state *st = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);
	clk_disable_unprepare(st->conv_clk);

	return 0;
}

static struct platform_driver adxcvr_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = adxcvr_of_match,
	},
	.probe		= adxcvr_probe,
	.remove		= adxcvr_remove,
};

module_platform_driver(adxcvr_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI-ADXCVR Module");
MODULE_LICENSE("GPL v2");
