// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADI Driver for Synopsys DesignWare Cores Mobile Storage Host Controller
 * It is based on sdhci-of-dwcmshc.c
 *
 * Copyright (c) 2023, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/mmc/mmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/sizes.h>

#include "sdhci-pltfm.h"

/* Vendor register offsets */
#define SDHCI_VENDOR1_MSHC_CTRL_R_OFF                  (0x508U)
#define SDHCI_VENDOR1_EMMC_CTRL_R_OFF                  (0x52CU)
#define SDHCI_VENDOR1_AT_CTRL_R_OFF                    (0x540U)

/* Vendor register field bit masks */
#define SDHCI_VENDOR1_NEGEDGE_DATAOUT_EN               BIT(1)
#define SDHCI_VENDOR1_ENH_STROBE_EN_BM                 BIT(8)
#define SDHCI_VENDOR1_CARD_IS_EMMC                     BIT(0)
#define SDHCI_VENDOR1_AT_EN                            BIT(0)
#define SDHCI_VENDOR1_SWIN_TH_EN                       BIT(2)
#define SDHCI_VENDOR1_POST_CHANGE_DLY                  GENMASK(20, 19)
#define SDHCI_VENDOR1_TUNE_CLK_STOP_EN                 BIT(16)

/* Vendor register field values */
#define POST_CHANGE_DLY_LESS_4_CYCLES                  (0x03U)
#define TUNE_CLK_STOP_EN                               (1U)

/* Vendor register Bit positions */
#define POST_CHANGE_DLY_OFF                            (19U)
#define TUNE_CLK_STOP_EN_OFF                           (16U)

#define SDHCI_DWCMSHC_ARG2_STUFF                       GENMASK(31, 16)

/* DWCMSHC specific Mode Select value */
#define DWCMSHC_CTRL_HS400              0x7

#define BOUNDARY_OK(addr, len) \
	((addr | (SZ_128M - 1)) == ((addr + len - 1) | (SZ_128M - 1)))

/* SDHCI PHY configure operations */
#define SDHCI_PHY_OPS_CFG_DLL_NO_CLK                   (1U)
#define SDHCI_PHY_OPS_ENABLE_DLL_AFTER_CLK             (2U)
#define SDHCI_PHY_OPS_SET_DELAY                        (3U)

/* To this day, ADI design integrates the 1.8V IP versions (Host Controller
 * and PHY for eMMC and Host Controller for SD). All eMMC speed modes are
 * supported and all modes can operate at 1.8V. On the other side, only low speed
 * modes are supported in SD card and those modes can only operate at 3.3V. That
 * is solved by adding an externel level shifter.
 * Driver does not manage well this SD scenario.
 * Note: This macro is just to identify the action taken on this issue
 */
#define SDHCI_ADI_IP_1_8V

/* PHY Delay Lines may cause a potential glitch on the RX clock (because PHY DL2
 * input (rx clock) is connected to PHY DL1 output (tx clock)). Delay lines
 * configuration comes from Synopsys, and.it is expected not to change for future
 * products.
 * Note: This macro is just to identify the action taken on this issue
 */
#define SDHCI_ADI_RX_CLOCK_GLITCH

/* MMC HS400 in Adrv906x requires data to be sent out on negedge of cclk_tx.
 * This is a soc-specific requirement (coming from Adrv906x GLS simulations).
 */
#define SDHCI_ADI_HS400_TX_CLK_NEGEDGE

#define SDHCI_IDLE_TIMEOUT                       (20) /* 20 ms */

/* There are up to 128 delay cells (each one around 50 ps) */
#define MAX_DELAY_LINE_TAPS                      (128)

struct dwcmshc_priv {
	struct clk *bus_clk;
	struct phy *phy;
	bool is_emmc;
	bool phy_init_done;
};

/*
 * If DMA addr spans 128MB boundary, we split the DMA transfer into two
 * so that each DMA transfer doesn't exceed the boundary.
 */
static void dwcmshc_adma_write_desc(struct sdhci_host *host, void **desc,
				    dma_addr_t addr, int len, unsigned int cmd)
{
	int tmplen, offset;

	if (likely(!len || BOUNDARY_OK(addr, len))) {
		sdhci_adma_write_desc(host, desc, addr, len, cmd);
		return;
	}

	offset = addr & (SZ_128M - 1);
	tmplen = SZ_128M - offset;
	sdhci_adma_write_desc(host, desc, addr, tmplen, cmd);

	addr += tmplen;
	len -= tmplen;
	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}

static void dwcmshc_check_auto_cmd23(struct mmc_host *mmc,
				     struct mmc_request *mrq)
{
	struct sdhci_host *host = mmc_priv(mmc);

	/*
	 * No matter V4 is enabled or not, ARGUMENT2 register is 32-bit
	 * block count register which doesn't support stuff bits of
	 * CMD23 argument on dwcmsch host controller.
	 */
	if (mrq->sbc && (mrq->sbc->arg & SDHCI_DWCMSHC_ARG2_STUFF))
		host->flags &= ~SDHCI_AUTO_CMD23;
	else
		host->flags |= SDHCI_AUTO_CMD23;
}

static void dwcmshc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	dwcmshc_check_auto_cmd23(mmc, mrq);

	sdhci_request(mmc, mrq);
}

static void adi_sdhci_set_clk_tx_negedge(struct sdhci_host *host, bool enable)
{
	u16 mshc_ctrl;

	mshc_ctrl = sdhci_readw(host, SDHCI_VENDOR1_MSHC_CTRL_R_OFF);
	if (enable)
		mshc_ctrl |= SDHCI_VENDOR1_NEGEDGE_DATAOUT_EN;
	else
		mshc_ctrl &= ~SDHCI_VENDOR1_NEGEDGE_DATAOUT_EN;
	sdhci_writew(host, mshc_ctrl, SDHCI_VENDOR1_MSHC_CTRL_R_OFF);
}

static void dwcmshc_set_uhs_signaling(struct sdhci_host *host,
				      unsigned int timing)
{
	u16 ctrl_2;

	/* Workaround: wrong SDHCI_CTRL_HS400 definition in general framework */
	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	/* Select Bus Speed Mode for host */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	if ((timing == MMC_TIMING_MMC_HS200) ||
	    (timing == MMC_TIMING_UHS_SDR104))
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
	else if (timing == MMC_TIMING_UHS_SDR12)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
	else if ((timing == MMC_TIMING_UHS_SDR25) ||
		 (timing == MMC_TIMING_MMC_HS))
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
	else if (timing == MMC_TIMING_UHS_SDR50)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
	else if ((timing == MMC_TIMING_UHS_DDR50) ||
		 (timing == MMC_TIMING_MMC_DDR52))
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
	else if (timing == MMC_TIMING_MMC_HS400)
		ctrl_2 |= DWCMSHC_CTRL_HS400;
	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);

#ifdef SDHCI_ADI_HS400_TX_CLK_NEGEDGE
	if (timing == MMC_TIMING_MMC_HS400)
		/* Required to meet timing (from GLS simulations) */
		adi_sdhci_set_clk_tx_negedge(host, true);
	else
#endif
	adi_sdhci_set_clk_tx_negedge(host, false);
}

#ifdef SDHCI_ADI_RX_CLOCK_GLITCH
static void adi_sdhci_fix_rx_clock_glitch(struct sdhci_host *host, u8 hs_timing)
{
	u32 reg;

	reg = sdhci_readl(host, SDHCI_VENDOR1_AT_CTRL_R_OFF);
	reg &= ~(SDHCI_VENDOR1_POST_CHANGE_DLY | SDHCI_VENDOR1_TUNE_CLK_STOP_EN);
	/* This configuration helps to fix this issue (verified in RTL and GLS simulations) */
	if ((hs_timing == MMC_TIMING_MMC_HS400) ||
	    (hs_timing == MMC_TIMING_MMC_HS200))
		reg |= (POST_CHANGE_DLY_LESS_4_CYCLES << POST_CHANGE_DLY_OFF);
	else
		reg |= (POST_CHANGE_DLY_LESS_4_CYCLES << POST_CHANGE_DLY_OFF) |
		       (TUNE_CLK_STOP_EN << TUNE_CLK_STOP_EN_OFF);
	sdhci_writel(host, reg, SDHCI_VENDOR1_AT_CTRL_R_OFF);
}
#endif

/*
 * PHY configuration is done through:
 *     int phy_configure(struct phy *phy, union phy_configure_opts *opts);
 * where:
 *     union phy_configure_opts {
 *         struct phy_configure_opts_mipi_dphy	mipi_dphy;
 *         struct phy_configure_opts_dp		dp;
 *     };
 *
 * None of the above structs meet MMC requirements, so a new one should be
 * added:
 *         struct phy_configure_opts_sdhci	sdhci;
 *
 * By now let's reuse 'dp' one for MMC purposes
 */
static int adi_sdhci_set_delay(struct sdhci_host *host, struct phy *phy, u8 hs_timing)
{
	union phy_configure_opts opts;

	if (!phy)
		return 0;

#ifdef SDHCI_ADI_RX_CLOCK_GLITCH
	/* eMMC PHY delay lines may cause a glitch on RX clock */
	adi_sdhci_fix_rx_clock_glitch(host, hs_timing);
#endif

	/* Reusing dp struct: link_rate as MMC operation event and lanes as speed mode */
	opts.dp.link_rate = SDHCI_PHY_OPS_SET_DELAY;
	opts.dp.lanes = hs_timing;

	return phy_configure(phy, &opts);
}

static int adi_sdhci_set_dll(struct phy *phy, u8 hs_timing, bool enable)
{
	union phy_configure_opts opts;

	if (!phy)
		return 0;

	/* Reusing dp struct: link_rate as MMC operation event */
	opts.dp.link_rate = enable ? SDHCI_PHY_OPS_ENABLE_DLL_AFTER_CLK :
			    SDHCI_PHY_OPS_CFG_DLL_NO_CLK;

	return phy_configure(phy, &opts);
}

static u16 adi_sdhci_calc_clk(struct sdhci_host *host, unsigned int clock,
			      unsigned int *actual_clock)
{
	int div = 0;
	int real_div = div;
	u16 clk = 0;

	/* Workaround: ADRV906X does not use SDHCI_DIV as specified
	 * in the SDHCI spec. Instead of 1/(2N), it is 1/(N+1).
	 */
	if (host->max_clk <= clock) {
		div = 0;
	} else {
		div = (host->max_clk / clock) - 1U;
		if ((host->max_clk % clock) != 0)
			div++;
	}
	real_div = div + 1;

	if (real_div)
		*actual_clock = host->max_clk / real_div;
	clk |= (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
		<< SDHCI_DIVIDER_HI_SHIFT;

	return clk;
}

static void adi_sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;
	int ret;
	u16 clk;
	u8 hs_timing;
	bool dll_en;

	host->mmc->actual_clock = 0;

	/* Stop clock */
	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	hs_timing = host->mmc->ios.timing;
	/*
	 * General sdhci framework calls 'set_clock' twice after configuring
	 * HS400. One before updating clock variable to 200 MHz (so still using
	 * 52 Mhz clock as in HS mode) and one after it.
	 * DLL fails to lock at clock freq lower than 100Mhz, so let's avoid DLL
	 * configuration before clock var is high enough.
	 */
	dll_en = (hs_timing == MMC_TIMING_MMC_HS400) &&
		 (clock > MMC_HIGH_52_MAX_DTR);

	/* Configure Delay Lines (PHY instance 1) */
	ret = adi_sdhci_set_delay(host, priv->phy, hs_timing);
	if (ret) {
		pr_err("Error setting delay lines %d\n", ret);
		return;
	}

	/* Disable and configure DLL (PHY instance 2) */
	ret = adi_sdhci_set_dll(priv->phy, hs_timing, false);
	if (ret) {
		pr_err("Error setting DLL %d\n", ret);
		return;
	}

	/* Configure and enable clock */
	clk = adi_sdhci_calc_clk(host, clock, &host->mmc->actual_clock);
	sdhci_enable_clk(host, clk);

	/* Enable DLL and wait for it to lock */
	if (dll_en) {
		ret = adi_sdhci_set_dll(priv->phy, hs_timing, true);
		if (ret) {
			pr_err("Error enabling DLL %d\n", ret);
			return;
		}
	}
}

static void adi_sdhci_reset(struct sdhci_host *host, u8 mask)
{
	int16_t reg;
	bool reset_all;

	reset_all = (mask & SDHCI_RESET_ALL) == SDHCI_RESET_ALL;

	if (reset_all)
		/* Save CARD_IS_EMMC bit state */
		reg = sdhci_readw(host, SDHCI_VENDOR1_EMMC_CTRL_R_OFF) & SDHCI_VENDOR1_CARD_IS_EMMC;

	sdhci_reset(host, mask);

	if (reset_all) {
		/* Restore CARD_IS_EMMC bit */
		reg = reg | (sdhci_readw(host, SDHCI_VENDOR1_EMMC_CTRL_R_OFF) & ~SDHCI_VENDOR1_CARD_IS_EMMC);
		sdhci_writew(host, reg, SDHCI_VENDOR1_EMMC_CTRL_R_OFF);
	}
}

/* Hook used for a totally different purpose (no reset).
 * mmc_rescan function is doing the card initialization (CMD0, CMD1, ...)
 * asynchronously to this ADI driver probe. We need to ensure that card
 * initialization does not happen before probe is complete (which includes PHY
 * init configuration as the last step)
 * Note: this workaround does only apply to eMMC, since SD path does not include
 * a PHY instance to configure).
 */
static void adi_sdhci_hw_reset(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;
	int max_cnt = 200;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	if (priv->is_emmc) {
		if (!priv) {
			pr_err("host priv is null");
			return;
		}

		while (!priv->phy_init_done) {
			if (--max_cnt <= 0) {
				pr_err("PHY init not completed on time");
				return;
			}

			mdelay(10);
		}
	}
}

/* This function is a pure copy-paste from sdhci.c file.
 * TODO: You can remove it when upgrading to a more recent kernel version
 *       (the function was made public on December 7th, 2023).
 */
static int __sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	int i;

	/*
	 * Issue opcode repeatedly till Execute Tuning is set to 0 or the number
	 * of loops reaches tuning loop count.
	 */
	for (i = 0; i < host->tuning_loop_count; i++) {
		u16 ctrl;

		sdhci_send_tuning(host, opcode);

		if (!host->tuning_done) {
			pr_debug("%s: Tuning timeout, falling back to fixed sampling clock\n",
				 mmc_hostname(host->mmc));
			sdhci_abort_tuning(host, opcode);
			return -ETIMEDOUT;
		}

		/* Spec does not require a delay between tuning cycles */
		if (host->tuning_delay > 0)
			mdelay(host->tuning_delay);

		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		if (!(ctrl & SDHCI_CTRL_EXEC_TUNING)) {
			if (ctrl & SDHCI_CTRL_TUNED_CLK)
				return 0; /* Success! */
			break;
		}
	}

	pr_info("%s: Tuning failed, falling back to fixed sampling clock\n",
		mmc_hostname(host->mmc));
	sdhci_reset_tuning(host);
	return -EAGAIN;
}

/* This hook is used to inject a workaround for an issue that has nothing to do
 * with tuning, but it is the only place to do it.
 * This code is a copy-paste from sdhci_execute_tuning (sdhci.c file), except
 * for the workaround.
 */
static int adi_sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	int err = 0;
	unsigned int tuning_count = 0;
	bool hs400_tuning;

	hs400_tuning = host->flags & SDHCI_HS400_TUNING;

	if (host->tuning_mode == SDHCI_TUNING_MODE_1)
		tuning_count = host->tuning_count;

	/*
	 * The Host Controller needs tuning in case of SDR104 and DDR50
	 * mode, and for SDR50 mode when Use Tuning for SDR50 is set in
	 * the Capabilities register.
	 * If the Host Controller supports the HS200 mode then the
	 * tuning function has to be executed.
	 */
	switch (host->timing) {
	/* HS400 tuning is done in HS200 mode */
	case MMC_TIMING_MMC_HS400:
		err = -EINVAL;
		goto out;

	case MMC_TIMING_MMC_HS200:
		/*
		 * Periodic re-tuning for HS400 is not expected to be needed, so
		 * disable it here.
		 */
		if (hs400_tuning)
			tuning_count = 0;
		break;

	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_UHS_DDR50:
		break;

	case MMC_TIMING_UHS_SDR50:
		if (host->flags & SDHCI_SDR50_NEEDS_TUNING)
			break;
		fallthrough;

	default:
		goto out;
	}
	host->mmc->retune_period = tuning_count;

	if (host->tuning_delay < 0)
		host->tuning_delay = opcode == MMC_SEND_TUNING_BLOCK;

	sdhci_start_tuning(host);

	host->tuning_err = __sdhci_execute_tuning(host, opcode);

	sdhci_end_tuning(host);

	/* When configuring HS400, the next step in the sequence after tuning
	 * (in HS200) is to downgrade to HS mode. This operation (CMD6) fails
	 * sporadically.
	 * Issue root cause:Unclear
	 * Fix options (empirically found):
	 * - repeat CMD6 (empirically it only fails the first attempt). This is
	 *   general Linux driver code (not ADI code)
	 * - reduce clock freq to 52MHz (this can be done in this hook)
	 */

	/* Reduce frequency to HS frequency */
	if (host->flags & SDHCI_HS400_TUNING) {
		struct mmc_ios *ios = &host->mmc->ios;
		host->mmc->ios.clock = 52000000;
		sdhci_set_ios(host->mmc, ios);
	}

	return 0;
out:
	return -1;
}

static const struct sdhci_ops sdhci_dwcmshc_ops = {
	.set_clock			= adi_sdhci_set_clock,
	.set_bus_width			= sdhci_set_bus_width,
	.set_uhs_signaling		= dwcmshc_set_uhs_signaling,
	.get_max_clock			= sdhci_pltfm_clk_get_max_clock,
	.reset				= adi_sdhci_reset,
	.adma_write_desc		= dwcmshc_adma_write_desc,
	.hw_reset			= adi_sdhci_hw_reset,
	.platform_execute_tuning	= adi_sdhci_execute_tuning,
};

static const struct sdhci_pltfm_data sdhci_dwcmshc_pdata = {
	.ops		= &sdhci_dwcmshc_ops,
	.quirks		= SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2	= SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static void adi_sdhci_hs400_enhanced_strobe(struct mmc_host *mmc,
					    struct mmc_ios *ios)
{
	struct sdhci_host *host = mmc_priv(mmc);
	u16 emmc_ctrl;

	emmc_ctrl = sdhci_readw(host, SDHCI_VENDOR1_EMMC_CTRL_R_OFF);
	if (ios->enhanced_strobe)
		emmc_ctrl |= SDHCI_VENDOR1_ENH_STROBE_EN_BM;
	else
		emmc_ctrl &= ~SDHCI_VENDOR1_ENH_STROBE_EN_BM;
	sdhci_writew(host, emmc_ctrl, SDHCI_VENDOR1_EMMC_CTRL_R_OFF);
}

static int adi_sdhci_deinit(struct sdhci_host *host)
{
	uint16_t u16_reg_data;
	unsigned int timeout = SDHCI_IDLE_TIMEOUT;

	/* Wait for the host controller to become idle before stopping card clock.*/
	while (sdhci_readl(host, SDHCI_PRESENT_STATE) &
	       (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) {
		if (--timeout == 0) {
			pr_err("Host controller is not idle\n");
			return -1;
		}
		udelay(1000);
	}

	/* Stop card clock, and turn off internal clock and PLL */
	u16_reg_data = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	u16_reg_data &= ~(SDHCI_CLOCK_CARD_EN | SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_PLL_EN);
	sdhci_writew(host, u16_reg_data, SDHCI_CLOCK_CONTROL);

	return 0;
}

static int dwcmshc_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct dwcmshc_priv *priv;
	int err;
	u32 extra;
	u32 u32_reg_data;
	u32 retune_period;
	bool is_emmc;

	host = sdhci_pltfm_init(pdev, &sdhci_dwcmshc_pdata,
				sizeof(struct dwcmshc_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	is_emmc = device_property_read_bool(&pdev->dev, "non-removable");
	priv->phy_init_done = 0;
	priv->is_emmc = is_emmc ? 1 : 0;

	/* Deinit to ensure a proper initialization */
	err = adi_sdhci_deinit(host);
	if (err)
		goto free_pltfm;

	if (is_emmc) {
		int16_t emmc_ctrl;

		/* Set CARD_IS_EMMC bit */
		emmc_ctrl = sdhci_readw(host, SDHCI_VENDOR1_EMMC_CTRL_R_OFF);
		if (!(emmc_ctrl & SDHCI_VENDOR1_CARD_IS_EMMC)) {
			emmc_ctrl |= SDHCI_VENDOR1_CARD_IS_EMMC;
			sdhci_writew(host, emmc_ctrl, SDHCI_VENDOR1_EMMC_CTRL_R_OFF);
		}

#ifdef SDHCI_ADI_RX_CLOCK_GLITCH
		adi_sdhci_fix_rx_clock_glitch(host, MMC_TIMING_LEGACY);
#endif
	}
#ifdef SDHCI_ADI_IP_1_8V
	/*
	 * Workaround for SD interface
	 */
	else {
		/* SD interface design is a bit special. IP side always works at
		 * 1.8V and the card side always works at 3.3V.
		 * A level shifter is added between the ASIC and the card.
		 *
		 * SD cards must work at 3.3V during initialization (ignoring
		 * newer LVS cards), and then depending on the speed mode used,
		 * it would require a switch to 1.8V. ADI design does not support
		 * that cases (only default and HS modes, both at 3.3V)
		 *
		 * Driver follows this flow, by configuring the initial signal
		 * voltage (HOST_CTRL2_R.SINGNALING_EN) to 3.3V ... and that
		 * behaviour can not be modified from device tree unless both:
		 * - a 1.8V vqmmc regulator is added (which does not apply to SD)
		 * - and, at least, one high speed mode is supported (which is
		 *   not the case).
		 * Let's indicate (force) that the only voltage supported (from
		 * Host Controller point of view) is 1.8V.
		 */
		host->flags &= ~SDHCI_SIGNALING_330;
		host->flags |= SDHCI_SIGNALING_180;
	}
#endif

	/*
	 * extra adma table cnt for cross 128M boundary handling.
	 */
	extra = DIV_ROUND_UP_ULL(dma_get_required_mask(&pdev->dev), SZ_128M);
	if (extra > SDHCI_MAX_SEGS)
		extra = SDHCI_MAX_SEGS;
	host->adma_table_cnt += extra;

	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		err = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		goto free_pltfm;
	}
	err = clk_prepare_enable(pltfm_host->clk);
	if (err)
		goto free_pltfm;

	priv->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(priv->bus_clk))
		clk_prepare_enable(priv->bus_clk);

	err = mmc_of_parse(host->mmc);
	if (err)
		goto err_clk;

	sdhci_get_of_property(pdev);

	host->mmc_host_ops.request = dwcmshc_request;
	host->mmc_host_ops.hs400_enhanced_strobe = adi_sdhci_hs400_enhanced_strobe;

	err = sdhci_add_host(host);
	if (err)
		goto err_clk;

	if (is_emmc) {
		/* Tuning procedure configuration: */

		/* Initial tuning: sweep through all the taps (128) */
		host->tuning_loop_count = MAX_DELAY_LINE_TAPS;
		u32_reg_data = sdhci_readl(host, SDHCI_VENDOR1_AT_CTRL_R_OFF);
		u32_reg_data &= ~SDHCI_VENDOR1_SWIN_TH_EN;

		/* Configure re-tuning mode */
		if (device_property_read_u32(&pdev->dev, "adi,retune-period", &retune_period) >= 0) {
			/* Re-tuning mode 1 (periodic) */
			host->tuning_mode = SDHCI_TUNING_MODE_1;
			host->tuning_count = retune_period;         /* Unit: seconds */

			u32_reg_data &= ~SDHCI_VENDOR1_AT_EN;
		} else {
			/* Re-tuning mode 3 (Auto-tuning) */
			host->tuning_mode = SDHCI_TUNING_MODE_3;
		}

		sdhci_writel(host, u32_reg_data, SDHCI_VENDOR1_AT_CTRL_R_OFF);
	}

	if (device_property_read_bool(&pdev->dev, "enable-phy-config")) {
		priv->phy = devm_phy_get(&pdev->dev, "phy_adi_sdhci");
		if (IS_ERR(priv->phy)) {
			err = dev_err_probe(&pdev->dev, PTR_ERR(priv->phy),
					    "No phy for sdhci-of-dwcmshc.\n");
			goto remove_host;
		}
		err = phy_init(priv->phy);
		if (err < 0) {
			dev_err(&pdev->dev, "phy_init err: %d\n", err);
			goto remove_host;
		}
		priv->phy_init_done = 1;
	} else {
		priv->phy = NULL;
	}

	return 0;

remove_host:
	sdhci_remove_host(host, 0);
err_clk:
	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);
free_pltfm:
	sdhci_pltfm_free(pdev);
	return err;
}

static int dwcmshc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dwcmshc_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;

	clk_disable_unprepare(pltfm_host->clk);
	if (!IS_ERR(priv->bus_clk))
		clk_disable_unprepare(priv->bus_clk);

	return ret;
}

static int dwcmshc_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret)
		return ret;

	if (!IS_ERR(priv->bus_clk)) {
		ret = clk_prepare_enable(priv->bus_clk);
		if (ret)
			return ret;
	}

	return sdhci_resume_host(host);
}
#endif

static SIMPLE_DEV_PM_OPS(dwcmshc_pmops, dwcmshc_suspend, dwcmshc_resume);

static const struct of_device_id sdhci_dwcmshc_dt_ids[] = {
	{ .compatible = "adi,dwcmshc-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_dwcmshc_dt_ids);

static struct platform_driver sdhci_dwcmshc_driver = {
	.driver			= {
		.name		= "sdhci-dwcmshc",
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = sdhci_dwcmshc_dt_ids,
		.pm		= &dwcmshc_pmops,
	},
	.probe			= dwcmshc_probe,
	.remove			= dwcmshc_remove,
};
module_platform_driver(sdhci_dwcmshc_driver);

MODULE_DESCRIPTION("ADI SDHCI platform driver for Synopsys DWC MSHC");
MODULE_LICENSE("GPL v2");
