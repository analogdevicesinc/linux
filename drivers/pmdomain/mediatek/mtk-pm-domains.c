// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Collabora Ltd.
 */
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_clk.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/mediatek/infracfg.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

#include "mt6735-pm-domains.h"
#include "mt6795-pm-domains.h"
#include "mt6893-pm-domains.h"
#include "mt8167-pm-domains.h"
#include "mt8173-pm-domains.h"
#include "mt8183-pm-domains.h"
#include "mt8186-pm-domains.h"
#include "mt8188-pm-domains.h"
#include "mt8192-pm-domains.h"
#include "mt8195-pm-domains.h"
#include "mt8196-pm-domains.h"
#include "mt8365-pm-domains.h"

#define MTK_POLL_DELAY_US		10
#define MTK_POLL_TIMEOUT		USEC_PER_SEC

#define MTK_HWV_POLL_DELAY_US		5
#define MTK_HWV_POLL_TIMEOUT		(300 * USEC_PER_MSEC)

#define MTK_HWV_PREPARE_DELAY_US	1
#define MTK_HWV_PREPARE_TIMEOUT		(3 * USEC_PER_MSEC)

#define PWR_RST_B_BIT			BIT(0)
#define PWR_ISO_BIT			BIT(1)
#define PWR_ON_BIT			BIT(2)
#define PWR_ON_2ND_BIT			BIT(3)
#define PWR_CLK_DIS_BIT			BIT(4)
#define PWR_SRAM_CLKISO_BIT		BIT(5)
#define PWR_SRAM_ISOINT_B_BIT		BIT(6)

#define PWR_RTFF_SAVE			BIT(24)
#define PWR_RTFF_NRESTORE		BIT(25)
#define PWR_RTFF_CLK_DIS		BIT(26)
#define PWR_RTFF_SAVE_FLAG		BIT(27)
#define PWR_RTFF_UFS_CLK_DIS		BIT(28)

#define MTK_SIP_KERNEL_HWCCF_CONTROL	MTK_SIP_SMC_CMD(0x540)

struct scpsys_domain {
	struct generic_pm_domain genpd;
	const struct scpsys_domain_data *data;
	const struct scpsys_hwv_domain_data *hwv_data;
	struct scpsys *scpsys;
	int num_clks;
	struct clk_bulk_data *clks;
	int num_subsys_clks;
	struct clk_bulk_data *subsys_clks;
	struct regulator *supply;
};

struct scpsys {
	struct device *dev;
	struct regmap *base;
	const struct scpsys_soc_data *soc_data;
	u8 bus_prot_index[BUS_PROT_BLOCK_COUNT];
	struct regmap **bus_prot;
	struct genpd_onecell_data pd_data;
	struct generic_pm_domain *domains[];
};

#define to_scpsys_domain(gpd) container_of(gpd, struct scpsys_domain, genpd)

static bool scpsys_domain_is_on(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	u32 mask = pd->data->sta_mask;
	u32 status, status2, mask2;

	mask2 = pd->data->sta2nd_mask ? pd->data->sta2nd_mask : mask;

	regmap_read(scpsys->base, pd->data->pwr_sta_offs, &status);
	status &= mask;

	regmap_read(scpsys->base, pd->data->pwr_sta2nd_offs, &status2);
	status2 &= mask2;

	/* A domain is on when both status bits are set. */
	return status && status2;
}

static bool scpsys_hwv_domain_is_disable_done(struct scpsys_domain *pd)
{
	const struct scpsys_hwv_domain_data *hwv = pd->hwv_data;
	u32 regs[2] = { hwv->done, hwv->clr_sta };
	u32 val[2];
	u32 mask = BIT(hwv->setclr_bit);

	regmap_multi_reg_read(pd->scpsys->base, regs, val, 2);

	/* Disable is done when the bit is set in DONE, cleared in CLR_STA */
	return (val[0] & mask) && !(val[1] & mask);
}

static bool scpsys_hwv_domain_is_enable_done(struct scpsys_domain *pd)
{
	const struct scpsys_hwv_domain_data *hwv = pd->hwv_data;
	u32 regs[3] = { hwv->done, hwv->en, hwv->set_sta };
	u32 val[3];
	u32 mask = BIT(hwv->setclr_bit);

	regmap_multi_reg_read(pd->scpsys->base, regs, val, 3);

	/* Enable is done when the bit is set in DONE and EN, cleared in SET_STA */
	return (val[0] & mask) && (val[1] & mask) && !(val[2] & mask);
}

static int scpsys_sec_infra_power_on(bool on)
{
	struct arm_smccc_res res;
	unsigned long cmd = on ? 1 : 0;

	arm_smccc_smc(MTK_SIP_KERNEL_HWCCF_CONTROL, cmd, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static int scpsys_sram_enable(struct scpsys_domain *pd)
{
	u32 expected_ack, pdn_ack = pd->data->sram_pdn_ack_bits;
	struct scpsys *scpsys = pd->scpsys;
	unsigned int tmp;
	int ret;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_SRAM_PDN_INVERTED)) {
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, pd->data->sram_pdn_bits);
		expected_ack = pdn_ack;
	} else {
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, pd->data->sram_pdn_bits);
		expected_ack = 0;
	}

	/* Either wait until SRAM_PDN_ACK all 1 or 0 */
	ret = regmap_read_poll_timeout(scpsys->base, pd->data->ctl_offs, tmp,
				       (tmp & pdn_ack) == expected_ack,
				       MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
	if (ret < 0)
		return ret;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_SRAM_ISO)) {
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_SRAM_ISOINT_B_BIT);
		udelay(1);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_SRAM_CLKISO_BIT);
	}

	return 0;
}

static int scpsys_sram_disable(struct scpsys_domain *pd)
{
	u32 expected_ack, pdn_ack = pd->data->sram_pdn_ack_bits;
	struct scpsys *scpsys = pd->scpsys;
	unsigned int tmp;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_SRAM_ISO)) {
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_SRAM_CLKISO_BIT);
		udelay(1);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_SRAM_ISOINT_B_BIT);
	}

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_SRAM_PDN_INVERTED)) {
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, pd->data->sram_pdn_bits);
		expected_ack = 0;
	} else {
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, pd->data->sram_pdn_bits);
		expected_ack = pdn_ack;
	}

	/* Either wait until SRAM_PDN_ACK all 1 or 0 */
	return regmap_read_poll_timeout(scpsys->base, pd->data->ctl_offs, tmp,
					(tmp & pdn_ack) == expected_ack,
					MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
}

static struct regmap *scpsys_bus_protect_get_regmap(struct scpsys_domain *pd,
						    const struct scpsys_bus_prot_data *bpd)
{
	struct scpsys *scpsys = pd->scpsys;
	unsigned short block_idx = scpsys->bus_prot_index[bpd->bus_prot_block];

	return scpsys->bus_prot[block_idx];
}

static struct regmap *scpsys_bus_protect_get_sta_regmap(struct scpsys_domain *pd,
							const struct scpsys_bus_prot_data *bpd)
{
	struct scpsys *scpsys = pd->scpsys;
	int block_idx = scpsys->bus_prot_index[bpd->bus_prot_sta_block];

	return scpsys->bus_prot[block_idx];
}

static int scpsys_bus_protect_clear(struct scpsys_domain *pd,
				    const struct scpsys_bus_prot_data *bpd)
{
	struct regmap *sta_regmap = scpsys_bus_protect_get_sta_regmap(pd, bpd);
	struct regmap *regmap = scpsys_bus_protect_get_regmap(pd, bpd);
	u32 sta_mask = bpd->bus_prot_sta_mask;
	u32 expected_ack;
	u32 val;

	expected_ack = (bpd->bus_prot_sta_block == BUS_PROT_BLOCK_INFRA_NAO ? sta_mask : 0);

	if (bpd->flags & BUS_PROT_REG_UPDATE)
		regmap_clear_bits(regmap, bpd->bus_prot_clr, bpd->bus_prot_set_clr_mask);
	else
		regmap_write(regmap, bpd->bus_prot_clr, bpd->bus_prot_set_clr_mask);

	if (bpd->flags & BUS_PROT_IGNORE_CLR_ACK)
		return 0;

	return regmap_read_poll_timeout(sta_regmap, bpd->bus_prot_sta,
					val, (val & sta_mask) == expected_ack,
					MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
}

static int scpsys_bus_protect_set(struct scpsys_domain *pd,
				  const struct scpsys_bus_prot_data *bpd)
{
	struct regmap *sta_regmap = scpsys_bus_protect_get_sta_regmap(pd, bpd);
	struct regmap *regmap = scpsys_bus_protect_get_regmap(pd, bpd);
	u32 sta_mask = bpd->bus_prot_sta_mask;
	u32 val;

	if (bpd->flags & BUS_PROT_REG_UPDATE)
		regmap_set_bits(regmap, bpd->bus_prot_set, bpd->bus_prot_set_clr_mask);
	else
		regmap_write(regmap, bpd->bus_prot_set, bpd->bus_prot_set_clr_mask);

	return regmap_read_poll_timeout(sta_regmap, bpd->bus_prot_sta,
					val, (val & sta_mask) == sta_mask,
					MTK_POLL_DELAY_US, MTK_POLL_TIMEOUT);
}

static int scpsys_bus_protect_enable(struct scpsys_domain *pd)
{
	for (int i = 0; i < SPM_MAX_BUS_PROT_DATA; i++) {
		const struct scpsys_bus_prot_data *bpd = &pd->data->bp_cfg[i];
		int ret;

		if (!bpd->bus_prot_set_clr_mask)
			break;

		if (bpd->flags & BUS_PROT_INVERTED)
			ret = scpsys_bus_protect_clear(pd, bpd);
		else
			ret = scpsys_bus_protect_set(pd, bpd);
		if (ret)
			return ret;
	}

	return 0;
}

static int scpsys_bus_protect_disable(struct scpsys_domain *pd)
{
	for (int i = SPM_MAX_BUS_PROT_DATA - 1; i >= 0; i--) {
		const struct scpsys_bus_prot_data *bpd = &pd->data->bp_cfg[i];
		int ret;

		if (!bpd->bus_prot_set_clr_mask)
			continue;

		if (bpd->flags & BUS_PROT_INVERTED)
			ret = scpsys_bus_protect_set(pd, bpd);
		else
			ret = scpsys_bus_protect_clear(pd, bpd);
		if (ret)
			return ret;
	}

	return 0;
}

static int scpsys_regulator_enable(struct regulator *supply)
{
	return supply ? regulator_enable(supply) : 0;
}

static int scpsys_regulator_disable(struct regulator *supply)
{
	return supply ? regulator_disable(supply) : 0;
}

static int scpsys_hwv_power_on(struct generic_pm_domain *genpd)
{
	struct scpsys_domain *pd = container_of(genpd, struct scpsys_domain, genpd);
	const struct scpsys_hwv_domain_data *hwv = pd->hwv_data;
	struct scpsys *scpsys = pd->scpsys;
	u32 val;
	int ret;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL)) {
		ret = scpsys_sec_infra_power_on(true);
		if (ret)
			return ret;
	}

	ret = scpsys_regulator_enable(pd->supply);
	if (ret)
		goto err_infra;

	ret = clk_bulk_prepare_enable(pd->num_clks, pd->clks);
	if (ret)
		goto err_reg;

	/* For HWV the subsys clocks refer to the HWV low power subsystem */
	ret = clk_bulk_prepare_enable(pd->num_subsys_clks, pd->subsys_clks);
	if (ret)
		goto err_disable_clks;

	/* Make sure the HW Voter is idle and able to accept commands */
	ret = regmap_read_poll_timeout_atomic(scpsys->base, hwv->done, val,
					      val & BIT(hwv->setclr_bit),
					      MTK_HWV_POLL_DELAY_US,
					      MTK_HWV_POLL_TIMEOUT);
	if (ret) {
		dev_err(scpsys->dev, "Failed to power on: HW Voter busy.\n");
		goto err_disable_subsys_clks;
	}

	/*
	 * Instruct the HWV to power on the MTCMOS (power domain): after that,
	 * the same bit will be unset immediately by the hardware.
	 */
	regmap_write(scpsys->base, hwv->set, BIT(hwv->setclr_bit));

	/*
	 * Wait until the HWV sets the bit again, signalling that its internal
	 * state machine was started and it now processing the vote command.
	 */
	ret = regmap_read_poll_timeout_atomic(scpsys->base, hwv->set, val,
					      val & BIT(hwv->setclr_bit),
					      MTK_HWV_PREPARE_DELAY_US,
					      MTK_HWV_PREPARE_TIMEOUT);
	if (ret) {
		dev_err(scpsys->dev, "Failed to power on: HW Voter not starting.\n");
		goto err_disable_subsys_clks;
	}

	/* Wait for ACK, signalling that the MTCMOS was enabled */
	ret = readx_poll_timeout_atomic(scpsys_hwv_domain_is_enable_done, pd, val, val,
					MTK_HWV_POLL_DELAY_US, MTK_HWV_POLL_TIMEOUT);
	if (ret) {
		dev_err(scpsys->dev, "Failed to power on: HW Voter ACK timeout.\n");
		goto err_disable_subsys_clks;
	}

	/* It's done! Disable the HWV low power subsystem clocks */
	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL))
		scpsys_sec_infra_power_on(false);

	return 0;

err_disable_subsys_clks:
	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);
err_disable_clks:
	clk_bulk_disable_unprepare(pd->num_clks, pd->clks);
err_reg:
	scpsys_regulator_disable(pd->supply);
err_infra:
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL))
		scpsys_sec_infra_power_on(false);
	return ret;
};

static int scpsys_hwv_power_off(struct generic_pm_domain *genpd)
{
	struct scpsys_domain *pd = container_of(genpd, struct scpsys_domain, genpd);
	const struct scpsys_hwv_domain_data *hwv = pd->hwv_data;
	struct scpsys *scpsys = pd->scpsys;
	u32 val;
	int ret;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL)) {
		ret = scpsys_sec_infra_power_on(true);
		if (ret)
			return ret;
	}

	ret = clk_bulk_prepare_enable(pd->num_subsys_clks, pd->subsys_clks);
	if (ret)
		goto err_infra;

	/* Make sure the HW Voter is idle and able to accept commands */
	ret = regmap_read_poll_timeout_atomic(scpsys->base, hwv->done, val,
					      val & BIT(hwv->setclr_bit),
					      MTK_HWV_POLL_DELAY_US,
					      MTK_HWV_POLL_TIMEOUT);
	if (ret)
		goto err_disable_subsys_clks;


	/*
	 * Instruct the HWV to power off the MTCMOS (power domain): differently
	 * from poweron, the bit will be kept set.
	 */
	regmap_write(scpsys->base, hwv->clr, BIT(hwv->setclr_bit));

	/*
	 * Wait until the HWV clears the bit, signalling that its internal
	 * state machine was started and it now processing the clear command.
	 */
	ret = regmap_read_poll_timeout_atomic(scpsys->base, hwv->clr, val,
					      !(val & BIT(hwv->setclr_bit)),
					      MTK_HWV_PREPARE_DELAY_US,
					      MTK_HWV_PREPARE_TIMEOUT);
	if (ret)
		goto err_disable_subsys_clks;

	/* Poweroff needs 100us for the HW to stabilize */
	udelay(100);

	/* Wait for ACK, signalling that the MTCMOS was disabled */
	ret = readx_poll_timeout_atomic(scpsys_hwv_domain_is_disable_done, pd, val, val,
					MTK_HWV_POLL_DELAY_US, MTK_HWV_POLL_TIMEOUT);
	if (ret)
		goto err_disable_subsys_clks;

	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);
	clk_bulk_disable_unprepare(pd->num_clks, pd->clks);

	scpsys_regulator_disable(pd->supply);

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL))
		scpsys_sec_infra_power_on(false);

	return 0;

err_disable_subsys_clks:
	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);
err_infra:
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_INFRA_PWR_CTL))
		scpsys_sec_infra_power_on(false);
	return ret;
};

static int scpsys_ctl_pwrseq_on(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	bool do_rtff_nrestore, tmp;
	int ret;

	/* subsys power on */
	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_BIT);
	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_2ND_BIT);

	/* wait until PWR_ACK = 1 */
	ret = readx_poll_timeout(scpsys_domain_is_on, pd, tmp, tmp, MTK_POLL_DELAY_US,
				 MTK_POLL_TIMEOUT);
	if (ret < 0)
		return ret;

	if (pd->data->rtff_type == SCPSYS_RTFF_TYPE_PCIE_PHY)
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);

	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_CLK_DIS_BIT);
	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_ISO_BIT);

	/* Wait for RTFF HW to sync buck isolation state if this is PCIe PHY RTFF */
	if (pd->data->rtff_type == SCPSYS_RTFF_TYPE_PCIE_PHY)
		udelay(5);

	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RST_B_BIT);

	/*
	 * RTFF HW state may be modified by secure world or remote processors.
	 *
	 * With the only exception of STOR_UFS, which always needs save/restore,
	 * check if this power domain's RTFF is already on before trying to do
	 * the NRESTORE procedure, otherwise the system will lock up.
	 */
	switch (pd->data->rtff_type) {
	case SCPSYS_RTFF_TYPE_GENERIC:
	case SCPSYS_RTFF_TYPE_PCIE_PHY:
	{
		u32 ctl_status;

		regmap_read(scpsys->base, pd->data->ctl_offs, &ctl_status);
		do_rtff_nrestore = ctl_status & PWR_RTFF_SAVE_FLAG;
		break;
	}
	case SCPSYS_RTFF_TYPE_STOR_UFS:
		/* STOR_UFS always needs NRESTORE */
		do_rtff_nrestore = true;
		break;
	default:
		do_rtff_nrestore = false;
		break;
	}

	/* Return early if RTFF NRESTORE shall not be done */
	if (!do_rtff_nrestore)
		return 0;

	switch (pd->data->rtff_type) {
	case SCPSYS_RTFF_TYPE_GENERIC:
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE_FLAG);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		break;
	case SCPSYS_RTFF_TYPE_PCIE_PHY:
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE_FLAG);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		break;
	case SCPSYS_RTFF_TYPE_STOR_UFS:
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_UFS_CLK_DIS);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_NRESTORE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_UFS_CLK_DIS);
		break;
	default:
		break;
	}

	return 0;
}

static void scpsys_ctl_pwrseq_off(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;

	switch (pd->data->rtff_type) {
	case SCPSYS_RTFF_TYPE_GENERIC:
	case SCPSYS_RTFF_TYPE_PCIE_PHY:
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_CLK_DIS);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE_FLAG);
		break;
	case SCPSYS_RTFF_TYPE_STOR_UFS:
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_UFS_CLK_DIS);
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_SAVE);
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RTFF_UFS_CLK_DIS);
		break;
	default:
		break;
	}

	/* subsys power off */
	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_ISO_BIT);

	/* Wait for RTFF HW to sync buck isolation state if this is PCIe PHY RTFF */
	if (pd->data->rtff_type == SCPSYS_RTFF_TYPE_PCIE_PHY)
		udelay(1);

	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_CLK_DIS_BIT);
	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RST_B_BIT);
	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_2ND_BIT);
	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_BIT);
}

static int scpsys_modem_pwrseq_on(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;
	bool tmp;
	int ret;

	if (!MTK_SCPD_CAPS(pd, MTK_SCPD_SKIP_RESET_B))
		regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_RST_B_BIT);

	regmap_set_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_BIT);

	/* wait until PWR_ACK = 1 */
	ret = readx_poll_timeout(scpsys_domain_is_on, pd, tmp, tmp, MTK_POLL_DELAY_US,
				 MTK_POLL_TIMEOUT);
	if (ret < 0)
		return ret;

	return 0;
}

static void scpsys_modem_pwrseq_off(struct scpsys_domain *pd)
{
	struct scpsys *scpsys = pd->scpsys;

	regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_ON_BIT);

	if (!MTK_SCPD_CAPS(pd, MTK_SCPD_SKIP_RESET_B))
		regmap_clear_bits(scpsys->base, pd->data->ctl_offs, PWR_RST_B_BIT);
}

static int scpsys_power_on(struct generic_pm_domain *genpd)
{
	struct scpsys_domain *pd = container_of(genpd, struct scpsys_domain, genpd);
	struct scpsys *scpsys = pd->scpsys;
	int ret;

	ret = scpsys_regulator_enable(pd->supply);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(pd->num_clks, pd->clks);
	if (ret)
		goto err_reg;

	if (pd->data->ext_buck_iso_offs && MTK_SCPD_CAPS(pd, MTK_SCPD_EXT_BUCK_ISO))
		regmap_clear_bits(scpsys->base, pd->data->ext_buck_iso_offs,
				  pd->data->ext_buck_iso_mask);

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MODEM_PWRSEQ))
		ret = scpsys_modem_pwrseq_on(pd);
	else
		ret = scpsys_ctl_pwrseq_on(pd);

	if (ret)
		goto err_pwr_ack;

	/*
	 * In few Mediatek platforms(e.g. MT6779), the bus protect policy is
	 * stricter, which leads to bus protect release must be prior to bus
	 * access.
	 */
	if (!MTK_SCPD_CAPS(pd, MTK_SCPD_STRICT_BUS_PROTECTION)) {
		ret = clk_bulk_prepare_enable(pd->num_subsys_clks,
					      pd->subsys_clks);
		if (ret)
			goto err_pwr_ack;
	}

	ret = scpsys_sram_enable(pd);
	if (ret < 0)
		goto err_disable_subsys_clks;

	ret = scpsys_bus_protect_disable(pd);
	if (ret < 0)
		goto err_disable_sram;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_STRICT_BUS_PROTECTION)) {
		ret = clk_bulk_prepare_enable(pd->num_subsys_clks,
					      pd->subsys_clks);
		if (ret)
			goto err_enable_bus_protect;
	}

	return 0;

err_enable_bus_protect:
	scpsys_bus_protect_enable(pd);
err_disable_sram:
	scpsys_sram_disable(pd);
err_disable_subsys_clks:
	if (!MTK_SCPD_CAPS(pd, MTK_SCPD_STRICT_BUS_PROTECTION))
		clk_bulk_disable_unprepare(pd->num_subsys_clks,
					   pd->subsys_clks);
err_pwr_ack:
	clk_bulk_disable_unprepare(pd->num_clks, pd->clks);
err_reg:
	scpsys_regulator_disable(pd->supply);
	return ret;
}

static int scpsys_power_off(struct generic_pm_domain *genpd)
{
	struct scpsys_domain *pd = container_of(genpd, struct scpsys_domain, genpd);
	struct scpsys *scpsys = pd->scpsys;
	bool tmp;
	int ret;

	ret = scpsys_bus_protect_enable(pd);
	if (ret < 0)
		return ret;

	ret = scpsys_sram_disable(pd);
	if (ret < 0)
		return ret;

	if (pd->data->ext_buck_iso_offs && MTK_SCPD_CAPS(pd, MTK_SCPD_EXT_BUCK_ISO))
		regmap_set_bits(scpsys->base, pd->data->ext_buck_iso_offs,
				pd->data->ext_buck_iso_mask);

	clk_bulk_disable_unprepare(pd->num_subsys_clks, pd->subsys_clks);

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_MODEM_PWRSEQ))
		scpsys_modem_pwrseq_off(pd);
	else
		scpsys_ctl_pwrseq_off(pd);

	/* wait until PWR_ACK = 0 */
	ret = readx_poll_timeout(scpsys_domain_is_on, pd, tmp, !tmp, MTK_POLL_DELAY_US,
				 MTK_POLL_TIMEOUT);
	if (ret < 0)
		return ret;

	clk_bulk_disable_unprepare(pd->num_clks, pd->clks);

	scpsys_regulator_disable(pd->supply);

	return 0;
}

static struct
generic_pm_domain *scpsys_add_one_domain(struct scpsys *scpsys, struct device_node *node)
{
	const struct scpsys_domain_data *domain_data;
	const struct scpsys_hwv_domain_data *hwv_domain_data;
	struct scpsys_domain *pd;
	struct property *prop;
	const char *clk_name;
	int i, ret, num_clks;
	struct clk *clk;
	int clk_ind = 0;
	u32 id;

	ret = of_property_read_u32(node, "reg", &id);
	if (ret) {
		dev_err(scpsys->dev, "%pOF: failed to retrieve domain id from reg: %d\n",
			node, ret);
		return ERR_PTR(-EINVAL);
	}

	switch (scpsys->soc_data->type) {
	case SCPSYS_MTCMOS_TYPE_DIRECT_CTL:
		if (id >= scpsys->soc_data->num_domains) {
			dev_err(scpsys->dev, "%pOF: invalid domain id %d\n", node, id);
			return ERR_PTR(-EINVAL);
		}

		domain_data = &scpsys->soc_data->domains_data[id];
		hwv_domain_data = NULL;

		if (domain_data->sta_mask == 0) {
			dev_err(scpsys->dev, "%pOF: undefined domain id %d\n", node, id);
			return ERR_PTR(-EINVAL);
		}

		break;
	case SCPSYS_MTCMOS_TYPE_HW_VOTER:
		if (id >= scpsys->soc_data->num_hwv_domains) {
			dev_err(scpsys->dev, "%pOF: invalid HWV domain id %d\n", node, id);
			return ERR_PTR(-EINVAL);
		}

		domain_data = NULL;
		hwv_domain_data = &scpsys->soc_data->hwv_domains_data[id];

		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	pd = devm_kzalloc(scpsys->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return ERR_PTR(-ENOMEM);

	pd->data = domain_data;
	pd->hwv_data = hwv_domain_data;
	pd->scpsys = scpsys;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_DOMAIN_SUPPLY)) {
		pd->supply = devm_of_regulator_get_optional(scpsys->dev, node, "domain");
		if (IS_ERR(pd->supply))
			return dev_err_cast_probe(scpsys->dev, pd->supply,
				      "%pOF: failed to get power supply.\n",
				      node);
	}

	num_clks = of_clk_get_parent_count(node);
	if (num_clks > 0) {
		/* Calculate number of subsys_clks */
		of_property_for_each_string(node, "clock-names", prop, clk_name) {
			char *subsys;

			subsys = strchr(clk_name, '-');
			if (subsys)
				pd->num_subsys_clks++;
			else
				pd->num_clks++;
		}

		pd->clks = devm_kcalloc(scpsys->dev, pd->num_clks, sizeof(*pd->clks), GFP_KERNEL);
		if (!pd->clks)
			return ERR_PTR(-ENOMEM);

		pd->subsys_clks = devm_kcalloc(scpsys->dev, pd->num_subsys_clks,
					       sizeof(*pd->subsys_clks), GFP_KERNEL);
		if (!pd->subsys_clks)
			return ERR_PTR(-ENOMEM);

	}

	for (i = 0; i < pd->num_clks; i++) {
		clk = of_clk_get(node, i);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err_probe(scpsys->dev, ret,
				      "%pOF: failed to get clk at index %d\n", node, i);
			goto err_put_clocks;
		}

		pd->clks[clk_ind++].clk = clk;
	}

	for (i = 0; i < pd->num_subsys_clks; i++) {
		clk = of_clk_get(node, i + clk_ind);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err_probe(scpsys->dev, ret,
				      "%pOF: failed to get clk at index %d\n", node,
				      i + clk_ind);
			goto err_put_subsys_clocks;
		}

		pd->subsys_clks[i].clk = clk;
	}

	if (scpsys->domains[id]) {
		ret = -EINVAL;
		dev_err(scpsys->dev,
			"power domain with id %d already exists, check your device-tree\n", id);
		goto err_put_subsys_clocks;
	}

	if (pd->data && pd->data->name)
		pd->genpd.name = pd->data->name;
	else if (pd->hwv_data && pd->hwv_data->name)
		pd->genpd.name = pd->hwv_data->name;
	else
		pd->genpd.name = node->name;

	if (scpsys->soc_data->type == SCPSYS_MTCMOS_TYPE_DIRECT_CTL) {
		pd->genpd.power_off = scpsys_power_off;
		pd->genpd.power_on = scpsys_power_on;
	} else {
		pd->genpd.power_off = scpsys_hwv_power_off;
		pd->genpd.power_on = scpsys_hwv_power_on;

		/* HW-Voter code can be invoked in atomic context */
		pd->genpd.flags |= GENPD_FLAG_IRQ_SAFE;
	}

	/*
	 * Initially turn on all domains to make the domains usable
	 * with !CONFIG_PM and to get the hardware in sync with the
	 * software.  The unused domains will be switched off during
	 * late_init time.
	 */
	if (MTK_SCPD_CAPS(pd, MTK_SCPD_KEEP_DEFAULT_OFF)) {
		if (scpsys_domain_is_on(pd))
			dev_warn(scpsys->dev,
				 "%pOF: A default off power domain has been ON\n", node);
	} else {
		ret = pd->genpd.power_on(&pd->genpd);
		if (ret < 0) {
			dev_err(scpsys->dev, "%pOF: failed to power on domain: %d\n", node, ret);
			goto err_put_subsys_clocks;
		}

		if (MTK_SCPD_CAPS(pd, MTK_SCPD_ALWAYS_ON))
			pd->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
	}

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_ACTIVE_WAKEUP))
		pd->genpd.flags |= GENPD_FLAG_ACTIVE_WAKEUP;

	if (MTK_SCPD_CAPS(pd, MTK_SCPD_KEEP_DEFAULT_OFF))
		pm_genpd_init(&pd->genpd, NULL, true);
	else
		pm_genpd_init(&pd->genpd, NULL, false);

	scpsys->domains[id] = &pd->genpd;

	return scpsys->pd_data.domains[id];

err_put_subsys_clocks:
	clk_bulk_put(pd->num_subsys_clks, pd->subsys_clks);
err_put_clocks:
	clk_bulk_put(pd->num_clks, pd->clks);
	return ERR_PTR(ret);
}

static int scpsys_add_subdomain(struct scpsys *scpsys, struct device_node *parent)
{
	struct generic_pm_domain *child_pd, *parent_pd;
	struct device_node *child;
	int ret;

	for_each_child_of_node(parent, child) {
		u32 id;

		ret = of_property_read_u32(parent, "reg", &id);
		if (ret) {
			dev_err(scpsys->dev, "%pOF: failed to get parent domain id\n", child);
			goto err_put_node;
		}

		if (!scpsys->pd_data.domains[id]) {
			ret = -EINVAL;
			dev_err(scpsys->dev, "power domain with id %d does not exist\n", id);
			goto err_put_node;
		}

		parent_pd = scpsys->pd_data.domains[id];

		child_pd = scpsys_add_one_domain(scpsys, child);
		if (IS_ERR(child_pd)) {
			ret = PTR_ERR(child_pd);
			dev_err_probe(scpsys->dev, ret, "%pOF: failed to get child domain id\n",
				      child);
			goto err_put_node;
		}

		/* recursive call to add all subdomains */
		ret = scpsys_add_subdomain(scpsys, child);
		if (ret)
			goto err_put_node;

		ret = pm_genpd_add_subdomain(parent_pd, child_pd);
		if (ret) {
			dev_err(scpsys->dev, "failed to add %s subdomain to parent %s\n",
				child_pd->name, parent_pd->name);
			goto err_put_node;
		} else {
			dev_dbg(scpsys->dev, "%s add subdomain: %s\n", parent_pd->name,
				child_pd->name);
		}
	}

	return 0;

err_put_node:
	of_node_put(child);
	return ret;
}

static void scpsys_remove_one_domain(struct scpsys_domain *pd)
{
	int ret;

	/*
	 * We're in the error cleanup already, so we only complain,
	 * but won't emit another error on top of the original one.
	 */
	ret = pm_genpd_remove(&pd->genpd);
	if (ret < 0)
		dev_err(pd->scpsys->dev,
			"failed to remove domain '%s' : %d - state may be inconsistent\n",
			pd->genpd.name, ret);
	if (scpsys_domain_is_on(pd))
		scpsys_power_off(&pd->genpd);

	clk_bulk_put(pd->num_clks, pd->clks);
	clk_bulk_put(pd->num_subsys_clks, pd->subsys_clks);
}

static void scpsys_domain_cleanup(struct scpsys *scpsys)
{
	struct generic_pm_domain *genpd;
	struct scpsys_domain *pd;
	int i;

	for (i = scpsys->pd_data.num_domains - 1; i >= 0; i--) {
		genpd = scpsys->pd_data.domains[i];
		if (genpd) {
			pd = to_scpsys_domain(genpd);
			scpsys_remove_one_domain(pd);
		}
	}
}

static int scpsys_get_bus_protection_legacy(struct device *dev, struct scpsys *scpsys)
{
	const u8 bp_blocks[3] = {
		BUS_PROT_BLOCK_INFRA, BUS_PROT_BLOCK_SMI, BUS_PROT_BLOCK_INFRA_NAO
	};
	struct device_node *np = dev->of_node;
	struct device_node *node, *smi_np;
	int num_regmaps = 0, i, j;
	struct regmap *regmap[3];

	/*
	 * Legacy code retrieves a maximum of three bus protection handles:
	 * some may be optional, or may not be, so the array of bp blocks
	 * that is normally passed in as platform data must be dynamically
	 * built in this case.
	 *
	 * Here, try to retrieve all of the regmaps that the legacy code
	 * supported and then count the number of the ones that are present,
	 * this makes it then possible to allocate the array of bus_prot
	 * regmaps and convert all to the new style handling.
	 */
	node = of_find_node_with_property(np, "mediatek,infracfg");
	if (node) {
		regmap[0] = syscon_regmap_lookup_by_phandle(node, "mediatek,infracfg");
		of_node_put(node);
		num_regmaps++;
		if (IS_ERR(regmap[0]))
			return dev_err_probe(dev, PTR_ERR(regmap[0]),
					     "%pOF: failed to get infracfg regmap\n",
					     node);
	} else {
		regmap[0] = NULL;
	}

	node = of_find_node_with_property(np, "mediatek,smi");
	if (node) {
		smi_np = of_parse_phandle(node, "mediatek,smi", 0);
		of_node_put(node);
		if (!smi_np)
			return -ENODEV;

		regmap[1] = device_node_to_regmap(smi_np);
		num_regmaps++;
		of_node_put(smi_np);
		if (IS_ERR(regmap[1]))
			return dev_err_probe(dev, PTR_ERR(regmap[1]),
					     "%pOF: failed to get SMI regmap\n",
					     node);
	} else {
		regmap[1] = NULL;
	}

	node = of_find_node_with_property(np, "mediatek,infracfg-nao");
	if (node) {
		regmap[2] = syscon_regmap_lookup_by_phandle(node, "mediatek,infracfg-nao");
		num_regmaps++;
		of_node_put(node);
		if (IS_ERR(regmap[2]))
			return dev_err_probe(dev, PTR_ERR(regmap[2]),
					     "%pOF: failed to get infracfg regmap\n",
					     node);
	} else {
		regmap[2] = NULL;
	}

	scpsys->bus_prot = devm_kmalloc_array(dev, num_regmaps,
					      sizeof(*scpsys->bus_prot), GFP_KERNEL);
	if (!scpsys->bus_prot)
		return -ENOMEM;

	for (i = 0, j = 0; i < ARRAY_SIZE(bp_blocks); i++) {
		enum scpsys_bus_prot_block bp_type;

		if (!regmap[i])
			continue;

		bp_type = bp_blocks[i];
		scpsys->bus_prot_index[bp_type] = j;
		scpsys->bus_prot[j] = regmap[i];

		j++;
	}

	return 0;
}

static int scpsys_get_bus_protection(struct device *dev, struct scpsys *scpsys)
{
	const struct scpsys_soc_data *soc = scpsys->soc_data;
	struct device_node *np = dev->of_node;
	int i, num_handles;

	num_handles = of_count_phandle_with_args(np, "access-controllers", NULL);
	if (num_handles < 0 || num_handles != soc->num_bus_prot_blocks)
		return dev_err_probe(dev, -EINVAL,
				     "Cannot get access controllers: expected %u, got %d\n",
				     soc->num_bus_prot_blocks, num_handles);

	scpsys->bus_prot = devm_kmalloc_array(dev, soc->num_bus_prot_blocks,
					      sizeof(*scpsys->bus_prot), GFP_KERNEL);
	if (!scpsys->bus_prot)
		return -ENOMEM;

	for (i = 0; i < soc->num_bus_prot_blocks; i++) {
		enum scpsys_bus_prot_block bp_type;
		struct device_node *node;

		node = of_parse_phandle(np, "access-controllers", i);
		if (!node)
			return -EINVAL;

		/*
		 * Index the bus protection regmaps so that we don't have to
		 * find the right one by type with a loop at every execution
		 * of power sequence(s).
		 */
		bp_type = soc->bus_prot_blocks[i];
		scpsys->bus_prot_index[bp_type] = i;

		scpsys->bus_prot[i] = device_node_to_regmap(node);
		of_node_put(node);
		if (IS_ERR_OR_NULL(scpsys->bus_prot[i]))
			return dev_err_probe(dev, scpsys->bus_prot[i] ?
					     PTR_ERR(scpsys->bus_prot[i]) : -ENXIO,
					     "Cannot get regmap for access controller %d\n", i);
	}

	return 0;
}

static const struct of_device_id scpsys_of_match[] = {
	{
		.compatible = "mediatek,mt6735-power-controller",
		.data = &mt6735_scpsys_data,
	},
	{
		.compatible = "mediatek,mt6795-power-controller",
		.data = &mt6795_scpsys_data,
	},
	{
		.compatible = "mediatek,mt6893-power-controller",
		.data = &mt6893_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8167-power-controller",
		.data = &mt8167_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8173-power-controller",
		.data = &mt8173_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8183-power-controller",
		.data = &mt8183_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8186-power-controller",
		.data = &mt8186_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8188-power-controller",
		.data = &mt8188_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8192-power-controller",
		.data = &mt8192_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8195-power-controller",
		.data = &mt8195_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8196-power-controller",
		.data = &mt8196_scpsys_data,
	},
	{
		.compatible = "mediatek,mt8196-hwv-hfrp-power-controller",
		.data = &mt8196_hfrpsys_hwv_data,
	},
	{
		.compatible = "mediatek,mt8196-hwv-scp-power-controller",
		.data = &mt8196_scpsys_hwv_data,
	},
	{
		.compatible = "mediatek,mt8365-power-controller",
		.data = &mt8365_scpsys_data,
	},
	{ }
};

static int scpsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct scpsys_soc_data *soc;
	struct device_node *node;
	struct device *parent;
	struct scpsys *scpsys;
	int num_domains, ret;

	soc = of_device_get_match_data(&pdev->dev);
	if (!soc) {
		dev_err(&pdev->dev, "no power controller data\n");
		return -EINVAL;
	}

	num_domains = soc->num_domains + soc->num_hwv_domains;

	scpsys = devm_kzalloc(dev, struct_size(scpsys, domains, num_domains), GFP_KERNEL);
	if (!scpsys)
		return -ENOMEM;

	scpsys->dev = dev;
	scpsys->soc_data = soc;

	scpsys->pd_data.domains = scpsys->domains;
	scpsys->pd_data.num_domains = soc->num_domains;

	parent = dev->parent;
	if (!parent) {
		dev_err(dev, "no parent for syscon devices\n");
		return -ENODEV;
	}

	scpsys->base = syscon_node_to_regmap(parent->of_node);
	if (IS_ERR(scpsys->base)) {
		dev_err(dev, "no regmap available\n");
		return PTR_ERR(scpsys->base);
	}

	if (of_find_property(np, "access-controllers", NULL))
		ret = scpsys_get_bus_protection(dev, scpsys);
	else
		ret = scpsys_get_bus_protection_legacy(dev, scpsys);

	if (ret)
		return ret;

	ret = -ENODEV;
	for_each_available_child_of_node(np, node) {
		struct generic_pm_domain *domain;

		domain = scpsys_add_one_domain(scpsys, node);
		if (IS_ERR(domain)) {
			ret = PTR_ERR(domain);
			of_node_put(node);
			goto err_cleanup_domains;
		}

		ret = scpsys_add_subdomain(scpsys, node);
		if (ret) {
			of_node_put(node);
			goto err_cleanup_domains;
		}
	}

	if (ret) {
		dev_dbg(dev, "no power domains present\n");
		return ret;
	}

	ret = of_genpd_add_provider_onecell(np, &scpsys->pd_data);
	if (ret) {
		dev_err(dev, "failed to add provider: %d\n", ret);
		goto err_cleanup_domains;
	}

	return 0;

err_cleanup_domains:
	scpsys_domain_cleanup(scpsys);
	return ret;
}

static struct platform_driver scpsys_pm_domain_driver = {
	.probe = scpsys_probe,
	.driver = {
		.name = "mtk-power-controller",
		.suppress_bind_attrs = true,
		.of_match_table = scpsys_of_match,
	},
};
builtin_platform_driver(scpsys_pm_domain_driver);
