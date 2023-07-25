// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#define FSL_SIP_DDR_DVFS                0xc2000004
#define DDR_DFS_GET_FSP_COUNT		0x10
#define DDR_DFS_GET_FSP_INFO		0x11
#define DDR_FSP_HIGH			0
#define DDR_FSP_MID			1
#define DDR_FSP_LOW			2
#define DDR_DFS_FSP_NUM_MIN		3

#define AUTO_CG_CTRL			0x10
#define HWFFC_ACG_FORCE_B		BIT(17)
#define AUTO_CG_EN			BIT(16)

#define DEFAULT_IDLE_STRAP		32768

#define VDD_SOC_LD_VOLTAGE		800000
#define VDD_SOC_ND_VOLTAGE		850000
#define VDD_SOC_OD_VOLTAGE		900000

enum SYS_PLL_CLKS {
	SYS_PLL_PFD0,
	SYS_PLL_PFD0_DIV2,
	SYS_PLL_PFD1,
	SYS_PLL_PFD1_DIV2,
	SYS_PLL_PFD2,
	SYS_PLL_PFD2_DIV2,
};

enum mode_type {
	OD_MODE,
	ND_MODE,
	LD_MODE,
	SWFFC_MODE,
	MODE_END,
};

enum clk_path_index {
	M33_ROOT,
	WAKEUP_AXI,
	MEDIA_AXI,
	ML_AXI,
	NIC_AXI,
	A55_PERIPH,
	A55_CORE,
	CLK_PATH_END,
};

struct critical_clk_path {
	/* clock name */
	char *name;
	/* clk */
	struct clk *clk;
	/* initial rate */
	unsigned long initial_rate;
	/* current rate */
	unsigned long current_rate;
	/* mode rate */
	unsigned long mode_rate[MODE_END];
};

#define CLK_PATH(n, o_rate, n_rate, l_rate)		\
	{						\
		.name = #n,				\
		.mode_rate = {o_rate, n_rate, l_rate },	\
	}

struct operating_mode {
	enum mode_type current_mode;
	enum mode_type resume_mode;
	bool auto_gate_enabled;
	unsigned int ssi_strap;
	/* critical clocks */
	struct critical_clk_path paths[CLK_PATH_END];
};

static struct operating_mode system_run_mode = {
	.paths = {
		CLK_PATH(m33_root, 250000000, 200000000, 133000000),
		CLK_PATH(wakeup_axi, 400000000, 312500000, 200000000),
		CLK_PATH(media_axi, 400000000, 333000000, 200000000),
		CLK_PATH(ml_axi, 1000000000, 800000000, 500000000),
		CLK_PATH(nic_axi, 500000000, 400000000, 250000000),
		CLK_PATH(a55_periph, 400000000, 333000000, 200000000),
		CLK_PATH(a55_core, 1700000000, 1400000000, 900000000),
	},
};

static struct clk_bulk_data clks[] = {
	{ .id = "sys_pll_pfd0" },
	{ .id = "sys_pll_pfd0_div2" },
	{ .id = "sys_pll_pfd1" },
	{ .id = "sys_pll_pfd1_div2" },
	{ .id = "sys_pll_pfd2" },
	{ .id = "sys_pll_pfd2_div2" },
};

static bool ld_mode_enabled;
static bool no_od_mode;
static unsigned int num_fsp;
static unsigned int fsp_table[3];
static struct regulator *soc_reg;
static struct regmap *regmap;
DEFINE_MUTEX(mode_mutex);

/* both HWFFC & SWFFC need to call this function */
static int scaling_dram_freq(unsigned int fsp_index)
{
	struct arm_smccc_res res;
	u32 num_cpus = num_online_cpus();

	/* need to check the return value ?*/
	arm_smccc_smc(FSL_SIP_DDR_DVFS, fsp_index, num_cpus,
		0, 0, 0, 0, 0, &res);

	return 0;
}

static void sys_freq_scaling(enum mode_type new_mode)
{
	int i;
	struct critical_clk_path *path = system_run_mode.paths;

	mutex_lock(&mode_mutex);

	if (new_mode == system_run_mode.current_mode) {
		pr_debug("System already in target mode, do nothing\n");
		mutex_unlock(&mode_mutex);
		return;
	}

	if (new_mode == OD_MODE) {
		/* increase the voltage first */
		regulator_set_voltage_tol(soc_reg, VDD_SOC_OD_VOLTAGE, 0);

		for (i = 0; i < CLK_PATH_END; i++) {
			if (i == M33_ROOT) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD0_DIV2].clk);
			} else if (i == MEDIA_AXI || i == A55_PERIPH) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD1].clk);
			} else if (i == ML_AXI || i == NIC_AXI) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD0].clk);
			} else if (i == WAKEUP_AXI) {
				clk_set_parent(path[i].clk, path[i].initial_rate > 312500000 ?
						clks[SYS_PLL_PFD1].clk : clks[SYS_PLL_PFD2].clk);
			}

			clk_set_rate(path[i].clk, path[i].mode_rate[OD_MODE]);
		}

		/* Scaling up the DDR frequency */
		scaling_dram_freq(0x0);
		pr_info("System switching to OD mode...\n");
	} else if (new_mode == ND_MODE) {
		/*
		 * if switch from LD mode to ND mode, voltage should be increase firstly.
		 */
		if (system_run_mode.current_mode == LD_MODE)
			regulator_set_voltage_tol(soc_reg, VDD_SOC_ND_VOLTAGE, 0);

		for (i = 0; i < CLK_PATH_END; i++) {
			if (i == M33_ROOT) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD1_DIV2].clk);
			} else if (i == MEDIA_AXI || i == A55_PERIPH) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD0].clk);
			} else if (i == ML_AXI || i == NIC_AXI) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD1].clk);
			} else if (i == WAKEUP_AXI) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD2].clk);
			}

				clk_set_rate(path[i].clk, path[i].mode_rate[ND_MODE]);
		}

		/* Scaling down the ddr frequency. */
		scaling_dram_freq(no_od_mode ? 0x0 : 0x1);

		if (system_run_mode.current_mode != LD_MODE)
			regulator_set_voltage_tol(soc_reg, VDD_SOC_ND_VOLTAGE, 0);

		pr_info("System switching to ND mode...\n");
	} else if (new_mode == LD_MODE || new_mode == SWFFC_MODE) {
		for (i = 0; i < CLK_PATH_END; i++) {
			/*
			 * NIC AXI frequency should be changed after all other clock
			 * has been slow down. Especially NIC AXI should be reduced
			 * after A55 related clocks.
			 */
			if (i == NIC_AXI)
				continue;

			if (i == M33_ROOT) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD1_DIV2].clk);
			} else if (i == MEDIA_AXI || i == A55_PERIPH) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD1].clk);
			} else if (i == ML_AXI) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD0].clk);
			} else if (i == WAKEUP_AXI) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD1].clk);
			}

			clk_set_rate(path[i].clk, path[i].mode_rate[LD_MODE]);
		}

		clk_set_parent(path[NIC_AXI].clk, clks[SYS_PLL_PFD0].clk);
		clk_set_rate(path[NIC_AXI].clk, path[NIC_AXI].mode_rate[LD_MODE]);

		/* Scaling down the ddr frequency. */
		scaling_dram_freq(new_mode == LD_MODE ? 0x1 : 0x2);

		regulator_set_voltage_tol(soc_reg, VDD_SOC_LD_VOLTAGE, 0);

		pr_info("System switching to LD/SWFFC mode...\n");
	}

	system_run_mode.current_mode = new_mode;

	mutex_unlock(&mode_mutex);
}

static ssize_t lpm_enable_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	switch (system_run_mode.current_mode) {
	case OD_MODE:
		return sprintf(buf, "System is in OD mode with DDR %d MTS!\n", fsp_table[0]);
	case ND_MODE:
		return sprintf(buf, "System is in ND mode with DDR %d MTS!\n",
			       no_od_mode ? fsp_table[0] : fsp_table[1]);
	case LD_MODE:
		return sprintf(buf, "System is in LD mode with DDR %d MTS!\n", fsp_table[1]);
	case SWFFC_MODE:
		return sprintf(buf, "System is in LD mode with DDR %d MTS!\n", fsp_table[2]);
	default:
		return sprintf(buf, "Unknown system mode\n");
	}
}

static ssize_t lpm_enable_store(struct device *dev,
			        struct device_attribute *attr,
				const char *buf, size_t count)
{
	u16 new_mode;

	if (kstrtou16(buf, 10, &new_mode))
		return -EINVAL;

	if (new_mode >= MODE_END)
		return -EINVAL;

	/* if only two ddr frequency setpoint, LD+SWFFC can not be supported */
	if (num_fsp <= 2 && new_mode == SWFFC_MODE)
		return -EINVAL;

	/* if LD mode is not enabled in dts, only OD & ND mode can be supported */
	if (!ld_mode_enabled && new_mode >= LD_MODE)
		return -EINVAL;

	if (new_mode == OD_MODE && no_od_mode)
		return -EINVAL;

	/* make sure auto clock gating is disabled before DDR frequency scaling */
	regmap_update_bits(regmap, AUTO_CG_CTRL, AUTO_CG_EN, 0);

	if ((system_run_mode.current_mode != OD_MODE && new_mode == SWFFC_MODE) ||
	    (system_run_mode.current_mode == SWFFC_MODE && new_mode != OD_MODE)) 
		sys_freq_scaling(no_od_mode ? ND_MODE : OD_MODE);

	sys_freq_scaling(new_mode);

	if (system_run_mode.auto_gate_enabled ||
	    (system_run_mode.current_mode == OD_MODE && fsp_table[0] >= 3733))
		regmap_update_bits(regmap, AUTO_CG_CTRL, AUTO_CG_EN, AUTO_CG_EN);

	return count;
}

static ssize_t auto_clk_gating_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	u32 val;

	regmap_read(regmap, AUTO_CG_CTRL, &val);
	if (val & AUTO_CG_EN)
		return sprintf(buf, "DDR auto clock gating enabled with idle strap: %u!\n", val & 0xffff);
	else
		return sprintf(buf, "DDR auto clock gating disabled!\n");
}

static ssize_t auto_clk_gating_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	u16 ssi_idle_strap;
	if (kstrtou16(buf, 10, &ssi_idle_strap))
		return -EINVAL;

	if (ssi_idle_strap) {
		/* idle strap should be set to a value >= 256 */
		regmap_update_bits(regmap, AUTO_CG_CTRL, 0xFFFF, ssi_idle_strap > 256 ?
			ssi_idle_strap : 0x100);
		regmap_update_bits(regmap, AUTO_CG_CTRL, HWFFC_ACG_FORCE_B | AUTO_CG_EN,
				   HWFFC_ACG_FORCE_B | AUTO_CG_EN);
		system_run_mode.ssi_strap = ssi_idle_strap;
		system_run_mode.auto_gate_enabled = true;
	} else {
		regmap_update_bits(regmap, AUTO_CG_CTRL, AUTO_CG_EN, 0);
		system_run_mode.auto_gate_enabled = false;
	}

	return count;
}

static DEVICE_ATTR(mode, 0644, lpm_enable_show, lpm_enable_store);
static DEVICE_ATTR(auto_clk_gating, 0644, auto_clk_gating_show, auto_clk_gating_store);

static const struct attribute *imx93_lpm_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_auto_clk_gating.attr,
	NULL
};

static int imx93_lpm_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	if (event == PM_SUSPEND_PREPARE) {
		system_run_mode.resume_mode = system_run_mode.current_mode;
		/* make sure auto clock gating is disabled */
		regmap_update_bits(regmap, AUTO_CG_CTRL, AUTO_CG_EN, 0);
		sys_freq_scaling(no_od_mode ? ND_MODE : OD_MODE);
		/* save the ssi idle strap */
		regmap_read(regmap, AUTO_CG_CTRL, &system_run_mode.ssi_strap);
	} else if (event == PM_POST_SUSPEND) {
		sys_freq_scaling(system_run_mode.resume_mode);

		/* restore the ssi idle strap */
		regmap_update_bits(regmap, AUTO_CG_CTRL, 0xFFFF, system_run_mode.ssi_strap);
		if (system_run_mode.auto_gate_enabled ||
		    (system_run_mode.current_mode == OD_MODE && fsp_table[0] >= 3733)) {
			/* enable the auto clock gating */
			regmap_update_bits(regmap, AUTO_CG_CTRL, HWFFC_ACG_FORCE_B | AUTO_CG_EN,
				 HWFFC_ACG_FORCE_B | AUTO_CG_EN);
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block imx93_lpm_pm_notifier = {
	.notifier_call = imx93_lpm_pm_notify,
};

/* sysfs for user control */
static int imx93_lpm_probe(struct platform_device *pdev)
{
	int i, err;
	struct arm_smccc_res res;
	struct critical_clk_path *path = system_run_mode.paths;

	/*
	 * get the supported frequency number, if only
	 * one setpoint, then no mode switching can be supported.
	 * if three setpoints(f0,f1,f2) available, we asuming fsp2
	 * is used for DDR SWFFC in LD mode 
	 */
	arm_smccc_smc(FSL_SIP_DDR_DVFS, DDR_DFS_GET_FSP_COUNT, 0,
		0, 0, 0, 0, 0, &res);
	num_fsp = res.a0;
	if (num_fsp < 2) {
		pr_info("no ddr frequency scaling can be supported");
		return -EINVAL;
	}

	/* get the available ddr fsp info */
	for (i = 0; i <num_fsp; i++) {
		arm_smccc_smc(FSL_SIP_DDR_DVFS, DDR_DFS_GET_FSP_INFO, i,
			0, 0, 0, 0, 0, &res);
		err = res.a0;
		if (err < 0)
			return -EINVAL;
		fsp_table[i] = res.a0;
	}

	/* ld mode can only be used when it is enabled explictly */
	ld_mode_enabled = of_property_read_bool(pdev->dev.of_node, "ld-mode-enabled");
	/* no OD mode for i.MX91/P */
	no_od_mode = of_property_read_bool(pdev->dev.of_node, "no-od-mode");

	regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "regmap");
	if (IS_ERR(regmap)) {
		dev_err(&pdev->dev, "get ddrmix blk ctrl regmap failed\n");
		return PTR_ERR(regmap);
	}

	soc_reg = devm_regulator_get(&pdev->dev, "soc");
	if (IS_ERR(soc_reg))
		return PTR_ERR(soc_reg);

	/*
	 * initial auto clock gating ssi strap, set to 32768 by default,
	 * too small value will impact the ddr performance significantly.
	 */
	regmap_update_bits(regmap, 0x10, 0xffff, DEFAULT_IDLE_STRAP);

	/*
	 * enable the ddr auto clock gating if highest ddr frequeency is 3733mts
	 * as it can provide a better balance between performance & power.
	 * Normally we assuming system boot from the highest frequency by default.
	 */
	if (fsp_table[0] >= 3733)
		regmap_update_bits(regmap, AUTO_CG_CTRL, HWFFC_ACG_FORCE_B | AUTO_CG_EN,
				   HWFFC_ACG_FORCE_B | AUTO_CG_EN);

	/* Get all the critical path's clock */
	for (i = 0; i < CLK_PATH_END; i++) {
		path[i].clk = devm_clk_get(&pdev->dev, path[i].name);
		if (IS_ERR(path[i].clk))
			return -ENODEV;
		path[i].initial_rate = clk_get_rate(path[i].clk);
	}

	err = clk_bulk_get(&pdev->dev, 6, clks);
	if (err) {
		dev_err(&pdev->dev, "failed to get bulk clks\n");
		return err;
	}

	/* Normally, we assuming the system in boot up in OD or ND(i.MX91/P) mode */
	system_run_mode.current_mode = no_od_mode ? ND_MODE : OD_MODE;

	/* create the sysfs file */
	err = sysfs_create_files(&pdev->dev.kobj, imx93_lpm_attrs);
	if (err) {
		dev_err(&pdev->dev, "creating i.MX93 LPM control sys file\n");
		return err;
	}

	register_pm_notifier(&imx93_lpm_pm_notifier);

	return 0;
}

static const struct of_device_id imx93_lpm_ids[] = {
	{.compatible = "nxp,imx93-lpm", },
	{ /* sentinel */}
};

static struct platform_driver imx93_lpm_driver = {
	.driver = {
		.name = "imx93-lpm",
		.owner = THIS_MODULE,
		.of_match_table = imx93_lpm_ids,
		},
	.probe = imx93_lpm_probe,
};
module_platform_driver(imx93_lpm_driver);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX93 Low Power Control Driver");
MODULE_LICENSE("GPL");
