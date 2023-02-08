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
#define DDR_FSP_HIGH			0
#define DDR_FSP_MID			1
#define DDR_FSP_LOW			2
#define DDR_DFS_FSP_NUM_MIN		3

#define AUTO_CG_CTRL			0x10
#define HWFFC_ACG_FORCE_B		BIT(17)
#define AUTO_CG_EN			BIT(16)

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
		CLK_PATH(a55_core, 1700000000, 1400000000, 90000000),
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

static unsigned int num_fsp;
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
	} else if (system_run_mode.current_mode == ND_MODE && new_mode == SWFFC_MODE) {
		pr_warn("DDR HWFFC enabled, please exit HWFFC first!\n");
		mutex_unlock(&mode_mutex);
		return;
	} else if (system_run_mode.current_mode == SWFFC_MODE && new_mode == ND_MODE) {
		pr_warn("DDR SWFFC enabled, please exit SWFFC first!\n");
		mutex_unlock(&mode_mutex);
		return;
	}

	if (new_mode == ND_MODE || new_mode == SWFFC_MODE) {
		for (i = 0; i < CLK_PATH_END; i++) {
			if (i == M33_ROOT) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD1_DIV2].clk);
			} else if (i == MEDIA_AXI || i == A55_PERIPH) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD0].clk);
			} else if (i == ML_AXI || i == NIC_AXI) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD1].clk);
			}

			clk_set_rate(path[i].clk, path[i].mode_rate[ND_MODE]);
		}

		/* Scaling down the ddr frequency. */
		scaling_dram_freq(new_mode == ND_MODE ? 0x1 : 0x2);

		regulator_set_voltage_tol(soc_reg, VDD_SOC_ND_VOLTAGE, 0);

		pr_info("System switching to ND/SWFFC mode...\n");
	} else if (new_mode == OD_MODE) {
		/* increase the voltage first */
		regulator_set_voltage_tol(soc_reg, VDD_SOC_OD_VOLTAGE, 0);

		for (i = 0; i < CLK_PATH_END; i++) {
			if (i == M33_ROOT) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD0_DIV2].clk);
			} else if (i == MEDIA_AXI || i == A55_PERIPH) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD1].clk);
			} else if (i == ML_AXI || i == NIC_AXI) {
				clk_set_parent(path[i].clk, clks[SYS_PLL_PFD0].clk);
			}

			clk_set_rate(path[i].clk, path[i].mode_rate[OD_MODE]);
		}

		/* Scaling up the DDR frequency */
		scaling_dram_freq(0x0);
		pr_info("System switching to OD mode...\n");
	}

	system_run_mode.current_mode = new_mode;

	mutex_unlock(&mode_mutex);
}

static ssize_t lpm_enable_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	if(system_run_mode.current_mode == ND_MODE) {
		return sprintf(buf, "System is in ND mode!\n");
	} else if(system_run_mode.current_mode == OD_MODE) {
		return sprintf(buf, "System is in OD mode!\n");
	} else {
		return sprintf(buf, "System is in SWFFC mode!\n");
	}
}

static ssize_t lpm_enable_store(struct device *dev,
			        struct device_attribute *attr,
				const char *buf, size_t count)
{
	if (!strncmp(buf, "nd", 2)) {
		sys_freq_scaling(ND_MODE);
	} else if (!strncmp(buf, "od", 2)) {
		sys_freq_scaling(OD_MODE);
	}

	return count;
}

static ssize_t auto_gate_enable_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	u32 val;

	regmap_read(regmap, AUTO_CG_CTRL, &val);
	if (val & AUTO_CG_EN)
		return sprintf(buf, "DDR auto clock gating enabled!\n");
	else
		return sprintf(buf, "DDR auto clock gating disabled!\n");
}

static ssize_t auto_gate_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	if (!strncmp(buf, "1", 1)) {
		regmap_update_bits(regmap, AUTO_CG_CTRL, HWFFC_ACG_FORCE_B | AUTO_CG_EN,
				 HWFFC_ACG_FORCE_B | AUTO_CG_EN);
		system_run_mode.auto_gate_enabled = true;
	} else if (!strncmp(buf, "0", 1)) {
		regmap_update_bits(regmap, AUTO_CG_CTRL, AUTO_CG_EN, 0);
		system_run_mode.auto_gate_enabled = false;
	}

	return count;
}

static ssize_t idle_delay_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	unsigned int ssi_idle_strap;

	regmap_read(regmap, AUTO_CG_CTRL, &ssi_idle_strap);
	ssi_idle_strap &= 0xFFFF;

	return sprintf(buf, "ddr idle delay is %d\n", ssi_idle_strap);
}

static ssize_t idle_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	u16 ssi_idle_strap;

	if (kstrtou16(buf, 10, &ssi_idle_strap))
		return -EINVAL;
	regmap_update_bits(regmap, 0x10, 0xFFFF, ssi_idle_strap);

	return count;
}

static ssize_t swffc_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	if (system_run_mode.current_mode == SWFFC_MODE)
		return sprintf(buf, "DDR SWFFC is enabled\n");
	 else
		return sprintf(buf, "DDR SWFFC is not enabled\n");
}

static ssize_t swffc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	if (!strncmp(buf, "1", 1)) {
		sys_freq_scaling(SWFFC_MODE);
	} else if (!strncmp(buf, "0", 1)) {
		sys_freq_scaling(OD_MODE);
	}

	return count;
}

static DEVICE_ATTR(mode, 0644, lpm_enable_show, lpm_enable_store);
static DEVICE_ATTR(auto_clk_gating, 0644, auto_gate_enable_show, auto_gate_enable_store);
static DEVICE_ATTR(idle_delay, 0644, idle_delay_show, idle_delay_store);
static DEVICE_ATTR(swffc, 0644, swffc_show, swffc_store);

static const struct attribute *imx93_lpm_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_auto_clk_gating.attr,
	&dev_attr_idle_delay.attr,
	&dev_attr_swffc.attr,
	NULL
};

static int imx93_lpm_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	if (event == PM_SUSPEND_PREPARE) {
		system_run_mode.resume_mode = system_run_mode.current_mode;
		sys_freq_scaling(OD_MODE);

		/* save the ssi idle strap */
		regmap_read(regmap, AUTO_CG_CTRL, &system_run_mode.ssi_strap);
		if (system_run_mode.auto_gate_enabled) {
			/* disable the auto clock gating */
			regmap_update_bits(regmap, AUTO_CG_CTRL, AUTO_CG_EN, 0);
		}
	} else if (event == PM_POST_SUSPEND) {
		sys_freq_scaling(system_run_mode.resume_mode);

		/* restore the ssi idle strap */
		regmap_update_bits(regmap, AUTO_CG_CTRL, 0xFFFF, system_run_mode.ssi_strap);
		if (system_run_mode.auto_gate_enabled) {
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
	 * one setpoint, then no mode switching can be supported
	 */
	arm_smccc_smc(FSL_SIP_DDR_DVFS, DDR_DFS_GET_FSP_COUNT, 0,
		0, 0, 0, 0, 0, &res);
	num_fsp = res.a0;
	if (num_fsp < 2) {
		pr_info("no ddr frequency scaling can be supported");
		return -EINVAL;
	}

	regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "regmap");
	if (IS_ERR(regmap)) {
		dev_err(&pdev->dev, "get ddrmix blk ctrl regmap failed\n");
		return PTR_ERR(regmap);
	}

	soc_reg = devm_regulator_get(&pdev->dev, "soc");
	if (IS_ERR(soc_reg))
		return PTR_ERR(soc_reg);

	/* initial auto clock gating ssi strap */
	regmap_update_bits(regmap, 0x10, 0xffff, 0x100);

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
