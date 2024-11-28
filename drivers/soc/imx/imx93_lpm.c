// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022, 2024 NXP
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/firmware/imx/se_api.h>
#include <linux/thermal.h>

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

#define MAX_COOLING_LEVEL 1

enum SYS_PLL_CLKS {
	SYS_PLL_PFD0,
	SYS_PLL_PFD0_DIV2,
	SYS_PLL_PFD1,
	SYS_PLL_PFD1_DIV2,
	SYS_PLL_PFD2,
	SYS_PLL_PFD2_DIV2,
	/*
	 * If SYS_PLL_PFD_END is selected for the parent clock of
	 * a clock, it means that the parent clock of this clock
	 * does not need to be changed.
	 */
	SYS_PLL_PFD_END,
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
	MEDIA_APB,
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
	/* mode parent */
	enum SYS_PLL_CLKS mode_parent[MODE_END];
};

#define CLK_PATH(n, o_rate, o_parent, \
					n_rate, n_parent, \
					l_rate, l_parent) \
	{						\
		.name = #n,				\
		.mode_rate = {o_rate, n_rate, l_rate },	\
		.mode_parent = {o_parent, n_parent, l_parent} \
	}

struct operating_mode {
	bool cooling_actived;
	bool suspend_prepared;
	enum mode_type manual_mode;
	enum mode_type current_mode;
	bool auto_gate_enabled;
	unsigned int ssi_strap;
	/* critical clocks */
	struct critical_clk_path paths[CLK_PATH_END];
};

static struct operating_mode system_run_mode;

static struct operating_mode system_run_mode_91 = {
	.paths = {
		[M33_ROOT] = CLK_PATH(m33_root, 0, SYS_PLL_PFD_END,
								200000000, SYS_PLL_PFD1_DIV2,
								133000000, SYS_PLL_PFD1_DIV2),
		[WAKEUP_AXI] = CLK_PATH(wakeup_axi, 0, SYS_PLL_PFD_END,
								250000000, SYS_PLL_PFD0,
								200000000, SYS_PLL_PFD1),
		[MEDIA_AXI] = CLK_PATH(media_axi, 0, SYS_PLL_PFD_END,
								333000000, SYS_PLL_PFD0,
								200000000, SYS_PLL_PFD1),
		[MEDIA_APB] = CLK_PATH(media_apb, 0, SYS_PLL_PFD_END,
								125000000, SYS_PLL_PFD0_DIV2,
								133000000, SYS_PLL_PFD1_DIV2),
		[ML_AXI] = CLK_PATH(ml_axi, 0, SYS_PLL_PFD_END,
								800000000, SYS_PLL_PFD1,
								500000000, SYS_PLL_PFD0),
		[NIC_AXI] = CLK_PATH(nic_axi, 0, SYS_PLL_PFD_END,
								333000000, SYS_PLL_PFD0,
								250000000, SYS_PLL_PFD0),
		[A55_PERIPH] = CLK_PATH(a55_periph, 0, SYS_PLL_PFD_END,
								333000000, SYS_PLL_PFD0,
								200000000, SYS_PLL_PFD1),
		[A55_CORE] = CLK_PATH(a55_core, 0, SYS_PLL_PFD_END,
								1400000000, SYS_PLL_PFD_END,
								900000000, SYS_PLL_PFD_END),
	},
};

static struct operating_mode system_run_mode_93 = {
	.paths = {
		[M33_ROOT] = CLK_PATH(m33_root, 250000000, SYS_PLL_PFD0_DIV2,
								200000000, SYS_PLL_PFD1_DIV2,
								133000000, SYS_PLL_PFD1_DIV2),
		/*
		 * the parent of wakeup axi clock in OD mode depends
		 * on its initial rate, here it is temporarily set
		 * to SYS_PLL_PFD_END
		 */
		[WAKEUP_AXI] = CLK_PATH(wakeup_axi, 400000000, SYS_PLL_PFD_END,
								312500000, SYS_PLL_PFD2,
								200000000, SYS_PLL_PFD1),
		[MEDIA_AXI] = CLK_PATH(media_axi, 400000000, SYS_PLL_PFD1,
								333000000, SYS_PLL_PFD0,
								200000000, SYS_PLL_PFD1),
		[MEDIA_APB] = CLK_PATH(media_apb, 133000000, SYS_PLL_PFD1_DIV2,
								125000000, SYS_PLL_PFD0_DIV2,
								133000000, SYS_PLL_PFD1_DIV2),
		[ML_AXI] = CLK_PATH(ml_axi, 1000000000, SYS_PLL_PFD0,
								800000000, SYS_PLL_PFD1,
								500000000, SYS_PLL_PFD0),
		[NIC_AXI] = CLK_PATH(nic_axi, 500000000, SYS_PLL_PFD0,
								400000000, SYS_PLL_PFD1,
								250000000, SYS_PLL_PFD0),
		[A55_PERIPH] = CLK_PATH(a55_periph, 400000000, SYS_PLL_PFD1,
								333000000, SYS_PLL_PFD0,
								200000000, SYS_PLL_PFD1),
		[A55_CORE] = CLK_PATH(a55_core, 1700000000, SYS_PLL_PFD_END,
								1400000000, SYS_PLL_PFD_END,
								900000000, SYS_PLL_PFD_END),
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
static void *se_data;
DEFINE_MUTEX(mode_mutex);

struct lpm_ctx {
	unsigned int level;
	struct thermal_cooling_device *cdev;
};

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

static void lpm_update_clk(struct critical_clk_path *path,
			   enum clk_path_index clk, enum mode_type mode)
{
	if (mode == SWFFC_MODE)
		mode = LD_MODE;

	if (path[clk].mode_parent[mode] != SYS_PLL_PFD_END)
		clk_set_parent(path[clk].clk,
			       clks[path[clk].mode_parent[mode]].clk);

	clk_set_rate(path[clk].clk, path[clk].mode_rate[mode]);
}

/* update all clocks except except_clk */
static void lpm_update_all_clks(struct critical_clk_path *path,
				enum mode_type mode, enum clk_path_index except_clk)
{
	int i;

	for (i = 0; i < CLK_PATH_END; i++) {
		if (i == except_clk)
			continue;

		lpm_update_clk(path, i, mode);
	}
}

/* Caller should hold mode_mutex lock */
static void sys_freq_scaling(enum mode_type new_mode)
{
	struct critical_clk_path *path = system_run_mode.paths;

	if (new_mode == system_run_mode.current_mode) {
		pr_debug("System already in target mode, do nothing\n");
		return;
	}

	if (new_mode == OD_MODE) {
		/* increase the voltage first */
		imx_se_voltage_change_req(se_data, true);
		regulator_set_voltage_tol(soc_reg, VDD_SOC_OD_VOLTAGE, 0);
		imx_se_voltage_change_req(se_data, false);

		/* Increase the NIC_AXI first */
		lpm_update_clk(path, NIC_AXI, new_mode);
		lpm_update_all_clks(path, new_mode, NIC_AXI);

		/* Scaling up the DDR frequency */
		scaling_dram_freq(0x0);
		pr_info("System switching to OD mode...\n");
	} else if (new_mode == ND_MODE) {
		/*
		 * if switch from LD mode to ND mode, voltage should be increase firstly.
		 */
		if (system_run_mode.current_mode == LD_MODE || no_od_mode) {
			regulator_set_voltage_tol(soc_reg, VDD_SOC_ND_VOLTAGE, 0);
			lpm_update_clk(path, NIC_AXI, new_mode);
		}

		lpm_update_all_clks(path, new_mode, NIC_AXI);

		/* Scaling down the ddr frequency. */
		scaling_dram_freq(no_od_mode ? 0x0 : 0x1);

		if (system_run_mode.current_mode != LD_MODE && !no_od_mode) {
			lpm_update_clk(path, NIC_AXI, new_mode);
			regulator_set_voltage_tol(soc_reg, VDD_SOC_ND_VOLTAGE, 0);
		}

		pr_info("System switching to ND mode...\n");
	} else if (new_mode == LD_MODE || new_mode == SWFFC_MODE) {
		/*
		 * NIC AXI frequency should be changed after all other clock
		 * has been slow down. Especially NIC AXI should be reduced
		 * after A55 related clocks.
		 */
		lpm_update_all_clks(path, new_mode, NIC_AXI);
		lpm_update_clk(path, NIC_AXI, new_mode);

		/* Scaling down the ddr frequency. */
		scaling_dram_freq(new_mode == LD_MODE ? 0x1 : 0x2);

		if (!no_od_mode)
			imx_se_voltage_change_req(se_data, true);

		regulator_set_voltage_tol(soc_reg, VDD_SOC_LD_VOLTAGE, 0);

		if (!no_od_mode)
			imx_se_voltage_change_req(se_data, false);

		pr_info("System switching to LD/SWFFC mode...\n");
	}

	system_run_mode.current_mode = new_mode;
}

/* Caller should hold mode_mutex lock */
static void lpm_switch_to_new_mode(enum mode_type new_mode)
{
	/* Skip if set to the same mode */
	if (new_mode == system_run_mode.current_mode)
		return;

	/* make sure auto clock gating is disabled before DDR frequency scaling */
	regmap_update_bits(regmap, AUTO_CG_CTRL, AUTO_CG_EN, 0);

	if ((system_run_mode.current_mode != OD_MODE && new_mode == SWFFC_MODE) ||
	    (system_run_mode.current_mode == SWFFC_MODE && new_mode != OD_MODE))
		sys_freq_scaling(no_od_mode ? ND_MODE : OD_MODE);

	sys_freq_scaling(new_mode);

	if (system_run_mode.auto_gate_enabled ||
	    (system_run_mode.current_mode == OD_MODE && fsp_table[0] >= 3733))
		regmap_update_bits(regmap, AUTO_CG_CTRL, AUTO_CG_EN, AUTO_CG_EN);
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

/* Caller should hold mode_mutex lock */
static enum mode_type lpm_get_tartget_mode(void)
{
	enum mode_type new_mode;

	if (system_run_mode.suspend_prepared) {
		new_mode = no_od_mode ? ND_MODE : OD_MODE;
	} else if (!system_run_mode.cooling_actived) {
		new_mode = system_run_mode.manual_mode;
	} else {
		new_mode = ld_mode_enabled ? LD_MODE : ND_MODE;

		/*
		 * manual setting mode is preferred if it makes the
		 * system colder
		 */
		if (new_mode < system_run_mode.manual_mode)
			new_mode = system_run_mode.manual_mode;
	}

	return new_mode;
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

	mutex_lock(&mode_mutex);

	system_run_mode.manual_mode = new_mode;

	lpm_switch_to_new_mode(lpm_get_tartget_mode());

	mutex_unlock(&mode_mutex);

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
		mutex_lock(&mode_mutex);
		system_run_mode.suspend_prepared = true;
		/* make sure auto clock gating is disabled */
		regmap_update_bits(regmap, AUTO_CG_CTRL, AUTO_CG_EN, 0);
		sys_freq_scaling(lpm_get_tartget_mode());
		/* save the ssi idle strap */
		regmap_read(regmap, AUTO_CG_CTRL, &system_run_mode.ssi_strap);
		mutex_unlock(&mode_mutex);
	} else if (event == PM_POST_SUSPEND) {
		mutex_lock(&mode_mutex);
		system_run_mode.suspend_prepared = false;
		sys_freq_scaling(lpm_get_tartget_mode());

		/* restore the ssi idle strap */
		regmap_update_bits(regmap, AUTO_CG_CTRL, 0xFFFF, system_run_mode.ssi_strap);
		if (system_run_mode.auto_gate_enabled ||
		    (system_run_mode.current_mode == OD_MODE && fsp_table[0] >= 3733)) {
			/* enable the auto clock gating */
			regmap_update_bits(regmap, AUTO_CG_CTRL, HWFFC_ACG_FORCE_B | AUTO_CG_EN,
				 HWFFC_ACG_FORCE_B | AUTO_CG_EN);
		}
		mutex_unlock(&mode_mutex);
	}

	return NOTIFY_OK;
}

static struct notifier_block imx93_lpm_pm_notifier = {
	.notifier_call = imx93_lpm_pm_notify,
};

static int lpm_get_max_state(struct thermal_cooling_device *cdev,
			     unsigned long *state)
{
	*state = MAX_COOLING_LEVEL;

	return 0;
}

static int lpm_get_cur_state(struct thermal_cooling_device *cdev,
			     unsigned long *state)
{
	struct lpm_ctx *ctx = cdev->devdata;

	*state = ctx->level;

	return 0;
}

static int lpm_set_cur_state(struct thermal_cooling_device *cdev,
			     unsigned long state)
{
	struct lpm_ctx *ctx = cdev->devdata;

	if (state > MAX_COOLING_LEVEL)
		return -EINVAL;

	if (state == ctx->level)
		return 0;

	mutex_lock(&mode_mutex);

	if (state == 0) {
		system_run_mode.cooling_actived = false;
	/* cool down. */
	} else if (state == 1) {
		system_run_mode.cooling_actived = true;
	} else {
		dev_err(&cdev->device, "Unsupported cooling level: %lu\n", state);
		return -EINVAL;
	}

	lpm_switch_to_new_mode(lpm_get_tartget_mode());

	ctx->level = state;

	mutex_unlock(&mode_mutex);

	return 0;
}

static const struct thermal_cooling_device_ops lpm_cooling_ops = {
	.get_max_state = lpm_get_max_state,
	.get_cur_state = lpm_get_cur_state,
	.set_cur_state = lpm_set_cur_state,
};

static int lpm_cooling_device_register(struct platform_device *pdev)
{
	int ret;
	struct thermal_cooling_device *cdev;
	struct device *dev = &pdev->dev;
	struct lpm_ctx *ctx;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	platform_set_drvdata(pdev, ctx);

	cdev = devm_thermal_of_cooling_device_register(dev, dev->of_node,
						       "lpm-cooling",
						       ctx,
						       &lpm_cooling_ops);
	if (IS_ERR(cdev)) {
		ret = PTR_ERR(cdev);
		dev_err(dev, "Failed to register lpm cooling device: %d\n",
			ret);
		return ret;
	}
	ctx->cdev = cdev;

	return 0;
}

/* sysfs for user control */
static int imx93_lpm_probe(struct platform_device *pdev)
{
	int i, err;
	struct arm_smccc_res res;
	struct critical_clk_path *path;

	if (of_machine_is_compatible("fsl,imx93"))
		system_run_mode = system_run_mode_93;
	else
		system_run_mode = system_run_mode_91;

	path = system_run_mode.paths;

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

	se_data = imx_get_se_data_info(SOC_ID_OF_IMX93, 0);
	if (!se_data) {
		dev_err(&pdev->dev, "get se-fw2 failed\n");
		return -ENODEV;
	}

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

	if (path[WAKEUP_AXI].initial_rate > 312500000)
		path[WAKEUP_AXI].mode_parent[OD_MODE] = SYS_PLL_PFD1;
	else
		path[WAKEUP_AXI].mode_parent[OD_MODE] = SYS_PLL_PFD2;

	err = clk_bulk_get(&pdev->dev, 6, clks);
	if (err) {
		dev_err(&pdev->dev, "failed to get bulk clks\n");
		return err;
	}

	/* Normally, we assuming the system in boot up in OD or ND(i.MX91/P) mode */
	system_run_mode.current_mode = no_od_mode ? ND_MODE : OD_MODE;
	system_run_mode.manual_mode = system_run_mode.current_mode;

	lpm_cooling_device_register(pdev);

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
