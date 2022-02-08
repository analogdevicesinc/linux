// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>

#define FSL_SIP_DDR_DVFS                0xc2000004
#define DDR_DFS_GET_FSP_COUNT		0x10
#define DDR_FSP_HIGH		2
#define DDR_FSP_LOW		1
#define DDR_DFS_FSP_NUM_MIN	3
#define DDR_BYPASS_DRATE	400

static struct clk *dram_sel;
static struct clk *dram_div;
static struct clk *pll4;
static struct clk *frosc;
static struct clk *spll2;
static struct clk *a35_sel;
static struct clk *nic_sel;
static struct clk *lpav_axi_sel;
static struct clk *nic_old_parent;
static struct clk *a35_old_parent;
static struct clk *lpav_old_parent;
static struct clk *pll4;

static bool lpm_enabled = false;
static bool bypass_enabled = false;
static bool sys_dvfs_enabled = false;
static struct device *imx8ulp_lpm_dev;
static int num_fsp;

static int scaling_dram_freq(unsigned int fsp_index)
{
	struct arm_smccc_res res;
	u32 num_cpus = num_online_cpus();

	local_irq_disable();

	/* need to check the return value ?*/ 
	arm_smccc_smc(FSL_SIP_DDR_DVFS, fsp_index, num_cpus,
		sys_dvfs_enabled, 0, 0, 0, 0, &res);

	local_irq_enable();

	/* Correct the clock tree & rate info as it has been updated in TF-A */
	if (fsp_index == DDR_FSP_HIGH) {
		clk_set_parent(dram_sel, pll4);
	} else if (bypass_enabled) {
		/* only need to correct the clock parent/child for bypass mode */
		clk_set_parent(dram_sel, frosc);
	}

	clk_get_rate(dram_div);

	return 0;
}

static void sys_freq_scaling(bool enter)
{
	int ret;
	if (enter) {
		if (sys_dvfs_enabled) {
			/*scaling down APD side NIC frequency, switch to fro 192MHz */
			nic_old_parent = clk_get_parent(nic_sel);
			lpav_old_parent = clk_get_parent(lpav_axi_sel);
			a35_old_parent = clk_get_parent(a35_sel);

			ret = clk_set_parent(nic_sel, frosc);
			if (ret)
				pr_err("failed to change nic clock parent:%d\n", ret);

			ret = clk_set_parent(lpav_axi_sel, frosc);
			if (ret)
				pr_err("failed to change lpav axi clock parent:%d\n", ret);

			/*
			 * scaling down the A35 core frequency, switch to fro 192MHz,
			 * then, change SPLL2 frequency to 500MHz.
			 */
			ret = clk_set_parent(a35_sel, frosc);
			if (ret)
				pr_err("failed to change a35 clock parent:%d\n", ret);

			/* change SPLL2 to UD 500MHz */
			ret = clk_set_rate(spll2, 500000000);
			if (ret)
				pr_err("failed to set spll2 frequency:%d\n", ret);

			/* switch A35 from frosc to spll2 */
			ret = clk_set_parent(a35_sel, a35_old_parent);
			if (ret)
				pr_err("failed to change a35 clock parent back:%d\n", ret);
		}

		/*
		 * scaling down the ddr frequency and change the BUCK3
		 * voltage to ND 1.0V if system level dvfs is enabled.
		 */
		scaling_dram_freq(DDR_FSP_LOW);

		pr_info("DDR enter low frequency mode\n");
	} else {
		/* prepare enable PLL4 first */
		clk_prepare_enable(pll4);
		/*
		 * exit LPM mode, increase the BUCK3 voltage to OD if system level
		 * dvfs enabled, scaling up the DDR frequency
		 */
		scaling_dram_freq(DDR_FSP_HIGH);

		if (sys_dvfs_enabled) {
			ret = clk_set_parent(a35_sel, frosc);
			if (ret)
				pr_err("failed to change a35 clock parent:%d\n", ret);

			/* change SPLL2 to OD 960MHz */
			ret = clk_set_rate(spll2, 960000000);
			if (ret)
				pr_err("failed to set spll2 frequency:%d\n", ret);

			/* switch A35 from frosc to spll2 */
			ret = clk_set_parent(a35_sel, a35_old_parent);
			if (ret)
				pr_err("failed to change a35 clock parent back:%d\n", ret);

			/* scaling up the NIC frequency */
			ret = clk_set_parent(nic_sel, nic_old_parent);
			if (ret)
				pr_err("failed to change nic clock parent:%d\n", ret);

			/* scaling up lpav axi frequency */
			ret = clk_set_parent(lpav_axi_sel, lpav_old_parent);
			if (ret)
				pr_err("failed to change lpav axi clock parent:%d\n", ret);
		}

		/* unprepare pll4 after clock tree info is correct */
		clk_disable_unprepare(pll4);

		pr_info("DDR Exit from low frequency mode\n");
	}
}
static ssize_t lpm_enable_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	if(lpm_enabled)
		return sprintf(buf, "i.MX8ULP LPM mode enabled\n");
	else
		return sprintf(buf, "i.MX8ULP LPM mode disabled\n");
}

static ssize_t lpm_enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	/*
	 * only support DDR DFS between PLL on and PLL bypass, so the valid
	 * num_fsp should be 3
	 */
	if (num_fsp < DDR_DFS_FSP_NUM_MIN)
		pr_info("DDR DFS only support with both F1 & F2 enabled\n");


	if ((strncmp(buf, "1", 1) == 0) && !lpm_enabled) {
		sys_freq_scaling(true);

		lpm_enabled = true;
	} else if (strncmp(buf, "0", 1) == 0) {
		if (lpm_enabled)
			sys_freq_scaling(false);

		lpm_enabled = false;
	}

	return size;
}
static DEVICE_ATTR(enable, 0644, lpm_enable_show,
			lpm_enable_store);

static int imx8ulp_lpm_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	/* if DDR is not in low frequency, return directly */
	if (!lpm_enabled)
		return NOTIFY_OK;

	if (event == PM_SUSPEND_PREPARE)
		sys_freq_scaling(false);
	else if (event == PM_POST_SUSPEND)
		sys_freq_scaling(true);

	return NOTIFY_OK;
}

static struct notifier_block imx8ulp_lpm_pm_notifier = {
	.notifier_call = imx8ulp_lpm_pm_notify,
};

/* sysfs for user control */
static int imx8ulp_lpm_probe(struct platform_device *pdev)
{
	int err;
	struct arm_smccc_res res;

	imx8ulp_lpm_dev = &pdev->dev;

	arm_smccc_smc(FSL_SIP_DDR_DVFS, DDR_DFS_GET_FSP_COUNT, 0,
		0, 0, 0, 0, 0, &res);
	num_fsp = res.a0;
	/* check F1 is bypass or not */
	if (res.a1 <= DDR_BYPASS_DRATE)
		bypass_enabled = true;

	/* only support DFS for F1 & F2 both enabled */
	if (num_fsp != DDR_DFS_FSP_NUM_MIN)
		return -ENODEV;

	/*
	 * check if system level dvfs is enabled, only when this is enabled,
	 * we can do system level frequency scaling and voltage change dynamically
	 */
	sys_dvfs_enabled = of_property_read_bool(pdev->dev.of_node, "sys-dvfs-enabled");

	/* get the necessary clocks */
	dram_sel = devm_clk_get(&pdev->dev, "ddr_sel");
	dram_div = devm_clk_get(&pdev->dev, "ddr_div");
	pll4 = devm_clk_get(&pdev->dev, "pll4");
	frosc = devm_clk_get(&pdev->dev, "frosc");
	/* below clock is used for system level od/nd mode swithing */
	nic_sel = devm_clk_get(&pdev->dev, "nic_sel");
	a35_sel = devm_clk_get(&pdev->dev, "a35_sel");
	spll2 = devm_clk_get(&pdev->dev, "spll2");
	lpav_axi_sel = devm_clk_get(&pdev->dev, "lpav_axi_sel");
	pll4 = devm_clk_get(&pdev->dev, "pll4");
	if (IS_ERR(dram_sel) || IS_ERR(dram_div) || IS_ERR(pll4) || IS_ERR(frosc) ||
	    IS_ERR(nic_sel) || IS_ERR(a35_sel) || IS_ERR(spll2) || IS_ERR(lpav_axi_sel) ||
	    IS_ERR(pll4))
		dev_err(&pdev->dev, "Get clocks failed\n");

	/* create the sysfs file */
	err = sysfs_create_file(&imx8ulp_lpm_dev->kobj, &dev_attr_enable.attr);
	if (err) {
		dev_err(&pdev->dev, "creating i.MX8ULP LPM control sys file\n");
		return err;
	}

	register_pm_notifier(&imx8ulp_lpm_pm_notifier);

	return 0;
}

static const struct of_device_id imx8ulp_lpm_ids[] = {
	{.compatible = "nxp, imx8ulp-lpm", },
	{ /* sentinel */}
};

static struct platform_driver imx8ulp_lpm_driver = {
	.driver = {
		.name = "imx8ulp-lpm",
		.owner = THIS_MODULE,
		.of_match_table = imx8ulp_lpm_ids,
		},
	.probe = imx8ulp_lpm_probe,
};
module_platform_driver(imx8ulp_lpm_driver);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX8ULP Low Power Control driver");
MODULE_LICENSE("GPL");
