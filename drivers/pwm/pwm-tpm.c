/*
 * Copyright 2017-2018 NXP.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#define TPM_GLOBAL	0x8
#define TPM_SC		0x10
#define TPM_CNT		0x14
#define TPM_MOD		0x18
#define TPM_C0SC	0x20
#define TPM_C0V		0x24

#define SC_CMOD		3
#define SC_CPWMS	BIT(5)
#define MSnB		BIT(5)
#define MSnA		BIT(4)
#define ELSnB		BIT(3)
#define ELSnA		BIT(2)

#define PERIOD_PERIOD_MAX 0x10000
#define PERIOD_DIV_MAX	8

struct tpm_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
	void __iomem *base;
};

static const unsigned int prediv[8] = {
	1, 2, 4, 8, 16, 32, 64, 128
};

#define to_tpm_pwm_chip(_chip)	container_of(_chip, struct tpm_pwm_chip, chip)

static int tpm_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct tpm_pwm_chip *tpm = to_tpm_pwm_chip(chip);
	int ret, val, div = 0;
	unsigned int period_cycles, duty_cycles;
	unsigned long rate;
	u64 c;

	rate = clk_get_rate(tpm->clk);
	/* calculate the period_cycles and duty_cycles */
	while (1) {
		c = rate / prediv[div];
		c = c * period_ns;
		do_div(c, 1000000000);
		if (c < PERIOD_PERIOD_MAX)
			break;
		div++;
		if (div >= 8)
			return -EINVAL;
	}

	/* enable the clock before writing the register */
	if (!pwm_is_enabled(pwm)) {
		ret = clk_prepare_enable(tpm->clk);
		if (ret)
			return ret;
	}

	/* set the pre-scale */
	val = readl(tpm->base + TPM_SC);
	val &= ~0x7;
	val |= div;
	writel(val, tpm->base + TPM_SC);

	period_cycles = c;
	c *= duty_ns;
	do_div(c, period_ns);
	duty_cycles = c;

	writel(period_cycles & 0xffff, tpm->base + TPM_MOD);
	writel(duty_cycles & 0xffff, tpm->base + TPM_C0V + pwm->hwpwm * 0x8);

	/* if pwm is not enabled, disable clk after setting */
	if (!pwm_is_enabled(pwm))
		clk_disable_unprepare(tpm->clk);

	return 0;
}

static int tpm_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct tpm_pwm_chip *tpm = to_tpm_pwm_chip(chip);
	int val;

	clk_prepare_enable(tpm->clk);
	/*
	 * To enable a tpm channel, CPWMS = 0, MSnB:MSnA = 0x0,
	 * for TPM normal polarity ELSnB:ELSnA = 2b'10,
	 * inverse ELSnB:ELSnA = 2b'01
	 */
	val = readl(tpm->base + TPM_C0SC + pwm->hwpwm * 0x8);

	val &= ~(MSnB | MSnA | ELSnB | ELSnA);
	val |= MSnB;
	val |= pwm->state.polarity ? ELSnA : ELSnB;

	writel(val, tpm->base + TPM_C0SC + pwm->hwpwm * 0x8);

	/* start the counter */
	val = readl(tpm->base + TPM_SC);
	val |= 0x1 << SC_CMOD;
	writel(val, tpm->base + TPM_SC);

	return 0;
}

static void tpm_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct tpm_pwm_chip *tpm = to_tpm_pwm_chip(chip);

	clk_disable_unprepare(tpm->clk);
}

static int tpm_pwm_set_polarity(struct pwm_chip *chip,
				    struct pwm_device *pwm,
				    enum pwm_polarity polarity)
{
	struct tpm_pwm_chip *tpm = to_tpm_pwm_chip(chip);
	int ret, val;

	/* enable the clock before writing the register */
	if (!pwm_is_enabled(pwm)) {
		ret = clk_prepare_enable(tpm->clk);
		if (ret)
			return ret;
	}

	val = readl(tpm->base + TPM_C0SC + pwm->hwpwm * 0x8);
	val &= ~(ELSnB | ELSnA);
	val |= pwm->state.polarity ? ELSnA : ELSnB;
	writel(val, tpm->base + TPM_C0SC + pwm->hwpwm * 0x8);

	if (!pwm_is_enabled(pwm))
		clk_disable_unprepare(tpm->clk);

	return  0;
}

static const struct pwm_ops tpm_pwm_ops = {
	.config		= tpm_pwm_config,
	.enable		= tpm_pwm_enable,
	.disable	= tpm_pwm_disable,
	.set_polarity	= tpm_pwm_set_polarity,
	.owner   = THIS_MODULE,
};

static int tpm_pwm_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct tpm_pwm_chip *tpm;
	struct resource *res;
	int ret;

	tpm = devm_kzalloc(&pdev->dev, sizeof(*tpm), GFP_KERNEL);
	if (!tpm)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	tpm->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(tpm->base))
		return PTR_ERR(tpm->base);

	tpm->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(tpm->clk))
		return PTR_ERR(tpm->clk);

	tpm->chip.dev = &pdev->dev;
	tpm->chip.ops = &tpm_pwm_ops;
	tpm->chip.base = -1;
	/*
	 * init the number of pwm in the pwm chip. if no "fsl,pwm-number"
	 * found, init the npwm to 2, as tpm module has at least two pwm channel
	 */
	ret = of_property_read_u32(np, "nxp,pwm-number", &tpm->chip.npwm);
	if (ret < 0) {
		dev_info(&pdev->dev, "default two pwm channel");
		tpm->chip.npwm = 2;
	}

	ret = pwmchip_add(&tpm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add pwm chip %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, tpm);

	return 0;
}

static int tpm_pwm_remove(struct platform_device *pdev)
{
	struct tpm_pwm_chip *tpm = platform_get_drvdata(pdev);

	return pwmchip_remove(&tpm->chip);
}

#ifdef CONFIG_PM_SLEEP
static int tpm_pwm_suspend(struct device *dev)
{
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int tpm_pwm_resume(struct device *dev)
{
	pinctrl_pm_select_default_state(dev);

	return 0;
};
#endif

static SIMPLE_DEV_PM_OPS(tpm_pwm_pm, tpm_pwm_suspend, tpm_pwm_resume);

static const struct of_device_id tpm_pwm_dt_ids[] = {
	{ .compatible = "nxp,tpm-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tpm_pwm_dt_ids);

static struct platform_driver tpm_pwm_driver = {
	.driver = {
		.name = "tpm-pwm",
		.of_match_table = tpm_pwm_dt_ids,
		.pm = &tpm_pwm_pm,
	},
	.probe	= tpm_pwm_probe,
	.remove = tpm_pwm_remove,
};
module_platform_driver(tpm_pwm_driver);

MODULE_ALIAS("platform:tpm-pwm");
MODULE_AUTHOR("Jacky Bai <ping.bai@nxp.com>");
MODULE_DESCRIPTION("NXP TPM PWM Driver");
MODULE_LICENSE("GPL v2");
