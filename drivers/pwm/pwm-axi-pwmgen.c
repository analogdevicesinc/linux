// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AXI PWM generator
 *
 * Copyright 2024 Analog Devices Inc.
 * Copyright 2024 Baylibre SAS
 *
 * Device docs: https://analogdevicesinc.github.io/hdl/library/axi_pwm_gen/index.html
 *
 * Limitations:
 * - The writes to registers for period and duty are shadowed until
 *   LOAD_CONFIG is written to AXI_PWMGEN_REG_CONFIG, at which point
 *   they take effect.
 * - Writing LOAD_CONFIG also has the effect of re-synchronizing all
 *   enabled channels, which could cause glitching on other channels. It
 *   is therefore expected that channels are assigned harmonic periods
 *   and all have a single user coordinating this.
 * - Supports normal polarity. Does not support changing polarity.
 * - On disable, the PWM output becomes low (inactive).
 */
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/fpga/adi-axi-common.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/version.h>

#define AXI_PWMGEN_REG_ID		0x04
#define AXI_PWMGEN_REG_SCRATCHPAD	0x08
#define AXI_PWMGEN_REG_CORE_MAGIC	0x0C
#define AXI_PWMGEN_REG_CONFIG		0x10
#define AXI_PWMGEN_REG_NPWM		0x14
#define AXI_PWMGEN_CHX_PERIOD(ch)	(0x40 + (4 * (ch)))
#define AXI_PWMGEN_CHX_DUTY(ch)		(0x80 + (4 * (ch)))
#define AXI_PWMGEN_CHX_OFFSET(ch)	(0xC0 + (4 * (ch)))
#define AXI_PWMGEN_REG_CORE_MAGIC_VAL	0x601A3471 /* Identification number to test during setup */
#define AXI_PWMGEN_LOAD_CONFIG		BIT(1)
#define AXI_PWMGEN_REG_CONFIG_RESET	BIT(0)

/*
 * This driver has some cpp magic to make it work on older and newer kernels.
 * Relevant changes are:
 *   v6.9-rc1~146^2~164 ("pwm: Provide pwmchip_alloc() function and a devm variant of it")
 *   v6.9-rc1~100^2^6   ("clk: Add a devm variant of clk_rate_exclusive_get()")
 *   v6.8-rc1~96^2~14   ("pwm: Make it possible to apply PWM changes in atomic context")
 *   v6.7-rc1~28^2~30   ("pwm: Manage owner assignment implicitly for drivers")
 *   v6.2-rc1~26^2~12   ("pwm: Make .get_state() callback return an error code")
 * Define some cpp symbols for these. This allows to forward port using
 * unifdef(1).
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 9, 0)
# define PWM_MISSING_PWMCHIP_ALLOC
# define MISSING_DEVM_CLK_RATE_EXCLUSIVE_GET
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
# define PWM_MISSING_ATOMIC
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 7, 0)
# define PWM_EXPLICIT_OWNER
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0)
# define PWM_GET_STATE_VOID
#endif

/* Another difference compared to the mainline driver is support for pwm_state::phase. */
#define PWM_HAS_PHASE_SUPPORT

#ifdef MISSING_DEVM_CLK_RATE_EXCLUSIVE_GET
static void devm_clk_rate_exclusive_put(void *data)
{
	struct clk *clk = data;

	clk_rate_exclusive_put(clk);
}

static int devm_clk_rate_exclusive_get(struct device *dev, struct clk *clk)
{
	int ret;

	ret = clk_rate_exclusive_get(clk);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, devm_clk_rate_exclusive_put, clk);
}
#endif

struct axi_pwmgen_ddata {
	struct regmap *regmap;
	unsigned long clk_rate_hz;
#ifdef PWM_MISSING_PWMCHIP_ALLOC
	struct pwm_chip chip;
#endif
};

static const struct regmap_config axi_pwmgen_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0xFC,
};

static struct axi_pwmgen_ddata *axi_pwmgen_ddata_from_chip(struct pwm_chip *chip)
{
#ifdef PWM_MISSING_PWMCHIP_ALLOC
	return container_of(chip, struct axi_pwmgen_ddata, chip);
#else
	return pwmchip_get_drvdata(chip);
#endif
}

static int axi_pwmgen_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct axi_pwmgen_ddata *ddata = axi_pwmgen_ddata_from_chip(chip);
	unsigned int ch = pwm->hwpwm;
	struct regmap *regmap = ddata->regmap;
	u64 period_cnt, duty_cnt;
	int ret;

	if (state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	if (state->enabled) {
#ifdef PWM_HAS_PHASE_SUPPORT
		u64 phase_cnt;

#endif
		period_cnt = mul_u64_u64_div_u64(state->period, ddata->clk_rate_hz, NSEC_PER_SEC);
		if (period_cnt > UINT_MAX)
			period_cnt = UINT_MAX;

		if (period_cnt == 0)
			return -EINVAL;

		ret = regmap_write(regmap, AXI_PWMGEN_CHX_PERIOD(ch), period_cnt);
		if (ret)
			return ret;

		duty_cnt = mul_u64_u64_div_u64(state->duty_cycle, ddata->clk_rate_hz, NSEC_PER_SEC);
		if (duty_cnt > UINT_MAX)
			duty_cnt = UINT_MAX;

		ret = regmap_write(regmap, AXI_PWMGEN_CHX_DUTY(ch), duty_cnt);
		if (ret)
			return ret;
#ifdef PWM_HAS_PHASE_SUPPORT

		phase_cnt = mul_u64_u64_div_u64(state->phase, ddata->clk_rate_hz, NSEC_PER_SEC);
		if (duty_cnt > UINT_MAX)
			duty_cnt = UINT_MAX;

		ret = regmap_write(regmap, AXI_PWMGEN_CHX_OFFSET(ch), phase_cnt);
		if (ret)
			return ret;
#endif
	} else {
		ret = regmap_write(regmap, AXI_PWMGEN_CHX_PERIOD(ch), 0);
		if (ret)
			return ret;

		ret = regmap_write(regmap, AXI_PWMGEN_CHX_DUTY(ch), 0);
		if (ret)
			return ret;
	}

	return regmap_write(regmap, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_LOAD_CONFIG);
}

static
#ifdef PWM_GET_STATE_VOID
void
#else
int
#endif
axi_pwmgen_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct axi_pwmgen_ddata *ddata = axi_pwmgen_ddata_from_chip(chip);
	struct regmap *regmap = ddata->regmap;
	unsigned int ch = pwm->hwpwm;
	u32 cnt;
	int ret;

	ret = regmap_read(regmap, AXI_PWMGEN_CHX_PERIOD(ch), &cnt);
	if (ret)
		return
#ifndef PWM_GET_STATE_VOID
			ret
#endif
			;

	state->enabled = cnt != 0;

	state->period = DIV_ROUND_UP_ULL((u64)cnt * NSEC_PER_SEC, ddata->clk_rate_hz);

	ret = regmap_read(regmap, AXI_PWMGEN_CHX_DUTY(ch), &cnt);
	if (ret)
		return
#ifndef PWM_GET_STATE_VOID
			ret
#endif
			;


	state->duty_cycle = DIV_ROUND_UP_ULL((u64)cnt * NSEC_PER_SEC, ddata->clk_rate_hz);

#ifdef PWM_HAS_PHASE_SUPPORT
	ret = regmap_read(regmap, AXI_PWMGEN_CHX_OFFSET(ch), &cnt);
	if (ret)
		return
#ifndef PWM_GET_STATE_VOID
			ret
#endif
			;

	state->phase = DIV_ROUND_UP_ULL((u64)cnt * NSEC_PER_SEC, ddata->clk_rate_hz);
#endif
	state->polarity = PWM_POLARITY_NORMAL;

	return
#ifndef PWM_GET_STATE_VOID
		0
#endif
		;
}

static const struct pwm_ops axi_pwmgen_pwm_ops = {
	.apply = axi_pwmgen_apply,
	.get_state = axi_pwmgen_get_state,
#ifdef PWM_EXPLICIT_OWNER
	.owner = THIS_MODULE,
#endif
};

static int axi_pwmgen_setup(struct regmap *regmap, struct device *dev)
{
	int ret;
	u32 val;

	ret = regmap_read(regmap, AXI_PWMGEN_REG_CORE_MAGIC, &val);
	if (ret)
		return ret;

	if (val != AXI_PWMGEN_REG_CORE_MAGIC_VAL)
		return dev_err_probe(dev, -ENODEV,
			"failed to read expected value from register: got %08x, expected %08x\n",
			val, AXI_PWMGEN_REG_CORE_MAGIC_VAL);

	ret = regmap_read(regmap, ADI_AXI_REG_VERSION, &val);
	if (ret)
		return ret;

	if (ADI_AXI_PCORE_VER_MAJOR(val) != 2) {
		return dev_err_probe(dev, -ENODEV, "Unsupported peripheral version %u.%u.%u\n",
			ADI_AXI_PCORE_VER_MAJOR(val),
			ADI_AXI_PCORE_VER_MINOR(val),
			ADI_AXI_PCORE_VER_PATCH(val));
	}

	/* Enable the core */
	ret = regmap_clear_bits(regmap, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_REG_CONFIG_RESET);
	if (ret)
		return ret;

	ret = regmap_read(regmap, AXI_PWMGEN_REG_NPWM, &val);
	if (ret)
		return ret;

	/* Return the number of PWMs */
	return val;
}

static int axi_pwmgen_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	struct pwm_chip *chip;
	struct axi_pwmgen_ddata *ddata;
	struct clk *clk;
	void __iomem *io_base;
	int ret;

	io_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(io_base))
		return PTR_ERR(io_base);

	regmap = devm_regmap_init_mmio(dev, io_base, &axi_pwmgen_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to init register map\n");

	ret = axi_pwmgen_setup(regmap, dev);
	if (ret < 0)
		return ret;

#ifdef PWM_MISSING_PWMCHIP_ALLOC
	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;
	chip = &ddata->chip;
	chip->npwm = ret;
	chip->dev = dev;
#else
	chip = devm_pwmchip_alloc(dev, ret, sizeof(*ddata));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ddata = pwmchip_get_drvdata(chip);
#endif
	ddata->regmap = regmap;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock\n");

	ret = devm_clk_rate_exclusive_get(dev, clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get exclusive rate\n");

	ddata->clk_rate_hz = clk_get_rate(clk);
	if (!ddata->clk_rate_hz || ddata->clk_rate_hz > NSEC_PER_SEC)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid clock rate: %lu\n", ddata->clk_rate_hz);

	chip->ops = &axi_pwmgen_pwm_ops;
#ifndef PWM_MISSING_ATOMIC
	chip->atomic = true;
#endif

	ret = devm_pwmchip_add(dev, chip);
	if (ret)
		return dev_err_probe(dev, ret, "could not add PWM chip\n");

	return 0;
}

static const struct of_device_id axi_pwmgen_ids[] = {
	{ .compatible = "adi,axi-pwmgen-2.00.a" },
	{ }
};
MODULE_DEVICE_TABLE(of, axi_pwmgen_ids);

static struct platform_driver axi_pwmgen_driver = {
	.driver = {
		.name = "axi-pwmgen",
		.of_match_table = axi_pwmgen_ids,
	},
	.probe = axi_pwmgen_probe,
};
module_platform_driver(axi_pwmgen_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_AUTHOR("Trevor Gamblin <tgamblin@baylibre.com>");
MODULE_DESCRIPTION("Driver for the Analog Devices AXI PWM generator");
