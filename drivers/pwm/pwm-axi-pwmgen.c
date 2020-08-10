// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AXI PWM generator
 *
 * Copyright 2020 Analog Devices Inc.
 */
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#define AXI_PWMGEN_REG_CORE_VERSION	0x00
#define AXI_PWMGEN_REG_ID		0x04
#define AXI_PWMGEN_REG_SCRATCHPAD	0x08
#define AXI_PWMGEN_REG_CORE_MAGIC	0x0C
#define AXI_PWMGEN_REG_CONFIG		0x10
#define AXI_PWMGEN_REG_PULSE_PERIOD	0x14
#define AXI_PWMGEN_REG_PULSE_WIDTH	0x18

#define AXI_PWMGEN_TEST_DATA		0x5A0F0081
#define AXI_PWMGEN_LOAD_CONIG		BIT(1)
#define AXI_PWMGEN_RESET		BIT(0)

struct axi_pwmgen {
	struct pwm_chip		chip;
	struct clk		*clk;
	void __iomem		*base;
	unsigned int		pwm_period;
	unsigned int		pwm_width;
};

static inline unsigned int axi_pwmgen_read(struct axi_pwmgen *pwm,
					   unsigned int reg)
{
	return readl(pwm->base + reg);
}

static inline void axi_pwmgen_write(struct axi_pwmgen *pwm,
				    unsigned int reg,
				    unsigned int value)
{
	writel(value, pwm->base + reg);
}

static inline struct axi_pwmgen *to_axi_pwmgen(struct pwm_chip *chip)
{
	return container_of(chip, struct axi_pwmgen, chip);
}

static int axi_pwmgen_apply(struct pwm_chip *chip, struct pwm_device *device,
			     struct pwm_state *state)
{
	unsigned long tmp, clk_rate, period_cnt, duty_cnt;
	struct axi_pwmgen *pwm;

	pwm = to_axi_pwmgen(chip);
	clk_rate = clk_get_rate(pwm->clk);

	/* Downscale by 1000 in order to avoid overflow when multiplying */
	tmp = (clk_rate / NSEC_PER_USEC) * state->period;
	period_cnt = DIV_ROUND_UP(tmp, USEC_PER_SEC);
	/* The register is 0 based */
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_PULSE_PERIOD, period_cnt - 1);

	/* Downscale by 1000 */
	tmp = (clk_rate / NSEC_PER_USEC) * state->duty_cycle;
	duty_cnt = DIV_ROUND_UP(tmp, USEC_PER_SEC);
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_PULSE_WIDTH, duty_cnt);

	/* Apply the new config */
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_LOAD_CONIG);

	return 0;
}

static const struct pwm_ops axi_pwmgen_pwm_ops = {
	.apply = axi_pwmgen_apply,
	.owner = THIS_MODULE,
};

static const struct of_device_id axi_pwmgen_ids[] = {
	{
		.compatible = "adi,axi-pwmgen",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, axi_pwmgen_ids);

static int axi_pwmgen_setup(struct pwm_chip *chip)
{
	struct axi_pwmgen *pwm;
	unsigned int reg;

	pwm = to_axi_pwmgen(chip);
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_SCRATCHPAD, AXI_PWMGEN_TEST_DATA);
	reg = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_SCRATCHPAD);
	if (reg != AXI_PWMGEN_TEST_DATA) {
		dev_err(chip->dev, "failed to access the device registers\n");
		return -EIO;
	}

	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_RESET);

	return 0;
}

static int axi_pwmgen_probe(struct platform_device *pdev)
{
	struct axi_pwmgen *pwm;
	struct resource *mem;
	int ret;

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pwm->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(pwm->base))
		return PTR_ERR(pwm->base);

	pwm->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pwm->clk))
		return PTR_ERR(pwm->clk);

	ret = clk_prepare_enable(pwm->clk);
	if (ret)
		return ret;

	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &axi_pwmgen_pwm_ops;
	pwm->chip.npwm = 2;
	pwm->chip.base = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_ID);

	ret = axi_pwmgen_setup(&pwm->chip);
	if (ret < 0)
		goto err_clk;

	ret = pwmchip_add(&pwm->chip);
	if (ret)
		goto err_clk;

	platform_set_drvdata(pdev, pwm);

	return 0;

err_clk:
	clk_disable_unprepare(pwm->clk);

	return ret;
}

static int axi_pwmgen_remove(struct platform_device *pdev)
{
	struct axi_pwmgen *pwm = platform_get_drvdata(pdev);
	int ret;

	ret = pwmchip_remove(&pwm->chip);

	clk_disable_unprepare(pwm->clk);

	return ret;
}

static struct platform_driver axi_pwmgen_driver = {
	.driver = {
		.name = "adi-axi-pwmgen",
		.of_match_table = axi_pwmgen_ids,
	},
	.probe = axi_pwmgen_probe,
	.remove = axi_pwmgen_remove,
};

module_platform_driver(axi_pwmgen_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog>");
MODULE_DESCRIPTION("Driver for the Analog Devices AXI PWM generator");
