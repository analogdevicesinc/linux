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
#define AXI_PWMGEN_REG_NPWM		0x14
#define AXI_PWMGEN_CH_PERIOD_BASE	0x40
#define AXI_PWMGEN_CH_DUTY_BASE		0x44
#define AXI_PWMGEN_CH_OFFSET_BASE	0x48
#define AXI_PWMGEN_CHX_PERIOD(ch)	(AXI_PWMGEN_CH_PERIOD_BASE + (12 * (ch)))
#define AXI_PWMGEN_CHX_DUTY(ch)		(AXI_PWMGEN_CH_DUTY_BASE + (12 * (ch)))
#define AXI_PWMGEN_CHX_OFFSET(ch)	(AXI_PWMGEN_CH_OFFSET_BASE + (12 * (ch)))
#define AXI_PWMGEN_TEST_DATA		0x5A0F0081
#define AXI_PWMGEN_LOAD_CONIG		BIT(1)
#define AXI_PWMGEN_RESET		BIT(0)

struct axi_pwmgen {
	struct pwm_chip		chip;
	struct clk		*clk;
	void __iomem		*base;

	/* Used to store the period when the channel is disabled */
	unsigned int		ch_period[4];
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

static void axi_pwmgen_write_mask(struct axi_pwmgen *pwm,
				  unsigned int reg,
				  unsigned int mask,
				  unsigned int value)
{
	unsigned int temp;

	temp = axi_pwmgen_read(pwm, reg);
	axi_pwmgen_write(pwm, reg, (temp & ~mask) | value);
}

static inline struct axi_pwmgen *to_axi_pwmgen(struct pwm_chip *chip)
{
	return container_of(chip, struct axi_pwmgen, chip);
}

static int axi_pwmgen_apply(struct pwm_chip *chip, struct pwm_device *device,
			     const struct pwm_state *state)
{
	unsigned long tmp, clk_rate, period_cnt, duty_cnt, offset_cnt;
	unsigned int ch = device->hwpwm;
	struct axi_pwmgen *pwm;

	pwm = to_axi_pwmgen(chip);
	clk_rate = clk_get_rate(pwm->clk);

	/* Downscale by 1000 in order to avoid overflow when multiplying */
	tmp = DIV_ROUND_CLOSEST(clk_rate, NSEC_PER_USEC);
	tmp *= state->period;
	period_cnt = DIV_ROUND_UP(tmp, USEC_PER_SEC);
	pwm->ch_period[ch] = period_cnt;
	/* The register is 0 based */
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_PERIOD(ch),
		state->enabled ? (pwm->ch_period[ch] - 1) : 0);

	/* Downscale by 1000 */
	tmp = DIV_ROUND_CLOSEST(clk_rate, NSEC_PER_USEC);
	tmp *= state->duty_cycle;
	duty_cnt = DIV_ROUND_UP(tmp, USEC_PER_SEC);
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_DUTY(ch), duty_cnt);

	tmp = DIV_ROUND_CLOSEST(clk_rate, NSEC_PER_USEC);
	tmp *= state->offset;
	offset_cnt = DIV_ROUND_UP(tmp, USEC_PER_SEC);
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_OFFSET(ch), state->offset ? offset_cnt : 0);

	/* Apply the new config */
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_LOAD_CONIG);

	return 0;
}

static void axi_pwmgen_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	unsigned int ch = pwm->hwpwm;
	struct axi_pwmgen *pwmgen = to_axi_pwmgen(chip);

	axi_pwmgen_write(pwmgen, AXI_PWMGEN_CHX_PERIOD(ch), 0);
	axi_pwmgen_write(pwmgen, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_LOAD_CONIG);
}

static int axi_pwmgen_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	unsigned int ch = pwm->hwpwm;
	struct axi_pwmgen *pwmgen = to_axi_pwmgen(chip);

	axi_pwmgen_write(pwmgen, AXI_PWMGEN_CHX_PERIOD(ch), pwmgen->ch_period[ch]);
	axi_pwmgen_write(pwmgen, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_LOAD_CONIG);

	return 0;
}

static const struct pwm_ops axi_pwmgen_pwm_ops = {
	.apply = axi_pwmgen_apply,
	.disable = axi_pwmgen_disable,
	.enable = axi_pwmgen_enable,
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
	int idx;

	pwm = to_axi_pwmgen(chip);
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_SCRATCHPAD, AXI_PWMGEN_TEST_DATA);
	reg = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_SCRATCHPAD);
	if (reg != AXI_PWMGEN_TEST_DATA) {
		dev_err(chip->dev, "failed to access the device registers\n");
		return -EIO;
	}

	pwm->chip.npwm = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_NPWM);
	if (pwm->chip.npwm > 4)
		return -EINVAL;

	/* Disable all the outputs */
	for (idx = 0; idx < pwm->chip.npwm; idx++) {
		axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_PERIOD(idx), 0);
		axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_DUTY(idx), 0);
		axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_OFFSET(idx), 0);
	}

	/* Enable the core */
	axi_pwmgen_write_mask(pwm, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_RESET, 0);

	return 0;
}

static void axi_pwmgen_clk_disable(void *data)
{
	clk_disable_unprepare(data);
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
	ret = devm_add_action_or_reset(&pdev->dev, axi_pwmgen_clk_disable,
				       pwm->clk);
	if (ret)
		return ret;

	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &axi_pwmgen_pwm_ops;
	pwm->chip.base = -1;

	ret = axi_pwmgen_setup(&pwm->chip);
	if (ret < 0)
		return ret;

	ret = pwmchip_add(&pwm->chip);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, pwm);

	return 0;
}

static int axi_pwmgen_remove(struct platform_device *pdev)
{
	struct axi_pwmgen *pwm = platform_get_drvdata(pdev);

	return pwmchip_remove(&pwm->chip);
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
