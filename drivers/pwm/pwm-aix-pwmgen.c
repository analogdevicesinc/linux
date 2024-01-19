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
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#define AXI_PWMGEN_VERSION_MAJOR(x)	((x >> 16) & 0xff)
#define AXI_PWMGEN_VERSION_MINOR(x)	((x >> 8) & 0xff)
#define AXI_PWMGEN_VERSION_PATCH(x)	(x & 0xff)

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
	bool			ch_enabled[4];
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
	struct axi_pwmgen *pwm = to_axi_pwmgen(chip);
	unsigned long clk_rate = clk_get_rate(pwm->clk);
	unsigned int ch = device->hwpwm;
	u64 period_cnt, duty_cnt;

	period_cnt = DIV_ROUND_UP_ULL(state->period * clk_rate, NSEC_PER_SEC);
	if (period_cnt > UINT_MAX)
		return -EINVAL;

	pwm->ch_period[ch] = period_cnt;
	pwm->ch_enabled[ch] = state->enabled;
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_PERIOD(ch),
			 state->enabled ? period_cnt : 0);

	duty_cnt = DIV_ROUND_UP_ULL(state->duty_cycle * clk_rate, NSEC_PER_SEC);
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_DUTY(ch), duty_cnt);

	/* Apply the new config */
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_LOAD_CONIG);

	return 0;
}

static int axi_pwmgen_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct axi_pwmgen *pwmgen = to_axi_pwmgen(chip);
	unsigned long rate = clk_get_rate(pwmgen->clk);
	size_t ch = pwm->hwpwm;
	unsigned int cnt;

	state->enabled = pwmgen->ch_enabled[ch];

	if (state->enabled) {
		cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_PERIOD(ch));
	} else {
		cnt = pwmgen->ch_period[ch];
	}

	state->period = DIV_ROUND_CLOSEST_ULL((u64)cnt * NSEC_PER_SEC, rate);

	cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_DUTY(ch));
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL((u64)cnt * NSEC_PER_SEC, rate);

	return 0;
}

static const struct pwm_ops axi_pwmgen_pwm_ops = {
	.apply = axi_pwmgen_apply,
	.get_state = axi_pwmgen_get_state,
};

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

	reg = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_CORE_VERSION);
	if (AXI_PWMGEN_VERSION_MAJOR(reg) != 1) {
		dev_err(chip->dev, "Unsupported peripheral version %u.%02u.%c\n",
			AXI_PWMGEN_VERSION_MAJOR(reg),
			AXI_PWMGEN_VERSION_MINOR(reg),
			AXI_PWMGEN_VERSION_PATCH(reg) + 'a');
		return -ENODEV;
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

	pwm->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(pwm->clk))
		return PTR_ERR(pwm->clk);

	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &axi_pwmgen_pwm_ops;
	pwm->chip.base = -1;

	ret = axi_pwmgen_setup(&pwm->chip);
	if (ret < 0)
		return ret;

	return devm_pwmchip_add(&pdev->dev, &pwm->chip);
}

static const struct of_device_id axi_pwmgen_ids[] = {
	{ .compatible = "adi,axi-pwmgen" },
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

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog>");
MODULE_DESCRIPTION("Driver for the Analog Devices AXI PWM generator");
