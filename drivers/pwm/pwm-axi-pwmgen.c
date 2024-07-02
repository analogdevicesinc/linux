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

#define AXI_PWMGEN_VERSION_MAJOR(x)	(((x) >> 16) & 0xff)
#define AXI_PWMGEN_VERSION_MINOR(x)	(((x) >> 8) & 0xff)
#define AXI_PWMGEN_VERSION_PATCH(x)	((x) & 0xff)

#define AXI_PWMGEN_REG_CORE_VERSION	0x00
#define AXI_PWMGEN_REG_ID		0x04
#define AXI_PWMGEN_REG_SCRATCHPAD	0x08
#define AXI_PWMGEN_REG_CORE_MAGIC	0x0C
#define AXI_PWMGEN_REG_CONFIG		0x10
#define AXI_PWMGEN_REG_NPWM		0x14
/* register layout is a bit different between v1 and v2 HDL */
#define AXI_PWMGEN_V1_CHX_PERIOD(ch)	(0x40 + 12 * (ch))
#define AXI_PWMGEN_V1_CHX_DUTY(ch)	(0x44 + 12 * (ch))
#define AXI_PWMGEN_V1_CHX_PHASE(ch)	(0x48 + 12 * (ch))
#define AXI_PWMGEN_V2_CHX_PERIOD(ch)	(0x40 + 4 * (ch))
#define AXI_PWMGEN_V2_CHX_DUTY(ch)	(0x80 + 4 * (ch))
#define AXI_PWMGEN_V2_CHX_PHASE(ch)	(0xC0 + 4 * (ch))
#define AXI_PWMGEN_CHX_PERIOD(p, ch) \
	((p)->hw_maj_ver == 1 ? AXI_PWMGEN_V1_CHX_PERIOD(ch) : AXI_PWMGEN_V2_CHX_PERIOD(ch))
#define AXI_PWMGEN_CHX_DUTY(p, ch) \
	((p)->hw_maj_ver == 1 ? AXI_PWMGEN_V1_CHX_DUTY(ch) : AXI_PWMGEN_V2_CHX_DUTY(ch))
#define AXI_PWMGEN_CHX_PHASE(p, ch) \
	((p)->hw_maj_ver == 1 ? AXI_PWMGEN_V1_CHX_PHASE(ch) : AXI_PWMGEN_V2_CHX_PHASE(ch))
#define AXI_PWMGEN_TEST_DATA		0x5A0F0081
#define AXI_PWMGEN_LOAD_CONIG		BIT(1)
#define AXI_PWMGEN_RESET		BIT(0)

#define AXI_PWMGEN_PSEC_PER_SEC		1000000000000ULL
#define AXI_PWMGEN_N_MAX_PWMS		16

static const unsigned long long axi_pwmgen_scale = 1000ULL; // old PWM_UNIT_NSEC

struct axi_pwmgen {
	struct clk		*clk;
	void __iomem		*base;
	u8			hw_maj_ver;

	/* Used to store the period when the channel is disabled */
	unsigned int		ch_period[AXI_PWMGEN_N_MAX_PWMS];
};

static inline struct axi_pwmgen *axi_pwmgen_from_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

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

static int axi_pwmgen_apply(struct pwm_chip *chip, struct pwm_device *device,
			     const struct pwm_state *state)
{
	unsigned long long rate, clk_period_ps, target, cnt;
	unsigned int ch = device->hwpwm;
	struct axi_pwmgen *pwm;

	pwm = axi_pwmgen_from_chip(chip);
	rate = clk_get_rate(pwm->clk);
	clk_period_ps = DIV_ROUND_CLOSEST_ULL(AXI_PWMGEN_PSEC_PER_SEC, rate);

	target = state->period * axi_pwmgen_scale;
	cnt = target ? DIV_ROUND_CLOSEST_ULL(target, clk_period_ps) : 0;
	pwm->ch_period[ch] = cnt;
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_PERIOD(pwm, ch),
			 state->enabled ? pwm->ch_period[ch] : 0);

	target = state->duty_cycle * axi_pwmgen_scale;
	cnt = target ? DIV_ROUND_CLOSEST_ULL(target, clk_period_ps) : 0;
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_DUTY(pwm, ch), cnt);

	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_PHASE(pwm, ch), 0);

	/* Apply the new config */
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_LOAD_CONIG);

	return 0;
}

static int axi_pwmgen_capture(struct pwm_chip *chip, struct pwm_device *device,
			      struct pwm_capture *capture,
			      unsigned long timeout __always_unused)
{
	struct axi_pwmgen *pwmgen = axi_pwmgen_from_chip(chip);
	unsigned long long rate, cnt, clk_period_ps;
	unsigned int ch = device->hwpwm;

	rate = clk_get_rate(pwmgen->clk);
	if (!rate)
		return -EINVAL;

	clk_period_ps = DIV_ROUND_CLOSEST_ULL(AXI_PWMGEN_PSEC_PER_SEC, rate);
	cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_PERIOD(pwmgen, ch));
	cnt *= clk_period_ps;
	if (cnt)
		capture->period = DIV_ROUND_CLOSEST_ULL(cnt,
				axi_pwmgen_scale);
	else
		capture->period = 0;
	cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_DUTY(pwmgen, ch));
	cnt *= clk_period_ps;
	if (cnt)
		capture->duty_cycle = DIV_ROUND_CLOSEST_ULL(cnt,
				axi_pwmgen_scale);
	else
		capture->duty_cycle = 0;

	return 0;
}

static int axi_pwmgen_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct pwm_capture capture;
	int ret;

	ret = axi_pwmgen_capture(chip, pwm, &capture, 0);
	if (ret < 0)
		return ret;

	state->enabled = state;
	state->period = capture.period;
	state->duty_cycle = capture.duty_cycle;

	return 0;
}

static const struct pwm_ops axi_pwmgen_pwm_ops = {
	.apply = axi_pwmgen_apply,
	.capture = axi_pwmgen_capture,
	.get_state = axi_pwmgen_get_state,
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

	pwm = axi_pwmgen_from_chip(chip);
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_SCRATCHPAD, AXI_PWMGEN_TEST_DATA);
	reg = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_SCRATCHPAD);
	if (reg != AXI_PWMGEN_TEST_DATA) {
		dev_err(&chip->dev, "failed to access the device registers\n");
		return -EIO;
	}

	reg = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_CORE_VERSION);
	pwm->hw_maj_ver = AXI_PWMGEN_VERSION_MAJOR(reg);

	if (pwm->hw_maj_ver != 1 && pwm->hw_maj_ver != 2) {
		dev_err(&chip->dev, "Unsupported peripheral version %u.%u.%u\n",
			AXI_PWMGEN_VERSION_MAJOR(reg),
			AXI_PWMGEN_VERSION_MINOR(reg),
			AXI_PWMGEN_VERSION_PATCH(reg));
		return -ENODEV;
	}

	chip->npwm = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_NPWM);
	if (chip->npwm > AXI_PWMGEN_N_MAX_PWMS)
		return -EINVAL;

	/* Disable all the outputs */
	for (idx = 0; idx < chip->npwm; idx++) {
		axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_PERIOD(pwm, idx), 0);
		axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_DUTY(pwm, idx), 0);
		axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_PHASE(pwm, idx), 0);
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
	struct pwm_chip *chip;
	struct resource *mem;
	int ret;

	chip = devm_pwmchip_alloc(&pdev->dev, 1, sizeof(*pwm));
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	pwm = axi_pwmgen_from_chip(chip);

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

	chip->ops = &axi_pwmgen_pwm_ops;

	ret = axi_pwmgen_setup(chip);
	if (ret < 0)
		return ret;

	return devm_pwmchip_add(&pdev->dev, chip);
}

static struct platform_driver axi_pwmgen_driver = {
	.driver = {
		.name = "adi,axi-pwmgen",
		.of_match_table = axi_pwmgen_ids,
	},
	.probe = axi_pwmgen_probe,
};
module_platform_driver(axi_pwmgen_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog>");
MODULE_DESCRIPTION("Driver for the Analog Devices AXI PWM generator");
