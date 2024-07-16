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

static const unsigned long long axi_pwmgen_scales[] = {
	[PWM_UNIT_SEC]  = 1000000000000ULL,
	[PWM_UNIT_MSEC] = 1000000000ULL,
	[PWM_UNIT_USEC] = 1000000ULL,
	[PWM_UNIT_NSEC] = 1000ULL,
	[PWM_UNIT_PSEC] = 1ULL,
};

struct axi_pwmgen {
	struct pwm_chip		chip;
	struct clk		*clk;
	void __iomem		*base;
	u8			hw_maj_ver;

	/* Used to store the period when the channel is disabled */
	unsigned int		ch_period[AXI_PWMGEN_N_MAX_PWMS];
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
	unsigned long long rate, clk_period_ps, target, cnt;
	unsigned int ch = device->hwpwm;
	struct axi_pwmgen *pwm;

	pwm = to_axi_pwmgen(chip);
	rate = clk_get_rate(pwm->clk);
	clk_period_ps = DIV_ROUND_CLOSEST_ULL(AXI_PWMGEN_PSEC_PER_SEC, rate);

	target = state->period * axi_pwmgen_scales[state->time_unit];
	cnt = target ? DIV_ROUND_CLOSEST_ULL(target, clk_period_ps) : 0;
	pwm->ch_period[ch] = cnt;
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_PERIOD(pwm, ch),
			 state->enabled ? pwm->ch_period[ch] : 0);

	target = state->duty_cycle * axi_pwmgen_scales[state->time_unit];
	cnt = target ? DIV_ROUND_CLOSEST_ULL(target, clk_period_ps) : 0;
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_DUTY(pwm, ch), cnt);

	target = state->phase * axi_pwmgen_scales[state->time_unit];
	cnt = target ? DIV_ROUND_CLOSEST_ULL(target, clk_period_ps) : 0;
	axi_pwmgen_write(pwm, AXI_PWMGEN_CHX_PHASE(pwm, ch), cnt);

	/* Apply the new config */
	axi_pwmgen_write(pwm, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_LOAD_CONIG);
	device->state.time_unit = state->time_unit;

	return 0;
}

static void axi_pwmgen_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct axi_pwmgen *pwmgen = to_axi_pwmgen(chip);
	unsigned long long rate, cnt, clk_period_ps;
	unsigned int ch = pwm->hwpwm;

	rate = clk_get_rate(pwmgen->clk);
	if (!rate)
		return;

	clk_period_ps = DIV_ROUND_CLOSEST_ULL(AXI_PWMGEN_PSEC_PER_SEC, rate);
	cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_PERIOD(pwmgen, ch));
	cnt *= clk_period_ps;
	if (cnt)
		state->period = DIV_ROUND_CLOSEST_ULL(cnt, PSEC_PER_NSEC);
	else
		state->period = 0;
	cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_DUTY(pwmgen, ch));
	cnt *= clk_period_ps;
	if (cnt)
		state->duty_cycle = DIV_ROUND_CLOSEST_ULL(cnt, PSEC_PER_NSEC);
	else
		state->duty_cycle = 0;
	cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_PHASE(pwmgen, ch));
	cnt *= clk_period_ps;
	if (cnt)
		state->phase = DIV_ROUND_CLOSEST_ULL(cnt, PSEC_PER_NSEC);
	else
		state->phase = 0;

	state->enabled = state->period > 0;
	state->time_unit = PWM_UNIT_NSEC;
}

static const struct pwm_ops axi_pwmgen_pwm_ops = {
	.apply = axi_pwmgen_apply,
	.get_state = axi_pwmgen_get_state,
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

	reg = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_CORE_VERSION);
	pwm->hw_maj_ver = AXI_PWMGEN_VERSION_MAJOR(reg);

	if (pwm->hw_maj_ver != 1 && pwm->hw_maj_ver != 2) {
		dev_err(chip->dev, "Unsupported peripheral version %u.%u.%u\n",
			AXI_PWMGEN_VERSION_MAJOR(reg),
			AXI_PWMGEN_VERSION_MINOR(reg),
			AXI_PWMGEN_VERSION_PATCH(reg));
		return -ENODEV;
	}

	pwm->chip.npwm = axi_pwmgen_read(pwm, AXI_PWMGEN_REG_NPWM);
	if (pwm->chip.npwm > AXI_PWMGEN_N_MAX_PWMS)
		return -EINVAL;

	/* Disable all the outputs */
	for (idx = 0; idx < pwm->chip.npwm; idx++) {
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

	return devm_pwmchip_add(&pdev->dev, &pwm->chip);
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
