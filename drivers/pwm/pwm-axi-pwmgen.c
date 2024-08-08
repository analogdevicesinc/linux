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

#define AXI_PWMGEN_N_MAX_PWMS		16

struct axi_pwmgen {
	struct pwm_chip		chip;
	struct clk		*clk;
	void __iomem		*base;
	u8			hw_maj_ver;
};

static inline unsigned int axi_pwmgen_read(struct axi_pwmgen *pwmgen,
					   unsigned int reg)
{
	return readl(pwmgen->base + reg);
}

static inline void axi_pwmgen_write(struct axi_pwmgen *pwmgen,
				    unsigned int reg,
				    unsigned int value)
{
	writel(value, pwmgen->base + reg);
}

static void axi_pwmgen_write_mask(struct axi_pwmgen *pwmgen,
				  unsigned int reg,
				  unsigned int mask,
				  unsigned int value)
{
	unsigned int temp;

	temp = axi_pwmgen_read(pwmgen, reg);
	axi_pwmgen_write(pwmgen, reg, (temp & ~mask) | value);
}

static inline struct axi_pwmgen *to_axi_pwmgen(struct pwm_chip *chip)
{
	return container_of(chip, struct axi_pwmgen, chip);
}

#ifndef mul_u64_u64_div_u64_roundclosest
static u64 mul_u64_u64_div_u64_roundclosest(u64 a, u64 b, u64 c)
{
	u64 res = mul_u64_u64_div_u64(a, b, c);
	/*
	 * Those multiplications might overflow but after the subtraction the
	 * error cancels out.
	 */
	u64 rem = a * b - c * res;

	if (rem * 2 >= c)
		res += 1;

	return res;
}
#endif

static int axi_pwmgen_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	unsigned long rate;
	unsigned long long cnt;
	unsigned int ch = pwm->hwpwm;
	struct axi_pwmgen *pwmgen = to_axi_pwmgen(chip);

	rate = clk_get_rate(pwmgen->clk);

	cnt = mul_u64_u64_div_u64_roundclosest(state->period, rate, NSEC_PER_SEC);
	if (cnt > U32_MAX)
		cnt = U32_MAX;
	axi_pwmgen_write(pwmgen, AXI_PWMGEN_CHX_PERIOD(pwmgen, ch),
			 state->enabled ? cnt : 0);

	cnt = mul_u64_u64_div_u64_roundclosest(state->duty_cycle, rate, NSEC_PER_SEC);
	if (cnt > U32_MAX)
		cnt = U32_MAX;
	axi_pwmgen_write(pwmgen, AXI_PWMGEN_CHX_DUTY(pwmgen, ch), cnt);

	cnt = mul_u64_u64_div_u64_roundclosest(state->phase, rate, NSEC_PER_SEC);
	if (cnt > U32_MAX)
		cnt = U32_MAX;
	axi_pwmgen_write(pwmgen, AXI_PWMGEN_CHX_PHASE(pwmgen, ch), cnt);

	/* Apply the new config */
	axi_pwmgen_write(pwmgen, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_LOAD_CONIG);

	return 0;
}

static void axi_pwmgen_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct axi_pwmgen *pwmgen = to_axi_pwmgen(chip);
	unsigned long rate;
	unsigned long long cnt;
	unsigned int ch = pwm->hwpwm;

	rate = clk_get_rate(pwmgen->clk);
	if (!rate)
		return;

	cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_PERIOD(pwmgen, ch));
	state->period = DIV_ROUND_CLOSEST_ULL(cnt * NSEC_PER_SEC, rate);

	cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_DUTY(pwmgen, ch));
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL(cnt * NSEC_PER_SEC, rate);

	cnt = axi_pwmgen_read(pwmgen, AXI_PWMGEN_CHX_PHASE(pwmgen, ch));
	state->phase = DIV_ROUND_CLOSEST_ULL(cnt * NSEC_PER_SEC, rate);

	state->enabled = state->period > 0;
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
	struct axi_pwmgen *pwmgen;
	unsigned int reg;
	int idx;

	pwmgen = to_axi_pwmgen(chip);
	axi_pwmgen_write(pwmgen, AXI_PWMGEN_REG_SCRATCHPAD, AXI_PWMGEN_TEST_DATA);
	reg = axi_pwmgen_read(pwmgen, AXI_PWMGEN_REG_SCRATCHPAD);
	if (reg != AXI_PWMGEN_TEST_DATA) {
		dev_err(chip->dev, "failed to access the device registers\n");
		return -EIO;
	}

	reg = axi_pwmgen_read(pwmgen, AXI_PWMGEN_REG_CORE_VERSION);
	pwmgen->hw_maj_ver = AXI_PWMGEN_VERSION_MAJOR(reg);

	if (pwmgen->hw_maj_ver != 1 && pwmgen->hw_maj_ver != 2) {
		dev_err(chip->dev, "Unsupported peripheral version %u.%u.%u\n",
			AXI_PWMGEN_VERSION_MAJOR(reg),
			AXI_PWMGEN_VERSION_MINOR(reg),
			AXI_PWMGEN_VERSION_PATCH(reg));
		return -ENODEV;
	}

	pwmgen->chip.npwm = axi_pwmgen_read(pwmgen, AXI_PWMGEN_REG_NPWM);
	if (pwmgen->chip.npwm > AXI_PWMGEN_N_MAX_PWMS)
		return -EINVAL;

	/* Disable all the outputs */
	for (idx = 0; idx < pwmgen->chip.npwm; idx++) {
		axi_pwmgen_write(pwmgen, AXI_PWMGEN_CHX_PERIOD(pwmgen, idx), 0);
		axi_pwmgen_write(pwmgen, AXI_PWMGEN_CHX_DUTY(pwmgen, idx), 0);
		axi_pwmgen_write(pwmgen, AXI_PWMGEN_CHX_PHASE(pwmgen, idx), 0);
	}

	/* Enable the core */
	axi_pwmgen_write_mask(pwmgen, AXI_PWMGEN_REG_CONFIG, AXI_PWMGEN_RESET, 0);

	return 0;
}

static void axi_pwmgen_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int axi_pwmgen_probe(struct platform_device *pdev)
{
	struct axi_pwmgen *pwmgen;
	struct resource *mem;
	int ret;

	pwmgen = devm_kzalloc(&pdev->dev, sizeof(*pwmgen), GFP_KERNEL);
	if (!pwmgen)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pwmgen->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(pwmgen->base))
		return PTR_ERR(pwmgen->base);

	pwmgen->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pwmgen->clk))
		return PTR_ERR(pwmgen->clk);

	ret = clk_prepare_enable(pwmgen->clk);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, axi_pwmgen_clk_disable,
				       pwmgen->clk);
	if (ret)
		return ret;

	pwmgen->chip.dev = &pdev->dev;
	pwmgen->chip.ops = &axi_pwmgen_pwm_ops;
	pwmgen->chip.base = -1;

	ret = axi_pwmgen_setup(&pwmgen->chip);
	if (ret < 0)
		return ret;

	return devm_pwmchip_add(&pdev->dev, &pwmgen->chip);
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
