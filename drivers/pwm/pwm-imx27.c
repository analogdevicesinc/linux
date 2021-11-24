// SPDX-License-Identifier: GPL-2.0
/*
 * simple driver for PWM (Pulse Width Modulator) controller
 *
 * Derived from pxa PWM driver by eric miao <eric.miao@marvell.com>
 *
 * Limitations:
 * - When disabled the output is driven to 0 independent of the configured
 *   polarity.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define MX3_PWMCR			0x00    /* PWM Control Register */
#define MX3_PWMSR			0x04    /* PWM Status Register */
#define MX3_PWMSAR			0x0C    /* PWM Sample Register */
#define MX3_PWMPR			0x10    /* PWM Period Register */
#define MX3_PWMCNR			0x14    /* PWM Counter Register */

#define MX3_PWMCR_FWM			GENMASK(27, 26)
#define MX3_PWMCR_STOPEN		BIT(25)
#define MX3_PWMCR_DOZEN			BIT(24)
#define MX3_PWMCR_WAITEN		BIT(23)
#define MX3_PWMCR_DBGEN			BIT(22)
#define MX3_PWMCR_BCTR			BIT(21)
#define MX3_PWMCR_HCTR			BIT(20)

#define MX3_PWMCR_POUTC			GENMASK(19, 18)
#define MX3_PWMCR_POUTC_NORMAL		0
#define MX3_PWMCR_POUTC_INVERTED	1
#define MX3_PWMCR_POUTC_OFF		2

#define MX3_PWMCR_CLKSRC		GENMASK(17, 16)
#define MX3_PWMCR_CLKSRC_OFF		0
#define MX3_PWMCR_CLKSRC_IPG		1
#define MX3_PWMCR_CLKSRC_IPG_HIGH	2
#define MX3_PWMCR_CLKSRC_IPG_32K	3

#define MX3_PWMCR_PRESCALER		GENMASK(15, 4)

#define MX3_PWMCR_SWR			BIT(3)

#define MX3_PWMCR_REPEAT		GENMASK(2, 1)
#define MX3_PWMCR_REPEAT_1X		0
#define MX3_PWMCR_REPEAT_2X		1
#define MX3_PWMCR_REPEAT_4X		2
#define MX3_PWMCR_REPEAT_8X		3

#define MX3_PWMCR_EN			BIT(0)

#define MX3_PWMSR_FWE			BIT(6)
#define MX3_PWMSR_CMP			BIT(5)
#define MX3_PWMSR_ROV			BIT(4)
#define MX3_PWMSR_FE			BIT(3)

#define MX3_PWMSR_FIFOAV		GENMASK(2, 0)
#define MX3_PWMSR_FIFOAV_EMPTY		0
#define MX3_PWMSR_FIFOAV_1WORD		1
#define MX3_PWMSR_FIFOAV_2WORDS		2
#define MX3_PWMSR_FIFOAV_3WORDS		3
#define MX3_PWMSR_FIFOAV_4WORDS		4

#define MX3_PWMCR_PRESCALER_SET(x)	FIELD_PREP(MX3_PWMCR_PRESCALER, (x) - 1)
#define MX3_PWMCR_PRESCALER_GET(x)	(FIELD_GET(MX3_PWMCR_PRESCALER, \
						   (x)) + 1)

#define MX3_PWM_SWR_LOOP		5

/* PWMPR register value of 0xffff has the same effect as 0xfffe */
#define MX3_PWMPR_MAX			0xfffe

struct pwm_imx27_chip {
	struct clk	*clk_ipg;
	struct clk	*clk_per;
	struct clk	*clk_32k;
	void __iomem	*mmio_base;

	/*
	 * The driver cannot read the current duty cycle from the hardware if
	 * the hardware is disabled. Cache the last programmed duty cycle
	 * value to return in that case.
	 */
	unsigned int duty_cycle;
	spinlock_t lock;
};

static inline struct pwm_imx27_chip *to_pwm_imx27_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int pwm_imx27_clk_prepare_enable(struct pwm_imx27_chip *imx)
{
	int ret;

	if (imx->clk_32k) {
		ret = clk_prepare_enable(imx->clk_32k);
		if (ret)
			goto err1;
	}

	ret = clk_prepare_enable(imx->clk_ipg);
	if (ret)
		goto err2;

	ret = clk_prepare_enable(imx->clk_per);
	if (ret)
		goto err3;

	return 0;
err3:
	clk_disable_unprepare(imx->clk_ipg);
err2:
	if (imx->clk_32k)
		clk_disable_unprepare(imx->clk_32k);
err1:
	return ret;
}

static void pwm_imx27_clk_disable_unprepare(struct pwm_imx27_chip *imx)
{
	clk_disable_unprepare(imx->clk_per);
	clk_disable_unprepare(imx->clk_ipg);
	if (imx->clk_32k)
		clk_disable_unprepare(imx->clk_32k);
}

static int pwm_imx27_get_state(struct pwm_chip *chip,
			       struct pwm_device *pwm, struct pwm_state *state)
{
	struct pwm_imx27_chip *imx = to_pwm_imx27_chip(chip);
	u32 period, prescaler, pwm_clk, val;
	u64 tmp;
	int ret;

	ret = pwm_imx27_clk_prepare_enable(imx);
	if (ret < 0)
		return ret;

	val = readl(imx->mmio_base + MX3_PWMCR);

	if (val & MX3_PWMCR_EN)
		state->enabled = true;
	else
		state->enabled = false;

	switch (FIELD_GET(MX3_PWMCR_POUTC, val)) {
	case MX3_PWMCR_POUTC_NORMAL:
		state->polarity = PWM_POLARITY_NORMAL;
		break;
	case MX3_PWMCR_POUTC_INVERTED:
		state->polarity = PWM_POLARITY_INVERSED;
		break;
	default:
		dev_warn(pwmchip_parent(chip), "can't set polarity, output disconnected");
	}

	prescaler = MX3_PWMCR_PRESCALER_GET(val);
	pwm_clk = clk_get_rate(imx->clk_per);
	val = readl(imx->mmio_base + MX3_PWMPR);
	period = val >= MX3_PWMPR_MAX ? MX3_PWMPR_MAX : val;

	/* PWMOUT (Hz) = PWMCLK / (PWMPR + 2) */
	tmp = NSEC_PER_SEC * (u64)(period + 2) * prescaler;
	state->period = DIV_ROUND_UP_ULL(tmp, pwm_clk);

	/*
	 * PWMSAR can be read only if PWM is enabled. If the PWM is disabled,
	 * use the cached value.
	 */
	if (state->enabled)
		val = readl(imx->mmio_base + MX3_PWMSAR);
	else
		val = imx->duty_cycle;

	tmp = NSEC_PER_SEC * (u64)(val) * prescaler;
	state->duty_cycle = DIV_ROUND_UP_ULL(tmp, pwm_clk);

	pwm_imx27_clk_disable_unprepare(imx);

	return 0;
}

static void pwm_imx27_sw_reset(struct pwm_chip *chip)
{
	struct pwm_imx27_chip *imx = to_pwm_imx27_chip(chip);
	struct device *dev = pwmchip_parent(chip);
	int wait_count = 0;
	u32 cr;

	writel(MX3_PWMCR_SWR, imx->mmio_base + MX3_PWMCR);
	do {
		usleep_range(200, 1000);
		cr = readl(imx->mmio_base + MX3_PWMCR);
	} while ((cr & MX3_PWMCR_SWR) &&
		 (wait_count++ < MX3_PWM_SWR_LOOP));

	if (cr & MX3_PWMCR_SWR)
		dev_warn(dev, "software reset timeout\n");
}

static void pwm_imx27_wait_fifo_slot(struct pwm_chip *chip,
				     struct pwm_device *pwm)
{
	struct pwm_imx27_chip *imx = to_pwm_imx27_chip(chip);
	struct device *dev = pwmchip_parent(chip);
	unsigned int period_ms;
	int fifoav;
	u32 sr;

	sr = readl(imx->mmio_base + MX3_PWMSR);
	fifoav = FIELD_GET(MX3_PWMSR_FIFOAV, sr);
	if (fifoav >= MX3_PWMSR_FIFOAV_3WORDS) {
		period_ms = DIV_ROUND_UP_ULL(pwm->state.period,
					 NSEC_PER_MSEC);
		msleep(period_ms * (fifoav - 2));

		sr = readl(imx->mmio_base + MX3_PWMSR);
		if (fifoav == FIELD_GET(MX3_PWMSR_FIFOAV, sr))
			dev_warn(dev, "there is no free FIFO slot\n");
	}
}

static int pwm_imx27_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	unsigned long period_cycles, duty_cycles, prescale, counter_check, flags;
	struct pwm_imx27_chip *imx = to_pwm_imx27_chip(chip);
	void __iomem *reg_sar = imx->mmio_base + MX3_PWMSAR;
	__force u32 sar_last, sar_current;
	unsigned long long c;
	unsigned long long clkrate;
	int ret;
	u32 cr, timeout = 1000;

	clkrate = clk_get_rate(imx->clk_per);
	c = clkrate * state->period;

	do_div(c, NSEC_PER_SEC);
	period_cycles = c;

	prescale = period_cycles / 0x10000 + 1;

	period_cycles /= prescale;
	c = clkrate * state->duty_cycle;
	do_div(c, NSEC_PER_SEC);
	duty_cycles = c;
	duty_cycles /= prescale;

	/*
	 * according to imx pwm RM, the real period value should be PERIOD
	 * value in PWMPR plus 2.
	 */
	if (period_cycles > 2)
		period_cycles -= 2;
	else
		period_cycles = 0;

	/*
	 * Wait for a free FIFO slot if the PWM is already enabled, and flush
	 * the FIFO if the PWM was disabled and is about to be enabled.
	 */
	if (pwm->state.enabled) {
		pwm_imx27_wait_fifo_slot(chip, pwm);
	} else {
		ret = pwm_imx27_clk_prepare_enable(imx);
		if (ret)
			return ret;

		pwm_imx27_sw_reset(chip);
	}

	/*
	 * This is a limited workaround. When the SAR FIFO is empty, the new
	 * write value will be directly applied to SAR even the current period
	 * is not over.
	 * If the new SAR value is less than the old one, and the counter is
	 * greater than the new SAR value, the current period will not filp
	 * the level. This will result in a pulse with a duty cycle of 100%.
	 * So, writing the current value of the SAR to SAR here before updating
	 * the new SAR value can avoid this issue.
	 *
	 * Add a spin lock and turn off the interrupt to ensure that the
	 * real-time performance can be guaranteed as much as possible when
	 * operating the following operations.
	 *
	 * 1. Add a threshold of 1.5us. If the time T between the read current
	 * count value CNR and the end of the cycle is less than 1.5us, wait
	 * for T to be longer than 1.5us before updating the SAR register.
	 * This is to avoid the situation that when the first SAR is written,
	 * the current cycle just ends and the SAR FIFO that just be written
	 * is emptied again.
	 *
	 * 2. Use __raw_writel() to minimize the interval between two writes to
	 * the SAR register to increase the fastest pwm frequency supported.
	 *
	 * When the PWM period is longer than 2us(or <500KHz), this workaround
	 * can solve this problem.
	 */
	if (duty_cycles < imx->duty_cycle) {
		c = clkrate * 1500;
		do_div(c, NSEC_PER_SEC);
		counter_check = c;
		sar_last = cpu_to_le32(imx->duty_cycle);
		sar_current = cpu_to_le32(duty_cycles);

		spin_lock_irqsave(&imx->lock, flags);
		if (state->period >= 2000) {
			while ((period_cycles -
				readl_relaxed(imx->mmio_base + MX3_PWMCNR))
				< counter_check) {
				if (!--timeout)
					break;
			};
		}
		if (!(MX3_PWMSR_FIFOAV &
		      readl_relaxed(imx->mmio_base + MX3_PWMSR)))
			__raw_writel(sar_last, reg_sar);
		__raw_writel(sar_current, reg_sar);
		spin_unlock_irqrestore(&imx->lock, flags);
	} else
		writel(duty_cycles, imx->mmio_base + MX3_PWMSAR);

	writel(period_cycles, imx->mmio_base + MX3_PWMPR);

	/*
	 * Store the duty cycle for future reference in cases where the
	 * MX3_PWMSAR register can't be read (i.e. when the PWM is disabled).
	 */
	imx->duty_cycle = duty_cycles;

	cr = MX3_PWMCR_PRESCALER_SET(prescale) |
	     MX3_PWMCR_STOPEN | MX3_PWMCR_DOZEN | MX3_PWMCR_WAITEN |
	     FIELD_PREP(MX3_PWMCR_CLKSRC, MX3_PWMCR_CLKSRC_IPG_HIGH) |
	     MX3_PWMCR_DBGEN;

	if (state->polarity == PWM_POLARITY_INVERSED)
		cr |= FIELD_PREP(MX3_PWMCR_POUTC,
				MX3_PWMCR_POUTC_INVERTED);

	if (state->enabled)
		cr |= MX3_PWMCR_EN;

	writel(cr, imx->mmio_base + MX3_PWMCR);

	if (!state->enabled)
		pwm_imx27_clk_disable_unprepare(imx);

	return 0;
}

static const struct pwm_ops pwm_imx27_ops = {
	.apply = pwm_imx27_apply,
	.get_state = pwm_imx27_get_state,
};

static const struct of_device_id pwm_imx27_dt_ids[] = {
	{ .compatible = "fsl,imx27-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pwm_imx27_dt_ids);

static int pwm_imx27_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct pwm_imx27_chip *imx;
	int ret;
	u32 pwmcr;

	chip = devm_pwmchip_alloc(&pdev->dev, 1, sizeof(*imx));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	imx = to_pwm_imx27_chip(chip);

	imx->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(imx->clk_ipg))
		return dev_err_probe(&pdev->dev, PTR_ERR(imx->clk_ipg),
				     "getting ipg clock failed\n");

	imx->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(imx->clk_per))
		return dev_err_probe(&pdev->dev, PTR_ERR(imx->clk_per),
				     "failed to get peripheral clock\n");

	imx->clk_32k = devm_clk_get_optional(&pdev->dev, "32k");
	if (IS_ERR(imx->clk_32k)) {
		dev_err(&pdev->dev, "getting 32k clock failed with %ld\n",
				PTR_ERR(imx->clk_32k));
		return PTR_ERR(imx->clk_32k);
	}

	spin_lock_init(&imx->lock);
	imx->duty_cycle = 0;
	chip->ops = &pwm_imx27_ops;

	imx->mmio_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(imx->mmio_base))
		return PTR_ERR(imx->mmio_base);

	ret = pwm_imx27_clk_prepare_enable(imx);
	if (ret)
		return ret;

	/* keep clks on if pwm is running */
	pwmcr = readl(imx->mmio_base + MX3_PWMCR);
	if (!(pwmcr & MX3_PWMCR_EN))
		pwm_imx27_clk_disable_unprepare(imx);

	return devm_pwmchip_add(&pdev->dev, chip);
}

static struct platform_driver imx_pwm_driver = {
	.driver = {
		.name = "pwm-imx27",
		.of_match_table = pwm_imx27_dt_ids,
	},
	.probe = pwm_imx27_probe,
};
module_platform_driver(imx_pwm_driver);

MODULE_DESCRIPTION("i.MX27 and later i.MX SoCs Pulse Width Modulator driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
