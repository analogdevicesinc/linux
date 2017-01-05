/*
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/busfreq-imx.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include "common.h"
#include "hardware.h"

#define MU_ATR0_OFFSET	0x0
#define MU_ARR0_OFFSET	0x10
#define MU_ARR1_OFFSET	0x14
#define MU_ASR		0x20
#define MU_ACR		0x24
#define MX7ULP_MU_TR0	0x20
#define MX7ULP_MU_RR0	0x40
#define MX7ULP_MU_RR1	0x44
#define MX7ULP_MU_SR	0x60
#define MX7ULP_MU_CR	0x64

#define MU_LPM_HANDSHAKE_INDEX		0
#define MU_RPMSG_HANDSHAKE_INDEX	1
#define MU_LPM_BUS_HIGH_READY_FOR_M4	0xFFFF6666
#define MU_LPM_M4_FREQ_CHANGE_READY	0xFFFF7777
#define MU_LPM_M4_REQUEST_HIGH_BUS	0x2222CCCC
#define MU_LPM_M4_RELEASE_HIGH_BUS	0x2222BBBB
#define MU_LPM_M4_WAKEUP_SRC_VAL	0x55555000
#define MU_LPM_M4_WAKEUP_SRC_MASK	0xFFFFF000
#define MU_LPM_M4_WAKEUP_IRQ_MASK	0xFF0
#define MU_LPM_M4_WAKEUP_IRQ_SHIFT	0x4
#define MU_LPM_M4_WAKEUP_ENABLE_MASK	0xF
#define MU_LPM_M4_WAKEUP_ENABLE_SHIFT	0x0

#define MU_LPM_M4_RUN_MODE	        0x5A5A0001
#define MU_LPM_M4_WAIT_MODE	        0x5A5A0002
#define MU_LPM_M4_STOP_MODE	        0x5A5A0003

#define MAX_NUM 10	/* enlarge it if overflow happen */

struct imx_mu_rpmsg_box {
	const char *name;
	struct blocking_notifier_head notifier;
};

static struct imx_mu_rpmsg_box mu_rpmsg_box = {
	.name	= "m4",
};

void __iomem *mu_base;
static u32 m4_message[MAX_NUM];
static u32 in_idx, out_idx;
static struct delayed_work mu_work, rpmsg_work;
static u32 m4_wake_irqs[4];
static bool m4_freq_low;
struct irq_domain *domain;
static bool m4_in_stop;
static struct clk *clk;
static DEFINE_SPINLOCK(mu_lock);

void imx_mu_set_m4_run_mode(void)
{
	m4_in_stop = false;
}

bool imx_mu_is_m4_in_stop(void)
{
	return m4_in_stop;
}

bool imx_mu_is_m4_in_low_freq(void)
{
	return m4_freq_low;
}

void imx_mu_enable_m4_irqs_in_gic(bool enable)
{
	int i, j;

	for (i = 0; i < 4; i++) {
		if (m4_wake_irqs[i] == 0)
			continue;
		for (j = 0; j < 32; j++) {
			if (m4_wake_irqs[i] & (1 << j)) {
				if (enable)
					enable_irq(irq_find_mapping(
						domain, i * 32 + j));
				else
					disable_irq(irq_find_mapping(
						domain, i * 32 + j));
			}
		}
	}
}

static irqreturn_t mcc_m4_dummy_isr(int irq, void *param)
{
	return IRQ_HANDLED;
}

static int imx_mu_send_message(unsigned int index, unsigned int data)
{
	u32 val, ep;
	int i, te_flag = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	/* wait for transfer buffer empty, and no event pending */
	do {
		if (cpu_is_imx7ulp())
			val = readl_relaxed(mu_base + MX7ULP_MU_SR);
		else
			val = readl_relaxed(mu_base + MU_ASR);
		ep = val & BIT(4);
		if (time_after(jiffies, timeout)) {
			pr_err("Waiting MU transmit buffer empty timeout!\n");
			return -EIO;
		}
	} while (((val & (1 << (20 + 3 - index))) == 0) || (ep == BIT(4)));

	if (cpu_is_imx7ulp())
		writel_relaxed(data, mu_base + index * 0x4 + MX7ULP_MU_TR0);
	else
		writel_relaxed(data, mu_base + index * 0x4 + MU_ATR0_OFFSET);

	/*
	 * make a double check that TEn is not empty after write
	 */
	if (cpu_is_imx7ulp())
		val = readl_relaxed(mu_base + MX7ULP_MU_SR);
	else
		val = readl_relaxed(mu_base + MU_ASR);
	ep = val & BIT(4);
	if (((val & (1 << (20 + (3 - index)))) == 0) || (ep == BIT(4)))
		return 0;
	else
		te_flag = 1;

	/*
	 * Make sure that TEn flag is changed, after the ATRn is filled up.
	 */
	for (i = 0; i < 100; i++) {
		if (cpu_is_imx7ulp())
			val = readl_relaxed(mu_base + MX7ULP_MU_SR);
		else
			val = readl_relaxed(mu_base + MU_ASR);
		ep = val & BIT(4);
		if (((val & (1 << (20 + 3 - index))) == 0) || (ep == BIT(4))) {
			/*
			 * BUG here. TEn flag is changes, after the
			 * ATRn is filled with MSG for a while.
			 */
			te_flag = 0;
			break;
		} else if (time_after(jiffies, timeout)) {
			/* Can't see TEn 1->0, maybe already handled! */
			te_flag = 1;
			break;
		}
	}
	if (te_flag == 0)
		pr_info("BUG: TEn is not changed immediately"
				"when ATRn is filled up.\n");

	return 0;
}

static void mu_work_handler(struct work_struct *work)
{
	int ret;
	u32 irq, enable, idx, mask, virq;
	struct of_phandle_args args;
	u32 message;
	unsigned long flags;

	spin_lock_irqsave(&mu_lock, flags);
	message = m4_message[out_idx % MAX_NUM];
	spin_unlock_irqrestore(&mu_lock, flags);

	pr_debug("receive M4 message 0x%x\n", message);

	switch (message) {
	case MU_LPM_M4_RUN_MODE:
	case MU_LPM_M4_WAIT_MODE:
		m4_in_stop = false;
		break;
	case MU_LPM_M4_STOP_MODE:
		m4_in_stop = true;
		break;
	case MU_LPM_M4_REQUEST_HIGH_BUS:
		request_bus_freq(BUS_FREQ_HIGH);
#ifdef CONFIG_SOC_IMX6SX
		if (cpu_is_imx6sx())
			imx6sx_set_m4_highfreq(true);
#endif
		imx_mu_send_message(MU_LPM_HANDSHAKE_INDEX,
			MU_LPM_BUS_HIGH_READY_FOR_M4);
		m4_freq_low = false;
		break;
	case MU_LPM_M4_RELEASE_HIGH_BUS:
		release_bus_freq(BUS_FREQ_HIGH);
#ifdef CONFIG_SOC_IMX6SX
		if (cpu_is_imx6sx()) {
			imx6sx_set_m4_highfreq(false);
			imx_mu_send_message(MU_LPM_HANDSHAKE_INDEX,
				MU_LPM_M4_FREQ_CHANGE_READY);
		}
#endif
		m4_freq_low = true;
		break;
	default:
		if ((message & MU_LPM_M4_WAKEUP_SRC_MASK) ==
			MU_LPM_M4_WAKEUP_SRC_VAL) {
			irq = (message & MU_LPM_M4_WAKEUP_IRQ_MASK) >>
				MU_LPM_M4_WAKEUP_IRQ_SHIFT;

			enable = (message & MU_LPM_M4_WAKEUP_ENABLE_MASK) >>
				MU_LPM_M4_WAKEUP_ENABLE_SHIFT;

			/* to hwirq start from 0 */
			irq -= 32;

			idx = irq / 32;
			mask = 1 << irq % 32;

			args.np = of_find_compatible_node(NULL, NULL, "fsl,imx6sx-gpc");
			args.args_count = 3;
			args.args[0] = 0;
			args.args[1] = irq;
			args.args[2] = IRQ_TYPE_LEVEL_HIGH;

			virq = irq_create_of_mapping(&args);

			if (enable && can_request_irq(virq, 0)) {
				ret = request_irq(virq, mcc_m4_dummy_isr,
					IRQF_NO_SUSPEND, "imx-m4-dummy", NULL);
				if (ret) {
					pr_err("%s: register interrupt %d failed, rc %d\n",
						__func__, virq, ret);
					break;
				}
				disable_irq(virq);
				m4_wake_irqs[idx] = m4_wake_irqs[idx] | mask;
			}
			imx_gpc_add_m4_wake_up_irq(irq, enable);
		}
		break;
	}

	spin_lock_irqsave(&mu_lock, flags);
	m4_message[out_idx % MAX_NUM] = 0;
	out_idx++;
	spin_unlock_irqrestore(&mu_lock, flags);

	/* enable RIE3 interrupt */
	if (cpu_is_imx7ulp())
		writel_relaxed(readl_relaxed(mu_base + MX7ULP_MU_CR) | BIT(27),
			mu_base + MX7ULP_MU_CR);
	else
		writel_relaxed(readl_relaxed(mu_base + MU_ACR) | BIT(27),
			mu_base + MU_ACR);
}

int imx_mu_rpmsg_send(unsigned int rpmsg)
{
	return imx_mu_send_message(MU_RPMSG_HANDSHAKE_INDEX, rpmsg);
}

int imx_mu_lpm_ready(bool ready)
{
	u32 val;

	if (cpu_is_imx7ulp()) {
		val = readl_relaxed(mu_base + MX7ULP_MU_CR);
		if (ready)
			writel_relaxed(val | BIT(0), mu_base + MX7ULP_MU_CR);
		else
			writel_relaxed(val & ~BIT(0), mu_base + MX7ULP_MU_CR);
	} else {
		val = readl_relaxed(mu_base + MU_ACR);
		if (ready)
			writel_relaxed(val | BIT(0), mu_base + MU_ACR);
		else
			writel_relaxed(val & ~BIT(0), mu_base + MU_ACR);
	}
	return 0;
}

int imx_mu_rpmsg_register_nb(const char *name, struct notifier_block *nb)
{
	if ((name == NULL) || (nb == NULL))
		return -EINVAL;

	if (!strcmp(mu_rpmsg_box.name, name))
		blocking_notifier_chain_register(&(mu_rpmsg_box.notifier), nb);
	else
		return -ENOENT;

	return 0;
}

int imx_mu_rpmsg_unregister_nb(const char *name, struct notifier_block *nb)
{
	if ((name == NULL) || (nb == NULL))
		return -EINVAL;

	if (!strcmp(mu_rpmsg_box.name, name))
		blocking_notifier_chain_unregister(&(mu_rpmsg_box.notifier),
				nb);
	else
		return -ENOENT;

	return 0;
}

static void rpmsg_work_handler(struct work_struct *work)
{
	u32 message;
	unsigned long flags;

	spin_lock_irqsave(&mu_lock, flags);
	/* handle all incoming mu message */
	while (in_idx != out_idx) {
		message = m4_message[out_idx % MAX_NUM];
		spin_unlock_irqrestore(&mu_lock, flags);

		blocking_notifier_call_chain(&(mu_rpmsg_box.notifier), 4,
						(void *)message);

		spin_lock_irqsave(&mu_lock, flags);
		m4_message[out_idx % MAX_NUM] = 0;
		out_idx++;
	}
	spin_unlock_irqrestore(&mu_lock, flags);
}

static irqreturn_t imx_mu_isr(int irq, void *param)
{
	u32 irqs;
	unsigned long flags;

	if (cpu_is_imx7ulp())
		irqs = readl_relaxed(mu_base + MX7ULP_MU_SR);
	else
		irqs = readl_relaxed(mu_base + MU_ASR);

	/* RPMSG */
	if (irqs & (1 << 26)) {
		spin_lock_irqsave(&mu_lock, flags);
		/* get message from receive buffer */
		if (cpu_is_imx7ulp())
			m4_message[in_idx % MAX_NUM] = readl_relaxed(mu_base +
						MX7ULP_MU_RR1);
		else
			m4_message[in_idx % MAX_NUM] = readl_relaxed(mu_base +
						MU_ARR1_OFFSET);
		in_idx++;
		/*
		 * Too many mu message not be handled in timely, can enlarge
		 * MAX_NUM */
		if (in_idx == out_idx) {
			spin_unlock_irqrestore(&mu_lock, flags);
			pr_err("MU overflow!\n");
			return IRQ_HANDLED;
		}
		spin_unlock_irqrestore(&mu_lock, flags);

		schedule_delayed_work(&rpmsg_work, 0);
	}

	if (irqs & (1 << 27)) {
		spin_lock_irqsave(&mu_lock, flags);
		/* get message from receive buffer */
		if (cpu_is_imx7ulp())
			m4_message[in_idx % MAX_NUM] = readl_relaxed(mu_base +
						MX7ULP_MU_RR0);
		else
			m4_message[in_idx % MAX_NUM] = readl_relaxed(mu_base +
						MU_ARR0_OFFSET);
		/* disable RIE3 interrupt */
		if (cpu_is_imx7ulp())
			writel_relaxed(readl_relaxed(mu_base + MX7ULP_MU_CR)
					& (~BIT(27)), mu_base + MX7ULP_MU_CR);
		else
			writel_relaxed(readl_relaxed(mu_base + MU_ACR)
					& (~BIT(27)), mu_base + MU_ACR);
		in_idx++;
		if (in_idx == out_idx) {
			spin_unlock_irqrestore(&mu_lock, flags);
			pr_err("MU overflow!\n");
			return IRQ_HANDLED;
		}
		spin_unlock_irqrestore(&mu_lock, flags);

		schedule_delayed_work(&mu_work, 0);
	}

	return IRQ_HANDLED;
}

static int imx_mu_probe(struct platform_device *pdev)
{
	int ret;
	u32 irq;
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6sx-mu");
	mu_base = of_iomap(np, 0);
	WARN_ON(!mu_base);

	ret = of_device_is_compatible(np, "fsl,imx7ulp-mu");
	if (ret)
		irq = platform_get_irq(pdev, 1);
	else
		irq = platform_get_irq(pdev, 0);
	ret = request_irq(irq, imx_mu_isr,
		IRQF_EARLY_RESUME, "imx-mu", &mu_rpmsg_box);
	if (ret) {
		pr_err("%s: register interrupt %d failed, rc %d\n",
			__func__, irq, ret);
		return ret;
	}

	ret = of_device_is_compatible(np, "fsl,imx7d-mu");
	if (ret) {
		clk = devm_clk_get(&pdev->dev, "mu");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev,
				"mu clock source missing or invalid\n");
			return PTR_ERR(clk);
		} else {
			ret = clk_prepare_enable(clk);
			if (ret) {
				dev_err(&pdev->dev,
					"unable to enable mu clock\n");
				return ret;
			}
		}

		/* MU always as a wakeup source for low power mode */
		imx_gpcv2_add_m4_wake_up_irq(irq_to_desc(irq)->irq_data.hwirq,
			true);
	} else {
		/* MU always as a wakeup source for low power mode */
		imx_gpc_add_m4_wake_up_irq(irq_to_desc(irq)->irq_data.hwirq, true);
	}

	INIT_DELAYED_WORK(&mu_work, mu_work_handler);
	INIT_DELAYED_WORK(&rpmsg_work, rpmsg_work_handler);
	/* bit0 of MX7ULP_MU_CR used to let m4 to know MU is ready now */
	if (cpu_is_imx7ulp())
		writel_relaxed(readl_relaxed(mu_base + MX7ULP_MU_CR) |
			BIT(0) | BIT(26) | BIT(27), mu_base + MX7ULP_MU_CR);
	else
		writel_relaxed(readl_relaxed(mu_base + MU_ACR) |
			BIT(26) | BIT(27), mu_base + MU_ACR);
	BLOCKING_INIT_NOTIFIER_HEAD(&(mu_rpmsg_box.notifier));

	pr_info("MU is ready for cross core communication!\n");

	return 0;
}

static const struct of_device_id imx_mu_ids[] = {
	{ .compatible = "fsl,imx6sx-mu" },
	{ .compatible = "fsl,imx7d-mu" },
	{ .compatible = "fsl,imx7ulp-mu" },
	{ }
};

#ifdef CONFIG_PM_SLEEP
static int mu_suspend(struct device *dev)
{
	return 0;
}

static int mu_resume(struct device *dev)
{
	int ret;

	if (!cpu_is_imx7ulp())
		return 0;

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "unable to enable mu clock\n");
		return ret;
	}

	writel_relaxed(readl_relaxed(mu_base + MX7ULP_MU_CR) |
			BIT(0) | BIT(26) | BIT(27), mu_base + MX7ULP_MU_CR);

	return 0;
}
#endif
static const struct dev_pm_ops mu_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(mu_suspend, mu_resume)
};

static struct platform_driver imx_mu_driver = {
	.driver = {
		.name   = "imx-mu",
		.owner  = THIS_MODULE,
		.pm = &mu_pm_ops,
		.of_match_table = imx_mu_ids,
	},
	.probe = imx_mu_probe,
};

static int __init imx_mu_init(void)
{
	return platform_driver_register(&imx_mu_driver);
}
subsys_initcall(imx_mu_init);
