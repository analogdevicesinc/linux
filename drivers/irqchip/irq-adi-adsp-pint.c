// SPDX-License-Identifier: GPL-2.0
/*
 * Pin Interrupt (PINT) controller for ADI ADSP SoCs
 *
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * For each PINT, there are up to two PORTs connected. It usually looks
 * something like this:
 *
 *        PINT0   PINT1    ...    PINT7
 *        /   \   /   \   /  \    /   \
 *       /     \ /     \ /    \  /     \
 *    PORTA   PORTB   PORTC    ...    PORTI
 *
 * Note that not all PORTs are connected to two PINTs: sometimes only one is
 * connected.
 *
 * The PINT hwirq space is 32 bits (4 bytes) wide, where each byte is mapped to
 * the upper- or lower-half of a connected PORT via 1-bit muxes:
 *
 *     BYTE3       BYTE2       BYTE1       BYTE0     } PINT hwirq space
 *       ^           ^           ^           ^
 *     __|__       __|__       __|__       __|__
 *    /_____\     /_____\     /_____\     /_____\    } 1-bit muxes
 *     0   1       0   1       0   1       0   1
 *     |   |       |   |       |   |       |   |
 *    PAH PBH     PAL PBL     PAH PBH     PAL PBL    } PORT halves
 *
 * The muxes are controlled via the PINT_ASSIGN register.
 *
 * If a given PORT half is assigned twice (i.e. to two different bytes in the
 * hwirq space), the hardware enables raceless handling of EDGE_BOTH interrupt
 * types on these GPIO lines through mirroring of the edge polarity in the two
 * respective hwirq bits. An interrupt is then generated on either a rising or
 * falling edge. When an interrupt is configured for EDGE_BOTH, the mirrored
 * hwirq is then masked and unmasked in tandem with the nominal hwirq. In order
 * to prevent misconfiguration, the driver rejects allocation of virqs
 * corresponding to the mirror bits. If no mirror is available, EDGE_BOTH is
 * rejected.
 *
 * For more information, check the section "PORT Event Control" in the hardware
 * reference manual [1].
 *
 * [1] https://www.analog.com/media/en/dsp-documentation/processor-manuals/adsp-sc595-sc596-sc598-hrm.pdf
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#define ADSP_PINT_NLINES 32

#define ADSP_PINT_MSK_SET	0x00
#define ADSP_PINT_MSK_CLR	0x04
#define ADSP_PINT_REQ		0x08
#define ADSP_PINT_ASSIGN	0x0c
#define ADSP_PINT_EDGE_SET	0x10
#define ADSP_PINT_EDGE_CLR	0x14
#define ADSP_PINT_INV_SET	0x18
#define ADSP_PINT_INV_CLR	0x1c

struct adsp_pint {
	struct device *dev;
	void __iomem *regs;
	struct irq_domain *domain;
	int irq;
	unsigned int mirror_mask;
	unsigned long edge_both_mask;
};

static void adsp_pint_enable(struct irq_data *d)
{
	struct adsp_pint *pint = irq_data_get_irq_chip_data(d);
	unsigned int bits = BIT(d->hwirq);

	if (test_bit(d->hwirq, &pint->edge_both_mask))
		bits |= bits << 16;

	writel(bits, pint->regs + ADSP_PINT_REQ);
	writel(bits, pint->regs + ADSP_PINT_MSK_SET);
}

static void adsp_pint_mask(struct irq_data *d)
{
	struct adsp_pint *pint = irq_data_get_irq_chip_data(d);
	unsigned int bits = BIT(d->hwirq);

	if (test_bit(d->hwirq, &pint->edge_both_mask))
		bits |= bits << 16;

	writel(bits, pint->regs + ADSP_PINT_MSK_CLR);
}

static void adsp_pint_unmask(struct irq_data *d)
{
	struct adsp_pint *pint = irq_data_get_irq_chip_data(d);
	unsigned int bits = BIT(d->hwirq);

	if (test_bit(d->hwirq, &pint->edge_both_mask))
		bits |= bits << 16;

	writel(bits, pint->regs + ADSP_PINT_MSK_SET);
}

static void adsp_pint_ack(struct irq_data *d)
{
	struct adsp_pint *pint = irq_data_get_irq_chip_data(d);
	unsigned int bits = BIT(d->hwirq);

	if (test_bit(d->hwirq, &pint->edge_both_mask))
		bits |= bits << 16;

	writel(bits, pint->regs + ADSP_PINT_REQ);
}

static int adsp_pint_set_type(struct irq_data *d, unsigned int type)
{
	struct adsp_pint *pint = irq_data_get_irq_chip_data(d);
	unsigned int bit = BIT(d->hwirq);

	clear_bit(d->hwirq, &pint->edge_both_mask);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		writel(bit, pint->regs + ADSP_PINT_EDGE_SET);
		writel(bit, pint->regs + ADSP_PINT_INV_CLR);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		writel(bit, pint->regs + ADSP_PINT_EDGE_SET);
		writel(bit, pint->regs + ADSP_PINT_INV_SET);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		if (!(pint->mirror_mask & (bit << 16)))
			return -EINVAL;

		set_bit(d->hwirq, &pint->edge_both_mask);
		writel(bit | (bit << 16), pint->regs + ADSP_PINT_EDGE_SET);
		writel(bit, pint->regs + ADSP_PINT_INV_CLR);
		writel(bit << 16, pint->regs + ADSP_PINT_INV_SET);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		writel(bit, pint->regs + ADSP_PINT_EDGE_CLR);
		writel(bit, pint->regs + ADSP_PINT_INV_CLR);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		writel(bit, pint->regs + ADSP_PINT_EDGE_CLR);
		writel(bit, pint->regs + ADSP_PINT_INV_SET);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct irq_chip adsp_pint_irq_chip = {
	.name = "adsp-pint",
	.irq_enable = adsp_pint_enable,
	.irq_mask = adsp_pint_mask,
	.irq_unmask = adsp_pint_unmask,
	.irq_ack = adsp_pint_ack,
	.irq_set_type = adsp_pint_set_type,
	.flags = IRQCHIP_SET_TYPE_MASKED,
};

static int adsp_pint_domain_alloc(struct irq_domain *d, unsigned int virq,
				  unsigned int nr_irqs, void *arg)
{
	struct adsp_pint *pint = d->host_data;
	struct irq_fwspec *fwspec = arg;
	irq_hw_number_t hwirq;
	unsigned int type;
	int ret;

	if (nr_irqs != 1)
		return -EINVAL;

	ret = irq_domain_translate_twocell(d, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	/* Disallow mapping of mirror hwirqs */
	if (pint->mirror_mask & BIT(hwirq))
		return -EINVAL;

	irq_domain_set_info(d, virq, hwirq, &adsp_pint_irq_chip, pint,
			    handle_bad_irq, NULL, NULL);

	return 0;
}

static const struct irq_domain_ops adsp_pint_domain_ops = {
	.translate = irq_domain_translate_twocell,
	.alloc = adsp_pint_domain_alloc,
	.free = irq_domain_free_irqs_common,
};

static void adsp_pint_handler(struct irq_desc *desc)
{
	struct adsp_pint *pint = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long req;
	int hwirq;

	chained_irq_enter(chip, desc);

	req = readl(pint->regs + ADSP_PINT_REQ);

	/* Fold any mirror interrupts TODO better explanation */
	req |= (pint->mirror_mask & req) >> 16;
	req &= ~pint->mirror_mask;

	for_each_set_bit(hwirq, &req, ADSP_PINT_NLINES)
		generic_handle_domain_irq(pint->domain, hwirq);

	chained_irq_exit(chip, desc);
}

static int adsp_pint_init(struct adsp_pint *pint, unsigned int assign)
{
	u8 b[4] = { assign, assign >> 8, assign >> 16, assign >> 24 };
	int i;

	/* ASSIGN muxes can only be set to 0 or 1 (connected PORT A or B) */
	for (i = 0; i < ARRAY_SIZE(b); i++) {
		if (b[i] > 1)
			return -EINVAL;
	}

	if (b[0] == b[2])
		pint->mirror_mask |= 0x00FF0000;

	if (b[1] == b[3])
		pint->mirror_mask |= 0xFF000000;

	writel(~0u, pint->regs + ADSP_PINT_MSK_CLR);
	writel(~0u, pint->regs + ADSP_PINT_REQ);
	writel(assign, pint->regs + ADSP_PINT_ASSIGN);

	return 0;
}

static int adsp_pint_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adsp_pint *pint;
	unsigned int assign;
	int ret;

	pint = devm_kzalloc(dev, sizeof(*pint), GFP_KERNEL);
	if (!pint)
		return -ENOMEM;

	platform_set_drvdata(pdev, pint);
	pint->dev = dev;

	pint->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pint->regs))
		return PTR_ERR(pint->regs);

	pint->irq = platform_get_irq(pdev, 0);
	if (pint->irq < 0)
		return pint->irq;

	ret = of_property_read_u32(dev->of_node, "adi,pint-assign", &assign);
	if (ret)
		return dev_err_probe(dev, ret,
				     "bad adi,pint-assign property\n");

	ret = adsp_pint_init(pint, assign);
	if (ret)
		return ret;

	pint->domain = devm_irq_domain_instantiate(
		dev, &(struct irq_domain_info){
			     .fwnode = of_fwnode_handle(dev->of_node),
			     .size = ADSP_PINT_NLINES,
			     .hwirq_max = ADSP_PINT_NLINES,
			     .ops = &adsp_pint_domain_ops,
			     .host_data = pint,
			     .dev = dev });
	if (IS_ERR(pint->domain))
		return PTR_ERR(pint->domain);

	irq_set_chained_handler_and_data(pint->irq, adsp_pint_handler, pint);

	return 0;
}

static void adsp_pint_remove(struct platform_device *pdev)
{
	struct adsp_pint *pint = platform_get_drvdata(pdev);

	irq_set_chained_handler_and_data(pint->irq, NULL, NULL);
}

static const struct of_device_id adsp_pint_of_match[] = {
	{ .compatible = "adi,adsp-sc598-pint" },
	{ }
};
MODULE_DEVICE_TABLE(of, adsp_pint_of_match);

static struct platform_driver adsp_pint_driver = {
	.driver = {
		.name		= "adsp-pint",
		.of_match_table	= adsp_pint_of_match,
	},
	.probe = adsp_pint_probe,
	.remove = adsp_pint_remove,
};
module_platform_driver(adsp_pint_driver);

MODULE_AUTHOR("Alvin Šipraga <alvin.sipraga@analog.com>");
MODULE_DESCRIPTION("ADI ADSP PINT interrupt controller driver");
MODULE_LICENSE("GPL");
