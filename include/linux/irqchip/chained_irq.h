/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Chained IRQ handlers support.
 *
 * Copyright (C) 2011 ARM Ltd.
 */
#ifndef __IRQCHIP_CHAINED_IRQ_H
#define __IRQCHIP_CHAINED_IRQ_H

#include <linux/cleanup.h>
#include <linux/irq.h>

/*
 * Entry/exit functions for chained handlers where the primary IRQ chip
 * may implement either fasteoi or level-trigger flow control.
 */
static inline void chained_irq_enter(struct irq_chip *chip,
				     struct irq_desc *desc)
{
	/* FastEOI controllers require no action on entry. */
	if (chip->irq_eoi)
		return;

	if (chip->irq_mask_ack) {
		chip->irq_mask_ack(&desc->irq_data);
	} else {
		chip->irq_mask(&desc->irq_data);
		if (chip->irq_ack)
			chip->irq_ack(&desc->irq_data);
	}
}

static inline void chained_irq_exit(struct irq_chip *chip,
				    struct irq_desc *desc)
{
	if (chip->irq_eoi)
		chip->irq_eoi(&desc->irq_data);
	else
		chip->irq_unmask(&desc->irq_data);
}

/*
 * Scope-based variant of the chained_irq_enter()/chained_irq_exit() pair.
 *
 * guard_chained_irq() invokes chained_irq_enter() immediately and arranges
 * for the matching chained_irq_exit() to run automatically when the
 * enclosing function (or compound statement) scope ends, no matter which
 * return path is taken. This removes the need to spell out chained_irq_exit()
 * on every exit path of a chained handler.
 *
 * Usage:
 *
 *	static void foo_irq_handler(struct irq_desc *desc)
 *	{
 *		struct irq_chip *chip = irq_desc_get_chip(desc);
 *
 *		guard_chained_irq(chip, desc);
 *		... handle the interrupt, return freely ...
 *	}
 */

/* Internal context for guard_chained_irq(); not for driver use. */
struct __chained_irq_guard_ctx {
	struct irq_chip *chip;
	struct irq_desc *desc;
};

DEFINE_CLASS(chained_irq, struct __chained_irq_guard_ctx,
	     chained_irq_exit(_T.chip, _T.desc),
	     ({ struct __chained_irq_guard_ctx _T = { .chip = chip, .desc = desc };
		chained_irq_enter(_T.chip, _T.desc); _T; }),
	     struct irq_chip *chip, struct irq_desc *desc)

#define guard_chained_irq(chip, desc) \
	guard(chained_irq)(chip, desc)

#endif /* __IRQCHIP_CHAINED_IRQ_H */
