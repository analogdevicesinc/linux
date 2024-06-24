// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2022 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@foss.st.com> for STMicroelectronics.
 */

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "reset-stm32.h"

struct stm32_reset_data {
	/* reset lock */
	spinlock_t			lock;
	struct reset_controller_dev	rcdev;
	void __iomem			*membase;
	u32				clear_offset;
	const struct stm32_reset_cfg	**reset_lines;
};

static inline struct stm32_reset_data *
to_stm32_reset_data(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct stm32_reset_data, rcdev);
}

static const struct stm32_reset_cfg *stm32_get_reset_line(struct reset_controller_dev *rcdev,
							  unsigned long id,
							  struct stm32_reset_cfg *line)
{
	struct stm32_reset_data *data = to_stm32_reset_data(rcdev);

	if (!data->reset_lines) {
		int reg_width = sizeof(u32);
		int bank = id / (reg_width * BITS_PER_BYTE);
		int offset = id % (reg_width * BITS_PER_BYTE);

		line->offset = bank * reg_width;
		line->bit_idx = offset;
		line->set_clr = (data->clear_offset ? true : false);

		return line;
	}

	return data->reset_lines[id];
}

static int stm32_reset_update(struct reset_controller_dev *rcdev,
			      unsigned long id, bool assert)
{
	struct stm32_reset_data *data = to_stm32_reset_data(rcdev);
	struct stm32_reset_cfg line_reset;
	const struct stm32_reset_cfg *ptr_line;

	ptr_line = stm32_get_reset_line(rcdev, id, &line_reset);
	if (!ptr_line)
		return -EPERM;

	if (ptr_line->set_clr) {
		void __iomem *addr;

		addr = data->membase + ptr_line->offset;
		if (!assert)
			addr += data->clear_offset;

		writel(BIT(ptr_line->bit_idx), addr);

	} else {
		unsigned long flags;
		u32 reg;

		spin_lock_irqsave(&data->lock, flags);

		reg = readl(data->membase + ptr_line->offset);

		if (assert)
			reg |= BIT(ptr_line->bit_idx);
		else
			reg &= ~BIT(ptr_line->bit_idx);

		writel(reg, data->membase + ptr_line->offset);

		spin_unlock_irqrestore(&data->lock, flags);
	}

	return 0;
}

static int stm32_reset_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	return stm32_reset_update(rcdev, id, true);
}

static int stm32_reset_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	return stm32_reset_update(rcdev, id, false);
}

static int stm32_reset_status(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct stm32_reset_data *data = to_stm32_reset_data(rcdev);
	struct stm32_reset_cfg line_reset;
	const struct stm32_reset_cfg *ptr_line;
	u32 reg;

	ptr_line = stm32_get_reset_line(rcdev, id, &line_reset);
	if (!ptr_line)
		return -EPERM;

	reg = readl(data->membase + ptr_line->offset);

	return !!(reg & BIT(ptr_line->bit_idx));
}

static const struct reset_control_ops stm32_reset_ops = {
	.assert		= stm32_reset_assert,
	.deassert	= stm32_reset_deassert,
	.status		= stm32_reset_status,
};

int stm32_rcc_reset_init(struct device *dev, struct clk_stm32_reset_data *data,
			 void __iomem *base)
{
	struct stm32_reset_data *reset_data;

	reset_data = kzalloc(sizeof(*reset_data), GFP_KERNEL);
	if (!reset_data)
		return -ENOMEM;

	spin_lock_init(&reset_data->lock);

	reset_data->membase = base;
	reset_data->rcdev.owner = THIS_MODULE;
	reset_data->rcdev.ops = &stm32_reset_ops;
	reset_data->rcdev.of_node = dev_of_node(dev);
	reset_data->rcdev.nr_resets = data->nr_lines;
	reset_data->reset_lines = data->reset_lines;
	reset_data->clear_offset = data->clear_offset;

	return reset_controller_register(&reset_data->rcdev);
}
