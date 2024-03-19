// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices Trigger Routing Unit (TRU) driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#ifndef ADI_TRU_H
#define ADI_TRU_H

#include <linux/mutex.h>

struct adi_tru {
	struct device *dev;
	void __iomem *base;
	struct mutex lock;
	u32 last_source_id;
	u32 last_target_id;
	u32 alias_id;
	bool preset_locked;
	struct list_head node;
};

/* Get TRU device by its alias ID */
struct adi_tru *adi_tru_get(u32 alias_id);

int adi_tru_enable(struct adi_tru *tru);

int adi_tru_disable(struct adi_tru *tru);

int adi_tru_soft_reset(struct adi_tru *tru);

int adi_tru_trigger(struct adi_tru *tru, int n, ...);

int adi_tru_connect_source_to_target(struct adi_tru *tru, u32 source, u32 target, bool locked);

#endif /* ADI_TRU_H */
