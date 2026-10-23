// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#include <linux/module.h>
#include "ahb.h"
#include "core.h"
#include "debug.h"
#include "hif.h"
#include "hw.h"
#include "pci.h"
#include "pcic.h"

void ath12k_pcic_config_static_window(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	u32 umac_window = u32_get_bits(ab_ahb->reg_base->umac_base, WINDOW_VALUE_MASK);
	u32 ce_window = u32_get_bits(ab_ahb->reg_base->ce_reg_base, WINDOW_VALUE_MASK);
	u32 window;

	window = (umac_window << 12) | (ce_window << 6);
	writel_relaxed(WINDOW_ENABLE_BIT | window, ab->mem + WINDOW_REG_ADDRESS);
	ab_ahb->static_window_reg = readl_relaxed(ab->mem + WINDOW_REG_ADDRESS);
}

static int ath12k_pcic_select_static_window(struct ath12k_base *ab, u32 addr)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	u32 window_bits, new_window, reg_value;
	const int max_retries = 10;
	int i;

	window_bits = u32_get_bits(addr, WINDOW_VALUE_MASK);

	/* Update window bits [5:0], preserving other bits */
	new_window = u32_replace_bits(ab_ahb->static_window_reg, window_bits, 0x3f);

	if (new_window == ab_ahb->static_window_reg)
		return 0;

	reg_value = WINDOW_ENABLE_BIT | new_window;
	writel_relaxed(reg_value, ab->mem + WINDOW_REG_ADDRESS);

	/* Wait for the write to take effect */
	for (i = 0; i < max_retries; i++) {
		if (readl_relaxed(ab->mem + WINDOW_REG_ADDRESS) == reg_value) {
			ab_ahb->static_window_reg = reg_value;
			return 0;
		}

		usleep_range(1000, 2000);
	}

	ath12k_warn(ab, "failed to set static window for cmem init\n");
	return -ETIMEDOUT;
}

int ath12k_pcic_cmem_read32(struct ath12k_base *ab, u32 addr, u32 *value)
{
	int ret;

	if (!value)
		return -EINVAL;

	if (addr < WINDOW_START) {
		*value = readl_relaxed(ab->mem + addr);
		return 0;
	}

	ret = ath12k_pcic_select_static_window(ab, addr);
	if (ret)
		return ret;

	*value = readl_relaxed(ab->mem + WINDOW_START + (addr & WINDOW_RANGE_MASK));
	return 0;
}

int ath12k_pcic_cmem_write32(struct ath12k_base *ab, u32 addr, u32 value)
{
	int ret;

	if (addr < WINDOW_START) {
		writel_relaxed(value, ab->mem + addr);
		return 0;
	}

	ret = ath12k_pcic_select_static_window(ab, addr);
	if (ret)
		return ret;

	writel_relaxed(value, ab->mem + WINDOW_START + (addr & WINDOW_RANGE_MASK));
	return 0;
}

static u32 ath12k_pcic_get_window_start(struct ath12k_base *ab, u32 offset)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	u32 window_start;

	/* If offset lies within DP register range, use 3rd window */
	if ((offset ^ ab_ahb->reg_base->umac_base) <= WINDOW_RANGE_MASK)
		window_start = 3 * WINDOW_START;
	/* If offset lies within CE register range, use 2nd window */
	else if ((offset ^ ab_ahb->reg_base->ce_reg_base) <= WINDOW_RANGE_MASK)
		window_start = 2 * WINDOW_START;
	else
		window_start = WINDOW_START;

	return window_start;
}

void ath12k_pcic_get_msi_address(struct ath12k_base *ab, u32 *msi_addr_lo,
				 u32 *msi_addr_hi)
{
	*msi_addr_lo = ab->msi.addr_lo;
	*msi_addr_hi = ab->msi.addr_hi;
}

void ath12k_pcic_stop(struct ath12k_base *ab)
{
	ath12k_pci_ce_irq_disable_sync(ab);
	timer_delete_sync(&ab->rx_replenish_retry);
	ath12k_ce_cleanup_pipes(ab);
}

int ath12k_pcic_start(struct ath12k_base *ab)
{
	ath12k_pci_ce_irqs_enable(ab);
	ath12k_ce_rx_post_buf(ab);
	return 0;
}

u32 ath12k_pcic_read32(struct ath12k_base *ab, u32 offset)
{
	u32 window_start;

	if (offset < WINDOW_START)
		return readl_relaxed(ab->mem + offset);

	window_start = ath12k_pcic_get_window_start(ab, offset);
	return readl_relaxed(ab->mem + window_start + (offset & WINDOW_RANGE_MASK));
}

void ath12k_pcic_write32(struct ath12k_base *ab, u32 offset, u32 value)
{
	u32 window_start;

	if (offset < WINDOW_START) {
		writel_relaxed(value, ab->mem + offset);
		return;
	}

	window_start = ath12k_pcic_get_window_start(ab, offset);
	writel_relaxed(value, ab->mem + window_start + (offset & WINDOW_RANGE_MASK));
}
