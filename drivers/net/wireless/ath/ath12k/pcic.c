// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#include <linux/module.h>
#include <linux/msi.h>
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

static void ath12k_pcic_ce_irq_disable(struct ath12k_base *ab, u16 ce_id)
{
	u32 irq_idx;

	irq_idx = ATH12K_MSI_IRQ_CE0_OFFSET + ce_id;
	disable_irq_nosync(ab->irq_num[irq_idx]);
}

static irqreturn_t ath12k_pcic_ce_interrupt_handler(int irq, void *arg)
{
	struct ath12k_ce_pipe *ce_pipe = arg;
	struct ath12k_base *ab = ce_pipe->ab;

	if (!test_bit(ATH12K_FLAG_CE_IRQ_ENABLED, &ab->dev_flags))
		return IRQ_HANDLED;

	/* last interrupt received for this CE */
	ce_pipe->timestamp = jiffies;

	ath12k_pcic_ce_irq_disable(ab, ce_pipe->pipe_num);
	queue_work(system_bh_wq, &ce_pipe->intr_wq);

	return IRQ_HANDLED;
}

static void ath12k_pcic_ext_grp_enable(struct ath12k_ext_irq_grp *irq_grp)
{
	int i;

	for (i = 0; i < irq_grp->num_irq; i++)
		enable_irq(irq_grp->ab->irq_num[irq_grp->irqs[i]]);
}

static int ath12k_pcic_ext_grp_napi_poll(struct napi_struct *napi, int budget)
{
	struct ath12k_ext_irq_grp *irq_grp = container_of(napi,
						struct ath12k_ext_irq_grp,
						napi);
	struct ath12k_base *ab = irq_grp->ab;
	int work_done;

	work_done = ath12k_dp_service_srng(ab->dp, irq_grp, budget);
	if (work_done < budget) {
		if (likely(napi_complete_done(napi, work_done)))
			ath12k_pcic_ext_grp_enable(irq_grp);
	}

	if (work_done > budget)
		work_done = budget;

	return work_done;
}

static int ath12k_pcic_ext_irq_config(struct ath12k_base *ab,
				      struct msi_desc **msi_desc, int num_msi)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	struct ath12k_ext_irq_grp *irq_grp;
	int j, i, k, vec_idx = 0, ret = 0;
	u8 userpd_id = ab_ahb->userpd_id;
	u32 num_irq, base_idx;
	struct msi_desc *desc;

	base_idx = ATH12K_MSI_IRQ_CE0_OFFSET + CE_COUNT_MAX;

	for (i = 0; i < ATH12K_EXT_IRQ_GRP_NUM_MAX; i++) {
		num_irq = 0;
		irq_grp = &ab->ext_irq_grp[i];
		irq_grp->ab = ab;
		irq_grp->grp_id = i;

		if (ab->hw_params->ring_mask->tx[i] ||
		    ab->hw_params->ring_mask->rx[i] ||
		    ab->hw_params->ring_mask->rx_err[i] ||
		    ab->hw_params->ring_mask->rx_wbm_rel[i] ||
		    ab->hw_params->ring_mask->reo_status[i] ||
		    ab->hw_params->ring_mask->host2rxdma[i] ||
		    ab->hw_params->ring_mask->rx_mon_dest[i]) {
			num_irq = 1;
		}

		if (!num_irq)
			continue;

		desc = msi_desc[vec_idx];

		irq_grp->napi_ndev = alloc_netdev_dummy(0);
		if (!irq_grp->napi_ndev) {
			ret = -ENOMEM;
			goto fail_allocate;
		}

		netif_napi_add(irq_grp->napi_ndev, &irq_grp->napi,
			       ath12k_pcic_ext_grp_napi_poll);
		irq_grp->num_irq = num_irq;
		irq_grp->irqs[0] = base_idx + i;

		for (j = 0; j < irq_grp->num_irq; j++) {
			int irq_idx = irq_grp->irqs[j];

			scnprintf(irq_grp->irq_name, sizeof(irq_grp->irq_name),
				  "pcic%u_wlan_dp_%u", userpd_id, i);
			irq_set_status_flags(desc->irq, IRQ_DISABLE_UNLAZY);
			ret = request_irq(desc->irq, ath12k_pci_ext_interrupt_handler,
					  IRQF_SHARED, irq_grp->irq_name, irq_grp);
			if (ret) {
				ath12k_err(ab, "failed request irq %d, for group %d : %d\n",
					   desc->irq, irq_grp->grp_id, ret);

				for (k = 0; k < j; k++)
					free_irq(ab->irq_num[irq_grp->irqs[k]], irq_grp);

				netif_napi_del(&irq_grp->napi);
				free_netdev(irq_grp->napi_ndev);
				irq_grp->napi_ndev = NULL;
				goto fail_allocate;
			}

			ab->irq_num[irq_idx] = desc->irq;
			disable_irq_nosync(ab->irq_num[irq_idx]);
		}

		vec_idx = (vec_idx + 1) % num_msi;
	}

	return 0;

fail_allocate:
	ath12k_pci_free_ext_irq(ab, i);
	return ret;
}

static int ath12k_pcic_ce_irq_config(struct ath12k_base *ab,
				     struct msi_desc **msi_desc, int num_msi)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	int irq_idx, ret, i, vec_idx = 0;
	u8 userpd_id = ab_ahb->userpd_id;
	struct ath12k_ce_pipe *ce_pipe;
	bool msi_info_captured = false;
	struct msi_desc *desc;

	for (i = 0; i < ab->hw_params->ce_count; i++) {
		if (ath12k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR)
			continue;

		ce_pipe = &ab->ce.ce_pipe[i];
		desc = msi_desc[vec_idx];
		INIT_WORK(&ce_pipe->intr_wq, ath12k_pci_ce_workqueue);
		irq_idx = ATH12K_MSI_IRQ_CE0_OFFSET + i;

		scnprintf(ce_pipe->irq_name, sizeof(ce_pipe->irq_name),
			  "pcic%u_wlan_ce_%u", userpd_id, i);
		ret = request_irq(desc->irq, ath12k_pcic_ce_interrupt_handler,
				  IRQF_SHARED, ce_pipe->irq_name, ce_pipe);
		if (ret) {
			ath12k_warn(ab, "failed to request irq %d: %d\n", irq_idx, ret);
			ath12k_pci_free_ce_irq(ab, i);
			return ret;
		}

		ab->irq_num[irq_idx] = desc->irq;
		ath12k_pcic_ce_irq_disable(ab, i);
		vec_idx = (vec_idx + 1) % num_msi;

		if (!msi_info_captured) {
			ab->msi.addr_lo = desc->msg.address_lo;
			ab->msi.addr_hi = desc->msg.address_hi;
			msi_info_captured = true;
		}
	}

	return 0;
}

static void ath12k_msi_msg_handler(struct msi_desc *desc, struct msi_msg *msg)
{
	desc->msg.address_lo = msg->address_lo;
	desc->msg.address_hi = msg->address_hi;
	desc->msg.data = msg->data;
}

static int ath12k_pcic_request_msi_irqs(struct ath12k_base *ab, int base_vector,
					int num_vectors,
					int (*irq_config)(struct ath12k_base *ab,
							  struct msi_desc **msi_desc,
							  int num_msi))
{
	struct msi_desc *msi_vecs[ATH12K_MAX_MSI_VECTOR];
	struct platform_device *pdev = ab->pdev;
	struct msi_desc *msi_desc;
	int num_msi = 0, ret;

	guard(msi_descs_lock)(&pdev->dev);

	msi_for_each_desc(msi_desc, &pdev->dev, MSI_DESC_ALL) {
		if (msi_desc->msi_index < base_vector ||
		    msi_desc->msi_index >= (num_vectors + base_vector))
			continue;

		if (num_msi >= ATH12K_MAX_MSI_VECTOR)
			return -EOVERFLOW;

		msi_vecs[num_msi++] = msi_desc;
	}

	if (!num_msi)
		return -EINVAL;

	ret = irq_config(ab, msi_vecs, num_msi);
	if (ret) {
		ath12k_warn(ab, "failed to config irq %d\n", ret);
		return ret;
	}

	return 0;
}

int ath12k_pcic_config_irq(struct ath12k_base *ab)
{
	int user_base_data, base_vector, num_vectors;
	struct platform_device *pdev = ab->pdev;
	int ret;

	ab->msi.config = &ath12k_msi_config[ATH12K_MSI_CONFIG_IPCI];

	ret = platform_device_msi_init_and_alloc_irqs(&pdev->dev,
						      ab->msi.config->total_vectors,
						      ath12k_msi_msg_handler);
	if (ret)
		return ret;

	ret = ath12k_pci_get_user_msi_assignment(ab, "CE", &num_vectors,
						 &user_base_data,
						 &base_vector);
	if (ret)
		goto free_msi;

	ret = ath12k_pcic_request_msi_irqs(ab, base_vector, num_vectors,
					   ath12k_pcic_ce_irq_config);
	if (ret) {
		ath12k_err(ab, "failed to config CE irq\n");
		goto free_msi;
	}

	ret = ath12k_pci_get_user_msi_assignment(ab, "DP", &num_vectors,
						 &user_base_data, &base_vector);
	if (ret)
		goto free_ce_irq;

	ret = ath12k_pcic_request_msi_irqs(ab, base_vector, num_vectors,
					   ath12k_pcic_ext_irq_config);
	if (ret) {
		ath12k_err(ab, "failed to config DP irq\n");
		goto free_ce_irq;
	}

	set_bit(ATH12K_FLAG_MULTI_MSI_VECTORS, &ab->dev_flags);
	return 0;

free_ce_irq:
	ath12k_pci_free_ce_irq(ab, ab->hw_params->ce_count);
free_msi:
	platform_device_msi_free_irqs_all(&pdev->dev);
	return ret;
}

void ath12k_pcic_free_irq(struct ath12k_base *ab)
{
	struct platform_device *pdev = ab->pdev;

	ath12k_pci_free_ce_irq(ab, ab->hw_params->ce_count);
	ath12k_pci_free_ext_irq(ab, ATH12K_EXT_IRQ_GRP_NUM_MAX);
	platform_device_msi_free_irqs_all(&pdev->dev);
}
